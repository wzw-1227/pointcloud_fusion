#include <ehpr_bridge/matching_io.h>
#include <ehpr_bridge/RobotSubmap.h>
#include <ehpr_bridge/InterRobotMatch.h>
#include <ehpr_bridge/MapAlignment.h>
#include <ehpr_bridge/session_map.h>
#include <std_srvs/Trigger.h>
#include <limits>
#include <ros/ros.h>
#include <lio_sam/Scan2MapDebug.h>
#include <ehpr_bridge/lio_quality_guard.h>
#include <ehpr_bridge/recent_cloud_buffer.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <array>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <boost/bind/bind.hpp>
namespace eb=ehpr_bridge;
static geometry_msgs::Transform transformMessage(const Eigen::Isometry3d& t) {
  geometry_msgs::Transform m;auto p=t.translation();Eigen::Quaterniond q(t.rotation());q.normalize();
  m.translation.x=p.x();m.translation.y=p.y();m.translation.z=p.z();
  m.rotation.x=q.x();m.rotation.y=q.y();m.rotation.z=q.z();m.rotation.w=q.w();return m;
}
static eb::SubmapPtr decode(const eb::RobotSubmap& m) {
  auto i=std::make_shared<eb::SubmapInput>();i->robot=m.robot_id;i->session=m.session_id;i->epoch=m.epoch;i->id=m.submap_id;
  i->stamp=m.header.stamp.toSec();i->frame=m.header.frame_id;i->local_frame=m.cloud_local.header.frame_id;i->partial=m.partial;
  const auto& p=m.pose_in_lio_map.position;const auto& q=m.pose_in_lio_map.orientation;
  Eigen::Quaterniond r(q.w,q.x,q.y,q.z);
  if(!r.coeffs().allFinite()||std::abs(r.norm()-1)>1e-3)throw std::invalid_argument("Invalid pose quaternion");
  i->pose.linear()=r.toRotationMatrix();i->pose.translation()=Eigen::Vector3d(p.x,p.y,p.z);
  const auto& g=m.gravity_alignment;i->gravity=Eigen::Quaterniond(g.w,g.x,g.y,g.z);
  const auto& c=m.cloud_local;
  if(c.header.stamp!=m.header.stamp || !c.width || !c.height || !c.point_step || c.is_bigendian ||
     uint64_t(c.row_step)<uint64_t(c.width)*c.point_step || c.data.size()<uint64_t(c.row_step)*c.height)
    throw std::invalid_argument("Invalid local cloud dimensions/header/byte order");
  for(const std::string field:{"x","y","z"}) {
    auto f=std::find_if(c.fields.begin(),c.fields.end(),[&](const auto& v){return v.name==field;});
    if(f==c.fields.end()||f->datatype!=sensor_msgs::PointField::FLOAT32||f->count!=1||uint64_t(f->offset)+4>c.point_step)
      throw std::invalid_argument("Invalid XYZ fields");
  }
  i->cloud.reset(new eb::Cloud);pcl::fromROSMsg(c,*i->cloud);return i;
}
class Node {
 public:
  Node(ros::NodeHandle& nh,const std::string& config,const std::string& root,const std::string& mode)
      :matcher_(eb::loadMatcherConfig(config)),writer_(root,config,mode) {
    replay_mode_=mode=="replay";
    matcher_.setArchiveDirectory((writer_.directory()/"history").string());
    auto y=YAML::LoadFile(config);queue_limit_=y["worker_queue_size"].as<size_t>();
    if(!queue_limit_)throw std::runtime_error("Invalid worker queue size");
    match_pub_=nh.advertise<eb::InterRobotMatch>("/stage3/match",10,false);
    ugv_pub_=nh.advertise<sensor_msgs::PointCloud2>("/stage3/match_cloud_ugv",1,true);
    uav_pub_=nh.advertise<sensor_msgs::PointCloud2>("/stage3/match_cloud_uav",1,true);
    fused_pub_=nh.advertise<sensor_msgs::PointCloud2>("/fused_submap",1,true);
    fused_compat_pub_=nh.advertise<sensor_msgs::PointCloud2>("/stage3/fused_submap",1,true);
    status_pub_=nh.advertise<std_msgs::String>("/stage3/status",1,true);
    if(matcher_.alignment()) {
      map_pub_=nh.advertise<sensor_msgs::PointCloud2>("/stage3/fused_map",1,true);
      alignment_pub_=nh.advertise<eb::MapAlignment>("/stage3/map_alignment",1,true);
      retry_service_=nh.advertiseService("/stage3/retry_history",&Node::retryHistory,this);
    }
    ros::NodeHandle priv("~");priv.param("require_lio_quality",require_quality_,false);
    priv.param("parent_launch_pid",parent_launch_pid_,0);
    if(mode!="live")require_quality_=false;
    std::string warm;priv.param<std::string>("warm_start_inputs",warm,"");
    if(!warm.empty()) {
      if(require_quality_)throw std::runtime_error("Warm start in LIO mode requires persisted per-observation quality; use the ideal frontend entry");
      auto inputs=eb::loadReplay(warm,true,true);matcher_.prime(inputs);
      for(const auto& input:inputs)last_input_generations_[input->robot]=input->session+":"+std::to_string(input->epoch);
      warm_start_waiting_=mode!="replay";
    }
    if(mode=="live" || mode=="ideal_frontend") {
      if(mode=="ideal_frontend") {
        bool enabled=false,sim_time=false;std::string environment;
        nh.getParam("/algorithm_test/enabled",enabled);nh.getParam("/use_sim_time",sim_time);
        nh.getParam("/stage1/environment",environment);
        if(!enabled || !sim_time || environment!="sim" ||
           matcher_.config().ugv_frame!="algorithm_ugv_odom" || matcher_.config().uav_frame!="algorithm_uav_odom") {
          writer_.finish("failed","ideal_frontend requires explicit Gazebo diagnostic and isolated frames");
          throw std::runtime_error("ideal_frontend mode requires the explicit Gazebo algorithm-test entry and isolated frames");
        }
        ROS_WARN("Algorithm-only ideal frontend: truth replaces LIO; localization acceptance is disabled");
      } else for(const std::string prefix:{"/lio_sam","/uav/lio_sam"}) {
        bool enabled=true;if(!nh.getParam(prefix+"/loopClosureEnableFlag",enabled)||enabled) {
          writer_.finish("failed","Both LIO loopClosureEnableFlag parameters must exist and be false");
          throw std::runtime_error("Restart both LIO instances with loopClosureEnableFlag=false");
        }
      }
      if(require_quality_)for(int robot=0;robot<2;++robot)
        subscribers_.push_back(nh.subscribe<lio_sam::Scan2MapDebug>(robot==0?"/lio_sam/debug/scan2map":"/uav/lio_sam/debug/scan2map",200,
          [this,robot](const lio_sam::Scan2MapDebug::ConstPtr& m){
            std::lock_guard<std::mutex> lock(mutex_);
            const bool was=quality_.quarantined(robot);
            quality_.add(robot,{m->header.stamp.toSec(),bool(m->optimization_performed),m->is_degenerate || m->weak_direction_count>0,m->lambda_min});
            if(!was && quality_.quarantined(robot)){quality_events_.push_back(robot);condition_.notify_one();}
          }));
      if(matcher_.config().use_recent_matching_cloud) {
        const auto& c=matcher_.config();eb::RecentCloudConfig recent;
        recent.window_seconds=c.recent_window_seconds;recent.voxel_size=c.recent_voxel_size;
        recent.min_frames=c.recent_min_frames;recent.max_frames=c.recent_max_frames;
        recent.reset_on_timestamp_rollback=bool(matcher_.alignment());
        for(int robot=0;robot<2;++robot) {
          recent_buffers_[robot]=std::make_unique<eb::RecentCloudBuffer>(recent);
          recent_cloud_subs_[robot]=std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
              nh,y[robot==0?"ugv_registered_cloud_topic":"uav_registered_cloud_topic"].as<std::string>(),20);
          recent_odom_subs_[robot]=std::make_unique<message_filters::Subscriber<nav_msgs::Odometry>>(
              nh,y[robot==0?"ugv_registered_odometry_topic":"uav_registered_odometry_topic"].as<std::string>(),20);
          recent_sync_[robot]=std::make_unique<RecentSync>(*recent_cloud_subs_[robot],*recent_odom_subs_[robot],20);
          recent_sync_[robot]->registerCallback(boost::bind(&Node::receiveRecentPtr,this,robot,
              boost::placeholders::_1,boost::placeholders::_2));
        }
      }
      for(int robot=0;robot<2;++robot)
        subscribers_.push_back(nh.subscribe<eb::RobotSubmap>(y[robot==0?"ugv_topic":"uav_topic"].as<std::string>(),10,
          [this,robot](const eb::RobotSubmap::ConstPtr& message){
            std::lock_guard<std::mutex> lock(mutex_);
            if(queue_.size()>=queue_limit_){queue_.pop_front();++queue_drops_;}
            queue_.push_back({message,robot});condition_.notify_one();}));
    }
    timer_=nh.createWallTimer(ros::WallDuration(1),[this](const ros::WallTimerEvent&){
      if(parent_launch_pid_>0&&::kill(parent_launch_pid_,0)<0&&errno==ESRCH){ros::shutdown();return;}
      std_msgs::String message;{std::lock_guard<std::mutex> lock(mutex_);message.data=status_;}status_pub_.publish(message);});
    ROS_INFO_STREAM("Stage 3 output: "<<writer_.directory());
  }
  ~Node(){stop();}
  void start() {worker_=std::thread([this]{run();});}
  void stop() {
    {std::lock_guard<std::mutex> lock(mutex_);stop_=true;condition_.notify_all();}
    if(worker_.joinable())worker_.join();
  }
  void replay(const std::string& directory) {
    try {
      for(auto input:eb::loadReplay(directory)) {consume(input);ros::spinOnce();ros::WallDuration(.05).sleep();}
      writer_.checkpoint(matcher_);writer_.finish("stopped");
    } catch(const std::exception& e) {writer_.finish("failed",e.what());throw;}
    ROS_INFO_STREAM("RESULT_DIR="<<writer_.directory());
  }
  bool failed() const {return failed_;}
  void clearDisplay() {
    eb::Cloud empty;sensor_msgs::PointCloud2 message;pcl::toROSMsg(empty,message);message.header.frame_id=matcher_.config().ugv_frame;
    ugv_pub_.publish(message);uav_pub_.publish(message);
    if(have_fused_||matcher_.alignment()) {
      pcl::PointCloud<pcl::PointXYZRGB> empty_rgb;sensor_msgs::PointCloud2 fused;
      pcl::toROSMsg(empty_rgb,fused);fused.header=message.header;
      fused_pub_.publish(fused);fused_compat_pub_.publish(fused);have_fused_=false;
    }
  }
 private:
  bool retryHistory(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
    {std::lock_guard<std::mutex> lock(mutex_);retry_requested_=true;condition_.notify_one();}
    response.success=true;response.message="Historical retry requested; live inputs have priority";return true;
  }
  void recordInvalidation(const std::string& reason,int robot=-1){
    if(reason=="time_rollback"||reason=="registered_odometry_reset"){
      for(int role=0;role<2;++role)if(robot<0||role==robot)blocked_generations_[role]=last_input_generations_[role];
      {std::lock_guard<std::mutex> lock(mutex_);auto end=std::remove_if(queue_.begin(),queue_.end(),[this](const auto& item){
        return item.first->session_id+":"+std::to_string(item.first->epoch)==blocked_generations_[item.second];});
        reset_discarded_inputs_+=std::distance(end,queue_.end());queue_.erase(end,queue_.end());}
      {std::lock_guard<std::mutex> lock(recent_mutex_);for(int role=0;role<2;++role)if((robot<0||role==robot)&&recent_buffers_[role]){recent_buffers_[role]->reset();live_scans_[role].reset();}if(robot<0)recent_clock_=-1;}
    }
    if(robot<0)matcher_.invalidate(reason);else matcher_.invalidateRobot(robot,reason);
    auto input=std::make_shared<eb::SubmapInput>();input->session="stage3_guard";if(robot>=0)input->robot=robot;
    eb::MatchEvent event;event.status="alignment_invalidated";event.reason=reason;event.clear_display=true;
    event.diagnostics["alignment_valid"]=0;complete(event,input);
  }
  void publishAdvanced() {
    const auto* state=matcher_.alignment();if(!state)return;const auto now=ros::WallTime::now();
    if(state->revision!=published_alignment_revision_) {
      eb::MapAlignment m;m.header.stamp=ros::Time::now();m.header.frame_id=matcher_.config().ugv_frame;
      m.valid=state->valid;m.revision=state->revision;m.reference_frame=matcher_.config().ugv_frame;m.query_frame=matcher_.config().uav_frame;
      m.map_transform=transformMessage(state->transform);m.reason=state->reason;
      if(state->valid){m.reference_session_id=state->reference->session;m.query_session_id=state->query->session;
        m.reference_epoch=state->reference->epoch;m.query_epoch=state->query->epoch;
        m.reference_submap_id=state->reference->id;m.query_submap_id=state->query->id;}
      alignment_pub_.publish(m);published_alignment_revision_=state->revision;
    }
    if(matcher_.mapRevision()!=published_map_revision_ && ((now-last_map_publish_).toSec()>=1||(!state->valid&&published_map_valid_))) {
      const auto fused=matcher_.fusedMap();sensor_msgs::PointCloud2 message;pcl::toROSMsg(fused.cloud,message);
      message.header.frame_id=matcher_.config().ugv_frame;message.header.stamp=ros::Time(0);map_pub_.publish(message);
      published_map_revision_=matcher_.mapRevision();last_map_publish_=now;published_map_valid_=state->valid;
    }
    if((now-last_checkpoint_).toSec()>=10){writer_.checkpoint(matcher_);last_checkpoint_=now;}
  }
  void receiveRecentPtr(int robot,const sensor_msgs::PointCloud2::ConstPtr& cloud,const nav_msgs::Odometry::ConstPtr& odom) {
    receiveRecent(robot,*cloud,*odom);
  }
  void receiveRecent(int robot,const sensor_msgs::PointCloud2& message,const nav_msgs::Odometry& odom) {
    const auto frame=robot==0?matcher_.config().ugv_frame:matcher_.config().uav_frame;
    if(message.header.frame_id!=frame||odom.header.frame_id!=frame||message.header.stamp!=odom.header.stamp)return;
    const auto& p=odom.pose.pose.position;const auto& q=odom.pose.pose.orientation;Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    if(!rotation.coeffs().allFinite()||std::abs(rotation.norm()-1)>1e-3)return;
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.linear()=rotation.toRotationMatrix();pose.translation()<<p.x,p.y,p.z;
    eb::Cloud::Ptr cloud(new eb::Cloud);pcl::fromROSMsg(message,*cloud);
    const double now=ros::Time::now().toSec();if(message.header.stamp.toSec()>now+.05)return;
    std::lock_guard<std::mutex> lock(recent_mutex_);
    bool rollback=recent_clock_>=0&&now<recent_clock_;
    if(rollback)for(auto& buffer:recent_buffers_)if(buffer)buffer->reset();
    auto generation=recent_buffers_[robot]->generation();
    recent_clock_=now;if(!recent_buffers_[robot]->add(cloud,pose,message.header.stamp.toSec()))return;
    auto scan=std::make_shared<eb::SubmapInput>();scan->robot=robot;scan->frame=frame;scan->pose=pose;
    scan->recent_start_stamp=scan->recent_end_stamp=scan->stamp=message.header.stamp.toSec();scan->recent_frames=1;
    scan->latest_scan.reset(new eb::Cloud);pcl::transformPointCloud(*cloud,*scan->latest_scan,Eigen::Affine3d(pose.inverse()));
    live_scans_[robot]=scan;
    if(rollback||generation!=recent_buffers_[robot]->generation()){registered_reset_mask_.fetch_or(rollback?3:(1<<robot));condition_.notify_one();}
  }
  void consume(eb::SubmapPtr input) {
    if(!replay_mode_&&matcher_.alignment()&&input->stamp>ros::Time::now().toSec()+.05){eb::MatchEvent event;event.status="rejected";
      event.reason="submap keyframe is in the future of the current ROS clock";complete(event,input);return;}
    if(require_quality_) {
      std::string reason;{std::lock_guard<std::mutex> lock(mutex_);reason=quality_.reason(input->robot,input->stamp,ros::Time::now().toSec());}
      if(!reason.empty()){eb::MatchEvent event;event.status="rejected";event.reason=reason;complete(event,input);return;}
    }
    if(matcher_.config().use_recent_matching_cloud&&recent_buffers_[input->robot]) {
      std::optional<eb::RecentCloudSnapshot> recent;
      {std::lock_guard<std::mutex> lock(recent_mutex_);
       recent=recent_buffers_[input->robot]->snapshot(input->pose,ros::Time::now().toSec());}
      if(!recent) {
        eb::MatchEvent event;event.status="waiting_recent_cloud";event.reason="recent synchronized scan window is warming up or stale";
        if(matcher_.alignment()){
          auto captured=matcher_.archiveInput(input);
          if(captured.status=="rejected"){complete(captured,input);return;}
          event.clear_display=captured.clear_display;event.diagnostics=std::move(captured.diagnostics);
          last_input_generations_[input->robot]=input->session+":"+std::to_string(input->epoch);
          if(warm_start_waiting_&&ros::Time::now().toSec()-input->stamp<=1)warm_current_seen_[input->robot]=true;
        }
        complete(event,input);return;
      }
      auto augmented=std::make_shared<eb::SubmapInput>(*input);augmented->recent_cloud=recent->local;
      augmented->recent_center_local=recent->center_local;augmented->recent_start_stamp=recent->start_stamp;
      augmented->recent_end_stamp=recent->end_stamp;augmented->recent_frames=recent->frames;augmented->latest_scan=recent->latest_scan;input=augmented;
    }
    if(warm_start_waiting_){auto event=matcher_.archiveInput(input);
      if(event.status!="rejected"){
        last_input_generations_[input->robot]=input->session+":"+std::to_string(input->epoch);
        if(ros::Time::now().toSec()-input->stamp<=1)warm_current_seen_[input->robot]=true;
        warm_start_waiting_=!warm_current_seen_[0]||!warm_current_seen_[1];
        event.status=warm_start_waiting_?"waiting_current_epochs":"history_primed";
        event.reason=warm_start_waiting_?"warm cache waits for fresh Stage 2 epochs on both robots":"current epochs confirmed; historical constraints await revalidation";
      }complete(event,input);return;}
    const auto started=std::chrono::steady_clock::now();
    auto event=matcher_.process(input);
    if(event.status!="rejected")last_input_generations_[input->robot]=input->session+":"+std::to_string(input->epoch);
    event.diagnostics["matching_processing_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    complete(event,input);
  }
  void complete(const eb::MatchEvent& event,eb::SubmapPtr input) {
    auto checked=event;
    if(require_quality_ && checked.match) {
      std::string reason;
      {std::lock_guard<std::mutex> lock(mutex_);
       for(const auto& m:{checked.match->reference,checked.match->query}) {
         auto why=matcher_.alignment()?quality_.currentReason(m->robot,ros::Time::now().toSec()):quality_.reason(m->robot,m->stamp,ros::Time::now().toSec());if(!why.empty()){reason=why;break;}
       }}
      if(!reason.empty()){checked.match.reset();checked.status="rejected";checked.reason=reason;checked.clear_display=true;}
      if(!reason.empty())matcher_.invalidate();
    }
    completeChecked(checked,input);
  }
  void completeChecked(const eb::MatchEvent& event,eb::SubmapPtr input) {
    // A fully supported correction can revoke and confirm in the same worker
    // event. Publish its intermediate invalid state before the new valid state.
    if(!event.revocation_reason.empty()){
      eb::MapAlignment invalid;invalid.header.stamp=ros::Time::now();invalid.header.frame_id=matcher_.config().ugv_frame;
      invalid.valid=false;invalid.revision=event.revoked_alignment_revision;invalid.reference_frame=matcher_.config().ugv_frame;
      invalid.query_frame=matcher_.config().uav_frame;invalid.map_transform.rotation.w=1;invalid.reason=event.revocation_reason;
      alignment_pub_.publish(invalid);published_alignment_revision_=invalid.revision;
    }
    if(event.clear_display){clearDisplay();if(matcher_.alignment()){
      pcl::PointCloud<pcl::PointXYZRGB> empty;sensor_msgs::PointCloud2 clear;pcl::toROSMsg(empty,clear);clear.header.frame_id=matcher_.config().ugv_frame;
      map_pub_.publish(clear);published_map_revision_=std::numeric_limits<uint64_t>::max();published_map_valid_=false;last_map_publish_=ros::WallTime::now();}}
    const auto path=writer_.write(event,input,matcher_);
    if(event.match && ros::ok()) {
      const auto& m=*event.match;eb::InterRobotMatch msg;
      msg.header.stamp.fromSec(m.query->stamp);msg.header.frame_id=m.query->local_frame;
      msg.reference_frame=m.reference->local_frame;msg.reference_stamp.fromSec(m.reference->stamp);
      msg.query_session_id=m.query->session;msg.reference_session_id=m.reference->session;
      msg.query_epoch=m.query->epoch;msg.reference_epoch=m.reference->epoch;
      msg.query_submap_id=m.query->id;msg.reference_submap_id=m.reference->id;
      msg.coarse_transform=transformMessage(m.coarse);msg.refined_transform=transformMessage(m.refined);
      msg.ransac_inliers=m.inliers;msg.fitness=m.fitness;msg.overlap_reference=m.overlap_reference;
      msg.overlap_query=m.overlap_query;msg.overlap_smaller_cloud=m.overlap;msg.residual_rmse=m.residual_rmse;
      msg.query_partial=m.query->partial;msg.reference_partial=m.reference->partial;msg.result_path=path;
      eb::Cloud ugv,uav;eb::Matcher::displayClouds(m,ugv,uav);sensor_msgs::PointCloud2 a,b;
      pcl::toROSMsg(ugv,a);pcl::toROSMsg(uav,b);a.header.frame_id=b.header.frame_id=m.reference->frame;
      a.header.stamp=b.header.stamp=ros::Time(0);
      const auto fused=eb::fuseSubmaps(m,matcher_.config().fusion_voxel_size);sensor_msgs::PointCloud2 c;
      pcl::toROSMsg(fused.cloud,c);c.header=a.header;
      ugv_pub_.publish(a);uav_pub_.publish(b);fused_pub_.publish(c);fused_compat_pub_.publish(c);have_fused_=true;match_pub_.publish(msg);
      ROS_INFO_STREAM("Accepted UAV "<<m.query->id<<" <-> UGV "<<m.reference->id<<", inliers="<<m.inliers<<", fitness="<<m.fitness<<", result="<<path);
    }
    auto summary=writer_.summary();
    summary["matching_profile"]=matcher_.config().matching_profile;
    if(event.match||event.attempted)summary["visibility_evidence_source"]=(event.match?event.match->visibility:event.attempted->visibility).source;
    else if(event.diagnostics.count("monitor_single_scan_source"))summary["visibility_evidence_source"]="bidirectional_single_scan";
    if(!event.revocation_reason.empty())summary["last_alignment_revocation_reason"]=event.revocation_reason;
    if(matcher_.alignment()){
      const auto it=event.diagnostics.find("alignment_state_code");
      summary["alignment_state"]=matcher_.alignment()->valid?"confirmed":
        ((it!=event.diagnostics.end()&&it->second==3)?"relocalizing":"waiting_confirmation");
    }
    summary["refinement_method"]=matcher_.config().matching_profile=="partial_contour"?"partial_3d_then_contour_xy_ground":
      (matcher_.config().matching_profile=="partial"?"gravity_point_to_plane_huber_irls":"pcl_gicp");
    {std::lock_guard<std::mutex> lock(mutex_);summary["worker_queue_drops"]=queue_drops_;summary["reset_discarded_inputs"]=reset_discarded_inputs_;summary["last_status"]=event.status;summary["last_reason"]=event.reason;summary["require_lio_quality"]=require_quality_;summary["last_diagnostics"]=YAML::Node();for(const auto& item:event.diagnostics)summary["last_diagnostics"][item.first]=item.second;status_=YAML::Dump(summary);}
    publishAdvanced();
  }
  void run() {
    try {
      while(true) {
        std::pair<eb::RobotSubmap::ConstPtr,int> item;int quality_robot=-1;
        bool idle=false,retry=false;
        {std::unique_lock<std::mutex> lock(mutex_);condition_.wait_for(lock,std::chrono::milliseconds(200),[this]{return stop_||!queue_.empty()||!quality_events_.empty()||retry_requested_;});
          if(stop_&&queue_.empty())break;
          retry=retry_requested_;retry_requested_=false;
          if(!quality_events_.empty()){quality_robot=quality_events_.front();quality_events_.pop_front();}
          else if(!queue_.empty()){item=queue_.front();queue_.pop_front();}else idle=true;}
        const double now_ros=ros::Time::now().toSec();
        const bool rollback=worker_ros_time_>=0&&now_ros<worker_ros_time_-.05;
        const int resets=registered_reset_mask_.exchange(0);
        if(rollback||resets==3)recordInvalidation(rollback?"time_rollback":"registered_odometry_reset");
        else if(resets)recordInvalidation("registered_odometry_reset",resets==1?0:1);
        worker_ros_time_=now_ros;
        if(!warm_start_waiting_&&matcher_.alignment()&&matcher_.alignment()->valid){
          std::array<eb::SubmapPtr,2> scans;{std::lock_guard<std::mutex> lock(recent_mutex_);scans=live_scans_;}
          auto check=matcher_.observeScans(scans[0],scans[1],now_ros);
          if(check){auto monitor=std::make_shared<eb::SubmapInput>();monitor->session="alignment_monitor";complete(*check,monitor);}
        }
        if(retry)matcher_.retryHistory();
        if(idle){if(!warm_start_waiting_&&(ros::WallTime::now()-last_history_search_).toSec()>=2){last_history_search_=ros::WallTime::now();
            auto started=std::chrono::steady_clock::now();auto task=matcher_.pollHistory();
            if(task){task->first.diagnostics["matching_processing_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();complete(task->first,task->second);}}
          publishAdvanced();continue;}
        if(quality_robot>=0) {
          matcher_.invalidate();auto input=std::make_shared<eb::SubmapInput>();input->robot=quality_robot;input->session="lio_quality_guard";
          eb::MatchEvent event;event.status="rejected";event.clear_display=true;
          event.reason="LIO quality quarantine: continuous degradation >=2 s; restart the full LIO/submap run";
          complete(event,input);continue;
        }
        eb::SubmapPtr input;
        try {input=decode(*item.first);if(input->robot!=item.second)throw std::invalid_argument("Robot identity disagrees with topic");}
        catch(const std::exception& e){
          auto invalid=std::make_shared<eb::SubmapInput>();invalid->robot=item.second;invalid->session=item.first->session_id;invalid->id=item.first->submap_id;
          eb::MatchEvent event;event.status="rejected";event.reason=e.what();complete(event,invalid);continue;}
        const auto generation=input->session+":"+std::to_string(input->epoch);
        if(!blocked_generations_[input->robot].empty()&&generation==blocked_generations_[input->robot]){
          ++reset_discarded_inputs_;eb::MatchEvent event;event.status="rejected";event.reason="clock or odometry reset requires a new Stage 2 epoch";
          event.diagnostics["map_ugv_voxels"]=matcher_.mapCloud(0).size();event.diagnostics["map_uav_voxels"]=matcher_.mapCloud(1).size();
          complete(event,input);continue;}
        blocked_generations_[input->robot].clear();
        consume(input);
      }
      writer_.checkpoint(matcher_);writer_.finish("stopped");
    }catch(const std::exception& e){
      failed_=true;ROS_FATAL_STREAM("Stage 3 stopped: "<<e.what());
      try{writer_.finish("failed",e.what());}catch(const std::exception& ex){ROS_ERROR_STREAM("Cannot save failure report: "<<ex.what());}
      ros::shutdown();
    }
  }
  eb::Matcher matcher_;eb::MatchWriter writer_;
  using RecentSync=message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>;
  std::array<std::unique_ptr<eb::RecentCloudBuffer>,2> recent_buffers_;
  std::array<std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>,2> recent_cloud_subs_;
  std::array<std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>>,2> recent_odom_subs_;
  std::array<std::unique_ptr<RecentSync>,2> recent_sync_;
  std::mutex recent_mutex_;double recent_clock_=-1;
  std::array<eb::SubmapPtr,2> live_scans_;
  int parent_launch_pid_=0;
  ros::Publisher match_pub_,ugv_pub_,uav_pub_,fused_pub_,fused_compat_pub_,status_pub_;std::vector<ros::Subscriber> subscribers_;ros::WallTimer timer_;
  ros::Publisher map_pub_,alignment_pub_;ros::ServiceServer retry_service_;
  bool retry_requested_=false;
  std::atomic<int> registered_reset_mask_{0};
  double worker_ros_time_=-1;
  std::array<std::string,2> last_input_generations_,blocked_generations_;
  uint64_t reset_discarded_inputs_=0;
  bool warm_start_waiting_=false;
  bool replay_mode_=false;
  std::array<bool,2> warm_current_seen_{};
  uint64_t published_map_revision_=std::numeric_limits<uint64_t>::max(),published_alignment_revision_=std::numeric_limits<uint64_t>::max();
  ros::WallTime last_map_publish_,last_checkpoint_,last_history_search_;
  bool published_map_valid_=false;
  bool have_fused_=false,require_quality_=false;eb::LioQualityGuard quality_;std::deque<int> quality_events_;
  std::mutex mutex_;std::condition_variable condition_;std::thread worker_;
  std::deque<std::pair<eb::RobotSubmap::ConstPtr,int>> queue_;bool stop_=false;std::atomic<bool> failed_{false};
  size_t queue_limit_=64;uint64_t queue_drops_=0;std::string status_="status: waiting_for_submaps";
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"stage3_matcher");ros::NodeHandle nh,priv("~");
  std::string config,root,mode,dataset;priv.getParam("config",config);priv.getParam("output_root",root);priv.param<std::string>("mode",mode,"live");
  try {
    if(mode!="live"&&mode!="replay"&&mode!="ideal_frontend")throw std::runtime_error("mode must be live, replay or ideal_frontend");
    Node node(nh,config,root,mode);
    if(mode=="replay") {
      if(!priv.getParam("input_directory",dataset))throw std::runtime_error("input_directory is required");
      node.replay(dataset);bool keep_alive=false;priv.param("keep_alive",keep_alive,false);
      if(keep_alive){node.start();ros::spin();node.stop();}
    }else{node.start();ros::spin();node.stop();}
    return node.failed()?1:0;
  }catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 1;}
}
