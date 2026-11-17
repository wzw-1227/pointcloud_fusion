#include <ehpr_bridge/submap_session.h>
#include <ehpr_bridge/RobotSubmap.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <message_filters/simple_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <chrono>
#include <ctime>
#include <csignal>
#include <unistd.h>

namespace fs = std::filesystem;
namespace eb = ehpr_bridge;
constexpr const char* kCommit = "b4b7137f7e675974e025fc3c8e0ab891d353739a";
static volatile std::sig_atomic_t stop_requested = 0;
static void requestStop(int) { stop_requested = 1; }

static void saveYaml(const fs::path& path, const YAML::Node& data) {
  const fs::path temporary = path.string() + ".tmp";
  std::ofstream out(temporary);
  out << YAML::Dump(data) << '\n';
  out.close();
  if (!out) throw std::runtime_error("Cannot write " + temporary.string());
  fs::rename(temporary, path);
}
static YAML::Node xmlToYaml(XmlRpc::XmlRpcValue value) {
  YAML::Node out;
  using V = XmlRpc::XmlRpcValue;
  switch (value.getType()) {
    case V::TypeBoolean: out = static_cast<bool>(value); break;
    case V::TypeInt: out = static_cast<int>(value); break;
    case V::TypeDouble: out = static_cast<double>(value); break;
    case V::TypeString: out = static_cast<std::string>(value); break;
    case V::TypeArray:
      out = YAML::Node(YAML::NodeType::Sequence);
      for (int i = 0; i < value.size(); ++i) out.push_back(xmlToYaml(value[i]));
      break;
    case V::TypeStruct:
      out = YAML::Node(YAML::NodeType::Map);
      for (auto i = value.begin(); i != value.end(); ++i) out[i->first] = xmlToYaml(i->second);
      break;
    default: break;
  }
  return out;
}
static YAML::Node matrixYaml(const Eigen::Matrix4d& matrix) {
  YAML::Node out;
  for (int r = 0; r < 4; ++r) {
    YAML::Node row;
    for (int c = 0; c < 4; ++c) row.push_back(matrix(r, c));
    out.push_back(row);
  }
  return out;
}
static geometry_msgs::Quaternion quaternionMsg(const Eigen::Quaterniond& q) {
  geometry_msgs::Quaternion out;
  out.x = q.x(); out.y = q.y(); out.z = q.z(); out.w = q.w();
  return out;
}
static std::string sessionName() {
  const auto now = std::chrono::system_clock::now();
  // User-facing run times are Asia/Shanghai, independent of host timezone.
  const std::time_t shanghai = std::chrono::system_clock::to_time_t(now) + 8 * 3600;
  std::tm calendar{};
  gmtime_r(&shanghai, &calendar);
  const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
  std::ostringstream out;
  out << std::put_time(&calendar, "%Y%m%d_%H%M%S_") << std::setw(6) << std::setfill('0') << micros << '_' << getpid();
  return out.str();
}
template <class T> class InputFilter : public message_filters::SimpleFilter<T> {
 public:
  void input(const boost::shared_ptr<const T>& message) { this->signalMessage(message); }
};

class RobotBridge {
  using Policy = message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>;
  using Sync = message_filters::Synchronizer<Policy>;
 public:
  RobotBridge(ros::NodeHandle& nh, const std::string& name, const YAML::Node& robot,
              const YAML::Node& config, const fs::path& directory, const std::string& session,
              const eb::SubmapConfig& map_config)
      : name_(name), session_(session), config_(robot), directory_(directory), core_(map_config) {
    frame_ = robot["reference_frame"].as<std::string>();
    child_ = robot["mapping_child"].as<std::string>();
    robot_id_ = robot["robot_id"].as<int>();
    preview_interval_=robot["preview_interval_seconds"]?robot["preview_interval_seconds"].as<double>():0;
    preview_warmup_=robot["preview_warmup_seconds"]?robot["preview_warmup_seconds"].as<double>():15;
    preview_min_frames_=robot["preview_min_frames"]?robot["preview_min_frames"].as<int>():20;
    preview_initial_displacement_=robot["preview_initial_displacement_m"]?robot["preview_initial_displacement_m"].as<double>():0;
    if(!std::isfinite(preview_interval_)||preview_interval_<0||!std::isfinite(preview_warmup_)||preview_warmup_<0||
       preview_min_frames_<1||!std::isfinite(preview_initial_displacement_)||preview_initial_displacement_<0)
      throw std::runtime_error("Invalid live preview configuration");
    if (robot_id_ != (name == "ugv" ? 0 : 1)) throw std::runtime_error("Invalid robot_id for " + name);
    queue_ = config["sync_queue_size"].as<int>();
    gap_ = config["max_gap_seconds"].as<double>();
    translation_jump_ = config["max_translation_jump_m"].as<double>();
    rotation_jump_ = config["max_rotation_jump_deg"].as<double>();
    if (queue_ <= 0 || !std::isfinite(gap_) || gap_ <= 0 ||
        !std::isfinite(translation_jump_) || translation_jump_ <= 0 ||
        !std::isfinite(rotation_jump_) || rotation_jump_ <= 0)
      throw std::runtime_error("Invalid synchronization/jump configuration");
    const std::string ns = robot["lio_namespace"].as<std::string>();
    parameter_root_ = (ns == "/" ? "" : ns) + "/lio_sam";
    bool loop_enabled = true;
    if (!nh.getParam(parameter_root_ + "/loopClosureEnableFlag", loop_enabled) || loop_enabled)
      throw std::runtime_error(name + ": restart LIO with loopClosureEnableFlag=false before attaching the builder");
    std::string actual_frame, actual_child;
    if (!nh.getParam(parameter_root_ + "/odometryFrame", actual_frame) || actual_frame != frame_ ||
        !nh.getParam(parameter_root_ + "/odomChildFrame", actual_child) || actual_child != child_)
      throw std::runtime_error(name + ": LIO frames disagree with Stage 2 configuration");
    ros::NodeHandle topics(nh, "/stage2/" + name);
    submap_pub_ = topics.advertise<eb::RobotSubmap>("submap", 1, true);
    cloud_pub_ = topics.advertise<sensor_msgs::PointCloud2>("submap_cloud", 1, true);
    diagnostics_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
    flush_service_ = topics.advertiseService("flush", &RobotBridge::flushService, this);
    createSynchronizer();
    cloud_sub_ = nh.subscribe(robot["cloud_topic"].as<std::string>(), queue_, &RobotBridge::cloudReceived, this);
    odom_sub_ = nh.subscribe(robot["odometry_topic"].as<std::string>(), queue_, &RobotBridge::odomReceived, this);
  }

  YAML::Node parameters(ros::NodeHandle& nh) const {
    XmlRpc::XmlRpcValue value;
    nh.getParam(parameter_root_, value);
    return xmlToYaml(value);
  }
  YAML::Node summary() const {
    YAML::Node out;
    out["robot_id"] = robot_id_; out["reference_frame"] = frame_;
    out["epoch"] = epoch_; out["matched_frames"] = matched_;
    out["cloud_received"] = cloud_received_; out["odometry_received"] = odom_received_;
    out["duplicate_or_late_drops"] = duplicate_drops_; out["unpaired_drops"] = unpaired_drops_;
    out["invalid_drops"] = invalid_drops_; out["expired_pair_drops"] = expired_pair_drops_;
    out["exports"] = exports_; out["travel_distance_m"] = core_.travelDistance();
    out["pending_tail"] = core_.pending(); out["resets"] = resets_;
    out["error"] = error_; out["last_reset_reason"] = last_reset_;
    out["preview_interval_seconds"]=preview_interval_;out["preview_initial_motion_seen"]=preview_motion_seen_;
    out["preview_epoch_frames"]=preview_frames_;
    return out;
  }

  void tick() {
    const double now = ros::Time::now().toSec();
    checkClock();
    expire(cloud_pending_, now);
    expire(odom_pending_, now);
    if(preview_interval_>0 && error_.empty() && have_match_ && preview_motion_seen_ &&
       now-last_match_stamp_<=gap_ && last_match_stamp_-preview_first_stamp_>=preview_warmup_ &&
       preview_frames_>=static_cast<uint64_t>(preview_min_frames_) &&
       last_match_stamp_-last_preview_stamp_>=preview_interval_) {
      auto snapshot=core_.preview(last_pose_,last_match_stamp_);
      if(snapshot){persist(*snapshot,"online_preview");last_preview_stamp_=last_match_stamp_;}
    }
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "stage2/" + name_;
    status.hardware_id = "EHPR " + session_;
    status.level = !error_.empty() ? status.ERROR : matched_ ? status.OK : status.WARN;
    status.message = !error_.empty() ? error_ : matched_ ? "building submaps" : "waiting for synchronized LIO input";
    const YAML::Node data = summary();
    for (auto entry : data) {
      diagnostic_msgs::KeyValue value;
      value.key = entry.first.as<std::string>();
      value.value = YAML::Dump(entry.second);
      status.values.push_back(value);
    }
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }
  void flush(const std::string& reason) {
    if (error_.empty()) {
      auto snapshot = core_.flush();
      if (snapshot) persist(*snapshot, reason);
    }
  }
 private:
  void createSynchronizer() {
    sync_ = std::make_unique<Sync>(Policy(queue_), cloud_filter_, odom_filter_);
    sync_->registerCallback(boost::bind(&RobotBridge::matched, this, boost::placeholders::_1, boost::placeholders::_2));
  }
  void resetEpoch(const std::string& reason) {
    core_.reset(); ++epoch_; ++resets_; have_match_ = false;
    preview_frames_=0;preview_motion_seen_=false;
    last_reset_ = reason;
    ROS_WARN_STREAM(name_ << " starts epoch " << epoch_ << ": " << reason);
  }
  void checkClock() {
    const double now = ros::Time::now().toSec();
    if (have_clock_ && now < last_clock_) {
      resetEpoch("ROS time moved backwards");
      cloud_pending_.clear(); odom_pending_.clear();
      have_cloud_stamp_ = have_odom_stamp_ = false;
      // Called outside Synchronizer callbacks, so reconstructing it is safe.
      createSynchronizer();
    }
    have_clock_ = true; last_clock_ = now;
  }
  void expire(std::map<uint64_t, double>& pending, double now) {
    for (auto it = pending.begin(); it != pending.end();) {
      if (now - it->second > gap_) { ++unpaired_drops_; it = pending.erase(it); }
      else ++it;
    }
  }
  bool timestamp(const ros::Time& stamp, bool& have, uint64_t& last,
                 std::map<uint64_t, double>& pending) {
    const uint64_t key = stamp.toNSec();
    if (have && key <= last) { ++duplicate_drops_; return false; }
    have = true; last = key;
    pending[key] = stamp.toSec();
    if (pending.size() > static_cast<size_t>(queue_)) { pending.erase(pending.begin()); ++unpaired_drops_; }
    return true;
  }
  void cloudReceived(const sensor_msgs::PointCloud2::ConstPtr& m) {
    checkClock(); ++cloud_received_;
    if (m->header.frame_id != frame_ || !m->width || !m->height ||
        m->point_step == 0 || m->row_step < static_cast<uint64_t>(m->width) * m->point_step ||
        m->data.size() < static_cast<uint64_t>(m->row_step) * m->height) {
      ++invalid_drops_; ROS_WARN_THROTTLE(5, "Rejected invalid Stage 2 cloud/frame"); return;
    }
    for (const std::string& field : {"x", "y", "z"}) {
      const auto it = std::find_if(m->fields.begin(), m->fields.end(), [&](const auto& f) { return f.name == field; });
      if (it == m->fields.end() || it->count != 1 || it->datatype != sensor_msgs::PointField::FLOAT32 ||
          it->offset + 4 > m->point_step) { ++invalid_drops_; return; }
    }
    if (!timestamp(m->header.stamp, have_cloud_stamp_, last_cloud_stamp_, cloud_pending_)) return;
    cloud_filter_.input(m);
  }
  void odomReceived(const nav_msgs::Odometry::ConstPtr& m) {
    checkClock(); ++odom_received_;
    const auto& p = m->pose.pose.position; const auto& q = m->pose.pose.orientation;
    Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    if (m->header.frame_id != frame_ || m->child_frame_id != child_ ||
        !Eigen::Vector3d(p.x, p.y, p.z).allFinite() || !rotation.coeffs().allFinite() ||
        std::abs(rotation.norm() - 1) > 1e-3) {
      ++invalid_drops_; ROS_WARN_THROTTLE(5, "Rejected invalid Stage 2 odometry/frame"); return;
    }
    if (!timestamp(m->header.stamp, have_odom_stamp_, last_odom_stamp_, odom_pending_)) return;
    odom_filter_.input(m);
  }
  void matched(const sensor_msgs::PointCloud2::ConstPtr& cloud,
               const nav_msgs::Odometry::ConstPtr& odom) {
    const uint64_t key = cloud->header.stamp.toNSec();
    const bool cloud_exists = cloud_pending_.erase(key) != 0;
    const bool odom_exists = odom_pending_.erase(key) != 0;
    const double stamp = cloud->header.stamp.toSec();
    if (!cloud_exists || !odom_exists || ros::Time::now().toSec() - stamp > gap_) {
      ++expired_pair_drops_; return;
    }
    const auto& p = odom->pose.pose.position; const auto& q = odom->pose.pose.orientation;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    if (have_match_ && stamp <= last_match_stamp_) { ++duplicate_drops_; return; }
    if (have_match_) {
      const double angle = Eigen::AngleAxisd(last_pose_.rotation().transpose() * pose.rotation()).angle() * 180 / M_PI;
      if (stamp - last_match_stamp_ > gap_) resetEpoch("synchronized input gap exceeds configured limit");
      else if ((pose.translation() - last_pose_.translation()).norm() > translation_jump_ || angle > rotation_jump_)
        resetEpoch("odometry pose jump exceeds configured limit");
    }
    eb::Cloud::Ptr input(new eb::Cloud);
    pcl::fromROSMsg(*cloud, *input);
    input->points.erase(std::remove_if(input->points.begin(), input->points.end(), [](const auto& p) {
      return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
    }), input->points.end());
    input->width = input->points.size(); input->height = 1; input->is_dense = true;
    if (input->empty()) { ++invalid_drops_; return; }
    auto snapshot = core_.add(input, pose, stamp);
    if(!have_match_) {
      preview_first_stamp_=last_preview_stamp_=stamp;preview_first_pose_=pose;
      preview_motion_seen_=preview_initial_displacement_==0;
    }
    if((pose.translation()-preview_first_pose_.translation()).norm()>=preview_initial_displacement_)preview_motion_seen_=true;
    ++preview_frames_;
    have_match_ = true; last_match_stamp_ = stamp; last_pose_ = pose; ++matched_;
    if (snapshot) persist(*snapshot, "distance");
  }
  bool flushService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (!error_.empty() || !core_.pending()) {
      response.success = false;
      response.message = error_.empty() ? "No new synchronized input since previous export" : error_;
      return true;
    }
    flush("manual");
    response.success = true;
    response.message = last_export_path_;
    return true;
  }
  void persist(const eb::SubmapSnapshot& snapshot, const std::string& reason) {
    std::ostringstream filename;
    filename << name_ << "_submap_" << std::setw(3) << std::setfill('0') << next_id_;
    const std::string base = filename.str();
    const fs::path pcd = directory_ / (base + ".pcd");
    try {
      const fs::path temp = pcd.string() + ".tmp";
      if (fs::exists(pcd) || fs::exists(directory_ / (base + ".yaml")))
        throw std::runtime_error("Refusing to overwrite " + pcd.string());
      if (pcl::io::savePCDFileBinary(temp.string(), *snapshot.local) != 0)
        throw std::runtime_error("PCD write failed: " + temp.string());
      YAML::Node meta;
      meta["session_id"] = session_; meta["robot"] = name_; meta["robot_id"] = robot_id_;
      meta["epoch"] = epoch_; meta["submap_id"] = next_id_;
      meta["reference_frame"] = frame_;
      const std::string local_frame = name_ + "_submap_" + session_ + "_" + std::to_string(next_id_);
      meta["local_frame"] = local_frame; meta["keyframe_timestamp"] = snapshot.key_stamp;
      meta["segment_start_timestamp"] = snapshot.start_stamp; meta["segment_end_timestamp"] = snapshot.end_stamp;
      meta["segment_travel_distance_m"] = snapshot.travel_distance; meta["segment_frames"] = snapshot.segment_frames;
      meta["point_count"] = snapshot.local->size(); meta["partial"] = snapshot.partial;
      meta["reason"] = reason; meta["pcd"] = pcd.filename().string();
      meta["source_cloud_topic"] = config_["cloud_topic"];
      meta["source_odometry_topic"] = config_["odometry_topic"];
      meta["T_reference_key"] = matrixYaml(snapshot.pose.matrix());
      Eigen::Matrix4d gravity = Eigen::Matrix4d::Identity(); gravity.topLeftCorner<3, 3>() = snapshot.gravity.toRotationMatrix();
      meta["T_gravity_key"] = matrixYaml(gravity);
      meta["gravity_source"] = "LIO lidar orientation roll/pitch; reference Z is gravity-aligned";
      meta["coordinate_convention"] = "T_A_B maps B points into A; P_local = inverse(T_reference_key) * P_registered";
      meta["display_timestamp_policy"] = "zero selects latest TF; submap geometry is fixed in reference_frame";
      meta["retained_history"] = true;
      meta["upstream_commit"] = kCommit;
      saveYaml(directory_ / (base + ".yaml"), meta);
      fs::rename(temp, pcd);

      eb::RobotSubmap message;
      message.header.stamp.fromSec(snapshot.key_stamp); message.header.frame_id = frame_;
      message.session_id = session_; message.robot_id = robot_id_; message.epoch = epoch_; message.submap_id = next_id_;
      const auto xyz = snapshot.pose.translation();
      message.pose_in_lio_map.position.x = xyz.x(); message.pose_in_lio_map.position.y = xyz.y(); message.pose_in_lio_map.position.z = xyz.z();
      message.pose_in_lio_map.orientation = quaternionMsg(Eigen::Quaterniond(snapshot.pose.rotation()));
      message.gravity_alignment = quaternionMsg(snapshot.gravity);
      pcl::toROSMsg(*snapshot.local, message.cloud_local);
      message.cloud_local.header.stamp = message.header.stamp; message.cloud_local.header.frame_id = local_frame;
      message.partial = snapshot.partial; message.pcd_path = pcd.string();
      eb::Cloud reference;
      pcl::transformPointCloud(*snapshot.local, reference, Eigen::Affine3d(snapshot.pose));
      sensor_msgs::PointCloud2 display; pcl::toROSMsg(reference, display); display.header = message.header;
      // This is a visualization snapshot of geometry fixed in the LIO frame.
      // Time zero selects latest TF, including RViz started after a latched export.
      // Scan/anchor times remain precise in RobotSubmap and its YAML metadata.
      display.header.stamp = ros::Time(0);
      exports_.push_back(meta);
      ++next_id_;
      last_export_path_ = pcd.string();
      if (ros::ok()) { submap_pub_.publish(message); cloud_pub_.publish(display); }
      ROS_INFO_STREAM("Saved " << pcd << " (" << snapshot.local->size() << " points, " << reason << ')');
    } catch (const std::exception& ex) {
      error_ = "Cannot export " + pcd.string() + ": " + ex.what();
      ros::shutdown();
      throw std::runtime_error(error_);
    }
  }
  std::string name_, session_, frame_, child_, parameter_root_, error_, last_reset_, last_export_path_;
  YAML::Node config_, exports_ = YAML::Node(YAML::NodeType::Sequence);
  fs::path directory_;
  eb::SubmapSession core_;
  int robot_id_ = 0, queue_ = 100;
  double gap_ = 1, translation_jump_ = 1, rotation_jump_ = 45, last_match_stamp_ = 0, last_clock_ = 0;
  uint32_t epoch_ = 0, next_id_ = 0;
  uint64_t cloud_received_ = 0, odom_received_ = 0, matched_ = 0, duplicate_drops_ = 0;
  uint64_t invalid_drops_ = 0, unpaired_drops_ = 0, expired_pair_drops_ = 0, resets_ = 0;
  uint64_t last_cloud_stamp_ = 0, last_odom_stamp_ = 0;
  bool have_cloud_stamp_ = false, have_odom_stamp_ = false, have_match_ = false, have_clock_ = false;
  Eigen::Isometry3d last_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d preview_first_pose_=Eigen::Isometry3d::Identity();
  double preview_interval_=0,preview_warmup_=15,preview_initial_displacement_=0,preview_first_stamp_=0,last_preview_stamp_=0;
  int preview_min_frames_=20;uint64_t preview_frames_=0;bool preview_motion_seen_=false;
  std::map<uint64_t, double> cloud_pending_, odom_pending_;
  InputFilter<sensor_msgs::PointCloud2> cloud_filter_;
  InputFilter<nav_msgs::Odometry> odom_filter_;
  std::unique_ptr<Sync> sync_;
  ros::Subscriber cloud_sub_, odom_sub_;
  ros::Publisher submap_pub_, cloud_pub_, diagnostics_pub_;
  ros::ServiceServer flush_service_;
};

int main(int argc, char** argv) {
  // Keep publishers connected while the shutdown tail is persisted and sent.
  ros::init(argc, argv, "stage2_submap_builder", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  ros::NodeHandle nh, private_nh("~");
  std::string config_path, selection, output_root;
  private_nh.param<std::string>("robot", selection, "both");
  if (!private_nh.getParam("config", config_path) || !private_nh.getParam("output_root", output_root)) {
    ROS_FATAL("config and output_root parameters are required"); return 1;
  }
  std::vector<std::unique_ptr<RobotBridge>> robots;
  fs::path directory;
  YAML::Node run;
  try {
    if (selection != "ugv" && selection != "uav" && selection != "both") throw std::runtime_error("robot must be ugv, uav or both");
    const YAML::Node config = YAML::LoadFile(config_path);
    if (config["upstream_commit"].as<std::string>() != kCommit) throw std::runtime_error("Configuration upstream version does not match compiled adapter");
    eb::SubmapConfig map_config;
    map_config.voxel_size = config["voxel_map"]["voxel_size"].as<double>();
    map_config.max_distance = config["voxel_map"]["max_distance"].as<double>();
    map_config.remove_distance = config["voxel_map"]["remove_distance"].as<double>();
    map_config.max_points_per_voxel = config["voxel_map"]["max_points_per_voxel"].as<int>();
    const std::string session = sessionName();
    directory = fs::absolute(output_root).lexically_normal() / session;
    if (!fs::create_directories(directory)) throw std::runtime_error("Run directory already exists");
    fs::copy_file(config_path, directory / "config.yaml");
    run["session_id"] = session; run["timezone"] = "Asia/Shanghai"; run["robot"] = selection;
    run["upstream_commit"] = kCommit; run["status"] = "initializing";
    saveYaml(directory / "summary.yaml", run);
    YAML::Node parameters;
    parameters["builder_config"] = config;
    for (const std::string& name : {"ugv", "uav"}) {
      if (selection == "both" || selection == name) {
        robots.push_back(std::make_unique<RobotBridge>(nh, name, config["robots"][name], config, directory, session, map_config));
        parameters["lio_parameters"][name] = robots.back()->parameters(nh);
      }
    }
    saveYaml(directory / "parameters.yaml", parameters);
    run["status"] = "running";
    ros::WallTimer timer = nh.createWallTimer(ros::WallDuration(1), [&](const ros::WallTimerEvent&) {
      YAML::Node observations;
      for (size_t i = 0; i < robots.size(); ++i) {
        robots[i]->tick();
        observations[robots[i]->summary()["robot_id"].as<int>() == 0 ? "ugv" : "uav"] = robots[i]->summary();
      }
      run["robots"] = observations;
      saveYaml(directory / "summary.yaml", run);
    });
    ROS_INFO_STREAM("Stage 2 " << selection << " output: " << directory);
    ros::WallRate callbacks(100);
    while (ros::ok() && !stop_requested) {
      ros::spinOnce();
      callbacks.sleep();
    }
    for (auto& robot : robots)
      if (!robot->summary()["error"].as<std::string>().empty())
        throw std::runtime_error(robot->summary()["error"].as<std::string>());
    for (auto& robot : robots) robot->flush("shutdown");
    run["status"] = "stopped";
    for (auto& robot : robots) run["robots"][robot->summary()["robot_id"].as<int>() == 0 ? "ugv" : "uav"] = robot->summary();
    saveYaml(directory / "summary.yaml", run);
    ros::WallDuration(0.1).sleep();
    ros::shutdown();
    return 0;
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Stage 2 stopped: " << error.what());
    run["status"] = "failed"; run["error"] = error.what();
    for (auto& robot : robots) run["robots"][robot->summary()["robot_id"].as<int>() == 0 ? "ugv" : "uav"] = robot->summary();
    if (!directory.empty()) {
      try { saveYaml(directory / "summary.yaml", run); }
      catch (const std::exception& failure) { ROS_ERROR_STREAM("Cannot save failure summary: " << failure.what()); }
    }
    return 1;
  }
}
