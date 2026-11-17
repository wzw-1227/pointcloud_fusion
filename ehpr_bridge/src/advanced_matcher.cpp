#include <ehpr_bridge/advanced_matcher.h>
#include <ehpr_bridge/alignment_tracker.h>
#include <ehpr/desc_db.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/persistence.hpp>
#include <fstream>
#include <iostream>
#include <atomic>
#include <unistd.h>

namespace ehpr_bridge {
namespace {
namespace fs=std::filesystem;
constexpr double pi=3.14159265358979323846;
Eigen::Quaterniond g(SubmapPtr p){return p->matching_gravity.value_or(canonicalMatchingGravity(p->pose.rotation()));}
YAML::Node matrix(const Eigen::Matrix4d& m){YAML::Node n;for(int i=0;i<4;++i)for(int j=0;j<4;++j)n[i].push_back(m(i,j));return n;}
void yamlFile(const fs::path& p,const YAML::Node& n){auto tmp=p;tmp+=".tmp";std::ofstream out(tmp);out<<n;out.close();if(!out)throw std::runtime_error("Cannot persist history metadata");fs::rename(tmp,p);}
SubmapPtr light(SubmapPtr p){auto q=std::make_shared<SubmapInput>(*p);q->cloud.reset();q->recent_cloud.reset();q->latest_scan.reset();q->visibility_scan.reset();return q;}
using D=std::shared_ptr<DescriptorManager>;
const std::array<std::pair<double,int>,6> profiles={{{.5,3},{.5,1},{.25,3},{.25,1},{.5,1},{.25,1}}};
Cloud::Ptr localCloud(SubmapPtr input,const MatcherConfig& c){
  Cloud::Ptr out(new Cloud);const auto& source=input->recent_cloud?input->recent_cloud:input->cloud;
  double radius=input->robot?c.matching_radius_uav:std::max(c.matching_radius_ugv,c.matching_radius_ugv_fallback);
  auto gravity=g(input);Eigen::Vector3d center=gravity*input->recent_center_local;
  if(source)for(const auto& p:*source){Eigen::Vector3d x=gravity*p.getVector3fMap().cast<double>();
    if(radius<=0||(x-center).head<2>().norm()<=radius)out->push_back(p);}return out;
}
D descriptor(SubmapPtr input,const Cloud::Ptr& cloud,int id,int profile,double angle){
  DescriptorManagerConfig cfg;cfg.n_row_=cfg.n_col_=210;cfg.reso_row_=cfg.reso_col_=profiles[profile].first;
  cfg.local_density_rows_=cfg.local_density_cols_=profiles[profile].second;cfg.height_res_=1;cfg.max_dens_=135;
  cfg.planar_bev_=profile>=4;
  auto out=std::make_shared<DescriptorManager>(cfg,id);Cloud aligned;
  Eigen::Matrix3d r=Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix();if(profile<4)r*=g(input).toRotationMatrix();
  double lo=1e9,hi=-lo;if(cloud)for(const auto& p:*cloud){Eigen::Vector3d x=r*p.getVector3fMap().cast<double>();aligned.push_back(pcl::PointXYZ(x.x(),x.y(),x.z()));lo=std::min(lo,x.z());hi=std::max(hi,x.z());}
  if(aligned.size()<20)return nullptr;
  if(profile>=4)out->makePlanarBEV(aligned);
  else{if(hi-lo<1e-5)return nullptr;Cloud::ConstPtr view(new Cloud(aligned));out->makeDensityBEV<pcl::PointXYZ>(view,input->identity());}
  out->makeFeature();out->releaseConstructionBuffers();return out;
}
}

struct AdvancedMatcher::Impl {
  struct Record {SubmapPtr input;fs::path path;std::array<D,6> descriptors;};
  struct Geometry {SubmapPtr input;Cloud::Ptr local;std::shared_ptr<PartialGeometry> partial;std::shared_ptr<ContourGeometry> contour;uint64_t touch=0,ground_revision=0;};
  MatcherConfig config;std::unique_ptr<Matcher> low;
  SessionMap maps;AlignmentTracker tracker;std::array<GroundAnchor,2> grounds;
  std::array<std::string,2> generations;std::set<std::string> seen,accepted_pairs;
  std::array<std::vector<Record>,2> records;
  std::array<std::array<std::unique_ptr<DescriptorDB>,6>,2> indices;
  std::array<std::array<size_t,6>,2> bank_counts{};
  std::array<std::map<size_t,Geometry>,2> geometry;
  std::array<SubmapPtr,2> latest;
  std::array<std::optional<Geometry>,2> latest_geometry;
  std::deque<std::pair<int,size_t>> jobs;std::set<std::pair<int,size_t>> queued;
  std::map<std::string,uint64_t> counts;
  std::map<std::pair<int,size_t>,uint64_t> queried;
  fs::path directory;bool temporary=false;uint64_t revision=1,ground_revision=1,touch=0,archive_serial=0;double last_monitor_attempt=-1;
  explicit Impl(const MatcherConfig& c):config(c),maps(c.fusion_voxel_size),tracker(c,maps){
    auto base=c;base.matching_profile="partial";
    base.database_capacity=std::min(c.database_capacity,std::max(1,(c.history_geometry_capacity-2)/3));
    low=std::make_unique<Matcher>(base,true);resetIndices();
    static std::atomic<uint64_t> serial{0};directory=fs::temp_directory_path()/("stage3_history_"+std::to_string(getpid())+"_"+std::to_string(serial++));temporary=true;fs::create_directories(directory);
  }
  ~Impl(){if(temporary){std::error_code ec;fs::remove_all(directory,ec);}}
  void resetIndices(int only=-1){for(int robot=0;robot<2;++robot){if(only>=0&&robot!=only)continue;bank_counts[robot]={};for(int k=0;k<6;++k){DescriptorDBConfig d;
    d.enhanced_=true;d.skip_num_=-1;d.n_row_=d.n_col_=config.image_size;d.reso_row_=d.reso_col_=profiles[k].first;
    d.bucket_size_=config.buckets;d.dist_thres_=config.hamming_distance;d.distri_mean_=config.distribution_mean;d.distri_var_=config.distribution_std;
    d.planar_inlier_distance_=k<4?config.partial_planar_inlier_distance:.75;indices[robot][k]=std::make_unique<DescriptorDB>(d);
  }}}
  void reset(const std::string& reason){low->invalidate();maps.reset(reason);tracker.reset();last_monitor_attempt=-1;grounds={};generations={};seen.clear();accepted_pairs.clear();
    for(auto& r:records)r.clear();for(auto& r:geometry)r.clear();latest={};latest_geometry={};jobs.clear();queued.clear();queried.clear();resetIndices();++revision;++ground_revision;
    persistGround();persistIndex();
  }
  void resetRobot(int robot,const std::string& reason){low->invalidate();maps.resetRobot(robot,reason);tracker.reset();last_monitor_attempt=-1;
    grounds[robot]={};generations[robot].clear();records[robot].clear();geometry[robot].clear();latest[robot].reset();latest_geometry[robot].reset();
    const auto prefix=std::to_string(robot)+":";for(auto it=seen.begin();it!=seen.end();)if(it->rfind(prefix,0)==0)it=seen.erase(it);else ++it;
    accepted_pairs.clear();queried.clear();jobs.clear();queued.clear();resetIndices(robot);++revision;++ground_revision;
    for(size_t id=0;id<records[1-robot].size();++id)enqueue(1-robot,id);
    persistGround();persistIndex();
  }
  void ground(SubmapPtr input){auto candidate=estimateHistoricalGround(*input);auto& current=grounds[input->robot];
    if(candidate.conflict){if(current.valid)current.conflict=true;++counts["ground_ambiguities"];++ground_revision;++revision;return;}
    if(candidate.valid&&current.valid&&std::abs(candidate.height-current.height)>.05){
      current.conflict=true;++counts["ground_conflicts"];++ground_revision;++revision;return;}
    if(candidate.valid&&(!current.valid||current.conflict||(candidate.support>current.support&&candidate.rmse<current.rmse))){
      current=candidate;++ground_revision;++revision;}
  }
  void persistGround(){YAML::Node n;n["version"]=1;n["revision"]=ground_revision;
    for(int robot=0;robot<2;++robot){const auto& a=grounds[robot];auto v=n[robot?"uav":"ugv"];
      v["valid"]=a.valid;v["conflict"]=a.conflict;v["source_identity"]=a.source;v["height_map_m"]=a.height;
      v["rmse_m"]=a.rmse;v["independent_voxels"]=a.support;v["span_m"]=a.span;v["second_direction_width_m"]=a.width;
      v["frame"]=robot?config.uav_frame:config.ugv_frame;v["generation"]=generations[robot];}
    yamlFile(directory/"ground.yaml",n);
  }
  void persistIndex(){YAML::Node index;index["version"]=1;
    for(int robot=0;robot<2;++robot)for(const auto& r:records[robot]){
      YAML::Node entry;entry["identity"]=r.input->identity();entry["observation"]=fs::relative(r.path,directory).string()+"/observation.yaml";
      entry["observation_quality_valid"]=r.input->observation_quality_valid;index["observations"].push_back(entry);}
    yamlFile(directory/"index.yaml",index);
  }
  void persist(const fs::path& folder,SubmapPtr p){fs::create_directories(folder);
    if(pcl::io::savePCDFileBinary((folder/"full.pcd").string(),*p->cloud))throw std::runtime_error("Cannot persist full history input");
    if(p->recent_cloud&&pcl::io::savePCDFileBinary((folder/"recent.pcd").string(),*p->recent_cloud))throw std::runtime_error("Cannot persist observation window");
    if(p->latest_scan&&pcl::io::savePCDFileBinary((folder/"scan.pcd").string(),*p->latest_scan))throw std::runtime_error("Cannot persist single scan");
    YAML::Node n;n["version"]=1;n["identity"]=p->identity();n["robot_id"]=p->robot;n["session_id"]=p->session;n["epoch"]=p->epoch;n["submap_id"]=p->id;
    n["reference_frame"]=p->frame;n["local_frame"]=p->local_frame;n["keyframe_timestamp"]=p->stamp;n["T_reference_key"]=matrix(p->pose.matrix());
    Eigen::Matrix4d gm=Eigen::Matrix4d::Identity();gm.topLeftCorner<3,3>()=g(p).toRotationMatrix();n["T_matching_gravity_key"]=matrix(gm);
    n["observation_quality_valid"]=p->observation_quality_valid;n["recent_start_stamp"]=p->recent_start_stamp;n["recent_end_stamp"]=p->recent_end_stamp;n["recent_frames"]=p->recent_frames;
    for(int i=0;i<3;++i)n["recent_center_local"].push_back(p->recent_center_local[i]);yamlFile(folder/"observation.yaml",n);
  }
  SubmapPtr load(int robot,size_t id){auto out=std::make_shared<SubmapInput>(*records[robot].at(id).input);const auto& path=records[robot][id].path;
    out->cloud.reset(new Cloud);if(pcl::io::loadPCDFile((path/"full.pcd").string(),*out->cloud)<0)throw std::runtime_error("Cannot load historical full cloud");
    if(fs::exists(path/"recent.pcd")){out->recent_cloud.reset(new Cloud);if(pcl::io::loadPCDFile((path/"recent.pcd").string(),*out->recent_cloud)<0)throw std::runtime_error("Cannot load historical observation");}
    if(fs::exists(path/"scan.pcd")){out->latest_scan.reset(new Cloud);if(pcl::io::loadPCDFile((path/"scan.pcd").string(),*out->latest_scan)<0)throw std::runtime_error("Cannot load historical scan");}
    ++counts["history_disk_loads"];return out;
  }
  Geometry prepare(SubmapPtr input,double radius=-1){auto view=config;if(radius>=0){view.matching_radius_ugv=radius;view.matching_radius_ugv_fallback=0;}
    Geometry v;v.input=input;v.local=localCloud(input,view);auto gr=g(input);v.partial=preparePartialGeometry(v.local,gr,gr*input->recent_center_local);
    if(config.matching_profile=="partial_contour"&&grounds[input->robot].valid&&!grounds[input->robot].conflict&&input->recent_cloud)v.contour=prepareContour(*input,grounds[input->robot],config);
    auto prepared=std::make_shared<SubmapInput>(*input);
    if(grounds[input->robot].valid&&!grounds[input->robot].conflict)
      prepared->visibility_scan=prepareVisibilityScan(*prepared,grounds[input->robot].height-input->pose.translation().z(),config);
    else if(!v.partial->horizontal_planes.empty())prepared->visibility_scan=prepareVisibilityScan(*prepared,
      std::min_element(v.partial->horizontal_planes.begin(),v.partial->horizontal_planes.end(),[](const auto& a,const auto& b){return a.height<b.height;})->height,config);
    v.input=prepared;v.ground_revision=ground_revision;v.touch=++touch;return v;
  }
  size_t cacheLimit(int robot) const {
    size_t reserved=robot==0?2*low->config().database_capacity+1:3;
    return std::max(size_t(1),size_t(config.history_geometry_capacity)>reserved?size_t(config.history_geometry_capacity)-reserved:1);
  }
  Geometry& get(int robot,size_t id){auto& cache=geometry[robot];auto it=cache.find(id);
    if(id==records[robot].size()&&latest_geometry[robot]){
      auto& v=*latest_geometry[robot];if(v.ground_revision!=ground_revision){v=prepare(v.input);}return v;
    }
    if(it==cache.end()){
      if(cache.size()>=cacheLimit(robot)){auto old=std::min_element(cache.begin(),cache.end(),[](const auto& a,const auto& b){return a.second.touch<b.second.touch;});cache.erase(old);++counts["history_geometry_evictions"];}
      it=cache.emplace(id,prepare(load(robot,id))).first;
    }
    it->second.touch=++touch;
    if(it->second.ground_revision!=ground_revision){if(grounds[robot].valid&&!grounds[robot].conflict&&it->second.input->recent_cloud)
        it->second.contour=prepareContour(*it->second.input,grounds[robot],config);it->second.ground_revision=ground_revision;}
    return it->second;
  }
  SubmapPtr observation(int robot,size_t id){return id==records[robot].size()?latest_geometry[robot]->input:records[robot].at(id).input;}
  void descriptorFiles(Record& rec){cv::FileStorage out((rec.path/"descriptors.yaml").string(),cv::FileStorage::WRITE);
    for(int k=0;k<6;++k)if(rec.descriptors[k]){auto d=rec.descriptors[k];out<<("orb_"+std::to_string(k))<<d->bev_descriptors_;
      cv::Mat points(d->getLevKeyPointsSize(),2,CV_64F);for(int j=0;j<points.rows;++j){auto p=d->getKeyPoint(j);points.at<double>(j,0)=p.x();points.at<double>(j,1)=p.y();}out<<("keypoints_"+std::to_string(k))<<points;
      std::vector<std::string> bins;for(int j=0;j<points.rows;++j){std::string s;boost::to_string(d->getKeyHeightBin(j),s);bins.push_back(s);}out<<("height_bins_"+std::to_string(k))<<bins;
    }if(!out.isOpened())throw std::runtime_error("Cannot persist historical descriptors");
  }
  bool novel(SubmapPtr p,const Geometry& v){const auto& rs=records[p->robot];if(rs.empty())return true;auto previous=rs.back().input;
    if(p->recent_frames&&!previous->recent_frames)return true;
    if((p->pose.translation()-previous->pose.translation()).norm()>=config.history_translation||
       Eigen::AngleAxisd(previous->pose.rotation().transpose()*p->pose.rotation()).angle()>=config.history_rotation_deg*pi/180)return true;
    auto& old=get(p->robot,rs.size()-1);std::set<std::array<int64_t,3>> cells;
    for(const auto& x:*old.local){Eigen::Vector3d a=old.input->pose*x.getVector3fMap().cast<double>();cells.insert({int64_t(std::floor(a.x()/.5)),int64_t(std::floor(a.y()/.5)),int64_t(std::floor(a.z()/.5))});}
    std::set<std::array<int64_t,3>> fresh;for(const auto& x:*v.local){Eigen::Vector3d a=p->pose*x.getVector3fMap().cast<double>();fresh.insert({int64_t(std::floor(a.x()/.5)),int64_t(std::floor(a.y()/.5)),int64_t(std::floor(a.z()/.5))});}
    size_t added=0;for(const auto& c:fresh)added+=!cells.count(c);return double(added)/std::max(size_t(1),fresh.size())>=config.history_novelty;
  }
  bool archive(SubmapPtr input,Geometry& v){if(!novel(input,v))return false;size_t id=records[input->robot].size();
    Record rec;rec.input=light(input);rec.path=directory/("observation_"+std::to_string(archive_serial++));persist(rec.path,input);
    for(int k=0;k<6;++k){if(k>=4&&config.matching_profile!="partial_contour")continue;
      if(k>=4&&input->robot==1&&(!v.contour||v.contour->height_above_ground<config.contour_min_height))continue;
      Cloud::Ptr points=k<4?v.local:(v.contour?v.contour->levels[2].cloud:Cloud::Ptr());rec.descriptors[k]=descriptor(input,points,int(bank_counts[input->robot][k]),k,0);
      if(rec.descriptors[k]&&rec.descriptors[k]->getLevKeyPointsSize()>=config.min_inliers){rec.descriptors[k]->attach_idx_=id;
        auto& db=indices[input->robot][k];db->getClassMatchables(rec.descriptors[k]);db->addClassToHBST(rec.descriptors[k]);++bank_counts[input->robot][k];}}
    descriptorFiles(rec);records[input->robot].push_back(std::move(rec));++revision;++counts["history_keyframes"];
    persistIndex();
    if(geometry[input->robot].size()>=cacheLimit(input->robot)){
      auto& cache=geometry[input->robot];auto old=std::min_element(cache.begin(),cache.end(),[](const auto& a,const auto& b){return a.second.touch<b.second.touch;});cache.erase(old);++counts["history_geometry_evictions"];}
    geometry[input->robot][id]=v;enqueue(input->robot,id);
    for(size_t j=0;j<records[1-input->robot].size();++j)enqueue(1-input->robot,j);return true;
  }
  void refreshContourIndex(int robot){
    for(int k=4;k<6;++k){DescriptorDBConfig d;d.enhanced_=true;d.skip_num_=-1;d.n_row_=d.n_col_=config.image_size;
      d.reso_row_=d.reso_col_=profiles[k].first;d.bucket_size_=config.buckets;d.dist_thres_=config.hamming_distance;
      d.distri_mean_=config.distribution_mean;d.distri_var_=config.distribution_std;d.planar_inlier_distance_=.75;
      indices[robot][k]=std::make_unique<DescriptorDB>(d);bank_counts[robot][k]=0;
      for(size_t id=0;id<records[robot].size();++id){auto& v=get(robot,id);auto& rec=records[robot][id];rec.descriptors[k].reset();
        if(!v.contour||(robot==1&&v.contour->height_above_ground<config.contour_min_height))continue;
        auto desc=descriptor(v.input,v.contour->levels[2].cloud,bank_counts[robot][k],k,0);
        if(!desc||desc->getLevKeyPointsSize()<config.min_inliers)continue;
        desc->attach_idx_=id;indices[robot][k]->getClassMatchables(desc);indices[robot][k]->addClassToHBST(desc);
        rec.descriptors[k]=desc;++bank_counts[robot][k];descriptorFiles(rec);}
    }++revision;++counts["historical_contour_index_rebuilds"];
  }
  void enqueue(int robot,size_t id){if(queued.insert({robot,id}).second)jobs.emplace_back(robot,id);}
  bool latestDistinct(int robot) const {return latest_geometry[robot]&&
    (records[robot].empty()||records[robot].back().input->identity()!=latest_geometry[robot]->input->identity());}
  MatchEvent search(Geometry& query,bool contour){
    MatchEvent event;event.status="query_pending";int role=query.input->robot,opposite=1-role;
    if(records[opposite].empty()){event.reason="waiting for opposite historical database";return event;}
    if(contour&&(!grounds[0].valid||!grounds[1].valid||grounds[0].conflict||grounds[1].conflict)){
      event.status="waiting_ground_anchor";event.reason="missing or conflicting historical ground anchor";return event;}
    struct Seed {CoarseCandidate c;size_t id;int profile;double angle;Eigen::Matrix4d pose;};std::vector<Seed> all;
    std::array<std::unique_ptr<DescriptorDB>,6> transient;
    if(latestDistinct(opposite)) {
      auto& v=get(opposite,records[opposite].size());for(int k=contour?4:0;k<(contour?6:4);++k){
        if(contour&&(!v.contour||(opposite==1&&v.contour->height_above_ground<config.contour_min_height)))continue;
        auto d=descriptor(v.input,contour?v.contour->levels[2].cloud:v.local,0,k,0);if(!d||d->getLevKeyPointsSize()<config.min_inliers)continue;
        d->attach_idx_=records[opposite].size();DescriptorDBConfig c;c.enhanced_=true;c.skip_num_=-1;c.n_row_=c.n_col_=config.image_size;
        c.reso_row_=c.reso_col_=profiles[k].first;c.bucket_size_=config.buckets;c.dist_thres_=config.hamming_distance;c.distri_mean_=config.distribution_mean;c.distri_var_=config.distribution_std;
        c.planar_inlier_distance_=contour?.75:config.partial_planar_inlier_distance;transient[k]=std::make_unique<DescriptorDB>(c);
        transient[k]->getClassMatchables(d);transient[k]->addClassToHBST(d);
      }
    }
    int begin=contour?4:0,end=contour?6:4;
    for(int k=begin;k<end;++k)for(int view=0;view<8;++view){double angle=view*pi/4;
      auto d=descriptor(query.input,contour?(query.contour?query.contour->levels[2].cloud:Cloud::Ptr()):query.local,int(query.input->id),k,angle);
      if(!d||d->getLevKeyPointsSize()<config.min_inliers)continue;auto& db=indices[opposite][k];db->getClassMatchables(d);
      auto candidates=db->queryFeatureCandidates(d,40);db->releaseQueryMatchables();
      if(transient[k]){transient[k]->getClassMatchables(d);auto extra=transient[k]->queryFeatureCandidates(d,4);candidates.insert(candidates.end(),extra.begin(),extra.end());transient[k]->releaseQueryMatchables();}
      for(const auto& c:candidates){if(c.inliers<size_t(config.min_inliers)||(!contour&&!c.height_valid))continue;
        Eigen::Matrix4d rq=Eigen::Matrix4d::Identity(),rr=rq;rq.topLeftCorner<3,3>()=Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix()*g(query.input).toRotationMatrix();
        rr.topLeftCorner<3,3>()=g(observation(opposite,c.reference_id)).toRotationMatrix();
        Eigen::Matrix4d pose=rq.inverse()*c.pose*rr;if(role==0)pose=pose.inverse().eval();all.push_back({c,size_t(c.reference_id),k,angle,pose});}
    }
    if(contour&&query.contour){std::set<size_t> shortlisted;for(const auto& s:all)shortlisted.insert(s.id);
      if(maps.alignment().valid){std::vector<std::pair<double,size_t>> nearby;
        for(size_t id=0;id<records[opposite].size()+(latestDistinct(opposite)?1:0);++id){auto p=observation(opposite,id);
          Eigen::Vector3d qp=query.input->pose*query.input->recent_center_local,rp=p->pose*p->recent_center_local;
          if(role==1)qp=maps.alignment().transform*qp;else rp=maps.alignment().transform*rp;
          nearby.push_back({(qp-rp).norm(),id});}
        std::stable_sort(nearby.begin(),nearby.end());for(size_t j=0;j<std::min(size_t(config.max_candidates),nearby.size());++j)shortlisted.insert(nearby[j].second);}
      for(size_t id:shortlisted){auto& other=get(opposite,id);auto& ar=role==1?other:query;auto& aq=role==1?query:other;
        if(!ar.contour||!aq.contour||aq.contour->height_above_ground<config.contour_min_height)continue;
        Eigen::Matrix4d gr=Eigen::Matrix4d::Identity(),gq=gr;gr.topLeftCorner<3,3>()=g(ar.input).toRotationMatrix();gq.topLeftCorner<3,3>()=g(aq.input).toRotationMatrix();
        if(maps.alignment().valid){Eigen::Isometry3d prior=aq.input->pose.inverse()*maps.alignment().transform.inverse()*ar.input->pose;
          Eigen::Isometry3d aligned;aligned.matrix()=gq*prior.matrix()*gr.inverse();size_t support=contourSeedSupport(*ar.contour,*aq.contour,aligned);
          if(support>=size_t(config.min_inliers)){CoarseCandidate c;c.reference_id=id;c.inliers=support;c.raw_correspondences=support;all.push_back({c,id,7,0,prior.matrix()});}}
        for(const auto& h:contourLineSeeds(*ar.contour,*aq.contour)){
          CoarseCandidate c;c.reference_id=id;c.inliers=h.inliers;c.raw_correspondences=h.inliers;c.planar_rmse=0;all.push_back({c,id,6,0,gq.inverse()*h.aligned.matrix()*gr});
        }
      }
    }
    std::stable_sort(all.begin(),all.end(),[](const auto& a,const auto& b){if((a.profile==7)!=(b.profile==7))return a.profile==7;
      if(a.c.inliers!=b.c.inliers)return a.c.inliers>b.c.inliers;
      if(a.c.planar_rmse!=b.c.planar_rmse)return a.c.planar_rmse<b.c.planar_rmse;return std::make_tuple(a.id,a.profile,a.angle)<std::make_tuple(b.id,b.profile,b.angle);});
    std::map<size_t,int> retained;std::vector<Seed> chosen;
    for(int pass=0;pass<(contour?2:1);++pass)for(const auto& s:all){auto found=retained.find(s.id);
      if(chosen.size()>=size_t(contour?40:30))break;
      if(found==retained.end()&&retained.size()>=size_t(config.max_candidates))continue;
      if(found!=retained.end()&&found->second>=(contour?4:3))continue;
      if(contour&&pass==0){bool same_direction=false;double yaw=std::atan2(s.pose(1,0),s.pose(0,0));
        for(const auto& old:chosen)if(old.id==s.id){double delta=std::atan2(std::sin(yaw-std::atan2(old.pose(1,0),old.pose(0,0))),std::cos(yaw-std::atan2(old.pose(1,0),old.pose(0,0))));
          if(std::abs(delta)<pi/6){same_direction=true;break;}}if(same_direction)continue;}
      bool duplicate=false;for(const auto& prior:chosen)if(prior.id==s.id){Eigen::Matrix4d delta=prior.pose.inverse()*s.pose;
        if(delta.topRightCorner<3,1>().head<2>().norm()<.25&&Eigen::AngleAxisd(delta.topLeftCorner<3,3>()).angle()<3*pi/180){duplicate=true;break;}}
      if(duplicate)continue;chosen.push_back(s);++retained[s.id];if(chosen.size()>=size_t(contour?40:30))break;
    }
    std::map<std::string,double> reflection_scores;
    for(const auto& s:chosen){auto& reference=get(opposite,s.id);auto& ar=role==1?reference:query;auto& aq=role==1?query:reference;
      CandidateAttempt attempt;attempt.reference_identity=ar.input->identity();attempt.reference_id=ar.input->id;attempt.inliers=s.c.inliers;
      attempt.height_valid=s.c.height_valid;attempt.height_pairs=s.c.height_pairs;attempt.empty_height_pairs=s.c.empty_height_pairs;
      attempt.bev_resolution=s.profile>=6?config.contour_grid:profiles[s.profile].first;attempt.height_neighborhood=s.profile>=6?0:profiles[s.profile].second;attempt.query_view_deg=s.angle*180/pi;
      attempt.generator=s.profile==7?"validated_map_prior":(s.profile==6?"ehpr_retrieved_line_ransac":"ehpr_orb");
      attempt.raw_correspondences=s.c.raw_correspondences;attempt.planar_rmse=s.c.planar_rmse;attempt.coarse=s.pose;attempt.coarse_determinant=s.pose.topLeftCorner<3,3>().determinant();
      attempt.reference_recent=bool(ar.input->recent_cloud);attempt.query_recent=bool(aq.input->recent_cloud);attempt.matching_radius_ugv=std::max(config.matching_radius_ugv,config.matching_radius_ugv_fallback);
      MatchResult m;m.reference=ar.input;m.query=aq.input;m.coarse.matrix()=s.pose;m.inliers=s.c.inliers;
      m.reference_recent=attempt.reference_recent;m.query_recent=attempt.query_recent;m.matching_radius_ugv=attempt.matching_radius_ugv;
      m.matching_reference_points=ar.local->size();m.matching_query_points=aq.local->size();
      if(contour){if(!ar.contour||!aq.contour)attempt.reason="missing recent contour observation";
        else attempt.reason=refineContour(*ar.contour,*aq.contour,grounds[0],grounds[1],config,m);
      }else attempt.reason=refinePartial(*ar.partial,*aq.partial,config,m);
      if(contour&&attempt.reason.empty()){
        const std::string pair=ar.input->identity()+"|"+aq.input->identity();
        auto inserted=reflection_scores.emplace(pair,0);
        if(inserted.second){auto reflected=reflectedContour(*ar.contour);auto seeds=contourLineSeeds(*reflected,*aq.contour);
          Eigen::Matrix4d gr=Eigen::Matrix4d::Identity(),gq=gr;gr.topLeftCorner<3,3>()=g(ar.input).toRotationMatrix();gq.topLeftCorner<3,3>()=g(aq.input).toRotationMatrix();
          for(size_t k=0;k<std::min(size_t(4),seeds.size());++k){MatchResult mirror;mirror.reference=ar.input;mirror.query=aq.input;
            mirror.coarse.matrix()=gq.inverse()*seeds[k].aligned.matrix()*gr;
            if(refineContour(*reflected,*aq.contour,grounds[0],grounds[1],config,mirror).empty())inserted.first->second=std::max(inserted.first->second,mirror.candidate_score);}}
        m.reflection_checked=true;m.reflection_score=inserted.first->second;
        if(m.reflection_score>m.candidate_score+config.partial_score_margin)attempt.reason="reflection explains contour better than rigid yaw alignment";
      }
      if(attempt.reason.empty()&&contour)attempt.reason=validateVisibility(m,config);
      if(contour&&maps.alignment().valid){m.alignment_prior_used=true;m.alignment_prior_revision=maps.alignment().revision;m.alignment_prior=maps.alignment().transform;}
      if(contour) {
        pcl::KdTreeFLANN<pcl::PointXYZ> rt,qt;rt.setInputCloud(ar.local);qt.setInputCloud(aq.local);
        std::vector<int> ids(1);std::vector<float> ds(1);double fitness=0,residual=0;size_t count=0,overlap=0;
        for(const auto& p:*ar.local){Eigen::Vector3d x=m.refined*p.getVector3fMap().cast<double>();
          if(qt.nearestKSearch(pcl::PointXYZ(x.x(),x.y(),x.z()),1,ids,ds)){if(ds[0]<=20){fitness+=ds[0];++count;}if(ds[0]<=config.overlap_radius*config.overlap_radius){residual+=ds[0];++overlap;}}}
        m.fitness=count?fitness/count:1e9;m.overlap_reference=double(overlap)/std::max(size_t(1),ar.local->size());
        m.residual_rmse=overlap?std::sqrt(residual/overlap):std::sqrt(m.fitness);overlap=0;
        for(const auto& p:*aq.local){Eigen::Vector3d x=m.refined.inverse()*p.getVector3fMap().cast<double>();
          if(rt.nearestKSearch(pcl::PointXYZ(x.x(),x.y(),x.z()),1,ids,ds)&&ds[0]<=config.overlap_radius*config.overlap_radius)++overlap;}
        m.overlap_query=double(overlap)/std::max(size_t(1),aq.local->size());m.overlap=ar.local->size()<=aq.local->size()?m.overlap_reference:m.overlap_query;
      }
      attempt.result=m;event.candidate_attempts.push_back(std::move(attempt));
    }
    event.diagnostics["candidate_count"]=chosen.size();event.diagnostics["history_retrieval"]=1;
    event.diagnostics["contour_search"]=contour;event.diagnostics["coarse_hypotheses_before_limits"]=all.size();selectPartialMatch(event,config);
    if(event.match&&contour)event.reason="EHPR contour and historical ground gates passed";
    if(chosen.empty())event.reason="no historical EHPR candidate";
    return event;
  }
  std::string revalidate(MatchResult& m,const Eigen::Isometry3d& map) {
    auto a=prepare(m.reference,m.matching_radius_ugv),b=prepare(m.query);m.reference=a.input;m.query=b.input;
    m.refined=m.query->pose.inverse()*map.inverse()*m.reference->pose;
    auto reason=m.registration_branch=="contour_xy_ground"?
      ((!a.contour||!b.contour)?std::string("missing contour evidence"):validateContourTransform(*a.contour,*b.contour,config,m)):
      validatePartialTransform(*a.partial,*b.partial,config,m);
    if(reason.empty())reason=validateVisibility(m,config);
    if(reason.empty()&&!m.visibility.sufficient)reason="insufficient visibility evidence";
    return reason;
  }
  void complete(MatchEvent& event){
    tracker.consider(event,[this](MatchResult& m,const Eigen::Isometry3d& t){return revalidate(m,t);});
    if(event.clear_display)accepted_pairs.clear();
    if(event.match){auto key=event.match->reference->identity()+"|"+event.match->query->identity();
      if(accepted_pairs.count(key)){event.match.reset();event.status="alignment_unchanged";event.reason="verified pair already published";}
      else{accepted_pairs.insert(key);++counts["accepted_matches"];}}
    event.diagnostics["history_ugv_keyframes"]=records[0].size();event.diagnostics["history_uav_keyframes"]=records[1].size();
    event.diagnostics["history_ugv_geometry_cache"]=geometry[0].size();event.diagnostics["history_uav_geometry_cache"]=geometry[1].size();
    event.diagnostics["history_pending_jobs"]=jobs.size();event.diagnostics["map_ugv_voxels"]=maps.size(0);event.diagnostics["map_uav_voxels"]=maps.size(1);
    event.diagnostics["alignment_valid"]=maps.alignment().valid;event.diagnostics["alignment_revision"]=maps.alignment().revision;
    event.diagnostics["ground_reference_valid"]=grounds[0].valid&&!grounds[0].conflict;event.diagnostics["ground_query_valid"]=grounds[1].valid&&!grounds[1].conflict;
    event.diagnostics["ground_revision"]=ground_revision;
    for(int robot=0;robot<2;++robot){const auto& anchor=grounds[robot];const std::string prefix=robot?"ground_query_":"ground_reference_";
      event.diagnostics[prefix+"height_map_m"]=anchor.height;event.diagnostics[prefix+"rmse_m"]=anchor.rmse;
      event.diagnostics[prefix+"independent_voxels"]=anchor.support;event.diagnostics[prefix+"span_m"]=anchor.span;
      event.diagnostics[prefix+"second_direction_width_m"]=anchor.width;
      if(!anchor.source.empty())event.diagnostics[prefix+"source_submap_id"]=std::stoul(anchor.source.substr(anchor.source.rfind(':')+1));}
    if(event.match&&event.match->registration_branch=="contour_xy_ground"){const auto& m=*event.match;
      event.diagnostics["contour_correspondences"]=m.contour_correspondences;event.diagnostics["contour_overlap_reference"]=m.contour_overlap_reference;
      event.diagnostics["contour_overlap_query"]=m.contour_overlap_query;event.diagnostics["contour_xy_rmse_m"]=m.contour_residual_rmse;
      event.diagnostics["query_height_above_ground_m"]=m.query_height_above_ground;event.diagnostics["candidate_score"]=m.candidate_score;}
    if(event.match&&event.match->registration_branch=="contour_xy_ground"){
      event.diagnostics["information_ratio"]=event.match->contour_information_ratio;
      event.diagnostics["direction_support_first"]=event.match->direction_support_first;
      event.diagnostics["direction_support_second"]=event.match->direction_support_second;}
  }
};

AdvancedMatcher::AdvancedMatcher(const MatcherConfig& c):impl_(std::make_unique<Impl>(c)){}
AdvancedMatcher::~AdvancedMatcher()=default;
void AdvancedMatcher::setDirectory(const std::string& p){if(!impl_->records[0].empty()||!impl_->records[1].empty())throw std::runtime_error("Archive directory must be set before input");
  if(impl_->temporary){std::error_code ec;fs::remove_all(impl_->directory,ec);}impl_->directory=fs::absolute(p);impl_->temporary=false;fs::create_directories(impl_->directory);}
void AdvancedMatcher::invalidate(const std::string& reason){impl_->reset(reason);}
void AdvancedMatcher::invalidateRobot(int robot,const std::string& reason){impl_->resetRobot(robot,reason);}
const SessionMap& AdvancedMatcher::map() const{return impl_->maps;}
size_t AdvancedMatcher::databaseSize() const{return impl_->records[0].size();}
const std::map<std::string,uint64_t>& AdvancedMatcher::counters() const{return impl_->counts;}
std::optional<MatchEvent> AdvancedMatcher::observeScans(SubmapPtr r,SubmapPtr q,double now){
  auto& s=*impl_;if(!r||!q||!s.latest[0]||!s.latest[1]||r->frame!=s.config.ugv_frame||q->frame!=s.config.uav_frame)return {};
  const double stamp=std::min(r->recent_end_stamp,q->recent_end_stamp);
  if(!s.maps.alignment().valid||std::abs(r->recent_end_stamp-q->recent_end_stamp)>.1+1e-6||
      now-r->recent_end_stamp>1||now-q->recent_end_stamp>1||stamp<s.last_monitor_attempt+.5-1e-6)return {};
  s.last_monitor_attempt=stamp;
  std::array<SubmapPtr,2> scans={r,q};
  for(int robot=0;robot<2;++robot){if(!s.grounds[robot].valid||s.grounds[robot].conflict)return {};
    auto p=std::make_shared<SubmapInput>(*scans[robot]);p->matching_gravity=canonicalMatchingGravity(p->pose.rotation());
    p->session=s.latest[robot]->session;p->epoch=s.latest[robot]->epoch;
    p->visibility_scan=prepareVisibilityScan(*p,s.grounds[robot].height-p->pose.translation().z(),s.config);scans[robot]=p;}
  auto event=s.tracker.observe(scans[0],scans[1],now);if(event){
    if(event->clear_display){s.accepted_pairs.clear();++s.counts["alignment_revocations"];s.queried.clear();
      for(int robot=0;robot<2;++robot)for(size_t id=0;id<s.records[robot].size();++id)s.enqueue(robot,id);}
    event->diagnostics["alignment_valid"]=s.maps.alignment().valid;event->diagnostics["alignment_revision"]=s.maps.alignment().revision;}
  return event;
}
MatchEvent AdvancedMatcher::process(SubmapPtr input,bool search){
  auto& s=*impl_;MatchEvent invalid;invalid.status="rejected";invalid.reason="invalid identity/frame/pose/cloud";
  if(!input||input->robot<0||input->robot>1||input->session.empty()||!input->cloud||input->cloud->size()<size_t(s.config.min_points)||
     input->local_frame.empty()||(!s.config.allow_partial&&input->partial)||
     !properTransform(input->pose.matrix())||!input->gravity.coeffs().allFinite()||std::abs(input->gravity.norm()-1)>1e-3||
     input->frame!=(input->robot?s.config.uav_frame:s.config.ugv_frame)||!std::isfinite(input->stamp)||!input->observation_quality_valid)return invalid;
  if(input->recent_cloud&&(!input->recent_frames||!std::isfinite(input->recent_start_stamp)||!std::isfinite(input->recent_end_stamp)||
     input->recent_end_stamp<input->recent_start_stamp||!input->recent_center_local.allFinite())){invalid.reason="invalid recent cloud metadata";return invalid;}
  for(const auto& p:*input->cloud)if(!p.getVector3fMap().allFinite()){invalid.reason="nonfinite cloud";return invalid;}
  if(input->recent_cloud)for(const auto& p:*input->recent_cloud)if(!p.getVector3fMap().allFinite()){invalid.reason="nonfinite recent cloud";return invalid;}
  if(input->latest_scan)for(const auto& p:*input->latest_scan)if(!p.getVector3fMap().allFinite()){invalid.reason="nonfinite latest scan";return invalid;}
  std::string generation=input->session+":"+std::to_string(input->epoch);bool cleared=false;
  if(s.generations[input->robot]==generation&&s.seen.count(input->identity())){
    invalid.status="duplicate";invalid.reason="identity already processed";return invalid;}
  // Finalized keyframes can arrive after a newer preview. Only an observation
  // window's end is a live clock; a keyframe anchor timestamp is not.
  bool rollback=s.latest[input->robot]&&input->recent_cloud&&s.latest[input->robot]->recent_frames&&
    input->recent_end_stamp<s.latest[input->robot]->recent_end_stamp-.05;
  if((!s.generations[input->robot].empty()&&s.generations[input->robot]!=generation)||rollback){
    if(rollback)s.reset("time_rollback");else s.resetRobot(input->robot,"session_or_epoch_reset");cleared=true;++s.counts["generation_resets"];}
  s.generations[input->robot]=generation;
  if(!s.seen.insert(input->identity()).second){invalid.status="duplicate";invalid.reason="identity already processed";return invalid;}
  ++s.counts["received"];auto p=std::make_shared<SubmapInput>(*input);p->matching_gravity=canonicalMatchingGravity(p->pose.rotation());input=p;
  bool had_ground=s.grounds[input->robot].valid&&!s.grounds[input->robot].conflict;auto gr=s.ground_revision;
  s.latest[input->robot]=light(input);s.maps.add(input);s.ground(input);
  if(gr!=s.ground_revision){s.persistGround();for(int r=0;r<2;++r)for(size_t id=0;id<s.records[r].size();++id)s.enqueue(r,id);}
  if(s.config.matching_profile=="partial_contour"&&!had_ground&&s.grounds[input->robot].valid&&!s.grounds[input->robot].conflict&&!s.records[input->robot].empty())s.refreshContourIndex(input->robot);
  auto geometry=s.prepare(input);input=geometry.input;s.archive(input,geometry);s.latest_geometry[input->robot]=geometry;
  MatchEvent event;
  if(search){event=s.low->process(input);
    if(!event.match&&(input->robot==0||s.low->databaseSize()==0)){auto reverse=s.search(geometry,false);if(reverse.match)event=std::move(reverse);}
    double minimum_height=input->pose.translation().z()+(g(input)*input->recent_center_local).z()-
      (s.grounds[input->robot].valid?s.grounds[input->robot].height:-.1);
    if(s.config.matching_profile=="partial_contour"&&!event.match&&input->recent_cloud&&((input->robot==1&&minimum_height>=s.config.contour_min_height)||input->robot==0)){
      auto high=s.search(geometry,true);if(high.match||input->robot==1){high.diagnostics["partial_candidate_count"]=event.candidate_attempts.size();event=std::move(high);}}
  }else{event.status="history_primed";event.reason="observation archived without publishing historical matches";}
  event.clear_display=event.clear_display||cleared;s.complete(event);return event;
}
void AdvancedMatcher::prime(const std::vector<SubmapPtr>& inputs){for(auto p:inputs)process(p,false);retryHistory();}
size_t AdvancedMatcher::retryHistory(){auto& s=*impl_;s.queried.clear();for(int r=0;r<2;++r)for(size_t i=0;i<s.records[r].size();++i)s.enqueue(r,i);return s.jobs.size();}
std::optional<std::pair<MatchEvent,SubmapPtr>> AdvancedMatcher::pollHistory(){auto& s=*impl_;
  while(!s.jobs.empty()){auto job=s.jobs.front();s.jobs.pop_front();s.queued.erase(job);if(s.queried[job]==s.revision)continue;s.queried[job]=s.revision;
    auto& geometry=s.get(job.first,job.second);auto input=geometry.input;auto event=s.search(geometry,false);
    if(!event.match&&s.config.matching_profile=="partial_contour"){auto high=s.search(geometry,true);if(high.match||high.candidate_attempts.size()>event.candidate_attempts.size())event=std::move(high);}
    ++s.counts["history_queries"];s.complete(event);return std::make_pair(std::move(event),input);}
  return std::nullopt;
}
}
