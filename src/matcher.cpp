#include <ehpr_bridge/matcher.h>
#include <ehpr_bridge/partial_registration.h>
#include <ehpr_bridge/advanced_matcher.h>
#include <ehpr_bridge/visibility.h>
#include <ehpr/desc_db.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <limits>
#include <iterator>
#include <stdexcept>
#include <tuple>
SequentialTimeProfiler stp("stage3_ehpr");
namespace ehpr_bridge {
static Eigen::Quaterniond matchingGravity(SubmapPtr input) {
  return input->matching_gravity.value_or(input->gravity);
}
std::string SubmapInput::identity() const {
  return std::to_string(robot)+":"+session+":"+std::to_string(epoch)+":"+std::to_string(id);
}
bool properTransform(const Eigen::Matrix4d& m) {
  return m.allFinite() && (m.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-6 &&
    (m.topLeftCorner<3,3>().transpose()*m.topLeftCorner<3,3>()-Eigen::Matrix3d::Identity()).norm()<1e-3 &&
    std::abs(m.topLeftCorner<3,3>().determinant()-1)<1e-3;
}
Matcher::Matcher(const MatcherConfig& c,bool candidate_only) : config_(c),candidate_only_(candidate_only) {
  cv::setNumThreads(1);
  if(!std::isfinite(c.recent_window_seconds)||c.recent_window_seconds<=0 ||
     !std::isfinite(c.recent_voxel_size)||c.recent_voxel_size<=0 ||
     c.recent_min_frames<1||c.recent_max_frames<c.recent_min_frames)
    throw std::invalid_argument("Invalid recent matching cloud configuration");
  if(c.matching_profile!="legacy"&&c.matching_profile!="enhanced"&&c.matching_profile!="partial"&&c.matching_profile!="partial_contour")throw std::invalid_argument("Invalid matching profile");
  if(c.matching_profile=="partial"||c.matching_profile=="partial_contour") {
    for(double ratio:{c.partial_high_coverage,c.partial_low_coverage,c.partial_information_ratio,c.partial_score_margin,c.partial_history_low_coverage})
      if(!std::isfinite(ratio)||ratio<=0||ratio>1)throw std::invalid_argument("Invalid partial ratio");
    for(double v:{c.partial_rmse,c.partial_span,c.partial_direction_angle_deg,c.partial_cluster_translation,c.partial_cluster_rotation_deg,c.partial_planar_inlier_distance})
      if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid partial distance/angle");
    if(c.partial_high_coverage<c.partial_low_coverage||c.partial_direction_angle_deg>90||
       c.partial_iterations<1||c.partial_min_pairs<3||c.partial_direction_min_pairs<3)
      throw std::invalid_argument("Invalid partial geometry configuration");
  }
  if (c.image_size<64 || c.image_size%2 || c.local_density<0 || c.buckets<1 || c.buckets>256 ||
      c.database_capacity<1 || c.hamming_distance<0 || c.hamming_distance>256 || c.min_inliers<5 ||
      c.min_points<20 || c.min_structural_points<3 || c.max_iterations<1 || c.max_candidates<1 || c.max_candidates>200 || !std::isfinite(c.min_overlap) || c.min_overlap<0 || c.min_overlap>1 ||
      !std::isfinite(c.min_structural_overlap) || c.min_structural_overlap<0 || c.min_structural_overlap>1 ||
      !std::isfinite(c.distribution_mean) || c.ugv_frame.empty() || c.uav_frame.empty())
    throw std::invalid_argument("Invalid Stage 3 integer/ratio configuration");
  for (double v : {c.bev_resolution,c.height_resolution,c.max_density,c.distribution_std,c.voxel_size,
                   c.correspondence_distance,c.max_fitness,c.overlap_radius,c.max_residual_rmse,c.fusion_voxel_size,c.structural_height_above_floor})
    if (!std::isfinite(v) || v<=0) throw std::invalid_argument("Stage 3 parameters must be finite and positive");
  for (double radius : {c.matching_radius_ugv, c.matching_radius_uav})
    if (!std::isfinite(radius) || radius < 0) throw std::invalid_argument("Matching radii must be finite and nonnegative");
  if (!std::isfinite(c.matching_radius_ugv_fallback) || c.matching_radius_ugv_fallback < 0 ||
      (c.matching_radius_ugv_fallback > 0 &&
       (c.matching_radius_ugv <= 0 || c.matching_radius_ugv_fallback <= c.matching_radius_ugv)))
    throw std::invalid_argument("Fallback radius must exceed a positive primary radius, or be zero");
  if(c.matching_profile=="partial_contour"||c.matching_profile=="partial") {
    for(double v:{c.contour_min_height,c.contour_grid,c.contour_distance,c.contour_rmse,c.history_translation,c.history_rotation_deg})
      if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid contour/history configuration");
    for(double v:{c.contour_high_coverage,c.contour_low_coverage,c.history_novelty})
      if(!std::isfinite(v)||v<=0||v>1)throw std::invalid_argument("Invalid contour/history ratio");
    if(c.contour_high_coverage<c.contour_low_coverage||c.contour_min_pairs<3||c.history_geometry_capacity<1)
      throw std::invalid_argument("Invalid contour/history limits");
    if(c.alignment_initial_observations<2||c.alignment_initial_observations>16||c.alignment_recovery_observations<3||c.alignment_recovery_observations>16||
       c.visibility_min_rays<1||c.visibility_min_structure_rays<1)throw std::invalid_argument("Invalid alignment evidence counts");
    for(double v:{c.alignment_baseline_m,c.alignment_structure_novelty,c.visibility_angle_deg,c.visibility_depth_m,c.visibility_max_conflict})
      if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid alignment evidence configuration");
    if(c.alignment_structure_novelty>1||c.visibility_max_conflict>1||c.visibility_angle_deg>5)throw std::invalid_argument("Invalid alignment evidence ratio/angle");
    if(!candidate_only_){advanced_=std::make_unique<AdvancedMatcher>(c);return;}
  }
  rebuild();
  if (c.matching_radius_ugv_fallback > 0) {
    auto fallback = c;
    fallback.matching_radius_ugv = c.matching_radius_ugv_fallback;
    fallback.matching_radius_ugv_fallback = 0;
    radius_fallback_ = std::make_unique<Matcher>(fallback,candidate_only_);
  }
}
Matcher::~Matcher() = default;
const std::map<std::string,uint64_t>& Matcher::counters() const {return advanced_?advanced_->counters():(radius_fallback_?combined_counters_:counters_);}
void Matcher::setArchiveDirectory(const std::string& path){if(advanced_)advanced_->setDirectory(path);}
void Matcher::prime(const std::vector<SubmapPtr>& inputs){if(advanced_)advanced_->prime(inputs);else for(auto p:inputs)process(p);}
MatchEvent Matcher::archiveInput(SubmapPtr input){return advanced_?advanced_->process(input,false):MatchEvent();}
void Matcher::invalidate(const std::string& reason){if(advanced_)advanced_->invalidate(reason);else invalidate();}
void Matcher::invalidateRobot(int robot,const std::string& reason){if(advanced_)advanced_->invalidateRobot(robot,reason);else invalidate();}
size_t Matcher::retryHistory(){return advanced_?advanced_->retryHistory():0;}
std::optional<std::pair<MatchEvent,SubmapPtr>> Matcher::pollHistory(){return advanced_?advanced_->pollHistory():std::nullopt;}
const MapAlignmentState* Matcher::alignment() const {return advanced_?&advanced_->map().alignment():nullptr;}
FusedSubmap Matcher::fusedMap() const {return advanced_?advanced_->map().fused():FusedSubmap();}
uint64_t Matcher::mapRevision() const {return advanced_?advanced_->map().revision():0;}
Cloud Matcher::mapCloud(int robot) const {return advanced_?advanced_->map().cloud(robot):Cloud();}
std::optional<MatchEvent> Matcher::observeScans(SubmapPtr r,SubmapPtr q,double now){return advanced_?advanced_->observeScans(r,q,now):std::nullopt;}
size_t Matcher::databaseSize() const {
  if(advanced_)return advanced_->databaseSize();
  return radius_fallback_ ? std::max(entries_.size(), radius_fallback_->databaseSize()) : entries_.size();
}
void Matcher::refreshCounters() {
  if (!radius_fallback_) return;
  combined_counters_ = counters_;
  // Count inputs once; both views contribute their actual geometric work.
  for (const auto* key : {"queries", "query_rejections", "candidate_attempts", "candidate_rejections",
                         "accepted_matches", "candidate_fallback_matches"}) {
    auto found = radius_fallback_->counters_.find(key);
    if (found != radius_fallback_->counters_.end()) combined_counters_[key] += found->second;
  }
  combined_counters_["radius_fallback_database_size"] = radius_fallback_->databaseSize();
}
static const std::vector<std::pair<double,int>> enhanced_profiles={{.5,3},{.5,1},{.25,3},{.25,1}};
Matcher::Variant Matcher::makeDescriptor(SubmapPtr input,const Cloud::Ptr& local,int id,
    double resolution,int neighborhood,double angle,cv::Mat* image) {
  DescriptorManagerConfig c;c.reso_row_=c.reso_col_=resolution;c.n_row_=c.n_col_=config_.image_size;
  c.height_res_=config_.height_resolution;c.max_dens_=config_.max_density;c.local_density_rows_=c.local_density_cols_=neighborhood;
  auto descriptor=std::make_shared<DescriptorManager>(c,id);Cloud::Ptr aligned(new Cloud);aligned->reserve(local->size());
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix()*matchingGravity(input).toRotationMatrix();
  for(const auto& point:*local){const Eigen::Vector3d g=rotation*point.getVector3fMap().cast<double>();aligned->push_back(pcl::PointXYZ(g.x(),g.y(),g.z()));}
  Cloud::ConstPtr view=aligned;descriptor->makeDensityBEV<pcl::PointXYZ>(view,input->identity());
  if(image)*image=descriptor->getBevImage();descriptor->makeFeature();size_t valid=0;
  for(int i=0;i<descriptor->getLevKeyPointsSize();++i)if(descriptor->getKeyHeightBin(i).any())++valid;
  const size_t features=descriptor->getLevKeyPointsSize();descriptor->releaseConstructionBuffers();
  return {descriptor,features,valid};
}
Matcher::Entry Matcher::describe(SubmapPtr input, int id, cv::Mat* image) {
  if(config_.matching_profile=="partial") {
    auto normalized=std::make_shared<SubmapInput>(*input);
    normalized->matching_gravity=canonicalMatchingGravity(input->pose.rotation());input=normalized;
  }
  const auto gravity=matchingGravity(input);
  const bool recent=config_.use_recent_matching_cloud && bool(input->recent_cloud);
  const auto& points=recent?input->recent_cloud:input->cloud;
  Eigen::Vector3d center=Eigen::Vector3d::Zero();if(recent)center=gravity*input->recent_center_local;
  Cloud::Ptr local(new Cloud);local->reserve(points->size());
  const double radius=input->robot==0?config_.matching_radius_ugv:config_.matching_radius_uav;
  double lo=std::numeric_limits<double>::infinity(),hi=-lo;
  for(const auto& point:*points){const Eigen::Vector3d g=gravity*point.getVector3fMap().cast<double>();
    if(radius>0&&(g-center).head<2>().squaredNorm()>radius*radius)continue;
    local->push_back(point);lo=std::min(lo,g.z());hi=std::max(hi,g.z());}
  if(local->size()<size_t(config_.min_points))throw std::invalid_argument("insufficient local matching points");
  if(!(hi-lo>1e-5))throw std::invalid_argument("no height range");
  if((hi-lo)/config_.height_resolution>4096)throw std::invalid_argument("height bin range too large");
  Entry entry;entry.input=input;entry.matching_cloud=local;entry.recent=recent;
  if(config_.matching_profile!="legacy") {
    for(const auto& profile:enhanced_profiles)entry.variants.push_back(makeDescriptor(input,local,id,profile.first,profile.second,0,
      entry.variants.empty()?image:nullptr));
  }else entry.variants.push_back(makeDescriptor(input,local,id,config_.bev_resolution,config_.local_density,0,image));
  const auto& first=entry.variants.front();entry.descriptor=first.descriptor;entry.height_features=first.features;entry.valid_height_features=first.valid_height;
  const auto maximum=std::max_element(entry.variants.begin(),entry.variants.end(),[](const auto& a,const auto& b){return a.features<b.features;});
  if(maximum->features<size_t(config_.min_inliers)&&config_.matching_profile=="legacy")throw std::invalid_argument("insufficient ORB features");
  if(config_.matching_profile=="partial"){
    entry.geometry=preparePartialGeometry(local,gravity,center);
    if(!entry.input->visibility_scan&&!entry.geometry->horizontal_planes.empty()){
      auto normalized=std::make_shared<SubmapInput>(*entry.input);
      const auto floor=std::min_element(entry.geometry->horizontal_planes.begin(),entry.geometry->horizontal_planes.end(),
        [](const auto& a,const auto& b){return a.height<b.height;});
      normalized->visibility_scan=prepareVisibilityScan(*normalized,floor->height,config_);entry.input=normalized;
    }
  }
  return entry;
}
void Matcher::rebuild() {
  extra_databases_.clear();
  const size_t count=config_.matching_profile!="legacy"?enhanced_profiles.size():1;
  for(size_t k=0;k<count;++k){DescriptorDBConfig c;
    c.reso_row_=c.reso_col_=config_.matching_profile!="legacy"?enhanced_profiles[k].first:config_.bev_resolution;
    c.n_row_=c.n_col_=config_.image_size;c.bucket_size_=config_.buckets;c.dist_thres_=config_.hamming_distance;
    c.distri_mean_=config_.distribution_mean;c.distri_var_=config_.distribution_std;c.skip_num_=-1;c.enhanced_=config_.matching_profile!="legacy";
    if(config_.matching_profile=="partial")c.planar_inlier_distance_=config_.partial_planar_inlier_distance;
    if(k==0)database_=std::make_unique<DescriptorDB>(c);else extra_databases_.push_back(std::make_unique<DescriptorDB>(c));}
  int id=0;
  for(auto& entry:entries_){entry=describe(entry.input,id++);
    for(size_t k=0;k<count;++k){auto& db=k==0?database_:extra_databases_[k-1];db->getClassMatchables(entry.variants[k].descriptor);db->addClassToHBST(entry.variants[k].descriptor);}}
}
void Matcher::invalidate() {
  if(advanced_){advanced_->invalidate();return;}
  pending_.reset();entries_.clear();for(auto& s:seen_)s.clear();for(auto& g:generation_)g.clear();rebuild();
  ++counters_["lio_quality_invalidations"];
  if (radius_fallback_) {radius_fallback_->invalidate();refreshCounters();}
}
MatchEvent Matcher::process(SubmapPtr input) {
  if(advanced_)return advanced_->process(input);
  if(config_.matching_profile=="partial") {
    auto result=processSingle(input,false);
    MatchEvent secondary;
    if(radius_fallback_)secondary=radius_fallback_->processSingle(input,false);
    result.clear_display=result.clear_display||secondary.clear_display;
    if(result.status=="rejected"||result.status=="duplicate"||!pending_){refreshCounters();return result;}
    auto event=query(30,true);
    if(radius_fallback_&&radius_fallback_->pending_) {
      auto extra=radius_fallback_->query(30,true);
      event.candidate_attempts.insert(event.candidate_attempts.end(),
          std::make_move_iterator(extra.candidate_attempts.begin()),std::make_move_iterator(extra.candidate_attempts.end()));
      event.diagnostics["fallback_coarse_hypotheses_before_limits"]=extra.diagnostics["coarse_hypotheses_before_limits"];
    }
    event.clear_display=result.clear_display;event.bev=result.bev;
    event.diagnostics.insert(result.diagnostics.begin(),result.diagnostics.end());
    // Bound the joint pool, not each radius independently. Repeated radius views
    // of one coarse pose are evidence from the same observation, not new votes.
    auto seeds=std::move(event.candidate_attempts);
    std::stable_sort(seeds.begin(),seeds.end(),[](const auto& a,const auto& b){
      if(a.inliers!=b.inliers)return a.inliers>b.inliers;
      if(a.planar_rmse!=b.planar_rmse)return a.planar_rmse<b.planar_rmse;
      if(a.result->matching_reference_points!=b.result->matching_reference_points)
        return a.result->matching_reference_points>b.result->matching_reference_points;
      return std::make_tuple(a.reference_id,a.matching_radius_ugv,a.query_view_deg)<
             std::make_tuple(b.reference_id,b.matching_radius_ugv,b.query_view_deg);});
    std::map<std::string,int> retained;
    event.candidate_attempts.clear();
    for(auto& seed:seeds) {
      if(retained.count(seed.reference_identity)==0&&retained.size()>=size_t(config_.max_candidates))continue;
      if(retained[seed.reference_identity]>=3)continue;
      bool duplicate=false;
      for(const auto& previous:event.candidate_attempts)if(previous.reference_identity==seed.reference_identity) {
        const Eigen::Matrix4d delta=previous.coarse.inverse()*seed.coarse;
        if(delta.topRightCorner<3,1>().norm()<1&&Eigen::AngleAxisd(delta.topLeftCorner<3,3>()).angle()<M_PI/18){duplicate=true;break;}
      }
      if(duplicate)continue;
      auto& owner=(radius_fallback_&&seed.matching_radius_ugv==config_.matching_radius_ugv_fallback)?*radius_fallback_:*this;
      auto reference=std::find_if(owner.entries_.begin(),owner.entries_.end(),[&](const auto& entry){return entry.input->identity()==seed.reference_identity;});
      if(reference==owner.entries_.end())throw std::runtime_error("Partial candidate identity invariant broken");
      if(!seed.height_valid)seed.reason="height alignment invalid";
      else if(seed.inliers<size_t(config_.min_inliers))seed.reason="insufficient RANSAC inliers";
      else seed.reason=refinePartial(*reference->geometry,*owner.pending_->geometry,config_,*seed.result);
      ++counters_["candidate_attempts"];if(!seed.reason.empty())++counters_["candidate_rejections"];
      ++retained[seed.reference_identity];event.candidate_attempts.push_back(std::move(seed));
      if(event.candidate_attempts.size()>=30)break;
    }
    event.diagnostics["candidate_count"]=event.candidate_attempts.size();
    selectPartialMatch(event,config_);
    if(event.match) {
      ++counters_["accepted_matches"];pending_.reset();if(radius_fallback_)radius_fallback_->pending_.reset();
      const bool fallback=event.match->matching_radius_ugv==config_.matching_radius_ugv_fallback;
      event.diagnostics["radius_fallback_selected"]=fallback;
      if(fallback)++counters_["radius_fallback_matches"];
    }else if(!entries_.empty())++counters_["query_rejections"];
    refreshCounters();return event;
  }
  auto primary = processSingle(input);
  if (!radius_fallback_) return primary;
  // Keep the second database current, but avoid another query after primary success.
  auto fallback = radius_fallback_->processSingle(input, !primary.match);
  const bool selected_fallback = !primary.match && bool(fallback.match);
  const bool clear = primary.clear_display || fallback.clear_display;
  const auto primary_attempts = primary.candidate_attempts.size();
  const bool use_fallback = selected_fallback ||
      (primary.status == "rejected" && fallback.status != "rejected");
  MatchEvent result;
  if (use_fallback) {
    fallback.candidate_attempts.insert(fallback.candidate_attempts.begin(),
        std::make_move_iterator(primary.candidate_attempts.begin()),
        std::make_move_iterator(primary.candidate_attempts.end()));
    if (fallback.diagnostics.count("candidate_rank")) fallback.diagnostics["candidate_rank"] += primary_attempts;
    result = std::move(fallback);
  } else {
    primary.candidate_attempts.insert(primary.candidate_attempts.end(),
        std::make_move_iterator(fallback.candidate_attempts.begin()),
        std::make_move_iterator(fallback.candidate_attempts.end()));
    result = std::move(primary);
  }
  result.clear_display = clear;
  result.diagnostics["matching_radius_ugv"] = use_fallback ?
      config_.matching_radius_ugv_fallback : config_.matching_radius_ugv;
  result.diagnostics["radius_fallback_selected"] = selected_fallback;
  result.diagnostics["candidate_count"] = result.candidate_attempts.size();
  if (result.match) {
    // A query accepted in either view must not be retried on a later UGV export.
    pending_.reset();radius_fallback_->pending_.reset();
    if (selected_fallback) ++counters_["radius_fallback_matches"];
  }
  refreshCounters();
  return result;
}
MatchEvent Matcher::processSingle(SubmapPtr input, bool run_query) {
  MatchEvent result;
  ++counters_["received"];
  auto reject=[&](const std::string& why) { result.status="rejected";result.reason=why;++counters_["rejected"];return result; };
  if (!input || input->robot<0 || input->robot>1 || input->session.empty() || input->local_frame.empty() ||
      !std::isfinite(input->stamp) || input->stamp<0 || !input->cloud || input->cloud->size()<size_t(config_.min_points) ||
      input->frame!=(input->robot==0?config_.ugv_frame:config_.uav_frame) || !properTransform(input->pose.matrix()) ||
      !input->gravity.coeffs().allFinite() || std::abs(input->gravity.norm()-1)>1e-3)
    return reject("invalid identity/frame/pose/cloud");
  for (const auto& p:*input->cloud) if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) return reject("nonfinite cloud");
  if(input->recent_cloud) {
    if(input->recent_cloud->size()<size_t(config_.min_points)||!input->recent_center_local.allFinite() ||
       !std::isfinite(input->recent_start_stamp)||!std::isfinite(input->recent_end_stamp) ||
       input->recent_start_stamp<0||input->recent_end_stamp<input->recent_start_stamp||!input->recent_frames)
      return reject("invalid recent cloud metadata");
    for(const auto& p:*input->recent_cloud)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return reject("nonfinite recent cloud");
  }
  if (input->partial && !config_.allow_partial) return reject("partial submaps disabled");
  const int robot=input->robot;
  const auto generation=input->session+":"+std::to_string(input->epoch);
  if (generation_[robot]!=generation) {
    if (!generation_[robot].empty()) {
      result.clear_display=true; ++counters_["generation_resets"];
      pending_.reset();
      if (robot==0) {entries_.clear();rebuild();}
    }
    generation_[robot]=generation; seen_[robot].clear();
  }
  if (!seen_[robot].insert(input->identity()).second) {
    result.status="duplicate";result.reason="identity already processed";++counters_["duplicates"];return result;
  }
  Entry entry;
  try { entry=describe(input,robot==0?int(entries_.size()):int(input->id),&result.bev); }
  catch (const std::invalid_argument& e) {return reject(e.what());}
  result.diagnostics["input_matching_points"]=entry.matching_cloud->size();
  result.diagnostics["input_full_points"]=input->cloud->size();
  result.diagnostics["input_recent_cloud_used"]=entry.recent;
  if(entry.recent) {
    result.diagnostics["recent_window_frames"]=input->recent_frames;
    result.diagnostics["recent_window_span_s"]=input->recent_end_stamp-input->recent_start_stamp;
  }
  result.diagnostics["input_height_features"]=entry.height_features;
  result.diagnostics["input_valid_height_features"]=entry.valid_height_features;
  result.diagnostics["input_valid_height_fraction"]=double(entry.valid_height_features)/std::max(size_t(1),entry.height_features);
  if (robot==0) {
    entries_.push_back(entry);
    if (entries_.size()>size_t(config_.database_capacity)) {
      entries_.pop_front();rebuild();++counters_["database_rebuilds"];
    } else {
      for(size_t k=0;k<entry.variants.size();++k){auto& db=k==0?database_:extra_databases_[k-1];
        db->getClassMatchables(entry.variants[k].descriptor);db->addClassToHBST(entry.variants[k].descriptor);}
    }
    ++counters_["database_insertions"];
    result.status="database_updated";
  } else { pending_=entry; }
  if (pending_ && run_query) {
    auto q=query();
    q.clear_display=result.clear_display; q.bev=result.bev;
    q.diagnostics.insert(result.diagnostics.begin(),result.diagnostics.end());
    return q;
  }
  return result;
}
static std::pair<double,double> coverage(const Cloud::Ptr& source,const Cloud::Ptr& target,double radius) {
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  int count=0; double sum=0;std::vector<int> indices(1);std::vector<float> distances(1);
  for (const auto& p:*source) if (tree.nearestKSearch(p,1,indices,distances)>0 && distances[0]<=radius*radius) {++count;sum+=distances[0];}
  return {double(count)/source->size(),count?std::sqrt(sum/count):std::numeric_limits<double>::infinity()};
}
static Cloud::Ptr elevatedPoints(const Cloud::Ptr& cloud,const Eigen::Quaterniond& gravity,double height) {
  // A data-derived lower-height baseline; no robot altitude or world pose is used.
  std::vector<double> heights;heights.reserve(cloud->size());
  for(const auto& p:*cloud)heights.push_back((gravity*p.getVector3fMap().cast<double>()).z());
  auto ordered=heights;const size_t k=ordered.size()/10;
  std::nth_element(ordered.begin(),ordered.begin()+k,ordered.end());const double cutoff=ordered[k]+height;
  Cloud::Ptr result(new Cloud);
  for(size_t i=0;i<heights.size();++i)if(heights[i]>=cutoff)result->push_back((*cloud)[i]);
  return result;
}
MatchEvent Matcher::query(size_t hypothesis_limit,bool defer_partial_refinement) {
  MatchEvent event;
  event.status="query_pending";
  if (entries_.empty()) {event.reason="waiting for UGV database";return event;}
  ++counters_["queries"];
  struct Choice {CoarseCandidate candidate;size_t profile=0;double angle=0;Variant query;};
  std::vector<Choice> choices;
  if(config_.matching_profile=="legacy") {
    database_->getClassMatchables(pending_->descriptor);
    for(const auto& c:database_->queryFeatureCandidates(pending_->descriptor,config_.max_candidates))
      choices.push_back({c,0,0,pending_->variants[0]});
    database_->releaseQueryMatchables();
  }else {
    std::vector<Choice> all;
    for(size_t profile=0;profile<enhanced_profiles.size();++profile)for(int view=0;view<8;++view){
      double angle=view*M_PI/4;auto query=view==0?pending_->variants[profile]:makeDescriptor(pending_->input,pending_->matching_cloud,
        int(pending_->input->id),enhanced_profiles[profile].first,enhanced_profiles[profile].second,angle);
      if(query.features<size_t(config_.min_inliers)||query.valid_height==0)continue;
      auto& db=profile==0?database_:extra_databases_[profile-1];db->getClassMatchables(query.descriptor);
      for(const auto& c:db->queryFeatureCandidates(query.descriptor,entries_.size()*3))
        all.push_back({c,profile,angle,query});
      db->releaseQueryMatchables();
    }
    std::stable_sort(all.begin(),all.end(),[](const auto& a,const auto& b){
      if(a.candidate.inliers!=b.candidate.inliers)return a.candidate.inliers>b.candidate.inliers;
      if(a.candidate.planar_rmse!=b.candidate.planar_rmse)return a.candidate.planar_rmse<b.candidate.planar_rmse;
      return std::make_tuple(a.candidate.reference_id,a.profile,a.angle)<std::make_tuple(b.candidate.reference_id,b.profile,b.angle);});
    std::map<int,int> per_reference;
    auto localPose=[&](const Choice& c){Eigen::Matrix4d gu=Eigen::Matrix4d::Identity(),gq=gu;
      gu.topLeftCorner<3,3>()=matchingGravity(entries_.at(c.candidate.reference_id).input).toRotationMatrix();
      gq.topLeftCorner<3,3>()=Eigen::AngleAxisd(c.angle,Eigen::Vector3d::UnitZ()).toRotationMatrix()*matchingGravity(pending_->input).toRotationMatrix();
      return Eigen::Matrix4d(gq.inverse()*c.candidate.pose*gu);};
    for(const auto& c:all){if(!c.candidate.height_valid||c.candidate.inliers<size_t(config_.min_inliers))continue;
      const int id=c.candidate.reference_id;if(per_reference.count(id)==0&&per_reference.size()>=size_t(config_.max_candidates))continue;
      if(per_reference[id]>=3)continue;const auto pose=localPose(c);if(!properTransform(pose))continue;
      bool duplicate=false;for(const auto& selected:choices)if(selected.candidate.reference_id==id){
        const Eigen::Matrix4d delta=localPose(selected).inverse()*pose;
        if(delta.topRightCorner<3,1>().norm()<1&&std::abs(std::atan2(delta(1,0),delta(0,0)))<M_PI/18){duplicate=true;break;}}
      if(duplicate)continue;choices.push_back(c);++per_reference[id];if(choices.size()>=hypothesis_limit)break;
    }
    if(choices.empty())for(size_t i=0;i<std::min(all.size(),size_t(config_.max_candidates));++i)choices.push_back(all[i]);
    event.diagnostics["coarse_hypotheses_before_limits"]=all.size();
  }
  event.diagnostics["candidate_count"]=choices.size();
  event.diagnostics["query_height_features"]=pending_->height_features;
  event.diagnostics["query_valid_height_features"]=pending_->valid_height_features;
  event.diagnostics["query_matching_points"]=pending_->matching_cloud->size();
  if(choices.empty()) {event.reason="no EHPR candidate";if(config_.matching_profile!="partial")++counters_["query_rejections"];return event;}
  for (size_t rank=0;rank<choices.size();++rank) {
    const auto& choice=choices[rank];
    const auto& candidate=choice.candidate;
    const int index=candidate.reference_id;
    if(index<0 || index>=int(entries_.size())) throw std::runtime_error("EHPR candidate index invariant broken");
    const auto& reference=entries_[index];
    CandidateAttempt attempt;attempt.reference_identity=reference.input->identity();attempt.reference_id=reference.input->id;
    attempt.matching_radius_ugv=config_.matching_radius_ugv;
    attempt.reference_recent=reference.recent;attempt.query_recent=pending_->recent;
    attempt.inliers=candidate.inliers;attempt.height_pairs=candidate.height_pairs;
    attempt.empty_height_pairs=candidate.empty_height_pairs;attempt.height_valid=candidate.height_valid;
    attempt.bev_resolution=config_.matching_profile!="legacy"?enhanced_profiles[choice.profile].first:config_.bev_resolution;
    attempt.height_neighborhood=config_.matching_profile!="legacy"?enhanced_profiles[choice.profile].second:config_.local_density;
    attempt.query_view_deg=choice.angle*180/M_PI;attempt.planar_rmse=candidate.planar_rmse;attempt.raw_correspondences=candidate.raw_correspondences;
    std::map<std::string,double> diagnostics={
      {"candidate_rank",double(rank)},{"reference_submap_id",double(reference.input->id)},
      {"height_pairs",double(candidate.height_pairs)},{"empty_height_pairs",double(candidate.empty_height_pairs)},
      {"empty_height_pair_fraction",double(candidate.empty_height_pairs)/std::max(size_t(1),candidate.height_pairs)},
      {"height_alignment_valid",candidate.height_valid?1.:0.},{"ransac_inliers",double(candidate.inliers)},
      {"reference_height_features",double(reference.variants[choice.profile].features)},
      {"reference_valid_height_features",double(reference.variants[choice.profile].valid_height)},
      {"query_height_features",double(choice.query.features)},
      {"query_valid_height_features",double(choice.query.valid_height)},
      {"bev_resolution",attempt.bev_resolution},{"height_neighborhood",double(attempt.height_neighborhood)},
      {"query_view_deg",attempt.query_view_deg},{"planar_rmse_m",attempt.planar_rmse},
      {"reference_matching_points",double(reference.matching_cloud->size())}};
    MatchResult m;m.reference=reference.input;m.query=pending_->input;m.inliers=int(candidate.inliers);
    m.reference_recent=reference.recent;m.query_recent=pending_->recent;
    m.matching_reference_points=reference.matching_cloud->size();m.matching_query_points=pending_->matching_cloud->size();
    m.matching_radius_ugv=config_.matching_radius_ugv;m.matching_radius_uav=config_.matching_radius_uav;
    Eigen::Matrix4d gu=Eigen::Matrix4d::Identity(),gq=gu;
    gu.topLeftCorner<3,3>()=matchingGravity(m.reference).toRotationMatrix();
    gq.topLeftCorner<3,3>()=Eigen::AngleAxisd(choice.angle,Eigen::Vector3d::UnitZ()).toRotationMatrix()*matchingGravity(m.query).toRotationMatrix();
    m.coarse.matrix()=gq.inverse()*candidate.pose*gu;
    attempt.coarse=m.coarse.matrix();attempt.coarse_determinant=m.coarse.rotation().determinant();
    if(config_.matching_profile=="partial"&&defer_partial_refinement) {
      attempt.result=m;event.candidate_attempts.push_back(attempt);continue;
    }
    auto evaluate=[&]() -> std::string {
      if(!candidate.height_valid)return "height alignment invalid";
      if(candidate.inliers<size_t(config_.min_inliers))return "insufficient RANSAC inliers";
      if(!properTransform(m.coarse.matrix()))return "invalid coarse rigid transform";
      if(config_.matching_profile=="partial") {
        const auto reason=refinePartial(*reference.geometry,*pending_->geometry,config_,m);
        attempt.result=m;return reason;
      }
      Cloud::Ptr source(new Cloud),target(new Cloud);
      pcl::VoxelGrid<pcl::PointXYZ> filter;filter.setLeafSize(config_.voxel_size,config_.voxel_size,config_.voxel_size);
      filter.setInputCloud(reference.matching_cloud);filter.filter(*source);
      filter.setInputCloud(pending_->matching_cloud);filter.filter(*target);
      if(source->size()<size_t(config_.min_points)||target->size()<size_t(config_.min_points))return "insufficient downsampled points";
      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;
      gicp.setMaximumIterations(config_.max_iterations);gicp.setMaxCorrespondenceDistance(config_.correspondence_distance);
      gicp.setInputSource(source);gicp.setInputTarget(target);Cloud::Ptr aligned(new Cloud);
      gicp.align(*aligned,m.coarse.matrix().cast<float>());
      if(!gicp.hasConverged())return "GICP not converged";
      m.refined.matrix()=gicp.getFinalTransformation().cast<double>();m.fitness=gicp.getFitnessScore(20);
      if(!properTransform(m.refined.matrix())||!std::isfinite(m.fitness)||m.fitness<0||m.fitness>=config_.max_fitness)
        return "invalid GICP transform/fitness";
      auto a=coverage(aligned,target,config_.overlap_radius),b=coverage(target,aligned,config_.overlap_radius);
      m.overlap_reference=a.first;m.overlap_query=b.first;
      const auto metric=source->size()<=target->size()?a:b;m.overlap=metric.first;m.residual_rmse=metric.second;
      attempt.result=m;
      if(m.overlap<config_.min_overlap || m.residual_rmse>config_.max_residual_rmse)return "overlap/residual quality gate failed";
      if(config_.min_structural_overlap>0) {
        m.structural_check_enabled=true;
        auto elevated_source=elevatedPoints(source,m.reference->gravity,config_.structural_height_above_floor);
        auto elevated_target=elevatedPoints(target,m.query->gravity,config_.structural_height_above_floor);
        m.structural_reference_points=elevated_source->size();m.structural_query_points=elevated_target->size();
        if(elevated_source->size()<size_t(config_.min_structural_points)||elevated_target->size()<size_t(config_.min_structural_points)) {
          attempt.result=m;return "insufficient elevated structural points";
        }
        Cloud::Ptr aligned_elevated(new Cloud);
        pcl::transformPointCloud(*elevated_source,*aligned_elevated,Eigen::Affine3d(m.refined));
        m.structural_overlap_reference=coverage(aligned_elevated,elevated_target,config_.overlap_radius).first;
        m.structural_overlap_query=coverage(elevated_target,aligned_elevated,config_.overlap_radius).first;
        attempt.result=m;
        if(std::min(m.structural_overlap_reference,m.structural_overlap_query)<config_.min_structural_overlap)
          return "bidirectional structural overlap gate failed";
      }
      return {};
    };
    ++counters_["candidate_attempts"];
    attempt.reason=evaluate();event.candidate_attempts.push_back(attempt);
    if(rank==0 || attempt.reason.empty()) {
      for(const auto& diagnostic:diagnostics)event.diagnostics[diagnostic.first]=diagnostic.second;
      event.attempted=attempt.result;
    }
    if(attempt.reason.empty()) {
      if(config_.matching_profile=="partial")continue;
      event.match=m;event.status="matched";event.reason="EHPR and GICP quality gates passed";
      if(rank>0)++counters_["candidate_fallback_matches"];
      pending_.reset();++counters_["accepted_matches"];return event;
    }
    ++counters_["candidate_rejections"];
  }
  event.reason=event.candidate_attempts.front().reason;
  if(config_.matching_profile=="partial")return event;
  ++counters_["query_rejections"];return event;
}
void Matcher::displayClouds(const MatchResult& m,Cloud& ugv,Cloud& uav) {
  pcl::transformPointCloud(*m.reference->cloud,ugv,Eigen::Affine3d(m.reference->pose));
  pcl::transformPointCloud(*m.query->cloud,uav,Eigen::Affine3d(m.reference->pose*m.refined.inverse()));
}
}
