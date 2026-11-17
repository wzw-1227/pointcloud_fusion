#include <ehpr_bridge/matching_io.h>
#include <ehpr_bridge/visibility.h>
#include <ehpr_bridge/session_map.h>
#include "stage3_source_record.h"
#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <unistd.h>
namespace fs=std::filesystem;
namespace ehpr_bridge {
constexpr const char* commit="b4b7137f7e675974e025fc3c8e0ab891d353739a";
template<class Point> static int saveMapCloud(const fs::path& path,const pcl::PointCloud<Point>& cloud,bool color=false) {
  if(!cloud.empty())return pcl::io::savePCDFileBinary(path.string(),cloud);
  std::ofstream out(path,std::ios::binary);out<<"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z"<<(color?" rgb":"")
    <<"\nSIZE 4 4 4"<<(color?" 4":"")<<"\nTYPE F F F"<<(color?" U":"")<<"\nCOUNT 1 1 1"<<(color?" 1":"")
    <<"\nWIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA binary\n";return out?0:-1;
}
static YAML::Node matrixNode(const Eigen::Matrix4d& m) {
  YAML::Node out;for(int r=0;r<4;++r){YAML::Node row;for(int c=0;c<4;++c)row.push_back(m(r,c));out.push_back(row);}return out;
}
static Eigen::Matrix4d readMatrix(const YAML::Node& n) {
  if (!n.IsSequence()||n.size()!=4) throw std::invalid_argument("Invalid transform shape");
  Eigen::Matrix4d m;for(int r=0;r<4;++r){if(!n[r].IsSequence()||n[r].size()!=4)throw std::invalid_argument("Invalid transform row");for(int c=0;c<4;++c)m(r,c)=n[r][c].as<double>();}
  if(!properTransform(m))throw std::invalid_argument("Invalid rigid transform");return m;
}
void saveYamlAtomic(const fs::path& path,const YAML::Node& data) {
  auto temp=path.string()+".tmp";std::ofstream stream(temp);stream<<YAML::Dump(data)<<'\n';stream.close();
  if(!stream)throw std::runtime_error("Cannot write "+temp);fs::rename(temp,path);
}
MatcherConfig loadMatcherConfig(const std::string& path) {
  const auto y=YAML::LoadFile(path);MatcherConfig c;
  if(y["upstream_commit"].as<std::string>()!=commit)throw std::invalid_argument("Wrong upstream commit");
#define READ(name) if(y[#name])c.name=y[#name].as<decltype(c.name)>()
  READ(bev_resolution);READ(height_resolution);READ(max_density);READ(image_size);READ(local_density);
  READ(matching_profile);
  READ(buckets);READ(hamming_distance);READ(distribution_mean);READ(distribution_std);READ(database_capacity);
  READ(min_inliers);READ(min_points);READ(max_iterations);READ(voxel_size);READ(correspondence_distance);
  READ(max_fitness);READ(overlap_radius);READ(min_overlap);READ(max_residual_rmse);READ(allow_partial);
  READ(ugv_frame);READ(uav_frame);
  READ(fusion_voxel_size);
  READ(matching_radius_ugv);READ(matching_radius_uav);READ(max_candidates);
  READ(matching_radius_ugv_fallback);
  READ(use_recent_matching_cloud);READ(recent_window_seconds);READ(recent_voxel_size);
  READ(recent_min_frames);READ(recent_max_frames);
  READ(min_structural_overlap);READ(structural_height_above_floor);READ(min_structural_points);
  READ(contour_min_height);READ(contour_grid);READ(contour_distance);READ(contour_high_coverage);READ(contour_low_coverage);READ(contour_rmse);READ(contour_min_pairs);
  READ(history_geometry_capacity);READ(history_translation);READ(history_rotation_deg);READ(history_novelty);
  READ(partial_high_coverage);READ(partial_low_coverage);READ(partial_rmse);READ(partial_span);
  READ(partial_direction_angle_deg);READ(partial_information_ratio);READ(partial_score_margin);
  READ(partial_cluster_translation);READ(partial_cluster_rotation_deg);READ(partial_min_pairs);
  READ(partial_direction_min_pairs);READ(partial_iterations);
  READ(partial_planar_inlier_distance);
  READ(partial_history_low_coverage);
  READ(alignment_initial_observations);READ(alignment_recovery_observations);
  READ(alignment_baseline_m);READ(alignment_structure_novelty);
  READ(visibility_min_rays);READ(visibility_min_structure_rays);READ(visibility_angle_deg);
  READ(visibility_depth_m);READ(visibility_max_conflict);
#undef READ
  return c;
}
std::vector<SubmapPtr> loadReplay(const fs::path& directory, bool chronological, bool ingestion_order) {
  std::vector<SubmapPtr> data;uint32_t last_epoch[2]={0,0};std::map<std::string,double> export_times;
  std::map<const SubmapInput*,uint64_t> received;
  for(const auto& file:fs::directory_iterator(directory)) {
    if(file.path().extension()!=".pcd")continue;
    auto sidecar=file.path();sidecar.replace_extension(".yaml");auto y=YAML::LoadFile(sidecar.string());
    if(y["upstream_commit"].as<std::string>()!=commit)throw std::runtime_error("Wrong PCD upstream version");
    auto input=std::make_shared<SubmapInput>();input->robot=y["robot_id"].as<int>();
    if(input->robot<0||input->robot>1)throw std::runtime_error("Invalid PCD robot id");
    input->session=y["session_id"].as<std::string>();input->epoch=y["epoch"].as<uint32_t>();input->id=y["submap_id"].as<uint32_t>();
    input->frame=y["reference_frame"].as<std::string>();input->local_frame=y["local_frame"].as<std::string>();
    input->stamp=y["keyframe_timestamp"].as<double>();input->partial=y["partial"].as<bool>();
    if(y["observation_quality_valid"])input->observation_quality_valid=y["observation_quality_valid"].as<bool>();
    input->pose.matrix()=readMatrix(y["T_reference_key"]);auto gravity=readMatrix(y["T_gravity_key"]);
    if(gravity.topRightCorner<3,1>().norm()>1e-9)throw std::runtime_error("Gravity transform contains translation");
    input->gravity=Eigen::Quaterniond(gravity.topLeftCorner<3,3>());
    if(y["T_matching_gravity_key"]) {
      const auto effective=readMatrix(y["T_matching_gravity_key"]);
      if(effective.topRightCorner<3,1>().norm()>1e-9)throw std::runtime_error("Matching gravity contains translation");
      input->matching_gravity=Eigen::Quaterniond(effective.topLeftCorner<3,3>());
    }
    input->cloud.reset(new Cloud);if(pcl::io::loadPCDFile(file.path().string(),*input->cloud)<0)throw std::runtime_error("Cannot read PCD");
    if(input->cloud->size()!=y["point_count"].as<size_t>())throw std::runtime_error("PCD point count mismatch");
    if(y["recent_matching_cloud"]) {
      auto recent=y["recent_matching_cloud"];fs::path relative(recent["pcd"].as<std::string>());
      if(relative.is_absolute()||relative.lexically_normal().string().find("..")!=std::string::npos)
        throw std::runtime_error("Recent PCD path must remain inside the replay directory");
      input->recent_cloud.reset(new Cloud);
      if(pcl::io::loadPCDFile((directory/relative).string(),*input->recent_cloud)<0)throw std::runtime_error("Cannot read recent PCD");
      if(input->recent_cloud->size()!=recent["point_count"].as<size_t>())throw std::runtime_error("Recent PCD point count mismatch");
      input->recent_start_stamp=recent["start_stamp"].as<double>();input->recent_end_stamp=recent["end_stamp"].as<double>();
      input->recent_frames=recent["frames"].as<size_t>();
      for(int axis=0;axis<3;++axis)input->recent_center_local[axis]=recent["center_local"][axis].as<double>();
      if(recent["latest_scan_pcd"]){fs::path scan(recent["latest_scan_pcd"].as<std::string>());
        if(scan.is_absolute()||scan.lexically_normal().string().find("..")!=std::string::npos)throw std::runtime_error("Invalid latest scan path");
        input->latest_scan.reset(new Cloud);if(pcl::io::loadPCDFile((directory/scan).string(),*input->latest_scan)<0)throw std::runtime_error("Cannot load latest scan");}
    }
    export_times[input->identity()]=y["segment_end_timestamp"]?y["segment_end_timestamp"].as<double>():input->stamp;
    last_epoch[input->robot]=std::max(last_epoch[input->robot],input->epoch);data.push_back(input);
    if(ingestion_order){
      if(y["ingestion_sequence"])received[input.get()]=y["ingestion_sequence"].as<uint64_t>();
      else{auto name=sidecar.stem().string(),marker=std::string("_submap_event_");auto at=name.rfind(marker);
        if(at!=std::string::npos)received[input.get()]=std::stoull(name.substr(at+marker.size()));}
    }
  }
  if(!chronological)data.erase(std::remove_if(data.begin(),data.end(),[&](const auto& x){return x->epoch!=last_epoch[x->robot];}),data.end());
  std::sort(data.begin(),data.end(),[&](const auto& a,const auto& b){
    if(ingestion_order&&received.size()==data.size()&&received.at(a.get())!=received.at(b.get()))return received.at(a.get())<received.at(b.get());
    if(chronological && export_times.at(a->identity())!=export_times.at(b->identity()))
      return export_times.at(a->identity())<export_times.at(b->identity());
    return std::make_pair(a->robot,a->id)<std::make_pair(b->robot,b->id);
  });
  if(data.empty())throw std::runtime_error("No submap PCD inputs");return data;
}
YAML::Node inputMetadata(const SubmapInput& m) {
  YAML::Node y;y["robot_id"]=m.robot;y["session_id"]=m.session;y["epoch"]=m.epoch;y["submap_id"]=m.id;
  y["session_id"].SetTag("tag:yaml.org,2002:str");
  y["keyframe_timestamp"]=m.stamp;y["reference_frame"]=m.frame;y["local_frame"]=m.local_frame;y["partial"]=m.partial;
  y["point_count"]=m.cloud->size();y["T_reference_key"]=matrixNode(m.pose.matrix());
  y["observation_quality_valid"]=m.observation_quality_valid;
  Eigen::Matrix4d g=Eigen::Matrix4d::Identity();g.topLeftCorner<3,3>()=m.gravity.toRotationMatrix();y["T_gravity_key"]=matrixNode(g);
  if(m.matching_gravity) {
    g.topLeftCorner<3,3>()=m.matching_gravity->toRotationMatrix();y["T_matching_gravity_key"]=matrixNode(g);
    y["matching_gravity_convention"]="Rz(-atan2(R_reference_key(1,0),R_reference_key(0,0))) * R_reference_key";
  }return y;
}
YAML::Node matchMetadata(const MatchResult& m) {
  YAML::Node y;y["reference"]=inputMetadata(*m.reference);y["query"]=inputMetadata(*m.query);
  y["T_query_reference_coarse"]=matrixNode(m.coarse.matrix());y["T_query_reference_refined"]=matrixNode(m.refined.matrix());
  y["convention"]="T_UAVkey_UGVkey maps UGV keyframe points into UAV keyframe; not current robot poses";
  y["has_competing_cluster"]=m.has_competing_cluster;
  if(m.visibility.available||m.refinement_method!="pcl_gicp"){
    auto v=y["visibility_validation"];v["version"]=2;v["source"]=m.visibility.source;v["available"]=m.visibility.available;
    v["sufficient"]=m.visibility.sufficient;v["conflict"]=m.visibility.conflict;v["reason"]=m.visibility.reason;
    v["reference_scan_stamp"]=m.visibility.reference_stamp;v["query_scan_stamp"]=m.visibility.query_stamp;
    auto direction=[&](const std::string& key,const VisibilityDirection& d){auto n=v[key];n["checked"]=d.checked;n["conflicts"]=d.conflicts;
      n["structure_checked"]=d.structure_checked;n["structure_conflicts"]=d.structure_conflicts;n["sufficient"]=d.sufficient;};
    direction("ugv_in_uav",m.visibility.ugv_in_uav);direction("uav_in_ugv",m.visibility.uav_in_ugv);
    auto a=y["alignment_validation"];a["version"]=1;a["state"]=m.alignment_state;a["independent_observations"]=m.independent_observations;
    a["required_observations"]=m.required_observations;a["confirmation_wait_sim_s"]=m.confirmation_wait_seconds;
  }
  y["ransac_inliers"]=m.inliers;y["fitness_m2"]=m.fitness;y["fitness_max_range_argument"]=20;
  y["overlap_reference"]=m.overlap_reference;y["overlap_query"]=m.overlap_query;
  y["overlap_smaller_cloud"]=m.overlap;y["residual_rmse_m"]=m.residual_rmse;
  auto scope=y["matching_scope"];
  scope["ugv_horizontal_radius_m"]=m.matching_radius_ugv;
  scope["uav_horizontal_radius_m"]=m.matching_radius_uav;
  scope["reference_points_before_downsampling"]=m.matching_reference_points;
  scope["query_points_before_downsampling"]=m.matching_query_points;
  scope["fusion_uses_full_input_clouds"]=true;
  scope["reference_cloud_source"]=m.reference_recent?"recent":"history";
  scope["query_cloud_source"]=m.query_recent?"recent":"history";
  if(m.reference_recent) {
    scope["reference_window_start_stamp"]=m.reference->recent_start_stamp;
    scope["reference_window_end_stamp"]=m.reference->recent_end_stamp;
    scope["reference_window_frames"]=m.reference->recent_frames;
  }
  if(m.query_recent) {
    scope["query_window_start_stamp"]=m.query->recent_start_stamp;
    scope["query_window_end_stamp"]=m.query->recent_end_stamp;
    scope["query_window_frames"]=m.query->recent_frames;
  }
  auto structure=y["structural_check"];
  structure["enabled"]=m.structural_check_enabled;
  structure["reference_points"]=m.structural_reference_points;structure["query_points"]=m.structural_query_points;
  structure["overlap_reference"]=m.structural_overlap_reference;structure["overlap_query"]=m.structural_overlap_query;
  y["refinement_method"]=m.refinement_method;
  y["registration_branch"]=m.registration_branch;
  if(m.registration_branch=="contour_xy_ground") {
    auto n=y["contour_validation"];n["version"]=1;n["converged"]=m.refinement_converged;n["iterations"]=m.refinement_iterations;
    n["reference_points"]=m.contour_reference_points;n["query_points"]=m.contour_query_points;n["independent_correspondences"]=m.contour_correspondences;
    n["overlap_reference"]=m.contour_overlap_reference;n["overlap_query"]=m.contour_overlap_query;n["xy_rmse_m"]=m.contour_residual_rmse;
    n["span_m"]=m.structural_span;n["direction_separation_deg"]=m.direction_separation_deg;n["direction_support_first"]=m.direction_support_first;n["direction_support_second"]=m.direction_support_second;
    n["xy_information_ratio"]=m.contour_information_ratio;n["query_height_above_ground_m"]=m.query_height_above_ground;
    n["ground_height_reference_map_m"]=m.ground_height_reference;n["ground_height_query_map_m"]=m.ground_height_query;
    n["ground_support_reference"]=m.ground_support_reference;n["ground_support_query"]=m.ground_support_query;
    n["ground_rmse_reference_m"]=m.ground_rmse_reference;n["ground_rmse_query_m"]=m.ground_rmse_query;
    n["candidate_score"]=m.candidate_score;n["score_margin"]=m.score_margin;
    n["reflection_checked"]=m.reflection_checked;n["reflection_score"]=m.reflection_score;
    n["visibility_check_enabled"]=m.visibility_check_enabled;n["visibility_checked_rays"]=m.visibility_checked_rays;n["visibility_contradictions"]=m.visibility_contradictions;
    n["height_source"]="independent_historical_ground_common_flat_ground";
    n["alignment_prior_used"]=m.alignment_prior_used;n["alignment_prior_revision"]=m.alignment_prior_revision;
    if(m.alignment_prior_used)n["T_UGVmap_UAVmap_prior"]=matrixNode(m.alignment_prior.matrix());
  }
  if(m.refinement_method=="gravity_point_to_plane_huber_irls") {
    auto partial=y["partial_validation"];partial["version"]=1;partial["converged"]=m.refinement_converged;
    partial["iterations"]=m.refinement_iterations;
    partial["last_translation_update_m"]=m.translation_update_m;
    partial["last_rotation_update_deg"]=m.rotation_update_deg;
    partial["effective_correspondences"]=m.effective_correspondences;
    partial["structural_correspondences"]=m.structural_correspondences;
    partial["correspondence_rmse_m"]=m.partial_residual_rmse;
    partial["structural_span_m"]=m.structural_span;partial["direction_separation_deg"]=m.direction_separation_deg;
    partial["direction_support_first"]=m.direction_support_first;partial["direction_support_second"]=m.direction_support_second;
    partial["horizontal_support_reference"]=m.horizontal_support_reference;
    partial["horizontal_support_query"]=m.horizontal_support_query;
    partial["horizontal_plane_residual_m"]=m.horizontal_plane_residual;
    partial["visibility_check_enabled"]=m.visibility_check_enabled;
    partial["visibility_checked_rays"]=m.visibility_checked_rays;
    partial["visibility_contradictions"]=m.visibility_contradictions;
    partial["historical_cloud_gate"]=!m.reference_recent||!m.query_recent;
    partial["information_ratio"]=m.information_ratio;partial["candidate_score"]=m.candidate_score;
    partial["score_margin"]=m.score_margin;
  }
  return y;
}
void MatchWriter::checkpoint(const Matcher& matcher) {
  auto state=matcher.alignment();if(!state)return;
  const std::string name="map_"+std::to_string(matcher.mapRevision());const auto folder=directory_/name;
  if(!fs::exists(folder)) {
    auto temporary=directory_/(name+".tmp");fs::create_directories(temporary);
    auto ugv=matcher.mapCloud(0),uav=matcher.mapCloud(1);auto fused=matcher.fusedMap();
    if(saveMapCloud(temporary/"ugv_map.pcd",ugv)||saveMapCloud(temporary/"uav_map.pcd",uav)||
       saveMapCloud(temporary/"fused_map.pcd",fused.cloud,true))throw std::runtime_error("Cannot persist cumulative maps");
    YAML::Node y;y["version"]=1;y["valid"]=state->valid;y["alignment_revision"]=state->revision;y["map_revision"]=matcher.mapRevision();
    y["reference_frame"]=matcher.config().ugv_frame;y["query_frame"]=matcher.config().uav_frame;y["T_UGVmap_UAVmap"]=matrixNode(state->transform.matrix());
    y["voxel_size_m"]=matcher.config().fusion_voxel_size;y["ugv_points"]=ugv.size();y["uav_points"]=uav.size();y["fused_points"]=fused.cloud.size();
    y["ugv_only_voxels"]=fused.ugv_only_voxels;y["uav_only_voxels"]=fused.uav_only_voxels;y["shared_voxels"]=fused.shared_voxels;y["reason"]=state->reason;
    if(state->valid){y["reference_identity"]=state->reference->identity();y["query_identity"]=state->query->identity();
      y["reference_session_id"]=state->reference->session;y["query_session_id"]=state->query->session;
      y["reference_session_id"].SetTag("tag:yaml.org,2002:str");y["query_session_id"].SetTag("tag:yaml.org,2002:str");
      y["reference_epoch"]=state->reference->epoch;y["query_epoch"]=state->query->epoch;}
    saveYamlAtomic(temporary/"map.yaml",y);fs::rename(temporary,folder);
  }
  summary_["latest_map_checkpoint"]=name+"/map.yaml";summary_["alignment_valid"]=state->valid;summary_["alignment_revision"]=state->revision;
  saveYamlAtomic(directory_/"summary.yaml",summary_);
}
MatchWriter::MatchWriter(const std::string& root,const std::string& config,const std::string& mode) {
  const auto now=std::chrono::system_clock::now();auto seconds=std::chrono::system_clock::to_time_t(now)+8*3600;std::tm calendar{};gmtime_r(&seconds,&calendar);
  auto micros=std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()%1000000;
  std::ostringstream name;name<<std::put_time(&calendar,"%Y%m%d_%H%M%S_")<<micros<<'_'<<getpid();
  directory_=fs::absolute(root).lexically_normal()/name.str();if(!fs::create_directories(directory_))throw std::runtime_error("Output directory already exists");
  fs::copy_file(config,directory_/"parameters.yaml");
  std::ofstream source_record(directory_/"compiled_source.json");
  source_record<<stage3_source_record;source_record.close();
  if(!source_record)throw std::runtime_error("Cannot write compiled source record");
  summary_["run_id"]=name.str();summary_["timezone"]="Asia/Shanghai";summary_["mode"]=mode;
  summary_["run_id"].SetTag("tag:yaml.org,2002:str");
  journal_.open(directory_/"events.yaml",std::ios::app);
  if(!journal_)throw std::runtime_error("Cannot open event journal");
  summary_["events_file"]="events.yaml";summary_["events_complete"]=false;
  summary_["upstream_commit"]=commit;summary_["status"]="running";saveYamlAtomic(directory_/"summary.yaml",summary_);
}
std::string MatchWriter::write(const MatchEvent& e,SubmapPtr input,const Matcher& matcher) {
  YAML::Node event;event["event_id"]=event_id_++;event["identity"]=input->identity();event["status"]=e.status;event["reason"]=e.reason;
  if(!e.revocation_reason.empty()){event["revocation_reason"]=e.revocation_reason;event["revoked_alignment_revision"]=e.revoked_alignment_revision;
    summary_["last_alignment_revocation_reason"]=e.revocation_reason;}
  event["matching_profile"]=matcher.config().matching_profile;
  event["refinement_method"]=matcher.config().matching_profile=="partial_contour"?"partial_3d_then_contour_xy_ground":
    (matcher.config().matching_profile=="partial"?"gravity_point_to_plane_huber_irls":"pcl_gicp");
  const bool advanced=matcher.alignment()!=nullptr;
  if(input->cloud&&!input->cloud->empty()&&(input->recent_cloud||advanced)&&
     (!advanced||(e.status!="duplicate"&&e.status!="rejected"))) {
    const auto folder=directory_/"recent_inputs";fs::create_directories(folder/"recent");
    const std::string name=(input->robot==0?"ugv":"uav")+std::string("_submap_event_")+std::to_string(event_id_);
    if(pcl::io::savePCDFileBinary((folder/(name+".pcd")).string(),*input->cloud) ||
       (input->recent_cloud&&pcl::io::savePCDFileBinary((folder/"recent"/(name+".pcd")).string(),*input->recent_cloud)))
      throw std::runtime_error("Cannot persist recent matching input");
    auto metadata=inputMetadata(*input);metadata["upstream_commit"]=commit;
    metadata["ingestion_sequence"]=event_id_;
    metadata["segment_end_timestamp"]=input->recent_cloud?input->recent_end_stamp:input->stamp;
    if(input->recent_cloud){
    auto recent=metadata["recent_matching_cloud"];recent["pcd"]="recent/"+name+".pcd";
    recent["point_count"]=input->recent_cloud->size();recent["start_stamp"]=input->recent_start_stamp;
    if(input->latest_scan){auto scan=folder/"recent"/(name+"_scan.pcd");if(pcl::io::savePCDFileBinary(scan.string(),*input->latest_scan))throw std::runtime_error("Cannot persist single scan");recent["latest_scan_pcd"]="recent/"+name+"_scan.pcd";}
    recent["end_stamp"]=input->recent_end_stamp;recent["frames"]=input->recent_frames;
    for(int axis=0;axis<3;++axis)recent["center_local"].push_back(input->recent_center_local[axis]);
    }
    saveYamlAtomic(folder/(name+".yaml"),metadata);event["input_path"]=(folder/(name+".yaml")).string();
  }
  for (const auto& item:e.diagnostics) event["diagnostics"][item.first]=item.second;
  for (const auto& attempt:e.candidate_attempts) {
    YAML::Node y;y["reference_identity"]=attempt.reference_identity;y["reference_submap_id"]=attempt.reference_id;
    y["matching_radius_ugv_m"]=attempt.matching_radius_ugv;
    y["reference_cloud_source"]=attempt.reference_recent?"recent":"history";
    y["query_cloud_source"]=attempt.query_recent?"recent":"history";
    y["ransac_inliers"]=attempt.inliers;y["height_alignment_valid"]=attempt.height_valid;
    y["height_pairs"]=attempt.height_pairs;y["empty_height_pairs"]=attempt.empty_height_pairs;
    y["reason"]=attempt.reason.empty()?"accepted":attempt.reason;
    y["bev_resolution"]=attempt.bev_resolution;y["height_neighborhood"]=attempt.height_neighborhood;
    y["query_view_deg"]=attempt.query_view_deg;y["planar_rmse_m"]=attempt.planar_rmse;
    y["raw_correspondences"]=attempt.raw_correspondences;y["independent_correspondences"]=attempt.inliers;
    y["candidate_generator"]=attempt.generator;
    y["coarse_rotation_determinant"]=attempt.coarse_determinant;
    y["coarse_transform"]=matrixNode(attempt.coarse);
    if(attempt.result)y["registration"]=matchMetadata(*attempt.result);
    event["candidate_attempts"].push_back(y);
  }
  if(!e.bev.empty()) {
    std::string file="bev_"+std::to_string(event_id_)+(input->robot==0?"_ugv.png":"_uav.png");cv::Mat image;e.bev.convertTo(image,CV_8U);
    if(!cv::imwrite((directory_/file).string(),image))throw std::runtime_error("BEV write failed");event["bev"]=file;
  }
  if(e.attempted)event["attempted_match"]=matchMetadata(*e.attempted);
  std::string path;
  if(e.match) {
    std::ostringstream name;name<<"match_"<<std::setw(3)<<std::setfill('0')<<match_id_;
    auto temporary=directory_/(name.str()+".tmp"),done=directory_/name.str();fs::create_directory(temporary);
    const auto& m=*e.match;Cloud aligned;pcl::transformPointCloud(*m.reference->cloud,aligned,Eigen::Affine3d(m.refined));
    if(pcl::io::savePCDFileBinary((temporary/"reference.pcd").string(),*m.reference->cloud) ||
       pcl::io::savePCDFileBinary((temporary/"query.pcd").string(),*m.query->cloud) ||
       pcl::io::savePCDFileBinary((temporary/"reference_in_query.pcd").string(),aligned))throw std::runtime_error("Matched PCD write failed");
    const auto fused=fuseSubmaps(m,matcher.config().fusion_voxel_size);
    if(pcl::io::savePCDFileBinary((temporary/"fused_submap.pcd").string(),fused.cloud))throw std::runtime_error("Fused PCD write failed");
    auto meta=matchMetadata(m);meta["upstream_commit"]=commit;meta["status"]="accepted";
    for(size_t index=0;!m.confirmation_supports.empty()&&index<=m.confirmation_supports.size();++index){
      const bool current=index==m.confirmation_supports.size();
      const auto& support=current?m:*m.confirmation_supports[index];
      const auto relative=current?fs::path("observation"):fs::path("confirmation")/("support_"+std::to_string(index));
      const auto folder=temporary/relative;fs::create_directories(folder);
      auto observation=[&](const std::string& role,SubmapPtr p){
        if(!p->cloud||!p->recent_cloud||!p->latest_scan)throw std::runtime_error("Confirmation lacks original scan evidence");
        fs::create_directories(folder/"recent");
        if(pcl::io::savePCDFileBinary((folder/(role+".pcd")).string(),*p->cloud)||
           pcl::io::savePCDFileBinary((folder/"recent"/(role+"_recent.pcd")).string(),*p->recent_cloud)||
           pcl::io::savePCDFileBinary((folder/"recent"/(role+"_scan.pcd")).string(),*p->latest_scan))throw std::runtime_error("Cannot persist confirmation evidence");
        auto n=inputMetadata(*p);n["upstream_commit"]=commit;auto recent=n["recent_matching_cloud"];
        recent["pcd"]="recent/"+role+"_recent.pcd";recent["latest_scan_pcd"]="recent/"+role+"_scan.pcd";recent["point_count"]=p->recent_cloud->size();
        recent["start_stamp"]=p->recent_start_stamp;recent["end_stamp"]=p->recent_end_stamp;recent["frames"]=p->recent_frames;
        for(int axis=0;axis<3;++axis)recent["center_local"].push_back(p->recent_center_local[axis]);
        n["visibility_floor_local_m"]=p->visibility_scan->floor_local;saveYamlAtomic(folder/(role+".yaml"),n);
      };
      observation("ugv",support.reference);observation("uav",support.query);
      saveYamlAtomic(folder/"registration.yaml",matchMetadata(support));
      if(current)meta["alignment_validation"]["current_observation"]=(relative/"registration.yaml").string();
      else meta["alignment_validation"]["supports"].push_back((relative/"registration.yaml").string());
    }
    if(m.registration_branch=="contour_xy_ground")for(const auto& d:e.diagnostics)
      if(d.first.rfind("ground_",0)==0)meta["ground_validation"][d.first]=d.second;
    auto fusion=meta["fusion"];fusion["pcd"]="fused_submap.pcd";fusion["reference_frame"]=m.reference->frame;
    fusion["voxel_size_m"]=matcher.config().fusion_voxel_size;fusion["point_count"]=fused.cloud.size();
    fusion["ugv_only_voxels"]=fused.ugv_only_voxels;fusion["uav_only_voxels"]=fused.uav_only_voxels;fusion["shared_voxels"]=fused.shared_voxels;
    fusion["colors"]="green=UGV only, magenta=UAV only, yellow=both contributed to voxel; not match confidence";
    saveYamlAtomic(temporary/"match.yaml",meta);
    fs::rename(temporary,done);++match_id_;path=(done/"match.yaml").string();event["result_path"]=path;
  }
  journal_<<"---\n"<<YAML::Dump(event)<<'\n';journal_.flush();
  if(!journal_)throw std::runtime_error("Cannot persist event journal");
  YAML::Node compact;
  for(const auto* key:{"event_id","identity","status","reason","result_path"})
    if(event[key])compact[key]=event[key];
  recent_events_.push_back(compact);if(recent_events_.size()>20)recent_events_.pop_front();
  summary_["events"]=YAML::Node(YAML::NodeType::Sequence);
  for(const auto& e:recent_events_)summary_["events"].push_back(e);
  summary_["event_count"]=event_id_;summary_["accepted_matches"]=match_id_;summary_["database_size"]=matcher.databaseSize();
  if(auto state=matcher.alignment()){
    summary_["alignment_valid"]=state->valid;summary_["alignment_revision"]=state->revision;
    auto code=e.diagnostics.find("alignment_state_code");summary_["alignment_state"]=state->valid?"confirmed":
      ((code!=e.diagnostics.end()&&code->second==3)?"relocalizing":"waiting_confirmation");
    for(const std::string key:{"independent_observations","required_observations"}){
      auto d=e.diagnostics.find(key);if(d!=e.diagnostics.end())summary_[key]=d->second;
    }
  }
  YAML::Node counts;for(const auto& item:matcher.counters())counts[item.first]=item.second;summary_["counters"]=counts;
  saveYamlAtomic(directory_/"summary.yaml",summary_);return path;
}
void MatchWriter::finish(const std::string& status,const std::string& error) {
  journal_.flush();
  const auto events=YAML::LoadAllFromFile((directory_/"events.yaml").string());
  summary_["events"]=YAML::Node(YAML::NodeType::Sequence);
  for(const auto& event:events)summary_["events"].push_back(event);
  summary_["events_complete"]=true;
  summary_["status"]=status;summary_["error"]=error;saveYamlAtomic(directory_/"summary.yaml",summary_);
  std::ofstream report(directory_/"report.md");report<<"# Stage 3–4 matching run\n\nStatus: "<<status<<"\n\nAccepted pairs: "<<match_id_<<"\n\n"<<error
    <<"\n\nMatching success alone does not certify Gazebo pose accuracy or full-map fusion.\n";
  report<<"\n## Events\n\n";
  for(const auto& e:events) {
    report<<"- `"<<e["identity"].as<std::string>()<<"`: "<<e["status"].as<std::string>()<<" — "<<e["reason"].as<std::string>();
    if(e["result_path"])report<<"; "<<e["result_path"].as<std::string>();
    report<<'\n';
  }
  if(!report)throw std::runtime_error("Cannot write matching report");
}
}
