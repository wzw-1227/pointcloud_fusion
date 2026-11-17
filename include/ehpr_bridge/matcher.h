#pragma once
#include <ehpr_bridge/submap_session.h>
#include <opencv2/core.hpp>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>
class DescriptorDB;
class DescriptorManager;
namespace ehpr_bridge {
struct PartialGeometry;
struct VisibilityScan;
class AdvancedMatcher;
struct MapAlignmentState;
struct MatcherConfig {
  std::string matching_profile = "legacy";
  double bev_resolution = .5, height_resolution = 1, max_density = 135;
  int image_size = 210, local_density = 3, buckets = 16, hamming_distance = 60;
  double distribution_mean = 115, distribution_std = 20;
  int database_capacity = 200, min_inliers = 7, min_points = 20, max_iterations = 10;
  double voxel_size = .8, correspondence_distance = 5, max_fitness = 2.5;
  double overlap_radius = .5, min_overlap = .3, max_residual_rmse = .3;
  double fusion_voxel_size = .15;
  // Zero preserves the complete upstream input. Radius is measured after gravity alignment.
  double matching_radius_ugv = 0, matching_radius_uav = 0;
  // Optional larger view, tried only when the primary view has no accepted match.
  double matching_radius_ugv_fallback = 0;
  bool use_recent_matching_cloud = false;
  double recent_window_seconds = 5, recent_voxel_size = .2;
  int recent_min_frames = 20, recent_max_frames = 100;
  int max_candidates = 5;
  double min_structural_overlap = 0, structural_height_above_floor = .4;
  int min_structural_points = 20;
  // Used only by partial; legacy/enhanced retain their original acceptance rules.
  double partial_high_coverage = .7, partial_low_coverage = .25;
  double partial_rmse = .2, partial_span = 2, partial_direction_angle_deg = 30;
  double partial_information_ratio = 1e-4, partial_score_margin = .05;
  double partial_cluster_translation = .25, partial_cluster_rotation_deg = 3;
  int partial_min_pairs = 80, partial_direction_min_pairs = 20;
  int partial_iterations = 20;
  double partial_planar_inlier_distance = 1.5;
  double partial_history_low_coverage = .6;
  double contour_min_height = 2.5, contour_grid = .2, contour_distance = .3;
  double contour_high_coverage = .7, contour_low_coverage = .25, contour_rmse = .2;
  int contour_min_pairs = 60, history_geometry_capacity = 200;
  double history_translation = 1, history_rotation_deg = 10, history_novelty = .1;
  int alignment_initial_observations = 2, alignment_recovery_observations = 3;
  double alignment_baseline_m = .5, alignment_structure_novelty = .1;
  int visibility_min_rays = 50, visibility_min_structure_rays = 20;
  double visibility_angle_deg = .5, visibility_depth_m = .75, visibility_max_conflict = .05;
  bool allow_partial = true;
  std::string ugv_frame = "odom", uav_frame = "uav_lio_odom";
};
struct SubmapInput {
  int robot = 0;
  std::string session, frame, local_frame;
  uint32_t epoch = 0, id = 0;
  double stamp = 0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond gravity = Eigen::Quaterniond::Identity();
  // Matcher-local normalization is recorded separately from the Stage 2 convention.
  std::optional<Eigen::Quaterniond> matching_gravity;
  Cloud::Ptr cloud;
  Cloud::Ptr recent_cloud;
  Eigen::Vector3d recent_center_local = Eigen::Vector3d::Zero();
  double recent_start_stamp = 0, recent_end_stamp = 0;
  size_t recent_frames = 0;
  bool partial = false;
  Cloud::Ptr latest_scan;
  std::shared_ptr<const VisibilityScan> visibility_scan;
  bool observation_quality_valid = true;
  std::string identity() const;
};
using SubmapPtr = std::shared_ptr<const SubmapInput>;
struct VisibilityDirection {
  size_t checked=0, conflicts=0, structure_checked=0, structure_conflicts=0;
  bool sufficient=false;
};
struct VisibilityReport {
  VisibilityDirection ugv_in_uav, uav_in_ugv;
  bool available=false, sufficient=false, conflict=false;
  std::string source="missing_single_scan", reason="insufficient visibility evidence";
  double reference_stamp=0, query_stamp=0;
};
struct MatchResult {
  SubmapPtr reference, query;
  Eigen::Isometry3d coarse = Eigen::Isometry3d::Identity(), refined = Eigen::Isometry3d::Identity();
  int inliers = 0;
  double fitness = -1, overlap_reference = 0, overlap_query = 0, overlap = 0, residual_rmse = 0;
  size_t matching_reference_points = 0, matching_query_points = 0;
  double matching_radius_ugv = 0, matching_radius_uav = 0;
  bool reference_recent = false, query_recent = false;
  bool structural_check_enabled = false;
  size_t structural_reference_points = 0, structural_query_points = 0;
  double structural_overlap_reference = 0, structural_overlap_query = 0;
  std::string refinement_method = "pcl_gicp";
  size_t effective_correspondences = 0, structural_correspondences = 0;
  double structural_span = 0, direction_separation_deg = 0, information_ratio = 0;
  size_t direction_support_first = 0, direction_support_second = 0;
  size_t horizontal_support_reference = 0, horizontal_support_query = 0;
  double horizontal_plane_residual = 0;
  bool visibility_check_enabled = false;
  size_t visibility_checked_rays = 0, visibility_contradictions = 0;
  double candidate_score = 0, score_margin = 0, partial_residual_rmse = 0;
  bool refinement_converged = false;
  int refinement_iterations = 0;
  double translation_update_m = 0, rotation_update_deg = 0;
  std::string registration_branch = "partial_3d";
  double contour_overlap_reference = 0, contour_overlap_query = 0, contour_residual_rmse = 0;
  double contour_information_ratio = 0, query_height_above_ground = 0;
  size_t contour_reference_points = 0, contour_query_points = 0, contour_correspondences = 0;
  double ground_height_reference = 0, ground_height_query = 0, ground_rmse_reference = 0, ground_rmse_query = 0;
  size_t ground_support_reference = 0, ground_support_query = 0;
  bool alignment_prior_used=false;
  uint64_t alignment_prior_revision=0;
  Eigen::Isometry3d alignment_prior=Eigen::Isometry3d::Identity();
  bool reflection_checked=false;
  double reflection_score=0;
  bool has_competing_cluster=false;
  VisibilityReport visibility;
  size_t independent_observations=0, required_observations=0;
  std::string alignment_state="candidate";
  double confirmation_wait_seconds=0;
  // Full observation pairs used for confirmation; entries have no nested supports.
  std::vector<std::shared_ptr<const MatchResult>> confirmation_supports;
};
struct CandidateAttempt {
  std::string reference_identity, reason;
  uint32_t reference_id = 0;
  size_t inliers = 0, height_pairs = 0, empty_height_pairs = 0;
  bool height_valid = false;
  double bev_resolution = 0, query_view_deg = 0, planar_rmse = 0, coarse_determinant = 0;
  double matching_radius_ugv = 0;
  bool reference_recent = false, query_recent = false;
  int height_neighborhood = 0;
  size_t raw_correspondences = 0;
  Eigen::Matrix4d coarse = Eigen::Matrix4d::Identity();
  std::optional<MatchResult> result;
  std::string generator="ehpr_orb";
};
struct MatchEvent {
  std::string status, reason;
  std::string revocation_reason;
  uint64_t revoked_alignment_revision = 0;
  bool clear_display = false;
  cv::Mat bev;
  std::map<std::string, double> diagnostics;
  std::optional<MatchResult> match;
  std::optional<MatchResult> attempted;
  std::vector<CandidateAttempt> candidate_attempts;
};
struct FusedSubmap {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  size_t ugv_only_voxels=0, uav_only_voxels=0, shared_voxels=0;
};
// Fuse this accepted pair in the UGV LIO frame. Color records voxel contributors.
FusedSubmap fuseSubmaps(const MatchResult& match,double voxel_size);
bool properTransform(const Eigen::Matrix4d& m);
class Matcher {
 public:
  explicit Matcher(const MatcherConfig& config, bool candidate_only=false);
  ~Matcher();
  MatchEvent process(SubmapPtr input);
  void invalidate();
  void invalidate(const std::string& reason);
  void invalidateRobot(int robot,const std::string& reason);
  size_t databaseSize() const;
  const std::map<std::string, uint64_t>& counters() const;
  const MatcherConfig& config() const { return config_; }
  static void displayClouds(const MatchResult& m, Cloud& ugv, Cloud& uav);
  void setArchiveDirectory(const std::string& directory);
  void prime(const std::vector<SubmapPtr>& inputs);
  MatchEvent archiveInput(SubmapPtr input);
  size_t retryHistory();
  std::optional<std::pair<MatchEvent,SubmapPtr>> pollHistory();
  const MapAlignmentState* alignment() const;
  FusedSubmap fusedMap() const;
  uint64_t mapRevision() const;
  Cloud mapCloud(int robot) const;
  std::optional<MatchEvent> observeScans(SubmapPtr reference,SubmapPtr query,double now);
 private:
  struct Variant { std::shared_ptr<DescriptorManager> descriptor; size_t features=0, valid_height=0; };
  struct Entry { SubmapPtr input; Cloud::Ptr matching_cloud; bool recent = false; std::shared_ptr<DescriptorManager> descriptor; size_t height_features = 0, valid_height_features = 0;
    std::vector<Variant> variants;
    std::shared_ptr<PartialGeometry> geometry;
  };
  Variant makeDescriptor(SubmapPtr input,const Cloud::Ptr& local,int id,double resolution,int neighborhood,double angle,cv::Mat* image=nullptr);
  Entry describe(SubmapPtr input, int id, cv::Mat* bev = nullptr);
  void rebuild();
  MatchEvent processSingle(SubmapPtr input, bool run_query = true);
  void refreshCounters();
  MatchEvent query(size_t hypothesis_limit=30,bool defer_partial_refinement=false);
  MatcherConfig config_;
  bool candidate_only_=false;
  std::unique_ptr<DescriptorDB> database_;
  std::vector<std::unique_ptr<DescriptorDB>> extra_databases_;
  std::deque<Entry> entries_;
  std::optional<Entry> pending_;
  std::set<std::string> seen_[2];
  std::string generation_[2];
  std::map<std::string, uint64_t> counters_;
  std::unique_ptr<Matcher> radius_fallback_;
  std::map<std::string, uint64_t> combined_counters_;
  std::unique_ptr<AdvancedMatcher> advanced_;
};
}
