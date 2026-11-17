#pragma once
#include <ehpr_bridge/submap_session.h>
#include <deque>

namespace ehpr_bridge {
struct RecentCloudConfig {
  double window_seconds = 5, voxel_size = .2;
  size_t min_frames = 20, max_frames = 100;
  bool reset_on_timestamp_rollback=false;
};
struct RecentCloudSnapshot {
  Cloud::Ptr local;
  Cloud::Ptr latest_scan;
  Eigen::Vector3d center_local = Eigen::Vector3d::Zero();
  double start_stamp = 0, end_stamp = 0;
  size_t frames = 0;
};
// All frames belong to one robot's registered odometry frame. No inter-robot pose is used.
class RecentCloudBuffer {
 public:
  explicit RecentCloudBuffer(const RecentCloudConfig& config);
  bool add(const Cloud::Ptr& registered, const Eigen::Isometry3d& pose, double stamp);
  std::optional<RecentCloudSnapshot> snapshot(const Eigen::Isometry3d& key_pose, double now) const;
  void reset() {frames_.clear();latest_scan_.reset();++generation_;}
  uint64_t generation() const {return generation_;}
 private:
  struct Frame {Cloud::Ptr cloud; Eigen::Isometry3d pose; double stamp;};
  RecentCloudConfig config_;
  std::deque<Frame> frames_;
  Cloud::Ptr latest_scan_;
  uint64_t generation_=0;
};
}
