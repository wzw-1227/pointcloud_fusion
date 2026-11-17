#pragma once
#include <algorithm>
#include <optional>
#include <memory>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
class VoxelMap;

namespace ehpr_bridge {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
struct SubmapConfig {
  double voxel_size = 1.0, max_distance = 20.0, remove_distance = 80.0;
  int max_points_per_voxel = 25;
};

struct SubmapSnapshot {
  Cloud::Ptr local;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond gravity = Eigen::Quaterniond::Identity();
  double key_stamp = 0, start_stamp = 0, end_stamp = 0, travel_distance = 0;
  size_t segment_frames = 0;
  bool partial = false;
};

// Per-robot state extracted from upstream test/multisession_video.cpp:369-413.
// VoxelMap itself is included directly from the pinned, unmodified upstream.
class SubmapSession {
 public:
  explicit SubmapSession(const SubmapConfig& config);
  ~SubmapSession();
  SubmapSession(const SubmapSession&) = delete;
  SubmapSession& operator=(const SubmapSession&) = delete;
  std::optional<SubmapSnapshot> add(const Cloud::Ptr& registered_cloud,
                                  const Eigen::Isometry3d& pose, double stamp);
  std::optional<SubmapSnapshot> flush();
  // Read-only partial snapshot anchored at the latest synchronized lidar pose.
  // Does not consume the segment or change the official 20 m trigger.
  std::optional<SubmapSnapshot> preview(const Eigen::Isometry3d& pose,double stamp) const;
  void reset();
  bool pending() const { return pending_; }
  double travelDistance() const { return travel_; }
  size_t points() const;
 private:
  SubmapSnapshot snapshot(bool partial);
  static Eigen::Quaterniond gravityRotation(const Eigen::Matrix3d& rotation);
  SubmapConfig config_;
  std::unique_ptr<VoxelMap> map_;
  Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();
  Eigen::Isometry3d key_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d first_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond key_gravity_ = Eigen::Quaterniond::Identity();
  double travel_ = 0, key_stamp_ = 0, start_stamp_ = 0, end_stamp_ = 0;
  size_t segment_frames_ = 0;
  bool initialized_ = false, record_keypose_ = false, pending_ = false;
};
}  // namespace ehpr_bridge
