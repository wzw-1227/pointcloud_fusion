#include <ehpr_bridge/submap_session.h>
#include <VoxelMap.hpp>
#include <cmath>
#include <stdexcept>

namespace ehpr_bridge {
SubmapSession::SubmapSession(const SubmapConfig& config) : config_(config) {
  if (!std::isfinite(config.voxel_size) || config.voxel_size <= 0 ||
      !std::isfinite(config.max_distance) || config.max_distance <= 0 ||
      !std::isfinite(config.remove_distance) || config.remove_distance <= 0 ||
      config.max_points_per_voxel <= 0)
    throw std::invalid_argument("VoxelMap configuration must be finite and positive");
  VoxelMapConfig upstream;
  upstream.voxel_size = config.voxel_size;
  upstream.max_distance = config.max_distance;
  upstream.remove_distance = config.remove_distance;
  upstream.max_points_per_voxel = config.max_points_per_voxel;
  map_ = std::make_unique<VoxelMap>(upstream);
}
SubmapSession::~SubmapSession() { map_->Clear(); }

Eigen::Quaterniond SubmapSession::gravityRotation(const Eigen::Matrix3d& rotation) {
  // Same Z-Y-X Euler decomposition and pitch*roll composition as upstream.
  const Eigen::Vector3d euler = rotation.eulerAngles(2, 1, 0);
  return Eigen::Quaterniond(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()));
}

std::optional<SubmapSnapshot> SubmapSession::add(const Cloud::Ptr& cloud,
                                               const Eigen::Isometry3d& pose, double stamp) {
  if (!cloud || cloud->empty() || !pose.matrix().allFinite() || !std::isfinite(stamp))
    throw std::invalid_argument("Submap input must contain finite pose/time and nonempty points");
  if (initialized_ && stamp <= end_stamp_)
    throw std::invalid_argument("Submap input timestamps must increase strictly");
  if (!initialized_) {
    last_pos_ = pose.translation();
    initialized_ = true;
  }
  if (!pending_) {
    first_pose_ = pose;
    start_stamp_ = stamp;
    segment_frames_ = 0;
  }
  map_->addPoints(cloud);
  travel_ += (pose.translation() - last_pos_).norm();
  last_pos_ = pose.translation();
  end_stamp_ = stamp;
  pending_ = true;
  ++segment_frames_;
  if (!record_keypose_ && travel_ >= config_.max_distance * 0.01) {
    key_pose_ = pose;
    key_stamp_ = stamp;
    key_gravity_ = gravityRotation(pose.rotation());
    record_keypose_ = true;
  }
  if (travel_ > config_.max_distance && record_keypose_) return snapshot(false);
  return std::nullopt;
}

SubmapSnapshot SubmapSession::snapshot(bool partial) {
  SubmapSnapshot result;
  result.local.reset(new Cloud);
  map_->getAllPoints(result.local);
  result.pose = record_keypose_ ? key_pose_ : first_pose_;
  result.key_stamp = record_keypose_ ? key_stamp_ : start_stamp_;
  result.gravity = record_keypose_ ? key_gravity_ : gravityRotation(first_pose_.rotation());
  pcl::transformPointCloud(*result.local, *result.local, Eigen::Affine3d(result.pose.inverse()));
  result.start_stamp = start_stamp_;
  result.end_stamp = end_stamp_;
  result.travel_distance = travel_;
  result.segment_frames = segment_frames_;
  result.partial = partial;
  // Upstream keeps nearby history after output; do not clear the map here.
  map_->removePointsFarFromLocation(last_pos_);
  travel_ = 0;
  record_keypose_ = false;
  pending_ = false;
  return result;
}

std::optional<SubmapSnapshot> SubmapSession::flush() {
  if (!pending_ || map_->Empty()) return std::nullopt;
  return snapshot(true);
}
std::optional<SubmapSnapshot> SubmapSession::preview(const Eigen::Isometry3d& pose,double stamp) const {
  if(!pending_||map_->Empty())return std::nullopt;
  if(!pose.matrix().allFinite()||!std::isfinite(stamp)||std::abs(stamp-end_stamp_)>1e-6)
    throw std::invalid_argument("Preview requires the latest synchronized pose/time");
  SubmapSnapshot result;result.local.reset(new Cloud);map_->getAllPoints(result.local);
  result.pose=pose;result.key_stamp=stamp;result.gravity=gravityRotation(pose.rotation());
  pcl::transformPointCloud(*result.local,*result.local,Eigen::Affine3d(pose.inverse()));
  result.start_stamp=start_stamp_;result.end_stamp=end_stamp_;result.travel_distance=travel_;
  result.segment_frames=segment_frames_;result.partial=true;return result;
}
void SubmapSession::reset() {
  map_->Clear();
  initialized_ = record_keypose_ = pending_ = false;
  travel_ = 0;
  segment_frames_ = 0;
}
size_t SubmapSession::points() const {
  Cloud::Ptr cloud(new Cloud);
  map_->getAllPoints(cloud);
  return cloud->size();
}
}  // namespace ehpr_bridge
