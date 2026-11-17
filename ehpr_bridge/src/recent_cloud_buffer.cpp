#include <ehpr_bridge/recent_cloud_buffer.h>
#include <ehpr_bridge/matcher.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <stdexcept>

namespace ehpr_bridge {
RecentCloudBuffer::RecentCloudBuffer(const RecentCloudConfig& config):config_(config) {
  if(!std::isfinite(config.window_seconds)||config.window_seconds<=0 ||
     !std::isfinite(config.voxel_size)||config.voxel_size<=0 ||
     !config.min_frames||config.max_frames<config.min_frames)
    throw std::invalid_argument("Invalid recent cloud window configuration");
}
bool RecentCloudBuffer::add(const Cloud::Ptr& cloud,const Eigen::Isometry3d& pose,double stamp) {
  if(!cloud||cloud->empty()||!properTransform(pose.matrix())||!std::isfinite(stamp)||stamp<0)
    return false;
  if(config_.reset_on_timestamp_rollback&&!frames_.empty()&&stamp<frames_.back().stamp-.05)reset();
  if(!frames_.empty()&&stamp<=frames_.back().stamp)return false;
  for(const auto& p:*cloud)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
  // A discontinuity must not mix the old odometry geometry into the new window.
  if(!frames_.empty()&&((pose.translation()-frames_.back().pose.translation()).norm()>1 ||
      Eigen::AngleAxisd(frames_.back().pose.rotation().transpose()*pose.rotation()).angle()>M_PI/4))reset();
  Cloud::Ptr reduced(new Cloud);pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(config_.voxel_size,config_.voxel_size,config_.voxel_size);
  filter.setInputCloud(cloud);filter.filter(*reduced);
  frames_.push_back({reduced,pose,stamp});
  latest_scan_=cloud;
  while(!frames_.empty()&&(stamp-frames_.front().stamp>config_.window_seconds || frames_.size()>config_.max_frames))
    frames_.pop_front();
  return true;
}
std::optional<RecentCloudSnapshot> RecentCloudBuffer::snapshot(const Eigen::Isometry3d& key_pose,double now) const {
  if(!properTransform(key_pose.matrix())||!std::isfinite(now))throw std::invalid_argument("Invalid recent cloud anchor/time");
  if(frames_.size()<config_.min_frames || now<frames_.back().stamp-.05 || now-frames_.back().stamp>1)return std::nullopt;
  Cloud::Ptr joined(new Cloud);
  for(const auto& frame:frames_)*joined+=*frame.cloud;
  Cloud reduced;pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(config_.voxel_size,config_.voxel_size,config_.voxel_size);
  filter.setInputCloud(joined);filter.filter(reduced);
  RecentCloudSnapshot out;out.local.reset(new Cloud);
  pcl::transformPointCloud(reduced,*out.local,Eigen::Affine3d(key_pose.inverse()));
  out.latest_scan.reset(new Cloud);
  pcl::transformPointCloud(*latest_scan_,*out.latest_scan,Eigen::Affine3d(key_pose.inverse()));
  out.center_local=key_pose.inverse()*frames_.back().pose.translation();
  out.start_stamp=frames_.front().stamp;out.end_stamp=frames_.back().stamp;out.frames=frames_.size();
  return out;
}
}
