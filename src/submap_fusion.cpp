#include <ehpr_bridge/matcher.h>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ehpr_bridge {
FusedSubmap fuseSubmaps(const MatchResult& match,double voxel_size) {
  if(!std::isfinite(voxel_size)||voxel_size<=0)throw std::invalid_argument("Invalid fusion voxel size");
  Cloud ugv,uav;Matcher::displayClouds(match,ugv,uav);
  struct Voxel {Eigen::Vector3d sum=Eigen::Vector3d::Zero();size_t count=0;unsigned contributors=0;};
  std::map<std::array<int64_t,3>,Voxel> voxels;
  auto add=[&](const Cloud& cloud,unsigned contributor) {
    for(const auto& point:cloud) {
      std::array<int64_t,3> key;
      for(int axis=0;axis<3;++axis) {
        const double cell=std::floor(double(point.getVector3fMap()[axis])/voxel_size);
        if(!std::isfinite(cell)||cell<=double(std::numeric_limits<int64_t>::min())||
           cell>=double(std::numeric_limits<int64_t>::max()))throw std::invalid_argument("Invalid fusion coordinate");
        key[axis]=static_cast<int64_t>(cell);
      }
      auto& voxel=voxels[key];voxel.sum+=point.getVector3fMap().cast<double>();++voxel.count;voxel.contributors|=contributor;
    }
  };
  add(ugv,1);add(uav,2);
  FusedSubmap result;result.cloud.reserve(voxels.size());
  for(const auto& cell:voxels) {
    const auto& voxel=cell.second;const auto mean=voxel.sum/double(voxel.count);
    pcl::PointXYZRGB point;point.x=mean.x();point.y=mean.y();point.z=mean.z();point.a=255;
    if(voxel.contributors==1){point.r=40;point.g=220;point.b=70;++result.ugv_only_voxels;}
    else if(voxel.contributors==2){point.r=230;point.g=70;point.b=220;++result.uav_only_voxels;}
    else {point.r=255;point.g=200;point.b=40;++result.shared_voxels;}
    result.cloud.push_back(point);
  }
  result.cloud.is_dense=true;return result;
}
}
