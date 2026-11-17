#pragma once
#include <ehpr_bridge/matcher.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <array>

namespace ehpr_bridge {
struct PartialLevel {
  Cloud::Ptr cloud;
  std::vector<Eigen::Vector3d> normals;
  std::vector<bool> valid, vertical;
  std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> tree;
};
struct HorizontalPlane { double height=0;size_t support=0; };
struct PartialGeometry {
  std::array<PartialLevel,2> levels;std::vector<HorizontalPlane> horizontal_planes;
  Eigen::Vector3d center=Eigen::Vector3d::Zero();
};
Eigen::Quaterniond canonicalMatchingGravity(const Eigen::Matrix3d& pose);
std::shared_ptr<PartialGeometry> preparePartialGeometry(const Cloud::Ptr& cloud,
                                                      const Eigen::Quaterniond& gravity,
                                                      const Eigen::Vector3d& center=Eigen::Vector3d::Zero());
std::string refinePartial(const PartialGeometry& reference,const PartialGeometry& query,
                          const MatcherConfig& config,MatchResult& result);
std::string validatePartialTransform(const PartialGeometry& reference,const PartialGeometry& query,
                                    const MatcherConfig& config,MatchResult& result);
// All candidates, including both radius views, are scored before publishing one.
void selectPartialMatch(MatchEvent& event,const MatcherConfig& config);
}
