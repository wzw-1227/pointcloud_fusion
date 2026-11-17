#pragma once
#include <ehpr_bridge/matcher.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace ehpr_bridge {
struct VisibilityScan {
  Cloud::Ptr points,directions;
  std::vector<double> depths;
  std::vector<bool> structure;
  std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> tree;
  Eigen::Vector3d center=Eigen::Vector3d::Zero();
  double floor_local=0;
};
// floor_local is the gravity-aligned floor height in the key frame.
std::shared_ptr<const VisibilityScan> prepareVisibilityScan(const SubmapInput& input,
                                                         double floor_local,const MatcherConfig& config);
VisibilityReport checkVisibility(const SubmapInput& reference,const SubmapInput& query,
                                 const Eigen::Isometry3d& query_from_reference,const MatcherConfig& config);
std::string validateVisibility(MatchResult& match,const MatcherConfig& config);
Eigen::Isometry3d mapTransform(const MatchResult& match);
bool closeAlignment(const Eigen::Isometry3d& a,const Eigen::Isometry3d& b,const MatcherConfig& config);
}
