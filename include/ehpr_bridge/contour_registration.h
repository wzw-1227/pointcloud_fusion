#pragma once
#include <ehpr_bridge/partial_registration.h>

namespace ehpr_bridge {
struct GroundAnchor {
  bool valid=false, conflict=false;
  double height=0, rmse=0, span=0, width=0;
  size_t support=0;
  std::string source;
};
GroundAnchor estimateHistoricalGround(const SubmapInput& input);
struct ContourLevel {
  Cloud::Ptr cloud;
  std::vector<Eigen::Vector2d> normals;
  std::vector<bool> valid;
  std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> tree;
};
struct ContourGeometry {
  std::array<ContourLevel,3> levels;
  double ground_local=0, height_above_ground=0;
};
struct ContourSeed {Eigen::Isometry3d aligned=Eigen::Isometry3d::Identity();size_t inliers=0;double quality=0;};
std::vector<ContourSeed> contourLineSeeds(const ContourGeometry& reference,const ContourGeometry& query);
size_t contourSeedSupport(const ContourGeometry& reference,const ContourGeometry& query,const Eigen::Isometry3d& aligned);
std::shared_ptr<ContourGeometry> reflectedContour(const ContourGeometry& input);
std::shared_ptr<ContourGeometry> prepareContour(const SubmapInput& input,const GroundAnchor& ground,
                                               const MatcherConfig& config);
std::string refineContour(const ContourGeometry& reference,const ContourGeometry& query,
                          const GroundAnchor& ground_reference,const GroundAnchor& ground_query,
                          const MatcherConfig& config,MatchResult& result);
std::string validateContourTransform(const ContourGeometry& reference,const ContourGeometry& query,
                                    const MatcherConfig& config,MatchResult& result);
}
