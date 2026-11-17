#pragma once
#include <ehpr_bridge/matcher.h>
#include <array>

namespace ehpr_bridge {
struct MapAlignmentState {
  bool valid=false;
  uint64_t revision=0;
  Eigen::Isometry3d transform=Eigen::Isometry3d::Identity();
  SubmapPtr reference,query;
  std::string reason="waiting_for_alignment";
};
class SessionMap {
 public:
  explicit SessionMap(double voxel=.15):voxel_(voxel){}
  bool add(SubmapPtr input);
  bool accept(const MatchResult& match,bool replace_verified=false);
  void invalidateAlignment(const std::string& reason);
  void reset(const std::string& reason="session_reset");
  void resetRobot(int robot,const std::string& reason);
  FusedSubmap fused() const;
  Cloud cloud(int robot) const;
  size_t size(int robot) const {return points_.at(robot).size();}
  uint64_t revision() const {return revision_;}
  const MapAlignmentState& alignment() const {return alignment_;}
 private:
  using Key=std::array<int64_t,3>;
  Key key(const Eigen::Vector3d& p) const;
  double voxel_;
  uint64_t revision_=0;
  std::array<std::map<Key,Eigen::Vector3d>,2> points_;
  std::set<std::string> seen_,constraints_;
  MapAlignmentState alignment_;
};
}
