#pragma once
#include <ehpr_bridge/session_map.h>
#include <ehpr_bridge/visibility.h>
#include <functional>

namespace ehpr_bridge {
// Owns trust decisions only. Robot maps, geometry and descriptor history outlive revocation.
class AlignmentTracker {
 public:
  using Revalidate=std::function<std::string(MatchResult&,const Eigen::Isometry3d&)>;
  AlignmentTracker(const MatcherConfig& config,SessionMap& maps):config_(config),maps_(maps){}
  void consider(MatchEvent& event,const Revalidate& revalidate);
  std::optional<MatchEvent> observe(SubmapPtr reference,SubmapPtr query,double now);
  void reset();
  std::string state() const;
  size_t observations() const;
 private:
  using Cell=std::array<int64_t,3>;
  struct Evidence {MatchResult match;std::array<std::set<Cell>,2> cells;};
  struct Hypothesis {Eigen::Isometry3d anchor;std::vector<Evidence> evidence,pool;};
  Evidence evidence(const MatchResult& match) const;
  bool independent(const Evidence& candidate,const std::vector<const Evidence*>& witnesses) const;
  void revoke(const std::string& reason);
  void diagnostics(MatchEvent& event) const;
  MatcherConfig config_;SessionMap& maps_;
  bool recovering_=false;
  Eigen::Isometry3d confirmed_anchor_=Eigen::Isometry3d::Identity();
  std::vector<Evidence> confirmed_;
  std::vector<Hypothesis> hypotheses_;
  int conflict_streak_=0;
  double last_monitor_stamp_=-1,last_reference_stamp_=-1,last_query_stamp_=-1;
};
}
