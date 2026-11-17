#pragma once
#include <ehpr_bridge/contour_registration.h>
#include <ehpr_bridge/session_map.h>
#include <filesystem>

namespace ehpr_bridge {
class AdvancedMatcher {
 public:
  explicit AdvancedMatcher(const MatcherConfig& config);
  ~AdvancedMatcher();
  MatchEvent process(SubmapPtr input,bool search=true);
  void invalidate(const std::string& reason="alignment_invalidated");
  void invalidateRobot(int robot,const std::string& reason);
  void setDirectory(const std::string& directory);
  void prime(const std::vector<SubmapPtr>& inputs);
  size_t retryHistory();
  std::optional<std::pair<MatchEvent,SubmapPtr>> pollHistory();
  size_t databaseSize() const;
  const std::map<std::string,uint64_t>& counters() const;
  const SessionMap& map() const;
  std::optional<MatchEvent> observeScans(SubmapPtr reference,SubmapPtr query,double now);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
