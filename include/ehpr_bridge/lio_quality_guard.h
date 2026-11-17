#pragma once
#include <array>
#include <cmath>
#include <map>
#include <string>
namespace ehpr_bridge {
// Diagnostic-only inputs: no truth, inter-robot position or display TF.
class LioQualityGuard {
 public:
  struct Sample { double stamp=0; bool optimized=false,degenerate=true; double eigen=0; };
  bool add(int robot,Sample s) {
    auto& r=robots_.at(robot);
    if(!std::isfinite(s.stamp))return false;
    if(!r.samples.empty() && s.stamp<r.samples.rbegin()->first)r.quarantined=true;
    if(r.samples.empty())r.first=s.stamp;
    const bool good=s.optimized && !s.degenerate && std::isfinite(s.eigen);
    if(good)r.bad_start=-1;
    else if(s.stamp>=r.first+15) {
      if(r.bad_start<0)r.bad_start=s.stamp;
      if(s.stamp-r.bad_start>=2)r.quarantined=true;
    }
    r.samples[s.stamp]=s;while(r.samples.size()>4096)r.samples.erase(r.samples.begin());
    return r.quarantined;
  }
  std::string reason(int robot,double key_stamp,double now) const {
    const auto& r=robots_.at(robot);
    if(r.quarantined)return "LIO quality quarantine: continuous degradation >=2 s; restart the full LIO/submap run";
    if(r.samples.empty() || now-r.samples.rbegin()->first>1 || now<r.samples.rbegin()->first-.05)
      return "missing/stale LIO scan diagnostics";
    const auto& latest=r.samples.rbegin()->second;
    if(!latest.optimized || latest.degenerate || !std::isfinite(latest.eigen))return "latest LIO scan is unoptimized/degenerate/invalid";
    auto it=r.samples.lower_bound(key_stamp-.001);
    if(it==r.samples.end() || std::abs(it->first-key_stamp)>.001)return "missing LIO keyframe scan diagnostics";
    if(!it->second.optimized || it->second.degenerate || !std::isfinite(it->second.eigen))return "submap keyframe LIO scan is unoptimized/degenerate/invalid";
    return {};
  }
  bool quarantined(int robot) const{return robots_.at(robot).quarantined;}
  std::string currentReason(int robot,double now) const {
    const auto& r=robots_.at(robot);
    if(r.quarantined)return "LIO quality quarantine";
    if(r.samples.empty()||now-r.samples.rbegin()->first>1||now<r.samples.rbegin()->first-.05)return "missing/stale LIO scan diagnostics";
    const auto& s=r.samples.rbegin()->second;
    if(!s.optimized||s.degenerate||!std::isfinite(s.eigen))return "latest LIO scan is unoptimized/degenerate/invalid";
    return {};
  }
 private:
  struct Robot {std::map<double,Sample> samples;double first=0,bad_start=-1;bool quarantined=false;};
  std::array<Robot,2> robots_;
};
}
