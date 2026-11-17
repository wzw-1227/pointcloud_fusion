#include <ehpr_bridge/alignment_tracker.h>
#include <algorithm>
#include <cmath>

namespace ehpr_bridge {
namespace {
SubmapPtr role(const MatchResult& m,int r){return r==0?m.reference:m.query;}
bool disjoint(const SubmapInput& a,const SubmapInput& b){return a.recent_start_stamp>b.recent_end_stamp+1e-6||b.recent_start_stamp>a.recent_end_stamp+1e-6;}
}
AlignmentTracker::Evidence AlignmentTracker::evidence(const MatchResult& m) const {
  Evidence out;out.match=m;out.match.confirmation_supports.clear();
  for(int r=0;r<2;++r){auto p=role(m,r);if(!p->visibility_scan)continue;const auto& scan=*p->visibility_scan;
    for(size_t i=0;i<scan.points->size();++i)if(scan.structure[i]){Eigen::Vector3d x=p->pose*(*scan.points)[i].getVector3fMap().cast<double>();
      out.cells[r].insert({int64_t(std::floor(x.x()/.5)),int64_t(std::floor(x.y()/.5)),int64_t(std::floor(x.z()/.5))});}}
  return out;
}
bool AlignmentTracker::independent(const Evidence& c,const std::vector<const Evidence*>& witnesses) const {
  if(witnesses.empty())return true;
  for(int r=0;r<2;++r){auto p=role(c.match,r);Eigen::Vector3d origin=p->pose*p->recent_center_local;
    bool separate=true;std::set<Cell> old;
    for(const auto* evidence:witnesses){const auto& e=*evidence;auto previous=role(e.match,r);
      if(previous->session!=p->session||previous->epoch!=p->epoch)return false;
      separate=separate&&disjoint(*p,*previous)&&(origin-previous->pose*previous->recent_center_local).norm()>=config_.alignment_baseline_m;
      old.insert(e.cells[r].begin(),e.cells[r].end());}
    size_t added=0;for(const auto& cell:c.cells[r])added+=!old.count(cell);
    if(separate&&!c.cells[r].empty()&&double(added)/c.cells[r].size()>=config_.alignment_structure_novelty)return true;
  }return false;
}
std::string AlignmentTracker::state() const {return maps_.alignment().valid?"confirmed":(recovering_?"relocalizing":"waiting_confirmation");}
size_t AlignmentTracker::observations() const {size_t n=0;for(const auto& h:hypotheses_)n=std::max(n,h.evidence.size());return maps_.alignment().valid?confirmed_.size():n;}
void AlignmentTracker::diagnostics(MatchEvent& e) const {
  e.diagnostics["alignment_state_code"]=maps_.alignment().valid?2:(recovering_?3:1);
  e.diagnostics["independent_observations"]=observations();
  e.diagnostics["required_observations"]=(recovering_||(maps_.alignment().valid&&!hypotheses_.empty()))?config_.alignment_recovery_observations:config_.alignment_initial_observations;
  e.diagnostics["alignment_hypotheses"]=hypotheses_.size();e.diagnostics["alignment_conflict_streak"]=conflict_streak_;
  const auto* candidate=e.match?&*e.match:(e.attempted?&*e.attempted:nullptr);
  if(candidate){const auto& v=candidate->visibility;
    e.diagnostics["visibility_single_scan_source"]=v.available;e.diagnostics["visibility_evidence_sufficient"]=v.sufficient;
    e.diagnostics["visibility_conflict"]=v.conflict;e.diagnostics["visibility_reference_stamp"]=v.reference_stamp;e.diagnostics["visibility_query_stamp"]=v.query_stamp;
    for(const auto& direction:{std::make_pair("ugv_in_uav_",v.ugv_in_uav),std::make_pair("uav_in_ugv_",v.uav_in_ugv)}){
      const auto prefix=std::string("visibility_")+direction.first;const auto& d=direction.second;
      e.diagnostics[prefix+"checked"]=d.checked;e.diagnostics[prefix+"conflicts"]=d.conflicts;
      e.diagnostics[prefix+"structure_checked"]=d.structure_checked;e.diagnostics[prefix+"structure_conflicts"]=d.structure_conflicts;}}
}
void AlignmentTracker::reset(){recovering_=false;hypotheses_.clear();confirmed_.clear();conflict_streak_=0;
  last_monitor_stamp_=last_reference_stamp_=last_query_stamp_=-1;confirmed_anchor_.setIdentity();}
void AlignmentTracker::revoke(const std::string& reason){maps_.invalidateAlignment(reason);recovering_=true;hypotheses_.clear();confirmed_.clear();conflict_streak_=0;}
void AlignmentTracker::consider(MatchEvent& event,const Revalidate& revalidate) {
  if(!event.match){diagnostics(event);return;}
  auto m=*event.match;event.match.reset();event.attempted=m;
  if(!m.visibility.sufficient||m.visibility.conflict){event.status="waiting_visibility_evidence";event.reason=m.visibility.reason;diagnostics(event);return;}
  const auto transform=mapTransform(m);const bool active=maps_.alignment().valid;
  const bool nearby=active&&closeAlignment(confirmed_anchor_,transform,config_);
  if(nearby){
    // Fixed anchor and all original witnesses prevent a chain of small erroneous updates.
    std::vector<std::shared_ptr<const MatchResult>> witnesses;
    bool valid=true;for(const auto& e:confirmed_){auto trial=e.match;if(!revalidate(trial,transform).empty()){valid=false;break;}
      witnesses.push_back(std::make_shared<MatchResult>(std::move(trial)));}
    if(valid){m.independent_observations=confirmed_.size();m.required_observations=recovering_?config_.alignment_recovery_observations:config_.alignment_initial_observations;
      m.alignment_state="confirmed";m.confirmation_supports=std::move(witnesses);
      if(maps_.accept(m)){event.match=m;event.status="matched";event.reason="confirmed alignment updated";diagnostics(event);return;}}
  }
  auto it=std::find_if(hypotheses_.begin(),hypotheses_.end(),[&](const auto& h){return closeAlignment(h.anchor,transform,config_);});
  if(it==hypotheses_.end()){
    if(hypotheses_.size()>=size_t(config_.max_candidates))hypotheses_.erase(hypotheses_.begin());
    hypotheses_.push_back({transform,{}});it=std::prev(hypotheses_.end());}
  auto fresh=evidence(m);
  auto duplicate=std::find_if(it->pool.begin(),it->pool.end(),[&](const auto& e){return
    e.match.reference->recent_end_stamp==m.reference->recent_end_stamp&&e.match.query->recent_end_stamp==m.query->recent_end_stamp;});
  if(duplicate!=it->pool.end()){
    event.status="waiting_confirmation";event.reason="observation pair already considered";diagnostics(event);return;
  }
  // Unselected candidates are not votes. Keep enough alternatives so an early
  // broad scan cannot permanently prevent two later independent views confirming.
  constexpr size_t pool_capacity=16;
  if(it->pool.size()>=pool_capacity)it->pool.erase(it->pool.begin());
  it->pool.push_back(std::move(fresh));it->evidence.clear();
  const size_t required=(recovering_||active)?config_.alignment_recovery_observations:config_.alignment_initial_observations;
  std::vector<const Evidence*> selected;std::vector<Evidence> verified;MatchResult proposed;
  size_t deepest=0,checked_quorums=0;std::string failure="needs nonoverlapping scan window, motion baseline and novel structure";
  std::function<bool(size_t)> search=[&](size_t begin){
    if(selected.size()>deepest){deepest=selected.size();it->evidence.clear();for(auto* e:selected)it->evidence.push_back(*e);}
    if(selected.size()==required){
      if(++checked_quorums>32)return false;
      auto best=*std::max_element(selected.begin(),selected.end(),[](const auto* a,const auto* b){return a->match.candidate_score<b->match.candidate_score;});
      auto candidate=best->match;const auto pose=mapTransform(candidate);std::vector<Evidence> witnesses;witnesses.reserve(required);
      for(const auto* e:selected){auto trial=e->match;if(!closeAlignment(it->anchor,mapTransform(trial),config_)||!closeAlignment(pose,mapTransform(trial),config_)||!revalidate(trial,pose).empty()){
          failure="representative fails a confirmation observation";return false;}
        auto witness=evidence(trial);std::vector<const Evidence*> previous;for(const auto& prior:witnesses)previous.push_back(&prior);
        if(!independent(witness,previous)){failure="confirmation independence changed during fixed-pose validation";return false;}
        witnesses.push_back(std::move(witness));}
      for(const auto& alternative:hypotheses_)if(&alternative!=&*it&&!alternative.pool.empty()){
        bool explains=true;for(const auto& e:witnesses){auto trial=e.match;if(!revalidate(trial,alternative.anchor).empty()){explains=false;break;}}
        if(explains){failure="competing map relation still explains confirmation observations";return false;}}
      if(active){size_t failures=0;for(const auto& e:witnesses){auto trial=e.match;failures+=!revalidate(trial,maps_.alignment().transform).empty();}
        if(failures<2){failure="old and new relations remain ambiguous";return false;}}
      proposed=std::move(candidate);verified=std::move(witnesses);return true;
    }
    if(checked_quorums>=32)return false;
    for(size_t i=begin;i<it->pool.size();++i)if(independent(it->pool[i],selected)){
      selected.push_back(&it->pool[i]);if(search(i+1))return true;selected.pop_back();
    }return false;
  };
  const bool ready=search(0);
  event.attempted->independent_observations=it->evidence.size();event.attempted->required_observations=required;event.attempted->alignment_state=state();
  event.diagnostics["confirmation_pool_size"]=it->pool.size();event.diagnostics["confirmation_quorums_checked"]=checked_quorums;
  if(!ready){event.status="waiting_confirmation";event.reason=it->pool.size()==1?"waiting for independent observations":failure;diagnostics(event);return;}
  const auto final_pose=mapTransform(proposed);
  if(active){
    revoke("independent observations invalidate old map alignment");event.clear_display=true;
    event.revocation_reason=maps_.alignment().reason;event.revoked_alignment_revision=maps_.alignment().revision;
  }
  proposed.confirmation_supports.clear();double lo=1e100,hi=-1e100;
  for(const auto& e:verified){proposed.confirmation_supports.push_back(std::make_shared<MatchResult>(e.match));
    double t=std::max(e.match.reference->recent_end_stamp,e.match.query->recent_end_stamp);lo=std::min(lo,t);hi=std::max(hi,t);}
  proposed.independent_observations=verified.size();proposed.required_observations=required;proposed.alignment_state="confirmed";proposed.confirmation_wait_seconds=hi-lo;
  maps_.accept(proposed,true);confirmed_anchor_=final_pose;confirmed_=std::move(verified);hypotheses_.clear();
  event.match=proposed;event.status="matched";event.reason=recovering_?"alignment recovered from independent observations":"alignment confirmed by independent observations";
  event.diagnostics["confirmation_wait_sim_s"]=proposed.confirmation_wait_seconds;diagnostics(event);
}
std::optional<MatchEvent> AlignmentTracker::observe(SubmapPtr r,SubmapPtr q,double now) {
  if(!maps_.alignment().valid||!r||!q||!std::isfinite(now))return {};
  double a=r->recent_end_stamp,b=q->recent_end_stamp,t=std::min(a,b);
  if(std::abs(a-b)>.1+1e-6||now-a>1||now-b>1||now<a-.05||now<b-.05||
     a<=last_reference_stamp_||b<=last_query_stamp_||t<last_monitor_stamp_+.5-1e-6)return {};
  last_monitor_stamp_=t;last_reference_stamp_=a;last_query_stamp_=b;
  auto report=checkVisibility(*r,*q,q->pose.inverse()*maps_.alignment().transform.inverse()*r->pose,config_);
  MatchEvent event;event.status="alignment_checked";event.reason=report.reason;
  auto add=[&](const std::string& prefix,const VisibilityDirection& d){event.diagnostics[prefix+"checked"]=d.checked;event.diagnostics[prefix+"conflicts"]=d.conflicts;
    event.diagnostics[prefix+"structure_checked"]=d.structure_checked;event.diagnostics[prefix+"structure_conflicts"]=d.structure_conflicts;};
  add("monitor_ugv_in_uav_",report.ugv_in_uav);add("monitor_uav_in_ugv_",report.uav_in_ugv);
  event.diagnostics["monitor_reference_stamp"]=a;event.diagnostics["monitor_query_stamp"]=b;
  event.diagnostics["monitor_single_scan_source"]=1;event.diagnostics["monitor_evidence_sufficient"]=report.sufficient;
  if(!report.sufficient){conflict_streak_=0;event.status="alignment_insufficient_evidence";event.reason="insufficient bidirectional live scan evidence";}
  else if(report.conflict){++conflict_streak_;event.status="alignment_suspect";
    if(conflict_streak_>=2){revoke("consecutive bidirectional single-scan conflicts");event.status="alignment_revoked";event.reason=maps_.alignment().reason;event.clear_display=true;
      event.revocation_reason=event.reason;event.revoked_alignment_revision=maps_.alignment().revision;}}
  else conflict_streak_=0;
  diagnostics(event);return event;
}
}
