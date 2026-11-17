#include <gtest/gtest.h>
#include <ehpr_bridge/alignment_tracker.h>
#include <ehpr_bridge/partial_registration.h>
#include <ehpr_bridge/matching_io.h>
#include <pcl/io/pcd_io.h>
#include <cstdlib>

using namespace ehpr_bridge;
namespace {
std::shared_ptr<SubmapInput> beam(int robot,double depth=5,double stamp=5) {
  auto p=std::make_shared<SubmapInput>();p->robot=robot;p->session="ray_fixture";p->id=uint32_t(stamp);
  p->frame=robot?"uav_lio_odom":"odom";p->local_frame="ray_key";p->stamp=stamp;
  p->recent_start_stamp=stamp-5;p->recent_end_stamp=stamp;p->recent_frames=50;
  p->matching_gravity=Eigen::Quaterniond::Identity();p->latest_scan.reset(new Cloud);
  for(double angle=-1;angle<=1;angle+=.015){Eigen::Vector3d ray(std::cos(angle),std::sin(angle),.15);ray.normalize();ray*=depth;
    p->latest_scan->push_back(pcl::PointXYZ(ray.x(),ray.y(),ray.z()));}
  p->cloud=p->recent_cloud=p->latest_scan;p->visibility_scan=prepareVisibilityScan(*p,-1,MatcherConfig{});return p;
}
MatchResult proposal(int phase,double x,double stamp) {
  MatchResult m;std::array<std::shared_ptr<SubmapInput>,2> p;
  for(int r=0;r<2;++r){p[r]=beam(r,5,stamp);p[r]->session="confirmation_fixture";p[r]->pose.translation().x()=r?0:x;
    p[r]->latest_scan.reset(new Cloud);
    for(double y=-3;y<=3;y+=.1)for(double z=.5;z<=2;z+=.1)
      p[r]->latest_scan->push_back(pcl::PointXYZ(6+phase*2-p[r]->pose.translation().x(),y,z));
    p[r]->cloud=p[r]->recent_cloud=p[r]->latest_scan;p[r]->visibility_scan=prepareVisibilityScan(*p[r],0,MatcherConfig{});}
  m.reference=p[0];m.query=p[1];m.refined=p[1]->pose.inverse()*p[0]->pose;m.candidate_score=.9;
  validateVisibility(m,MatcherConfig{});return m;
}
std::string knownScene(MatchResult& m,const Eigen::Isometry3d& map){
  if(map.translation().norm()>.2||Eigen::AngleAxisd(map.rotation()).angle()>.03)return "wrong scene geometry";
  m.refined=m.query->pose.inverse()*map.inverse()*m.reference->pose;
  auto reason=validateVisibility(m,MatcherConfig{});if(reason.empty()&&!m.visibility.sufficient)reason="insufficient visibility evidence";return reason;
}
MatchEvent offer(AlignmentTracker& tracker,MatchResult m){MatchEvent e;e.match=std::move(m);tracker.consider(e,knownScene);return e;}
}
TEST(Visibility, ReverseConflictAndOccludedDirectionAreSeparate) {
  MatcherConfig c;auto r=beam(0,10),q=beam(1,5);auto report=checkVisibility(*r,*q,Eigen::Isometry3d::Identity(),c);
  ASSERT_TRUE(report.sufficient);EXPECT_EQ(report.ugv_in_uav.conflicts,0u);EXPECT_GT(report.uav_in_ugv.conflicts,50u);EXPECT_TRUE(report.conflict);
  auto same=beam(1,10);auto good=checkVisibility(*r,*same,Eigen::Isometry3d::Identity(),c);
  EXPECT_TRUE(good.sufficient);EXPECT_FALSE(good.conflict);
}
TEST(Visibility, DuplicateReturnsCannotManufactureIndependentEvidence) {
  auto p=beam(0);auto original=p->visibility_scan->points->size();auto once=*p->latest_scan;
  for(int n=0;n<100;++n)*p->latest_scan+=once;
  p->visibility_scan=prepareVisibilityScan(*p,-1,MatcherConfig{});EXPECT_EQ(p->visibility_scan->points->size(),original);
  auto q=beam(1);auto report=checkVisibility(*p,*q,Eigen::Isometry3d::Identity(),MatcherConfig{});
  EXPECT_LE(report.ugv_in_uav.checked,original);EXPECT_LE(report.uav_in_ugv.checked,original);
}
TEST(Visibility, MissingScansAndUnknownDirectionsStayInsufficient) {
  auto r=beam(0),q=beam(1);q->latest_scan.reset();q->visibility_scan=prepareVisibilityScan(*q,-1,MatcherConfig{});
  EXPECT_FALSE(checkVisibility(*r,*q,Eigen::Isometry3d::Identity(),MatcherConfig{}).sufficient);
  q=beam(1);Eigen::Isometry3d rotation=Eigen::Isometry3d::Identity();rotation.linear()=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto unknown=checkVisibility(*r,*q,rotation,MatcherConfig{});EXPECT_FALSE(unknown.sufficient);EXPECT_FALSE(unknown.conflict);
}
TEST(Visibility, NonGroundConflictsCannotBeDilutedByFloor) {
  auto r=beam(0,5),q=beam(1,10);Cloud floor;
  for(double azimuth=-M_PI;azimuth<M_PI;azimuth+=M_PI/180)for(double elevation=-.9;elevation<-.1;elevation+=.05){
    const double radius=-.8/std::tan(elevation);floor.push_back(pcl::PointXYZ(radius*std::cos(azimuth),radius*std::sin(azimuth),-.8));}
  *r->latest_scan+=floor;*q->latest_scan+=floor;
  r->visibility_scan=prepareVisibilityScan(*r,-1,MatcherConfig{});q->visibility_scan=prepareVisibilityScan(*q,-1,MatcherConfig{});
  auto report=checkVisibility(*r,*q,Eigen::Isometry3d::Identity(),MatcherConfig{});
  ASSERT_TRUE(report.sufficient);
  EXPECT_LE(report.ugv_in_uav.conflicts,.05*report.ugv_in_uav.checked);
  EXPECT_GT(report.ugv_in_uav.structure_conflicts,.05*report.ugv_in_uav.structure_checked);
  EXPECT_TRUE(report.conflict);EXPECT_GT(report.ugv_in_uav.structure_conflicts,20u);
}
TEST(Visibility, AccumulatedCloudCannotCreateOrHideSingleScanConflicts) {
  auto r=beam(0,10),q=beam(1,10);
  r->recent_cloud=beam(0,5)->latest_scan;
  r->visibility_scan=prepareVisibilityScan(*r,-1,MatcherConfig{});
  auto report=checkVisibility(*r,*q,Eigen::Isometry3d::Identity(),MatcherConfig{});
  ASSERT_TRUE(report.sufficient);EXPECT_FALSE(report.conflict);
  r->latest_scan=beam(0,5)->latest_scan;r->recent_cloud=q->latest_scan;
  r->visibility_scan=prepareVisibilityScan(*r,-1,MatcherConfig{});
  report=checkVisibility(*r,*q,Eigen::Isometry3d::Identity(),MatcherConfig{});
  EXPECT_TRUE(report.sufficient);EXPECT_TRUE(report.conflict);
}
TEST(Visibility, ActualOriginAndKeyTransformAreUsed) {
  auto m=proposal(0,.7,5);ASSERT_TRUE(m.visibility.sufficient);EXPECT_FALSE(m.visibility.conflict);
  auto r=std::make_shared<SubmapInput>(*m.reference);auto q=std::make_shared<SubmapInput>(*m.query);
  // Re-anchor the same scan without changing its actual acquisition origin.
  r->latest_scan.reset(new Cloud);pcl::transformPointCloud(*m.reference->latest_scan,*r->latest_scan,Eigen::Affine3d(m.reference->pose));
  r->pose.setIdentity();r->recent_center_local<<.7,0,0;r->visibility_scan=prepareVisibilityScan(*r,0,MatcherConfig{});
  auto report=checkVisibility(*r,*q,Eigen::Isometry3d::Identity(),MatcherConfig{});EXPECT_TRUE(report.sufficient);EXPECT_FALSE(report.conflict);
}
TEST(Visibility, RecordedGroundedUavFalseAlignmentFailsWhileTruthPasses) {
  const auto folder=std::filesystem::path(__FILE__).parent_path()/"data/alignment_false_match";
  auto inputs=loadReplay(folder);ASSERT_EQ(inputs.size(),2u);std::array<SubmapPtr,2> scans;
  for(auto original:inputs){auto p=std::make_shared<SubmapInput>(*original);p->matching_gravity=canonicalMatchingGravity(p->pose.rotation());
    auto geometry=preparePartialGeometry(p->recent_cloud,*p->matching_gravity,(*p->matching_gravity)*p->recent_center_local);
    ASSERT_FALSE(geometry->horizontal_planes.empty());double floor=geometry->horizontal_planes.front().height;
    for(auto plane:geometry->horizontal_planes)floor=std::min(floor,plane.height);
    p->visibility_scan=prepareVisibilityScan(*p,floor,MatcherConfig{});scans[p->robot]=p;}
  Eigen::Isometry3d wrong;auto y=YAML::LoadFile((folder/"bad_seed.yaml").string())["accepted_wrong_transform"];
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)wrong.matrix()(i,j)=y[i][j].as<double>();
  auto report=checkVisibility(*scans[0],*scans[1],wrong,MatcherConfig{});
  ASSERT_TRUE(report.sufficient);EXPECT_TRUE(report.conflict);
  // Independent fixed map relation from the diagnostic, used only by this test.
  Eigen::Isometry3d truth=Eigen::Isometry3d::Identity();truth.linear()=Eigen::AngleAxisd(-1.1988835486e-5,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  truth.translation()<<-11.2000349276,-1.9998677177,-.0320174911;
  auto good=checkVisibility(*scans[0],*scans[1],scans[1]->pose.inverse()*truth.inverse()*scans[0]->pose,MatcherConfig{});
  EXPECT_TRUE(good.sufficient);EXPECT_FALSE(good.conflict);
}
TEST(AlignmentSafety, RepeatedAndOverlappingWindowsDoNotConfirm) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);auto first=proposal(0,0,5);ASSERT_TRUE(first.visibility.sufficient);
  EXPECT_FALSE(offer(tracker,first).match);EXPECT_EQ(tracker.observations(),1u);
  for(int n=0;n<5;++n){auto duplicate=first;auto r=std::make_shared<SubmapInput>(*first.reference);r->id+=n+1;duplicate.reference=r;EXPECT_FALSE(offer(tracker,duplicate).match);}
  EXPECT_EQ(tracker.observations(),1u);EXPECT_FALSE(map.alignment().valid);
  EXPECT_FALSE(offer(tracker,proposal(1,.7,8)).match);EXPECT_EQ(tracker.observations(),1u);
  auto confirmed=offer(tracker,proposal(1,.7,11));ASSERT_TRUE(confirmed.match)<<confirmed.reason;
  EXPECT_TRUE(map.alignment().valid);EXPECT_EQ(confirmed.match->independent_observations,2u);
}
TEST(AlignmentSafety, StationaryNewWindowsAndMotionWithoutNoveltyDoNotVote) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);offer(tracker,proposal(0,0,5));
  EXPECT_FALSE(offer(tracker,proposal(1,0,11)).match);
  EXPECT_EQ(tracker.observations(),1u);
  SessionMap another;AlignmentTracker other(c,another);offer(other,proposal(0,0,5));
  EXPECT_FALSE(offer(other,proposal(0,.7,17)).match);EXPECT_EQ(other.observations(),1u);
}
TEST(AlignmentSafety, LaterIndependentViewsCanReplaceUnconfirmedFirstWitness) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);offer(tracker,proposal(0,0,5));
  auto narrow=proposal(0,0,11);
  for(int role=0;role<2;++role){auto p=std::make_shared<SubmapInput>(*(role?narrow.query:narrow.reference));p->latest_scan.reset(new Cloud);
    for(const auto& point:*(role?narrow.query:narrow.reference)->latest_scan)if(point.y>0)p->latest_scan->push_back(point);
    p->cloud=p->recent_cloud=p->latest_scan;p->visibility_scan=prepareVisibilityScan(*p,0,c);
    if(role)narrow.query=p;else narrow.reference=p;}
  validateVisibility(narrow,c);ASSERT_TRUE(narrow.visibility.sufficient);EXPECT_FALSE(offer(tracker,narrow).match);
  auto later=offer(tracker,proposal(0,.7,17));ASSERT_TRUE(later.match)<<later.reason;
  EXPECT_EQ(later.match->independent_observations,2u);
  EXPECT_DOUBLE_EQ(later.match->confirmation_supports[0]->query->recent_end_stamp,11);
}
TEST(AlignmentSafety, OnlyNewCandidateWithVisibilityCanEstablishRelation) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);auto m=proposal(0,0,5);m.visibility.sufficient=false;
  auto event=offer(tracker,m);EXPECT_EQ(event.status,"waiting_visibility_evidence");EXPECT_FALSE(map.alignment().valid);EXPECT_EQ(tracker.observations(),0u);
}
TEST(AlignmentSafety, ConflictsRevokeWithoutDeletingMapsAndRecoveryRequiresThree) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);auto m=proposal(0,0,5);map.add(m.reference);map.add(m.query);
  auto wrong=m;wrong.refined.translation().x()=-2;ASSERT_TRUE(map.accept(wrong));const auto ugv=map.size(0),uav=map.size(1);
  auto sample=proposal(0,0,6);auto a=sample.reference,b=sample.query;auto one=tracker.observe(a,b,6);ASSERT_TRUE(one);EXPECT_FALSE(one->clear_display);
  ASSERT_EQ(one->diagnostics.at("alignment_conflict_streak"),1);
  auto duplicate=tracker.observe(a,b,6.2);EXPECT_FALSE(duplicate);
  sample=proposal(0,0,6.6);a=sample.reference;b=sample.query;auto two=tracker.observe(a,b,6.6);ASSERT_TRUE(two);EXPECT_TRUE(two->clear_display);
  EXPECT_FALSE(map.alignment().valid);EXPECT_EQ(tracker.state(),"relocalizing");EXPECT_TRUE(map.fused().cloud.empty());
  EXPECT_EQ(map.size(0),ugv);EXPECT_EQ(map.size(1),uav);
  EXPECT_FALSE(offer(tracker,proposal(0,0,15)).match);EXPECT_FALSE(offer(tracker,proposal(1,.7,21)).match);
  auto recovered=offer(tracker,proposal(2,1.4,27));ASSERT_TRUE(recovered.match)<<recovered.reason;
  EXPECT_EQ(recovered.match->independent_observations,3u);EXPECT_LT(map.alignment().transform.translation().norm(),1e-9);
  EXPECT_EQ(map.size(0),ugv);EXPECT_EQ(map.size(1),uav);EXPECT_FALSE(map.fused().cloud.empty());
}
TEST(AlignmentSafety, CorrectLargeCorrectionIsNotRejectedByOldAlignment) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);auto wrong=proposal(0,0,5);wrong.refined.translation().x()=-2;map.accept(wrong);
  EXPECT_FALSE(offer(tracker,proposal(0,0,5)).match);EXPECT_FALSE(offer(tracker,proposal(1,.7,11)).match);
  auto recovered=offer(tracker,proposal(2,1.4,17));ASSERT_TRUE(recovered.match)<<recovered.reason;EXPECT_TRUE(recovered.clear_display);
  EXPECT_EQ(recovered.revocation_reason,"independent observations invalidate old map alignment");
  EXPECT_LT(recovered.revoked_alignment_revision,map.alignment().revision);
  EXPECT_LT(map.alignment().transform.translation().norm(),1e-9);
}
TEST(AlignmentSafety, WeakOrStaleMonitorEvidenceCannotRevoke) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);map.accept(proposal(0,0,5));
  EXPECT_FALSE(tracker.observe(beam(0,5,5),beam(1,5,5),7));
  EXPECT_FALSE(tracker.observe(beam(0,5,8),beam(1,5,8.2),8.2));
  EXPECT_TRUE(map.alignment().valid);
}
TEST(AlignmentSafety, OneSidedConflictWithoutBidirectionalEvidenceCannotRevoke) {
  MatcherConfig c;auto scan=[](int robot,double stamp){auto p=beam(robot,robot?5:10,stamp);
    for(size_t i=0;i<p->latest_scan->size();++i){auto& point=(*p->latest_scan)[i];point.z=0;
      if(robot&&i%2){point.x*=3;point.y*=3;}}
    p->visibility_scan=prepareVisibilityScan(*p,-1,MatcherConfig{});return p;};
  auto r=scan(0,5),q=scan(1,5);
  Eigen::Isometry3d relation=Eigen::Isometry3d::Identity();relation.translation().x()=2;
  auto report=checkVisibility(*r,*q,relation,c);
  ASSERT_NE(report.ugv_in_uav.checked,report.uav_in_ugv.checked);
  // One direction has enough returns and contradicts this relation; the other
  // direction remains below the required evidence threshold.
  c.visibility_min_rays=std::max(report.ugv_in_uav.checked,report.uav_in_ugv.checked);
  c.visibility_min_structure_rays=1;
  report=checkVisibility(*r,*q,relation,c);
  ASSERT_FALSE(report.sufficient);ASSERT_TRUE(report.conflict);
  SessionMap map;MatchResult wrong;wrong.reference=r;wrong.query=q;wrong.refined=relation;ASSERT_TRUE(map.accept(wrong));
  AlignmentTracker tracker(c,map);
  for(double stamp:{6.,6.6}){auto event=tracker.observe(scan(0,stamp),scan(1,stamp),stamp);ASSERT_TRUE(event);
    EXPECT_EQ(event->status,"alignment_insufficient_evidence");EXPECT_FALSE(event->clear_display);}
  EXPECT_TRUE(map.alignment().valid);
}
TEST(AlignmentSafety, SmallUpdatesCannotWalkAwayFromConfirmedAnchor) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);offer(tracker,proposal(0,0,5));
  ASSERT_TRUE(offer(tracker,proposal(1,.7,11)).match);
  auto validate=[](MatchResult& m,const Eigen::Isometry3d& pose){m.refined=m.query->pose.inverse()*pose.inverse()*m.reference->pose;return std::string();};
  for(double shift:{.1,.2}){auto m=proposal(2,1.4,20+shift*10);Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation().x()=shift;
    m.refined=m.query->pose.inverse()*pose.inverse()*m.reference->pose;MatchEvent event;event.match=m;tracker.consider(event,validate);ASSERT_TRUE(event.match);}
  auto m=proposal(3,2.1,30);Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation().x()=.3;
  m.refined=m.query->pose.inverse()*pose.inverse()*m.reference->pose;MatchEvent event;event.match=m;tracker.consider(event,validate);
  EXPECT_FALSE(event.match);EXPECT_NEAR(map.alignment().transform.translation().x(),.2,1e-10);
}
TEST(AlignmentSafety, AmbiguousOldAndNewRelationsDoNotForceReplacement) {
  MatcherConfig c;SessionMap map;AlignmentTracker tracker(c,map);auto wrong=proposal(0,0,5);wrong.refined.translation().x()=-2;map.accept(wrong);
  for(int i=0;i<3;++i){MatchEvent event;event.match=proposal(i,i*.7,5+i*6);
    tracker.consider(event,[](MatchResult&,const Eigen::Isometry3d&){return std::string();});
    EXPECT_FALSE(event.match);if(i==2)EXPECT_EQ(event.reason,"old and new relations remain ambiguous");}
  EXPECT_NEAR(map.alignment().transform.translation().x(),2,1e-10);
}
TEST(AlignmentSafety, MapCheckpointPreservesNumericSessionIdentifiersAsStrings) {
  char temp[]="/tmp/stage3_session_tag_XXXXXX";ASSERT_NE(mkdtemp(temp),nullptr);std::filesystem::path folder(temp);
  auto config=folder/"config.yaml";saveYamlAtomic(config,YAML::Load("upstream_commit: b4b7137f7e675974e025fc3c8e0ab891d353739a"));
  MatcherConfig c;c.matching_profile="partial";Matcher matcher(c);auto m=proposal(0,0,5);
  auto r=std::make_shared<SubmapInput>(*m.reference),q=std::make_shared<SubmapInput>(*m.query);
  r->session=q->session="20261014_001000_000000_123";matcher.archiveInput(r);matcher.archiveInput(q);
  // Inject only the checkpoint state; this test does not publish a match or bypass a production interface.
  auto* state=const_cast<MapAlignmentState*>(matcher.alignment());state->valid=true;state->reference=r;state->query=q;
  {MatchWriter writer((folder/"run").string(),config.string(),"serialization_test");writer.checkpoint(matcher);
    auto summary=YAML::LoadFile((writer.directory()/"summary.yaml").string());auto meta=YAML::LoadFile((writer.directory()/summary["latest_map_checkpoint"].as<std::string>()).string());
    EXPECT_EQ(meta["reference_session_id"].Tag(),"tag:yaml.org,2002:str");EXPECT_EQ(meta["query_session_id"].Tag(),"tag:yaml.org,2002:str");
    EXPECT_EQ(meta["reference_session_id"].as<std::string>(),r->session);}
  std::filesystem::remove_all(folder);
}
