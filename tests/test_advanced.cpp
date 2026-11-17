#include <gtest/gtest.h>
#include <ehpr_bridge/advanced_matcher.h>
#include <ehpr_bridge/matching_io.h>
#include <ehpr_bridge/recent_cloud_buffer.h>
#include <pcl/io/pcd_io.h>
#include <cstdlib>

using namespace ehpr_bridge;
namespace {
Cloud::Ptr geometry(bool second=true,bool floor=true) {
  Cloud::Ptr out(new Cloud);
  if(floor)for(float x=-5;x<=7;x+=.2f)for(float y=-4;y<=6;y+=.2f)out->push_back(pcl::PointXYZ(x,y,-.35));
  for(float z=.1;z<2.2;z+=.15f){
    for(float x=-5;x<=7;x+=.15f)out->push_back(pcl::PointXYZ(x,6,z));
    if(second)for(float y=-4;y<=6;y+=.15f)out->push_back(pcl::PointXYZ(7,y,z));
  }return out;
}
std::shared_ptr<SubmapInput> input(int robot,Cloud::Ptr cloud,uint32_t id=0) {
  auto out=std::make_shared<SubmapInput>();out->robot=robot;out->session="advanced_fixture";out->id=id;
  out->frame=robot?"uav_lio_odom":"odom";out->local_frame="key_"+std::to_string(robot)+"_"+std::to_string(id);
  out->cloud=cloud;out->stamp=id+1;out->matching_gravity=Eigen::Quaterniond::Identity();return out;
}
std::shared_ptr<SubmapInput> high(bool second=true) {
  auto out=input(1,geometry(second));out->pose.translation().z()=2.45;
  pcl::transformPointCloud(*out->cloud,*out->cloud,Eigen::Affine3d(out->pose.inverse()));
  out->recent_cloud.reset(new Cloud);
  for(const auto& p:*out->cloud)if(p.z>-.8&&p.z<-.25)out->recent_cloud->push_back(p);
  out->recent_frames=50;out->recent_start_stamp=1;out->recent_end_stamp=6;return out;
}
}
TEST(AdvancedGround, HistoricalCloudProvidesHeightAfterFloorLeavesRecentWindow) {
  auto q=high();auto ground=estimateHistoricalGround(*q);ASSERT_TRUE(ground.valid);
  EXPECT_NEAR(ground.height,-.35,.01);EXPECT_GE(ground.support,100u);EXPECT_LE(ground.rmse,.03);
  MatcherConfig c;c.matching_profile="partial_contour";
  auto contour=prepareContour(*q,ground,c);EXPECT_NEAR(contour->height_above_ground,2.8,.01);
}
TEST(AdvancedGround, NarrowWallTopAndMultipleGroundLevelsAreRejected) {
  auto thin=input(0,Cloud::Ptr(new Cloud));
  for(float x=-6;x<6;x+=.1f)for(float y=0;y<.15;y+=.05f)thin->cloud->push_back(pcl::PointXYZ(x,y,-.35));
  EXPECT_FALSE(estimateHistoricalGround(*thin).valid);
  auto levels=input(0,geometry());auto second=geometry(false);
  for(auto& p:*second)if(p.z<0){p.z=-.8;levels->cloud->push_back(p);}
  auto ground=estimateHistoricalGround(*levels);EXPECT_TRUE(ground.conflict);EXPECT_FALSE(ground.valid);
}
TEST(AdvancedContour, DisjointVerticalBandsRecoverXYAndHistoricalHeight) {
  auto r=input(0,geometry());r->recent_cloud.reset(new Cloud);
  for(const auto& p:*r->cloud)if(p.z>.05&&p.z<1.5)r->recent_cloud->push_back(p);
  auto q=high();auto gr=estimateHistoricalGround(*r),gq=estimateHistoricalGround(*q);
  MatcherConfig c;c.matching_profile="partial_contour";MatchResult m;m.reference=r;m.query=q;
  m.coarse=q->pose.inverse();m.coarse.translation().x()+=.2;m.coarse.translation().y()-=.15;
  auto reason=refineContour(*prepareContour(*r,gr,c),*prepareContour(*q,gq,c),gr,gq,c,m);
  ASSERT_TRUE(reason.empty())<<reason<<"; pairs="<<m.contour_correspondences<<"; coverage="<<m.contour_overlap_reference<<","<<m.contour_overlap_query;
  EXPECT_LT((m.refined.translation()-q->pose.inverse().translation()).norm(),.10);
  EXPECT_GE(m.contour_correspondences,60u);EXPECT_LE(m.contour_residual_rmse,.20);
}
TEST(AdvancedContour, SingleWallHasNoLongitudinalConstraint) {
  auto r=input(0,geometry(false));r->recent_cloud=r->cloud;auto q=high(false);
  auto gr=estimateHistoricalGround(*r),gq=estimateHistoricalGround(*q);MatcherConfig c;
  MatchResult m;m.reference=r;m.query=q;m.coarse=q->pose.inverse();
  auto reason=refineContour(*prepareContour(*r,gr,c),*prepareContour(*q,gq,c),gr,gq,c,m);
  EXPECT_FALSE(reason.empty());EXPECT_LT(m.contour_information_ratio,1e-4);
}
TEST(AdvancedContour, DifferentMapOriginHeightsDoNotBecomeAnOdometryHeightDifference) {
  auto r=input(0,geometry());r->recent_cloud.reset(new Cloud);
  for(const auto& p:*r->cloud)if(p.z>.05&&p.z<1.5)r->recent_cloud->push_back(p);
  auto q=input(1,geometry());q->pose.translation().z()=1.75;
  Eigen::Isometry3d offset=Eigen::Isometry3d::Identity();offset.translation().z()=-2.45;
  pcl::transformPointCloud(*q->cloud,*q->cloud,Eigen::Affine3d(offset));q->recent_cloud.reset(new Cloud);
  for(const auto& p:*q->cloud)if(p.z>-.8&&p.z<-.25)q->recent_cloud->push_back(p);
  auto gr=estimateHistoricalGround(*r),gq=estimateHistoricalGround(*q);ASSERT_TRUE(gr.valid);ASSERT_TRUE(gq.valid);
  EXPECT_NEAR(gr.height,-.35,.01);EXPECT_NEAR(gq.height,-1.05,.01);
  MatcherConfig c;MatchResult m;m.reference=r;m.query=q;m.coarse.translation().z()=-1.75;
  ASSERT_TRUE(refineContour(*prepareContour(*r,gr,c),*prepareContour(*q,gq,c),gr,gq,c,m).empty());
  // Physical sensor heights are 0.35 and 2.80 m; their own odometry readings
  // are 0 and 1.75 m. A direct subtraction would be wrong by 0.70 m.
  EXPECT_NEAR(m.refined.translation().z(),.35-2.80,.01);
  SessionMap map;map.add(r);map.add(q);ASSERT_TRUE(map.accept(m));
  EXPECT_NEAR(map.alignment().transform.translation().z(),.70,.01);EXPECT_GT(map.fused().shared_voxels,100u);
}
TEST(SessionMap, UsesMapTransformAndKeepsAllReceivedRegions) {
  SessionMap map;auto r=input(0,geometry());auto q=input(1,geometry());
  Eigen::Isometry3d expected=Eigen::Isometry3d::Identity();expected.translation()<<2,-3,.2;
  pcl::transformPointCloud(*q->cloud,*q->cloud,Eigen::Affine3d(expected.inverse()));
  ASSERT_TRUE(map.add(r));ASSERT_TRUE(map.add(q));auto revision=map.revision();EXPECT_FALSE(map.add(q));EXPECT_EQ(map.revision(),revision);
  MatchResult m;m.reference=r;m.query=q;m.refined=expected.inverse();ASSERT_TRUE(map.accept(m));
  EXPECT_LT((map.alignment().transform.matrix()-expected.matrix()).norm(),1e-10);
  auto fused=map.fused();EXPECT_GT(fused.shared_voxels,100u);
  auto remote=input(0,Cloud::Ptr(new Cloud),1);remote->cloud->push_back(pcl::PointXYZ(120,4,1));map.add(remote);
  auto local=input(0,Cloud::Ptr(new Cloud),2);local->cloud->push_back(pcl::PointXYZ(0,0,1));map.add(local);
  bool kept=false;for(const auto& p:map.cloud(0))kept|=p.x>119;EXPECT_TRUE(kept);
}
TEST(SessionMap, ConsistentUpdatesReprojectOriginalMapsAndConflictPreservesAlignment) {
  SessionMap map;auto r=input(0,geometry()),q=input(1,geometry());map.add(r);map.add(q);
  MatchResult m;m.reference=r;m.query=q;ASSERT_TRUE(map.accept(m));auto count=map.fused().cloud.size();
  auto q2=std::make_shared<SubmapInput>(*q);q2->id=1;m.query=q2;m.refined.translation().x()=-.1;
  ASSERT_TRUE(map.accept(m));EXPECT_NEAR(map.alignment().transform.translation().x(),.1,1e-10);
  auto direct=SessionMap();direct.add(r);direct.add(q);ASSERT_TRUE(direct.accept(m));
  EXPECT_EQ(map.fused().cloud.size(),direct.fused().cloud.size());EXPECT_EQ(map.size(1),direct.size(1));
  auto before=map.alignment().transform;m.refined.translation().x()=-4;
  EXPECT_FALSE(map.accept(m));EXPECT_LT((map.alignment().transform.matrix()-before.matrix()).norm(),1e-10);
  map.reset();EXPECT_FALSE(map.alignment().valid);EXPECT_TRUE(map.fused().cloud.empty());EXPECT_EQ(map.size(0),0u);
}
TEST(AdvancedHistory, GeometryCacheEvictionDoesNotDeleteDiskHistory) {
  MatcherConfig c;c.matching_profile="partial_contour";c.history_geometry_capacity=2;
  Matcher matcher(c);std::vector<SubmapPtr> inputs;
  for(int i=0;i<5;++i){auto p=input(0,geometry(),i);p->pose.translation().x()=2*i;inputs.push_back(p);}
  matcher.prime(inputs);EXPECT_EQ(matcher.databaseSize(),5u);EXPECT_GE(matcher.counters().at("history_geometry_evictions"),3u);
  auto task=matcher.pollHistory();ASSERT_TRUE(task);EXPECT_GE(matcher.counters().at("history_disk_loads"),1u);
}
TEST(AdvancedHistory, SessionQualityAndWindowClockAreIndependentOfKeyframeAnchors) {
  MatcherConfig c;c.matching_profile="partial_contour";AdvancedMatcher matcher(c);
  auto a=input(0,geometry());a->stamp=10;a->recent_cloud=a->cloud;a->recent_frames=50;a->recent_start_stamp=6;a->recent_end_stamp=11;
  matcher.process(a,false);auto size=matcher.map().size(0);ASSERT_GT(size,0u);
  auto late=std::make_shared<SubmapInput>(*a);late->id=1;late->stamp=9;late->recent_end_stamp=12;
  matcher.process(late,false);EXPECT_EQ(matcher.map().size(0),size);EXPECT_EQ(matcher.counters().count("generation_resets"),0u);
  EXPECT_EQ(matcher.process(a,false).status,"duplicate");EXPECT_EQ(matcher.counters().count("generation_resets"),0u);
  auto bad=std::make_shared<SubmapInput>(*late);bad->id=2;bad->observation_quality_valid=false;
  EXPECT_EQ(matcher.process(bad,false).status,"rejected");EXPECT_EQ(matcher.map().size(0),size);
  auto rollback=std::make_shared<SubmapInput>(*late);rollback->id=3;rollback->recent_start_stamp=1;rollback->recent_end_stamp=5;
  matcher.process(rollback,false);EXPECT_EQ(matcher.counters().at("generation_resets"),1u);
  auto epoch=std::make_shared<SubmapInput>(*rollback);epoch->epoch=1;epoch->id=0;
  matcher.process(epoch,false);EXPECT_EQ(matcher.counters().at("generation_resets"),2u);EXPECT_FALSE(matcher.map().alignment().valid);
}
TEST(SessionMap, RepeatedPreviewsAndInputOrderChooseTheSameRealVoxelRepresentative) {
  SessionMap first,second;auto a=input(0,Cloud::Ptr(new Cloud));
  a->cloud->push_back(pcl::PointXYZ(.001,.002,.003));a->cloud->push_back(pcl::PointXYZ(.07,.08,.08));
  auto b=input(0,Cloud::Ptr(new Cloud),1);b->cloud->push_back(pcl::PointXYZ(.08,.07,.07));
  first.add(a);first.add(b);second.add(b);second.add(a);
  for(int i=2;i<100;++i){auto duplicate=std::make_shared<SubmapInput>(*a);duplicate->id=i;first.add(duplicate);}
  ASSERT_EQ(first.size(0),1u);ASSERT_EQ(second.size(0),1u);
  EXPECT_EQ(first.cloud(0)[0].getVector3fMap(),second.cloud(0)[0].getVector3fMap());
}
TEST(AdvancedHistory, OneRobotEpochResetPreservesTheUnchangedPeerSession) {
  MatcherConfig c;c.matching_profile="partial_contour";Matcher matcher(c);
  auto r=input(0,geometry()),q=input(1,geometry());q->cloud->push_back(pcl::PointXYZ(140,2,1));
  matcher.prime({r,q});auto peer=matcher.mapCloud(1);auto size=peer.size();ASSERT_GT(size,100u);
  auto next=input(0,geometry());next->epoch=1;matcher.archiveInput(next);
  EXPECT_FALSE(matcher.alignment()->valid);EXPECT_EQ(matcher.mapCloud(1).size(),size);
  bool kept=false;for(const auto& p:matcher.mapCloud(1))kept|=p.x>139;EXPECT_TRUE(kept);
  matcher.invalidateRobot(0,"registered_odometry_reset");EXPECT_TRUE(matcher.mapCloud(0).empty());EXPECT_EQ(matcher.mapCloud(1).size(),size);
  matcher.invalidate("time_rollback");EXPECT_TRUE(matcher.mapCloud(1).empty());
}
TEST(AdvancedContour, FloorOnlyAndRepeatedCorrespondencesCannotPass) {
  auto r=input(0,geometry());r->recent_cloud.reset(new Cloud);for(const auto& p:*r->cloud)if(p.z<0)r->recent_cloud->push_back(p);
  auto q=high();auto gr=estimateHistoricalGround(*r),gq=estimateHistoricalGround(*q);MatcherConfig c;
  MatchResult m;m.reference=r;m.query=q;m.coarse=q->pose.inverse();
  EXPECT_FALSE(refineContour(*prepareContour(*r,gr,c),*prepareContour(*q,gq,c),gr,gq,c,m).empty());
  r->recent_cloud.reset(new Cloud);for(int i=0;i<3000;++i)r->recent_cloud->push_back(pcl::PointXYZ(7,6,1));
  auto contour=prepareContour(*r,gr,c);EXPECT_LE(contour->levels[2].cloud->size(),1u);
  EXPECT_FALSE(refineContour(*contour,*prepareContour(*q,gq,c),gr,gq,c,m).empty());
}
TEST(RecentCloud, ClockRollbackAndPoseJumpExposeNewGenerations) {
  RecentCloudConfig config;config.min_frames=1;config.reset_on_timestamp_rollback=true;RecentCloudBuffer buffer(config);auto points=geometry();
  EXPECT_TRUE(buffer.add(points,Eigen::Isometry3d::Identity(),10));EXPECT_EQ(buffer.generation(),0u);
  EXPECT_TRUE(buffer.add(points,Eigen::Isometry3d::Identity(),5));EXPECT_EQ(buffer.generation(),1u);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation().x()=2;
  EXPECT_TRUE(buffer.add(points,pose,6));EXPECT_EQ(buffer.generation(),2u);
}
TEST(AdvancedHistory, WarmReceiptOrderPreservesEpochResetDespiteClockRollback) {
  char name[]="/tmp/stage3_restore_XXXXXX";ASSERT_NE(mkdtemp(name),nullptr);std::filesystem::path directory(name);
  for(int sequence=0;sequence<2;++sequence){auto p=input(0,geometry(false));p->epoch=sequence;p->stamp=sequence?1:100;
    p->pose.translation().x()=sequence?123:0;auto stem=directory/("ugv_submap_event_"+std::to_string(sequence+1));
    ASSERT_EQ(pcl::io::savePCDFileBinary(stem.string()+".pcd",*p->cloud),0);
    auto metadata=inputMetadata(*p);metadata["upstream_commit"]="b4b7137f7e675974e025fc3c8e0ab891d353739a";
    metadata["ingestion_sequence"]=sequence;metadata["segment_end_timestamp"]=p->stamp;saveYamlAtomic(stem.string()+".yaml",metadata);}
  auto restored=loadReplay(directory,true,true);ASSERT_EQ(restored.size(),2u);EXPECT_EQ(restored[0]->epoch,0u);EXPECT_EQ(restored[1]->epoch,1u);
  MatcherConfig c;c.matching_profile="partial_contour";Matcher matcher(c);matcher.prime(restored);
  EXPECT_FALSE(matcher.alignment()->valid);for(const auto& p:matcher.mapCloud(0))EXPECT_GT(p.x,100);
  std::filesystem::remove_all(directory);
}
TEST(RecentCloud, VisibilitySnapshotRetainsActualRaysInsteadOfVoxelCentroids) {
  RecentCloudConfig config;config.min_frames=1;RecentCloudBuffer buffer(config);Cloud::Ptr points(new Cloud);
  points->push_back(pcl::PointXYZ(1.01,2.01,3.01));points->push_back(pcl::PointXYZ(1.09,2.09,3.09));
  ASSERT_TRUE(buffer.add(points,Eigen::Isometry3d::Identity(),1));auto snapshot=buffer.snapshot(Eigen::Isometry3d::Identity(),1);ASSERT_TRUE(snapshot);
  EXPECT_EQ(snapshot->local->size(),1u);ASSERT_EQ(snapshot->latest_scan->size(),2u);
  EXPECT_EQ((*snapshot->latest_scan)[0].x,(*points)[0].x);EXPECT_EQ((*snapshot->latest_scan)[1].x,(*points)[1].x);
}
TEST(AdvancedHistory, NonfiniteSingleScanDoesNotEnterHistoryOrMap) {
  MatcherConfig c;c.matching_profile="partial_contour";Matcher matcher(c);auto p=input(0,geometry());
  p->latest_scan.reset(new Cloud);p->latest_scan->push_back(pcl::PointXYZ(NAN,1,1));
  EXPECT_EQ(matcher.process(p).reason,"nonfinite latest scan");EXPECT_EQ(matcher.databaseSize(),0u);EXPECT_TRUE(matcher.mapCloud(0).empty());
}
