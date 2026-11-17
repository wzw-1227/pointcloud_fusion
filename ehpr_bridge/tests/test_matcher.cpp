#include <gtest/gtest.h>
#include <ehpr_bridge/matcher.h>
#include <ehpr_bridge/matching_io.h>
#include <ehpr_bridge/lio_quality_guard.h>
#include <ehpr_bridge/recent_cloud_buffer.h>
#include <pcl/io/pcd_io.h>
#include <ehpr/desc_db.h>
#include <ehpr/AlignRansac2D.h>
#include <random>
#include <limits>
using namespace ehpr_bridge;
TEST(RansacRotation, RigidReflectionCollinearAndDuplicateInputs) {
  const Eigen::Matrix2d rotation=Eigen::Rotation2Dd(.37).toRotationMatrix();
  std::vector<map_closures::PointPair> rigid,mirror,line,duplicate;
  for(int i=0;i<12;++i){Eigen::Vector2d p(i*3.,(i*i%17)*2.);
    rigid.emplace_back(p,rotation*p+Eigen::Vector2d(1,-2));
    mirror.emplace_back(p,Eigen::Vector2d(-p.x(),p.y()));
    line.emplace_back(Eigen::Vector2d(i*3.,0),Eigen::Vector2d(i*3.+1,0));
    duplicate.emplace_back(Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero());}
  const auto good=map_closures::RansacAlignment2D(rigid);
  EXPECT_GE(good.second.size(),7u);EXPECT_LT((good.first.linear()-rotation).norm(),1e-6);
  EXPECT_LT((good.first.translation()-Eigen::Vector2d(1,-2)).norm(),1e-6);
  for(const auto* points:{&rigid,&mirror,&line,&duplicate}){
    auto result=map_closures::RansacAlignment2D(*points);
    EXPECT_TRUE(result.first.matrix().allFinite());
    EXPECT_NEAR(result.first.linear().determinant(),1,1e-8);
    EXPECT_LT((result.first.linear().transpose()*result.first.linear()-Eigen::Matrix2d::Identity()).norm(),1e-8);
  }
}
TEST(RansacRotation, EnhancedCorrespondencesAreUniqueAndDeterministic) {
  std::vector<map_closures::PointPair> pairs;
  Eigen::Isometry2d expected=Eigen::Isometry2d::Identity();expected.linear()=Eigen::Rotation2Dd(.31).toRotationMatrix();expected.translation()<<2,-1;
  for(int i=0;i<15;++i){Eigen::Vector2d p(i*2.,(i*i%19)*1.5);
    for(int duplicate=0;duplicate<3;++duplicate)pairs.emplace_back(p,expected*p,i,i);
    // Outliers must not themselves form a second exact 15-point rigid model.
    pairs.emplace_back(p,Eigen::Vector2d(i*1.3+4,(i*i%11)*2.7+8),i,(i+3)%15);}
  const auto a=map_closures::RansacHypotheses2D(pairs,17),b=map_closures::RansacHypotheses2D(pairs,17);
  ASSERT_FALSE(a.empty());ASSERT_EQ(a.size(),b.size());EXPECT_EQ(a[0].inliers.size(),15u);
  EXPECT_LT((a[0].pose.matrix()-expected.matrix()).norm(),1e-6);
  for(size_t i=0;i<a.size();++i){EXPECT_EQ(a[i].inliers,b[i].inliers);EXPECT_EQ(a[i].pose.matrix(),b[i].pose.matrix());
    std::set<int> r,q;for(int j:a[i].inliers){r.insert(pairs[j].ref_idx);q.insert(pairs[j].qur_idx);}
    EXPECT_EQ(r.size(),a[i].inliers.size());EXPECT_EQ(q.size(),a[i].inliers.size());}
  std::vector<map_closures::PointPair> degenerate(20,map_closures::PointPair(Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero(),0,0));
  EXPECT_TRUE(map_closures::RansacHypotheses2D(degenerate,17).empty());
}
static Cloud::Ptr scene(int seed=42) {
  Cloud::Ptr c(new Cloud);std::mt19937 rng(seed);std::uniform_real_distribution<float> xy(-20,20),width(1,4),height(1,5);
  for(int k=0;k<32;++k) {
    float x=xy(rng),y=xy(rng),w=width(rng),d=width(rng),h=height(rng);
    for(float a=0;a<w;a+=.17)for(float z=0;z<h;z+=.17){c->push_back(pcl::PointXYZ(x+a,y,z));c->push_back(pcl::PointXYZ(x+a,y+d,z));}
    for(float a=0;a<d;a+=.17)for(float z=0;z<h;z+=.17){c->push_back(pcl::PointXYZ(x,y+a,z));c->push_back(pcl::PointXYZ(x+w,y+a,z));}
  }
  return c;
}
static std::shared_ptr<SubmapInput> input(int robot,Cloud::Ptr c,uint32_t id=0) {
  auto i=std::make_shared<SubmapInput>();i->robot=robot;i->cloud=c;i->id=id;i->session="fixture";i->stamp=id+1;
  i->frame=robot==0?"odom":"uav_lio_odom";i->local_frame=(robot==0?"ugv_":"uav_")+std::to_string(id);i->partial=true;return i;
}
TEST(Matching, KnownPoseGravityAndDisplayDirection) {
  Matcher matcher(MatcherConfig{});auto ref=input(0,scene());auto q=input(1,Cloud::Ptr(new Cloud));
  q->pose.linear()=(Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.08,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.05,Eigen::Vector3d::UnitX())).toRotationMatrix();
  q->pose.translation()=Eigen::Vector3d(1.2,-2.1,1.4);q->gravity=Eigen::AngleAxisd(.08,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.05,Eigen::Vector3d::UnitX());
  auto expected=q->pose.inverse();pcl::transformPointCloud(*ref->cloud,*q->cloud,Eigen::Affine3d(expected));
  EXPECT_EQ(matcher.process(ref).status,"database_updated");auto result=matcher.process(q);
  ASSERT_TRUE(result.match)<<result.reason;
  EXPECT_GT(result.diagnostics.at("query_valid_height_features"),0);
  EXPECT_GE(result.diagnostics.at("height_pairs"),7);
  EXPECT_EQ(result.diagnostics.at("height_alignment_valid"),1);
  EXPECT_LT((result.match->refined.translation()-expected.translation()).norm(),.2);
  EXPECT_LT(Eigen::AngleAxisd(result.match->refined.rotation().transpose()*expected.rotation()).angle()*180/M_PI,3);
  Cloud a,b;Matcher::displayClouds(*result.match,a,b);ASSERT_EQ(a.size(),b.size());
  double error=0;for(size_t k=0;k<a.size();++k)error+=(a[k].getVector3fMap()-b[k].getVector3fMap()).norm();
  EXPECT_LT(error/a.size(),.2);
  EXPECT_EQ(matcher.process(q).status,"duplicate");
  if(const char* path=std::getenv("STAGE3_FIXTURE_DIR")) {
    std::filesystem::create_directories(path);
    for(auto sample:{ref,q}) {
      auto file=std::filesystem::path(path)/(sample->robot==0?"ugv_submap_000":"uav_submap_000");
      pcl::io::savePCDFileBinary(file.string()+".pcd",*sample->cloud);
      auto metadata=inputMetadata(*sample);
      metadata["upstream_commit"]="b4b7137f7e675974e025fc3c8e0ab891d353739a";
      saveYamlAtomic(file.string()+".yaml",metadata);
    }
  }
}
TEST(Matching, PendingQueryRetryCapacityAndEpochIsolation) {
  MatcherConfig config;config.database_capacity=2;Matcher matcher(config);auto cloud=scene();
  EXPECT_EQ(matcher.process(input(1,cloud)).reason,"waiting for UGV database");
  auto first=matcher.process(input(0,cloud));ASSERT_TRUE(first.match)<<first.reason;
  EXPECT_EQ(matcher.process(input(0,cloud,1)).status,"database_updated");
  matcher.process(input(0,cloud,2));EXPECT_EQ(matcher.databaseSize(),2u);
  EXPECT_EQ(matcher.counters().at("database_rebuilds"),1u);
  auto q=input(1,cloud,3);q->epoch=1;auto qr=matcher.process(q);EXPECT_TRUE(qr.clear_display);EXPECT_EQ(matcher.databaseSize(),2u);
  auto r=input(0,cloud,4);r->epoch=1;auto rr=matcher.process(r);EXPECT_TRUE(rr.clear_display);EXPECT_EQ(matcher.databaseSize(),1u);
}
TEST(Matching, InvalidAndFlatCloudsRejected) {
  Matcher matcher(MatcherConfig{});auto i=input(0,Cloud::Ptr(new Cloud));EXPECT_EQ(matcher.process(i).status,"rejected");
  for(int k=0;k<100;++k)i->cloud->push_back(pcl::PointXYZ(k*.2,0,0));i->id=1;
  EXPECT_EQ(matcher.process(i).reason,"no height range");i->id=2;i->gravity=Eigen::Quaterniond(0,0,0,0);
  EXPECT_EQ(matcher.process(i).status,"rejected");i->id=3;i->gravity=Eigen::Quaterniond::Identity();i->frame="wrong";
  EXPECT_EQ(matcher.process(i).status,"rejected");
  MatcherConfig config;config.allow_partial=false;Matcher strict(config);EXPECT_EQ(strict.process(input(0,scene())).reason,"partial submaps disabled");
}
TEST(Matching, NonoverlappingGeometryDoesNotPass) {
  Matcher matcher(MatcherConfig{});matcher.process(input(0,scene(7)));auto r=matcher.process(input(1,scene(812)));
  EXPECT_FALSE(r.match)<<"Unrelated scenes must not publish accepted matches";
}
TEST(Matching, NoFeaturesAndGicpFailureAndLowOverlap) {
  Matcher matcher(MatcherConfig{});Cloud::Ptr thin(new Cloud);
  for(int z=0;z<30;++z)for(int x=0;x<30;++x)thin->push_back(pcl::PointXYZ(x*.17,0,z*.17));
  EXPECT_EQ(matcher.process(input(0,thin)).reason,"insufficient ORB features");
  auto cloud=scene();MatcherConfig tiny;tiny.correspondence_distance=1e-6;Matcher no_correspondences(tiny);
  no_correspondences.process(input(0,cloud));auto failed=no_correspondences.process(input(1,cloud));
  EXPECT_FALSE(failed.match);EXPECT_EQ(failed.reason,"GICP not converged");
  Cloud::Ptr noisy(new Cloud(*cloud));std::mt19937 rng(57);std::uniform_real_distribution<float> noise(-.015,.015);
  for(auto& p:*noisy){p.x+=noise(rng);p.y+=noise(rng);p.z+=noise(rng);}
  MatcherConfig small;small.overlap_radius=1e-5;Matcher low_overlap(small);
  low_overlap.process(input(0,cloud));auto result=low_overlap.process(input(1,noisy));
  EXPECT_FALSE(result.match);EXPECT_EQ(result.reason,"overlap/residual quality gate failed");
}
TEST(Matching, SparseCandidateIdsAndFirstCrossRobotReference) {
  DescriptorManagerConfig c;c.n_row_=c.n_col_=210;c.reso_row_=c.reso_col_=.5;c.height_res_=1;c.max_dens_=135;c.local_density_rows_=c.local_density_cols_=3;
  auto make=[&](int id){auto d=std::make_shared<DescriptorManager>(c,id);Cloud::ConstPtr points=scene();d->makeDensityBEV<pcl::PointXYZ>(points);d->makeFeature();return d;};
  auto a=make(0),b=make(1),q=make(0);a->bev_descriptors_.setTo(0);
  DescriptorDBConfig cfg;cfg.n_row_=cfg.n_col_=210;cfg.reso_row_=cfg.reso_col_=.5;cfg.bucket_size_=16;cfg.dist_thres_=60;cfg.skip_num_=-1;
  DescriptorDB db(cfg);db.getClassMatchables(a);db.addClassToHBST(a);db.getClassMatchables(b);db.addClassToHBST(b);
  db.getClassMatchables(q);std::vector<int> ids;std::vector<double> counts;Eigen::Matrix4d pose;bool height=false;
  db.queryFeatureHamming(q,ids,counts,pose,&height);db.releaseQueryMatchables();
  ASSERT_EQ(ids.size(),1u);EXPECT_EQ(ids[0],1);EXPECT_TRUE(height);EXPECT_TRUE(properTransform(pose));
  for(int k=0;k<q->bev_descriptors_.rows;++k)
    const_cast<boost::dynamic_bitset<>&>(q->getKeyHeightBin(k)).reset();
  db.getClassMatchables(q);db.queryFeatureHamming(q,ids,counts,pose,&height);db.releaseQueryMatchables();
  EXPECT_FALSE(height);EXPECT_TRUE(properTransform(pose));
}
TEST(Matching, DescriptorParityAndEmptyRansacGuard) {
  DescriptorManagerConfig c;c.n_row_=c.n_col_=210;c.reso_row_=c.reso_col_=.5;
  c.height_res_=1;c.max_dens_=135;c.local_density_rows_=c.local_density_cols_=3;
  auto cloud=scene();Cloud::ConstPtr points=cloud;DescriptorManager a(c,0),b(c,0);
  a.makeDensityBEV<pcl::PointXYZ>(points);b.makeDensityBEV<pcl::PointXYZ>(points);
  a.makeFeature();b.makeFeature();EXPECT_EQ(cv::norm(a.getBevImage(),b.getBevImage()),0);
  EXPECT_EQ(cv::norm(a.bev_descriptors_,b.bev_descriptors_),0);
  Matcher wrapper(MatcherConfig{});auto event=wrapper.process(input(0,cloud));
  EXPECT_EQ(cv::norm(event.bev,a.getBevImage()),0);
  auto result=map_closures::RansacAlignment2D({});EXPECT_TRUE(result.second.empty());EXPECT_TRUE(result.first.matrix().allFinite());
}
TEST(Matching, RankedCandidatesKeepIndependentHeightValidity) {
  DescriptorManagerConfig c;c.n_row_=c.n_col_=210;c.reso_row_=c.reso_col_=.5;
  c.height_res_=1;c.max_dens_=135;c.local_density_rows_=c.local_density_cols_=3;
  auto make=[&](int id){auto d=std::make_shared<DescriptorManager>(c,id);Cloud::ConstPtr cloud=scene();
    d->makeDensityBEV<pcl::PointXYZ>(cloud);d->makeFeature();return d;};
  auto invalid=make(0),valid=make(1),query=make(0);
  for(int k=0;k<invalid->getLevKeyPointsSize();++k)
    const_cast<boost::dynamic_bitset<>&>(invalid->getKeyHeightBin(k)).reset();
  DescriptorDBConfig cfg;cfg.n_row_=cfg.n_col_=210;cfg.reso_row_=cfg.reso_col_=.5;
  cfg.bucket_size_=16;cfg.dist_thres_=60;cfg.skip_num_=-1;
  DescriptorDB db(cfg);
  for(auto d:{invalid,valid}){db.getClassMatchables(d);db.addClassToHBST(d);}
  db.getClassMatchables(query);const auto candidates=db.queryFeatureCandidates(query,2);db.releaseQueryMatchables();
  ASSERT_EQ(candidates.size(),2u);
  EXPECT_EQ(candidates[0].reference_id,0);EXPECT_FALSE(candidates[0].height_valid);
  EXPECT_EQ(candidates[1].reference_id,1);EXPECT_TRUE(candidates[1].height_valid);
  EXPECT_EQ(candidates[0].empty_height_pairs,candidates[0].height_pairs);
  EXPECT_LT(candidates[1].empty_height_pairs,candidates[1].height_pairs);
  EXPECT_TRUE(properTransform(candidates[1].pose));
}
TEST(Matching, LocalRegistrationRetainsFullHistoryForFusion) {
  auto nearby=scene();for(auto& p:*nearby){p.x*=.3f;p.y*=.3f;}
  auto complete=Cloud::Ptr(new Cloud(*nearby));
  for(auto p:*nearby){p.x-=25;complete->push_back(p);}
  MatcherConfig config;config.matching_radius_ugv=8;config.voxel_size=.5;config.correspondence_distance=1;
  config.min_structural_overlap=.6;
  Matcher matcher(config);auto ref=input(0,complete),query=input(1,nearby);
  ASSERT_EQ(matcher.process(ref).status,"database_updated");auto event=matcher.process(query);
  ASSERT_TRUE(event.match)<<event.reason;
  EXPECT_LT(event.match->matching_reference_points,event.match->reference->cloud->size());
  EXPECT_EQ(event.match->reference->cloud->size(),complete->size());
  EXPECT_LT(event.match->refined.translation().norm(),.2);
  EXPECT_TRUE(event.match->structural_check_enabled);
  EXPECT_GE(std::min(event.match->structural_overlap_reference,event.match->structural_overlap_query),.6);
  const auto fused=fuseSubmaps(*event.match,.15);
  float minimum=0;for(const auto& p:fused.cloud)minimum=std::min(minimum,p.x);
  EXPECT_LT(minimum,-20);EXPECT_GT(fused.ugv_only_voxels,0u);
  MatcherConfig invalid=config;invalid.matching_radius_ugv=-1;EXPECT_THROW(Matcher bad(invalid),std::invalid_argument);
}
TEST(Matching, StructuralGateRefusesInsufficientEvidence) {
  MatcherConfig config;config.min_structural_overlap=.6;config.min_structural_points=1000000;
  Matcher matcher(config);auto cloud=scene();matcher.process(input(0,cloud));auto event=matcher.process(input(1,cloud));
  EXPECT_FALSE(event.match);EXPECT_EQ(event.reason,"insufficient elevated structural points");
  ASSERT_EQ(event.candidate_attempts.size(),1u);ASSERT_TRUE(event.candidate_attempts[0].result);
  EXPECT_TRUE(event.candidate_attempts[0].result->structural_check_enabled);
  EXPECT_GE(event.candidate_attempts[0].result->overlap,config.min_overlap);
}
TEST(Matching, LargerRadiusFallbackPreservesHistoryAndConsumesQuery) {
  auto nearby=scene();for(auto& p:*nearby){p.x*=.3f;p.y*=.3f;}
  auto complete=Cloud::Ptr(new Cloud(*nearby));
  for(auto p:*nearby){p.x-=25;complete->push_back(p);}
  MatcherConfig config;config.matching_radius_ugv=2;config.matching_radius_ugv_fallback=8;
  config.voxel_size=.5;config.correspondence_distance=1;config.min_structural_overlap=.6;
  auto ref=input(0,complete),query=input(1,nearby);
  auto primary_config=config;primary_config.matching_radius_ugv_fallback=0;
  Matcher primary(primary_config);primary.process(ref);EXPECT_FALSE(primary.process(query).match);
  Matcher matcher(config);matcher.process(ref);auto event=matcher.process(query);
  ASSERT_TRUE(event.match)<<event.reason;
  EXPECT_DOUBLE_EQ(event.match->matching_radius_ugv,8);
  EXPECT_EQ(event.diagnostics.at("radius_fallback_selected"),1);
  EXPECT_EQ(matcher.counters().at("accepted_matches"),1u);
  EXPECT_EQ(matcher.counters().at("received"),2u);
  EXPECT_EQ(matcher.counters().at("radius_fallback_matches"),1u);
  EXPECT_GE(std::min(event.match->structural_overlap_reference,event.match->structural_overlap_query),.6);
  EXPECT_LT(event.match->matching_reference_points,complete->size());
  EXPECT_LT(event.match->refined.translation().norm(),.2);
  const auto fused=fuseSubmaps(*event.match,.15);float minimum=0;
  for(const auto& p:fused.cloud)minimum=std::min(minimum,p.x);EXPECT_LT(minimum,-20);
  // Both databases must forget an accepted pending UAV query.
  EXPECT_FALSE(matcher.process(input(0,complete,1)).match);
  EXPECT_EQ(matcher.counters().at("accepted_matches"),1u);
  matcher.invalidate();EXPECT_EQ(matcher.databaseSize(),0u);
  EXPECT_EQ(matcher.process(input(1,nearby,2)).reason,"waiting for UGV database");
  for(double radius:{-1.,2.,1.,std::numeric_limits<double>::quiet_NaN()}) {
    auto invalid=config;invalid.matching_radius_ugv_fallback=radius;
    EXPECT_THROW(Matcher bad(invalid),std::invalid_argument);
  }
}
TEST(Matching, PrimaryRadiusSuccessDoesNotQueryFallback) {
  auto cloud=scene();for(auto& p:*cloud){p.x*=.3f;p.y*=.3f;}
  MatcherConfig config;config.matching_radius_ugv=8;config.matching_radius_ugv_fallback=12;
  config.voxel_size=.5;config.correspondence_distance=1;config.min_structural_overlap=.6;
  Matcher matcher(config);matcher.process(input(0,cloud));auto event=matcher.process(input(1,cloud));
  ASSERT_TRUE(event.match)<<event.reason;
  EXPECT_DOUBLE_EQ(event.match->matching_radius_ugv,8);
  EXPECT_EQ(event.diagnostics.at("radius_fallback_selected"),0);
  EXPECT_EQ(matcher.counters().at("queries"),1u);
  EXPECT_EQ(matcher.counters().at("accepted_matches"),1u);
  EXPECT_EQ(matcher.counters().at("radius_fallback_database_size"),1u);
  EXPECT_FALSE(matcher.process(input(0,cloud,1)).match);
}
TEST(RecentCloud, ExpiresHistoryAndTransformsIntoTheSubmapKeyframe) {
  RecentCloudConfig config;config.window_seconds=2;config.min_frames=2;config.max_frames=10;
  RecentCloudBuffer buffer(config);auto a=Cloud::Ptr(new Cloud),b=Cloud::Ptr(new Cloud);
  a->push_back(pcl::PointXYZ(1,2,3));b->push_back(pcl::PointXYZ(4,5,6));
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()<<.1,.2,.3;
  ASSERT_TRUE(buffer.add(a,pose,1));EXPECT_FALSE(buffer.snapshot(pose,1));
  ASSERT_TRUE(buffer.add(b,pose,2));auto first=buffer.snapshot(pose,2);ASSERT_TRUE(first);
  ASSERT_EQ(first->local->size(),2u);EXPECT_NEAR((*first->local)[0].x,.9,1e-6);
  EXPECT_DOUBLE_EQ(first->start_stamp,1);EXPECT_DOUBLE_EQ(first->end_stamp,2);
  EXPECT_LT(first->center_local.norm(),1e-9);EXPECT_FALSE(buffer.add(a,pose,1.5));
  Eigen::Isometry3d old_key=Eigen::Isometry3d::Identity();old_key.translation().x()=10;
  auto anchored=buffer.snapshot(old_key,2);ASSERT_TRUE(anchored);
  EXPECT_NEAR(anchored->center_local.x(),-9.9,1e-9);
  ASSERT_TRUE(buffer.add(b,pose,5));EXPECT_FALSE(buffer.snapshot(pose,5));
  ASSERT_TRUE(buffer.add(b,pose,6));auto recent=buffer.snapshot(pose,6);ASSERT_TRUE(recent);
  ASSERT_EQ(recent->local->size(),1u);EXPECT_NEAR((*recent->local)[0].x,3.9,1e-6);
  EXPECT_FALSE(buffer.snapshot(pose,8));buffer.reset();EXPECT_FALSE(buffer.snapshot(pose,6));
}
TEST(Matching, RecentWindowDrivesMatchingWhileFusionRetainsFullHistory) {
  auto nearby=scene();for(auto& p:*nearby){p.x*=.3f;p.y*=.3f;}
  auto complete=Cloud::Ptr(new Cloud(*nearby));for(auto p:*nearby){p.x-=25;complete->push_back(p);}
  auto ref=input(0,complete),query=input(1,nearby);
  for(auto sample:{ref,query}){sample->recent_cloud=nearby;sample->recent_start_stamp=1;sample->recent_end_stamp=6;sample->recent_frames=50;}
  MatcherConfig config;config.use_recent_matching_cloud=true;config.matching_radius_ugv=8;
  config.matching_radius_ugv_fallback=12;config.voxel_size=.5;config.correspondence_distance=1;config.min_structural_overlap=.6;
  Matcher matcher(config);matcher.process(ref);auto event=matcher.process(query);ASSERT_TRUE(event.match)<<event.reason;
  EXPECT_TRUE(event.match->reference_recent);EXPECT_TRUE(event.match->query_recent);
  EXPECT_EQ(event.diagnostics.at("input_recent_cloud_used"),1);
  EXPECT_LT(event.match->matching_reference_points,complete->size());
  auto fused=fuseSubmaps(*event.match,.15);float minimum=0;
  for(const auto& p:fused.cloud)minimum=std::min(minimum,p.x);EXPECT_LT(minimum,-20);
  EXPECT_FALSE(matcher.process(input(0,complete,1)).match);
  auto invalid=input(1,nearby,2);invalid->recent_cloud=nearby;invalid->recent_frames=0;
  EXPECT_EQ(matcher.process(invalid).reason,"invalid recent cloud metadata");
}
TEST(Matching, RecentWindowsRejectUnrelatedScenes) {
  MatcherConfig config;config.use_recent_matching_cloud=true;config.voxel_size=.5;
  config.correspondence_distance=1;config.min_structural_overlap=.6;
  auto ref=input(0,scene(7)),query=input(1,scene(812));
  for(auto sample:{ref,query}) {
    sample->recent_cloud=sample->cloud;sample->recent_start_stamp=1;
    sample->recent_end_stamp=6;sample->recent_frames=50;
  }
  Matcher matcher(config);matcher.process(ref);EXPECT_FALSE(matcher.process(query).match);
}
TEST(Matching, FusionKeepsComplementaryPointsAndUsesUgvFrame) {
  auto r=input(0,Cloud::Ptr(new Cloud)),q=input(1,Cloud::Ptr(new Cloud));
  r->cloud->push_back(pcl::PointXYZ(-.1,.1,.1));r->cloud->push_back(pcl::PointXYZ(-.12,.1,.1));
  r->cloud->push_back(pcl::PointXYZ(1.1,.1,.1));
  Cloud query_world;query_world.push_back(pcl::PointXYZ(1.1,.1,.1));query_world.push_back(pcl::PointXYZ(2.1,.1,.1));
  r->pose.linear()=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();r->pose.translation()=Eigen::Vector3d(10,20,0);
  MatchResult match;match.reference=r;match.query=q;
  match.refined.linear()=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ()).toRotationMatrix();match.refined.translation()=Eigen::Vector3d(2,3,.5);
  pcl::transformPointCloud(query_world,*q->cloud,Eigen::Affine3d(match.refined));
  auto fused=fuseSubmaps(match,1);
  ASSERT_EQ(fused.cloud.size(),3u);EXPECT_EQ(fused.ugv_only_voxels,1u);EXPECT_EQ(fused.uav_only_voxels,1u);EXPECT_EQ(fused.shared_voxels,1u);
  for(const auto& p:fused.cloud) {
    EXPECT_NEAR(p.x,9.9,1e-5);EXPECT_NEAR(p.z,.1,1e-5);
    if(p.r==40){EXPECT_EQ(p.g,220);EXPECT_NEAR(p.y,19.89,1e-5);}
    else if(p.r==230){EXPECT_EQ(p.g,70);EXPECT_NEAR(p.y,22.1,1e-5);}
    else {EXPECT_EQ(p.r,255);EXPECT_EQ(p.g,200);EXPECT_NEAR(p.y,21.1,1e-5);}
  }
  EXPECT_THROW(fuseSubmaps(match,0),std::invalid_argument);
}
TEST(Matching, ComplementaryCoverageCanMatchAndFuse) {
  auto world=scene();auto r=input(0,Cloud::Ptr(new Cloud)),q=input(1,Cloud::Ptr(new Cloud));Cloud query_world;
  for(const auto& p:*world){if(p.x<16)r->cloud->push_back(p);if(p.x>-16)query_world.push_back(p);}
  q->pose.translation()=Eigen::Vector3d(4,-4,0);
  pcl::transformPointCloud(query_world,*q->cloud,Eigen::Affine3d(q->pose.inverse()));
  Matcher matcher(MatcherConfig{});matcher.process(r);auto event=matcher.process(q);ASSERT_TRUE(event.match)<<event.reason;
  auto fused=fuseSubmaps(*event.match,.15);EXPECT_GT(fused.ugv_only_voxels,0u);EXPECT_GT(fused.uav_only_voxels,0u);
  float ref_max=-1e9f,fused_max=ref_max;for(const auto& p:*r->cloud)ref_max=std::max(ref_max,p.x);for(const auto& p:fused.cloud)fused_max=std::max(fused_max,p.x);
  EXPECT_GT(fused_max,ref_max+1);
  if(const char* path=std::getenv("STAGE3_COMPLEMENTARY_DIR")) {
    std::filesystem::create_directories(path);
    for(auto sample:{r,q}) {
      auto file=std::filesystem::path(path)/(sample->robot==0?"ugv_submap_000":"uav_submap_000");
      pcl::io::savePCDFileBinary(file.string()+".pcd",*sample->cloud);
      auto metadata=inputMetadata(*sample);metadata["upstream_commit"]="b4b7137f7e675974e025fc3c8e0ab891d353739a";
      metadata["fixture"]="synthetic complementary coverage; known relative pose; not Gazebo or flight data";
      saveYamlAtomic(file.string()+".yaml",metadata);
    }
  }
}
TEST(LioQuality, KeyframeCurrentAndTimestampChecks) {
  LioQualityGuard g;g.add(0,{0,true,false,200});g.add(0,{10,true,false,200});
  EXPECT_TRUE(g.reason(0,10,10).empty());EXPECT_FALSE(g.reason(0,9,10).empty());
  EXPECT_FALSE(g.reason(0,10,12).empty());
  g.add(0,{11,false,true,NAN});EXPECT_FALSE(g.reason(0,10,11).empty());
  g.add(0,{12,true,false,200});EXPECT_TRUE(g.reason(0,10,12).empty());
  EXPECT_FALSE(g.reason(1,10,12).empty());
}
TEST(LioQuality, QuarantineCannotBeClearedByHealthyFrames) {
  LioQualityGuard g;g.add(0,{0,false,true,NAN});
  g.add(0,{15,true,true,20});g.add(0,{16,true,true,20});EXPECT_FALSE(g.quarantined(0));
  g.add(0,{17,true,true,20});EXPECT_TRUE(g.quarantined(0));
  g.add(0,{18,true,false,200});EXPECT_FALSE(g.reason(0,18,18).empty());
  g.add(1,{18,true,false,200});EXPECT_TRUE(g.reason(1,18,18).empty());
}
TEST(LioQuality, DatabaseInvalidationRemovesPendingQueries) {
  Matcher matcher(MatcherConfig{});auto ref=input(0,scene());matcher.process(ref);matcher.invalidate();
  EXPECT_EQ(matcher.databaseSize(),0u);auto result=matcher.process(input(1,scene()));EXPECT_FALSE(result.match);
  EXPECT_EQ(result.reason,"waiting for UGV database");
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
