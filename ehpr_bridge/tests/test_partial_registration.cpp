#include <gtest/gtest.h>
#include <ehpr_bridge/partial_registration.h>
#include <ehpr_bridge/matching_io.h>
#include <limits>
using namespace ehpr_bridge;

static Cloud::Ptr room(bool extra=false,bool floor=true,bool second=true) {
  Cloud::Ptr cloud(new Cloud);
  for(float x=-3;x<=5;x+=.15f)for(float y=-2;y<=6;y+=.15f)
    if(floor)cloud->push_back(pcl::PointXYZ(x,y,0));
  for(float u=-3;u<=5;u+=.15f)for(float z=0;z<=2.5;z+=.15f) {
    cloud->push_back(pcl::PointXYZ(u,6,z));
    if(extra)cloud->push_back(pcl::PointXYZ(u,-2,z));
  }
  for(float u=-2;u<=6;u+=.15f)for(float z=0;z<=2.5;z+=.15f) {
    if(second)cloud->push_back(pcl::PointXYZ(5,u,z));
    if(extra)cloud->push_back(pcl::PointXYZ(-3,u,z));
  }for(auto& p:*cloud)p.z-=1;return cloud;
}
static SubmapPtr sample(int robot,Cloud::Ptr cloud) {
  auto p=std::make_shared<SubmapInput>();p->robot=robot;p->session="partial_fixture";
  p->frame=robot==0?"odom":"uav_lio_odom";p->local_frame="key";p->cloud=cloud;
  p->matching_gravity=Eigen::Quaterniond::Identity();return p;
}
TEST(Partial, CanonicalGravityIsContinuousAcrossYawZero) {
  Eigen::Matrix3d previous;
  for(double yaw:{-.00001,0.,.00001}) {
    const Eigen::Matrix3d tilt=(Eigen::AngleAxisd(.04,Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(-.03,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Matrix3d pose=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*tilt;
    const auto g=canonicalMatchingGravity(pose);
    EXPECT_LT((g.toRotationMatrix()-tilt).norm(),1e-10);
    EXPECT_LT((g*pose.transpose()*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm(),1e-10);
  }
}
TEST(Partial, UnequalObservationsRefineUsingIndependentGeometry) {
  MatcherConfig config;config.matching_profile="partial";config.min_inliers=5;
  auto a=room(),b=room(true);MatchResult m;m.reference=sample(0,a);m.query=sample(1,b);
  m.reference_recent=m.query_recent=true;
  m.coarse.translation()<<.2,-.15,.1;
  m.coarse.linear()=Eigen::AngleAxisd(.025,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto reason=refinePartial(*preparePartialGeometry(a,Eigen::Quaterniond::Identity()),
                           *preparePartialGeometry(b,Eigen::Quaterniond::Identity()),config,m);
  ASSERT_TRUE(reason.empty())<<reason<<"; translation="<<m.refined.translation().transpose();
  EXPECT_LT(m.refined.translation().norm(),.08);EXPECT_LT(Eigen::AngleAxisd(m.refined.rotation()).angle(),.02);
  EXPECT_GE(m.structural_correspondences,80u);EXPECT_GE(m.information_ratio,1e-4);
  EXPECT_LE(m.structural_correspondences,std::min(m.structural_reference_points,m.structural_query_points));
  EXPECT_LT(std::min(m.structural_overlap_reference,m.structural_overlap_query),.7);
}
TEST(Partial, HistoricalAccumulationDoesNotInventRayEvidence) {
  const auto directory=std::filesystem::path(__FILE__).parent_path()/"data/partial_wrong_seed";
  const auto inputs=loadReplay(directory);ASSERT_EQ(inputs.size(),2u);
  std::array<SubmapPtr,2> normalized;
  for(auto input:inputs){auto copy=std::make_shared<SubmapInput>(*input);
    copy->matching_gravity=canonicalMatchingGravity(copy->pose.rotation());normalized[input->robot]=copy;}
  MatchResult m;m.reference=normalized[0];m.query=normalized[1];m.reference_recent=m.query_recent=true;
  const auto seed=YAML::LoadFile((directory/"bad_seed.yaml").string())["coarse_transform"];
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)m.coarse.matrix()(i,j)=seed[i][j].as<double>();
  std::array<std::shared_ptr<PartialGeometry>,2> geometry;
  for(int robot=0;robot<2;++robot){const auto& input=normalized[robot];const auto g=*input->matching_gravity;
    geometry[robot]=preparePartialGeometry(input->recent_cloud,g,g*input->recent_center_local);}
  MatcherConfig c;c.matching_profile="partial";
  const auto reason=refinePartial(*geometry[0],*geometry[1],c,m);
  EXPECT_FALSE(m.visibility.available);
  EXPECT_FALSE(m.visibility.sufficient); // This old fixture has no single scan attachments.
  EXPECT_EQ(m.visibility_checked_rays,0u);
}
TEST(Partial, TiltedKeysRecoverLocalTransformDirection) {
  auto a=room(true);Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
  pose.linear()=(Eigen::AngleAxisd(-.1,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.04,Eigen::Vector3d::UnitY())*
                 Eigen::AngleAxisd(-.03,Eigen::Vector3d::UnitX())).toRotationMatrix();pose.translation()<<.3,-.2,.5;
  Cloud::Ptr b(new Cloud);pcl::transformPointCloud(*a,*b,Eigen::Affine3d(pose.inverse()));
  auto q=std::make_shared<SubmapInput>(*sample(1,b));q->pose=pose;q->matching_gravity=canonicalMatchingGravity(pose.rotation());
  MatchResult m;m.reference=sample(0,a);m.query=q;m.coarse=pose.inverse();m.coarse.translation().x()+=.15;
  MatcherConfig c;c.matching_profile="partial";
  auto reason=refinePartial(*preparePartialGeometry(a,Eigen::Quaterniond::Identity()),
                           *preparePartialGeometry(b,*q->matching_gravity),c,m);
  ASSERT_TRUE(reason.empty())<<reason<<"; translation="<<m.refined.translation().transpose()
      <<"; final updates="<<m.translation_update_m<<" m / "<<m.rotation_update_deg<<" deg";
  EXPECT_LT((m.refined.translation()-pose.inverse().translation()).norm(),.08);
  EXPECT_LT(Eigen::AngleAxisd(m.refined.rotation().transpose()*pose.inverse().rotation()).angle(),.02);
}
TEST(Partial, UnobservablePlaneAndRepeatedPointsCannotPass) {
  MatcherConfig config;config.matching_profile="partial";
  for(bool ground:{false,true}) {
    auto a=ground?Cloud::Ptr(new Cloud):room(false,false,false);
    if(ground)for(float x=0;x<8;x+=.15f)for(float y=0;y<8;y+=.15f)a->push_back(pcl::PointXYZ(x,y,0));
    MatchResult m;m.reference=sample(0,a);m.query=sample(1,a);
    auto g=preparePartialGeometry(a,Eigen::Quaterniond::Identity());
    EXPECT_FALSE(refinePartial(*g,*g,config,m).empty());
  }
  auto repeated=Cloud::Ptr(new Cloud);for(int i=0;i<1000;++i)repeated->push_back(pcl::PointXYZ(1,2,3));
  auto g=preparePartialGeometry(repeated,Eigen::Quaterniond::Identity());
  EXPECT_EQ(g->levels[1].cloud->size(),1u);
}
TEST(Partial, SingleWallWithFloorAndDisjointHeightBandsCannotPass) {
  MatcherConfig c;c.matching_profile="partial";
  auto wall=room(false,true,false);MatchResult single;
  single.reference=sample(0,wall);single.query=sample(1,wall);
  auto geometry=preparePartialGeometry(wall,Eigen::Quaterniond::Identity());
  EXPECT_FALSE(refinePartial(*geometry,*geometry,c,single).empty());
  auto low=room(),high=room(false,false,true);for(auto& p:*high)p.z+=4;
  MatchResult disjoint;disjoint.reference=sample(0,low);disjoint.query=sample(1,high);
  EXPECT_EQ(refinePartial(*preparePartialGeometry(low,Eigen::Quaterniond::Identity()),
                          *preparePartialGeometry(high,Eigen::Quaterniond::Identity()),c,disjoint),
            "insufficient horizontal plane support for height");
}
TEST(Partial, CandidateClusteringAndAmbiguityUseMapTransforms) {
  MatcherConfig config;MatchEvent event;event.status="query_pending";
  auto a=room(),b=room(true);
  for(double x:{0.,.1,3.}) {
    CandidateAttempt attempt;MatchResult m;m.reference=sample(0,a);m.query=sample(1,b);
    m.refined.translation().x()=x;m.candidate_score=x==3?.78:.8;
    attempt.result=m;event.candidate_attempts.push_back(attempt);
  }
  selectPartialMatch(event,config);EXPECT_FALSE(event.match);
  EXPECT_EQ(event.reason,"ambiguous partial registration hypotheses");
  EXPECT_EQ(event.diagnostics.at("candidate_pose_clusters"),2);
  event.candidate_attempts.back().result->candidate_score=.6;
  selectPartialMatch(event,config);ASSERT_TRUE(event.match);
  EXPECT_NEAR(event.match->score_margin,.2,1e-10);
}
TEST(Partial, InvalidConfigurationIsRejected) {
  MatcherConfig c;c.matching_profile="partial";c.partial_high_coverage=.2;
  EXPECT_THROW(Matcher m(c),std::invalid_argument);
  c.partial_high_coverage=.7;c.partial_information_ratio=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Matcher m(c),std::invalid_argument);
}
TEST(Partial, DuplicateQueriesAndGenerationChangesPreserveLifecycle) {
  MatcherConfig c;c.matching_profile="partial";c.min_inliers=5;
  c.matching_radius_ugv=8;c.matching_radius_ugv_fallback=12;
  Matcher matcher(c);auto cloud=room(true);
  auto query=std::make_shared<SubmapInput>(*sample(1,cloud));query->stamp=1;
  EXPECT_EQ(matcher.process(query).reason,"waiting for UGV database");
  EXPECT_EQ(matcher.process(query).status,"duplicate");
  auto reference=std::make_shared<SubmapInput>(*sample(0,cloud));reference->stamp=2;
  matcher.process(reference);EXPECT_EQ(matcher.databaseSize(),1u);
  EXPECT_EQ(matcher.process(reference).status,"duplicate");
  reference->epoch=1;reference->id=1;reference->stamp=3;
  const auto reset=matcher.process(reference);EXPECT_TRUE(reset.clear_display);
  EXPECT_FALSE(reset.match);EXPECT_EQ(matcher.databaseSize(),1u);
  auto new_session=std::make_shared<SubmapInput>(*reference);
  new_session->session="next_session";new_session->stamp=4;
  EXPECT_TRUE(matcher.process(new_session).clear_display);EXPECT_EQ(matcher.databaseSize(),1u);
}
