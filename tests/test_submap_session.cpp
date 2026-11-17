#include <gtest/gtest.h>
#include <ehpr_bridge/submap_session.h>
// Upstream defines this unused free function in its header without inline.
// Only rename it in the independent reference TU to avoid an ODR collision.
#define voxelDownSample upstream_reference_voxelDownSample
#include <VoxelMap.hpp>
#undef voxelDownSample
#include <limits>

using namespace ehpr_bridge;
static Eigen::Isometry3d pose(double x, double y = 0, double z = 0) {
  auto p = Eigen::Isometry3d::Identity(); p.translation() = Eigen::Vector3d(x, y, z); return p;
}
static Cloud::Ptr cloud(std::initializer_list<Eigen::Vector3d> points) {
  Cloud::Ptr out(new Cloud);
  for (const auto& p : points) out->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
  return out;
}
static std::vector<Eigen::Vector3f> sorted(const Cloud& cloud) {
  std::vector<Eigen::Vector3f> out;
  for (const auto& p : cloud) out.push_back(p.getVector3fMap());
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
    return std::make_tuple(a.x(), a.y(), a.z()) < std::make_tuple(b.x(), b.y(), b.z());
  });
  return out;
}

TEST(Submaps, UpstreamGenerateSubmapParity) {
  SubmapConfig config;
  SubmapSession bridge(config);
  VoxelMapConfig upstream_config;
  upstream_config.voxel_size = config.voxel_size;
  upstream_config.max_distance = config.max_distance;
  upstream_config.remove_distance = config.remove_distance;
  upstream_config.max_points_per_voxel = config.max_points_per_voxel;
  VoxelMap reference(upstream_config);
  Eigen::Vector3d last = Eigen::Vector3d::Zero();
  Eigen::Isometry3d key = pose(0);
  bool key_recorded = false;
  double distance = 0;
  int outputs = 0;
  for (int i = 0; i < 170; ++i) {
    auto current_pose = pose(i * .25);
    current_pose.linear() = (Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(-.03, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(.04, Eigen::Vector3d::UnitX())).toRotationMatrix();
    auto scan = cloud({{i * .25, -1.5, .5}, {i * .25, .5, .5}, {-2, -2, -2}});
    reference.addPoints(scan);
    distance += (current_pose.translation() - last).norm();
    last = current_pose.translation();
    if (!key_recorded && distance >= config.max_distance * .01) { key = current_pose; key_recorded = true; }
    const bool expected = distance > config.max_distance && key_recorded;
    const auto actual = bridge.add(scan, current_pose, i * .1);
    ASSERT_EQ(actual.has_value(), expected);
    if (expected) {
      Cloud::Ptr points(new Cloud);
      reference.getAllPoints(points);
      pcl::transformPointCloud(*points, *points, Eigen::Affine3d(key.inverse()));
      const auto a = sorted(*actual->local), b = sorted(*points);
      ASSERT_EQ(a.size(), b.size());
      for (size_t j = 0; j < a.size(); ++j) EXPECT_LT((a[j] - b[j]).norm(), 1e-5);
      EXPECT_LT((actual->pose.matrix() - key.matrix()).norm(), 1e-10);
      const auto euler = Eigen::Quaterniond(key.rotation()).matrix().eulerAngles(2, 1, 0);
      Eigen::Quaterniond gravity = Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
      EXPECT_LT(actual->gravity.angularDistance(gravity), 1e-10);
      reference.removePointsFarFromLocation(last);
      distance = 0; key_recorded = false; ++outputs;
    }
  }
  EXPECT_EQ(outputs, 2);
  reference.Clear();
}

TEST(Submaps, LocalCloudAndGravityTransform) {
  SubmapSession session(SubmapConfig{});
  auto p = pose(10, -3, 1);
  p.linear() = (Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-.2, Eigen::Vector3d::UnitX())).toRotationMatrix();
  auto input = cloud({{12, -2, 2}, {9, -3, .8}});
  session.add(input, p, 1);
  const auto s = session.flush(); ASSERT_TRUE(s); EXPECT_TRUE(s->partial);
  Cloud restored;
  pcl::transformPointCloud(*s->local, restored, Eigen::Affine3d(s->pose));
  auto a = sorted(restored), b = sorted(*input);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) EXPECT_LT((a[i] - b[i]).norm(), 1e-4);
  EXPECT_LT((s->gravity * p.rotation().transpose() * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm(), 1e-10);
  EXPECT_FALSE(session.flush());
}
TEST(Submaps, StrictTwentyMeterTriggerAndEarlyKeyframe) {
  SubmapSession session(SubmapConfig{});
  const auto input = cloud({{0, 0, 1}});
  session.add(input, pose(0), 0);
  session.add(input, pose(.1), 1);
  session.add(input, pose(.2), 2);
  EXPECT_FALSE(session.add(input, pose(20), 3));
  auto s = session.add(input, pose(20.1), 4);
  ASSERT_TRUE(s); EXPECT_FALSE(s->partial); EXPECT_DOUBLE_EQ(s->key_stamp, 2);
  EXPECT_NEAR(s->pose.translation().x(), .2, 1e-9);
}
TEST(Submaps, RetainsHistoryAndRemovesFarVoxelsAfterExport) {
  SubmapSession session(SubmapConfig{});
  session.add(cloud({{0, 0, 1}, {200, 0, 1}}), pose(0), 0);
  auto first = session.add(cloud({{20.1, 0, 1}}), pose(20.1), 1);
  ASSERT_TRUE(first); EXPECT_EQ(first->local->size(), 3u);
  session.add(cloud({{20.5, 0, 1}}), pose(20.5), 2);
  auto second = session.flush(); ASSERT_TRUE(second);
  Cloud reference; pcl::transformPointCloud(*second->local, reference, Eigen::Affine3d(second->pose));
  EXPECT_TRUE(std::any_of(reference.begin(), reference.end(), [](const auto& p) { return std::abs(p.x) < 1e-4; }));
  EXPECT_FALSE(std::any_of(reference.begin(), reference.end(), [](const auto& p) { return p.x > 100; }));
}
TEST(Submaps, UpstreamVoxelSpacingCapacityAndNegativeCoordinates) {
  SubmapSession session(SubmapConfig{});
  Cloud::Ptr input(new Cloud);
  for (int x = 0; x < 4; ++x) for (int y = 0; y < 4; ++y) for (int z = 0; z < 4; ++z)
    input->push_back(pcl::PointXYZ(-.9 + x * .25, -.9 + y * .25, -.9 + z * .25));
  session.add(input, pose(0), 0);
  session.add(input, pose(0), 1);
  auto s = session.flush(); ASSERT_TRUE(s); EXPECT_EQ(s->local->size(), 25u);
  for (size_t i = 0; i < s->local->size(); ++i) for (size_t j = i + 1; j < s->local->size(); ++j)
    EXPECT_GE((s->local->points[i].getVector3fMap() - s->local->points[j].getVector3fMap()).norm(), .2 - 1e-6);
}
TEST(Submaps, RobotInstancesAndResetAreIndependent) {
  SubmapSession ugv(SubmapConfig{}), uav(SubmapConfig{});
  ugv.add(cloud({{1, 0, 0}}), pose(0), 1);
  uav.add(cloud({{10, 0, 0}}), pose(0), 1);
  ugv.reset(); EXPECT_FALSE(ugv.flush());
  auto s = uav.flush(); ASSERT_TRUE(s); EXPECT_EQ(s->local->size(), 1u);
  EXPECT_FLOAT_EQ(s->local->front().x, 10);
}
TEST(Submaps, InvalidInputsAndTimestampOrderAreRejected) {
  SubmapSession session(SubmapConfig{});
  EXPECT_THROW(session.add(Cloud::Ptr(new Cloud), pose(0), 0), std::invalid_argument);
  auto p = pose(0); p.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(session.add(cloud({{0, 0, 0}}), p, 1), std::invalid_argument);
  session.add(cloud({{0, 0, 0}}), pose(0), 1);
  EXPECT_THROW(session.add(cloud({{0, 0, 0}}), pose(0), 1), std::invalid_argument);
  EXPECT_THROW(session.add(cloud({{0, 0, 0}}), pose(0), .5), std::invalid_argument);
}
TEST(Submaps, InvalidConfigurationIsRejected) {
  SubmapConfig config; config.voxel_size = 0;
  EXPECT_THROW(SubmapSession instance(config), std::invalid_argument);
}
TEST(Submaps, LivePreviewUsesLatestAnchorWithoutConsumingTwentyMeterSegment) {
  SubmapSession session(SubmapConfig{});auto scan=cloud({{0,1,1},{2,1,1}});
  EXPECT_FALSE(session.preview(pose(0),0));
  session.add(scan,pose(0),1);session.add(scan,pose(1),2);
  auto preview=session.preview(pose(1),2);ASSERT_TRUE(preview);EXPECT_TRUE(preview->partial);
  EXPECT_DOUBLE_EQ(preview->key_stamp,2);EXPECT_NEAR(preview->pose.translation().x(),1,1e-10);
  EXPECT_TRUE(session.pending());EXPECT_NEAR(session.travelDistance(),1,1e-10);
  EXPECT_THROW(session.preview(pose(1),1),std::invalid_argument);
  EXPECT_FALSE(session.add(scan,pose(20),3));
  auto full=session.add(scan,pose(20.1),4);ASSERT_TRUE(full);EXPECT_FALSE(full->partial);
  EXPECT_DOUBLE_EQ(full->key_stamp,2);EXPECT_NEAR(full->travel_distance,20.1,1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
