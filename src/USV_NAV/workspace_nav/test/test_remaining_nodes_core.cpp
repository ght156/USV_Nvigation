#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "workspace_nav/cloud_filter_core.hpp"
#include "workspace_nav/nav_status_logic.hpp"
#include "workspace_nav/zone_geometry.hpp"

namespace mb = workspace_nav::mission_bridge;
namespace zg = workspace_nav::zone_geometry;

TEST(ZoneGeometry, PointInPolygonAndFill) {
  const std::vector<mb::Point> square{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_TRUE(zg::point_in_polygon({5, 5}, square));
  EXPECT_FALSE(zg::point_in_polygon({15, 5}, square));
  EXPECT_FALSE(zg::point_in_polygon({5, 5}, {{0, 0}, {1, 1}}));

  std::vector<int8_t> mask(400, zg::kFree);
  EXPECT_EQ(zg::fill_polygon_grid(mask, 20, 20, square, zg::kKeepout), 100U);
  EXPECT_EQ(mask[5 * 20 + 5], zg::kKeepout);
  EXPECT_EQ(mask[15 * 20 + 15], zg::kFree);
}

TEST(ZoneGeometry, DrawPolylineDoesNotCloseAndSupportsDilation) {
  std::vector<int8_t> mask(400, zg::kFree);
  zg::draw_polyline_grid(mask, 20, 20, {{0, 10}, {19, 10}}, zg::kKeepout, 0);
  for (int x = 0; x < 20; ++x) EXPECT_EQ(mask[10 * 20 + x], zg::kKeepout);
  EXPECT_EQ(mask[0], zg::kFree);
  EXPECT_EQ(mask[9 * 20], zg::kFree);

  std::fill(mask.begin(), mask.end(), zg::kFree);
  zg::draw_polyline_grid(mask, 20, 20, {{0, 10}, {19, 10}}, zg::kKeepout, 1);
  EXPECT_EQ(mask[9 * 20 + 5], zg::kKeepout);
  EXPECT_EQ(mask[11 * 20 + 5], zg::kKeepout);
  EXPECT_EQ(mask[8 * 20 + 5], zg::kFree);
}

TEST(ZoneGeometry, InclusionAndExclusionProduceExpectedMask) {
  const zg::Fill inclusion = zg::Fill::polygon({{5, 5}, {15, 5}, {15, 15}, {5, 15}});
  const zg::Fill exclusion = zg::Fill::circle({10, 10}, 2.0);
  const auto mask = zg::build_fence_mask(20, 20, {inclusion}, {exclusion}, {}, 2);
  EXPECT_EQ(mask[2 * 20 + 2], zg::kKeepout);
  EXPECT_EQ(mask[6 * 20 + 6], zg::kFree);
  EXPECT_EQ(mask[10 * 20 + 10], zg::kKeepout);

  const auto empty = zg::build_fence_mask(20, 20, {}, {}, {}, 2);
  EXPECT_TRUE(std::all_of(empty.begin(), empty.end(), [](int8_t value) {return value == 0;}));
}

TEST(ZoneGeometry, CircleRasterizationUsesCellCenters) {
  std::vector<int8_t> mask(400, zg::kFree);
  zg::fill_circle_grid(mask, 20, 20, {10, 10}, 4.0, zg::kKeepout);
  EXPECT_EQ(mask[10 * 20 + 10], zg::kKeepout);
  EXPECT_EQ(mask[10 * 20 + 13], zg::kKeepout);
  EXPECT_EQ(mask[10 * 20 + 14], zg::kFree);
  EXPECT_EQ(mask[0], zg::kFree);
}

TEST(NavStatusLogic, LinearCommandAndDisplacementCountAsProgress) {
  workspace_nav::nav_status::StuckTracker tracker(2.0, 60.0, 2.0);
  tracker.update(true, {0, 0}, 0.0, 0.0, false);
  const auto update = tracker.update(true, {6, 0}, 100.0, 100.0, true);
  EXPECT_EQ(update, workspace_nav::nav_status::StuckEvent::Progress);
  EXPECT_FALSE(tracker.stuck());
  EXPECT_DOUBLE_EQ(tracker.last_progress_time(), 100.0);
}

TEST(NavStatusLogic, AngularOrZeroCommandDriftDoesNotClearStuck) {
  workspace_nav::nav_status::StuckTracker tracker(2.0, 60.0, 2.0);
  tracker.update(true, {0, 0}, 0.0, 0.0, false);
  EXPECT_EQ(
    tracker.update(true, {6, 0}, 100.0, 100.0, false),
    workspace_nav::nav_status::StuckEvent::StuckNoProgress);
  EXPECT_TRUE(tracker.stuck());
}

TEST(NavStatusLogic, StaleCommandStreamWinsOverDisplacement) {
  workspace_nav::nav_status::StuckTracker tracker(2.0, 60.0, 2.0);
  tracker.update(true, {0, 0}, 0.0, 90.0, true);
  EXPECT_EQ(
    tracker.update(true, {6, 0}, 100.0, 90.0, true),
    workspace_nav::nav_status::StuckEvent::StuckCommandStale);
  EXPECT_TRUE(tracker.stuck());
}

TEST(NavStatusLogic, MissionResetClearsCommandAndProgressHistory) {
  workspace_nav::nav_status::StuckTracker tracker(2.0, 60.0, 2.0);
  tracker.update(true, {0, 0}, 0.0, 0.0, false);
  tracker.update(true, {0, 0}, 100.0, 90.0, false);
  ASSERT_TRUE(tracker.stuck());
  tracker.reset();
  EXPECT_FALSE(tracker.stuck());
  EXPECT_FALSE(tracker.has_progress_baseline());
  EXPECT_EQ(
    tracker.update(false, {0, 0}, 200.0, 90.0, false),
    workspace_nav::nav_status::StuckEvent::None);
}

TEST(CloudFilter, ParsesBoxesAndRejectsInvalidLength) {
  const auto boxes = workspace_nav::cloud_filter::parse_mask_boxes(
    {2.7, -1.2, -0.4, 3.7, 1.4, 0.65});
  ASSERT_EQ(boxes.size(), 1U);
  EXPECT_TRUE(boxes[0].contains(3.0, 0.0, 0.0));
  EXPECT_FALSE(boxes[0].contains(4.0, 0.0, 0.0));
  EXPECT_THROW(
    workspace_nav::cloud_filter::parse_mask_boxes({1.0, 2.0}), std::invalid_argument);
}

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "livox";
  cloud.height = 1;
  cloud.width = 2;
  cloud.point_step = 16;
  cloud.row_step = 32;
  cloud.is_dense = true;
  const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
  for (std::size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = i * 4;
    field.datatype = i == 3 ? sensor_msgs::msg::PointField::UINT32 :
      sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.data.resize(32);
  const std::array<float, 3> first_point{3.0f, 0.0f, 0.0f};
  const std::array<float, 3> second_point{10.0f, 0.0f, 0.0f};
  std::memcpy(cloud.data.data(), first_point.data(), 12);
  std::memcpy(cloud.data.data() + 16, second_point.data(), 12);
  const uint32_t first_intensity = 7;
  const uint32_t second_intensity = 42;
  std::memcpy(cloud.data.data() + 12, &first_intensity, 4);
  std::memcpy(cloud.data.data() + 28, &second_intensity, 4);
  return cloud;
}

}  // namespace

TEST(CloudFilter, DropsOnlyMaskedPointsAndPreservesBinaryFields) {
  const auto cloud = make_cloud();
  const auto boxes = workspace_nav::cloud_filter::parse_mask_boxes(
    {2.7, -1.2, -0.4, 3.7, 1.4, 0.65});
  const auto output = workspace_nav::cloud_filter::filter_point_cloud(
    cloud, boxes, workspace_nav::cloud_filter::Transform3D::identity());
  ASSERT_EQ(output.width, 1U);
  ASSERT_EQ(output.data.size(), 16U);
  float x = 0.0f;
  uint32_t intensity = 0;
  std::memcpy(&x, output.data.data(), 4);
  std::memcpy(&intensity, output.data.data() + 12, 4);
  EXPECT_FLOAT_EQ(x, 10.0f);
  EXPECT_EQ(intensity, 42U);
  EXPECT_EQ(output.header.frame_id, "livox");
  EXPECT_EQ(output.fields.size(), cloud.fields.size());
}

TEST(CloudFilter, AppliesLockedTransformBeforeMasking) {
  const auto cloud = make_cloud();
  const auto boxes = workspace_nav::cloud_filter::parse_mask_boxes(
    {2.7, -1.2, -0.4, 3.7, 1.4, 0.65});
  auto transform = workspace_nav::cloud_filter::Transform3D::identity();
  transform.translation = {-7.0, 0.0, 0.0};
  const auto output = workspace_nav::cloud_filter::filter_point_cloud(cloud, boxes, transform);
  EXPECT_EQ(output.width, 1U);
  float x = 0.0f;
  std::memcpy(&x, output.data.data(), 4);
  EXPECT_FLOAT_EQ(x, 3.0f);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
