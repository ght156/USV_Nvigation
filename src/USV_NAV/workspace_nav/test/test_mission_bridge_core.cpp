#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "workspace_nav/mission_bridge_core.hpp"

namespace mb = workspace_nav::mission_bridge;

TEST(MissionBridgeCore, ParsesLegacyAndExplicitWaypointMessages) {
  const auto legacy = mb::parse_waypoint_message(
    R"([[31.0, 120.0], {"latitude": 31.0001, "longitude": 120.0001}])");
  ASSERT_TRUE(legacy.has_value());
  ASSERT_EQ(legacy->waypoints.size(), 2U);
  EXPECT_FALSE(legacy->explicit_replan);
  EXPECT_DOUBLE_EQ(legacy->waypoints[1].latitude, 31.0001);

  const auto explicit_route = mb::parse_waypoint_message(
    R"(data: {"waypoints": [[31.0, 120.0]], "command": "replan", "mission_id": " m-7 ", "command_id": " c-9 "})");
  ASSERT_TRUE(explicit_route.has_value());
  EXPECT_TRUE(explicit_route->explicit_replan);
  EXPECT_EQ(explicit_route->mission_id, "m-7");
  EXPECT_EQ(explicit_route->command_id, "c-9");
}

TEST(MissionBridgeCore, RejectsInvalidWaypointPayloads) {
  EXPECT_FALSE(mb::parse_waypoint_message("not json").has_value());
  EXPECT_FALSE(mb::parse_waypoint_message(R"({"waypoints": []})").has_value());
  EXPECT_FALSE(mb::parse_waypoint_message(R"([[0.0, 0.0]])").has_value());
  EXPECT_FALSE(mb::parse_waypoint_message(R"([[91.0, 120.0]])").has_value());
  EXPECT_FALSE(mb::parse_waypoint_message(R"([[31.0, "bad"]])").has_value());
}

TEST(MissionBridgeCore, ConvertsGeodeticDeltaAndMapOrigin) {
  const auto delta = mb::geodetic_delta_enu_m(31.0, 120.0, 31.0, 120.001);
  EXPECT_NEAR(delta.x, 95.419427, 0.00001);
  EXPECT_NEAR(delta.y, 0.0, 1e-9);

  const auto point = mb::enu_delta_to_map_xy(10.0, 20.0, 100.0, 200.0, M_PI_2);
  EXPECT_NEAR(point.x, 80.0, 1e-9);
  EXPECT_NEAR(point.y, 210.0, 1e-9);
}

TEST(MissionBridgeCore, UtmConversionMatchesKnownWgs84Point) {
  const auto utm = mb::lat_lon_to_utm(31.0, 120.0);
  EXPECT_EQ(utm.zone, 51);
  EXPECT_TRUE(utm.northern);
  EXPECT_NEAR(utm.easting, 213539.919, 0.1);
  EXPECT_NEAR(utm.northing, 3433466.669, 0.1);
}

TEST(MissionBridgeCore, NearWaypointOnlySkipsWhenYawIsUnspecified) {
  EXPECT_TRUE(mb::should_skip_waypoint(0.5, 1.0, mb::kYawUnspecified));
  EXPECT_FALSE(mb::should_skip_waypoint(0.5, 1.0, 90.0));
  EXPECT_FALSE(mb::should_skip_waypoint(1.5, 1.0, mb::kYawUnspecified));
  EXPECT_TRUE(mb::yaw_is_specified(-360.0));
  EXPECT_TRUE(mb::yaw_is_specified(360.0));
  EXPECT_FALSE(mb::yaw_is_specified(360.1));
  EXPECT_FALSE(mb::yaw_is_specified(NAN));
}

TEST(MissionBridgeCore, MissionHashIsStableAndMissionIdSensitive) {
  const std::vector<mb::LatLon> route{{31.0, 120.0}, {31.0001, 120.0001}};
  EXPECT_EQ(mb::waypoint_mission_hash(route, "m-1"), mb::waypoint_mission_hash(route, "m-1"));
  EXPECT_NE(mb::waypoint_mission_hash(route, "m-1"), mb::waypoint_mission_hash(route, "m-2"));
  EXPECT_EQ(mb::waypoint_mission_hash(route, "").size(), 64U);
}

TEST(MissionBridgeCore, FenceChecksInclusionBeforeExclusion) {
  const mb::Fence inclusion{
    "work", "inclusion", "polygon", {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}},
    std::nullopt, 0.0};
  const mb::Fence exclusion{
    "keepout", "exclusion", "circle", {}, mb::Point{5.0, 5.0}, 1.0};
  const std::vector<mb::Fence> fences{inclusion, exclusion};

  EXPECT_EQ(mb::fence_violation_index({20.0, 5.0}, fences), 0U);
  EXPECT_EQ(mb::fence_violation_index({5.0, 5.0}, fences), 1U);
  EXPECT_FALSE(mb::fence_violation_index({2.0, 2.0}, fences).has_value());
}

TEST(MissionBridgeCore, CtrlStuckWatchdogRequiresContinuousRecentFault) {
  mb::CtrlStuckWatchdog watchdog(30.0);
  watchdog.update(mb::MissionState::Running, true, 100.0);
  watchdog.update(mb::MissionState::Running, true, 129.9);
  EXPECT_FALSE(watchdog.should_fail(129.9));
  watchdog.update(mb::MissionState::Running, true, 130.0);
  EXPECT_TRUE(watchdog.should_fail(130.0));

  watchdog.update(mb::MissionState::Running, false, 130.0);
  EXPECT_FALSE(watchdog.should_fail(200.0));

  watchdog.update(mb::MissionState::Running, true, 100.0);
  EXPECT_FALSE(watchdog.should_fail(200.0));
  watchdog.update(mb::MissionState::Paused, true, 200.0);
  EXPECT_FALSE(watchdog.should_fail(300.0));
}

TEST(MissionBridgeCore, MapsNav2FailuresToNavigateTaskResultCodes) {
  EXPECT_EQ(mb::map_nav2_failure(0, "planner failed to find path"), 5);
  EXPECT_EQ(mb::map_nav2_failure(0, "controller stuck after collision"), 6);
  EXPECT_EQ(mb::map_nav2_failure(150, ""), 5);
  EXPECT_EQ(mb::map_nav2_failure(250, ""), 6);
  EXPECT_EQ(mb::map_nav2_failure(0, "unknown"), 99);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
