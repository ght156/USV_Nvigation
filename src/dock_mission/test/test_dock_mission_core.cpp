#include "dock_mission/bay_loader.hpp"
#include "dock_mission/dock_task_api.hpp"
#include "dock_mission/gnss_staging.hpp"
#include "dock_mission/mission_logic.hpp"
#include "dock_mission/task_event.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace dock_mission
{
namespace
{

class TempFile
{
public:
  explicit TempFile(const std::string & text)
  {
    static int sequence = 0;
    path_ = std::filesystem::temp_directory_path() /
      ("dock_mission_cpp_test_" + std::to_string(++sequence) + ".yaml");
    std::ofstream(path_) << text;
  }
  ~TempFile() {std::filesystem::remove(path_);}
  const std::filesystem::path & path() const {return path_;}
private:
  std::filesystem::path path_;
};

TEST(TaskEvent, ParsesJsonAndLegacyPayloads)
{
  auto parsed = parse_task_event(
    R"({"event":"TASK_COMPLETED","detail":{"mission_id":"dock_staging"}})");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->first, "TASK_COMPLETED");
  EXPECT_EQ(parsed->second.at("mission_id"), "dock_staging");

  parsed = parse_task_event(R"({"event":"TASK_FAILED","detail":"bad"})");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->first, "TASK_FAILED");
  EXPECT_TRUE(parsed->second.empty());

  EXPECT_EQ(parse_task_event("something TASK_COMPLETED happened")->first, "TASK_COMPLETED");
  EXPECT_EQ(parse_task_event("TASK_FAILED nav")->first, "TASK_FAILED");
  EXPECT_FALSE(parse_task_event("not an event").has_value());
  EXPECT_FALSE(parse_task_event("").has_value());
}

TEST(DockTaskApi, ParsesFullCommandWithoutChangingDegreeYaw)
{
  auto command = parse_gcs_command(
    R"({"action":"one_click_dock","mission_id":"m001","command_id":"c1","dock_lat":"30.123456","dock_lon":"120.123456","dock_yaw":"90"})");
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->command, DockCommand::ONE_CLICK_DOCK);
  EXPECT_EQ(command->mission_id, "m001");
  EXPECT_EQ(command->command_id, "c1");
  ASSERT_TRUE(command->has_dock_point());
  EXPECT_DOUBLE_EQ(command->dock_yaw.value(), 90.0);
  EXPECT_DOUBLE_EQ(command->dock_lat.value(), 30.123456);
}

TEST(DockTaskApi, KeepsLegacyAndSentinelCommandsCompatible)
{
  auto legacy = parse_gcs_command(
    R"({"action":"dock_only","mission_id":"m","command_id":"c"})");
  ASSERT_TRUE(legacy.has_value());
  EXPECT_EQ(legacy->command, DockCommand::DOCK_ONLY);
  EXPECT_FALSE(legacy->has_dock_point());

  auto sentinel = parse_gcs_command(
    R"({"action":"one_click_dock","dock_lat":65536.0,"dock_lon":120.4,"dock_yaw":65536.0})");
  ASSERT_TRUE(sentinel.has_value());
  EXPECT_FALSE(sentinel->dock_lat.has_value());
  EXPECT_DOUBLE_EQ(sentinel->dock_lon.value(), 120.4);
  EXPECT_DOUBLE_EQ(sentinel->dock_yaw.value(), DOCK_YAW_UNSPECIFIED);
  EXPECT_FALSE(sentinel->has_dock_point());

  EXPECT_FALSE(parse_gcs_command(R"({"action":"fly_away"})").has_value());
  EXPECT_FALSE(parse_gcs_command("not json").has_value());
}

TEST(DockTaskApi, RejectsOnlyTrulyInvalidCoordinates)
{
  EXPECT_TRUE(is_valid_dock_point(30.1, 120.1));
  EXPECT_FALSE(is_valid_dock_point(0.0, 0.0));
  EXPECT_FALSE(is_valid_dock_point(95.0, 120.0));
  EXPECT_FALSE(is_valid_dock_point(30.0, -181.0));
}

TEST(DockTaskApi, BuildsUpperLayerStatusAndEventsWithOriginalSchema)
{
  TaskStatus status;
  status.state = MissionState::MONITOR_DOCK;
  status.mode = "dock";
  status.dock_active = true;
  status.retry_count = 2;
  status.retry_max = 5;
  status.mission_id = "m";
  status.command_id = "c";
  status.elapsed_sec = 12.345;
  status.odom_speed = 0.12349;
  auto json = build_task_status(status);
  EXPECT_EQ(json.at("schema_version"), 1);
  EXPECT_EQ(json.at("run_state"), "RUNNING");
  EXPECT_EQ(json.at("phase"), "DOCKING");
  EXPECT_EQ(json.at("dock_active"), true);
  EXPECT_EQ(json.at("retry_count"), 2);
  EXPECT_EQ(json.at("elapsed_sec"), 12.35);
  EXPECT_EQ(json.at("odom_speed"), 0.123);

  auto event = build_task_event("DOCK_SUCCEEDED", "m", "c", {{"reason", "ok"}});
  EXPECT_EQ(event.at("event"), "DOCK_SUCCEEDED");
  EXPECT_EQ(event.at("mission_id"), "m");
  EXPECT_EQ(event.at("detail").at("reason"), "ok");
}

TEST(BayLoader, LoadsOriginalDatabaseFieldsAndDefaults)
{
  TempFile file(R"(
bays:
  bay2:
    name: Test bay2
    standoff_m: 4.0
    map_staging: {x: 3.5, y: 0.0, yaw: 0.0}
    gnss_staging: {latitude: 31.48618175, longitude: 120.36797148, yaw_deg: 0.0}
  minimal:
    name: minimal
)");
  auto bay = load_bay("bay2", file.path());
  EXPECT_EQ(bay.bay_id, "bay2");
  EXPECT_EQ(bay.name, "Test bay2");
  EXPECT_DOUBLE_EQ(bay.map_pose.x, 3.5);
  EXPECT_DOUBLE_EQ(bay.gnss.longitude, 120.36797148);
  EXPECT_THROW(load_bay("unknown", file.path()), std::out_of_range);

  auto minimal = load_bay("minimal", file.path());
  EXPECT_DOUBLE_EQ(minimal.standoff_m, 4.0);
  EXPECT_DOUBLE_EQ(minimal.gnss.latitude, 0.0);
  EXPECT_DOUBLE_EQ(minimal.map_pose.x, 0.0);
}

TEST(GnssStaging, PreservesDatumAndMapInverseBehavior)
{
  TempFile map(R"(
image: map.pgm
resolution: 1.0
origin: [0.0, 0.0, 0.0]
ref_gnss_10: [120.36793468, 31.48618175]
)");
  auto datum = map_xy_to_gnss(0.0, 0.0, map.path());
  EXPECT_NEAR(datum.latitude, 31.48618175, 1e-8);
  EXPECT_NEAR(datum.longitude, 120.36793468, 1e-8);

  BayRecord bay;
  bay.bay_id = "bay2";
  bay.gnss = {31.48618175, 120.36797148, 90.0};
  bay.map_pose = {3.5, 0.0, 0.5};
  auto direct = make_staging_waypoint(bay, true);
  EXPECT_DOUBLE_EQ(direct[0], bay.gnss.latitude);
  EXPECT_DOUBLE_EQ(direct[1], bay.gnss.longitude);
  EXPECT_DOUBLE_EQ(direct[2], 90.0);
  auto inverse = make_staging_waypoint(bay, false, map.path());
  EXPECT_NEAR(inverse[0], 31.48618175, 1e-8);
  EXPECT_GT(inverse[1], 120.36793468);
  EXPECT_NEAR(inverse[2], 28.6478897565, 1e-8);

  bay.gnss = {0.0, 0.0, 0.0};
  EXPECT_THROW(make_staging_waypoint(bay, true), std::invalid_argument);
}

TEST(MissionLogic, InDockRequiresFreshValidAnchor)
{
  InDockLimits limits;
  InDockSnapshot pose;
  pose.xy = {0.0, 0.0};
  pose.pose_age = 0.1;
  pose.source = "VISION";
  pose.anchor_status = "UNKNOWN";
  pose.anchor_age = 0.1;
  auto result = evaluate_in_dock(pose, limits);
  EXPECT_FALSE(result.pose_valid);
  EXPECT_FALSE(result.in_dock);
  EXPECT_NE(result.message.find("UNKNOWN"), std::string::npos);

  pose.anchor_status = "VALID";
  result = evaluate_in_dock(pose, limits);
  EXPECT_TRUE(result.pose_valid);
  EXPECT_TRUE(result.in_dock);

  pose.xy = {-3.5, 0.1};
  EXPECT_FALSE(evaluate_in_dock(pose, limits).in_dock);
  pose.xy = {0.0, 1.2};
  EXPECT_FALSE(evaluate_in_dock(pose, limits).in_dock);
}

TEST(MissionLogic, InDockRejectsStalePredictionAndPose)
{
  InDockLimits limits;
  InDockSnapshot pose;
  pose.xy = {0.1, -0.1};
  pose.pose_age = 0.1;
  pose.source = "ODOM_PREDICTION";
  pose.anchor_status = "VALID";
  pose.anchor_age = 0.1;
  pose.measurement_age = 2.0;
  pose.measurement_age_stamp_age = 0.1;
  EXPECT_TRUE(evaluate_in_dock(pose, limits).pose_valid);
  pose.measurement_age = 10.0;
  auto result = evaluate_in_dock(pose, limits);
  EXPECT_FALSE(result.pose_valid);
  EXPECT_NE(result.message.find("过期"), std::string::npos);
  pose.measurement_age = 0.0;
  pose.pose_age = 10.0;
  EXPECT_FALSE(evaluate_in_dock(pose, limits).pose_valid);
}

TEST(MissionLogic, SettleNeedsSeveralLowRateStillFrames)
{
  SettleParams params;
  params.settle_sec = 0.0;
  SettleMonitor monitor(params);
  EXPECT_FALSE(monitor.ready(0.0));
  monitor.observe(0.0, 0.0, 0.0);
  monitor.observe(0.0, 0.0, 0.5);
  EXPECT_FALSE(monitor.ready(0.5));
  monitor.observe(0.0, 0.0, 1.0);
  EXPECT_TRUE(monitor.ready(1.0));

  monitor.reset();
  monitor.observe(0.0, 0.0, 0.0);
  monitor.observe(0.0, 0.0, 0.2);
  monitor.observe(0.0, 0.0, 1.4);
  EXPECT_FALSE(monitor.ready(1.4));

  monitor.reset();
  monitor.observe(0.0, 0.2, 0.0);
  monitor.observe(0.0, 0.2, 0.5);
  monitor.observe(0.0, 0.2, 1.0);
  EXPECT_FALSE(monitor.ready(1.0));
}

}  // namespace
}  // namespace dock_mission
