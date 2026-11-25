#include "usv_docking/anchor_filter.hpp"
#include "usv_docking/docking_common.hpp"
#include "usv_docking/motion_control_core.hpp"
#include "usv_docking/safety_core.hpp"
#include "usv_docking/yaw_rate_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace usv_docking
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(PlanarMath, WrapComposeInverseAndQuaternionMatchPython)
{
  EXPECT_NEAR(wrap_angle(kPi), -kPi, 1e-12);
  EXPECT_NEAR(wrap_angle(3.0 * kPi), -kPi, 1e-12);
  EXPECT_NEAR(wrap_angle(-3.0 * kPi), -kPi, 1e-12);
  auto composed = compose_2d({0.0, 0.0, kPi / 2.0}, {1.0, 0.0, 0.0});
  EXPECT_NEAR(composed.x, 0.0, 1e-12);
  EXPECT_NEAR(composed.y, 1.0, 1e-12);

  Pose2D a{1.5, -2.0, 0.7};
  Pose2D b{-0.5, 1.0, -1.2};
  auto roundtrip = compose_2d(inverse_2d(a), compose_2d(a, b));
  EXPECT_NEAR(roundtrip.x, b.x, 1e-12);
  EXPECT_NEAR(roundtrip.y, b.y, 1e-12);
  EXPECT_NEAR(wrap_angle(roundtrip.yaw - b.yaw), 0.0, 1e-12);

  for (double yaw : {-2.9, -1.0, 0.0, 0.5, 3.0}) {
    auto q = yaw_to_quat(yaw);
    EXPECT_NEAR(wrap_angle(quat_to_planar_yaw(q[0], q[1], q[2], q[3]) - yaw), 0.0, 1e-12);
  }
}

TEST(AnchorFilter, SeedNeedsFullConsensusAndUsesMedian)
{
  AnchorParams params;
  params.seed_frames = 3;
  AnchorFilter filter(params);
  EXPECT_FALSE(filter.update({1.0, 2.0, 0.1}, 2.0, 0.0).has_value());
  EXPECT_FALSE(filter.update({1.1, 2.1, 0.1}, 2.0, 0.1).has_value());
  auto anchor = filter.update({0.9, 1.9, 0.1}, 2.0, 0.2);
  ASSERT_TRUE(anchor.has_value());
  EXPECT_NEAR(anchor->x, 1.0, 1e-12);
  EXPECT_NEAR(anchor->y, 2.0, 1e-12);
}

TEST(AnchorFilter, SpreadGateSlidesSeedWindow)
{
  AnchorParams params;
  params.seed_frames = 2;
  params.seed_max_spread_m = 0.4;
  AnchorFilter filter(params);
  filter.update({0.0, 0.0, 0.0}, 2.0, 0.0);
  EXPECT_FALSE(filter.update({2.0, 0.0, 0.0}, 2.0, 0.1).has_value());
  EXPECT_TRUE(filter.update({2.1, 0.0, 0.0}, 2.0, 0.2).has_value());
}

TEST(AnchorFilter, RejectClusterUnlockAdsorbsEma)
{
  AnchorParams params;
  params.seed_frames = 1;
  params.jump_reject_unlock_frames = 2;
  params.max_position_jump = 0.6;
  AnchorFilter filter(params);
  filter.update({0.0, 0.0, 0.0}, 2.0, 0.0);
  EXPECT_NEAR(filter.update({1.0, 0.0, 0.0}, 2.0, 0.1)->x, 0.0, 1e-12);
  auto relocated = filter.update({1.1, 0.0, 0.0}, 2.0, 0.2);
  EXPECT_GT(relocated->x, 0.9);
}

TEST(AnchorFilter, RelocationYawDistanceAndDualTagGuards)
{
  AnchorParams params;
  params.seed_frames = 1;
  params.jump_reject_unlock_frames = 1;
  params.reloc_max_yaw_jump_deg = 10.0;
  AnchorFilter filter(params);
  filter.update({0.0, 0.0, 0.0}, 2.0, 0.0);
  auto held = filter.update({1.0, 0.0, 0.5}, 2.0, 0.1);
  EXPECT_NEAR(held->yaw, 0.0, 1e-12);

  filter.reset();
  filter.update({0.0, 0.0, 0.0}, 2.0, 0.0);
  auto far = filter.update({1.0, 0.0, 0.0}, 5.0, 0.1);
  EXPECT_NEAR(far->x, 0.0, 1e-12);

  filter.reset();
  filter.update({0.0, 0.0, 0.0}, 2.0, 0.0, 2);
  auto single = filter.update({1.0, 0.0, 0.0}, 2.0, 0.1, 1);
  EXPECT_NEAR(single->x, 0.0, 1e-12);
  auto dual = filter.update({1.0, 0.0, 0.0}, 2.0, 0.2, 2);
  EXPECT_GT(dual->x, 0.0);
}

TEST(YawRateEstimator, UsesWrappedOdometryDerivativeAndLowPass)
{
  YawRateParams params;
  params.ema_alpha = 0.35;
  YawRateEstimator estimator(params);
  EXPECT_FALSE(estimator.update(0.0, 0.0).has_value());
  EXPECT_NEAR(estimator.update(0.1, 0.1).value(), 1.0, 1e-12);
  EXPECT_NEAR(estimator.update(0.1, 0.2).value(), 0.65, 1e-12);
}

TEST(YawRateEstimator, DerivativeWrapsAtPiBoundary)
{
  YawRateEstimator estimator;
  estimator.update(kPi - 0.05, 0.0);
  EXPECT_NEAR(estimator.update(-kPi + 0.05, 0.1).value(), 1.0, 1e-12);
}

TEST(YawRateEstimator, TimeoutAndInvalidGapRejectStaleRate)
{
  YawRateParams params;
  params.timeout_sec = 0.3;
  YawRateEstimator estimator(params);
  estimator.update(0.0, 0.0);
  estimator.update(0.1, 0.1);
  EXPECT_TRUE(estimator.value(0.35).has_value());
  EXPECT_FALSE(estimator.value(0.41).has_value());
  EXPECT_FALSE(estimator.update(0.1, 0.7).has_value());
  EXPECT_FALSE(estimator.value(0.71).has_value());
}

class MotionCoreTest : public ::testing::Test
{
protected:
  MotionParams params;
  MotionControlCore core;

  MotionInput input(const std::string & mode, double x, double y, double yaw)
  {
    MotionInput value;
    value.mode = mode;
    value.pose = Pose2D{x, y, yaw};
    value.pose_source = SRC_VISION;
    value.pose_fresh = true;
    value.mode_fresh = true;
    value.safety_fresh = true;
    return value;
  }
};

TEST_F(MotionCoreTest, BackInAndExitControlSignsMatchPython)
{
  auto output = core.compute_back_in(-2.0, 0.2, 0.0, 0.25, 0.12, false, params);
  EXPECT_LT(output.first, 0.0);
  EXPECT_LT(output.second, 0.0);
  output = core.compute_back_in(-2.0, 0.0, 0.1, 0.25, 0.12, false, params);
  EXPECT_LT(output.second, 0.0);
  output = core.compute_exit_forward(0.2, 0.0, 0.3, params);
  EXPECT_GT(output.first, 0.0);
  EXPECT_GT(output.second, 0.0);
  output = core.compute_exit_forward(0.0, 0.1, 0.3, params);
  EXPECT_LT(output.second, 0.0);
}

TEST_F(MotionCoreTest, ApproachIsSternFirstAndAxisTargetIsCapped)
{
  auto output = core.compute_approach(-5.0, 0.0, kPi, params);
  EXPECT_LT(output.first, 0.0);
  output = core.compute_approach(-1.0, 0.0, kPi, params);
  EXPECT_GT(output.first, 0.0);
  core.compute_approach(-10.0, 2.0, kPi, params);
  EXPECT_GE(core.axis_target_x(), params.approach_axis_max_out_x);
}

TEST_F(MotionCoreTest, Gate2StopsTranslationAndFinalPushKeepsReversing)
{
  auto output = core.compute_back_in(-2.0, 0.6, 0.0, 0.25, 0.12, false, params);
  EXPECT_DOUBLE_EQ(output.first, 0.0);
  output = core.compute_back_in(0.0, 0.0, 0.0, 0.15, 0.12, true, params);
  EXPECT_LT(output.first, 0.0);
}

TEST_F(MotionCoreTest, HoldEntryAndSafetyBypassAccelerationRamp)
{
  auto value = input(MODE_BACK_IN, -2.0, 0.0, kPi);
  core.step(value, params);
  value = input(MODE_HOLD, -2.0, 0.0, kPi);
  auto output = core.step(value, params);
  EXPECT_TRUE(output.publish);
  EXPECT_DOUBLE_EQ(output.linear, 0.0);
  EXPECT_DOUBLE_EQ(output.angular, 0.0);

  value = input(MODE_BACK_IN, -2.0, 0.0, kPi);
  core.step(value, params);
  value.safety_stop = true;
  output = core.step(value, params);
  EXPECT_DOUBLE_EQ(output.linear, 0.0);
  EXPECT_DOUBLE_EQ(output.angular, 0.0);
}

TEST_F(MotionCoreTest, InvalidStaleAndNonVisionExitAreSafeZero)
{
  for (const auto & mode : {MODE_APPROACH, MODE_ALIGN, MODE_BACK_IN, MODE_FINAL_DOCK}) {
    auto value = input(mode, -2.0, 0.0, kPi);
    value.pose_source = SRC_INVALID;
    auto output = core.step(value, params);
    EXPECT_DOUBLE_EQ(output.linear, 0.0);
    EXPECT_TRUE(output.publish);
  }
  for (const auto & mode : {MODE_EXIT_FORWARD, MODE_UNDOCK_FORWARD}) {
    auto value = input(mode, -2.0, 0.0, kPi);
    value.pose_source = SRC_PREDICTION;
    auto output = core.step(value, params);
    EXPECT_DOUBLE_EQ(output.linear, 0.0);
    value.pose_source = SRC_VISION;
    value.pose_fresh = false;
    output = core.step(value, params);
    EXPECT_DOUBLE_EQ(output.linear, 0.0);
  }
}

TEST_F(MotionCoreTest, NonHoldModeUsesRampAndHoldBurstThenSilences)
{
  auto value = input(MODE_SEARCH, -2.0, 0.0, kPi);
  value.pose.reset();
  value.pose_source = SRC_INVALID;
  value.pose_fresh = false;
  auto output = core.step(value, params);
  EXPECT_GT(output.angular, 0.0);
  EXPECT_LT(output.angular, params.search_angular_speed);

  value = input(MODE_HOLD, -2.0, 0.0, kPi);
  int published = 0;
  for (int i = 0; i < 100; ++i) {
    output = core.step(value, params);
    published += output.publish ? 1 : 0;
    EXPECT_DOUBLE_EQ(output.linear, 0.0);
    EXPECT_DOUBLE_EQ(output.angular, 0.0);
  }
  EXPECT_LE(published, 25);
}

TEST(SafetyCore, AlignAndApproachUseIndependentPredictionWindows)
{
  SafetyParams params;
  SafetyCore core;
  SafetyInput input;
  input.state = DockState::ALIGN_ENTRY;
  input.pose_source = SRC_PREDICTION;
  input.measurement_age = 4.0;
  input.pose = Pose2D{-2.5, 0.0, kPi};
  for (int i = 0; i < params.violation_cycles + 1; ++i) {
    auto output = core.check(input, i * 0.1, params);
    EXPECT_TRUE(output.abort_reason.empty());
  }
  input.state = DockState::APPROACH_ENTRY;
  input.measurement_age = params.odom_prediction_timeout + 1.0;
  SafetyOutput output;
  for (int i = 0; i < params.violation_cycles + 1; ++i) {
    output = core.check(input, 2.0 + i * 0.1, params);
  }
  EXPECT_EQ(output.abort_reason, "ODOM_TIMEOUT");
}

TEST(SafetyCore, CorridorErrorsDebounceAndExitInvalidOnlyAdvises)
{
  SafetyParams params;
  SafetyCore core;
  SafetyInput input;
  input.state = DockState::BACK_IN;
  input.pose_source = SRC_VISION;
  input.pose = Pose2D{-1.0, params.max_lateral_error + 0.1, kPi};
  SafetyOutput output;
  for (int i = 0; i < params.violation_cycles; ++i) {
    output = core.check(input, i * 0.1, params);
  }
  EXPECT_EQ(output.abort_reason, "LATERAL_ERROR");

  SafetyCore exit_core;
  input.state = DockState::ABORT_EXIT;
  input.pose_source = SRC_INVALID;
  input.measurement_age = std::numeric_limits<double>::infinity();
  output = exit_core.check(input, 20.0, params);
  EXPECT_TRUE(output.abort_reason.empty());
  output = exit_core.check(input, 26.0, params);
  EXPECT_EQ(output.abort_reason, "EXIT_POSE_LOST");
}

TEST(SafetyCore, SilentPoseOrFsmStopsMotion)
{
  SafetyParams params;
  SafetyCore core;
  SafetyInput input;
  input.state = DockState::APPROACH_ENTRY;
  input.pose_source = SRC_VISION;
  input.pose = Pose2D{-2.5, 0.0, kPi};
  input.pose_silent = true;
  auto output = core.check(input, 1.0, params);
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(output.abort_reason, "TOPIC_TIMEOUT:POSE");
  input.pose_silent = false;
  input.fsm_silent = true;
  output = core.check(input, 1.1, params);
  EXPECT_TRUE(output.stop);
  EXPECT_TRUE(output.abort_reason.empty());
}

}  // namespace
}  // namespace usv_docking
