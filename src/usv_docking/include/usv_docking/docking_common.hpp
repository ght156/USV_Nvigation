#ifndef USV_DOCKING__DOCKING_COMMON_HPP_
#define USV_DOCKING__DOCKING_COMMON_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace usv_docking
{

constexpr double PI = 3.14159265358979323846;

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct DockState
{
  static constexpr const char * IDLE = "IDLE";
  static constexpr const char * ACQUIRE_TAG = "ACQUIRE_TAG";
  static constexpr const char * APPROACH_ENTRY = "APPROACH_ENTRY";
  static constexpr const char * ALIGN_ENTRY = "ALIGN_ENTRY";
  static constexpr const char * BACK_IN = "BACK_IN";
  static constexpr const char * FINAL_DOCK = "FINAL_DOCK";
  static constexpr const char * DOCKED = "DOCKED";
  static constexpr const char * REACQUIRE_TAG = "REACQUIRE_TAG";
  static constexpr const char * ABORT_EXIT = "ABORT_EXIT";
  static constexpr const char * FAILED = "FAILED";
  static constexpr const char * UNDOCK_EXIT = "UNDOCK_EXIT";
  static constexpr const char * UNDOCK_SETTLE = "UNDOCK_SETTLE";
  static constexpr const char * WAIT_DOCK_OPEN = "WAIT_DOCK_OPEN";
  static constexpr const char * WAIT_DOCK_RELEASE = "WAIT_DOCK_RELEASE";
};

inline constexpr const char * MODE_HOLD = "HOLD";
inline constexpr const char * MODE_SEARCH = "SEARCH";
inline constexpr const char * MODE_APPROACH = "APPROACH";
inline constexpr const char * MODE_ALIGN = "ALIGN";
inline constexpr const char * MODE_BACK_IN = "BACK_IN";
inline constexpr const char * MODE_FINAL_DOCK = "FINAL_DOCK";
inline constexpr const char * MODE_SEARCH_LIMITED = "SEARCH_LIMITED";
inline constexpr const char * MODE_EXIT_FORWARD = "EXIT_FORWARD";
inline constexpr const char * MODE_UNDOCK_FORWARD = "UNDOCK_FORWARD";
inline constexpr const char * MODE_DOCKED_HOLD = "DOCKED_HOLD";

inline constexpr const char * SRC_VISION = "VISION";
inline constexpr const char * SRC_PREDICTION = "ODOM_PREDICTION";
inline constexpr const char * SRC_INVALID = "INVALID";

double wrap_angle(double angle);
double yaw_from_quat(double z, double w);
Pose2D compose_2d(const Pose2D & a, const Pose2D & b);
Pose2D inverse_2d(const Pose2D & pose);
double quat_to_planar_yaw(double qx, double qy, double qz, double qw);
std::array<double, 4> yaw_to_quat(double yaw);

template<typename T>
T clamp_value(T value, T limit)
{
  return std::max(-limit, std::min(limit, value));
}

}  // namespace usv_docking

#endif  // USV_DOCKING__DOCKING_COMMON_HPP_
