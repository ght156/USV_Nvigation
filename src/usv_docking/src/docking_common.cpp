#include "usv_docking/docking_common.hpp"

namespace usv_docking
{

double wrap_angle(double angle)
{
  double wrapped = std::fmod(angle + PI, 2.0 * PI);
  if (wrapped < 0.0) wrapped += 2.0 * PI;
  return wrapped - PI;
}

double yaw_from_quat(double z, double w)
{
  return std::atan2(2.0 * z * w, 1.0 - 2.0 * z * z);
}

Pose2D compose_2d(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap_angle(a.yaw + b.yaw)};
}

Pose2D inverse_2d(const Pose2D & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {-c * pose.x - s * pose.y, s * pose.x - c * pose.y, wrap_angle(-pose.yaw)};
}

double quat_to_planar_yaw(double qx, double qy, double qz, double qw)
{
  const double vx = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double vy = 2.0 * (qx * qy + qz * qw);
  return std::atan2(vy, vx);
}

std::array<double, 4> yaw_to_quat(double yaw)
{
  return {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
}

}  // namespace usv_docking
