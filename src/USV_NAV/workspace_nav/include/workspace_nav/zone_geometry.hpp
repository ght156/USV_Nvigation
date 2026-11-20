#ifndef WORKSPACE_NAV__ZONE_GEOMETRY_HPP_
#define WORKSPACE_NAV__ZONE_GEOMETRY_HPP_

#include <cstdint>
#include <vector>

#include "workspace_nav/mission_bridge_core.hpp"

namespace workspace_nav::zone_geometry
{

using mission_bridge::Point;

inline constexpr int8_t kKeepout = 100;
inline constexpr int8_t kFree = 0;

struct Fill
{
  enum class Type {Polygon, Circle};
  Type type{Type::Polygon};
  std::vector<Point> points;
  Point center{0.0, 0.0};
  double radius{0.0};

  static Fill polygon(std::vector<Point> points);
  static Fill circle(Point center, double radius);
};

bool point_in_polygon(const Point & point, const std::vector<Point> & vertices);
std::size_t fill_polygon_grid(
  std::vector<int8_t> & mask, int width, int height,
  const std::vector<Point> & points, int8_t value);
std::size_t fill_circle_grid(
  std::vector<int8_t> & mask, int width, int height,
  const Point & center, double radius_cells, int8_t value);
std::size_t draw_polyline_grid(
  std::vector<int8_t> & mask, int width, int height,
  const std::vector<Point> & points, int8_t value, int dilation_cells = 0);
std::vector<int8_t> build_fence_mask(
  int width, int height,
  const std::vector<Fill> & inclusion_fills,
  const std::vector<Fill> & exclusion_fills,
  const std::vector<std::vector<Point>> & boundaries,
  int boundary_dilation_cells = 2);
double distance_to_fence(const Point & point, const mission_bridge::Fence & fence);

}  // namespace workspace_nav::zone_geometry

#endif  // WORKSPACE_NAV__ZONE_GEOMETRY_HPP_
