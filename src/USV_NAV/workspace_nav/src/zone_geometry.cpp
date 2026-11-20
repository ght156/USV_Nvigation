#include "workspace_nav/zone_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace workspace_nav::zone_geometry
{

Fill Fill::polygon(std::vector<Point> points)
{
  Fill fill;
  fill.points = std::move(points);
  return fill;
}

Fill Fill::circle(Point center, double radius)
{
  Fill fill;
  fill.type = Type::Circle;
  fill.center = center;
  fill.radius = radius;
  return fill;
}

bool point_in_polygon(const Point & point, const std::vector<Point> & vertices)
{
  if (vertices.size() < 3) return false;
  bool inside = false;
  std::size_t previous = vertices.size() - 1;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const auto & a = vertices[i];
    const auto & b = vertices[previous];
    if ((a.y > point.y) != (b.y > point.y)) {
      const double crossing = (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
      if (point.x < crossing) inside = !inside;
    }
    previous = i;
  }
  return inside;
}

std::size_t fill_polygon_grid(
  std::vector<int8_t> & mask, int width, int height,
  const std::vector<Point> & points, int8_t value)
{
  if (points.size() < 3) return 0;
  const auto minmax_x = std::minmax_element(
    points.begin(), points.end(), [](const Point & a, const Point & b) {return a.x < b.x;});
  const auto minmax_y = std::minmax_element(
    points.begin(), points.end(), [](const Point & a, const Point & b) {return a.y < b.y;});
  const int min_x = std::max(0, static_cast<int>(minmax_x.first->x));
  const int max_x = std::min(width - 1, static_cast<int>(minmax_x.second->x) + 1);
  const int min_y = std::max(0, static_cast<int>(minmax_y.first->y));
  const int max_y = std::min(height - 1, static_cast<int>(minmax_y.second->y) + 1);
  std::size_t changed = 0;
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      if (!point_in_polygon({x + 0.5, y + 0.5}, points)) continue;
      const std::size_t index = y * width + x;
      if (mask[index] != value) {
        mask[index] = value;
        ++changed;
      }
    }
  }
  return changed;
}

std::size_t fill_circle_grid(
  std::vector<int8_t> & mask, int width, int height,
  const Point & center, double radius_cells, int8_t value)
{
  if (radius_cells <= 0.0) return 0;
  const double radius_squared = radius_cells * radius_cells;
  const int min_x = std::max(0, static_cast<int>(center.x - radius_cells));
  const int max_x = std::min(width - 1, static_cast<int>(center.x + radius_cells) + 1);
  const int min_y = std::max(0, static_cast<int>(center.y - radius_cells));
  const int max_y = std::min(height - 1, static_cast<int>(center.y + radius_cells) + 1);
  std::size_t changed = 0;
  for (int y = min_y; y <= max_y; ++y) {
    const double dy = y + 0.5 - center.y;
    for (int x = min_x; x <= max_x; ++x) {
      const double dx = x + 0.5 - center.x;
      if (dx * dx + dy * dy > radius_squared) continue;
      const std::size_t index = y * width + x;
      if (mask[index] != value) {
        mask[index] = value;
        ++changed;
      }
    }
  }
  return changed;
}

std::size_t draw_polyline_grid(
  std::vector<int8_t> & mask, int width, int height,
  const std::vector<Point> & points, int8_t value, int dilation_cells)
{
  if (points.size() < 2) return 0;
  const int dilation = std::max(0, dilation_cells);
  std::size_t changed = 0;
  const auto stamp = [&](int center_x, int center_y) {
      for (int y = center_y - dilation; y <= center_y + dilation; ++y) {
        if (y < 0 || y >= height) continue;
        for (int x = center_x - dilation; x <= center_x + dilation; ++x) {
          if (x < 0 || x >= width) continue;
          const std::size_t index = y * width + x;
          if (mask[index] != value) {
            mask[index] = value;
            ++changed;
          }
        }
      }
    };
  for (std::size_t segment = 0; segment + 1 < points.size(); ++segment) {
    int x0 = static_cast<int>(std::lround(points[segment].x));
    int y0 = static_cast<int>(std::lround(points[segment].y));
    const int x1 = static_cast<int>(std::lround(points[segment + 1].x));
    const int y1 = static_cast<int>(std::lround(points[segment + 1].y));
    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
      stamp(x0, y0);
      if (x0 == x1 && y0 == y1) break;
      const int doubled = 2 * error;
      if (doubled >= dy) {
        error += dy;
        x0 += sx;
      }
      if (doubled <= dx) {
        error += dx;
        y0 += sy;
      }
    }
  }
  return changed;
}

std::vector<int8_t> build_fence_mask(
  int width, int height,
  const std::vector<Fill> & inclusion_fills,
  const std::vector<Fill> & exclusion_fills,
  const std::vector<std::vector<Point>> & boundaries,
  int boundary_dilation_cells)
{
  std::vector<int8_t> mask(width * height, inclusion_fills.empty() ? kFree : kKeepout);
  const auto apply = [&](const Fill & fill, int8_t value) {
      if (fill.type == Fill::Type::Polygon) {
        fill_polygon_grid(mask, width, height, fill.points, value);
      } else {
        fill_circle_grid(mask, width, height, fill.center, fill.radius, value);
      }
    };
  for (const auto & fill : inclusion_fills) apply(fill, kFree);
  for (const auto & fill : exclusion_fills) apply(fill, kKeepout);
  for (const auto & boundary : boundaries) {
    draw_polyline_grid(mask, width, height, boundary, kKeepout, boundary_dilation_cells);
  }
  return mask;
}

namespace
{

double point_segment_distance(const Point & point, const Point & a, const Point & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  if (dx == 0.0 && dy == 0.0) return std::hypot(point.x - a.x, point.y - a.y);
  const double ratio = std::clamp(
    ((point.x - a.x) * dx + (point.y - a.y) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
  return std::hypot(point.x - (a.x + ratio * dx), point.y - (a.y + ratio * dy));
}

}  // namespace

double distance_to_fence(const Point & point, const mission_bridge::Fence & fence)
{
  if (fence.shape == "circle" && fence.center) {
    return std::abs(std::hypot(point.x - fence.center->x, point.y - fence.center->y) -
      fence.radius_m);
  }
  if (fence.points.size() < 2) return 0.0;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < fence.points.size(); ++i) {
    distance = std::min(
      distance, point_segment_distance(point, fence.points[i],
      fence.points[(i + 1) % fence.points.size()]));
  }
  return distance;
}

}  // namespace workspace_nav::zone_geometry
