#include "workspace_nav/cloud_filter_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace workspace_nav::cloud_filter
{

bool MaskBox::contains(double x, double y, double z) const
{
  return x >= bounds[0] && x <= bounds[3] && y >= bounds[1] && y <= bounds[4] &&
         z >= bounds[2] && z <= bounds[5];
}

Transform3D Transform3D::identity() {return Transform3D{};}

Eigen::Vector3d Transform3D::apply(double x, double y, double z) const
{
  return rotation * Eigen::Vector3d(x, y, z) + translation;
}

std::vector<MaskBox> parse_mask_boxes(const std::vector<double> & flat)
{
  if (flat.size() % 6 != 0) {
    throw std::invalid_argument("mask_boxes length must be a multiple of 6");
  }
  std::vector<MaskBox> boxes;
  for (std::size_t i = 0; i < flat.size(); i += 6) {
    boxes.push_back({{flat[i], flat[i + 1], flat[i + 2],
      flat[i + 3], flat[i + 4], flat[i + 5]}});
  }
  return boxes;
}

namespace
{

struct XYZFields
{
  std::size_t x;
  std::size_t y;
  std::size_t z;
  std::size_t element_size;
  bool double_precision;
  bool big_endian;
};

std::size_t field_offset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto field = std::find_if(
    cloud.fields.begin(), cloud.fields.end(), [&](const auto & candidate) {
      return candidate.name == name;
    });
  if (field == cloud.fields.end()) throw std::invalid_argument("PointCloud2 missing " + name);
  if (field->datatype != sensor_msgs::msg::PointField::FLOAT32 &&
    field->datatype != sensor_msgs::msg::PointField::FLOAT64)
  {
    throw std::invalid_argument("PointCloud2 XYZ fields must be FLOAT32/FLOAT64");
  }
  return field->offset;
}

template<typename T>
T read_scalar(const uint8_t * data, bool reverse)
{
  std::array<uint8_t, sizeof(T)> bytes;
  if (reverse) {
    std::reverse_copy(data, data + sizeof(T), bytes.begin());
  } else {
    std::copy(data, data + sizeof(T), bytes.begin());
  }
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

double read_coordinate(
  const uint8_t * data, const XYZFields & fields, std::size_t offset)
{
  const uint8_t * position = data + offset;
  if (fields.double_precision) {
    return read_scalar<double>(position, fields.big_endian);
  }
  return read_scalar<float>(position, fields.big_endian);
}

}  // namespace

sensor_msgs::msg::PointCloud2 filter_point_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const std::vector<MaskBox> & boxes,
  const Transform3D & transform)
{
  if (boxes.empty() || input.width == 0 || input.height == 0) return input;
  XYZFields fields{
    field_offset(input, "x"), field_offset(input, "y"), field_offset(input, "z"),
    0, false, input.is_bigendian};
  const auto x_field = std::find_if(
    input.fields.begin(), input.fields.end(), [](const auto & field) {return field.name == "x";});
  fields.double_precision = x_field->datatype == sensor_msgs::msg::PointField::FLOAT64;
  fields.element_size = fields.double_precision ? 8 : 4;

  sensor_msgs::msg::PointCloud2 output = input;
  output.height = 1;
  output.width = 0;
  output.row_step = 0;
  output.data.clear();
  output.data.reserve(static_cast<std::size_t>(input.width) * input.height * input.point_step);
  bool dropped = false;
  for (std::size_t row = 0; row < input.height; ++row) {
    const uint8_t * row_data = input.data.data() + row * input.row_step;
    for (std::size_t column = 0; column < input.width; ++column) {
      const uint8_t * point = row_data + column * input.point_step;
      const double x = read_coordinate(point, fields, fields.x);
      const double y = read_coordinate(point, fields, fields.y);
      const double z = read_coordinate(point, fields, fields.z);
      const bool finite = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
      bool keep = finite || input.is_dense;
      if (keep && finite) {
        const auto base = transform.apply(x, y, z);
        for (const auto & box : boxes) {
          if (box.contains(base.x(), base.y(), base.z())) {
            keep = false;
            break;
          }
        }
      }
      if (keep) {
        output.data.insert(output.data.end(), point, point + input.point_step);
        ++output.width;
      } else {
        dropped = true;
      }
    }
  }
  if (!dropped) return input;
  output.row_step = output.width * output.point_step;
  return output;
}

}  // namespace workspace_nav::cloud_filter
