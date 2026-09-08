// Broadcast map→odom using map.yaml ref_gnss + BW GI320 GNGGA/INSPVAA.
// Only publishes map→odom (odom→base_link left to MAVROS).
// Re-locks when GNSS degrades then recovers, map vs GNSS mismatch,
// FC local origin jumps, or prolonged IMU-only coasting.

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <yaml-cpp/yaml.h>

#include "bw_gi320_driver/msg/gi320_gngga.hpp"
#include "bw_gi320_driver/msg/gi320_inspvaa.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace usv_localization
{
namespace
{

constexpr double kWgs84A = 6378137.0;

struct LatLon
{
  double lat{0.0};
  double lon{0.0};
};

bool load_ref_gnss(
  const std::string & yaml_path, const std::string & ref_key, LatLon & out, std::string & err)
{
  try {
    YAML::Node cfg = YAML::LoadFile(yaml_path);
    if (!cfg || !cfg.IsMap()) {
      err = "map yaml root is not a mapping: " + yaml_path;
      return false;
    }
    if (!cfg[ref_key]) {
      err = "key not found: " + ref_key;
      return false;
    }
    const YAML::Node arr = cfg[ref_key];
    if (!arr.IsSequence() || arr.size() < 2) {
      err = ref_key + " must be [longitude, latitude]";
      return false;
    }
    out.lon = arr[0].as<double>();
    out.lat = arr[1].as<double>();
    return true;
  } catch (const std::exception & e) {
    err = e.what();
    return false;
  }
}

void latlon_to_enu(
  double lat_deg, double lon_deg, double lat0_deg, double lon0_deg, double & east, double & north)
{
  const double dlat = (lat_deg - lat0_deg) * M_PI / 180.0;
  const double dlon = (lon_deg - lon0_deg) * M_PI / 180.0;
  north = dlat * kWgs84A;
  east = dlon * kWgs84A * std::cos(lat0_deg * M_PI / 180.0);
}

inline double wrap_pi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

inline double quat_to_yaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

class UsvMapOdomTfNode : public rclcpp::Node
{
public:
  UsvMapOdomTfNode()
  : Node("usv_map_odom_tf")
  {
    map_config_yaml_ = declare_parameter<std::string>("map_config_yaml", "");
    map_origin_ref_key_ = declare_parameter<std::string>("map_origin_ref_key", "ref_gnss_10");
    const double manual_lat = declare_parameter<double>("map_origin_latitude", 0.0);
    const double manual_lon = declare_parameter<double>("map_origin_longitude", 0.0);

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

    gngga_topic_ = declare_parameter<std::string>("gngga_topic", "/gi320/gngga");
    inspvaa_topic_ = declare_parameter<std::string>("inspvaa_topic", "/gi320/inspvaa");
    fc_local_odom_topic_ =
      declare_parameter<std::string>("fc_local_odom_topic", "/mavros/gps_input/local");

    // 栅格轴相对 ENU 的固定偏角（只用于经纬度→map 平面；不等于锁定时 map→odom 航向）
    map_odom_yaw_deg_ = declare_parameter<double>("map_odom_yaw_deg", 0.0);
    // 锁定 map→odom 时写入初始化航向：yaw_map_odom = yaw_gnss_map - yaw_fc_odom
    use_gnss_yaw_at_lock_ = declare_parameter<bool>("use_gnss_yaw_at_lock", true);
    // GI320 yaw：0°=北、CCW；ROS ENU：0°=东、CCW → 默认 -90°
    gi320_yaw_to_enu_offset_deg_ =
      declare_parameter<double>("gi320_yaw_to_enu_offset_deg", -90.0);
    republish_hz_ = declare_parameter<double>("republish_hz", 20.0);

    min_fix_type_ = static_cast<uint8_t>(declare_parameter<int>("min_fix_type", 4));
    imu_only_fix_type_ = static_cast<uint8_t>(declare_parameter<int>("imu_only_fix_type", 6));
    max_hdop_ = declare_parameter<double>("max_hdop", 2.5);

    map_gnss_mismatch_m_ = declare_parameter<double>("map_gnss_mismatch_m", 5.0);
    fc_origin_jump_m_ = declare_parameter<double>("fc_origin_jump_m", 5.0);
    gnss_degrade_hold_s_ = declare_parameter<double>("gnss_degrade_hold_s", 2.0);
    imu_only_timeout_s_ = declare_parameter<double>("imu_only_timeout_s", 30.0);

    // TF cannot carry covariance; publish nav_msgs/Odometry(map→odom) with pose cov.
    publish_map_odom_topic_ = declare_parameter<bool>("publish_map_odom_topic", true);
    map_odom_topic_ =
      declare_parameter<std::string>("map_odom_topic", "/usv_localization/map_odom");
    // σ_xy ≈ max(cov_xy_min_m, hdop * cov_xy_hdop_scale_m); also max with FC odom cov if present
    cov_xy_min_m_ = declare_parameter<double>("cov_xy_min_m", 0.05);
    cov_xy_hdop_scale_m_ = declare_parameter<double>("cov_xy_hdop_scale_m", 0.3);
    cov_yaw_rad2_ = declare_parameter<double>("cov_yaw_rad2", 0.01);
    cov_unknown_ = declare_parameter<double>("cov_unknown", 1.0e6);

    if (!map_config_yaml_.empty()) {
      std::string err;
      if (!load_ref_gnss(map_config_yaml_, map_origin_ref_key_, map_anchor_, err)) {
        throw std::runtime_error("Failed to load map anchor: " + err);
      }
      RCLCPP_INFO(
        get_logger(), "map anchor from %s[%s] → lat=%.8f lon=%.8f",
        map_config_yaml_.c_str(), map_origin_ref_key_.c_str(),
        map_anchor_.lat, map_anchor_.lon);
    } else {
      map_anchor_.lat = manual_lat;
      map_anchor_.lon = manual_lon;
      if (manual_lat == 0.0 && manual_lon == 0.0) {
        throw std::runtime_error(
          "map_config_yaml empty and map_origin_latitude/longitude are 0");
      }
      RCLCPP_INFO(
        get_logger(), "map anchor manual → lat=%.8f lon=%.8f",
        map_anchor_.lat, map_anchor_.lon);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    sub_gngga_ = create_subscription<bw_gi320_driver::msg::Gi320Gngga>(
      gngga_topic_, sensor_qos,
      std::bind(&UsvMapOdomTfNode::on_gngga, this, std::placeholders::_1));
    sub_inspvaa_ = create_subscription<bw_gi320_driver::msg::Gi320Inspvaa>(
      inspvaa_topic_, sensor_qos,
      std::bind(&UsvMapOdomTfNode::on_inspvaa, this, std::placeholders::_1));

    if (!fc_local_odom_topic_.empty()) {
      sub_fc_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        fc_local_odom_topic_, sensor_qos,
        std::bind(&UsvMapOdomTfNode::on_fc_odom, this, std::placeholders::_1));
    }

    if (publish_map_odom_topic_) {
      pub_map_odom_ = create_publisher<nav_msgs::msg::Odometry>(map_odom_topic_, 10);
    }

    const double hz = std::max(1.0, republish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&UsvMapOdomTfNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "usv_map_odom_tf: map→odom TF; cov_topic=%s; gga=%s inspvaa=%s fc_odom=%s "
      "mismatch=%.1fm fc_jump=%.1fm imu_only_timeout=%.1fs min_fix=%u",
      publish_map_odom_topic_ ? map_odom_topic_.c_str() : "(off)",
      gngga_topic_.c_str(), inspvaa_topic_.c_str(),
      fc_local_odom_topic_.empty() ? "(off)" : fc_local_odom_topic_.c_str(),
      map_gnss_mismatch_m_, fc_origin_jump_m_, imu_only_timeout_s_,
      static_cast<unsigned>(min_fix_type_));
  }

private:
  bool gnss_quality_ok_locked() const
  {
    if (!have_gngga_) {return false;}
    if (last_fix_type_ < min_fix_type_) {return false;}
    if (last_fix_type_ == imu_only_fix_type_) {return false;}
    if (max_hdop_ > 0.0 && last_hdop_ > max_hdop_) {return false;}
    return true;
  }

  double map_grid_yaw() const
  {
    return map_odom_yaw_deg_ * M_PI / 180.0;
  }

  /** GI320 航向(度) → map 系航向(rad)：先转 ENU，再加栅格偏角。 */
  double gi320_yaw_to_map(double gi320_yaw_deg) const
  {
    const double yaw_enu =
      wrap_pi(gi320_yaw_deg * M_PI / 180.0 +
        gi320_yaw_to_enu_offset_deg_ * M_PI / 180.0);
    return wrap_pi(yaw_enu + map_grid_yaw());
  }

  void gnss_to_map_xy(double lat, double lon, double & mx, double & my) const
  {
    double east = 0.0;
    double north = 0.0;
    latlon_to_enu(lat, lon, map_anchor_.lat, map_anchor_.lon, east, north);
    const double yaw_bias = map_grid_yaw();
    const double c = std::cos(yaw_bias);
    const double s = std::sin(yaw_bias);
    mx = c * east - s * north;
    my = s * east + c * north;
  }

  /** Fill 6x6 pose covariance for map→odom (row-major). */
  void fill_map_odom_covariance(std::array<double, 36> & cov) const
  {
    cov.fill(0.0);
    const double hdop = std::max(0.01f, last_hdop_);
    double sigma_xy = std::max(cov_xy_min_m_, static_cast<double>(hdop) * cov_xy_hdop_scale_m_);
    // Fold in FC local odom xy variance when finite and not a “unknown” placeholder.
    if (have_fc_cov_) {
      const double fxx = last_fc_cov_[0];
      const double fyy = last_fc_cov_[7];
      if (fxx > 0.0 && fxx < cov_unknown_ * 0.5) {
        sigma_xy = std::max(sigma_xy, std::sqrt(fxx));
      }
      if (fyy > 0.0 && fyy < cov_unknown_ * 0.5) {
        sigma_xy = std::max(sigma_xy, std::sqrt(fyy));
      }
    }
    const double var_xy = sigma_xy * sigma_xy;
    double var_yaw = cov_yaw_rad2_;
    if (have_fc_cov_) {
      const double fyyaw = last_fc_cov_[35];
      if (fyyaw > 0.0 && fyyaw < cov_unknown_ * 0.5) {
        var_yaw = std::max(var_yaw, fyyaw);
      }
    }
    cov[0] = var_xy;             // x
    cov[7] = var_xy;             // y
    cov[14] = cov_unknown_;      // z
    cov[21] = cov_unknown_;      // roll
    cov[28] = cov_unknown_;      // pitch
    cov[35] = var_yaw;           // yaw
  }

  void request_reset(const std::string & reason)
  {
    if (!locked_) {
      RCLCPP_DEBUG(get_logger(), "reset ignored (unlocked): %s", reason.c_str());
      return;
    }
    RCLCPP_WARN(get_logger(), "Reset map→odom: %s", reason.c_str());
    locked_ = false;
    gnss_was_bad_ = false;
    bad_since_set_ = false;
    imu_only_since_set_ = false;
  }

  void try_lock(const bw_gi320_driver::msg::Gi320Inspvaa & msg)
  {
    if (!gnss_quality_ok_locked()) {return;}
    // Need MAVROS odom so map→odom composes correctly with odom→base_link.
    if (!fc_local_odom_topic_.empty() && !have_fc_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for %s before locking map→odom", fc_local_odom_topic_.c_str());
      return;
    }

    double gnss_mx = 0.0;
    double gnss_my = 0.0;
    gnss_to_map_xy(msg.latitude, msg.longitude, gnss_mx, gnss_my);

    const double ox = have_fc_odom_ ? last_fc_x_ : 0.0;
    const double oy = have_fc_odom_ ? last_fc_y_ : 0.0;
    const double oyaw = have_fc_odom_ ? last_fc_yaw_ : 0.0;

    // 初始化航向：让 map→odom→base 的航向对齐 GNSS（map 系）
    if (use_gnss_yaw_at_lock_ && have_fc_odom_) {
      const double yaw_gnss_map = gi320_yaw_to_map(msg.yaw);
      map_odom_yaw_ = wrap_pi(yaw_gnss_map - oyaw);
    } else {
      map_odom_yaw_ = map_grid_yaw();
    }

    const double c = std::cos(map_odom_yaw_);
    const double s = std::sin(map_odom_yaw_);
    // map_T_odom * odom_T_base ≈ GNSS in map  ⇒  t = gnss_map - R(yaw)*odom_xy
    map_odom_tx_ = gnss_mx - (c * ox - s * oy);
    map_odom_ty_ = gnss_my - (s * ox + c * oy);
    locked_ = true;
    lock_stamp_ = msg.stamp;

    RCLCPP_INFO(
      get_logger(),
      "Locked map→odom: t=(%.3f, %.3f) yaw=%.2fdeg (grid=%.1f) "
      "gnss_yaw=%.1fdeg odom_yaw=%.1fdeg  gnss=(%.8f, %.8f) fc_odom=(%.3f, %.3f)",
      map_odom_tx_, map_odom_ty_, map_odom_yaw_ * 180.0 / M_PI, map_odom_yaw_deg_,
      msg.yaw, oyaw * 180.0 / M_PI,
      msg.latitude, msg.longitude, ox, oy);
  }

  void on_gngga(const bw_gi320_driver::msg::Gi320Gngga::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mu_);
    have_gngga_ = true;
    last_fix_type_ = msg->fix_type;
    last_hdop_ = msg->hdop;

    const bool ok = gnss_quality_ok_locked();
    const rclcpp::Time now = this->get_clock()->now();

    if (!ok) {
      if (!bad_since_set_) {
        bad_since_ = now;
        bad_since_set_ = true;
      }
      if ((now - bad_since_).seconds() >= gnss_degrade_hold_s_) {
        gnss_was_bad_ = true;
      }

      const bool imu_only =
        (last_fix_type_ == imu_only_fix_type_) ||
        (last_fix_type_ < min_fix_type_);
      if (imu_only) {
        if (!imu_only_since_set_) {
          imu_only_since_ = now;
          imu_only_since_set_ = true;
        } else if ((now - imu_only_since_).seconds() >= imu_only_timeout_s_) {
          request_reset("prolonged IMU-only / poor GNSS coasting");
        }
      }
    } else {
      bad_since_set_ = false;
      imu_only_since_set_ = false;
      if (gnss_was_bad_ && locked_) {
        // Unlock; next INSPVAA + FC odom will re-lock map→odom.
        request_reset("GNSS quality recovered after degrade");
      }
      gnss_was_bad_ = false;
    }
  }

  void on_inspvaa(const bw_gi320_driver::msg::Gi320Inspvaa::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mu_);
    last_inspvaa_ = *msg;
    have_inspvaa_ = true;

    if (!locked_) {
      try_lock(*msg);
      return;
    }

    double gnss_mx = 0.0;
    double gnss_my = 0.0;
    gnss_to_map_xy(msg->latitude, msg->longitude, gnss_mx, gnss_my);

    const double ox = have_fc_odom_ ? last_fc_x_ : 0.0;
    const double oy = have_fc_odom_ ? last_fc_y_ : 0.0;
    const double pred_mx =
      map_odom_tx_ + std::cos(map_odom_yaw_) * ox - std::sin(map_odom_yaw_) * oy;
    const double pred_my =
      map_odom_ty_ + std::sin(map_odom_yaw_) * ox + std::cos(map_odom_yaw_) * oy;

    const double err = std::hypot(pred_mx - gnss_mx, pred_my - gnss_my);
    if (map_gnss_mismatch_m_ > 0.0 && err > map_gnss_mismatch_m_ && gnss_quality_ok_locked()) {
      request_reset(
        "map vs GNSS mismatch err=" + std::to_string(err) + "m");
      try_lock(*msg);
    }
  }

  void on_fc_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mu_);
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    if (have_fc_odom_) {
      const double jump = std::hypot(x - last_fc_x_, y - last_fc_y_);
      if (fc_origin_jump_m_ > 0.0 && jump > fc_origin_jump_m_) {
        request_reset(
          "FC local odom jump " + std::to_string(jump) + "m (origin changed?)");
      }
    }
    last_fc_x_ = x;
    last_fc_y_ = y;
    last_fc_yaw_ = quat_to_yaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    have_fc_odom_ = true;
    last_fc_cov_ = msg->pose.covariance;
    have_fc_cov_ = true;
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!locked_ || !have_inspvaa_) {return;}

    const auto & msg = last_inspvaa_;
    rclcpp::Time stamp(msg.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = this->get_clock()->now();
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, map_odom_yaw_);

    geometry_msgs::msg::TransformStamped tf_mo;
    tf_mo.header.stamp = stamp;
    tf_mo.header.frame_id = map_frame_;
    tf_mo.child_frame_id = odom_frame_;
    tf_mo.transform.translation.x = map_odom_tx_;
    tf_mo.transform.translation.y = map_odom_ty_;
    tf_mo.transform.translation.z = 0.0;
    tf_mo.transform.rotation.x = q.x();
    tf_mo.transform.rotation.y = q.y();
    tf_mo.transform.rotation.z = q.z();
    tf_mo.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_mo);

    if (pub_map_odom_) {
      nav_msgs::msg::Odometry od;
      od.header.stamp = stamp;
      od.header.frame_id = map_frame_;
      od.child_frame_id = odom_frame_;
      od.pose.pose.position.x = map_odom_tx_;
      od.pose.pose.position.y = map_odom_ty_;
      od.pose.pose.position.z = 0.0;
      od.pose.pose.orientation.x = q.x();
      od.pose.pose.orientation.y = q.y();
      od.pose.pose.orientation.z = q.z();
      od.pose.pose.orientation.w = q.w();
      fill_map_odom_covariance(od.pose.covariance);
      // twist unknown for map→odom (static between re-locks)
      od.twist.covariance[0] = cov_unknown_;
      od.twist.covariance[7] = cov_unknown_;
      od.twist.covariance[14] = cov_unknown_;
      od.twist.covariance[21] = cov_unknown_;
      od.twist.covariance[28] = cov_unknown_;
      od.twist.covariance[35] = cov_unknown_;
      pub_map_odom_->publish(od);
    }
  }

  std::string map_config_yaml_;
  std::string map_origin_ref_key_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string gngga_topic_;
  std::string inspvaa_topic_;
  std::string fc_local_odom_topic_;
  std::string map_odom_topic_;

  double map_odom_yaw_deg_{0.0};
  bool use_gnss_yaw_at_lock_{true};
  double gi320_yaw_to_enu_offset_deg_{-90.0};
  double republish_hz_{20.0};
  uint8_t min_fix_type_{4};
  uint8_t imu_only_fix_type_{6};
  double max_hdop_{2.5};
  double map_gnss_mismatch_m_{5.0};
  double fc_origin_jump_m_{5.0};
  double gnss_degrade_hold_s_{2.0};
  double imu_only_timeout_s_{30.0};
  bool publish_map_odom_topic_{true};
  double cov_xy_min_m_{0.05};
  double cov_xy_hdop_scale_m_{0.3};
  double cov_yaw_rad2_{0.01};
  double cov_unknown_{1.0e6};

  LatLon map_anchor_{};

  bool locked_{false};
  double map_odom_tx_{0.0};
  double map_odom_ty_{0.0};
  double map_odom_yaw_{0.0};
  builtin_interfaces::msg::Time lock_stamp_{};

  bool have_gngga_{false};
  uint8_t last_fix_type_{0};
  float last_hdop_{99.0f};

  bool have_inspvaa_{false};
  bw_gi320_driver::msg::Gi320Inspvaa last_inspvaa_{};

  bool gnss_was_bad_{false};
  bool bad_since_set_{false};
  rclcpp::Time bad_since_{0, 0, RCL_ROS_TIME};
  bool imu_only_since_set_{false};
  rclcpp::Time imu_only_since_{0, 0, RCL_ROS_TIME};

  bool have_fc_odom_{false};
  double last_fc_x_{0.0};
  double last_fc_y_{0.0};
  double last_fc_yaw_{0.0};
  bool have_fc_cov_{false};
  std::array<double, 36> last_fc_cov_{};

  std::mutex mu_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<bw_gi320_driver::msg::Gi320Gngga>::SharedPtr sub_gngga_;
  rclcpp::Subscription<bw_gi320_driver::msg::Gi320Inspvaa>::SharedPtr sub_inspvaa_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_fc_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usv_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<usv_localization::UsvMapOdomTfNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "usv_map_odom_tf failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
