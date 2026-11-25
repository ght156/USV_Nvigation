#include "usv_docking/yaw_rate_estimator.hpp"

#include "usv_docking/docking_common.hpp"

#include <cmath>

namespace usv_docking
{

YawRateEstimator::YawRateEstimator(const YawRateParams & params)
: params_(params)
{
}

void YawRateEstimator::reset()
{
  last_yaw_.reset();
  last_stamp_.reset();
  rate_.reset();
}

std::optional<double> YawRateEstimator::value(double now_sec) const
{
  if (!rate_ || !last_stamp_ || now_sec < *last_stamp_ ||
    now_sec - *last_stamp_ > params_.timeout_sec)
  {
    return std::nullopt;
  }
  return rate_;
}

std::optional<double> YawRateEstimator::update(double yaw, double stamp_sec)
{
  if (!std::isfinite(yaw) || !std::isfinite(stamp_sec)) {
    reset();
    return std::nullopt;
  }
  if (last_yaw_ && last_stamp_) {
    const double dt = stamp_sec - *last_stamp_;
    if (dt > 1e-3 && dt < 0.5) {
      const double raw_rate = wrap_angle(yaw - *last_yaw_) / dt;
      rate_ = rate_ ?
        std::optional<double>(*rate_ + params_.ema_alpha * (raw_rate - *rate_)) :
        std::optional<double>(raw_rate);
    } else {
      rate_.reset();
    }
  }
  last_yaw_ = yaw;
  last_stamp_ = stamp_sec;
  return value(stamp_sec);
}

}  // namespace usv_docking
