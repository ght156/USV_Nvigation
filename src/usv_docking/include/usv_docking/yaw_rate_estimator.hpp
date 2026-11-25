#ifndef USV_DOCKING__YAW_RATE_ESTIMATOR_HPP_
#define USV_DOCKING__YAW_RATE_ESTIMATOR_HPP_

#include <optional>

namespace usv_docking
{

struct YawRateParams
{
  double ema_alpha{0.35};
  double timeout_sec{0.30};
};

class YawRateEstimator
{
public:
  explicit YawRateEstimator(const YawRateParams & params = YawRateParams{});
  std::optional<double> update(double yaw, double stamp_sec);
  std::optional<double> value(double now_sec) const;
  void reset();

private:
  YawRateParams params_;
  std::optional<double> last_yaw_;
  std::optional<double> last_stamp_;
  std::optional<double> rate_;
};

}  // namespace usv_docking

#endif  // USV_DOCKING__YAW_RATE_ESTIMATOR_HPP_
