#ifndef USV_DOCKING__ANCHOR_FILTER_HPP_
#define USV_DOCKING__ANCHOR_FILTER_HPP_

#include "usv_docking/docking_common.hpp"

#include <deque>
#include <optional>
#include <string>

namespace usv_docking
{

struct AnchorParams
{
  double ema_alpha_position{0.25};
  double ema_alpha_yaw{0.15};
  double max_position_jump{0.60};
  double max_yaw_jump_deg{20.0};
  double reloc_max_yaw_jump_deg{10.0};
  double reloc_guard_escape_sec{8.0};
  double reloc_max_dist_m{4.0};
  bool reloc_require_dual_tag{true};
  int jump_reject_unlock_frames{12};
  int seed_frames{8};
  double seed_max_spread_m{0.40};
  double seed_max_yaw_spread_deg{15.0};
};

class AnchorFilter
{
public:
  explicit AnchorFilter(const AnchorParams & params = AnchorParams{});
  std::optional<Pose2D> update(
    const Pose2D & raw, double distance, double now, int visible_tag_count = 0);
  void reset();
  const std::optional<Pose2D> & anchor() const {return anchor_;}
  std::string status() const;
  int reject_count() const {return reject_count_;}
  std::size_t seed_size() const {return seed_buffer_.size();}

private:
  bool relocation_trustworthy(double distance, int visible_tag_count) const;
  AnchorParams params_;
  std::optional<Pose2D> anchor_;
  std::deque<Pose2D> seed_buffer_;
  int reject_count_{0};
  std::optional<Pose2D> reject_ema_;
  std::optional<double> reject_cluster_start_;
};

}  // namespace usv_docking

#endif  // USV_DOCKING__ANCHOR_FILTER_HPP_
