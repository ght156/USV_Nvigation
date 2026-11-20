#include "usv_docking/anchor_filter.hpp"

#include <algorithm>
#include <numeric>
#include <vector>

namespace usv_docking
{

AnchorFilter::AnchorFilter(const AnchorParams & params)
: params_(params)
{
}

void AnchorFilter::reset()
{
  anchor_.reset();
  seed_buffer_.clear();
  reject_count_ = 0;
  reject_ema_.reset();
  reject_cluster_start_.reset();
}

std::string AnchorFilter::status() const
{
  if (!anchor_) return "UNANCHORED";
  return reject_count_ > 0 ? "RELOCALIZING" : "VALID";
}

bool AnchorFilter::relocation_trustworthy(double distance, int visible_tag_count) const
{
  if (distance > params_.reloc_max_dist_m) return false;
  return !params_.reloc_require_dual_tag || visible_tag_count != 1;
}

std::optional<Pose2D> AnchorFilter::update(
  const Pose2D & raw, double distance, double now, int visible_tag_count)
{
  if (!anchor_) {
    seed_buffer_.push_back(raw);
    if (static_cast<int>(seed_buffer_.size()) < params_.seed_frames) return std::nullopt;
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto & pose : seed_buffer_) {
      xs.push_back(pose.x);
      ys.push_back(pose.y);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    const double mx = xs[xs.size() / 2];
    const double my = ys[ys.size() / 2];
    double spread = 0.0;
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const auto & pose : seed_buffer_) {
      spread = std::max(spread, std::hypot(pose.x - mx, pose.y - my));
      sin_sum += std::sin(pose.yaw);
      cos_sum += std::cos(pose.yaw);
    }
    const double cy = std::atan2(sin_sum, cos_sum);
    double yaw_spread = 0.0;
    for (const auto & pose : seed_buffer_) {
      yaw_spread = std::max(yaw_spread, std::abs(wrap_angle(pose.yaw - cy)));
    }
    if (spread > params_.seed_max_spread_m ||
      yaw_spread > params_.seed_max_yaw_spread_deg * PI / 180.0)
    {
      seed_buffer_.pop_front();
      return std::nullopt;
    }
    anchor_ = Pose2D{mx, my, cy};
    seed_buffer_.clear();
    reject_count_ = 0;
    reject_ema_.reset();
    reject_cluster_start_.reset();
    return anchor_;
  }

  const double dp = std::hypot(raw.x - anchor_->x, raw.y - anchor_->y);
  const double dyaw = std::abs(wrap_angle(raw.yaw - anchor_->yaw));
  if (dp > params_.max_position_jump || dyaw > params_.max_yaw_jump_deg * PI / 180.0) {
    ++reject_count_;
    if (!reject_ema_) {
      reject_ema_ = raw;
      if (!reject_cluster_start_) reject_cluster_start_ = now;
    } else {
      reject_ema_->x += 0.4 * (raw.x - reject_ema_->x);
      reject_ema_->y += 0.4 * (raw.y - reject_ema_->y);
      reject_ema_->yaw = wrap_angle(
        reject_ema_->yaw + 0.4 * wrap_angle(raw.yaw - reject_ema_->yaw));
    }
    if (reject_count_ >= params_.jump_reject_unlock_frames) {
      const double reloc_yaw = std::abs(wrap_angle(reject_ema_->yaw - anchor_->yaw));
      if (reloc_yaw > params_.reloc_max_yaw_jump_deg * PI / 180.0) {
        const double cluster_age = now - reject_cluster_start_.value_or(now);
        if (cluster_age <= params_.reloc_guard_escape_sec) {
          reject_count_ = 0;
          reject_ema_.reset();
          return anchor_;
        }
      }
      if (!relocation_trustworthy(distance, visible_tag_count)) {
        reject_count_ = 0;
        reject_ema_.reset();
        return anchor_;
      }
      anchor_ = reject_ema_;
      reject_count_ = 0;
      reject_ema_.reset();
      reject_cluster_start_.reset();
    }
    return anchor_;
  }

  reject_count_ = 0;
  reject_ema_.reset();
  reject_cluster_start_.reset();
  anchor_->x += params_.ema_alpha_position * (raw.x - anchor_->x);
  anchor_->y += params_.ema_alpha_position * (raw.y - anchor_->y);
  anchor_->yaw = wrap_angle(
    anchor_->yaw + params_.ema_alpha_yaw * wrap_angle(raw.yaw - anchor_->yaw));
  return anchor_;
}

}  // namespace usv_docking
