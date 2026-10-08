#include "cognitive3d_ros/clock_policy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace cognitive3d_ros {
namespace {

double WallEpochSeconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

}  // namespace

ClockPolicy::ClockPolicy(rclcpp::Clock::SharedPtr ros_clock, bool use_sim_time,
                         bool sim_time_on_wire)
    : ros_clock_(std::move(ros_clock)),
      use_sim_time_(use_sim_time),
      sim_time_on_wire_(sim_time_on_wire) {}

double ClockPolicy::WireTime() const {
  if (!sim_time_on_wire_) return WallEpochSeconds() + wall_clock_lead_.load();
  std::lock_guard<std::mutex> lock(anchor_mu_);
  return AnchoredWireTimeLocked(RosTime());
}

void ClockPolicy::Anchor(double wall_now, double ros_now) {
  if (!sim_time_on_wire_) return;
  std::lock_guard<std::mutex> lock(anchor_mu_);
  anchored_ = false;
  anchor_pending_ = true;
  if (ros_now > 0.0) SetAnchorLocked(wall_now, ros_now);
}

void ClockPolicy::LeadWallClock(double seconds) {
  if (sim_time_on_wire_) return;
  wall_clock_lead_ = std::isfinite(seconds) && seconds > 0.0 ? seconds : 0.0;
}

double ClockPolicy::wall_clock_lead() const { return wall_clock_lead_.load(); }

double ClockPolicy::LatestWireTime() const {
  std::lock_guard<std::mutex> lock(anchor_mu_);
  return latest_wire_time_;
}

bool ClockPolicy::anchored() const {
  std::lock_guard<std::mutex> lock(anchor_mu_);
  return anchored_;
}

void ClockPolicy::SetAnchorLocked(double wall_now, double ros_now) const {
  // Never earlier than the latest wire time already issued. Below real time wall
  // now is always later, so this only bites when the sim ran faster than real time
  // and the previous session's records reach past wall now.
  wall_anchor_ = std::max(wall_now, latest_wire_time_);
  ros_anchor_ = ros_now;
  anchored_ = true;
  anchor_pending_ = false;
}

double ClockPolicy::AnchoredWireTimeLocked(double ros_now) const {
  if (anchor_pending_ && ros_now > 0.0) SetAnchorLocked(WallEpochSeconds(), ros_now);
  const double wire = std::max(
      anchored_ ? wall_anchor_ + (ros_now - ros_anchor_) : WallEpochSeconds(), latest_now_wire_);
  latest_now_wire_ = wire;
  latest_wire_time_ = std::max(latest_wire_time_, wire);
  return wire;
}

double ClockPolicy::RosTime() const {
  if (!ros_clock_) return 0.0;
  return ros_clock_->now().seconds();
}

bool ClockPolicy::RosTimeReady() const {
  if (!use_sim_time_) return true;
  return RosTime() > 0.0;
}

double ClockPolicy::HeaderStampToWireTime(double header_stamp_seconds) const {
  if (sim_time_on_wire_) {
    std::lock_guard<std::mutex> lock(anchor_mu_);
    const double ros_now = RosTime();
    const double now_on_wire = AnchoredWireTimeLocked(ros_now);
    if (header_stamp_seconds <= 0.0 || ros_now <= 0.0) return now_on_wire;
    const double age = ros_now - header_stamp_seconds;
    if (!std::isfinite(age) || age < -1.0 || age > 3600.0) return now_on_wire;
    // Anchored, `now_on_wire - age` is `wall_anchor + (stamp - ros_anchor)`.
    const double wire = now_on_wire - age;
    latest_wire_time_ = std::max(latest_wire_time_, wire);
    return wire;
  }
  if (header_stamp_seconds <= 0.0) return WireTime();
  const double ros_now = RosTime();
  if (ros_now <= 0.0) return WireTime();
  const double age = ros_now - header_stamp_seconds;
  // Guard against a stamp from the future or an absurdly old one, either of
  // which usually means clock skew between machines rather than a real age.
  if (!std::isfinite(age) || age < -1.0 || age > 3600.0) return WireTime();
  return WireTime() - age;
}

void ClockPolicy::OnBackwardJump(JumpCallback callback) {
  on_backward_jump_ = std::move(callback);
}

bool ClockPolicy::Poll() {
  const double now = RosTime();
  if (last_ros_time_ < 0.0) {
    last_ros_time_ = now;
    return false;
  }
  const double delta = now - last_ros_time_;
  last_ros_time_ = now;
  if (delta < -backward_jump_threshold_s) {
    backward_jump_pending_ = true;
    if (on_backward_jump_) on_backward_jump_(delta);
    return true;
  }
  return false;
}

bool ClockPolicy::ConsumeBackwardJump() { return backward_jump_pending_.exchange(false); }

}  // namespace cognitive3d_ros
