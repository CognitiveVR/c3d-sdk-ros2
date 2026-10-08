#pragma once

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace cognitive3d_ros {

// The clock policy, which is the single most likely way this build fails
// silently.
//
// Every Cognitive3D payload timestamp is Unix epoch seconds, and the session id
// is literally "{epochSeconds}_{deviceId}". Under use_sim_time, Gazebo's /clock
// starts near zero. Putting ROS time on the wire therefore lands the session in
// January 1970 and the dashboard shows nothing useful.
//
// The rule, in one line: ROS TIME NEVER GOES ON THE WIRE.
//
// Every record is stamped with epoch seconds -- wall clock unless `sim_time_on_wire`
// is on -- and no record carries ROS time. ROS time is read for three things only:
// a message's age, its header stamp against ROS now, which HeaderStampToWireTime
// subtracts from the wire's now; a backward jump, which closes the session; and,
// under `sim_time_on_wire`, how far the wire has advanced since the session's
// anchor.
//
// `sim_time_on_wire` keeps the calendar year and changes the rate: the session is
// anchored to a wall-clock instant at its start and advances with ROS time from
// there, so a simulation running below real time is not stretched on the wire.
// ROS time itself still never reaches the wire.
class ClockPolicy {
 public:
  explicit ClockPolicy(rclcpp::Clock::SharedPtr ros_clock, bool use_sim_time,
                       bool sim_time_on_wire = false);

  // Epoch seconds for the wire. Wall clock plus the lead (see LeadWallClock), unless
  // `sim_time_on_wire` is on and an anchor is in force, in which case
  // `wall_anchor + (ROS now - ros_anchor)`. Under `sim_time_on_wire` it never decreases: after a
  // backward ROS jump it holds at the latest value it returned until ROS time or a new anchor
  // carries it past.
  double WireTime() const;

  // The latest wire time issued so far, a header-stamp placement included.
  double LatestWireTime() const;

  // Pins the pair every later wire time is measured from. A no-op unless
  // `sim_time_on_wire` is on. A `ros_now` of zero is never captured: the anchor is
  // left pending and taken at the first wire time after /clock has been published,
  // with wall clock on the wire until then.
  void Anchor(double wall_now, double ros_now);
  bool anchored() const;

  // Off `sim_time_on_wire`, how far the wire runs ahead of the wall clock: a session whose start
  // is later than wall now -- its id is a whole second past the last one issued -- keeps every
  // record, a header placement included, at or after that start, spaced as the wall clock spaces
  // them. Negative is none. A no-op under `sim_time_on_wire`, where the anchor does this.
  void LeadWallClock(double seconds);
  double wall_clock_lead() const;
  bool sim_time_on_wire() const { return sim_time_on_wire_; }

  // Current ROS time in seconds. Under sim time this may start near zero and
  // may jump. Never put it on the wire.
  double RosTime() const;

  // Converts a message header stamp into the wire domain by measuring its age
  // against ROS now and subtracting that age from wall-clock now. This keeps
  // relative ordering and spacing from the sensor's own clock while still
  // landing in a real calendar year. Anchored, the stamp is placed directly:
  // `wall_anchor + (stamp - ros_anchor)`, behind the same sanity guard.
  double HeaderStampToWireTime(double header_stamp_seconds) const;

  bool use_sim_time() const { return use_sim_time_; }

  // Fires when ROS time moves backwards past the threshold: a sim reset, a bag
  // loop, or a large clock correction. The session must be closed and a new one
  // opened, otherwise durations go negative and two runs interleave.
  using JumpCallback = std::function<void(double delta_seconds)>;
  void OnBackwardJump(JumpCallback callback);

  // Call periodically. Returns true when a backward jump was observed on this
  // call. Polling rather than relying solely on rclcpp::JumpHandler keeps this
  // testable without a live clock.
  bool Poll();

  // True once per observed backward jump, for whoever acts on it -- which is NOT necessarily
  // whoever observed it. `Poll` is edge-triggered and TWO timers call it: the sample timer, which
  // restarts the session, and the discovery timer, which only wants the clock kept current. With
  // the edge as the only signal, a discovery tick landing between the jump and the next sample
  // tick would swallow the restart, and the node would log "the current session must be closed and
  // a new one opened" over a session that then stays open.
  bool ConsumeBackwardJump();

  // A ROS time of exactly zero means nobody has published /clock yet. That is
  // uninitialised, not an epoch, and recording against it produces nonsense.
  bool RosTimeReady() const;

  double backward_jump_threshold_s = 0.5;

 private:
  void SetAnchorLocked(double wall_now, double ros_now) const;
  double AnchoredWireTimeLocked(double ros_now) const;

  rclcpp::Clock::SharedPtr ros_clock_;
  bool use_sim_time_;
  bool sim_time_on_wire_;
  std::atomic<double> wall_clock_lead_{0.0};
  // Mutable because the pending anchor is resolved from inside the const readers.
  mutable std::mutex anchor_mu_;
  mutable bool anchor_pending_ = false;
  mutable bool anchored_ = false;
  mutable double wall_anchor_ = 0.0;
  mutable double ros_anchor_ = 0.0;
  mutable double latest_wire_time_ = 0.0;
  // The latest "now" issued, which is the floor every later one is held at. Header
  // placements are left out: a stamp may legitimately run up to a second ahead of now.
  mutable double latest_now_wire_ = 0.0;
  mutable double last_ros_time_ = -1.0;
  // Atomic so that Poll and ConsumeBackwardJump stay correct when called from two threads: a plain
  // bool loses the latch if a Poll lands between Consume's read and its write, and the jump is then
  // gone for good because `last_ros_time_` has already moved.
  std::atomic<bool> backward_jump_pending_{false};
  JumpCallback on_backward_jump_;
};

}  // namespace cognitive3d_ros
