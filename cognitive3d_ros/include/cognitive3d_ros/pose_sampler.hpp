#pragma once

#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cognitive3d_ros/session_runner.hpp"

namespace cognitive3d_ros {

// Which axis convention a configured `gaze_frame` uses.
//
// REP-103 gives a robot body frame +X forward, +Y left, +Z up, while a camera
// OPTICAL frame is +Z forward, +X right, +Y down. A TurtleBot 4 publishes both
// `oakd_rgb_camera_frame` and `oakd_rgb_camera_optical_frame`, and the optical
// one is the one that MEANS "where the camera looks" -- so it is the one an
// operator reaches for, and reading its +X as forward turns it 90 degrees off.
enum class FrameConvention {
  // Infer from the frame name. The default, because the frame an operator names
  // is usually the optical one and the naming convention is reliable enough to
  // act on when the alternative is a silent 90 degree error.
  kAuto,
  kBodyForwardX,     // +X forward, +Y left, +Z up
  kOpticalForwardZ,  // +Z forward, +X right, +Y down
};

bool ParseFrameConvention(const std::string& text, FrameConvention* out);
const char* FrameConventionName(FrameConvention c);

// True for a frame name following the ROS convention of suffixing an optical
// frame, e.g. `oakd_rgb_camera_optical_frame` or `front_camera_optical`.
bool FrameNameLooksOptical(const std::string& frame);

// `configured` with kAuto resolved against the frame name. Never returns kAuto,
// so every caller downstream handles exactly two cases.
FrameConvention ResolveFrameConvention(const std::string& frame, FrameConvention configured);

struct PoseSamplerConfig {
  // REP-105: map is absolute but jumps on relocalisation, odom is smooth but
  // drifts. We prefer map for position and fall back to odom, and we emit an
  // event when map jumps so a spatial aggregate can exclude the discontinuity.
  std::string preferred_reference_frame = "map";
  std::string fallback_reference_frame = "odom";
  // How long the preferred frame may still be missing, once the fallback has
  // resolved, before the fallback is adopted for good.
  //
  // The fallback resolving FIRST is the normal startup order, not a verdict:
  // `odom -> base_link` is published from the first wheel tick, while nav2_amcl
  // waits for a scan and then broadcasts `map -> odom` at 1 to 3.5 Hz. Adopting
  // the fallback inside that window costs one pose at the odom origin -- metres
  // from where the robot is -- and a `robot.reference_frame_unavailable` event
  // that is a false alarm, on every robot whose localiser is slower than its
  // odometry, which is all of them.
  //
  // Nothing is recorded while the grace runs. A robot that genuinely has no
  // localiser loses this much of the start of its timeline and then proceeds on
  // the fallback with the event recorded, which is the outcome that has to stay.
  double reference_frame_grace_s = 5.0;
  std::string robot_frame = "base_link";
  // Optional. When set, the gaze channel reports this frame's pose instead of the
  // robot's: its position and its rotation, by the gaze rotation rule, so a viewer
  // draws the viewpoint where this frame is and turns it with this frame. Both move
  // together, never one without the other. The robot body is carried separately
  // only as the dynamic object, which `robot_dynamic_object` turns on.
  //
  // Posed at the INSTANT the robot_frame pose resolved to: the body's transform
  // composed with `robot_frame -> this frame` at the body's stamp, or at that
  // joint's latest when its broadcaster has not published that far yet. Two
  // independent "latest" lookups resolve to two instants whenever the chains have
  // different newest stamps, and a turning robot then carries a viewpoint that
  // trails its own body.
  std::string gaze_frame;
  FrameConvention gaze_frame_convention = FrameConvention::kAuto;

  double rate_hz = 30.0;
  // A pose whose transform stamp has not advanced for this long, on the wire's clock, is STALE:
  // skipped and counted rather than recorded as where the robot is now. A publisher that dies
  // leaves its last transform in the buffer, and without this the robot is recorded standing
  // still for the rest of the session with no failure counted. 0 turns the check off; the node
  // sets it from `rate_hz` with StaleTransformAfterSeconds. A static transform, stamped zero, is
  // never stale.
  double stale_transform_after_s = 0.0;

  // How far apart, on the wire's clock, the two poses a relocalisation check compares are: about
  // this far at a `rate_hz` of 10 or more, and consecutive samples below it (see
  // RelocalisationCheckDue). The thresholds below are sized against it, which is why it is not a
  // parameter.
  //
  // Consecutive samples are the wrong pair at a higher rate. A localiser that future-dates its
  // transform, as nav2_amcl does by its `transform_tolerance`, is interpolated by tf2 between two
  // of its updates, so one correction reaches the pose as a ramp an update period long. Behind
  // scans at 7 Hz, a 90 degree correction sampled at 30 Hz is about 25 degrees per sample, and no
  // per-sample threshold that also lets a fast turn through can see it.
  static constexpr double kRelocalisationCheckIntervalS = 0.1;

  // A map-frame position moving further than this between two relocalisation
  // checks is a relocalisation jump, not motion.
  //
  // Per CHECK, not per second, so the legitimate motion it has to clear is what a
  // base covers in one check interval: 0.1 s at 10 Hz and above, where a 0.5 m
  // step is 5 m/s, faster than any indoor base. Below 10 Hz the interval is one
  // sample, and halving the sample rate doubles what one honest sample covers.
  double relocalisation_jump_m = 0.5;

  // A map-frame ORIENTATION turning further than this between two relocalisation
  // checks is a relocalisation jump, not rotation.
  //
  // Position alone cannot see the jump this exists for. A localiser that
  // re-registers against a rotationally ambiguous room re-estimates its yaw by
  // a right angle with the robot standing still, so the positional step is
  // millimetres and the orientation step is 90 degrees. That is not a lesser
  // case of the same fault: the viewer turns the robot by the gaze ORIENTATION, so
  // an unannotated turn corrupts the spatial timeline itself rather than inflating
  // a travel metric.
  //
  // 45 degrees, per check, chosen against both sides:
  //   - Legitimate rotation has to pass underneath it. A base spinning at its own
  //     limit -- 1.90 rad/s, 109 deg/s, a TurtleBot 4 flat out -- covers 11
  //     degrees in 0.1 s, and 45 leaves room for a check that came a whole period
  //     late and for a base twice that fast. Sustained, 45 degrees per 0.1 s is
  //     450 deg/s, which no ground robot does.
  //   - A real jump has to reach it. A localiser re-registering against an
  //     ambiguous room re-estimates its yaw by a right angle, so the threshold
  //     sits at half of that with the margin on the right side. A correction that
  //     tf2 spreads over longer than one interval is split between two checks and
  //     can reach the threshold in neither, at any `rate_hz`.
  // Like `relocalisation_jump_m` this is per check, so a `rate_hz` below 10 Hz
  // requires raising this with it: each sample then covers more of a turn, and a
  // fast one would read as a jump.
  double relocalisation_jump_deg = 45.0;

  std::string robot_display_name = "Robot";
  std::string robot_mesh = "robot_placeholder";

  // Yaw offset in degrees applied to the DYNAMIC OBJECT only, never to the gaze
  // rotation, to account for a mesh whose own forward axis is not +X in the robot
  // frame.
  //
  // This is common: an artist-authored or URDF mesh is often modelled facing
  // sideways relative to base_link. The symptom is a robot that renders driving
  // crabwise while its gaze rotation correctly follows the path. Applying the
  // offset to the pose instead would turn the gaze rotation too, which just moves
  // the bug.
  double mesh_yaw_offset_deg = 0.0;
};

// How long a transform may go without its stamp advancing before the pose built from it is stale:
// ten sample periods, and never under five seconds, which clears a localiser that publishes at
// 0.5 Hz without future-dating its transform.
double StaleTransformAfterSeconds(double rate_hz);

// Whether a relocalisation check is due, `elapsed_s` after the pose it compares against. Due at
// kRelocalisationCheckIntervalS less a tenth of a sample period (of the interval, below 10 Hz),
// so a sample that arrives a little early is still the one checked: a 30 Hz timer checks every
// third sample and never slips to every sixth, and below 10 Hz every sample is checked.
bool RelocalisationCheckDue(double elapsed_s, double rate_hz);

// The event-cap source key the pose channel's own events are charged to. Not a
// topic name and unable to collide with one, because these come from the tf tree.
constexpr const char* kPoseSamplerEventSource = "pose_sampler";

// Samples the tf tree on a timer and produces the two channels that make a
// session replayable: the gaze stream, which is the spatial timeline and the
// only carrier of session properties, and the robot dynamic object.
//
// Deliberately does not subscribe to /tf directly. tf2's Buffer already
// maintains the transform tree; reconstructing chains by hand from raw
// TFMessage is a classic way to get subtly wrong poses.
class PoseSampler {
 public:
  PoseSampler(std::shared_ptr<tf2_ros::Buffer> buffer, PoseSamplerConfig config);

  // Returns true when a pose was recorded.
  bool Sample(double wire_time, SessionRunner& runner, std::string* status_out);
  void Reset();

  const std::string& active_reference_frame() const { return active_reference_frame_; }
  // kAuto resolved once, at construction. The node reads this back to log the
  // choice and to record it as a session property, so there is one resolution
  // site rather than two that can drift apart.
  FrameConvention resolved_gaze_frame_convention() const {
    return resolved_gaze_frame_convention_;
  }
  // Positional and rotational jumps counted TOGETHER, deliberately. This feeds
  // the shipped `c3d.localisation_jumps` series, whose meaning is "how much of
  // this timeline is the localiser rather than the robot" -- one number that
  // splitting would change the meaning of for every existing reader. A consumer
  // that needs to tell them apart reads the event, which carries `distance_m`,
  // `rotation_deg` and a `kind` naming which condition fired.
  std::uint64_t relocalisation_jumps() const { return relocalisation_jumps_; }
  std::uint64_t lookup_failures() const { return lookup_failures_; }
  // Samples skipped because the transform had stopped advancing. Reset per session.
  std::uint64_t stale_transforms() const { return stale_transforms_; }
  // True when the last Sample() recorded nothing because the transform was stale.
  bool last_sample_stale() const { return last_sample_stale_; }
  // True when the last Sample() recorded nothing because it was waiting out
  // `reference_frame_grace_s`. NOT a lookup failure and must not be counted as
  // one: a pose was available and was deliberately not taken.
  bool holding_for_preferred_frame() const { return holding_for_preferred_frame_; }
  // Samples that fell back to the robot pose because `gaze_frame` could not be
  // resolved. Counted separately from lookup_failures_ because it is NOT a
  // dropped sample: the session looks healthy, every gaze record is well formed,
  // and the only symptom is a viewpoint at the wrong height. One dropped
  // character in a frame name reaches this and nothing else.
  std::uint64_t gaze_frame_lookup_failures() const { return gaze_frame_lookup_failures_; }
  // True when the `gaze_frame` lookup has failed since this last returned true.
  //
  // The bar lives HERE, beside the counter it compares against and cleared by
  // the same Reset(), because a bar on the CALLER outlives the counter: the
  // counter resets per session, so after any restart `0 > N` would be false
  // forever and the operator warning would go permanently silent on a run where
  // every gaze sample came from the wrong frame. One decision, one lifetime.
  bool ConsumeGazeFrameFailureNotice();

  // Where the robot BODY last was, in the active reference frame, or false before
  // the first successful sample. An externally published event is stamped with
  // this: an event at the origin is indistinguishable from an event at the
  // origin, and "where did this happen" is most of what a spatial analytics
  // platform is for. Deliberately the body and not the camera -- an event is
  // something the robot did.
  bool last_body_position(c3d::Vec3* out) const;

  // Emitted once per session: the robot's frame topology, which is the cheapest
  // useful description of what kind of machine produced the data.
  void EmitTopology(double wire_time, SessionRunner& runner);

  // The per-source event cap this channel's events are charged to, or null for
  // none. Every source the node has is capped, and this one is a source like any
  // other: a frame flapping between `map` and `odom` produces an event per sample.
  void set_event_limiter(EventRateLimiter* limiter) { event_limiter_ = limiter; }

  // An arbitrary sensor's pose in the reference frame in force, in ROS axes, for
  // the camera and lidar streams.
  //
  // It reuses the SAME frame decision the gaze channel made -- the frame already
  // adopted, and nothing until one is -- rather than resolving its own. Two frame
  // decisions in one session is how a camera frame ends up posed against `odom`
  // while the pose timeline is in `map`, which puts the pictures metres from the
  // path they were taken on with nothing on the wire to say so. That includes the
  // `reference_frame_grace_s` window, where the gaze channel is recording nothing
  // and this must record nothing either.
  //
  // `stamp` is the message's own, not the latest available: a frame is posed where
  // the robot WAS when it was taken. A lookup that fails returns false and the
  // caller drops the record.
  //
  // A lookup waits up to 50 ms for the transform at `stamp`, which is usually
  // about to arrive. After one fails, lookups for that frame take what the buffer
  // holds, and wait again at most once a second, until one succeeds. They run on
  // the node's one callback thread, where a frame tf cannot resolve would
  // otherwise cost 50 ms per record: at 30 records a second, the whole thread and
  // the pose timer with it.
  bool LookupSensorPose(const std::string& frame, const rclcpp::Time& stamp,
                        c3d::Vec3* position, c3d::Quat* rotation);

 private:
  // Through the capped door, like everything else. The position is passed rather
  // than read back from `last_position_`: a frame report is stamped where the robot
  // is in the frame BEING REPORTED, which on a frame change is not where the last
  // sample put it.
  bool RecordEvent(double wire_time, SessionRunner& runner, const std::string& name,
                   const c3d::Vec3& position, bool position_known,
                   c3d::Properties properties);

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  PoseSamplerConfig config_;
  FrameConvention resolved_gaze_frame_convention_;
  std::string active_reference_frame_;
  bool have_last_ = false;
  c3d::Vec3 last_position_;
  // The pose the next relocalisation check compares against, and the wire time it was taken at.
  // Cleared wherever `have_last_` is: a pose kept across a frame change would compare `map`
  // against `odom` and report the difference between the frames as a jump, and one kept across
  // a stale stretch or a session restart would report the motion in between as one.
  bool have_jump_anchor_ = false;
  c3d::Vec3 jump_anchor_position_;
  c3d::Quat jump_anchor_rotation_;
  double jump_anchor_time_ = 0.0;
  std::uint64_t relocalisation_jumps_ = 0;
  std::uint64_t lookup_failures_ = 0;
  std::uint64_t stale_transforms_ = 0;
  bool last_sample_stale_ = false;
  // The stamp the transform in force last carried, in which frame, and the wire time it was
  // first seen at. A stamp unchanged across samples is how a stopped publisher shows itself.
  std::int64_t last_stamp_ns_ = -1;
  std::string last_stamp_frame_;
  double last_stamp_since_ = 0.0;
  std::uint64_t gaze_frame_lookup_failures_ = 0;
  std::uint64_t gaze_frame_failures_noticed_ = 0;
  bool holding_for_preferred_frame_ = false;
  bool fallback_seen_ = false;
  double fallback_first_seen_ = 0.0;
  bool topology_emitted_ = false;
  EventRateLimiter* event_limiter_ = nullptr;
  // Per sensor frame whose last lookup failed: when a lookup for it may wait again.
  std::map<std::string, std::chrono::steady_clock::time_point> sensor_wait_not_before_;
};

}  // namespace cognitive3d_ros
