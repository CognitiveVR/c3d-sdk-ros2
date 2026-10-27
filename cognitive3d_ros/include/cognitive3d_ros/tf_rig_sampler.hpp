#pragma once

#include <tf2_ros/buffer.h>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "c3d/pose_manifest.hpp"
#include "cognitive3d_ros/event_rate_limiter.hpp"
#include "cognitive3d_ros/pose_rig.hpp"
#include "cognitive3d_ros/session_runner.hpp"

namespace cognitive3d_ros {

// The event-cap source key the pose stream's own events are charged to. Not a topic name and
// unable to collide with one.
constexpr const char* kPoseRigEventSource = "pose_rig";

// A transform as the rig sampler reads it: ROS axes, and the stamp of the data it came from, 0
// for a static transform.
struct RigTransform {
  c3d::Vec3 translation;
  c3d::Quat rotation;
  std::int64_t stamp_ns = 0;
};

// What the rig sampler reads from tf. An interface so the sampler's rules -- first seen, hold,
// lost -- are tested against a tree a test writes tick by tick; Tf2RigTransforms is the real one.
class RigTransformSource {
 public:
  virtual ~RigTransformSource() = default;
  // Every frame tf knows, with its parent at the latest time, "" for a frame with none.
  virtual std::vector<RigFrame> Frames() const = 0;
  // `child` in `parent` at the latest time tf has for the chain.
  virtual bool Latest(const std::string& parent, const std::string& child,
                      RigTransform* out) const = 0;
  // `child` in `parent` at `stamp_ns`, or at `child`'s own latest when its broadcaster has not
  // published that far yet. Never waits. 0 is the latest.
  virtual bool AtStamp(const std::string& parent, const std::string& child, std::int64_t stamp_ns,
                       RigTransform* out) const = 0;
};

// The source over a tf2 buffer. The frame list and the parents come from BufferCore's
// `_getFrameStrings` and `_getParent`, which Humble and Jazzy both have; they are underscored
// for "not for general use", so each call is guarded and a throw reads as no parent.
class Tf2RigTransforms : public RigTransformSource {
 public:
  explicit Tf2RigTransforms(std::shared_ptr<tf2_ros::Buffer> buffer);
  std::vector<RigFrame> Frames() const override;
  bool Latest(const std::string& parent, const std::string& child,
              RigTransform* out) const override;
  bool AtStamp(const std::string& parent, const std::string& child, std::int64_t stamp_ns,
               RigTransform* out) const override;

 private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
};

// Where the rig sampler's output goes. SessionRigSink is the session; a test captures.
class PoseRigSink {
 public:
  virtual ~PoseRigSink() = default;
  virtual bool SendManifest(const c3d::pose::Manifest& manifest, std::string* error) = 0;
  // One sample of every joint of the last manifest sent, in its order and in ROS axes. `t_us` is
  // the wire's epoch microseconds; `enabled` is set only where visibility changes.
  virtual bool RecordSample(const std::string& manifest_id, std::int64_t t_us,
                            const std::vector<RosJointPose>& joints, std::optional<bool> enabled,
                            std::string* error) = 0;
  // An event stamped where the robot is, in ROS axes; `position_known` false before any body.
  virtual void RecordEvent(double wire_time, const std::string& name,
                           const c3d::Vec3& ros_position, bool position_known,
                           c3d::Properties properties) = 0;
};

// The session behind a SessionRunner, which applies the pose convention, with the rig's events
// through the per-source cap like every other event the node makes. A manifest that goes out also
// sets `c3d.ros.pose.frame_count` to its joint count.
class SessionRigSink : public PoseRigSink {
 public:
  SessionRigSink(SessionRunner& runner, EventRateLimiter* limiter)
      : runner_(runner), limiter_(limiter) {}
  bool SendManifest(const c3d::pose::Manifest& manifest, std::string* error) override;
  bool RecordSample(const std::string& manifest_id, std::int64_t t_us,
                    const std::vector<RosJointPose>& joints, std::optional<bool> enabled,
                    std::string* error) override;
  void RecordEvent(double wire_time, const std::string& name, const c3d::Vec3& ros_position,
                   bool position_known, c3d::Properties properties) override;

 private:
  SessionRunner& runner_;
  EventRateLimiter* limiter_;
};

struct TfRigSamplerConfig {
  std::string robot_frame = "base_link";
  // Frames left out of the rig, each with its subtree.
  std::set<std::string> excluded;
  RigManifestOptions manifest;
  // A transform whose stamp has not advanced for this long, on the wire's clock, is stale: tf
  // keeps answering a stopped broadcaster's last transform for ever. A stale body records
  // nothing; a stale joint is held. The node sets it with StaleTransformAfterSeconds from the
  // rig's own rate.
  double stale_after_s = 5.0;
  // A joint not seen for longer than this, on the wire's clock, leaves the rig: a manifest
  // re-send without it and a `robot.frame_lost` event. tf2's cache window, which is how long tf
  // itself keeps a frame's history.
  double frame_lost_after_s = 10.0;
  // A body missing this long, on the wire's clock, is recorded as one sample with `enabled`
  // false, so a viewer seeking into the gap does not draw the rig standing where it was last
  // seen. The next sample with a body says `enabled` true.
  double disable_after_s = 1.0;
};

// Samples the robot's tf tree into the pose stream: one rig per robot, one manifest entry, one
// sample per tick (c3d-stream-formatter's ros2/1 profile).
//
// Each tick looks up the body as the gaze channel does, in the reference frame the gaze channel
// is in, at the latest time tf has, so the rig's root is where the gaze channel puts the robot.
// Every other joint is looked up in its tf parent at the BODY's stamp, so one sample is one
// instant of the robot, falling back to the joint's own latest when its broadcaster has not
// published that far yet.
//
// The joint list follows tf, and every change to it is a manifest re-send:
//   - a frame joins after its first successful lookup, since before it there is no value to send;
//   - a frame whose lookup fails is held: its last value is sent again and counted, once per
//     joint and sample, in held_frames(), and the first hold of each frame is said once;
//   - a frame not seen for longer than `frame_lost_after_s` leaves, with its subtree, and records
//     `robot.frame_lost` with the frame and the epoch second it was last seen; if it comes back,
//     it joins again;
//   - a frame whose tf parent changes is sent again under the new one.
class TfRigSampler {
 public:
  enum class Outcome {
    kRecorded,
    // The gaze channel has not settled on a reference frame yet.
    kNoReference,
    kNoBody,
    kStaleBody,
    // Nothing recorded: the session refused the manifest or the sample, which TakeWarnings says
    // once per reason, or the wall clock stepped back behind the last sample.
    kRefused,
  };

  TfRigSampler(std::shared_ptr<const RigTransformSource> source, TfRigSamplerConfig config);

  // One tick at `wire_time` (epoch seconds, the wire's clock). `reference` is the frame the gaze
  // channel is sampling in, "" while it has none.
  Outcome Tick(double wire_time, const std::string& reference, PoseRigSink& sink);
  // Everything per session: the next session sends its manifest again from nothing.
  void Reset();

  const TfRigSamplerConfig& config() const { return config_; }
  std::uint64_t samples() const { return samples_; }
  std::uint64_t held_frames() const { return held_frames_; }
  std::uint64_t frames_lost() const { return frames_lost_; }
  std::uint64_t manifest_sends() const { return manifest_sends_; }
  // The joints of the manifest last sent, root first; empty before the first.
  const std::vector<RigFrame>& joints() const { return joints_; }
  // What the node should log, each said once: the first hold of a frame, a refused send.
  std::vector<std::string> TakeWarnings();

 private:
  struct FrameState {
    bool seen = false;
    RigTransform value;
    // The stamp in `value`, and the wire time it last advanced at.
    std::int64_t stamp_ns = 0;
    double stamp_since = 0.0;
    double last_seen = 0.0;
    bool hold_said = false;
  };

  // A tick with no body: past `disable_after_s`, one sample of the last values, `enabled` false.
  void MissBody(double wire_time, PoseRigSink& sink);
  bool Record(std::int64_t t_us, const std::vector<RosJointPose>& joints,
              std::optional<bool> enabled, PoseRigSink& sink);

  std::shared_ptr<const RigTransformSource> source_;
  TfRigSamplerConfig config_;
  std::map<std::string, FrameState> frames_;
  std::vector<RigFrame> joints_;
  std::vector<RosJointPose> last_values_;
  bool manifest_sent_ = false;
  bool enabled_ = true;
  bool have_body_ = false;
  c3d::Vec3 body_position_;
  double body_seen_ = 0.0;
  std::int64_t body_stamp_ns_ = -1;
  std::string body_reference_;
  double body_stamp_since_ = 0.0;
  std::int64_t last_t_us_ = -1;
  std::uint64_t samples_ = 0;
  std::uint64_t held_frames_ = 0;
  std::uint64_t frames_lost_ = 0;
  std::uint64_t manifest_sends_ = 0;
  std::vector<std::string> warnings_;
  std::string last_refusal_;
};

}  // namespace cognitive3d_ros
