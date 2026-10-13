#include "cognitive3d_ros/pose_sampler.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <vector>

namespace cognitive3d_ros {
namespace {

c3d::Vec3 ToVec3(const geometry_msgs::msg::Vector3& v) {
  return c3d::Vec3{v.x, v.y, v.z};
}

c3d::Quat ToQuat(const geometry_msgs::msg::Quaternion& q) {
  return c3d::Quat{q.x, q.y, q.z, q.w};
}

c3d::Quat QuatMul(const c3d::Quat& a, const c3d::Quat& b) {
  return c3d::Quat{a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                   a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                   a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                   a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// Compose a body-frame yaw onto an orientation. Post-multiplying applies the yaw
// in the object's own frame, which is what a mesh alignment offset means.
c3d::Quat ApplyBodyYaw(const c3d::Quat& q, double radians) {
  if (radians == 0.0) return q;
  const double s = std::sin(radians / 2.0), c = std::cos(radians / 2.0);
  return QuatMul(q, c3d::Quat{0.0, 0.0, s, c});
}

// Re-expresses an optical-frame orientation as its body-frame equivalent:
//   R(q * kOpticalToBody) . (1,0,0) == R(q) . (0,0,1)
// and likewise for the other two axes, so the result is a genuine body-style
// rotation and not merely one whose forward axis happens to agree.
//
// This is the inverse of the (-0.5, 0.5, -0.5, 0.5) that every ROS camera driver
// publishes between `*_camera_frame` and `*_optical_frame`.
const c3d::Quat kOpticalToBody{0.5, -0.5, 0.5, 0.5};


// The shortest angle between two orientations, in degrees: the geodesic between
// them on the unit quaternion sphere.
//
// Not a difference of Euler angles. Those are discontinuous at their own wrap
// points, so a yaw stepping 179 -> -179 -- two degrees of real rotation -- reads
// as 358, and the jump detector would fire on the arithmetic rather than on the
// robot. The quaternion distance has no wrap to be near.
//
// |dot| and not dot, because q and -q name the same rotation and a bare dot
// product reads the sign flip as a half turn. Defence and not an observed case:
// a quaternion published with the opposite sign does not arrive here with it,
// because tf2 canonicalises on the way through -- pinned by nothing, therefore,
// since a test for it passes whether the absolute value is there or not.
double AngleBetweenDegrees(const c3d::Quat& a, const c3d::Quat& b) {
  const double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  // acos is defined on [-1, 1] and rounding in a near-identical pair puts |dot|
  // a hair above 1, where acos is NaN -- which compares false against every
  // threshold and would silently disable the check for the stillest robots.
  const double clamped = std::min(1.0, std::fabs(dot));
  return 2.0 * std::acos(clamped) * 180.0 / M_PI;
}

// Rotate a vector by a unit quaternion: v + w t + q.xyz x t, with t = 2 q.xyz x v.
c3d::Vec3 Rotate(const c3d::Quat& q, const c3d::Vec3& v) {
  const double tx = 2.0 * (q.y * v.z - q.z * v.y);
  const double ty = 2.0 * (q.z * v.x - q.x * v.z);
  const double tz = 2.0 * (q.x * v.y - q.y * v.x);
  return c3d::Vec3{v.x + q.w * tx + q.y * tz - q.z * ty,
                   v.y + q.w * ty + q.z * tx - q.x * tz,
                   v.z + q.w * tz + q.x * ty - q.y * tx};
}

// A frame attached to the robot, posed in the reference frame at the instant the
// BODY transform resolved to -- the body's own pose composed with
// `robot_frame -> frame` looked up at the body's stamp.
//
// The whole chain looked up at "latest" is not that instant. tf2 resolves
// "latest" to the oldest of the newest stamps along a chain, so a chain that adds a joint
// broadcaster on its own clock resolves to a DIFFERENT instant from the body's,
// and on a turning robot the camera's pose then trails the body's by that
// difference: a 20 Hz head broadcaster behind a SLAM body pose puts the head
// pose tens of milliseconds behind, which at 128 deg/s of yaw is 15 degrees of
// head turn that never happened. Composing onto the body transform itself makes
// the body part of the two poses the same numbers, not merely the same moment.
//
// The joint is looked up at the body's stamp and, when its broadcaster has not
// yet published that far, at its own latest instead. Never waits: this runs on
// the sampling timer, and a joint at most one broadcaster period old is visible
// only while the joint itself is moving, where the skew it replaces is visible
// whenever the robot turns. A joint with no data at all fails, and the caller
// decides what a failure means.
bool LookupAtBodyTime(const tf2_ros::Buffer& buffer, const std::string& robot_frame,
                      const geometry_msgs::msg::TransformStamped& body,
                      const std::string& frame, c3d::Vec3* origin, c3d::Quat* rotation) {
  const c3d::Vec3 body_position = ToVec3(body.transform.translation);
  const c3d::Quat body_rotation = ToQuat(body.transform.rotation);
  if (frame == robot_frame) {
    *origin = body_position;
    *rotation = body_rotation;
    return true;
  }
  const tf2::TimePoint body_time(std::chrono::seconds(body.header.stamp.sec) +
                                 std::chrono::nanoseconds(body.header.stamp.nanosec));
  geometry_msgs::msg::TransformStamped joint;
  try {
    joint = buffer.lookupTransform(robot_frame, frame, body_time);
  } catch (const tf2::ExtrapolationException&) {
    try {
      joint = buffer.lookupTransform(robot_frame, frame, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      return false;
    }
  } catch (const tf2::TransformException&) {
    return false;
  }
  const c3d::Vec3 offset = Rotate(body_rotation, ToVec3(joint.transform.translation));
  *origin = c3d::Vec3{body_position.x + offset.x, body_position.y + offset.y,
                      body_position.z + offset.z};
  *rotation = QuatMul(body_rotation, ToQuat(joint.transform.rotation));
  return true;
}

}  // namespace

bool ParseFrameConvention(const std::string& text, FrameConvention* out) {
  if (text.empty() || text == "auto") { *out = FrameConvention::kAuto; return true; }
  if (text == "body_forward_x" || text == "body") {
    *out = FrameConvention::kBodyForwardX;
    return true;
  }
  if (text == "optical_forward_z" || text == "optical") {
    *out = FrameConvention::kOpticalForwardZ;
    return true;
  }
  return false;
}

const char* FrameConventionName(FrameConvention c) {
  switch (c) {
    case FrameConvention::kBodyForwardX: return "body_forward_x";
    case FrameConvention::kOpticalForwardZ: return "optical_forward_z";
    case FrameConvention::kAuto: break;
  }
  return "auto";
}

bool FrameNameLooksOptical(const std::string& frame) {
  // Any `_` or `/` delimited token equal to "optical", case-insensitively.
  // A suffix test on `_optical_frame` / `_optical` alone misses the real names
  // remapped multi-robot graphs produce -- `camera_optical_frame_2`,
  // `oakd/RGB_CAMERA_OPTICAL_FRAME` -- and a miss here is silent: the rotation
  // comes out 90 degrees off, every record is still well formed, and
  // node.cpp's warning is gated on precisely the condition that a miss makes
  // false. A frame carrying the token "optical" that is not an optical frame is
  // not a thing; the residual risk is the other direction, which is what the
  // explicit `gaze_frame_convention` override and the startup log are for.
  //
  // One named counter-example, because the token is NOT universally reliable and
  // claiming it was would be worse than the special case: a PX4/mavros optical
  // FLOW sensor publishes `optical_flow_link` / `optical_flow_frame`, which is
  // body-style and carries the token. Matching the token without excluding it
  // turns a working frame into a silent 90 degree error.
  // The exclusion is an ADJACENCY -- `optical` immediately followed by `flow` --
  // and not a `flow` token anywhere in the name. Unscoped, it would defeat the
  // very namespaced frames the token match exists for:
  // `robot_flow/oakd_rgb_camera_optical_frame` would resolve to body and every
  // gaze sample come out 90 degrees off, with the warning that guards this gated
  // on the same predicate and therefore unable to fire.
  bool optical_camera = false;
  std::string token, previous;
  auto take = [&] {
    if (previous == "optical" && token == "flow") {
      optical_camera = false;  // an optical FLOW sensor: body-style
    } else if (token == "optical") {
      optical_camera = true;
    }
    previous = token;
    token.clear();
  };
  for (std::size_t i = 0; i <= frame.size(); ++i) {
    const char c = i < frame.size() ? frame[i] : '_';
    if (c == '_' || c == '/') {
      take();
    } else {
      token.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  return optical_camera;
}

FrameConvention ResolveFrameConvention(const std::string& frame, FrameConvention configured) {
  if (configured != FrameConvention::kAuto) return configured;
  return FrameNameLooksOptical(frame) ? FrameConvention::kOpticalForwardZ
                                      : FrameConvention::kBodyForwardX;
}

PoseSampler::PoseSampler(std::shared_ptr<tf2_ros::Buffer> buffer, PoseSamplerConfig config)
    : buffer_(std::move(buffer)),
      config_(std::move(config)),
      resolved_gaze_frame_convention_(
          ResolveFrameConvention(config_.gaze_frame, config_.gaze_frame_convention)) {}

double StaleTransformAfterSeconds(double rate_hz) {
  const double periods = rate_hz > 0.0 ? 10.0 / rate_hz : 0.0;
  return std::max(5.0, periods);
}

bool RelocalisationCheckDue(double elapsed_s, double rate_hz) {
  constexpr double kInterval = PoseSamplerConfig::kRelocalisationCheckIntervalS;
  const double period = rate_hz > 0.0 ? 1.0 / rate_hz : kInterval;
  return elapsed_s >= kInterval - 0.1 * std::min(kInterval, period);
}

void PoseSampler::Reset() {
  have_last_ = false;
  have_jump_anchor_ = false;
  relocalisation_jumps_ = 0;
  lookup_failures_ = 0;
  stale_transforms_ = 0;
  last_sample_stale_ = false;
  last_stamp_ns_ = -1;
  last_stamp_frame_.clear();
  last_stamp_since_ = 0.0;
  gaze_frame_lookup_failures_ = 0;
  gaze_frame_failures_noticed_ = 0;
  holding_for_preferred_frame_ = false;
  fallback_seen_ = false;
  fallback_first_seen_ = 0.0;
  topology_emitted_ = false;
  active_reference_frame_.clear();
}

bool PoseSampler::last_body_position(c3d::Vec3* out) const {
  if (!have_last_) return false;
  if (out) *out = last_position_;
  return true;
}

bool PoseSampler::ConsumeGazeFrameFailureNotice() {
  if (gaze_frame_lookup_failures_ <= gaze_frame_failures_noticed_) return false;
  gaze_frame_failures_noticed_ = gaze_frame_lookup_failures_;
  return true;
}

void PoseSampler::EmitTopology(double wire_time, SessionRunner& runner) {
  if (topology_emitted_ || runner.session() == nullptr || !buffer_) return;
  std::vector<std::string> frames;
  buffer_->_getFrameStrings(frames);

  c3d::Properties props;
  props.emplace_back("frame_count", static_cast<std::int64_t>(frames.size()));
  props.emplace_back("reference_frame", active_reference_frame_.empty()
                                            ? config_.preferred_reference_frame
                                            : active_reference_frame_);
  props.emplace_back("robot_frame", config_.robot_frame);

  // Bounded: a URDF with 200 links would otherwise produce a property per link.
  std::ostringstream joined;
  std::size_t taken = 0;
  for (const auto& f : frames) {
    if (taken >= 40) {
      joined << ", ... (" << (frames.size() - taken) << " more)";
      break;
    }
    if (taken > 0) joined << ", ";
    joined << f;
    ++taken;
  }
  props.emplace_back("frames", joined.str());
  // Emitted from the tail of a successful sample, so the body position is already
  // there; the guard is for a future caller, not for this one.
  c3d::Vec3 position;
  const bool position_known = last_body_position(&position);
  // Marked emitted only once it actually went, so a sample that also reported a
  // frame decision does not cost the session its topology: at a cap of 1 Hz the
  // two compete for one token, and setting the flag regardless would drop the
  // only copy there will ever be. Retried on the next sample instead.
  if (RecordEvent(wire_time, runner, "robot.topology", position, position_known,
                  std::move(props))) {
    topology_emitted_ = true;
  }
}

bool PoseSampler::RecordEvent(double wire_time, SessionRunner& runner, const std::string& name,
                              const c3d::Vec3& position, bool position_known,
                              c3d::Properties properties) {
  return RecordCappedEvent(runner, event_limiter_, kPoseSamplerEventSource, wire_time, name,
                           position, position_known, std::move(properties));
}

bool PoseSampler::LookupSensorPose(const std::string& frame, const rclcpp::Time& stamp,
                                   c3d::Vec3* position, c3d::Quat* rotation) const {
  if (!buffer_ || frame.empty()) return false;

  // Whatever the gaze channel settled on, and nothing else. Until it has settled
  // there is no answer to give, so the record is DROPPED and counted, for the
  // same reason Sample() drops its own inside `reference_frame_grace_s`:
  // resolving the fallback here while the gaze channel still holds out for the
  // preferred frame puts the first seconds of pictures in `odom` beside a pose
  // timeline in `map`, and the row carries nothing that says which one it is.
  // An empty preferred frame is the deliberate fallback-only run, which has
  // nothing to wait for.
  std::string reference = active_reference_frame_;
  if (reference.empty()) {
    if (!config_.preferred_reference_frame.empty()) return false;
    reference = config_.fallback_reference_frame;
  }
  if (reference.empty()) return false;

  try {
    const auto transform = buffer_->lookupTransform(
        reference, frame, tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())));
    if (position != nullptr) *position = ToVec3(transform.transform.translation);
    if (rotation != nullptr) *rotation = ToQuat(transform.transform.rotation);
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

bool PoseSampler::Sample(double wire_time, SessionRunner& runner, std::string* status_out) {
  if (!buffer_ || runner.session() == nullptr) return false;

  geometry_msgs::msg::TransformStamped transform;
  std::string used_frame;
  std::string error;

  // TimePointZero means "latest available", which sidesteps extrapolation
  // exceptions entirely. We are sampling for analytics, not closing a control
  // loop, so the latest pose is the right one.
  auto try_lookup = [&](const std::string& reference) {
    if (reference.empty()) return false;
    std::string why;
    if (!buffer_->canTransform(reference, config_.robot_frame, tf2::TimePointZero,
                               tf2::Duration(0), &why)) {
      if (error.empty()) error = reference + ": " + why;
      return false;
    }
    try {
      transform = buffer_->lookupTransform(reference, config_.robot_frame, tf2::TimePointZero);
      used_frame = reference;
      return true;
    } catch (const tf2::TransformException& e) {
      if (error.empty()) error = std::string(reference) + ": " + e.what();
      return false;
    }
  };

  holding_for_preferred_frame_ = false;
  last_sample_stale_ = false;
  const bool on_preferred = try_lookup(config_.preferred_reference_frame);
  if (!on_preferred && !try_lookup(config_.fallback_reference_frame)) {
    ++lookup_failures_;
    if (status_out) *status_out = error.empty() ? "no transform available" : error;
    return false;
  }

  // Only the fallback answers, and nothing has been adopted yet: give the
  // preferred frame its grace before treating that as the run's answer. The
  // sample is DROPPED rather than taken from the fallback, because a pose in the
  // wrong frame is not a lesser version of the right one -- odom's origin is
  // wherever the robot booted, so it lands metres from the scene.
  if (!on_preferred && active_reference_frame_.empty() &&
      !config_.preferred_reference_frame.empty()) {
    if (!fallback_seen_) {
      fallback_seen_ = true;
      fallback_first_seen_ = wire_time;
    }
    if (wire_time - fallback_first_seen_ < config_.reference_frame_grace_s) {
      holding_for_preferred_frame_ = true;
      if (status_out) {
        std::ostringstream waiting;
        waiting << config_.preferred_reference_frame << " -> " << config_.robot_frame
                << " has not resolved yet; holding up to " << config_.reference_frame_grace_s
                << "s for it before falling back to " << config_.fallback_reference_frame;
        *status_out = waiting.str();
      }
      return false;
    }
  }

  // A stamp that has not moved since the last sample, for longer than the bound, is a publisher
  // that stopped: tf still answers with its last transform, which is not where the robot is now.
  // Stamp zero is a static chain, which never moves and is never stale.
  if (config_.stale_transform_after_s > 0.0) {
    const std::int64_t stamp_ns =
        static_cast<std::int64_t>(transform.header.stamp.sec) * 1000000000 +
        transform.header.stamp.nanosec;
    if (stamp_ns != last_stamp_ns_ || used_frame != last_stamp_frame_) {
      last_stamp_ns_ = stamp_ns;
      last_stamp_frame_ = used_frame;
      last_stamp_since_ = wire_time;
    } else if (stamp_ns != 0 &&
               wire_time - last_stamp_since_ > config_.stale_transform_after_s) {
      ++stale_transforms_;
      last_sample_stale_ = true;
      // The next fresh pose is compared with nothing, rather than with a pose from before the
      // gap: a robot that moved while its transform was stale did not jump.
      have_last_ = false;
      have_jump_anchor_ = false;
      if (status_out) {
        std::ostringstream stale;
        stale << used_frame << " -> " << config_.robot_frame << " has not advanced for "
              << (wire_time - last_stamp_since_) << "s";
        *status_out = stale.str();
      }
      return false;
    }
  }

  // Read before the frame report below, because the events that report the frame
  // are stamped where the robot is in the frame they are reporting.
  const c3d::Vec3 position = ToVec3(transform.transform.translation);
  const c3d::Quat rotation = ToQuat(transform.transform.rotation);

  if (used_frame != active_reference_frame_) {
    // Falling back from map to odom, or recovering, changes the meaning of every
    // subsequent position. Worth an event rather than a silent switch.
    if (!active_reference_frame_.empty()) {
      c3d::Properties props;
      props.emplace_back("from", active_reference_frame_);
      props.emplace_back("to", used_frame);
      RecordEvent(wire_time, runner, "robot.reference_frame_changed", position, true,
                  std::move(props));
      have_last_ = false;
      have_jump_anchor_ = false;
    } else if (!config_.preferred_reference_frame.empty() &&
               used_frame != config_.preferred_reference_frame) {
      // STARTING on the fallback is not a transition, so the branch above cannot see it -- and it
      // is the more damaging case. A session that never reached its preferred frame is anchored
      // to one whose origin is wherever the robot booted, so the whole trajectory sits at the
      // spawn offset from the scene it is drawn in. `map` does not exist at all on a stock robot
      // without amcl or SLAM, which makes this the DEFAULT outcome rather than an edge case.
      // An EMPTY preferred frame is not that case: `try_lookup` treats it as unconfigured, so
      // a fallback-only run is deliberate and has nothing to report.
      c3d::Properties props;
      props.emplace_back("wanted", config_.preferred_reference_frame);
      props.emplace_back("using", used_frame);
      RecordEvent(wire_time, runner, "robot.reference_frame_unavailable", position, true,
                  std::move(props));
    }
    active_reference_frame_ = used_frame;
  }

  // A map-frame pose jumps discretely on every localisation update, in POSITION
  // or in ORIENTATION and not necessarily in both. Annotating the jump lets a
  // spatial aggregate exclude it instead of counting it as several metres of
  // travel -- and, for the rotational case, keeps an orientation the robot never
  // took from reaching the gaze channel unlabelled.
  //
  // One event type for both, and one counter. A reader already handling
  // `robot.localisation_jump` keeps working and gains a property; a second event
  // name would have to be discovered before it could be ignored.
  //
  // Compared with the pose one check interval back, not with the previous sample,
  // and then compared from here: a correction tf2 spreads over a few samples is
  // seen whole, and a discrete one is seen once. A wire time earlier than the
  // anchor's is a clock that went back, and the anchor is taken again.
  const bool on_preferred_frame = used_frame == config_.preferred_reference_frame;
  if (on_preferred_frame && (!have_jump_anchor_ || wire_time < jump_anchor_time_)) {
    have_jump_anchor_ = true;
    jump_anchor_position_ = position;
    jump_anchor_rotation_ = rotation;
    jump_anchor_time_ = wire_time;
  } else if (on_preferred_frame &&
             RelocalisationCheckDue(wire_time - jump_anchor_time_, config_.rate_hz)) {
    const double dx = position.x - jump_anchor_position_.x;
    const double dy = position.y - jump_anchor_position_.y;
    const double dz = position.z - jump_anchor_position_.z;
    const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double turn_deg = AngleBetweenDegrees(jump_anchor_rotation_, rotation);
    const bool moved = step > config_.relocalisation_jump_m;
    const bool turned = turn_deg > config_.relocalisation_jump_deg;
    // A sample that trips both is ONE jump, not two: it is one discontinuity in
    // one pose, and counting it twice would make the health series disagree with
    // the number of events it is derived from.
    if (moved || turned) {
      ++relocalisation_jumps_;
      c3d::Properties props;
      props.emplace_back("distance_m", step);
      props.emplace_back("rotation_deg", turn_deg);
      // Both magnitudes travel on every jump, so a reader can see how far the
      // other axis moved; `kind` says which one crossed its threshold, which the
      // magnitudes alone cannot tell it without knowing the configured limits.
      props.emplace_back("kind", std::string(moved ? (turned ? "both" : "position")
                                                   : "rotation"));
      props.emplace_back("frame", used_frame);
      RecordEvent(wire_time, runner, "robot.localisation_jump", position, true,
                  std::move(props));
    }
    jump_anchor_position_ = position;
    jump_anchor_rotation_ = rotation;
    jump_anchor_time_ = wire_time;
  }
  last_position_ = position;
  have_last_ = true;

  // The gaze channel. Robots have no eyes, but they have a forward axis, and this
  // channel is what gives Session Replay its spatial timeline: the viewer places
  // the robot at `p` and turns it by `r`. Without it the session opens empty.
  //
  // Position and rotation, and no gaze point. A point a fixed distance along the
  // forward axis is not anywhere the robot looked, and the platform counts every
  // gaze point it is sent -- into heatmaps, gaze on objects and every gaze
  // aggregate -- so a made-up one is worse than none. Both come from one frame:
  // a camera's rotation beside the body's position is a viewpoint that exists
  // nowhere on the robot.
  c3d::Quat gaze_rotation = rotation;
  c3d::Vec3 gaze_origin = position;
  if (!config_.gaze_frame.empty()) {
    // At the body's instant, so the viewpoint and the dynamic object drawn under
    // it are one moment of the robot and not two.
    c3d::Vec3 frame_origin;
    c3d::Quat frame_rotation;
    if (LookupAtBodyTime(*buffer_, config_.robot_frame, transform, config_.gaze_frame,
                         &frame_origin, &frame_rotation)) {
      gaze_origin = frame_origin;
      gaze_rotation = frame_rotation;
      if (resolved_gaze_frame_convention_ == FrameConvention::kOpticalForwardZ) {
        gaze_rotation = QuatMul(gaze_rotation, kOpticalToBody);
      }
    } else {
      // Fall back to the body pose rather than dropping the sample -- a typo in
      // a frame name must not cost the whole session. But it has to be COUNTED,
      // because every other signal says the run is fine: the fallback moves the
      // position and the rotation together, the sample is recorded, and the
      // session properties still name the gaze frame. The only symptom of
      // `oakd_rgb_camera_optical_fram` is a replay whose viewpoint sits at floor
      // level.
      ++gaze_frame_lookup_failures_;
    }
  }

  runner.RecordPose(wire_time, gaze_origin, gaze_rotation);

  runner.RegisterRobot(wire_time, config_.robot_display_name, config_.robot_mesh);
  // The dynamic object is the robot BODY -- always base_link, never the camera --
  // and it gets the mesh alignment offset so the model faces where the robot is
  // going. Applying that offset to the gaze pose instead would turn the viewpoint
  // off the path, which just relocates the bug.
  runner.RecordRobotDynamic(
      wire_time, position,
      ApplyBodyYaw(rotation, config_.mesh_yaw_offset_deg * M_PI / 180.0));

  EmitTopology(wire_time, runner);
  if (status_out) *status_out = used_frame;
  return true;
}

}  // namespace cognitive3d_ros
