#include "cognitive3d_ros/tf_rig_sampler.hpp"

#include <tf2/exceptions.h>

#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

namespace cognitive3d_ros {
namespace {

RigTransform FromMessage(const geometry_msgs::msg::TransformStamped& t) {
  RigTransform out;
  out.translation = c3d::Vec3{t.transform.translation.x, t.transform.translation.y,
                              t.transform.translation.z};
  out.rotation = c3d::Quat{t.transform.rotation.x, t.transform.rotation.y,
                           t.transform.rotation.z, t.transform.rotation.w};
  out.stamp_ns = static_cast<std::int64_t>(t.header.stamp.sec) * 1000000000 +
                 static_cast<std::int64_t>(t.header.stamp.nanosec);
  return out;
}

bool SameJoints(const std::vector<RigFrame>& a, const std::vector<RigFrame>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].name != b[i].name || a[i].parent != b[i].parent) return false;
  }
  return true;
}

}  // namespace

Tf2RigTransforms::Tf2RigTransforms(std::shared_ptr<tf2_ros::Buffer> buffer)
    : buffer_(std::move(buffer)) {}

std::vector<RigFrame> Tf2RigTransforms::Frames() const {
  std::vector<RigFrame> frames;
  if (!buffer_) return frames;
  std::vector<std::string> names;
  try {
    buffer_->_getFrameStrings(names);
  } catch (const std::exception&) {
    return frames;
  }
  for (const std::string& name : names) {
    std::string parent;
    try {
      if (!buffer_->_getParent(name, tf2::TimePointZero, parent)) parent.clear();
    } catch (const std::exception&) {
      parent.clear();
    }
    frames.push_back(RigFrame{name, parent});
  }
  return frames;
}

bool Tf2RigTransforms::Latest(const std::string& parent, const std::string& child,
                              RigTransform* out) const {
  if (!buffer_) return false;
  try {
    *out = FromMessage(buffer_->lookupTransform(parent, child, tf2::TimePointZero));
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

bool Tf2RigTransforms::AtStamp(const std::string& parent, const std::string& child,
                               std::int64_t stamp_ns, RigTransform* out) const {
  if (!buffer_) return false;
  if (stamp_ns <= 0) return Latest(parent, child, out);
  try {
    *out = FromMessage(buffer_->lookupTransform(
        parent, child, tf2::TimePoint(std::chrono::nanoseconds(stamp_ns))));
    return true;
  } catch (const tf2::ExtrapolationException&) {
    // The joint's broadcaster has not published as far as the body's yet, or tf no longer holds
    // that far back: its own latest. How old that is, is the sampler's question.
    return Latest(parent, child, out);
  } catch (const tf2::TransformException&) {
    return false;
  }
}

bool SessionRigSink::SendManifest(const c3d::pose::Manifest& manifest, std::string* error) {
  if (!runner_.SendPoseManifest(manifest, error)) return false;
  if (runner_.session() != nullptr) {
    runner_.session()->SetProperty("c3d.ros.pose.frame_count",
                                   static_cast<std::int64_t>(manifest.joints.size()));
  }
  return true;
}

bool SessionRigSink::RecordSample(const std::string& manifest_id, std::int64_t t_us,
                                  const std::vector<RosJointPose>& joints,
                                  std::optional<bool> enabled, std::string* error) {
  return runner_.RecordPoseRigSample(manifest_id, t_us, joints, enabled, error);
}

void SessionRigSink::RecordEvent(double wire_time, const std::string& name,
                                 const c3d::Vec3& ros_position, bool position_known,
                                 c3d::Properties properties) {
  RecordCappedEvent(runner_, limiter_, kPoseRigEventSource, wire_time, name, ros_position,
                    position_known, std::move(properties));
}

TfRigSampler::TfRigSampler(std::shared_ptr<const RigTransformSource> source,
                           TfRigSamplerConfig config)
    : source_(std::move(source)), config_(std::move(config)) {}

void TfRigSampler::Reset() {
  frames_.clear();
  joints_.clear();
  last_values_.clear();
  manifest_sent_ = false;
  enabled_ = true;
  have_body_ = false;
  body_position_ = c3d::Vec3{};
  body_seen_ = 0.0;
  body_stamp_ns_ = -1;
  body_reference_.clear();
  body_stamp_since_ = 0.0;
  last_t_us_ = -1;
  samples_ = 0;
  held_frames_ = 0;
  frames_lost_ = 0;
  manifest_sends_ = 0;
  warnings_.clear();
  last_refusal_.clear();
}

std::vector<std::string> TfRigSampler::TakeWarnings() {
  std::vector<std::string> taken;
  taken.swap(warnings_);
  return taken;
}

bool TfRigSampler::Record(std::int64_t t_us, const std::vector<RosJointPose>& joints,
                          std::optional<bool> enabled, PoseRigSink& sink) {
  // The encoder takes times that never decrease. A wall clock stepped back records nothing
  // until it passes the last sample again, rather than a time that never happened.
  if (t_us < last_t_us_) return false;
  std::string error;
  if (!sink.RecordSample(config_.manifest.manifest_id, t_us, joints, enabled, &error)) {
    if (error != last_refusal_) {
      last_refusal_ = error;
      warnings_.push_back("the pose stream refused a sample: " + error);
    }
    return false;
  }
  last_t_us_ = t_us;
  last_values_ = joints;
  ++samples_;
  return true;
}

void TfRigSampler::MissBody(double wire_time, PoseRigSink& sink) {
  if (!manifest_sent_ || !enabled_ || !have_body_ || last_values_.size() != joints_.size()) return;
  if (wire_time - body_seen_ < config_.disable_after_s) return;
  // The values it was last seen with, so a reader that ignores `enabled` still sees nothing
  // move that did not.
  const std::vector<RosJointPose> held = last_values_;
  if (Record(std::llround(wire_time * 1e6), held, false, sink)) enabled_ = false;
}

TfRigSampler::Outcome TfRigSampler::Tick(double wire_time, const std::string& reference,
                                         PoseRigSink& sink) {
  if (reference.empty()) {
    MissBody(wire_time, sink);
    return Outcome::kNoReference;
  }
  RigTransform body;
  if (!source_->Latest(reference, config_.robot_frame, &body)) {
    MissBody(wire_time, sink);
    return Outcome::kNoBody;
  }
  // The gaze channel's stale rule: a stamp that stops advancing is a broadcaster that stopped,
  // and tf answers with its last transform for ever. A static chain, stamp 0, never goes stale.
  if (reference != body_reference_ || body.stamp_ns != body_stamp_ns_) {
    body_reference_ = reference;
    body_stamp_ns_ = body.stamp_ns;
    body_stamp_since_ = wire_time;
  } else if (body.stamp_ns != 0 && wire_time - body_stamp_since_ > config_.stale_after_s) {
    MissBody(wire_time, sink);
    return Outcome::kStaleBody;
  }
  have_body_ = true;
  body_position_ = body.translation;
  body_seen_ = wire_time;

  std::vector<RigFrame> frames =
      RigFramesUnder(source_->Frames(), config_.robot_frame, config_.excluded);
  if (frames.empty()) frames.push_back(RigFrame{config_.robot_frame, ""});

  // Every frame under the body, looked up in its parent at the body's stamp.
  const std::int64_t stale_ns = static_cast<std::int64_t>(config_.stale_after_s * 1e9);
  std::set<std::string> fresh;
  for (std::size_t i = 1; i < frames.size(); ++i) {
    const RigFrame& frame = frames[i];
    FrameState& state = frames_[frame.name];
    RigTransform found;
    bool ok = source_->AtStamp(frame.parent, frame.name, body.stamp_ns, &found);
    if (ok && found.stamp_ns != 0) {
      // Stale two ways. Against the body: a joint whose newest transform is that much older than
      // the body's has stopped, which a session that starts after it stopped sees at once. Against
      // itself, for a body on a static chain, which has no stamp to compare with.
      if (body.stamp_ns != 0 && body.stamp_ns - found.stamp_ns > stale_ns) {
        ok = false;
      } else if (!state.seen || found.stamp_ns != state.stamp_ns) {
        state.stamp_ns = found.stamp_ns;
        state.stamp_since = wire_time;
      } else if (wire_time - state.stamp_since > config_.stale_after_s) {
        ok = false;
      }
    }
    if (ok) {
      state.seen = true;
      state.value = found;
      state.last_seen = wire_time;
      fresh.insert(frame.name);
    }
  }

  // Which joints this sample has: the rig in tf order, less what never resolved, less what has
  // been gone too long, less any frame whose parent is not in it.
  std::set<std::string> joined;
  for (const RigFrame& joint : joints_) joined.insert(joint.name);
  std::vector<RigFrame> next{frames[0]};
  std::set<std::string> in_next{frames[0].name};
  std::vector<RigFrame> lost;
  for (std::size_t i = 1; i < frames.size(); ++i) {
    const RigFrame& frame = frames[i];
    if (in_next.count(frame.parent) == 0) continue;
    if (fresh.count(frame.name) == 0) {
      if (joined.count(frame.name) == 0) continue;
      if (wire_time - frames_[frame.name].last_seen > config_.frame_lost_after_s) {
        lost.push_back(frame);
        continue;
      }
    }
    next.push_back(frame);
    in_next.insert(frame.name);
  }

  if (!manifest_sent_ || !SameJoints(next, joints_)) {
    std::string error;
    if (!sink.SendManifest(BuildRigManifest(next, config_.manifest), &error)) {
      if (error != last_refusal_) {
        last_refusal_ = error;
        warnings_.push_back("the pose stream refused the robot's manifest: " + error);
      }
      return Outcome::kRefused;
    }
    joints_ = next;
    manifest_sent_ = true;
    ++manifest_sends_;
    for (const RigFrame& frame : lost) {
      FrameState& state = frames_[frame.name];
      ++frames_lost_;
      c3d::Properties properties;
      properties.emplace_back("frame", frame.name);
      properties.emplace_back("parent", frame.parent);
      properties.emplace_back("last_seen_epoch_seconds", state.last_seen);
      sink.RecordEvent(wire_time, "robot.frame_lost", body_position_, true,
                       std::move(properties));
      // Said again if it comes back and is lost again. Its stamp is kept: a frame rejoins when
      // its transform advances, not because it was dropped.
      state.hold_said = false;
    }
  }

  std::vector<RosJointPose> values;
  values.reserve(joints_.size());
  values.push_back(RosJointPose{body.translation, body.rotation});
  std::uint64_t held = 0;
  for (std::size_t j = 1; j < joints_.size(); ++j) {
    FrameState& state = frames_[joints_[j].name];
    values.push_back(RosJointPose{state.value.translation, state.value.rotation});
    if (fresh.count(joints_[j].name) != 0) continue;
    ++held;
    if (!state.hold_said) {
      state.hold_said = true;
      std::ostringstream said;
      said << "pose stream: tf frame '" << joints_[j].name << "' could not be looked up in '"
           << joints_[j].parent << "'; its last value is sent again until it resolves, "
           << "counted in c3d.stream.pose.heldFrames, and it leaves the rig after "
           << config_.frame_lost_after_s << "s";
      warnings_.push_back(said.str());
    }
  }
  std::optional<bool> enabled;
  if (!enabled_) enabled = true;
  if (!Record(std::llround(wire_time * 1e6), values, enabled, sink)) return Outcome::kRefused;
  enabled_ = true;
  held_frames_ += held;
  return Outcome::kRecorded;
}

}  // namespace cognitive3d_ros
