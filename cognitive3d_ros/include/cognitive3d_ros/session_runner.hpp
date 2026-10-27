#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "c3d/media.hpp"
#include "c3d/pose_manifest.hpp"
#include "c3d/session.hpp"
#include "cognitive3d_ros/coordinate_transform.hpp"
#include "cognitive3d_ros/event_rate_limiter.hpp"
#include "cognitive3d_ros/pose_rig.hpp"

namespace rclcpp {
class Logger;
}

namespace cognitive3d_ros {

// The pose a camera frame and a point cloud carry: the position map, and the
// rotation conjugated by that same map. A LaserScan takes ScanPlanePose below.
//
// The ROTATION takes the DYNAMIC channel's rule, not the gaze channel's, and the
// difference is only visible under `gltf_authored`. The gaze rule carries an
// extra fixed turn that answers how the platform reads a gaze rotation, as a
// headset's, local +Z forward; nothing reads these records that way. What they
// must satisfy instead is
// `world = rotate(pose.rotation, ConvertPosition(p)) + pose.position` for a point
// cloud's own points, and only the conjugation of the position map -- the dynamic
// rule -- satisfies it. Sending the gaze rotation here would put every cloud 90
// degrees out while the pose itself still looked right.
c3d::Pose ScenePose(const c3d::Vec3& ros_position, const c3d::Quat& ros_rotation,
                    PoseConvention convention);

// The pose a LaserScan record carries. The same position, and the SCAN-PLANE
// rotation instead.
//
// The other two records carry coordinates this SDK already converted -- a camera
// frame carries none at all, a point cloud carries points that took the position
// map per point -- so the conjugated rotation is what reconstructs them. A
// LaserScan carries its rays as the sensor published them, angle and range in the
// sensor's own plane, and the only rotation that lands THOSE in the scene without
// the reader knowing the convention is ConvertScanPlaneRotation. The reader's rule
// is `world = rotate(pose.rotation, (r cos t, r sin t, 0)) + pose.position`.
c3d::Pose ScanPlanePose(const c3d::Vec3& ros_position, const c3d::Quat& ros_rotation,
                        PoseConvention convention);

// Owns the Cognitive3D session and everything around it that is not ROS.
//
// Kept separate from the node so the translators can be unit tested against a
// SessionRunner backed by a fake transport, with no ROS graph in the way.
class SessionRunner {
 public:
  SessionRunner(const c3d::Config& config, PoseConvention convention,
                std::shared_ptr<c3d::ITransport> transport,
                std::shared_ptr<c3d::ISpool> spool,
                std::shared_ptr<c3d::ILogger> logger);

  // Closes any session left open by a previous process that died without an
  // orderly shutdown. Call once before Begin. Returns how many were recovered.
  int RecoverOrphans();

  bool Begin();
  // Ends the current session. It does NOT begin another -- and a caller reopening one must go back
  // through the node, because `Session::End` restores the properties to the snapshot taken inside
  // `Begin`, so an End/Begin pair here yields a session carrying none of the node's identity, none
  // of the operator's `session_properties`, and no name.
  // `end_time` is forwarded to `Session::End`.
  void End(const std::string& reason, std::optional<double> end_time = std::nullopt);
  bool active() const;

  c3d::Session* session() { return session_.get(); }
  const c3d::Session* session() const { return session_.get(); }
  PoseConvention convention() const { return convention_; }

  // Poses enter the session model through these, and only these, so the
  // coordinate convention is applied in exactly one place.
  //
  // A gaze record is a position and a rotation and nothing else: the robot did not
  // look at any point, so none is sent.
  void RecordPose(double t, const c3d::Vec3& ros_position, const c3d::Quat& ros_rotation);
  void RecordRobotDynamic(double t, const c3d::Vec3& ros_position,
                          const c3d::Quat& ros_rotation);
  void RegisterRobot(double t, const std::string& name, const std::string& mesh);
  // An event stamped with a position, which therefore has to come through here
  // too: the header above promises the coordinate convention is applied in
  // exactly one place, and an event point is a position like any other. EVERY
  // event takes this route -- the externally published ones, the translators'
  // and the pose sampler's -- so there is no second spelling of the conversion.
  void RecordEventAt(double t, const std::string& name, const c3d::Vec3& ros_position,
                     const c3d::Properties& properties, const std::string& dynamic_id);

  // The binary streams. Same rule as above: the pose arrives in ROS axes and the
  // convention is applied here, in one place, by `ScenePose` -- or, for a
  // LaserScan alone, by `ScanPlanePose`.
  void RecordCameraFrame(const std::string& camera_id, std::uint64_t time_ms, std::uint32_t seq,
                         const std::string& frame_id, const c3d::Vec3& ros_position,
                         const c3d::Quat& ros_rotation, const std::string& format,
                         std::uint16_t width, std::uint16_t height, const std::string& bytes,
                         const c3d::CameraFov& fov);
  void RecordLaserScan(const std::string& sensor_id, std::uint64_t time_ms, std::uint32_t seq,
                       const std::string& frame_id, const c3d::Vec3& ros_position,
                       const c3d::Quat& ros_rotation, const c3d::LaserScanData& scan);
  void RecordPointCloud(const std::string& sensor_id, std::uint64_t time_ms, std::uint32_t seq,
                        const std::string& frame_id, const c3d::Vec3& ros_position,
                        const c3d::Quat& ros_rotation, const c3d::EncodedPointCloud& cloud);

  // The pose stream. A manifest carries no coordinates and goes to the session as it is; a
  // sample's joints arrive in ROS axes and each takes RigJointScenePose here, the root and the
  // parent-local joints alike. False, with the session's reason, for anything it refuses.
  bool SendPoseManifest(const c3d::pose::Manifest& manifest, std::string* error = nullptr);
  bool RecordPoseRigSample(const std::string& manifest_id, std::int64_t t_us,
                           const std::vector<RosJointPose>& ros_joints,
                           std::optional<bool> enabled, std::string* error = nullptr);

  // Whether this run registers the robot as a dynamic object and records a dynamics sample
  // per pose. OFF by default: the platform draws the robot from the pose timeline, so the
  // object -- and the mesh upload behind it -- is a legacy requirement of the dashboards that
  // still read the dynamics stream.
  //
  // Gaze, events, sensors and the binary streams are untouched either way. An event may still
  // name a dynamic object through `RecordEventAt`'s `dynamic_id`: the id is a value the caller
  // supplies, not a claim that this run registered anything.
  void set_robot_dynamic_object(bool enabled) { robot_dynamic_object_ = enabled; }
  bool robot_dynamic_object() const { return robot_dynamic_object_; }

  // The dynamic-object id this run emits, in the telemetry manifest and on every dynamics
  // sample. It is a CROSS-SYSTEM contract, not an internal name: the object uploaded to the
  // dashboard has to be registered under this same id, or the replay joins nothing to the
  // uploaded row and the analytics layer reports zero gaze on an object with a full timeline.
  const std::string& robot_object_id() const { return robot_object_id_; }
  void set_robot_object_id(const std::string& id) { robot_object_id_ = id; }

  // Raises the robot's dynamic-object position along UP, in metres, under every
  // pose convention, and on the DYNAMIC channel only -- gaze, events and sensors
  // are untouched.
  //
  // A vertical trim for a mesh whose origin is not where the consumer wants the
  // model to sit. It is a property of a particular mesh, which is why it is
  // configuration with a default of 0.0 and not a constant in here; a consumer
  // whose mesh origin is already correct never sets it.
  //
  // It does NOT compensate for any known viewer behaviour. A model rendered half
  // buried is more often a scene asset that declares its floor mesh twice, a few
  // centimetres apart, which is fixed in the scene's export rather than here.
  // Whether the viewer places a dynamic mesh by its own origin is untested; do
  // not treat this knob as evidence that it does not.
  void set_dynamic_up_offset_meters(double metres) { dynamic_up_offset_meters_ = metres; }
  double dynamic_up_offset_meters() const { return dynamic_up_offset_meters_; }

 private:
  c3d::Config config_;
  PoseConvention convention_;
  std::shared_ptr<c3d::ITransport> transport_;
  std::shared_ptr<c3d::ISpool> spool_;
  std::shared_ptr<c3d::ILogger> logger_;
  std::unique_ptr<c3d::Session> session_;
  std::string robot_object_id_ = "robot";
  // False here and false as the parameter's default, so an unconfigured runner records what an
  // unconfigured node records.
  bool robot_dynamic_object_ = false;
  bool robot_registered_ = false;
  double dynamic_up_offset_meters_ = 0.0;
};

// The door every event the node itself produces goes through: the translators',
// the pose channel's and the node's own. An externally published `~/events` message
// spends the same budget through the limiter directly, because it is the one event
// that can carry a dynamic-object id.
//
// Three rules in one place. The point is where the robot last was, and it reaches
// the session through SessionRunner, which owns the coordinate convention. A point
// that is not known yet is the origin plus `c3d.position_unknown` rather than a
// claim. And the per-source cap applies HERE: `source` is its key -- a topic for a
// translator, a channel name for an event no topic produced -- so one source in a
// loop spends its own budget and not the session's.
//
// Returns whether the event reached the session. A caller carrying a state machine
// has to READ this: the cap and the no-session check both discard, and a machine
// advanced past a discarded event describes a transition the session never saw --
// a suppressed `degraded` followed by a `recovered` from nothing.
bool RecordCappedEvent(SessionRunner& runner, EventRateLimiter* limiter,
                       const std::string& source, double wire_time, const std::string& name,
                       const c3d::Vec3& position, bool position_known,
                       c3d::Properties properties);

}  // namespace cognitive3d_ros
