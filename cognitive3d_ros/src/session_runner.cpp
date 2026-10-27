#include "cognitive3d_ros/session_runner.hpp"

#include "c3d/version.hpp"

namespace cognitive3d_ros {

SessionRunner::SessionRunner(const c3d::Config& config, PoseConvention convention,
                             std::shared_ptr<c3d::ITransport> transport,
                             std::shared_ptr<c3d::ISpool> spool,
                             std::shared_ptr<c3d::ILogger> logger)
    : config_(config),
      convention_(convention),
      transport_(std::move(transport)),
      spool_(std::move(spool)),
      logger_(std::move(logger)) {
  std::string error;
  session_ = c3d::Session::Create(config_, transport_, spool_, logger_, &error);
  if (!session_ && logger_) logger_->Error("session could not be created: " + error);
}

int SessionRunner::RecoverOrphans() {
  return session_ ? session_->RecoverOrphanedSessions() : 0;
}

bool SessionRunner::Begin() {
  if (!session_) return false;
  robot_registered_ = false;
  if (!session_->Begin()) return false;
  session_->SetProperty("c3d.ros.pose_convention", std::string(PoseConventionName(convention_)));
  return true;
}

void SessionRunner::End(const std::string& reason, std::optional<double> end_time) {
  if (session_ && session_->active()) session_->End(reason, end_time);
}

bool SessionRunner::active() const { return session_ && session_->active(); }

void SessionRunner::RecordPose(double t, const c3d::Vec3& p, const c3d::Quat& r) {
  if (!session_) return;
  session_->RecordGaze(t, ConvertPosition(p, convention_), ConvertGazeRotation(r, convention_));
}

void SessionRunner::RegisterRobot(double t, const std::string& name, const std::string& mesh) {
  if (!session_ || !robot_dynamic_object_ || robot_registered_) return;
  session_->RegisterDynamic(robot_object_id_, name, mesh, t, c3d::Vec3{}, c3d::Quat{});
  robot_registered_ = true;
}

void SessionRunner::RecordEventAt(double t, const std::string& name,
                                  const c3d::Vec3& ros_position,
                                  const c3d::Properties& properties,
                                  const std::string& dynamic_id) {
  if (!session_) return;
  session_->RecordEvent(name, t, ConvertPosition(ros_position, convention_), properties,
                        dynamic_id);
}

bool RecordCappedEvent(SessionRunner& runner, EventRateLimiter* limiter,
                       const std::string& source, double wire_time, const std::string& name,
                       const c3d::Vec3& position, bool position_known,
                       c3d::Properties properties) {
  if (runner.session() == nullptr) return false;
  if (limiter != nullptr && !limiter->Allow(source, wire_time)) return false;

  c3d::Vec3 point;
  if (position_known) {
    point = position;
  } else {
    properties.emplace_back("c3d.position_unknown", true);
  }
  runner.RecordEventAt(wire_time, name, point, properties, std::string());
  return true;
}

c3d::Pose ScenePose(const c3d::Vec3& ros_position, const c3d::Quat& ros_rotation,
                    PoseConvention convention) {
  c3d::Pose pose;
  pose.position = ConvertPosition(ros_position, convention);
  pose.rotation = ConvertDynamicRotation(ros_rotation, convention);
  return pose;
}

c3d::Pose ScanPlanePose(const c3d::Vec3& ros_position, const c3d::Quat& ros_rotation,
                        PoseConvention convention) {
  c3d::Pose pose;
  pose.position = ConvertPosition(ros_position, convention);
  pose.rotation = ConvertScanPlaneRotation(ros_rotation, convention);
  return pose;
}

void SessionRunner::RecordCameraFrame(const std::string& camera_id, std::uint64_t time_ms,
                                      std::uint32_t seq, const std::string& frame_id,
                                      const c3d::Vec3& ros_position,
                                      const c3d::Quat& ros_rotation, const std::string& format,
                                      std::uint16_t width, std::uint16_t height,
                                      const std::string& bytes, const c3d::CameraFov& fov) {
  if (!session_) return;
  // The field of view is two angles and no direction, so no convention applies to it.
  session_->RecordCameraFrame(camera_id, time_ms, seq, frame_id,
                              ScenePose(ros_position, ros_rotation, convention_), format,
                              width, height, bytes, fov);
}

void SessionRunner::RecordLaserScan(const std::string& sensor_id, std::uint64_t time_ms,
                                    std::uint32_t seq, const std::string& frame_id,
                                    const c3d::Vec3& ros_position, const c3d::Quat& ros_rotation,
                                    const c3d::LaserScanData& scan) {
  if (!session_) return;
  session_->RecordLaserScan(sensor_id, time_ms, seq, frame_id,
                            ScanPlanePose(ros_position, ros_rotation, convention_), scan);
}

void SessionRunner::RecordPointCloud(const std::string& sensor_id, std::uint64_t time_ms,
                                     std::uint32_t seq, const std::string& frame_id,
                                     const c3d::Vec3& ros_position,
                                     const c3d::Quat& ros_rotation,
                                     const c3d::EncodedPointCloud& cloud) {
  if (!session_) return;
  session_->RecordPointCloud(sensor_id, time_ms, seq, frame_id,
                             ScenePose(ros_position, ros_rotation, convention_), cloud);
}

bool SessionRunner::SendPoseManifest(const c3d::pose::Manifest& manifest, std::string* error) {
  if (!session_) {
    if (error) *error = "no session";
    return false;
  }
  return session_->SetPoseManifest(manifest, error);
}

bool SessionRunner::RecordPoseRigSample(const std::string& manifest_id, std::int64_t t_us,
                                        const std::vector<RosJointPose>& ros_joints,
                                        std::optional<bool> enabled, std::string* error) {
  if (!session_) {
    if (error) *error = "no session";
    return false;
  }
  c3d::pose::PoseSample sample;
  sample.t_us = t_us;
  sample.enabled = enabled;
  sample.joints.reserve(ros_joints.size());
  for (const RosJointPose& joint : ros_joints) {
    sample.joints.push_back(RigJointScenePose(joint, convention_));
  }
  return session_->RecordPoseSample(manifest_id, sample, error);
}

void SessionRunner::RecordRobotDynamic(double t, const c3d::Vec3& p, const c3d::Quat& r) {
  // REGISTRATION is the gate, and it carries `robot_dynamic_object` for this channel too: with the
  // switch off nothing is registered, and a snapshot against an id no manifest names is a row the
  // replay cannot join to anything.
  if (!session_ || !robot_registered_) return;
  // Added along ROS up (+Z) BEFORE the conversion, so every convention carries it
  // to its own up axis by construction. Added afterwards it would have to name
  // the target frame's up axis per convention, and under rep103 -- where no
  // conversion happens -- the axis a Y-up convention calls up is ROS LEFT, so a
  // vertical trim would shift the model sideways.
  //
  // The DYNAMIC channel only. Gaze is a ray from the camera and events are
  // points on the floor; neither is a mesh with a mesh's origin, so applying this
  // to all three would lift the gaze origin off the robot and float every event
  // above the ground.
  const c3d::Vec3 lifted{p.x, p.y, p.z + dynamic_up_offset_meters_};
  session_->RecordDynamic(robot_object_id_, t, ConvertPosition(lifted, convention_),
                          ConvertDynamicRotation(r, convention_));
}

}  // namespace cognitive3d_ros
