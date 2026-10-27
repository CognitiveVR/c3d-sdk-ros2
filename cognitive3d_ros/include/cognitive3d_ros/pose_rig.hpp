// The robot as a rig on the pose stream: which tf frames it is made of, the manifest entry that
// describes them, and the conversion every joint takes. Pure functions over frame lists and poses,
// with no ROS in them, so every rule here is testable without a tf buffer.
//
// The profile is `ros2/1`: one rig per robot. Joint 0 is `robot_frame`, carrying its pose in the
// session's reference frame; every other tf frame under it is a joint carrying its transform in
// its tf parent; `meshName` is the frame's name. Everything the manifest says comes from tf and
// from this node's own configuration (tier 1). A URDF on `/robot_description` would add more keys
// (tier 2), through the seam marked below; nothing implements it yet.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "c3d/pose.hpp"
#include "c3d/pose_manifest.hpp"
#include "cognitive3d_ros/coordinate_transform.hpp"

namespace cognitive3d_ros {

// The manifest's `c3d.profile`: which key convention its properties follow. Versioned on its
// own, apart from the SDK and from any model format.
constexpr const char* kRigProfile = "ros2/1";

// One tf frame and its parent at the latest time tf knows, "" for a frame with none.
struct RigFrame {
  std::string name;
  std::string parent;
};

// The rig's frames, in manifest order: `robot_frame` first, then every frame whose chain of
// parents reaches it, depth first with siblings by name, so a parent always comes before its
// children and the same tree always gives the same order.
//
// An excluded frame takes its whole subtree with it: a frame below it would name a parent the
// rig does not have. `robot_frame` itself is never excluded (configure refuses it). Empty when
// `robot_frame` is not in `tree` at all. `parent` is "" on the first entry.
std::vector<RigFrame> RigFramesUnder(const std::vector<RigFrame>& tree,
                                     const std::string& robot_frame,
                                     const std::set<std::string>& excluded);

// The `c3d.kind` each configured sensor's frame carries, from (frame, kind) pairs in the order
// cameras, lidars, described sensors. A frame named twice keeps the first kind it was given, so an
// RGB-D unit whose camera and point cloud share one frame is tagged `camera`. An empty frame
// names nothing.
std::map<std::string, std::string> RigFrameKinds(
    const std::vector<std::pair<std::string, std::string>>& frames);

// `ros2-` and 16 hex digits: the 64-bit FNV-1a hash of `device_id`'s bytes. The same robot gets
// the same manifest id in every session without configuration, which is what lets a consumer
// follow one robot across sessions; FNV-1a is fixed by its definition, so no compiler, platform
// or library version can change it.
std::string DefaultPoseManifestId(const std::string& device_id);

struct RigManifestOptions {
  std::string manifest_id;
  // `name` and `ros.robotName`: the node's `device_id`, until a URDF names the robot.
  std::string robot_name;
  // The uploaded per-link model and its file type, both "" until one exists: an empty mesh
  // tells Session Replay there is nothing to draw from this stream, and the robot is drawn from
  // the gaze stream as before.
  std::string mesh;
  std::string file_type;
  // tf frame -> `c3d.kind`, from RigFrameKinds.
  std::map<std::string, std::string> kinds;
};

// TIER 2 SEAM. A URDF read from `/robot_description` decorates the tf-only manifest: object keys
// such as `ros.urdfSha256` and `c3d.model.format`, joint keys such as `ros.jointName`,
// `ros.jointType` and `ros.limit.*`, matched by link name. tf still decides which joints exist;
// a decoration only adds keys, and the result must still pass c3d::pose::ManifestProblem. Not
// implemented: BuildRigManifest takes one so that the URDF reader plugs in here and nowhere
// else.
using RigManifestDecoration = std::function<void(c3d::pose::Manifest*)>;

// The tf-only (tier 1) manifest of the rig `frames`, as RigFramesUnder orders them: one entry,
// `riggingMappingType` "meshName", joint j named after frame j and carrying position and
// rotation, never scale. Object properties `c3d.profile` and `ros.robotName`; joint properties
// `ros.parent` on every joint but the root, and `c3d.kind` on a configured sensor's frame.
// `root` is "": the rig has joints of its own.
c3d::pose::Manifest BuildRigManifest(const std::vector<RigFrame>& frames,
                                     const RigManifestOptions& options,
                                     const RigManifestDecoration& decorate = nullptr);

// One joint's transform in ROS axes: joint 0 in the reference frame, every other joint in its tf
// parent.
struct RosJointPose {
  c3d::Vec3 position;
  c3d::Quat rotation;
};

// A joint as the pose stream carries it under `convention`: the position map, and the rotation
// by the DYNAMIC rule, for every joint, the root included.
//
// Not the gaze rule. Session Replay mirrors a rig's rotations the way it mirrors a Dynamic
// Object's, so only the conjugation by the position map lands each joint where the robot's own
// link is, and only a conjugation composes down the tree: converting each parent-local transform
// and composing them gives the converted world pose of every link. The gaze rule's extra fixed
// turn under `gltf_authored` would compound once per level.
//
// Where the position map is a reflection (unity, gltf_authored) a rotation axis is a
// pseudovector: a joint turning by +a about axis k in ROS turns by +a about
// ConvertAngularVelocity(k), which is -M k, on the wire, and the viewer's own mirror brings it
// back to a turn by +a about the axis where the link is drawn.
c3d::pose::JointPose RigJointScenePose(const RosJointPose& ros, PoseConvention convention);

}  // namespace cognitive3d_ros
