// The robot as a rig on the pose stream: which tf frames it is made of, the manifest entry that
// describes them, and the conversion every joint takes. Pure functions over frame lists and poses,
// with no ROS in them, so every rule here is testable without a tf buffer.
//
// The profile is `ros2/1`: one rig per robot. Joint 0 is `robot_frame`, carrying its pose in the
// session's reference frame; every other tf frame under it is a joint carrying its transform in
// its tf parent; `meshName` is the frame's name. tf and this node's own configuration decide the
// manifest (tier 1), which is valid by itself. A robot description, when `/robot_description`
// has one, only adds keys to it (tier 2): it never adds a joint, and never removes one.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "c3d/pose.hpp"
#include "c3d/pose_manifest.hpp"
#include "cognitive3d_ros/coordinate_transform.hpp"
#include "cognitive3d_ros/urdf_description.hpp"

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
  // `name`, and `ros.robotName` until a robot description names the robot: the node's `device_id`.
  std::string robot_name;
  // The uploaded per-link model and its file type, both "" until one exists: an empty mesh
  // tells Session Replay there is nothing to draw from this stream, and the robot is drawn from
  // the gaze stream as before.
  std::string mesh;
  std::string file_type;
  // tf frame -> `c3d.kind`, from RigFrameKinds.
  std::map<std::string, std::string> kinds;
};

// Adds keys to a tier 1 manifest, after BuildRigManifest has built it. tf still decides which
// joints exist; the result must still pass c3d::pose::ManifestProblem.
using RigManifestDecoration = std::function<void(c3d::pose::Manifest*)>;

// The tf-only (tier 1) manifest of the rig `frames`, as RigFramesUnder orders them: one entry,
// `riggingMappingType` "meshName", joint j named after frame j and carrying position and
// rotation, never scale. Object properties `c3d.profile` and `ros.robotName`; joint properties
// `ros.parent` on every joint but the root, and `c3d.kind` on a configured sensor's frame.
// `root` is "": the rig has joints of its own.
c3d::pose::Manifest BuildRigManifest(const std::vector<RigFrame>& frames,
                                     const RigManifestOptions& options,
                                     const RigManifestDecoration& decorate = nullptr);

// Tier 2: the keys a robot description adds, matched to the rig by link name.
//
// On the object: `ros.robotName` becomes `<robot name>` (the manifest's `name` stays the node's
// `device_id`), and `ros.urdfSha256`, `c3d.model.format` "urdf" and `c3d.model.formatVersion`
// join it. On every joint but the root whose frame is a link that some URDF joint has as its
// child: `ros.jointName` and `ros.jointType` from that joint, and each of its
// `ros.limit.lower/.upper/.effort/.velocity/.acceleration/.deceleration/.jerk` that is finite,
// so an absent limit means unbounded in every URDF version. A frame the description does not
// name, and the root, keep their tier 1 keys only.
//
// A value the platform could not take -- a name past its string limit, a limit past float32 --
// is left out rather than refused, and should the result still break a shape rule (a description
// whose names are not UTF-8) the manifest stays tier 1: a description never costs the robot its
// rig.
void DecorateRigManifest(const UrdfDescription& description, c3d::pose::Manifest* manifest);

// DecorateRigManifest as a decoration for BuildRigManifest; null for a null description.
RigManifestDecoration UrdfDecoration(std::shared_ptr<const UrdfDescription> description);

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
