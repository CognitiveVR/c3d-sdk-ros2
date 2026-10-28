#include "cognitive3d_ros/pose_rig.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace cognitive3d_ros {

std::vector<RigFrame> RigFramesUnder(const std::vector<RigFrame>& tree,
                                     const std::string& robot_frame,
                                     const std::set<std::string>& excluded) {
  std::map<std::string, std::vector<std::string>> children;
  bool known = false;
  for (const RigFrame& frame : tree) {
    if (frame.name == robot_frame) known = true;
    if (frame.name.empty() || frame.parent.empty() || frame.name == frame.parent) continue;
    if (frame.parent == robot_frame) known = true;
    children[frame.parent].push_back(frame.name);
  }
  std::vector<RigFrame> rig;
  if (robot_frame.empty() || !known) return rig;
  for (auto& [parent, names] : children) {
    std::sort(names.begin(), names.end());
    names.erase(std::unique(names.begin(), names.end()), names.end());
  }

  // Depth first from the root. `visited` guards a list that is not a tree: tf's is, but this
  // takes any list, and a cycle in one would otherwise never end.
  std::set<std::string> visited{robot_frame};
  rig.push_back(RigFrame{robot_frame, ""});
  std::vector<RigFrame> stack;
  const auto push_children = [&](const std::string& parent) {
    const auto found = children.find(parent);
    if (found == children.end()) return;
    for (auto it = found->second.rbegin(); it != found->second.rend(); ++it) {
      if (excluded.count(*it) != 0 || visited.count(*it) != 0) continue;
      stack.push_back(RigFrame{*it, parent});
    }
  };
  push_children(robot_frame);
  while (!stack.empty()) {
    RigFrame frame = stack.back();
    stack.pop_back();
    if (!visited.insert(frame.name).second) continue;
    rig.push_back(frame);
    push_children(frame.name);
  }
  return rig;
}

std::map<std::string, std::string> RigFrameKinds(
    const std::vector<std::pair<std::string, std::string>>& frames) {
  std::map<std::string, std::string> kinds;
  for (const auto& [frame, kind] : frames) {
    if (frame.empty() || kind.empty()) continue;
    kinds.emplace(frame, kind);
  }
  return kinds;
}

std::string DefaultPoseManifestId(const std::string& device_id) {
  std::uint64_t hash = 0xcbf29ce484222325ull;
  for (const char c : device_id) {
    hash ^= static_cast<unsigned char>(c);
    hash *= 0x100000001b3ull;
  }
  char hex[17];
  std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(hash));
  return std::string("ros2-") + hex;
}

c3d::pose::Manifest BuildRigManifest(const std::vector<RigFrame>& frames,
                                     const RigManifestOptions& options,
                                     const RigManifestDecoration& decorate) {
  c3d::pose::Manifest manifest;
  manifest.manifest_id = options.manifest_id;
  manifest.name = options.robot_name;
  manifest.mesh = options.mesh;
  manifest.file_type = options.file_type;
  manifest.rigging_mapping_type = "meshName";
  manifest.properties.emplace_back("c3d.profile", std::string(kRigProfile));
  manifest.properties.emplace_back("ros.robotName", options.robot_name);
  for (std::size_t j = 0; j < frames.size(); ++j) {
    c3d::pose::ManifestJoint joint;
    joint.mesh_name = frames[j].name;
    // tf has no scale, so a robot's joint never carries it.
    joint.groups = c3d::pose::kPosition | c3d::pose::kRotation;
    // The root's pose is in the reference frame, which is not a joint of the rig: naming it
    // here would describe a parent the rig does not have.
    if (j > 0) joint.properties.emplace_back("ros.parent", frames[j].parent);
    const auto kind = options.kinds.find(frames[j].name);
    if (kind != options.kinds.end()) joint.properties.emplace_back("c3d.kind", kind->second);
    manifest.joints.push_back(std::move(joint));
  }
  if (decorate) decorate(&manifest);
  return manifest;
}

void DecorateRigManifest(const UrdfDescription& description, c3d::pose::Manifest* manifest) {
  const c3d::pose::Manifest tier1 = *manifest;
  const auto fits = [](const std::string& text) {
    return !text.empty() && text.size() <= c3d::pose::kMaxPropertyStringBytes;
  };
  if (fits(description.robot_name)) {
    for (auto& [key, value] : manifest->properties) {
      if (key == "ros.robotName") value = description.robot_name;
    }
  }
  manifest->properties.emplace_back("ros.urdfSha256", description.sha256);
  manifest->properties.emplace_back("c3d.model.format", std::string("urdf"));
  manifest->properties.emplace_back("c3d.model.formatVersion", description.format_version);

  // The root's pose is in the reference frame, not in a parent link of the rig, so no URDF joint
  // describes how it moves there.
  for (std::size_t j = 1; j < manifest->joints.size(); ++j) {
    c3d::pose::ManifestJoint& rig_joint = manifest->joints[j];
    const UrdfJoint* joint = description.JointWithChild(rig_joint.mesh_name);
    if (joint == nullptr) continue;
    if (fits(joint->name)) rig_joint.properties.emplace_back("ros.jointName", joint->name);
    rig_joint.properties.emplace_back("ros.jointType", joint->type);
    if (!joint->has_limit) continue;
    const std::pair<const char*, double> limits[] = {
        {"ros.limit.lower", joint->limits.lower},
        {"ros.limit.upper", joint->limits.upper},
        {"ros.limit.effort", joint->limits.effort},
        {"ros.limit.velocity", joint->limits.velocity},
        {"ros.limit.acceleration", joint->limits.acceleration},
        {"ros.limit.deceleration", joint->limits.deceleration},
        {"ros.limit.jerk", joint->limits.jerk},
    };
    for (const auto& [key, value] : limits) {
      // Infinite is unbounded, which absence says. Past float32 the platform could not store it,
      // and a limit that large bounds nothing a robot does.
      if (!std::isfinite(value) || std::fabs(value) > c3d::pose::kMaxPropertyNumber) continue;
      rig_joint.properties.emplace_back(key, value);
    }
  }
  if (!c3d::pose::ManifestProblem(*manifest).empty()) *manifest = tier1;
}

RigManifestDecoration UrdfDecoration(std::shared_ptr<const UrdfDescription> description) {
  if (!description) return nullptr;
  return [description](c3d::pose::Manifest* manifest) {
    DecorateRigManifest(*description, manifest);
  };
}

c3d::pose::JointPose RigJointScenePose(const RosJointPose& ros, PoseConvention convention) {
  c3d::pose::JointPose joint;
  joint.position = ConvertPosition(ros.position, convention);
  joint.rotation = ConvertDynamicRotation(ros.rotation, convention);
  return joint;
}

}  // namespace cognitive3d_ros
