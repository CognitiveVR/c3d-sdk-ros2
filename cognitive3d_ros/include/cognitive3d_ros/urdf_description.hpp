// A robot description (URDF) as the pose stream reads it: the robot's name, its format version,
// its links, and each joint's name, type, parent, child and limits. Nothing else.
//
// The description arrives as the expanded XML `robot_state_publisher` republishes on
// `/robot_description`. That XML can be a customer's CAD and kinematics, and its `<ros2_control>`
// and `<transmission>` blocks can carry addresses, serial ports and hardware parameters. So the
// parser reads a fixed whitelist -- `<robot name version>`, each `<link name>`, and each
// `<joint name type>` with its `<parent link>`, `<child link>`, `<limit>` and whether it has a
// `<mimic>` -- from the direct children of `<robot>` only, and keeps nothing else. The raw XML
// is hashed and dropped; it never leaves this file, and no refusal quotes it.
//
// No ROS in here: a pure function over a string, testable without a graph.
#pragma once

#include <limits>
#include <string>
#include <vector>

namespace cognitive3d_ros {

// One joint's limits as its format version resolves them. A limit the version leaves unbounded is
// infinite: position limits on every type but revolute and prismatic; in URDF 1.2, any limit not
// given (deceleration not given is acceleration's). The manifest omits every infinite limit, so
// "absent" means unbounded in every version.
struct UrdfLimits {
  static constexpr double kInf = std::numeric_limits<double>::infinity();
  double lower = -kInf;
  double upper = kInf;
  double effort = kInf;
  double velocity = kInf;
  double acceleration = kInf;
  double deceleration = kInf;
  double jerk = kInf;
};

struct UrdfJoint {
  std::string name;
  // fixed, revolute, continuous, prismatic, floating or planar.
  std::string type;
  std::string parent;
  std::string child;
  // Whether the joint has a `<limit>`. Without one, every limit above is infinite.
  bool has_limit = false;
  UrdfLimits limits;
  // Whether the joint mimics another. Noted only: its value is in tf like any joint's.
  bool mimic = false;
};

struct UrdfDescription {
  std::string robot_name;
  // "1.0", "1.1" or "1.2": `<robot version>`, "1.0" when the attribute is absent.
  std::string format_version;
  // Lowercase hex SHA-256 of the XML bytes exactly as received.
  std::string sha256;
  // In document order.
  std::vector<std::string> links;
  std::vector<UrdfJoint> joints;

  // The joint whose child is `link`, or null: the root link and a name the description does not
  // have are no joint's child.
  const UrdfJoint* JointWithChild(const std::string& link) const;
};

// Parses `xml` into `out`. False, with `error` set, for a description the URDF format itself
// refuses -- XML that does not parse, a root that is not `<robot>`, a robot with no name, a
// version other than 1.0, 1.1 and 1.2, a joint of an unknown type or naming a link that is not
// there, a link that is two joints' child, a limit that is not a number, a revolute or prismatic
// joint with no `<limit>`, or one its version requires a value of that is missing -- since
// `robot_state_publisher` would refuse it too, and decorating from half a description would
// say something no URDF parser says. `error` names an element by index and a joint or link by
// its name, and never quotes any other part of the XML.
bool ParseUrdfDescription(const std::string& xml, UrdfDescription* out, std::string* error);

// What the node logs for a description it read: the robot's name, the version, the counts and the
// head of the hash. Nothing else from the XML.
std::string DescribeUrdf(const UrdfDescription& description);

}  // namespace cognitive3d_ros
