#include "cognitive3d_ros/urdf_description.hpp"

#include <tinyxml2.h>

#include <charconv>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <system_error>

#include "c3d/sha256.hpp"

namespace cognitive3d_ros {
namespace {

const std::set<std::string>& JointTypes() {
  static const std::set<std::string> types = {"fixed",     "revolute", "continuous",
                                              "prismatic", "floating", "planar"};
  return types;
}

bool HasPositionLimits(const std::string& type) {
  return type == "revolute" || type == "prismatic";
}

// A limit's text as a number: optional surrounding whitespace and a leading '+', then the
// shortest-round-trip grammar, in the C locale whatever the process's locale says. "inf" reads
// as unbounded; NaN is not a limit.
bool ParseNumber(const char* text, double* out) {
  const char* begin = text;
  const char* end = text + std::strlen(text);
  while (begin < end && std::strchr(" \t\r\n", *begin) != nullptr) ++begin;
  while (end > begin && std::strchr(" \t\r\n", *(end - 1)) != nullptr) --end;
  if (begin < end && *begin == '+') ++begin;
  if (begin == end || *begin == '+' || *begin == ' ') return false;
  double value = 0.0;
  const auto result = std::from_chars(begin, end, value, std::chars_format::general);
  if (result.ec != std::errc() || result.ptr != end || std::isnan(value)) return false;
  *out = value;
  return true;
}

bool Fail(std::string* error, const std::string& what) {
  if (error) *error = what;
  return false;
}

// `<limit>` under one joint, by its version's rules. `where` names the joint for a refusal.
bool ReadLimits(const tinyxml2::XMLElement* limit, const std::string& version, UrdfJoint* joint,
                const std::string& where, std::string* error) {
  const bool v12 = version == "1.2";
  const bool positional = HasPositionLimits(joint->type);
  UrdfLimits& limits = joint->limits;
  // Reads `name` into `*value` when present. False only for a value that is not a number.
  const auto read = [&](const char* name, double* value, bool* present) {
    const char* text = limit->Attribute(name);
    *present = text != nullptr;
    if (text == nullptr) return true;
    if (ParseNumber(text, value)) return true;
    return Fail(error, where + ": <limit " + name + "> is not a number");
  };
  bool present = false;

  if (positional) {
    double lower = 0.0, upper = 0.0;
    bool has_lower = false, has_upper = false;
    if (!read("lower", &lower, &has_lower) || !read("upper", &upper, &has_upper)) return false;
    if (v12 && (!has_lower || !has_upper)) {
      return Fail(error, where + ": a " + joint->type + " joint in URDF 1.2 gives both <limit " +
                             "lower> and <limit upper>");
    }
    // Before 1.2 an unset position limit is 0, which is what every parser of those versions
    // resolves it to, however odd a joint that cannot move is.
    limits.lower = lower;
    limits.upper = upper;
  }
  // Any other type has no position limits, whatever its <limit> says: a continuous joint turns
  // without end, and a 0/0 written on one is not a joint that cannot move.

  double effort = UrdfLimits::kInf, velocity = UrdfLimits::kInf;
  if (!read("effort", &effort, &present)) return false;
  if (!present && !v12) return Fail(error, where + ": <limit> has no effort, which URDF " +
                                               version + " requires");
  if (!read("velocity", &velocity, &present)) return false;
  if (!present && !v12) return Fail(error, where + ": <limit> has no velocity, which URDF " +
                                               version + " requires");
  limits.effort = effort;
  limits.velocity = velocity;

  if (v12) {
    double acceleration = UrdfLimits::kInf, deceleration = UrdfLimits::kInf,
           jerk = UrdfLimits::kInf;
    bool has_deceleration = false;
    if (!read("acceleration", &acceleration, &present) ||
        !read("deceleration", &deceleration, &has_deceleration) ||
        !read("jerk", &jerk, &present)) {
      return false;
    }
    limits.acceleration = acceleration;
    limits.deceleration = has_deceleration ? deceleration : acceleration;
    limits.jerk = jerk;
  }
  // Before 1.2 there is no acceleration, deceleration or jerk, and an attribute by that name is
  // not part of the format: it stays unbounded.
  return true;
}

}  // namespace

const UrdfJoint* UrdfDescription::JointWithChild(const std::string& link) const {
  for (const UrdfJoint& joint : joints) {
    if (joint.child == link) return &joint;
  }
  return nullptr;
}

bool ParseUrdfDescription(const std::string& xml, UrdfDescription* out, std::string* error) {
  *out = UrdfDescription();
  tinyxml2::XMLDocument document(/*processEntities=*/true, tinyxml2::PRESERVE_WHITESPACE);
  if (document.Parse(xml.data(), xml.size()) != tinyxml2::XML_SUCCESS) {
    // The error's name and line only: tinyxml2's own message can quote the document.
    return Fail(error, std::string("not well-formed XML (") + document.ErrorName() + " at line " +
                           std::to_string(document.ErrorLineNum()) + ")");
  }
  const tinyxml2::XMLElement* robot = document.RootElement();
  if (robot == nullptr || std::strcmp(robot->Name(), "robot") != 0) {
    return Fail(error, "the root element is not <robot>");
  }
  const char* name = robot->Attribute("name");
  if (name == nullptr || *name == '\0') return Fail(error, "<robot> has no name");
  const char* version = robot->Attribute("version");
  const std::string resolved_version = version == nullptr ? "1.0" : version;
  if (resolved_version != "1.0" && resolved_version != "1.1" && resolved_version != "1.2") {
    return Fail(error, "<robot version> is not 1.0, 1.1 or 1.2");
  }

  UrdfDescription parsed;
  parsed.robot_name = name;
  parsed.format_version = resolved_version;
  std::set<std::string> links;
  std::size_t link_index = 0;
  for (const tinyxml2::XMLElement* link = robot->FirstChildElement("link"); link != nullptr;
       link = link->NextSiblingElement("link"), ++link_index) {
    const char* link_name = link->Attribute("name");
    if (link_name == nullptr || *link_name == '\0') {
      return Fail(error, "<link> " + std::to_string(link_index) + " has no name");
    }
    if (!links.insert(link_name).second) {
      return Fail(error, std::string("link '") + link_name + "' is declared twice");
    }
    parsed.links.push_back(link_name);
  }

  std::set<std::string> joint_names;
  std::map<std::string, std::string> parent_joint;  // child link -> its joint
  std::size_t joint_index = 0;
  for (const tinyxml2::XMLElement* element = robot->FirstChildElement("joint");
       element != nullptr; element = element->NextSiblingElement("joint"), ++joint_index) {
    UrdfJoint joint;
    const char* joint_name = element->Attribute("name");
    if (joint_name == nullptr || *joint_name == '\0') {
      return Fail(error, "<joint> " + std::to_string(joint_index) + " has no name");
    }
    joint.name = joint_name;
    const std::string where = "joint '" + joint.name + "'";
    if (!joint_names.insert(joint.name).second) return Fail(error, where + " is declared twice");
    const char* type = element->Attribute("type");
    if (type == nullptr || JointTypes().count(type) == 0) {
      return Fail(error, where + " is not of a URDF joint type (fixed, revolute, continuous, " +
                             "prismatic, floating, planar)");
    }
    joint.type = type;
    const tinyxml2::XMLElement* parent = element->FirstChildElement("parent");
    const tinyxml2::XMLElement* child = element->FirstChildElement("child");
    const char* parent_link = parent == nullptr ? nullptr : parent->Attribute("link");
    const char* child_link = child == nullptr ? nullptr : child->Attribute("link");
    if (parent_link == nullptr || child_link == nullptr) {
      return Fail(error, where + " has no <parent link> or no <child link>");
    }
    joint.parent = parent_link;
    joint.child = child_link;
    if (links.count(joint.parent) == 0 || links.count(joint.child) == 0) {
      return Fail(error, where + " names a link the description does not declare");
    }
    const auto [claimed, inserted] = parent_joint.emplace(joint.child, joint.name);
    if (!inserted) {
      return Fail(error, "link '" + joint.child + "' is the child of both joint '" +
                             claimed->second + "' and " + where);
    }
    const tinyxml2::XMLElement* limit = element->FirstChildElement("limit");
    if (limit != nullptr) {
      joint.has_limit = true;
      if (!ReadLimits(limit, parsed.format_version, &joint, where, error)) return false;
    } else if (HasPositionLimits(joint.type)) {
      return Fail(error, where + " is " + joint.type + " and has no <limit>");
    }
    joint.mimic = element->FirstChildElement("mimic") != nullptr;
    parsed.joints.push_back(std::move(joint));
  }

  parsed.sha256 = c3d::Sha256Hex(xml);
  *out = std::move(parsed);
  return true;
}

std::string DescribeUrdf(const UrdfDescription& description) {
  return "robot '" + description.robot_name + "', URDF " + description.format_version + ", " +
         std::to_string(description.links.size()) + " link(s), " +
         std::to_string(description.joints.size()) + " joint(s), sha256 " +
         description.sha256.substr(0, 12);
}

}  // namespace cognitive3d_ros
