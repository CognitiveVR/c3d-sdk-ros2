// The ingest gateway's shape rules for a manifest entry, in the order it checks them, so a
// refusal here reads like the 400 the gateway would have answered.

#include "c3d/pose_manifest.hpp"

#include <cmath>
#include <set>

#include "util.hpp"

namespace c3d {
namespace pose {
namespace {

std::string PropertiesProblem(const Properties& properties, const std::string& path) {
  if (properties.size() > kMaxProperties) {
    return path + ": " + std::to_string(properties.size()) + " properties, at most " +
           std::to_string(kMaxProperties) + " per entry";
  }
  std::set<std::string> keys;
  for (std::size_t p = 0; p < properties.size(); ++p) {
    const std::string at = path + "[" + std::to_string(p) + "]";
    const std::string& key = properties[p].first;
    const PropertyValue& value = properties[p].second;
    if (key.empty() || key.size() > kMaxPropertyKeyBytes) {
      return at + ".key: " + std::to_string(key.size()) + " bytes, must be 1.." +
             std::to_string(kMaxPropertyKeyBytes) + " UTF-8 bytes";
    }
    if (!IsValidUtf8(key)) return at + ".key: not valid UTF-8";
    if (!keys.insert(key).second) return at + ".key: '" + key + "' is listed twice in this entry";
    switch (value.kind()) {
      case PropertyValue::Kind::kString:
        if (value.as_string().size() > kMaxPropertyStringBytes) {
          return at + ".str: " + std::to_string(value.as_string().size()) + " bytes, at most " +
                 std::to_string(kMaxPropertyStringBytes) + " UTF-8 bytes";
        }
        if (!IsValidUtf8(value.as_string())) return at + ".str: not valid UTF-8";
        break;
      case PropertyValue::Kind::kDouble:
        // Stored as a 32-bit float, and NaN or infinity does not survive the JSON read path.
        if (!std::isfinite(value.as_double()) || std::fabs(value.as_double()) > kMaxPropertyNumber) {
          return at + ".num: " + std::to_string(value.as_double()) +
                 " must be finite and within 32-bit float range";
        }
        break;
      case PropertyValue::Kind::kInt:  // every int64 is within float32 range
      case PropertyValue::Kind::kBool:
        break;
    }
  }
  return "";
}

}  // namespace

std::string ManifestProblem(const Manifest& m) {
  if (m.manifest_id.empty()) return "manifest_id: must not be empty";
  // The id is a key on the wire and in storage: one invalid byte and two writers of the same
  // id disagree about it.
  if (!IsValidUtf8(m.manifest_id)) return "manifest_id: not valid UTF-8";
  if (m.name.empty()) return "name: must not be empty";
  if (m.mesh.empty() && !m.file_type.empty()) {
    return "mesh: empty while file_type is set; set both or neither";
  }
  if (m.file_type.empty() && !m.mesh.empty()) {
    return "file_type: empty while mesh is set; set both or neither";
  }
  const std::string& mapping = m.rigging_mapping_type;
  if (!mapping.empty() && mapping != "jointID" && mapping != "meshName") {
    return "rigging_mapping_type: unsupported value '" + mapping +
           "', expected 'jointID', 'meshName' or empty";
  }
  if (!m.root.empty()) {
    if (m.root == m.manifest_id) return "root: names the entry itself";
    if (!m.joints.empty()) {
      return "joints: must be empty, the entry is a follower (root is set) and uses its root's "
             "joints";
    }
  } else if (m.joints.empty()) {
    return "joints: must not be empty unless root is set";
  }
  std::string problem = PropertiesProblem(m.properties, "properties");
  if (!problem.empty()) return problem;
  for (std::size_t j = 0; j < m.joints.size(); ++j) {
    const std::string at = "joints[" + std::to_string(j) + "]";
    const std::uint8_t groups = m.joints[j].groups;
    // Not a gateway rule but the block format's: a joint carries at least one group, and
    // there are three.
    if (groups == 0 || (groups & ~(kPosition | kRotation | kScale)) != 0) {
      return at + ".groups: " + std::to_string(groups) +
             " is not a set of position (1), rotation (2) and scale (4)";
    }
    problem = PropertiesProblem(m.joints[j].properties, at + ".properties");
    if (!problem.empty()) return problem;
  }
  return "";
}

}  // namespace pose
}  // namespace c3d
