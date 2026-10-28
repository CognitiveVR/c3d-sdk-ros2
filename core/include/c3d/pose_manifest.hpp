// What a rigged object on the pose stream IS: its manifest entry, the description every one of
// its blocks is decoded against. Session::SetPoseManifest sends it; c3d/pose.hpp's samples
// carry the joints it names, in its order.
//
// The fields are the platform's ObjectManifest (core/capnp/pose_payload.capnp), less the two
// a session fills in itself: `j` is a joint's index in `joints`, and `firstSeq` is computed
// from the object's encoder when the send is made (c3d-stream-formatter's docs/pose/spec.md,
// §10.1 rule 1). ManifestProblem holds an entry to the ingest gateway's shape rules, so a
// manifest that would be refused is refused here, before any of its object's blocks depend
// on it, rather than discarded with the part that carried it.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "c3d/pose.hpp"
#include "c3d/types.hpp"

namespace c3d {
namespace pose {

// Gateway shape rules for a property list (spec §9).
constexpr std::size_t kMaxProperties = 64;
constexpr std::size_t kMaxPropertyKeyBytes = 64;
constexpr std::size_t kMaxPropertyStringBytes = 1024;
constexpr double kMaxPropertyNumber = 3.4028235e38;  // float32: what storage keeps

struct ManifestJoint {
  // The node or skin joint this joint drives, matched by name under "meshName" mapping; for a
  // robot, its tf frame.
  std::string mesh_name;
  // The groups every sample of this joint carries (kPosition | kRotation | kScale). A robot's
  // joints never carry scale.
  std::uint8_t groups = kPosition | kRotation;
  // About the joint: `ros.parent`, `ros.jointType`, `c3d.kind`, ... An int travels as a number.
  Properties properties;
};

struct Manifest {
  // The object's id, the key its blocks carry. Stable for the session: the same id later is a
  // re-send of the same object.
  std::string manifest_id;
  std::string name;  // required
  // The uploaded model and its file type, both set or both "" (no model).
  std::string mesh;
  std::string file_type;
  // "jointID", "meshName", or "" (not sent).
  std::string rigging_mapping_type;
  // Index 0 is the root. Empty exactly when `root` is set.
  std::vector<ManifestJoint> joints;
  // About the object: `c3d.profile`, `ros.robotName`, ...
  Properties properties;
  // "" for an object with joints and blocks of its own. Set, the entry is a follower: drawn
  // from the blocks and joints of the object it names, which must already have been sent in
  // this session, with no joints and no samples of its own. An id is a follower or not for
  // the whole session.
  std::string root;
};

// "" for an entry the gateway accepts, otherwise the first rule it breaks, named by field as
// the gateway names it (`joints[3].properties[0].key: ...`).
std::string ManifestProblem(const Manifest& manifest);

}  // namespace pose
}  // namespace c3d
