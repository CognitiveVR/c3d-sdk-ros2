// Cognitive3D ingest core. ROS-free by design: this directory must never
// include a ROS header. tools/build_and_test.sh builds it with no ROS in the
// environment.
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace c3d {

struct Vec3 {
  double x{0.0}, y{0.0}, z{0.0};
};

struct Quat {
  double x{0.0}, y{0.0}, z{0.0}, w{1.0};
};

// A property value. Deliberately not std::variant-with-null: a null property can
// drop its key and emit a bare `null`, which is invalid JSON. With no null
// representation, that output cannot be produced.
class PropertyValue {
 public:
  enum class Kind { kString, kDouble, kInt, kBool };

  PropertyValue() : kind_(Kind::kInt), i_(0) {}
  PropertyValue(const char* v) : kind_(Kind::kString), s_(v) {}
  PropertyValue(std::string v) : kind_(Kind::kString), s_(std::move(v)) {}
  PropertyValue(double v) : kind_(Kind::kDouble), d_(v) {}
  PropertyValue(float v) : kind_(Kind::kDouble), d_(static_cast<double>(v)) {}
  PropertyValue(int v) : kind_(Kind::kInt), i_(v) {}
  PropertyValue(std::int64_t v) : kind_(Kind::kInt), i_(v) {}
  PropertyValue(bool v) : kind_(Kind::kBool), b_(v) {}

  Kind kind() const { return kind_; }
  const std::string& as_string() const { return s_; }
  double as_double() const { return d_; }
  std::int64_t as_int() const { return i_; }
  bool as_bool() const { return b_; }

  bool operator==(const PropertyValue& o) const;

 private:
  Kind kind_;
  std::string s_;
  double d_{0.0};
  std::int64_t i_{0};
  bool b_{false};
};

// Ordered rather than a map, so serialization is deterministic and diffable.
using Properties = std::vector<std::pair<std::string, PropertyValue>>;

// The four JSON streams, then the three binary ones. kCamera, kLidar and kPose share
// the spool, the pump and the retry rules with the rest and differ in three places
// only: a Cap'n Proto body instead of JSON, `application/octet-stream` instead of
// `application/json`, and a session-keyed URL that carries no scene id. kPose's parts
// are also cut differently, by its own upload queue (Session's pose stream).
enum class Stream { kGaze, kEvents, kSensors, kDynamics, kCamera, kLidar, kPose };

const char* StreamName(Stream s);

// True for the three streams whose body is packed Cap'n Proto rather than JSON.
bool IsBinaryStream(Stream s);

// Eviction priority when the spool hits its cap. Lower tier is discarded first.
// Events carry objectives, failures and session boundaries, so they outlive
// pose and scalar samples. A gaze part carrying session properties is the one
// exception, evicted with the events: see Chunk::carries_properties.
int StreamPriority(Stream s);

}  // namespace c3d
