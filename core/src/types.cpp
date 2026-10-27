#include "c3d/types.hpp"

namespace c3d {

bool PropertyValue::operator==(const PropertyValue& o) const {
  if (kind_ != o.kind_) return false;
  switch (kind_) {
    case Kind::kString: return s_ == o.s_;
    case Kind::kDouble: return d_ == o.d_;
    case Kind::kInt: return i_ == o.i_;
    case Kind::kBool: return b_ == o.b_;
  }
  return false;
}

const char* StreamName(Stream s) {
  switch (s) {
    case Stream::kGaze: return "gaze";
    case Stream::kEvents: return "events";
    case Stream::kSensors: return "sensors";
    case Stream::kDynamics: return "dynamics";
    case Stream::kCamera: return "camera";
    case Stream::kLidar: return "lidar";
    case Stream::kPose: return "pose";
  }
  return "events";
}

bool IsBinaryStream(Stream s) {
  return s == Stream::kCamera || s == Stream::kLidar || s == Stream::kPose;
}

int StreamPriority(Stream s) {
  // Events carry objectives, failures and session boundaries. If the spool has
  // to shed load, poses and scalars go first.
  switch (s) {
    case Stream::kEvents: return 3;
    case Stream::kDynamics: return 2;
    case Stream::kGaze: return 1;
    case Stream::kSensors: return 1;
    // Below everything else, and by a whole tier rather than by a hair. One
    // camera part is two orders of magnitude larger than a gaze part, so a spool
    // under pressure with a camera on would otherwise shed the spatial timeline
    // to keep pictures of it. A lost frame is a lost picture; a lost pose is a
    // hole in the thing every other stream is indexed by.
    case Stream::kCamera: return 0;
    case Stream::kLidar: return 0;
    // A rig's pose is the spatial timeline gaze is, joint by joint: gaze's tier.
    case Stream::kPose: return 1;
  }
  return 1;
}

}  // namespace c3d
