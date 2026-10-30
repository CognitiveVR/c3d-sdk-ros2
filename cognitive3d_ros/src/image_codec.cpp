#include "cognitive3d_ros/image_codec.hpp"

namespace cognitive3d_ros {

bool AcceptedImageEncoding(const std::string& encoding) {
  return encoding == "rgb8" || encoding == "bgr8" || encoding == "rgba8" ||
         encoding == "bgra8" || encoding == "mono8";
}

bool IsDepthEncoding(const std::string& encoding) {
  // The two REP-118 depth encodings, plus the mono16 a stereo driver publishes
  // disparity in. `mono8` is deliberately NOT here: an 8-bit single channel is a
  // greyscale picture and is accepted above.
  return encoding == "32FC1" || encoding == "16UC1" || encoding == "mono16";
}

}  // namespace cognitive3d_ros
