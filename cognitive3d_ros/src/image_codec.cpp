#include "cognitive3d_ros/image_codec.hpp"

#include <algorithm>
#include <cctype>

namespace cognitive3d_ros {
namespace {

bool IsStartOfFrame(unsigned char marker) {
  // C0-CF are frame headers EXCEPT C4 (define Huffman table), C8 (reserved) and
  // CC (define arithmetic coding conditioning), which carry a length like any
  // other segment and must be skipped rather than read as dimensions.
  if (marker < 0xC0 || marker > 0xCF) return false;
  return marker != 0xC4 && marker != 0xC8 && marker != 0xCC;
}

bool HasNoPayload(unsigned char marker) {
  // SOI, EOI, TEM and the eight restart markers are two bytes with no length
  // field at all, so reading a length after one walks into the entropy data.
  if (marker == 0xD8 || marker == 0xD9 || marker == 0x01) return true;
  return marker >= 0xD0 && marker <= 0xD7;
}

std::string Lowercase(const std::string& text) {
  std::string out = text;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

}  // namespace

bool JpegDimensions(const std::string& jpeg, std::uint16_t* width, std::uint16_t* height) {
  if (jpeg.size() < 4) return false;
  const unsigned char* data = reinterpret_cast<const unsigned char*>(jpeg.data());
  if (data[0] != 0xFF || data[1] != 0xD8) return false;

  std::size_t i = 2;
  while (i + 1 < jpeg.size()) {
    if (data[i] != 0xFF) return false;
    // Fill bytes: a marker may be preceded by any number of 0xFF.
    std::size_t marker_index = i + 1;
    while (marker_index < jpeg.size() && data[marker_index] == 0xFF) ++marker_index;
    if (marker_index >= jpeg.size()) return false;
    const unsigned char marker = data[marker_index];

    if (HasNoPayload(marker)) {
      i = marker_index + 1;
      continue;
    }
    if (marker_index + 2 >= jpeg.size()) return false;
    const std::size_t length =
        (static_cast<std::size_t>(data[marker_index + 1]) << 8) | data[marker_index + 2];
    if (length < 2 || marker_index + 1 + length > jpeg.size()) return false;

    if (IsStartOfFrame(marker)) {
      // length(2) precision(1) height(2) width(2)
      if (length < 7) return false;
      const std::size_t payload = marker_index + 3;
      if (width != nullptr) {
        *width = static_cast<std::uint16_t>((data[payload + 3] << 8) | data[payload + 4]);
      }
      if (height != nullptr) {
        *height = static_cast<std::uint16_t>((data[payload + 1] << 8) | data[payload + 2]);
      }
      return true;
    }
    i = marker_index + 1 + length;
  }
  return false;
}

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

bool CompressedFormatIsJpeg(const std::string& format) {
  const std::string lowered = Lowercase(format);
  return lowered.find("jpeg") != std::string::npos || lowered.find("jpg") != std::string::npos;
}

}  // namespace cognitive3d_ros
