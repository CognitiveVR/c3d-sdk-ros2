#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "c3d/media.hpp"
#include "c3d/types.hpp"

namespace c3d {

// What one buffered camera frame holds until a part is cut. The bytes are the
// encoded image exactly as they will reach the wire; nothing here is re-encoded
// at flush time, because a flush runs on the worker thread and must stay cheap.
struct CameraFrameRecord {
  std::string camera_id;
  std::uint64_t time_ms = 0;
  std::uint32_t seq = 0;
  std::string frame_id;
  Pose pose;
  std::string format;
  std::uint16_t width = 0;
  std::uint16_t height = 0;
  std::string data;
  // Full angles in degrees, already through IsKnownCameraFov: either both are a
  // usable angle or both are 0, which the schema reads as "not recorded".
  float hfov_deg = 0.0f;
  float vfov_deg = 0.0f;
};

// One scan, carrying exactly one of the two arms. `is_pointcloud` picks which,
// because the Cap'n Proto side is a real union and a record that set both or
// neither could not be written at all.
struct LidarScanRecord {
  std::string sensor_id;
  std::uint64_t time_ms = 0;
  std::uint32_t seq = 0;
  std::string frame_id;
  Pose pose;
  bool is_pointcloud = false;
  LaserScanData scan;
  EncodedPointCloud cloud;
};

// The eight header fields both payloads share, at the same ordinals. Written out
// once here and twice on the wire, because Cap'n Proto has no inheritance.
struct BinaryEnvelope {
  std::string user_id;
  std::string session_id;
  std::string scene_id;
  std::uint32_t scene_version = 0;
  std::string format_version;
  std::uint64_t session_start_ms = 0;
  std::string sdk_version;
  std::uint32_t part = 0;
};

// PACKED Cap'n Proto, which is the encoding the gateway reads. The result is an
// opaque byte string as far as the spool and the transport are concerned --
// including the NUL bytes packing produces, which is why the spool cannot keep it
// in a JSON string field.
std::string SerializeCameraPayload(const BinaryEnvelope& envelope,
                                   const std::vector<CameraFrameRecord>& frames);
std::string SerializeLidarPayload(const BinaryEnvelope& envelope,
                                  const std::vector<LidarScanRecord>& scans);

}  // namespace c3d
