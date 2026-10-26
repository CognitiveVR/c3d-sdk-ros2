#pragma once

#include <cstdint>
#include <string>

#include "c3d/scan_block.hpp"
#include "c3d/sensor_blocks.hpp"

namespace c3d {

// The seven header fields both payloads share, at the same ordinals. Written out
// once here and twice on the wire, because Cap'n Proto has no inheritance.
struct BinaryEnvelope {
  std::string user_id;
  std::string session_id;
  std::string scene_id;
  std::uint32_t scene_version = 0;
  std::string format_version;
  std::uint64_t session_start_ms = 0;
  std::string sdk_version;
};

// PACKED Cap'n Proto, which is the encoding the gateway reads. The result is an
// opaque byte string as far as the spool and the transport are concerned --
// including the NUL bytes packing produces, which is why the spool cannot keep it
// in a JSON string field.
//
// One payload per closed unit, as an assembler in c3d/sensor_blocks.hpp closed it:
// a record's per-entry vectors run in parallel, one element per entry.
std::string SerializeCameraChunk(const BinaryEnvelope& envelope, const CameraChunkRecord& chunk);
// `encoded` is scanblock::Encode of `block.scans`. The geometry is written once, from
// the first scan, which the assembler made every scan's; the timing goes per entry.
std::string SerializeScanBlock(const BinaryEnvelope& envelope, const ScanBlockRecord& block,
                               const scanblock::EncodedScanBlock& encoded);
std::string SerializeCloudObject(const BinaryEnvelope& envelope, const CloudObjectRecord& object);

}  // namespace c3d
