#include "capnp_payloads.hpp"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/io.h>

#include <cstring>

#include "camera_payload.capnp.h"
#include "lidar_payload.capnp.h"

namespace c3d {
namespace {

// The ONE translation unit that sees a generated Cap'n Proto header. Everything
// above it deals in the plain structs in capnp_payloads.hpp, so the schemas'
// shape stays a private detail of this file and c3d/session.hpp stays a header a
// consumer can include without capnp on the include path.

kj::StringPtr Str(const std::string& s) { return kj::StringPtr(s.c_str(), s.size()); }

capnp::Data::Reader Bytes(const std::string& s) {
  return capnp::Data::Reader(reinterpret_cast<const kj::byte*>(s.data()), s.size());
}

void WritePose(cvr::capnp::Pose::Builder out, const Pose& pose) {
  out.setPx(static_cast<float>(pose.position.x));
  out.setPy(static_cast<float>(pose.position.y));
  out.setPz(static_cast<float>(pose.position.z));
  out.setQx(static_cast<float>(pose.rotation.x));
  out.setQy(static_cast<float>(pose.rotation.y));
  out.setQz(static_cast<float>(pose.rotation.z));
  // Written even when it is 1. `qw` DEFAULTS to 1 in the schema, and Cap'n Proto
  // stores a field equal to its default as nothing at all -- which is correct and
  // is still not something to rely on here, because the default exists to make an
  // UNWRITTEN pose readable, not to excuse a writer from stating the rotation it
  // measured. All seven components go out on every pose.
  out.setQw(static_cast<float>(pose.rotation.w));
}

template <typename Builder>
void WriteHeader(Builder payload, const BinaryEnvelope& envelope) {
  payload.setUserId(Str(envelope.user_id));
  payload.setSessionId(Str(envelope.session_id));
  payload.setSceneId(Str(envelope.scene_id));
  payload.setSceneVersion(envelope.scene_version);
  payload.setFormatVersion(Str(envelope.format_version));
  payload.setSessionStartMs(envelope.session_start_ms);
  payload.setSdkVersion(Str(envelope.sdk_version));
}

void WriteLidarHeader(cvr::capnp::LidarPayload::Builder payload, const BinaryEnvelope& envelope,
                      const std::string& sensor_id, std::uint32_t seq, std::uint64_t start_ms,
                      std::uint64_t end_ms, const std::string& frame_id) {
  WriteHeader(payload, envelope);
  payload.setSensorId(Str(sensor_id));
  payload.setSeq(seq);
  payload.setStartMs(start_ms);
  payload.setEndMs(end_ms);
  payload.setFrameId(Str(frame_id));
}

template <typename Builder>
void WriteEntry(Builder out, const SensorEntry& entry) {
  out.setTime(entry.time_ms);
  out.setSeq(entry.seq);
  WritePose(out.initPose(), entry.pose);
}

cvr::capnp::RangeUnit RangeUnitOf(scanblock::RangeUnit unit) {
  return unit == scanblock::RangeUnit::kCentimetre ? cvr::capnp::RangeUnit::CENTIMETRE
                                                   : cvr::capnp::RangeUnit::MILLIMETRE;
}

cvr::capnp::CloudEncoding CloudEncodingOf(PointCloudEncoding encoding) {
  return encoding == PointCloudEncoding::kPlanarDeltaShuffleZstd
             ? cvr::capnp::CloudEncoding::PLANAR_DELTA_SHUFFLE_ZSTD
             : cvr::capnp::CloudEncoding::PLANAR_ZSTD;
}

std::string Pack(capnp::MessageBuilder& message) {
  kj::VectorOutputStream out;
  capnp::writePackedMessage(out, message);
  const kj::ArrayPtr<const kj::byte> bytes = out.getArray();
  return std::string(reinterpret_cast<const char*>(bytes.begin()), bytes.size());
}

}  // namespace

std::string CameraIdOfPayload(const std::string& packed) {
  try {
    kj::ArrayInputStream in(
        kj::ArrayPtr<const kj::byte>(reinterpret_cast<const kj::byte*>(packed.data()),
                                     packed.size()));
    capnp::PackedMessageReader reader(in);
    return reader.getRoot<cvr::capnp::CameraPayload>().getCameraId().cStr();
  } catch (const kj::Exception&) {
    return std::string();
  }
}

std::string SerializeCameraChunk(const BinaryEnvelope& envelope, const CameraChunkRecord& chunk) {
  capnp::MallocMessageBuilder message;
  auto payload = message.initRoot<cvr::capnp::CameraPayload>();
  WriteHeader(payload, envelope);

  const VideoChunkContext& context = chunk.context;
  payload.setCameraId(Str(chunk.camera_id));
  payload.setSeq(chunk.chunk_seq);
  payload.setStartMs(chunk.start_ms);
  payload.setEndMs(chunk.end_ms);
  payload.setCodec(Str(context.codec));
  payload.setCodecParams(Bytes(context.codec_params));
  payload.setWidth(context.width);
  payload.setHeight(context.height);
  payload.setNominalFps(context.nominal_fps);
  payload.setFrameId(Str(context.frame_id));
  payload.setHfovDegrees(context.hfov_deg);
  payload.setVfovDegrees(context.vfov_deg);
  payload.setEncoderSettings(Str(context.encoder_settings));

  auto frames = payload.initFrames(static_cast<unsigned int>(chunk.frames.size()));
  for (unsigned int i = 0; i < chunk.frames.size(); ++i) {
    const FrameMeta& meta = chunk.frames[i];
    auto frame = frames[i];
    frame.setTime(meta.time_ms);
    frame.setSeq(meta.seq);
    WritePose(frame.initPose(), meta.pose);
    frame.setByteOffset(meta.byte_offset);
    frame.setByteLength(meta.byte_length);
    frame.setKeyframe(meta.keyframe);
  }
  payload.setData(Bytes(chunk.data));
  return Pack(message);
}

std::string SerializeScanBlock(const BinaryEnvelope& envelope, const ScanBlockRecord& block,
                               const scanblock::EncodedScanBlock& encoded) {
  capnp::MallocMessageBuilder message;
  auto payload = message.initRoot<cvr::capnp::LidarPayload>();
  WriteLidarHeader(payload, envelope, block.sensor_id, block.block_seq, block.start_ms,
                   block.end_ms, block.frame_id);

  auto scans = payload.initScans();
  if (!block.scans.empty()) {
    const LaserScanData& first = block.scans.front();
    scans.setAngleMin(first.angle_min);
    scans.setAngleMax(first.angle_max);
    scans.setAngleIncrement(first.angle_increment);
    scans.setRangeMin(first.range_min);
    scans.setRangeMax(first.range_max);
    // At most scanblock::kMaxRaysPerScan: the assembler refuses a longer scan.
    scans.setRays(static_cast<std::uint16_t>(first.ranges.size()));
  }
  scans.setRangeUnit(RangeUnitOf(encoded.unit));
  scans.setEncoding(cvr::capnp::ScanEncoding::DELTA_SHUFFLE_ZSTD);

  auto entries = scans.initEntries(static_cast<unsigned int>(block.entries.size()));
  for (unsigned int i = 0; i < block.entries.size(); ++i) {
    auto entry = entries[i];
    WriteEntry(entry, block.entries[i]);
    entry.setTimeIncrement(block.scans[i].time_increment);
    entry.setScanTime(block.scans[i].scan_time);
  }
  scans.setRanges(Bytes(encoded.ranges));
  scans.setIntensities(Bytes(encoded.intensities));
  return Pack(message);
}

std::string SerializeCloudObject(const BinaryEnvelope& envelope, const CloudObjectRecord& object) {
  capnp::MallocMessageBuilder message;
  auto payload = message.initRoot<cvr::capnp::LidarPayload>();
  WriteLidarHeader(payload, envelope, object.sensor_id, object.object_seq, object.start_ms,
                   object.end_ms, object.frame_id);

  auto clouds = payload.initClouds();
  clouds.setPosition(PointPositionName(object.position));
  clouds.setColor(PointColorName(object.color));
  clouds.setIntensity(PointIntensityName(object.intensity));
  clouds.setVoxelLeafMm(object.voxel_leaf_mm);
  clouds.setEncoding(CloudEncodingOf(object.encoding));
  clouds.setZstdLevel(object.zstd_level);

  std::size_t total = 0;
  for (const std::string& block : object.blocks) total += block.size();
  auto entries = clouds.initEntries(static_cast<unsigned int>(object.entries.size()));
  auto data = clouds.initData(static_cast<unsigned int>(total));
  // Each cloud's block is found again from its own offset and length: a reader decodes
  // one cloud without touching the others.
  std::size_t offset = 0;
  for (unsigned int i = 0; i < object.entries.size(); ++i) {
    const std::string& block = object.blocks[i];
    auto entry = entries[i];
    WriteEntry(entry, object.entries[i]);
    entry.setPointCount(object.point_counts[i]);
    entry.setByteOffset(static_cast<std::uint32_t>(offset));
    entry.setByteLength(static_cast<std::uint32_t>(block.size()));
    if (!block.empty()) std::memcpy(data.begin() + offset, block.data(), block.size());
    offset += block.size();
  }
  return Pack(message);
}

}  // namespace c3d
