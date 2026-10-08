#include "capnp_payloads.hpp"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/io.h>

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
  payload.setPart(envelope.part);
}

std::string Pack(capnp::MessageBuilder& message) {
  kj::VectorOutputStream out;
  capnp::writePackedMessage(out, message);
  const kj::ArrayPtr<const kj::byte> bytes = out.getArray();
  return std::string(reinterpret_cast<const char*>(bytes.begin()), bytes.size());
}

}  // namespace

std::string SerializeCameraPayload(const BinaryEnvelope& envelope,
                                   const std::vector<CameraFrameRecord>& frames) {
  capnp::MallocMessageBuilder message;
  auto payload = message.initRoot<cvr::capnp::CameraPayload>();
  WriteHeader(payload, envelope);

  auto list = payload.initFrames(static_cast<unsigned int>(frames.size()));
  for (unsigned int i = 0; i < frames.size(); ++i) {
    const CameraFrameRecord& record = frames[i];
    auto frame = list[i];
    frame.setCameraId(Str(record.camera_id));
    frame.setTime(record.time_ms);
    frame.setSeq(record.seq);
    frame.setFrameId(Str(record.frame_id));
    WritePose(frame.initPose(), record.pose);
    frame.setFormat(Str(record.format));
    frame.setWidth(record.width);
    frame.setHeight(record.height);
    frame.setData(Bytes(record.data));
    frame.setHfovDegrees(record.hfov_deg);
    frame.setVfovDegrees(record.vfov_deg);
  }
  return Pack(message);
}

std::string SerializeLidarPayload(const BinaryEnvelope& envelope,
                                  const std::vector<LidarScanRecord>& scans) {
  capnp::MallocMessageBuilder message;
  auto payload = message.initRoot<cvr::capnp::LidarPayload>();
  WriteHeader(payload, envelope);

  auto list = payload.initScans(static_cast<unsigned int>(scans.size()));
  for (unsigned int i = 0; i < scans.size(); ++i) {
    const LidarScanRecord& record = scans[i];
    auto scan = list[i];
    scan.setSensorId(Str(record.sensor_id));
    scan.setTime(record.time_ms);
    scan.setSeq(record.seq);
    scan.setFrameId(Str(record.frame_id));
    WritePose(scan.initPose(), record.pose);

    if (record.is_pointcloud) {
      auto cloud = scan.initPointcloud();
      cloud.setPointCount(record.cloud.point_count);
      cloud.setPosition(PointPositionName(record.cloud.position));
      cloud.setColor(PointColorName(record.cloud.color));
      cloud.setIntensity(PointIntensityName(record.cloud.intensity));
      cloud.setVoxelLeafMm(record.cloud.voxel_leaf_mm);
      cloud.setZstdLevel(record.cloud.zstd_level);
      cloud.setData(Bytes(record.cloud.data));
      continue;
    }

    auto laser = scan.initLaserscan();
    laser.setAngleMin(record.scan.angle_min);
    laser.setAngleMax(record.scan.angle_max);
    laser.setAngleIncrement(record.scan.angle_increment);
    laser.setTimeIncrement(record.scan.time_increment);
    laser.setScanTime(record.scan.scan_time);
    laser.setRangeMin(record.scan.range_min);
    laser.setRangeMax(record.scan.range_max);
    auto ranges = laser.initRanges(static_cast<unsigned int>(record.scan.ranges.size()));
    for (unsigned int r = 0; r < record.scan.ranges.size(); ++r) {
      ranges.set(r, record.scan.ranges[r]);
    }
    auto intensities =
        laser.initIntensities(static_cast<unsigned int>(record.scan.intensities.size()));
    for (unsigned int r = 0; r < record.scan.intensities.size(); ++r) {
      intensities.set(r, record.scan.intensities[r]);
    }
  }
  return Pack(message);
}

}  // namespace c3d
