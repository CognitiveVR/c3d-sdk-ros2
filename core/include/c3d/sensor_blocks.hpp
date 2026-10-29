// Per-sensor assemblers. Each groups one sensor's records into the unit that becomes one uploaded
// object: a video chunk, a laser-scan block, a point-cloud object.
//
// No clock inside: every time arrives as an argument, so the caller owns the clock and a test
// drives an assembler with literals.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "c3d/media.hpp"

namespace c3d {

// The encoder's keyframe interval. A chunk is cut at every keyframe, so this is also a chunk's
// length; VideoChunkAssembler itself never reads it.
constexpr double kVideoChunkSeconds = 3.0;
constexpr std::uint64_t kScanBlockMilliseconds = 3000;
constexpr std::uint64_t kCloudObjectMaxBytes = 2ull * 1024 * 1024;
constexpr std::uint64_t kCloudObjectMilliseconds = 5000;
// The data gateway answers 413 to a request body larger than this, and an object it refuses is
// deleted, so no object is built that could reach it.
constexpr std::uint64_t kMaxObjectBytes = 10ull * 1024 * 1024;
// The most an open video chunk's data holds; the rest of kMaxObjectBytes is the envelope, a
// FrameMeta per frame and what packing adds.
constexpr std::uint64_t kMaxVideoChunkBytes = 8ull * 1024 * 1024;
// A chunk's first unit is never refused for size: its re-encode would be as large, and refused
// again. One up to this is kept as a chunk of its own; a larger one is dropped.
constexpr std::uint64_t kMaxVideoFirstUnitBytes = 9ull * 1024 * 1024;
// The most an open scan block's codes hold before it is closed, whatever the stamps do: a lidar
// whose stamp stops advancing would otherwise grow one without bound.
constexpr std::uint64_t kMaxScanBlockCodeBytes = 32ull * 1024 * 1024;
// The idle closes have no constant here: the caller passes them in from its configuration.

struct SensorEntry {
  std::uint64_t time_ms = 0;
  std::uint32_t seq = 0;
  Pose pose;
};

// `start_ms` is the first entry's time and `end_ms` the largest; entries keep append order; each
// assembler numbers the units it closes from 0.

struct FrameMeta {
  std::uint64_t time_ms = 0;
  std::uint32_t seq = 0;
  Pose pose;
  std::uint32_t byte_offset = 0, byte_length = 0;  // the access unit's place in the chunk's `data`
  bool keyframe = false;
};

struct VideoChunkContext {
  std::string frame_id;
  std::uint16_t width = 0, height = 0;
  float nominal_fps = 0.0f;
  float hfov_deg = 0.0f, vfov_deg = 0.0f;  // 0 by 0 is not recorded, as in CameraFov
  std::string codec = "h264";
  std::string codec_params;      // the avcC record
  std::string encoder_settings;  // for example "x264 ultrafast crf=28 threads=1 gop=3s"
};

struct CameraChunkRecord {
  std::string camera_id;
  std::uint32_t chunk_seq = 0;
  std::uint64_t start_ms = 0, end_ms = 0;
  VideoChunkContext context;  // the first frame's
  std::vector<FrameMeta> frames;
  std::string data;
};

// What became of one access unit handed to VideoChunkAssembler::Append.
enum class VideoUnitFate {
  kAppended,
  kDropped,  // counted in dropped_before_keyframe()
  // Not kept and not counted: the caller is to re-encode it, and every unit after it, from a new
  // keyframe.
  kRefused,
};

class VideoChunkAssembler {
 public:
  explicit VideoChunkAssembler(std::string camera_id);
  // A chunk starts at a keyframe, which is what makes it decodable on its own. A keyframe closes
  // the open chunk (returned) and opens the next; one larger than kMaxVideoFirstUnitBytes closes
  // the open chunk and is dropped. A non-keyframe with no open chunk, or one whose width, height,
  // frame_id or codec_params differ from the open chunk's, is dropped and counted.
  //
  // A non-keyframe that would take the open chunk's data past kMaxVideoChunkBytes closes it
  // (returned). With `fate`, the unit is then refused, and so is every non-keyframe after it until
  // a keyframe. Without, it is dropped and counted, as is every non-keyframe after it until a
  // keyframe.
  std::vector<CameraChunkRecord> Append(const SensorEntry& entry, const VideoChunkContext& context,
                                        std::string access_unit, bool keyframe,
                                        VideoUnitFate* fate = nullptr);
  // 0 or 1; a closed chunk ends the GOP early and is still decodable.
  std::vector<CameraChunkRecord> Close();
  std::uint64_t dropped_before_keyframe() const;
  std::size_t buffered_frames() const;
  std::uint64_t buffered_bytes() const;

 private:
  std::string camera_id_;
  std::uint32_t next_seq_ = 0;
  std::optional<CameraChunkRecord> open_;
  std::uint64_t dropped_before_keyframe_ = 0;
  bool refusing_ = false;
};

struct ScanBlockRecord {
  std::string sensor_id, frame_id;
  std::uint32_t block_seq = 0;
  std::uint64_t start_ms = 0, end_ms = 0;
  bool intensities = false;
  std::vector<SensorEntry> entries;
  std::vector<LaserScanData> scans;
};

class ScanBlockAssembler {
 public:
  explicit ScanBlockAssembler(std::string sensor_id);
  // A block's `intensities` is decided by its first scan: present when that scan carries exactly
  // `rays` of them. Closes the open block first when entry.time_ms >= start_ms +
  // kScanBlockMilliseconds, when scanblock::GeometryOf(scan) differs from the block's (ray count
  // included; time_increment and scan_time never: they travel per scan), when the scan's
  // intensity presence differs from the block's, when frame_id differs, or when the scan would
  // take the block's codes -- 2 bytes a ray, twice that with intensities -- past
  // kMaxScanBlockCodeBytes; then appends. A scan with no ranges, or with more than
  // scanblock::kMaxRaysPerScan, is refused: nothing is returned or appended and the idle time does
  // not restart. The caller counts the refusal as a drop.
  std::vector<ScanBlockRecord> Append(const SensorEntry& entry, const std::string& frame_id,
                                      const LaserScanData& scan, double now_steady);
  // Closes the open block once `idle_s` have passed since the last Append.
  std::vector<ScanBlockRecord> CloseIfIdle(double now_steady, double idle_s);
  std::vector<ScanBlockRecord> Close();
  std::size_t buffered_scans() const;

 private:
  std::string sensor_id_;
  std::uint32_t next_seq_ = 0;
  std::optional<ScanBlockRecord> open_;
  double last_append_steady_ = 0.0;
};

struct CloudObjectRecord {
  std::string sensor_id, frame_id;
  std::uint32_t object_seq = 0;
  std::uint64_t start_ms = 0, end_ms = 0;
  PointPosition position = PointPosition::kInt16Mm;
  PointColor color = PointColor::kNone;
  PointIntensity intensity = PointIntensity::kNone;
  std::uint16_t voxel_leaf_mm = 0;
  PointCloudEncoding encoding = PointCloudEncoding::kPlanarZstd;
  std::uint8_t zstd_level = 0;
  std::vector<SensorEntry> entries;
  std::vector<std::uint32_t> point_counts;
  std::vector<std::string> blocks;
  std::uint64_t bytes = 0;  // the sum of the blocks' sizes
};

class CloudObjectAssembler {
 public:
  explicit CloudObjectAssembler(std::string sensor_id);
  // Closes the open object first when bytes + cloud.data.size() > kCloudObjectMaxBytes, when
  // entry.time_ms >= start_ms + kCloudObjectMilliseconds, or when any format field or frame_id
  // differs; appends; then closes again at once if the object has reached the cap, which a cloud
  // as large as the cap does alone. Returns 0, 1 or 2 objects.
  std::vector<CloudObjectRecord> Append(const SensorEntry& entry, const std::string& frame_id,
                                        const EncodedPointCloud& cloud, double now_steady);
  // Closes the open object once `idle_s` have passed since the last Append.
  std::vector<CloudObjectRecord> CloseIfIdle(double now_steady, double idle_s);
  std::vector<CloudObjectRecord> Close();
  std::size_t buffered_clouds() const;
  std::uint64_t buffered_bytes() const;

 private:
  std::string sensor_id_;
  std::uint32_t next_seq_ = 0;
  std::optional<CloudObjectRecord> open_;
  double last_append_steady_ = 0.0;
};

}  // namespace c3d
