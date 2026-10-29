#include "c3d/sensor_blocks.hpp"

#include <algorithm>
#include <utility>

#include "c3d/scan_block.hpp"

namespace c3d {
namespace {

template <typename Record>
std::vector<Record> TakeOpen(std::optional<Record>& open) {
  std::vector<Record> out;
  if (open) {
    out.push_back(std::move(*open));
    open.reset();
  }
  return out;
}

bool SameStream(const VideoChunkContext& a, const VideoChunkContext& b) {
  return a.width == b.width && a.height == b.height && a.frame_id == b.frame_id &&
         a.codec_params == b.codec_params;
}

bool HasIntensities(const LaserScanData& scan) {
  return scan.intensities.size() == scan.ranges.size();
}

bool SameFormat(const CloudObjectRecord& object, const std::string& frame_id,
                const EncodedPointCloud& cloud) {
  return object.position == cloud.position && object.color == cloud.color &&
         object.intensity == cloud.intensity && object.voxel_leaf_mm == cloud.voxel_leaf_mm &&
         object.encoding == cloud.encoding && object.zstd_level == cloud.zstd_level &&
         object.frame_id == frame_id;
}

}  // namespace

VideoChunkAssembler::VideoChunkAssembler(std::string camera_id)
    : camera_id_(std::move(camera_id)) {}

std::vector<CameraChunkRecord> VideoChunkAssembler::Append(const SensorEntry& entry,
                                                          const VideoChunkContext& context,
                                                          std::string access_unit, bool keyframe,
                                                          VideoUnitFate* fate) {
  std::vector<CameraChunkRecord> out;
  VideoUnitFate unused = VideoUnitFate::kAppended;
  VideoUnitFate& result = fate ? *fate : unused;
  result = VideoUnitFate::kDropped;
  if (keyframe) {
    refusing_ = false;
    out = TakeOpen(open_);
    if (access_unit.size() > kMaxVideoFirstUnitBytes) {
      ++dropped_before_keyframe_;
      return out;
    }
    open_.emplace();
    open_->camera_id = camera_id_;
    open_->chunk_seq = next_seq_++;
    open_->start_ms = entry.time_ms;
    open_->end_ms = entry.time_ms;
    open_->context = context;
  } else if (refusing_) {
    result = VideoUnitFate::kRefused;
    return out;
  } else if (!open_ || !SameStream(open_->context, context)) {
    ++dropped_before_keyframe_;
    return out;
  } else if (open_->data.size() + access_unit.size() > kMaxVideoChunkBytes) {
    out = TakeOpen(open_);
    if (fate) {
      refusing_ = true;
      result = VideoUnitFate::kRefused;
    } else {
      ++dropped_before_keyframe_;
    }
    return out;
  }
  FrameMeta frame;
  frame.time_ms = entry.time_ms;
  frame.seq = entry.seq;
  frame.pose = entry.pose;
  frame.byte_offset = static_cast<std::uint32_t>(open_->data.size());
  frame.byte_length = static_cast<std::uint32_t>(access_unit.size());
  frame.keyframe = keyframe;
  open_->frames.push_back(frame);
  open_->data += access_unit;
  open_->end_ms = std::max(open_->end_ms, entry.time_ms);
  result = VideoUnitFate::kAppended;
  return out;
}

std::vector<CameraChunkRecord> VideoChunkAssembler::Close() { return TakeOpen(open_); }

std::uint64_t VideoChunkAssembler::dropped_before_keyframe() const {
  return dropped_before_keyframe_;
}

std::size_t VideoChunkAssembler::buffered_frames() const {
  return open_ ? open_->frames.size() : 0;
}

std::uint64_t VideoChunkAssembler::buffered_bytes() const {
  return open_ ? open_->data.size() : 0;
}

ScanBlockAssembler::ScanBlockAssembler(std::string sensor_id) : sensor_id_(std::move(sensor_id)) {}

std::vector<ScanBlockRecord> ScanBlockAssembler::Append(const SensorEntry& entry,
                                                        const std::string& frame_id,
                                                        const LaserScanData& scan,
                                                        double now_steady) {
  std::vector<ScanBlockRecord> out;
  if (scan.ranges.empty() || scan.ranges.size() > scanblock::kMaxRaysPerScan) return out;
  const bool intensities = HasIntensities(scan);
  // Every scan in a block has the same rays and the same intensity presence, so the block's codes
  // are its scans times this.
  const std::uint64_t code_bytes =
      static_cast<std::uint64_t>(scan.ranges.size()) * 2u * (intensities ? 2u : 1u);
  if (open_ && (entry.time_ms >= open_->start_ms + kScanBlockMilliseconds ||
                !(scanblock::GeometryOf(scan) == scanblock::GeometryOf(open_->scans.front())) ||
                intensities != open_->intensities || frame_id != open_->frame_id ||
                (open_->scans.size() + 1) * code_bytes > kMaxScanBlockCodeBytes)) {
    out = TakeOpen(open_);
  }
  if (!open_) {
    open_.emplace();
    open_->sensor_id = sensor_id_;
    open_->frame_id = frame_id;
    open_->block_seq = next_seq_++;
    open_->start_ms = entry.time_ms;
    open_->end_ms = entry.time_ms;
    open_->intensities = intensities;
  }
  open_->entries.push_back(entry);
  open_->scans.push_back(scan);
  open_->end_ms = std::max(open_->end_ms, entry.time_ms);
  last_append_steady_ = now_steady;
  return out;
}

std::vector<ScanBlockRecord> ScanBlockAssembler::CloseIfIdle(double now_steady, double idle_s) {
  if (open_ && now_steady - last_append_steady_ >= idle_s) return TakeOpen(open_);
  return {};
}

std::vector<ScanBlockRecord> ScanBlockAssembler::Close() { return TakeOpen(open_); }

std::size_t ScanBlockAssembler::buffered_scans() const {
  return open_ ? open_->scans.size() : 0;
}

CloudObjectAssembler::CloudObjectAssembler(std::string sensor_id)
    : sensor_id_(std::move(sensor_id)) {}

std::vector<CloudObjectRecord> CloudObjectAssembler::Append(const SensorEntry& entry,
                                                            const std::string& frame_id,
                                                            const EncodedPointCloud& cloud,
                                                            double now_steady) {
  std::vector<CloudObjectRecord> out;
  if (open_ && (open_->bytes + cloud.data.size() > kCloudObjectMaxBytes ||
                entry.time_ms >= open_->start_ms + kCloudObjectMilliseconds ||
                !SameFormat(*open_, frame_id, cloud))) {
    out = TakeOpen(open_);
  }
  if (!open_) {
    open_.emplace();
    open_->sensor_id = sensor_id_;
    open_->frame_id = frame_id;
    open_->object_seq = next_seq_++;
    open_->start_ms = entry.time_ms;
    open_->end_ms = entry.time_ms;
    open_->position = cloud.position;
    open_->color = cloud.color;
    open_->intensity = cloud.intensity;
    open_->voxel_leaf_mm = cloud.voxel_leaf_mm;
    open_->encoding = cloud.encoding;
    open_->zstd_level = cloud.zstd_level;
  }
  open_->entries.push_back(entry);
  open_->point_counts.push_back(cloud.point_count);
  open_->blocks.push_back(cloud.data);
  open_->bytes += cloud.data.size();
  open_->end_ms = std::max(open_->end_ms, entry.time_ms);
  last_append_steady_ = now_steady;
  if (open_->bytes >= kCloudObjectMaxBytes) {
    for (CloudObjectRecord& full : TakeOpen(open_)) out.push_back(std::move(full));
  }
  return out;
}

std::vector<CloudObjectRecord> CloudObjectAssembler::CloseIfIdle(double now_steady,
                                                                 double idle_s) {
  if (open_ && now_steady - last_append_steady_ >= idle_s) return TakeOpen(open_);
  return {};
}

std::vector<CloudObjectRecord> CloudObjectAssembler::Close() { return TakeOpen(open_); }

std::size_t CloudObjectAssembler::buffered_clouds() const {
  return open_ ? open_->blocks.size() : 0;
}

std::uint64_t CloudObjectAssembler::buffered_bytes() const {
  return open_ ? open_->bytes : 0;
}

}  // namespace c3d
