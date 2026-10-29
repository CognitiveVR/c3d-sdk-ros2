#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "c3d/scan_block.hpp"
#include "c3d/sensor_blocks.hpp"

using namespace c3d;

namespace {

SensorEntry EntryAt(std::uint64_t time_ms, std::uint32_t seq) {
  SensorEntry entry;
  entry.time_ms = time_ms;
  entry.seq = seq;
  entry.pose.position.x = static_cast<double>(seq);
  return entry;
}

VideoChunkContext CameraContext() {
  VideoChunkContext context;
  context.frame_id = "front_camera_optical_frame";
  context.width = 640;
  context.height = 480;
  context.nominal_fps = 30.0f;
  context.hfov_deg = 69.0f;
  context.vfov_deg = 42.0f;
  context.codec_params = std::string("\x01\x64\x00\x1e", 4);
  context.encoder_settings = "x264 ultrafast crf=28 threads=1 gop=3s";
  return context;
}

LaserScanData ScanOf(std::size_t rays, bool intensities = false) {
  LaserScanData scan;
  scan.angle_min = -1.5f;
  scan.angle_max = 1.5f;
  scan.angle_increment = 3.0f / static_cast<float>(rays);
  scan.time_increment = 0.0001f;
  scan.scan_time = 0.1f;
  scan.range_min = 0.15f;
  scan.range_max = 12.0f;
  scan.ranges.assign(rays, 2.0f);
  if (intensities) scan.intensities.assign(rays, 47.0f);
  return scan;
}

EncodedPointCloud CloudOf(std::size_t bytes, std::uint32_t points = 10, char fill = 'c') {
  EncodedPointCloud cloud;
  cloud.ok = true;
  cloud.point_count = points;
  cloud.position = PointPosition::kInt16Mm;
  cloud.color = PointColor::kNone;
  cloud.intensity = PointIntensity::kNone;
  cloud.voxel_leaf_mm = 50;
  cloud.zstd_level = 3;
  cloud.encoding = PointCloudEncoding::kPlanarDeltaShuffleZstd;
  cloud.data.assign(bytes, fill);
  return cloud;
}

}  // namespace

TEST(VideoChunkAssembler, AKeyframeClosesTheOpenChunkAndOpensTheNext) {
  VideoChunkAssembler video("front");
  const VideoChunkContext context = CameraContext();
  std::vector<CameraChunkRecord> chunks;
  for (std::uint32_t i = 0; i < 12; ++i) {
    const bool keyframe = i == 0 || i == 5 || i == 10;
    std::vector<CameraChunkRecord> closed = video.Append(
        EntryAt(1000 + 33 * i, i), context, std::string(4, static_cast<char>('a' + i)), keyframe);
    EXPECT_EQ(closed.size(), (i == 5 || i == 10) ? 1u : 0u) << "access unit " << i;
    for (CameraChunkRecord& chunk : closed) chunks.push_back(std::move(chunk));
  }
  EXPECT_EQ(video.buffered_frames(), 2u);
  for (CameraChunkRecord& chunk : video.Close()) chunks.push_back(std::move(chunk));
  EXPECT_EQ(video.buffered_frames(), 0u);
  EXPECT_TRUE(video.Close().empty());

  ASSERT_EQ(chunks.size(), 3u);
  const std::size_t sizes[] = {5, 5, 2};
  const std::uint32_t first_seq[] = {0, 5, 10};
  const std::uint64_t starts[] = {1000, 1165, 1330};
  const std::uint64_t ends[] = {1132, 1297, 1363};
  for (std::size_t k = 0; k < 3; ++k) {
    const CameraChunkRecord& chunk = chunks[k];
    EXPECT_EQ(chunk.camera_id, "front");
    EXPECT_EQ(chunk.chunk_seq, k);
    EXPECT_EQ(chunk.start_ms, starts[k]);
    EXPECT_EQ(chunk.end_ms, ends[k]);
    ASSERT_EQ(chunk.frames.size(), sizes[k]) << "chunk " << k;
    EXPECT_EQ(chunk.data.size(), 4 * sizes[k]);
    for (std::size_t j = 0; j < sizes[k]; ++j) {
      EXPECT_EQ(chunk.frames[j].seq, first_seq[k] + j);
      EXPECT_EQ(chunk.frames[j].keyframe, j == 0) << "chunk " << k << " frame " << j;
    }
  }
}

TEST(VideoChunkAssembler, OffsetsAndLengthsTileTheData) {
  VideoChunkAssembler video("front");
  const VideoChunkContext context = CameraContext();
  const std::vector<std::string> units = {"IIIIIII", "p", "PPPPPPPPPPPPP", "qqqq"};
  for (std::uint32_t i = 0; i < units.size(); ++i) {
    EXPECT_TRUE(video.Append(EntryAt(33 * i, i), context, units[i], i == 0).empty());
  }
  EXPECT_EQ(video.buffered_frames(), 4u);
  EXPECT_EQ(video.buffered_bytes(), 25u);

  const std::vector<CameraChunkRecord> chunks = video.Close();
  ASSERT_EQ(chunks.size(), 1u);
  const CameraChunkRecord& chunk = chunks[0];
  EXPECT_EQ(chunk.data, "IIIIIII" "p" "PPPPPPPPPPPPP" "qqqq");
  ASSERT_EQ(chunk.frames.size(), 4u);
  const std::uint32_t offsets[] = {0, 7, 8, 21};
  const std::uint32_t lengths[] = {7, 1, 13, 4};
  std::uint64_t total = 0;
  for (std::uint32_t i = 0; i < 4; ++i) {
    const FrameMeta& frame = chunk.frames[i];
    EXPECT_EQ(frame.byte_offset, offsets[i]) << "frame " << i;
    EXPECT_EQ(frame.byte_length, lengths[i]) << "frame " << i;
    EXPECT_EQ(chunk.data.substr(frame.byte_offset, frame.byte_length), units[i]);
    EXPECT_EQ(frame.time_ms, 33u * i);
    EXPECT_EQ(frame.seq, i);
    EXPECT_DOUBLE_EQ(frame.pose.position.x, static_cast<double>(i));
    if (i > 0) EXPECT_GT(frame.byte_offset, chunk.frames[i - 1].byte_offset);
    total += frame.byte_length;
  }
  EXPECT_EQ(total, chunk.data.size());
}

TEST(VideoChunkAssembler, ANonKeyframeWithNoOpenChunkIsDroppedAndCounted) {
  VideoChunkAssembler video("front");
  const VideoChunkContext context = CameraContext();
  EXPECT_TRUE(video.Append(EntryAt(0, 0), context, "p0", false).empty());
  EXPECT_EQ(video.dropped_before_keyframe(), 1u);
  EXPECT_EQ(video.buffered_frames(), 0u);

  EXPECT_TRUE(video.Append(EntryAt(33, 1), context, "I1", true).empty());
  EXPECT_TRUE(video.Append(EntryAt(66, 2), context, "p2", false).empty());
  ASSERT_EQ(video.Close().size(), 1u);

  // After a close, a P-frame belongs to a GOP whose keyframe went with the closed chunk.
  EXPECT_TRUE(video.Append(EntryAt(99, 3), context, "p3", false).empty());
  EXPECT_EQ(video.dropped_before_keyframe(), 2u);
  EXPECT_EQ(video.buffered_frames(), 0u);
  EXPECT_EQ(video.buffered_bytes(), 0u);

  EXPECT_TRUE(video.Append(EntryAt(133, 4), context, "I4", true).empty());
  EXPECT_EQ(video.buffered_frames(), 1u);
  EXPECT_TRUE(video.Append(EntryAt(166, 5), context, "p5", false).empty());
  EXPECT_EQ(video.dropped_before_keyframe(), 2u);

  const std::vector<CameraChunkRecord> chunks = video.Close();
  ASSERT_EQ(chunks.size(), 1u);
  EXPECT_EQ(chunks[0].chunk_seq, 1u);
  EXPECT_EQ(chunks[0].data, "I4p5");
  EXPECT_EQ(chunks[0].start_ms, 133u);
  EXPECT_EQ(chunks[0].end_ms, 166u);
}

TEST(VideoChunkAssembler, AContextChangeWithoutAKeyframeIsDroppedAndCounted) {
  VideoChunkAssembler video("front");
  const VideoChunkContext context = CameraContext();
  EXPECT_TRUE(video.Append(EntryAt(0, 0), context, "I0", true).empty());
  EXPECT_TRUE(video.Append(EntryAt(33, 1), context, "p1", false).empty());

  VideoChunkContext narrower = context;
  narrower.width = 320;
  VideoChunkContext shorter = context;
  shorter.height = 240;
  VideoChunkContext elsewhere = context;
  elsewhere.frame_id = "rear_camera_optical_frame";
  VideoChunkContext restarted = context;
  restarted.codec_params = std::string("\x01\x42\x00\x1e", 4);

  EXPECT_TRUE(video.Append(EntryAt(66, 2), narrower, "p2", false).empty());
  EXPECT_EQ(video.dropped_before_keyframe(), 1u);
  EXPECT_TRUE(video.Append(EntryAt(99, 3), shorter, "p3", false).empty());
  EXPECT_EQ(video.dropped_before_keyframe(), 2u);
  EXPECT_TRUE(video.Append(EntryAt(133, 4), elsewhere, "p4", false).empty());
  EXPECT_EQ(video.dropped_before_keyframe(), 3u);
  EXPECT_TRUE(video.Append(EntryAt(166, 5), restarted, "p5", false).empty());
  EXPECT_EQ(video.dropped_before_keyframe(), 4u);
  EXPECT_EQ(video.buffered_frames(), 2u);
  EXPECT_EQ(video.buffered_bytes(), 4u);

  // A keyframe in the new context starts a chunk of its own.
  const std::vector<CameraChunkRecord> closed =
      video.Append(EntryAt(200, 6), narrower, "I6", true);
  ASSERT_EQ(closed.size(), 1u);
  EXPECT_EQ(closed[0].data, "I0p1");
  EXPECT_EQ(closed[0].context.width, 640);
  const std::vector<CameraChunkRecord> rest = video.Close();
  ASSERT_EQ(rest.size(), 1u);
  EXPECT_EQ(rest[0].data, "I6");
  EXPECT_EQ(rest[0].context.width, 320);
  EXPECT_EQ(video.dropped_before_keyframe(), 4u);
}

// A stream with no keyframe would grow one chunk without bound, so the chunk closes at
// kMaxVideoChunkBytes, and the frames after it have no keyframe to decode from until the next one.
TEST(VideoChunkAssembler, AChunkClosesAtItsByteCapAndTheFramesUntilTheNextKeyframeAreDropped) {
  VideoChunkAssembler video("front");
  const VideoChunkContext context = CameraContext();
  EXPECT_TRUE(
      video.Append(EntryAt(0, 0), context, std::string(kMaxVideoChunkBytes - 10, 'I'), true)
          .empty());
  EXPECT_TRUE(video.Append(EntryAt(33, 1), context, std::string(10, 'p'), false).empty())
      << "a frame that fills the chunk exactly to the cap closed it";
  EXPECT_EQ(video.buffered_bytes(), kMaxVideoChunkBytes);
  EXPECT_EQ(video.dropped_before_keyframe(), 0u);

  const std::vector<CameraChunkRecord> closed = video.Append(EntryAt(66, 2), context, "p", false);
  ASSERT_EQ(closed.size(), 1u) << "a frame one byte past the cap joined the chunk";
  EXPECT_EQ(closed[0].data.size(), kMaxVideoChunkBytes);
  EXPECT_EQ(closed[0].frames.size(), 2u);
  EXPECT_EQ(closed[0].end_ms, 33u);
  EXPECT_EQ(video.dropped_before_keyframe(), 1u);
  EXPECT_EQ(video.buffered_frames(), 0u);
  EXPECT_TRUE(video.Append(EntryAt(99, 3), context, "p", false).empty());
  EXPECT_EQ(video.dropped_before_keyframe(), 2u);

  EXPECT_TRUE(video.Append(EntryAt(133, 4), context, "I4", true).empty());
  EXPECT_TRUE(video.Append(EntryAt(166, 5), context, "p5", false).empty());
  const std::vector<CameraChunkRecord> next = video.Close();
  ASSERT_EQ(next.size(), 1u);
  EXPECT_EQ(next[0].chunk_seq, 1u);
  EXPECT_EQ(next[0].data, "I4p5");
  EXPECT_EQ(video.dropped_before_keyframe(), 2u);
}

// Asked for a fate, the assembler refuses the unit that would pass the cap instead of dropping it:
// the chunk before it closes whole, and that unit and every one after it until a keyframe are left
// to the caller to re-encode from a new keyframe. Nothing refused is counted as dropped.
TEST(VideoChunkAssembler, AUnitPastTheCapIsRefusedAndSoIsEveryUnitUntilTheNextKeyframe) {
  VideoChunkAssembler video("front");
  const VideoChunkContext context = CameraContext();
  VideoUnitFate fate = VideoUnitFate::kDropped;
  EXPECT_TRUE(video.Append(EntryAt(0, 0), context, std::string(kMaxVideoChunkBytes - 10, 'I'),
                           true, &fate)
                  .empty());
  EXPECT_EQ(fate, VideoUnitFate::kAppended);
  EXPECT_TRUE(video.Append(EntryAt(33, 1), context, std::string(10, 'p'), false, &fate).empty());
  EXPECT_EQ(fate, VideoUnitFate::kAppended) << "a unit that fills the chunk to the cap was refused";

  const std::vector<CameraChunkRecord> closed =
      video.Append(EntryAt(66, 2), context, "p", false, &fate);
  EXPECT_EQ(fate, VideoUnitFate::kRefused);
  ASSERT_EQ(closed.size(), 1u) << "the chunk before the refused unit stayed open";
  EXPECT_EQ(closed[0].data.size(), kMaxVideoChunkBytes);
  EXPECT_EQ(closed[0].frames.size(), 2u);
  EXPECT_TRUE(video.Append(EntryAt(99, 3), context, "p", false, &fate).empty());
  EXPECT_EQ(fate, VideoUnitFate::kRefused);
  EXPECT_EQ(video.dropped_before_keyframe(), 0u);
  EXPECT_EQ(video.buffered_frames(), 0u);

  EXPECT_TRUE(video.Append(EntryAt(66, 2), context, "I2", true, &fate).empty());
  EXPECT_EQ(fate, VideoUnitFate::kAppended);
  EXPECT_TRUE(video.Append(EntryAt(99, 3), context, "p3", false, &fate).empty());
  EXPECT_EQ(fate, VideoUnitFate::kAppended);
  const std::vector<CameraChunkRecord> next = video.Close();
  ASSERT_EQ(next.size(), 1u);
  EXPECT_EQ(next[0].chunk_seq, 1u);
  EXPECT_EQ(next[0].data, "I2p3");
  EXPECT_EQ(video.dropped_before_keyframe(), 0u);
}

// A chunk's first unit is a keyframe, and its re-encode would be one as large, so it is never
// refused: kept, as a chunk of one, up to kMaxVideoFirstUnitBytes, and dropped above it, the units
// after it then dropped until the next keyframe.
TEST(VideoChunkAssembler, AKeyframeIsNeverRefusedKeptUpToItsCapAndDroppedAbove) {
  VideoChunkAssembler video("front");
  const VideoChunkContext context = CameraContext();
  VideoUnitFate fate = VideoUnitFate::kRefused;
  EXPECT_TRUE(video.Append(EntryAt(0, 0), context, std::string(kMaxVideoFirstUnitBytes, 'I'), true,
                           &fate)
                  .empty());
  EXPECT_EQ(fate, VideoUnitFate::kAppended);
  EXPECT_EQ(video.buffered_bytes(), kMaxVideoFirstUnitBytes);

  const std::vector<CameraChunkRecord> closed = video.Append(
      EntryAt(33, 1), context, std::string(kMaxVideoFirstUnitBytes + 1, 'I'), true, &fate);
  EXPECT_EQ(fate, VideoUnitFate::kDropped);
  ASSERT_EQ(closed.size(), 1u);
  EXPECT_EQ(closed[0].data.size(), kMaxVideoFirstUnitBytes);
  EXPECT_EQ(video.dropped_before_keyframe(), 1u);
  EXPECT_TRUE(video.Append(EntryAt(66, 2), context, "p", false, &fate).empty());
  EXPECT_EQ(fate, VideoUnitFate::kDropped);
  EXPECT_EQ(video.dropped_before_keyframe(), 2u);
  EXPECT_EQ(video.buffered_frames(), 0u);
}

TEST(VideoChunkAssembler, TheChunkCarriesTheFirstFramesContext) {
  VideoChunkAssembler video("front");
  const VideoChunkContext first = CameraContext();
  VideoChunkContext zoomed = first;
  zoomed.hfov_deg = 30.0f;
  zoomed.vfov_deg = 18.0f;
  for (std::uint32_t i = 0; i < 5; ++i) {
    EXPECT_TRUE(video.Append(EntryAt(33 * i, i), i < 3 ? first : zoomed, "u", i == 0).empty());
  }
  EXPECT_EQ(video.dropped_before_keyframe(), 0u);

  const std::vector<CameraChunkRecord> chunks = video.Close();
  ASSERT_EQ(chunks.size(), 1u);
  EXPECT_EQ(chunks[0].frames.size(), 5u);
  const VideoChunkContext& context = chunks[0].context;
  EXPECT_FLOAT_EQ(context.hfov_deg, 69.0f);
  EXPECT_FLOAT_EQ(context.vfov_deg, 42.0f);
  EXPECT_EQ(context.frame_id, "front_camera_optical_frame");
  EXPECT_EQ(context.width, 640);
  EXPECT_EQ(context.height, 480);
  EXPECT_FLOAT_EQ(context.nominal_fps, 30.0f);
  EXPECT_EQ(context.codec, "h264");
  EXPECT_EQ(context.codec_params, std::string("\x01\x64\x00\x1e", 4));
  EXPECT_EQ(context.encoder_settings, "x264 ultrafast crf=28 threads=1 gop=3s");
}

TEST(ScanBlockAssembler, ThreeSecondsOfStampsCloseABlock) {
  ScanBlockAssembler lidar("front_lidar");
  const LaserScanData scan = ScanOf(640);
  EXPECT_TRUE(lidar.Append(EntryAt(0, 0), "laser", scan, 10.0).empty());
  EXPECT_TRUE(lidar.Append(EntryAt(1000, 1), "laser", scan, 11.0).empty());
  EXPECT_TRUE(lidar.Append(EntryAt(2999, 2), "laser", scan, 12.0).empty());
  EXPECT_EQ(lidar.buffered_scans(), 3u);

  const std::vector<ScanBlockRecord> closed = lidar.Append(EntryAt(3000, 3), "laser", scan, 13.0);
  ASSERT_EQ(closed.size(), 1u);
  const ScanBlockRecord& block = closed[0];
  EXPECT_EQ(block.sensor_id, "front_lidar");
  EXPECT_EQ(block.frame_id, "laser");
  EXPECT_EQ(block.block_seq, 0u);
  EXPECT_EQ(block.start_ms, 0u);
  EXPECT_EQ(block.end_ms, 2999u);
  EXPECT_FALSE(block.intensities);
  EXPECT_EQ(block.scans.size(), 3u);
  ASSERT_EQ(block.entries.size(), 3u);
  const std::uint64_t stamps[] = {0, 1000, 2999};
  for (std::uint32_t i = 0; i < 3; ++i) {
    EXPECT_EQ(block.entries[i].time_ms, stamps[i]);
    EXPECT_EQ(block.entries[i].seq, i);
    EXPECT_DOUBLE_EQ(block.entries[i].pose.position.x, static_cast<double>(i));
  }
  EXPECT_EQ(lidar.buffered_scans(), 1u);

  const std::vector<ScanBlockRecord> rest = lidar.Close();
  ASSERT_EQ(rest.size(), 1u);
  EXPECT_EQ(rest[0].block_seq, 1u);
  EXPECT_EQ(rest[0].start_ms, 3000u);
  EXPECT_EQ(rest[0].end_ms, 3000u);
  ASSERT_EQ(rest[0].entries.size(), 1u);
  EXPECT_EQ(rest[0].entries[0].seq, 3u);
}

// A lidar whose stamp stops advancing never reaches 3 s of stamps, so a block is also bounded by its
// codes: it closes before the scan that would take them past kMaxScanBlockCodeBytes, intensities
// counting double.
TEST(ScanBlockAssembler, ABlockClosesBeforeItsCodesWouldPassTheCap) {
  for (const bool intensities : {false, true}) {
    SCOPED_TRACE(intensities ? "with intensities" : "ranges only");
    ScanBlockAssembler blocks("scan");
    // 32 KiB of codes a scan either way: 16,384 rays, or 8,192 with as many intensities.
    const LaserScanData scan = ScanOf(intensities ? 8192 : 16384, intensities);
    const std::uint32_t fit = static_cast<std::uint32_t>(kMaxScanBlockCodeBytes / (32 * 1024));
    ASSERT_EQ(fit, 1024u);
    for (std::uint32_t i = 0; i < fit; ++i) {
      ASSERT_TRUE(blocks.Append(EntryAt(5000, i), "laser", scan, 1.0).empty())
          << "closed at scan " << i << ", short of the cap";
    }
    const std::vector<ScanBlockRecord> closed =
        blocks.Append(EntryAt(5000, fit), "laser", scan, 1.0);
    ASSERT_EQ(closed.size(), 1u) << "a scan past the cap joined the block";
    EXPECT_EQ(closed[0].scans.size(), fit);
    EXPECT_EQ(closed[0].intensities, intensities);
    EXPECT_EQ(blocks.buffered_scans(), 1u);
  }
}

TEST(ScanBlockAssembler, AGeometryChangeClosesTheBlock) {
  ScanBlockAssembler lidar("front_lidar");
  const LaserScanData base = ScanOf(640);
  LaserScanData finer = base;
  finer.angle_increment = base.angle_increment / 2.0f;
  LaserScanData more_rays = finer;
  more_rays.ranges.push_back(2.0f);
  LaserScanData farther = more_rays;
  farther.range_max = 30.0f;

  EXPECT_TRUE(lidar.Append(EntryAt(0, 0), "laser", base, 10.0).empty());
  const std::vector<ScanBlockRecord> after_increment =
      lidar.Append(EntryAt(100, 1), "laser", finer, 10.1);
  ASSERT_EQ(after_increment.size(), 1u);
  const std::vector<ScanBlockRecord> after_rays =
      lidar.Append(EntryAt(200, 2), "laser", more_rays, 10.2);
  ASSERT_EQ(after_rays.size(), 1u);
  const std::vector<ScanBlockRecord> after_range =
      lidar.Append(EntryAt(300, 3), "laser", farther, 10.3);
  ASSERT_EQ(after_range.size(), 1u);
  // The same scan in another frame does not join the block either.
  const std::vector<ScanBlockRecord> after_frame =
      lidar.Append(EntryAt(400, 4), "laser_rear", farther, 10.4);
  ASSERT_EQ(after_frame.size(), 1u);
  const std::vector<ScanBlockRecord> last = lidar.Close();
  ASSERT_EQ(last.size(), 1u);

  const ScanBlockRecord* blocks[] = {&after_increment[0], &after_rays[0], &after_range[0],
                                     &after_frame[0], &last[0]};
  const LaserScanData* expected[] = {&base, &finer, &more_rays, &farther, &farther};
  const char* frames[] = {"laser", "laser", "laser", "laser", "laser_rear"};
  for (std::uint32_t k = 0; k < 5; ++k) {
    EXPECT_EQ(blocks[k]->block_seq, k);
    EXPECT_EQ(blocks[k]->frame_id, frames[k]);
    ASSERT_EQ(blocks[k]->scans.size(), 1u) << "block " << k;
    EXPECT_EQ(blocks[k]->entries[0].seq, k);
    EXPECT_FLOAT_EQ(blocks[k]->scans[0].angle_increment, expected[k]->angle_increment);
    EXPECT_EQ(blocks[k]->scans[0].ranges.size(), expected[k]->ranges.size());
    EXPECT_FLOAT_EQ(blocks[k]->scans[0].range_max, expected[k]->range_max);
  }
  EXPECT_EQ(blocks[1]->scans[0].ranges.size(), 640u);
  EXPECT_EQ(blocks[2]->scans[0].ranges.size(), 641u);
}

TEST(ScanBlockAssembler, TimingJitterDoesNotCloseTheBlock) {
  ScanBlockAssembler lidar("front_lidar");
  for (std::uint32_t i = 0; i < 30; ++i) {
    LaserScanData scan = ScanOf(640);
    scan.scan_time = 0.1f + 0.0003f * static_cast<float>(i);
    scan.time_increment = scan.scan_time / 640.0f;
    EXPECT_TRUE(lidar.Append(EntryAt(100 * i, i), "laser", scan, 10.0 + 0.1 * i).empty())
        << "scan " << i;
  }
  EXPECT_EQ(lidar.buffered_scans(), 30u);

  const std::vector<ScanBlockRecord> blocks = lidar.Close();
  ASSERT_EQ(blocks.size(), 1u);
  ASSERT_EQ(blocks[0].scans.size(), 30u);
  EXPECT_EQ(blocks[0].start_ms, 0u);
  EXPECT_EQ(blocks[0].end_ms, 2900u);
  for (std::uint32_t i = 0; i < 30; ++i) {
    const float scan_time = 0.1f + 0.0003f * static_cast<float>(i);
    EXPECT_FLOAT_EQ(blocks[0].scans[i].scan_time, scan_time);
    EXPECT_FLOAT_EQ(blocks[0].scans[i].time_increment, scan_time / 640.0f);
  }
}

TEST(ScanBlockAssembler, AnIntensityPresenceChangeClosesTheBlock) {
  ScanBlockAssembler lidar("front_lidar");
  const LaserScanData with = ScanOf(640, true);
  const LaserScanData without = ScanOf(640, false);
  LaserScanData partial = with;
  partial.intensities.resize(320);

  EXPECT_TRUE(lidar.Append(EntryAt(0, 0), "laser", with, 10.0).empty());
  const std::vector<ScanBlockRecord> first = lidar.Append(EntryAt(100, 1), "laser", without, 10.1);
  ASSERT_EQ(first.size(), 1u);
  EXPECT_TRUE(first[0].intensities);
  EXPECT_EQ(first[0].scans.size(), 1u);

  // Fewer intensities than rays is no intensities.
  EXPECT_TRUE(lidar.Append(EntryAt(200, 2), "laser", partial, 10.2).empty());
  const std::vector<ScanBlockRecord> second = lidar.Append(EntryAt(300, 3), "laser", with, 10.3);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_FALSE(second[0].intensities);
  EXPECT_EQ(second[0].scans.size(), 2u);

  const std::vector<ScanBlockRecord> third = lidar.Close();
  ASSERT_EQ(third.size(), 1u);
  EXPECT_TRUE(third[0].intensities);
  EXPECT_EQ(third[0].scans.size(), 1u);
  EXPECT_EQ(third[0].block_seq, 2u);
}

TEST(ScanBlockAssembler, IdleAndForceClose) {
  ScanBlockAssembler lidar("front_lidar");
  EXPECT_TRUE(lidar.Close().empty());
  EXPECT_TRUE(lidar.CloseIfIdle(1000.0, 3.0).empty());

  const LaserScanData scan = ScanOf(640);
  EXPECT_TRUE(lidar.Append(EntryAt(0, 0), "laser", scan, 100.0).empty());
  EXPECT_TRUE(lidar.Append(EntryAt(100, 1), "laser", scan, 101.0).empty());
  // Idle counts from the last append, not the first.
  EXPECT_TRUE(lidar.CloseIfIdle(103.9, 3.0).empty());
  EXPECT_EQ(lidar.buffered_scans(), 2u);
  const std::vector<ScanBlockRecord> idle = lidar.CloseIfIdle(104.0, 3.0);
  ASSERT_EQ(idle.size(), 1u);
  EXPECT_EQ(idle[0].scans.size(), 2u);
  EXPECT_EQ(idle[0].block_seq, 0u);
  EXPECT_EQ(lidar.buffered_scans(), 0u);
  EXPECT_TRUE(lidar.CloseIfIdle(1000.0, 3.0).empty());

  EXPECT_TRUE(lidar.Append(EntryAt(200, 2), "laser", scan, 105.0).empty());
  EXPECT_TRUE(lidar.CloseIfIdle(105.4, 0.5).empty());
  const std::vector<ScanBlockRecord> shortened = lidar.CloseIfIdle(105.5, 0.5);
  ASSERT_EQ(shortened.size(), 1u);
  EXPECT_EQ(shortened[0].block_seq, 1u);

  EXPECT_TRUE(lidar.Append(EntryAt(300, 3), "laser", scan, 106.0).empty());
  const std::vector<ScanBlockRecord> forced = lidar.Close();
  ASSERT_EQ(forced.size(), 1u);
  EXPECT_EQ(forced[0].scans.size(), 1u);
  EXPECT_EQ(forced[0].block_seq, 2u);
  EXPECT_EQ(lidar.buffered_scans(), 0u);
  EXPECT_TRUE(lidar.Close().empty());
}

TEST(ScanBlockAssembler, AScanWithNoRangesIsRefused) {
  ScanBlockAssembler lidar("front_lidar");
  LaserScanData empty = ScanOf(640);
  empty.ranges.clear();
  EXPECT_TRUE(lidar.Append(EntryAt(0, 0), "laser", empty, 10.0).empty());
  EXPECT_EQ(lidar.buffered_scans(), 0u);
  EXPECT_TRUE(lidar.Close().empty());

  // With a block open, a refused scan neither closes nor joins it, and does not reset its idle.
  const LaserScanData scan = ScanOf(640);
  EXPECT_TRUE(lidar.Append(EntryAt(100, 1), "laser", scan, 20.0).empty());
  EXPECT_TRUE(lidar.Append(EntryAt(200, 2), "laser", empty, 22.0).empty());
  const LaserScanData too_many = ScanOf(scanblock::kMaxRaysPerScan + 1);
  EXPECT_TRUE(lidar.Append(EntryAt(300, 3), "laser", too_many, 22.5).empty());
  EXPECT_EQ(lidar.buffered_scans(), 1u);
  const std::vector<ScanBlockRecord> idle = lidar.CloseIfIdle(23.0, 3.0);
  ASSERT_EQ(idle.size(), 1u);
  ASSERT_EQ(idle[0].entries.size(), 1u);
  EXPECT_EQ(idle[0].entries[0].seq, 1u);
  EXPECT_EQ(idle[0].block_seq, 0u);

  const LaserScanData widest = ScanOf(scanblock::kMaxRaysPerScan);
  EXPECT_TRUE(lidar.Append(EntryAt(400, 4), "laser", widest, 30.0).empty());
  EXPECT_EQ(lidar.buffered_scans(), 1u);
  const std::vector<ScanBlockRecord> last = lidar.Close();
  ASSERT_EQ(last.size(), 1u);
  EXPECT_EQ(last[0].scans[0].ranges.size(), 65535u);
  EXPECT_EQ(last[0].block_seq, 1u);
}

TEST(CloudObjectAssembler, TheByteCapClosesBeforeTheCloudThatWouldExceedIt) {
  {
    // 20 clouds of 100,000 B are 2,000,000 B; a 21st would make 2,100,000, past 2,097,152.
    CloudObjectAssembler lidar("depth");
    std::vector<CloudObjectRecord> closed;
    for (std::uint32_t i = 0; i < 25; ++i) {
      std::vector<CloudObjectRecord> out =
          lidar.Append(EntryAt(i, i), "depth_frame", CloudOf(100000), 1.0);
      EXPECT_EQ(out.size(), i == 20 ? 1u : 0u) << "cloud " << i;
      for (CloudObjectRecord& object : out) closed.push_back(std::move(object));
    }
    ASSERT_EQ(closed.size(), 1u);
    EXPECT_EQ(closed[0].blocks.size(), 20u);
    EXPECT_EQ(closed[0].bytes, 2000000u);
    EXPECT_EQ(closed[0].object_seq, 0u);
    EXPECT_EQ(closed[0].end_ms, 19u);
    EXPECT_EQ(lidar.buffered_clouds(), 5u);
    EXPECT_EQ(lidar.buffered_bytes(), 500000u);
    const std::vector<CloudObjectRecord> rest = lidar.Close();
    ASSERT_EQ(rest.size(), 1u);
    EXPECT_EQ(rest[0].start_ms, 20u);
    EXPECT_EQ(rest[0].blocks.size(), 5u);
    EXPECT_EQ(rest[0].object_seq, 1u);
  }

  // 21 clouds of 99,864 B are 2,097,144 B: 8 B under the cap.
  const auto fill = [](CloudObjectAssembler& lidar) {
    for (std::uint32_t i = 0; i < 21; ++i) {
      EXPECT_TRUE(lidar.Append(EntryAt(i, i), "depth_frame", CloudOf(99864), 1.0).empty());
    }
    EXPECT_EQ(lidar.buffered_bytes(), 2097144u);
  };
  {
    CloudObjectAssembler lidar("depth");
    fill(lidar);
    const std::vector<CloudObjectRecord> out =
        lidar.Append(EntryAt(21, 21), "depth_frame", CloudOf(9), 1.0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blocks.size(), 21u);
    EXPECT_EQ(out[0].bytes, 2097144u);
    EXPECT_EQ(lidar.buffered_clouds(), 1u);
    EXPECT_EQ(lidar.buffered_bytes(), 9u);
  }
  {
    // Exactly the cap fits, and a full object closes as the cloud that filled it joins.
    CloudObjectAssembler lidar("depth");
    fill(lidar);
    const std::vector<CloudObjectRecord> out =
        lidar.Append(EntryAt(21, 21), "depth_frame", CloudOf(8), 1.0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].blocks.size(), 22u);
    EXPECT_EQ(out[0].bytes, 2097152u);
    EXPECT_EQ(lidar.buffered_clouds(), 0u);
    EXPECT_EQ(lidar.buffered_bytes(), 0u);
  }
}

TEST(CloudObjectAssembler, FiveSecondsOfStampsCloseAnObject) {
  CloudObjectAssembler lidar("depth");
  const std::uint64_t stamps[] = {0, 1000, 2000, 4999, 4000};
  for (std::uint32_t k = 0; k < 5; ++k) {
    EXPECT_TRUE(lidar
                    .Append(EntryAt(stamps[k], k), "depth_frame",
                            CloudOf(100 + k, 7 + k, static_cast<char>('a' + k)), 1.0)
                    .empty())
        << "cloud " << k;
  }
  const std::vector<CloudObjectRecord> out =
      lidar.Append(EntryAt(5000, 5), "depth_frame", CloudOf(105, 12, 'f'), 1.0);
  ASSERT_EQ(out.size(), 1u);
  const CloudObjectRecord& object = out[0];
  EXPECT_EQ(object.sensor_id, "depth");
  EXPECT_EQ(object.frame_id, "depth_frame");
  EXPECT_EQ(object.object_seq, 0u);
  EXPECT_EQ(object.start_ms, 0u);
  EXPECT_EQ(object.end_ms, 4999u);
  EXPECT_EQ(object.bytes, 510u);
  EXPECT_EQ(object.position, PointPosition::kInt16Mm);
  EXPECT_EQ(object.color, PointColor::kNone);
  EXPECT_EQ(object.intensity, PointIntensity::kNone);
  EXPECT_EQ(object.voxel_leaf_mm, 50u);
  EXPECT_EQ(object.encoding, PointCloudEncoding::kPlanarDeltaShuffleZstd);
  EXPECT_EQ(object.zstd_level, 3u);
  ASSERT_EQ(object.entries.size(), 5u);
  ASSERT_EQ(object.point_counts.size(), 5u);
  ASSERT_EQ(object.blocks.size(), 5u);
  const std::uint32_t point_counts[] = {7, 8, 9, 10, 11};
  for (std::uint32_t k = 0; k < 5; ++k) {
    EXPECT_EQ(object.entries[k].time_ms, stamps[k]);
    EXPECT_EQ(object.entries[k].seq, k);
    EXPECT_EQ(object.point_counts[k], point_counts[k]);
    EXPECT_EQ(object.blocks[k], std::string(100 + k, static_cast<char>('a' + k)));
  }
  EXPECT_EQ(lidar.buffered_clouds(), 1u);
  EXPECT_EQ(lidar.buffered_bytes(), 105u);

  const std::vector<CloudObjectRecord> rest = lidar.Close();
  ASSERT_EQ(rest.size(), 1u);
  EXPECT_EQ(rest[0].object_seq, 1u);
  EXPECT_EQ(rest[0].start_ms, 5000u);
  EXPECT_EQ(rest[0].end_ms, 5000u);
}

TEST(CloudObjectAssembler, ACloudOverTheCapAloneIsItsOwnObject) {
  CloudObjectAssembler lidar("depth");
  for (std::uint32_t i = 0; i < 3; ++i) {
    EXPECT_TRUE(lidar.Append(EntryAt(100 * i, i), "depth_frame", CloudOf(1000), 1.0).empty());
  }
  const std::vector<CloudObjectRecord> out =
      lidar.Append(EntryAt(300, 3), "depth_frame", CloudOf(2621440), 1.0);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].object_seq, 0u);
  EXPECT_EQ(out[0].blocks.size(), 3u);
  EXPECT_EQ(out[0].bytes, 3000u);
  EXPECT_EQ(out[1].object_seq, 1u);
  ASSERT_EQ(out[1].blocks.size(), 1u);
  EXPECT_EQ(out[1].bytes, 2621440u);
  EXPECT_EQ(out[1].start_ms, 300u);
  EXPECT_EQ(out[1].entries[0].seq, 3u);
  EXPECT_EQ(lidar.buffered_clouds(), 0u);
  EXPECT_EQ(lidar.buffered_bytes(), 0u);
  EXPECT_TRUE(lidar.Close().empty());

  const std::vector<CloudObjectRecord> exact =
      lidar.Append(EntryAt(400, 4), "depth_frame", CloudOf(2097152), 1.0);
  ASSERT_EQ(exact.size(), 1u);
  EXPECT_EQ(exact[0].object_seq, 2u);
  EXPECT_EQ(exact[0].blocks.size(), 1u);
  EXPECT_EQ(lidar.buffered_clouds(), 0u);
}

TEST(CloudObjectAssembler, AFormatChangeClosesTheObject) {
  CloudObjectAssembler lidar("depth");
  EncodedPointCloud cloud = CloudOf(100);
  EXPECT_TRUE(lidar.Append(EntryAt(0, 0), "depth_frame", cloud, 1.0).empty());
  EXPECT_TRUE(lidar.Append(EntryAt(10, 1), "depth_frame", cloud, 1.0).empty());
  EXPECT_EQ(lidar.buffered_clouds(), 2u);

  const std::vector<std::pair<const char*, void (*)(EncodedPointCloud&)>> changes = {
      {"voxel_leaf_mm", [](EncodedPointCloud& c) { c.voxel_leaf_mm = 100; }},
      {"color", [](EncodedPointCloud& c) { c.color = PointColor::kRgb565; }},
      {"position", [](EncodedPointCloud& c) { c.position = PointPosition::kFloat32; }},
      {"intensity", [](EncodedPointCloud& c) { c.intensity = PointIntensity::kUint8; }},
      {"encoding", [](EncodedPointCloud& c) { c.encoding = PointCloudEncoding::kPlanarZstd; }},
      {"zstd_level", [](EncodedPointCloud& c) { c.zstd_level = 1; }},
  };
  std::uint32_t seq = 2;
  std::uint32_t object_seq = 0;
  for (const auto& [field, apply] : changes) {
    const EncodedPointCloud previous = cloud;
    apply(cloud);
    const std::vector<CloudObjectRecord> out =
        lidar.Append(EntryAt(10 * seq, seq), "depth_frame", cloud, 1.0);
    ++seq;
    ASSERT_EQ(out.size(), 1u) << field;
    EXPECT_EQ(out[0].object_seq, object_seq) << field;
    EXPECT_EQ(out[0].blocks.size(), object_seq == 0 ? 2u : 1u) << field;
    EXPECT_EQ(out[0].voxel_leaf_mm, previous.voxel_leaf_mm) << field;
    EXPECT_EQ(out[0].color, previous.color) << field;
    EXPECT_EQ(out[0].position, previous.position) << field;
    EXPECT_EQ(out[0].intensity, previous.intensity) << field;
    EXPECT_EQ(out[0].encoding, previous.encoding) << field;
    EXPECT_EQ(out[0].zstd_level, previous.zstd_level) << field;
    ++object_seq;
  }

  const std::vector<CloudObjectRecord> moved =
      lidar.Append(EntryAt(10 * seq, seq), "other_frame", cloud, 1.0);
  ASSERT_EQ(moved.size(), 1u);
  EXPECT_EQ(moved[0].frame_id, "depth_frame");
  EXPECT_EQ(moved[0].object_seq, 6u);

  const std::vector<CloudObjectRecord> last = lidar.Close();
  ASSERT_EQ(last.size(), 1u);
  EXPECT_EQ(last[0].frame_id, "other_frame");
  EXPECT_EQ(last[0].object_seq, 7u);
  EXPECT_EQ(last[0].voxel_leaf_mm, 100u);
  EXPECT_EQ(last[0].color, PointColor::kRgb565);
  EXPECT_EQ(last[0].position, PointPosition::kFloat32);
  EXPECT_EQ(last[0].intensity, PointIntensity::kUint8);
  EXPECT_EQ(last[0].encoding, PointCloudEncoding::kPlanarZstd);
  EXPECT_EQ(last[0].zstd_level, 1u);
}

TEST(CloudObjectAssembler, IdleAndForceClose) {
  CloudObjectAssembler lidar("depth");
  EXPECT_TRUE(lidar.Close().empty());
  EXPECT_TRUE(lidar.CloseIfIdle(1000.0, 5.0).empty());

  const EncodedPointCloud cloud = CloudOf(100);
  EXPECT_TRUE(lidar.Append(EntryAt(0, 0), "depth_frame", cloud, 100.0).empty());
  EXPECT_TRUE(lidar.Append(EntryAt(100, 1), "depth_frame", cloud, 101.0).empty());
  // Idle counts from the last append, not the first.
  EXPECT_TRUE(lidar.CloseIfIdle(105.9, 5.0).empty());
  EXPECT_EQ(lidar.buffered_clouds(), 2u);
  const std::vector<CloudObjectRecord> idle = lidar.CloseIfIdle(106.0, 5.0);
  ASSERT_EQ(idle.size(), 1u);
  EXPECT_EQ(idle[0].blocks.size(), 2u);
  EXPECT_EQ(idle[0].object_seq, 0u);
  EXPECT_EQ(lidar.buffered_clouds(), 0u);
  EXPECT_EQ(lidar.buffered_bytes(), 0u);
  EXPECT_TRUE(lidar.CloseIfIdle(1000.0, 5.0).empty());

  EXPECT_TRUE(lidar.Append(EntryAt(200, 2), "depth_frame", cloud, 107.0).empty());
  EXPECT_TRUE(lidar.CloseIfIdle(107.4, 0.5).empty());
  const std::vector<CloudObjectRecord> shortened = lidar.CloseIfIdle(107.5, 0.5);
  ASSERT_EQ(shortened.size(), 1u);
  EXPECT_EQ(shortened[0].object_seq, 1u);

  EXPECT_TRUE(lidar.Append(EntryAt(300, 3), "depth_frame", cloud, 108.0).empty());
  const std::vector<CloudObjectRecord> forced = lidar.Close();
  ASSERT_EQ(forced.size(), 1u);
  EXPECT_EQ(forced[0].blocks.size(), 1u);
  EXPECT_EQ(forced[0].object_seq, 2u);
  EXPECT_EQ(lidar.buffered_clouds(), 0u);
  EXPECT_TRUE(lidar.Close().empty());
}
