#include <gtest/gtest.h>
#include <zstd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "c3d/media.hpp"

using namespace c3d;

namespace {

PointCloudOptions RawOptions() {
  // The encoder's own defaults downsample and compress, which is right for a
  // robot and wrong for a test that wants to see the bytes it wrote. Every test
  // below starts from "nothing applied" and turns on exactly what it is testing.
  PointCloudOptions options;
  options.voxel_leaf_m = 0.0;
  options.zstd_level = 0;
  return options;
}

std::int16_t ReadI16(const std::string& block, std::size_t index) {
  const std::uint16_t low = static_cast<unsigned char>(block[index * 2]);
  const std::uint16_t high = static_cast<unsigned char>(block[index * 2 + 1]);
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(low | (high << 8)));
}

float ReadF32(const std::string& block, std::size_t byte_offset) {
  std::uint32_t bits = static_cast<unsigned char>(block[byte_offset]) |
                       (static_cast<std::uint32_t>(static_cast<unsigned char>(block[byte_offset + 1])) << 8) |
                       (static_cast<std::uint32_t>(static_cast<unsigned char>(block[byte_offset + 2])) << 16) |
                       (static_cast<std::uint32_t>(static_cast<unsigned char>(block[byte_offset + 3])) << 24);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::string Decompress(const EncodedPointCloud& cloud) {
  if (cloud.zstd_level == 0) return cloud.data;
  const unsigned long long size = ZSTD_getFrameContentSize(cloud.data.data(), cloud.data.size());
  EXPECT_NE(size, ZSTD_CONTENTSIZE_ERROR);
  EXPECT_NE(size, ZSTD_CONTENTSIZE_UNKNOWN);
  std::string out;
  out.resize(static_cast<std::size_t>(size));
  const std::size_t written =
      ZSTD_decompress(&out[0], out.size(), cloud.data.data(), cloud.data.size());
  EXPECT_FALSE(ZSTD_isError(written));
  out.resize(written);
  return out;
}

// The rgb565 and rgb332 decoders exactly as docs/data-format.md writes them. Written out
// here rather than shared with the encoder on purpose: a test that decodes with
// the encoder's own helper proves the two agree with each other and nothing about
// whether either agrees with the gateway.
void DecodeRgb565(std::uint16_t v, int* r, int* g, int* b) {
  *r = ((v >> 11) << 3) | (v >> 13);
  *g = (((v >> 5) & 63) << 2) | ((v >> 9) & 3);
  *b = ((v & 31) << 3) | ((v >> 2) & 7);
}

void DecodeRgb332(std::uint8_t v, int* r, int* g, int* b) {
  *r = (v & 0xE0) | ((v & 0xE0) >> 3) | ((v & 0xE0) >> 6);
  *g = ((v << 3) & 0xE0) | (v & 0x1C) | ((v >> 3) & 0x03);
  *b = ((v & 0x03) << 6) | ((v & 0x03) << 4) | ((v & 0x03) << 2) | (v & 0x03);
}

}  // namespace

TEST(PointCloudEncoder, VoxelCellCollapsesToItsCentroid) {
  PointCloudInput input;
  // Three points inside one 10 cm cell and one well outside it.
  input.xyz = {0.01f, 0.02f, 0.03f,
               0.03f, 0.04f, 0.05f,
               0.05f, 0.06f, 0.07f,
               1.00f, 1.00f, 1.00f};
  PointCloudOptions options = RawOptions();
  options.voxel_leaf_m = 0.1;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  EXPECT_EQ(cloud.point_count, 2u);
  EXPECT_EQ(cloud.voxel_leaf_mm, 100u);

  const std::string block = Decompress(cloud);
  // Planes are X[N] Y[N] Z[N]; the cell at the origin sorts first.
  EXPECT_EQ(ReadI16(block, 0), 30);   // mean of 0.01, 0.03, 0.05
  EXPECT_EQ(ReadI16(block, 2), 40);   // mean of 0.02, 0.04, 0.06 -- Y plane starts at index 2
  EXPECT_EQ(ReadI16(block, 4), 50);   // Z plane
  EXPECT_EQ(ReadI16(block, 1), 1000);
}

TEST(PointCloudEncoder, ACloudOverTheCapIsDroppedWholeAndSaysWhy) {
  PointCloudInput input;
  for (int i = 0; i < 50; ++i) {
    input.xyz.push_back(static_cast<float>(i));
    input.xyz.push_back(0.0f);
    input.xyz.push_back(0.0f);
  }
  PointCloudOptions options = RawOptions();
  options.max_points = 49;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  EXPECT_FALSE(cloud.ok);
  EXPECT_EQ(cloud.reason, "oversize");
  EXPECT_EQ(cloud.point_count, 0u);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(PointCloudEncoder, TheCapIsAppliedAfterDownsamplingNotBefore) {
  // 60 points that collapse to 3 voxels. A cap of 10 rejects the cloud if it is
  // applied to the input and accepts it if it is applied to the output, which is
  // the whole reason the order is stated in the algorithm.
  PointCloudInput input;
  for (int i = 0; i < 60; ++i) {
    const float base = static_cast<float>(i % 3);
    input.xyz.push_back(base + 0.001f * static_cast<float>(i));
    input.xyz.push_back(0.0f);
    input.xyz.push_back(0.0f);
  }
  PointCloudOptions options = RawOptions();
  options.voxel_leaf_m = 1.0;
  options.max_points = 10;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  EXPECT_EQ(cloud.point_count, 3u);
}

TEST(PointCloudEncoder, ShuffledInputProducesIdenticalBytes) {
  std::vector<std::array<float, 3>> points;
  for (int i = 0; i < 200; ++i) {
    points.push_back({static_cast<float>(i % 7) * 0.31f,
                      static_cast<float>((i * 13) % 11) * 0.27f,
                      static_cast<float>((i * 7) % 5) * 0.19f});
  }

  auto encode = [](const std::vector<std::array<float, 3>>& ordered, double leaf) {
    PointCloudInput input;
    for (const auto& p : ordered) {
      input.xyz.push_back(p[0]);
      input.xyz.push_back(p[1]);
      input.xyz.push_back(p[2]);
    }
    PointCloudOptions options = RawOptions();
    options.voxel_leaf_m = leaf;
    return pointcloud::Encode(input, options);
  };

  const EncodedPointCloud straight = encode(points, 0.05);
  const EncodedPointCloud straight_raw = encode(points, 0.0);
  std::mt19937 rng(12345);
  std::shuffle(points.begin(), points.end(), rng);
  const EncodedPointCloud shuffled = encode(points, 0.05);
  const EncodedPointCloud shuffled_raw = encode(points, 0.0);

  ASSERT_TRUE(straight.ok);
  EXPECT_EQ(straight.data, shuffled.data);
  EXPECT_EQ(straight.point_count, shuffled.point_count);
  // And with no downsampling at all, where two points can share a sort key and
  // only the value tiebreak keeps the order total.
  ASSERT_TRUE(straight_raw.ok);
  EXPECT_EQ(straight_raw.data, shuffled_raw.data);
}

TEST(PointCloudEncoder, NonFiniteCoordinatesAreFilteredAndCounted) {
  PointCloudInput input;
  input.xyz = {0.0f, 0.0f, 0.0f,
               std::numeric_limits<float>::quiet_NaN(), 1.0f, 1.0f,
               2.0f, std::numeric_limits<float>::infinity(), 2.0f,
               3.0f, 3.0f, 3.0f};
  const EncodedPointCloud cloud = pointcloud::Encode(input, RawOptions());
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  EXPECT_EQ(cloud.point_count, 2u);
  EXPECT_EQ(cloud.dropped_non_finite, 2u);
}

TEST(PointCloudEncoder, Int16DropsWhatItCannotCarryAndClampsNothingIntoRange) {
  PointCloudInput input;
  input.xyz = {1.0f, 2.0f, 3.0f,
               40.0f, 0.0f, 0.0f,     // 40 m: past +/-32.767
               0.0f, -33.0f, 0.0f,    // and on another axis, in the other direction
               32.7f, 0.0f, 0.0f};
  const EncodedPointCloud cloud = pointcloud::Encode(input, RawOptions());
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  EXPECT_EQ(cloud.point_count, 2u);
  EXPECT_EQ(cloud.dropped_out_of_range, 2u);

  // The survivors are the two in range, and neither was rewritten to the limit:
  // a clamped point would be a measurement the sensor never made. They come out
  // in (iz, iy, ix) order, so the one at z = 0 is first whatever order it arrived.
  const std::string block = Decompress(cloud);
  EXPECT_EQ(ReadI16(block, 0), 32700);
  EXPECT_EQ(ReadI16(block, 1), 1000);
}

TEST(PointCloudEncoder, PlanarLayoutOfThreePointsIsByteExact) {
  PointCloudInput input;
  // Already sorted by (iz, iy, ix) so the expected bytes can be written out.
  input.xyz = {0.001f, 0.002f, 0.003f,
               0.004f, 0.005f, 0.006f,
               0.007f, 0.008f, 0.009f};
  input.rgb = {255, 0, 0, 0, 255, 0, 0, 0, 255};
  input.intensity = {10.0f, 20.0f, 30.0f};
  input.intensity_is_integral = true;
  PointCloudOptions options = RawOptions();
  options.color = PointColor::kRgb888;
  options.intensity = PointIntensity::kUint8;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  ASSERT_EQ(cloud.point_count, 3u);

  const std::string block = Decompress(cloud);
  const std::string expected(
      "\x01\x00\x04\x00\x07\x00"          // X[3], int16 mm, little-endian
      "\x02\x00\x05\x00\x08\x00"          // Y[3]
      "\x03\x00\x06\x00\x09\x00"          // Z[3]
      "\xff\x00\x00\x00\xff\x00\x00\x00\xff"  // C[3], rgb888
      "\x0a\x14\x1e",                     // I[3], uint8
      3 * 2 * 3 + 9 + 3);
  EXPECT_EQ(block, expected);
  EXPECT_EQ(cloud.raw_bytes, expected.size());
}

TEST(PointCloudEncoder, Float32PositionsAreFourBytesPerPlaneEntry) {
  PointCloudInput input;
  input.xyz = {1.5f, 2.5f, 3.5f};
  PointCloudOptions options = RawOptions();
  options.position = PointPosition::kFloat32;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  const std::string block = Decompress(cloud);
  ASSERT_EQ(block.size(), 12u);
  EXPECT_FLOAT_EQ(ReadF32(block, 0), 1.5f);
  EXPECT_FLOAT_EQ(ReadF32(block, 4), 2.5f);
  EXPECT_FLOAT_EQ(ReadF32(block, 8), 3.5f);
  // float32 carries what int16_mm would have dropped, which is the only reason
  // the format exists.
  EXPECT_EQ(cloud.dropped_out_of_range, 0u);
}

TEST(PointCloudEncoder, Rgb565SurvivesARoundTripWithinItsQuantisationStep) {
  PointCloudInput input;
  std::vector<std::uint8_t> colours;
  for (int v = 0; v < 256; v += 17) {
    input.xyz.push_back(static_cast<float>(v) * 0.01f);
    input.xyz.push_back(0.0f);
    input.xyz.push_back(0.0f);
    colours.push_back(static_cast<std::uint8_t>(v));
    colours.push_back(static_cast<std::uint8_t>(255 - v));
    colours.push_back(static_cast<std::uint8_t>((v * 3) % 256));
  }
  input.rgb = colours;
  PointCloudOptions options = RawOptions();
  options.color = PointColor::kRgb565;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  const std::string block = Decompress(cloud);
  const std::size_t n = cloud.point_count;
  const std::size_t colour_offset = n * 3 * 2;
  ASSERT_EQ(block.size(), colour_offset + n * 2);

  for (std::size_t i = 0; i < n; ++i) {
    const std::uint16_t packed =
        static_cast<unsigned char>(block[colour_offset + i * 2]) |
        static_cast<std::uint16_t>(static_cast<unsigned char>(block[colour_offset + i * 2 + 1]) << 8);
    int r = 0, g = 0, b = 0;
    DecodeRgb565(packed, &r, &g, &b);
    // 5 bits on red and blue, 6 on green: the step is 8 and 4 respectively.
    EXPECT_LE(std::abs(r - colours[i * 3 + 0]), 7);
    EXPECT_LE(std::abs(g - colours[i * 3 + 1]), 3);
    EXPECT_LE(std::abs(b - colours[i * 3 + 2]), 7);
  }
}

TEST(PointCloudEncoder, Rgb332SurvivesARoundTripAndKeepsBlackAndWhiteExact) {
  PointCloudInput input;
  input.xyz = {0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f};
  input.rgb = {0, 0, 0, 255, 255, 255, 130, 200, 90};
  PointCloudOptions options = RawOptions();
  options.color = PointColor::kRgb332;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  const std::string block = Decompress(cloud);
  const std::size_t colour_offset = 3 * 3 * 2;
  ASSERT_EQ(block.size(), colour_offset + 3);

  int r = 0, g = 0, b = 0;
  DecodeRgb332(static_cast<std::uint8_t>(block[colour_offset]), &r, &g, &b);
  EXPECT_EQ(r, 0);
  EXPECT_EQ(g, 0);
  EXPECT_EQ(b, 0);
  DecodeRgb332(static_cast<std::uint8_t>(block[colour_offset + 1]), &r, &g, &b);
  EXPECT_EQ(r, 255);
  EXPECT_EQ(g, 255);
  EXPECT_EQ(b, 255);
  DecodeRgb332(static_cast<std::uint8_t>(block[colour_offset + 2]), &r, &g, &b);
  EXPECT_LE(std::abs(r - 130), 31);
  EXPECT_LE(std::abs(g - 200), 31);
  EXPECT_LE(std::abs(b - 90), 63);
}

TEST(PointCloudEncoder, UnitScaleFloatIntensityIsStretchedAndIntegralIsNot) {
  PointCloudInput unit;
  unit.xyz = {0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f};
  unit.intensity = {0.5f, 1.0f};
  unit.intensity_is_integral = false;
  PointCloudOptions options = RawOptions();
  options.intensity = PointIntensity::kUint8;

  const EncodedPointCloud scaled = pointcloud::Encode(unit, options);
  ASSERT_TRUE(scaled.ok) << scaled.reason;
  std::string block = Decompress(scaled);
  EXPECT_EQ(static_cast<unsigned char>(block[block.size() - 2]), 128);
  EXPECT_EQ(static_cast<unsigned char>(block[block.size() - 1]), 255);

  // The SAME values from an integer-typed field are a nearly black cloud, and
  // scaling them would invent brightness that is not there.
  PointCloudInput integral = unit;
  integral.intensity_is_integral = true;
  const EncodedPointCloud unscaled = pointcloud::Encode(integral, options);
  ASSERT_TRUE(unscaled.ok) << unscaled.reason;
  block = Decompress(unscaled);
  EXPECT_EQ(static_cast<unsigned char>(block[block.size() - 2]), 1);  // round(0.5) == 1
  EXPECT_EQ(static_cast<unsigned char>(block[block.size() - 1]), 1);
}

TEST(PointCloudEncoder, AFloatIntensityAbove1IsTakenAsAlready0To255) {
  PointCloudInput input;
  input.xyz = {0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f};
  input.intensity = {12.0f, 400.0f};
  input.intensity_is_integral = false;
  PointCloudOptions options = RawOptions();
  options.intensity = PointIntensity::kUint8;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  const std::string block = Decompress(cloud);
  EXPECT_EQ(static_cast<unsigned char>(block[block.size() - 2]), 12);
  EXPECT_EQ(static_cast<unsigned char>(block[block.size() - 1]), 255);
}

TEST(PointCloudEncoder, AnAttributeTheCallerDidNotSupplyIsReportedAsAbsent) {
  PointCloudInput input;
  input.xyz = {0.0f, 0.0f, 0.0f};
  PointCloudOptions options = RawOptions();
  options.color = PointColor::kRgb565;
  options.intensity = PointIntensity::kUint8;

  const EncodedPointCloud cloud = pointcloud::Encode(input, options);
  ASSERT_TRUE(cloud.ok) << cloud.reason;
  EXPECT_EQ(cloud.color, PointColor::kNone);
  EXPECT_EQ(cloud.intensity, PointIntensity::kNone);
  // Positions only: no colour plane, no intensity plane.
  EXPECT_EQ(Decompress(cloud).size(), 6u);
}

TEST(PointCloudEncoder, ZstdRoundTripsAndLevelZeroIsThePlainBlock) {
  PointCloudInput input;
  for (int i = 0; i < 500; ++i) {
    input.xyz.push_back(static_cast<float>(i) * 0.01f);
    input.xyz.push_back(0.0f);
    input.xyz.push_back(0.0f);
  }
  PointCloudOptions raw = RawOptions();
  const EncodedPointCloud plain = pointcloud::Encode(input, raw);
  ASSERT_TRUE(plain.ok);
  EXPECT_EQ(plain.zstd_level, 0u);
  EXPECT_EQ(plain.data.size(), plain.raw_bytes);

  // The encoder's own defaults, every point kept so the block compares with the plain one: zstd
  // level 3 over delta-coded, byte-shuffled position planes.
  PointCloudOptions defaults;
  defaults.voxel_leaf_m = 0.0;
  const EncodedPointCloud compressed = pointcloud::Encode(input, defaults);
  ASSERT_TRUE(compressed.ok);
  EXPECT_EQ(compressed.zstd_level, 3u);
  EXPECT_EQ(compressed.encoding, PointCloudEncoding::kPlanarDeltaShuffleZstd);
  EXPECT_LT(compressed.data.size(), plain.data.size());
  EXPECT_EQ(pointcloud::InverseDeltaShufflePositionPlanes(Decompress(compressed),
                                                          compressed.point_count, 2),
            plain.data);
}

TEST(PointCloudEncoder, WireSpellingsAreTheLowercaseConstantsTheGatewayValidates) {
  EXPECT_STREQ(PointPositionName(PointPosition::kInt16Mm), "int16_mm");
  EXPECT_STREQ(PointPositionName(PointPosition::kFloat32), "float32");
  EXPECT_STREQ(PointColorName(PointColor::kNone), "none");
  EXPECT_STREQ(PointColorName(PointColor::kRgb565), "rgb565");
  EXPECT_STREQ(PointColorName(PointColor::kRgb332), "rgb332");
  EXPECT_STREQ(PointColorName(PointColor::kRgb888), "rgb888");
  EXPECT_STREQ(PointIntensityName(PointIntensity::kNone), "none");
  EXPECT_STREQ(PointIntensityName(PointIntensity::kUint8), "uint8");

  PointColor colour = PointColor::kRgb888;
  EXPECT_FALSE(ParsePointColor("RGB565", &colour));
  EXPECT_FALSE(ParsePointColor("rgb5650", &colour));
  EXPECT_TRUE(ParsePointColor("rgb565", &colour));
  EXPECT_EQ(colour, PointColor::kRgb565);
}

TEST(PointCloudEncoder, AnEmptyCloudIsRefusedRatherThanEncodedAsZeroPoints) {
  EXPECT_EQ(pointcloud::Encode(PointCloudInput{}, RawOptions()).reason, "empty");

  PointCloudInput all_nan;
  all_nan.xyz = {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f};
  EXPECT_EQ(pointcloud::Encode(all_nan, RawOptions()).reason, "no_finite_points");

  PointCloudInput all_far;
  all_far.xyz = {5000.0f, 0.0f, 0.0f};
  EXPECT_EQ(pointcloud::Encode(all_far, RawOptions()).reason, "no_points_in_range");
}

namespace {

PointCloudInput ThreePointFixture() {
  PointCloudInput input;
  input.xyz = {0.001f, 0.002f, 0.003f,
               0.004f, 0.005f, 0.006f,
               0.007f, 0.008f, 0.009f};
  input.rgb = {255, 0, 0, 0, 255, 0, 0, 0, 255};
  input.intensity = {10.0f, 20.0f, 30.0f};
  input.intensity_is_integral = true;
  return input;
}

PointCloudInput RandomCloud(unsigned seed, int points) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> position(-30.0f, 30.0f);
  std::uniform_int_distribution<int> byte(0, 255);
  PointCloudInput input;
  for (int i = 0; i < points; ++i) {
    for (int axis = 0; axis < 3; ++axis) input.xyz.push_back(position(rng));
    for (int channel = 0; channel < 3; ++channel) {
      input.rgb.push_back(static_cast<std::uint8_t>(byte(rng)));
    }
    input.intensity.push_back(static_cast<float>(byte(rng)));
  }
  input.intensity_is_integral = true;
  return input;
}

}  // namespace

TEST(PointCloudEncoder, DeltaShuffleOfThreePointsIsByteExact) {
  PointCloudOptions options = RawOptions();
  options.color = PointColor::kRgb888;
  options.intensity = PointIntensity::kUint8;
  const EncodedPointCloud plain = pointcloud::Encode(ThreePointFixture(), options);
  ASSERT_TRUE(plain.ok) << plain.reason;

  const std::string transformed = pointcloud::DeltaShufflePositionPlanes(plain.data, 3, 2);
  const std::string expected(
      "\x01\x03\x03\x00\x00\x00"          // X deltas 1,3,3: low bytes, then high bytes
      "\x02\x03\x03\x00\x00\x00"          // Y deltas 2,3,3
      "\x03\x03\x03\x00\x00\x00"          // Z deltas 3,3,3
      "\xff\x00\x00\x00\xff\x00\x00\x00\xff"  // C[3], rgb888, verbatim
      "\x0a\x14\x1e",                     // I[3], uint8, verbatim
      3 * 2 * 3 + 9 + 3);
  EXPECT_EQ(transformed, expected);
  EXPECT_EQ(pointcloud::InverseDeltaShufflePositionPlanes(transformed, 3, 2), plain.data);
}

TEST(PointCloudEncoder, DeltaShuffleRoundTripsAndLeavesTheAttributePlanesAlone) {
  PointCloudOptions options = RawOptions();
  options.color = PointColor::kRgb565;
  options.intensity = PointIntensity::kUint8;
  const EncodedPointCloud plain = pointcloud::Encode(RandomCloud(7, 1000), options);
  ASSERT_TRUE(plain.ok) << plain.reason;
  const std::size_t n = plain.point_count;
  ASSERT_GT(n, 0u);

  const std::string transformed = pointcloud::DeltaShufflePositionPlanes(plain.data, n, 2);
  ASSERT_EQ(transformed.size(), plain.data.size());
  EXPECT_NE(transformed, plain.data);
  const std::size_t positions = n * 2 * 3;
  EXPECT_EQ(transformed.substr(positions), plain.data.substr(positions));
  EXPECT_EQ(pointcloud::InverseDeltaShufflePositionPlanes(transformed, n, 2), plain.data);
}

TEST(PointCloudEncoder, TheResultSaysWhichEncodingWasWritten) {
  const PointCloudInput input = RandomCloud(11, 300);

  PointCloudOptions shuffled = RawOptions();
  shuffled.encoding = PointCloudEncoding::kPlanarDeltaShuffleZstd;
  shuffled.zstd_level = 3;
  const EncodedPointCloud delta = pointcloud::Encode(input, shuffled);
  ASSERT_TRUE(delta.ok);
  EXPECT_EQ(delta.encoding, PointCloudEncoding::kPlanarDeltaShuffleZstd);

  const std::string plain_block = pointcloud::Encode(input, RawOptions()).data;

  PointCloudOptions raw = shuffled;
  raw.zstd_level = 0;
  const EncodedPointCloud raw_cloud = pointcloud::Encode(input, raw);
  ASSERT_TRUE(raw_cloud.ok);
  EXPECT_EQ(raw_cloud.encoding, PointCloudEncoding::kPlanarZstd);
  EXPECT_EQ(raw_cloud.zstd_level, 0u);
  EXPECT_EQ(raw_cloud.data, plain_block);

  PointCloudOptions floats = shuffled;
  floats.position = PointPosition::kFloat32;
  const EncodedPointCloud float_cloud = pointcloud::Encode(input, floats);
  ASSERT_TRUE(float_cloud.ok);
  EXPECT_EQ(float_cloud.encoding, PointCloudEncoding::kPlanarZstd);
  PointCloudOptions float_raw = RawOptions();
  float_raw.position = PointPosition::kFloat32;
  EXPECT_EQ(Decompress(float_cloud), pointcloud::Encode(input, float_raw).data);

  PointCloudOptions plain_zstd = shuffled;
  plain_zstd.encoding = PointCloudEncoding::kPlanarZstd;
  const EncodedPointCloud plain = pointcloud::Encode(input, plain_zstd);
  ASSERT_TRUE(plain.ok);
  EXPECT_EQ(plain.encoding, PointCloudEncoding::kPlanarZstd);
  std::string expected_zstd(ZSTD_compressBound(plain_block.size()), '\0');
  const std::size_t expected_size = ZSTD_compress(&expected_zstd[0], expected_zstd.size(),
                                                  plain_block.data(), plain_block.size(), 3);
  ASSERT_FALSE(ZSTD_isError(expected_size));
  expected_zstd.resize(expected_size);
  EXPECT_EQ(plain.data, expected_zstd);
}

// What the session property states is what Encode writes, for every combination the encoding can
// be asked for in.
TEST(PointCloudEncoder, TheEffectiveEncodingIsTheOneEncodeWrites) {
  const PointCloudInput input = RandomCloud(13, 200);
  for (const PointCloudEncoding asked :
       {PointCloudEncoding::kPlanarDeltaShuffleZstd, PointCloudEncoding::kPlanarZstd}) {
    for (const PointPosition position : {PointPosition::kInt16Mm, PointPosition::kFloat32}) {
      for (const int level : {0, 3}) {
        SCOPED_TRACE(std::string(PointCloudEncodingName(asked)) + " " +
                     PointPositionName(position) + " level " + std::to_string(level));
        PointCloudOptions options = RawOptions();
        options.encoding = asked;
        options.position = position;
        options.zstd_level = level;
        const bool delta = asked == PointCloudEncoding::kPlanarDeltaShuffleZstd &&
                           position == PointPosition::kInt16Mm && level > 0;
        const PointCloudEncoding expected =
            delta ? PointCloudEncoding::kPlanarDeltaShuffleZstd : PointCloudEncoding::kPlanarZstd;
        EXPECT_EQ(pointcloud::EffectiveEncoding(options), expected);
        const EncodedPointCloud encoded = pointcloud::Encode(input, options);
        ASSERT_TRUE(encoded.ok) << encoded.reason;
        EXPECT_EQ(encoded.encoding, expected);
      }
    }
  }
}

TEST(PointCloudEncoder, DeltaShuffledOutputDecodesToTheSamePointsAsThePlainBlock) {
  const PointCloudInput input = RandomCloud(23, 2000);
  PointCloudOptions options = RawOptions();
  options.color = PointColor::kRgb565;
  options.intensity = PointIntensity::kUint8;
  const EncodedPointCloud plain = pointcloud::Encode(input, options);
  ASSERT_TRUE(plain.ok);

  options.encoding = PointCloudEncoding::kPlanarDeltaShuffleZstd;
  options.zstd_level = 3;
  const EncodedPointCloud delta = pointcloud::Encode(input, options);
  ASSERT_TRUE(delta.ok);
  ASSERT_EQ(delta.encoding, PointCloudEncoding::kPlanarDeltaShuffleZstd);
  EXPECT_EQ(delta.point_count, plain.point_count);
  EXPECT_EQ(delta.raw_bytes, plain.raw_bytes);

  const std::string block = Decompress(delta);
  EXPECT_EQ(pointcloud::InverseDeltaShufflePositionPlanes(block, delta.point_count, 2), plain.data);
}

TEST(PointCloudEncoder, EncodingNamesAreTheSchemaEnumerants) {
  EXPECT_STREQ(PointCloudEncodingName(PointCloudEncoding::kPlanarZstd), "planarZstd");
  EXPECT_STREQ(PointCloudEncodingName(PointCloudEncoding::kPlanarDeltaShuffleZstd),
               "planarDeltaShuffleZstd");
}
