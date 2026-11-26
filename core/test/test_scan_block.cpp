#include <gtest/gtest.h>
#include <zstd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "c3d/scan_block.hpp"

using namespace c3d;
using namespace c3d::scanblock;

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

LaserScanData ScanOf(std::size_t rays, float range_max = 12.0f) {
  LaserScanData scan;
  scan.angle_min = -1.0f;
  scan.angle_max = 1.0f;
  scan.angle_increment = 2.0f / static_cast<float>(rays);
  scan.range_min = 0.1f;
  scan.range_max = range_max;
  scan.ranges.assign(rays, 1.0f);
  return scan;
}

std::string Decompress(const std::string& compressed, std::size_t raw_bytes) {
  std::string raw(raw_bytes, '\0');
  const std::size_t n =
      ZSTD_decompress(raw.data(), raw.size(), compressed.data(), compressed.size());
  EXPECT_FALSE(ZSTD_isError(n));
  EXPECT_EQ(n, raw_bytes);
  return raw;
}

}  // namespace

TEST(ScanBlock, TheUnitIsAMillimetreUpTo65MetresAndACentimetreBeyond) {
  EXPECT_EQ(ChooseRangeUnit(12.0f), RangeUnit::kMillimetre);
  EXPECT_EQ(ChooseRangeUnit(65.0f), RangeUnit::kMillimetre);
  EXPECT_EQ(ChooseRangeUnit(65.01f), RangeUnit::kCentimetre);
  EXPECT_EQ(ChooseRangeUnit(100.0f), RangeUnit::kCentimetre);
  EXPECT_DOUBLE_EQ(UnitMetres(RangeUnit::kMillimetre), 0.001);
  EXPECT_DOUBLE_EQ(UnitMetres(RangeUnit::kCentimetre), 0.01);
}

TEST(ScanBlock, QuantiseKeepsTheSentinelsAndClipsTheRest) {
  const RangeUnit mm = RangeUnit::kMillimetre;
  EXPECT_EQ(QuantiseRange(1.2345f, mm), 1235);
  EXPECT_EQ(QuantiseRange(1.2344f, mm), 1234);
  EXPECT_EQ(QuantiseRange(kInf, mm), 0xFFFF);
  EXPECT_EQ(QuantiseRange(kNaN, mm), 0xFFFE);
  EXPECT_EQ(QuantiseRange(-kInf, mm), 0x0000);
  EXPECT_EQ(QuantiseRange(70.0f, mm), 0xFFFD);
  EXPECT_EQ(QuantiseRange(-0.5f, mm), 0);
  EXPECT_EQ(QuantiseRange(0.0f, mm), 0);
  EXPECT_EQ(QuantiseRange(1.2345f, RangeUnit::kCentimetre), 123);
}

TEST(ScanBlock, TheTimingFieldsAreNotPartOfTheGeometry) {
  LaserScanData a = ScanOf(10);
  LaserScanData b = a;
  b.time_increment = 0.001f;
  b.scan_time = 0.1f;
  EXPECT_TRUE(GeometryOf(a) == GeometryOf(b));
  EXPECT_EQ(GeometryOf(a).rays, 10u);
  b.angle_increment *= 2.0f;
  EXPECT_FALSE(GeometryOf(a) == GeometryOf(b));

  const ScanGeometry base = GeometryOf(a);
  for (int field = 0; field < 6; ++field) {
    LaserScanData c = a;
    switch (field) {
      case 0: c.angle_min += 0.5f; break;
      case 1: c.angle_max += 0.5f; break;
      case 2: c.angle_increment += 0.5f; break;
      case 3: c.range_min += 0.5f; break;
      case 4: c.range_max += 0.5f; break;
      case 5: c.ranges.push_back(1.0f); break;
    }
    EXPECT_FALSE(GeometryOf(c) == base) << "field " << field;
  }
}

TEST(ScanBlock, DequantiseInvertsQuantiseWithinHalfAUnit) {
  for (RangeUnit unit : {RangeUnit::kMillimetre, RangeUnit::kCentimetre}) {
    const double half = UnitMetres(unit) / 2.0 * 1.0001;
    for (float metres : {0.5f, 1.2345f, 7.777f, 30.0f, 64.9f}) {
      const float back = DequantiseRange(QuantiseRange(metres, unit), unit);
      EXPECT_LE(std::fabs(static_cast<double>(back) - metres), half) << metres;
    }
    EXPECT_EQ(DequantiseRange(kCodePosInf, unit), kInf);
    EXPECT_EQ(DequantiseRange(kCodeNegInf, unit), -kInf);
    EXPECT_TRUE(std::isnan(DequantiseRange(kCodeNaN, unit)));
  }
}

TEST(ScanBlock, DeltaShuffleOfAHandWorkedBlockIsByteExact) {
  const std::vector<std::uint16_t> codes = {10, 12, 11, 5, 5, 0xFFFF};
  const std::string expected("\x0A\x02\xFF\x05\x00\xFA\x00\x00\xFF\x00\x00\xFF", 12);
  EXPECT_EQ(DeltaShuffle(codes, 3), expected);
  EXPECT_EQ(InverseDeltaShuffle(expected, 3), codes);
}

TEST(ScanBlock, InverseDeltaShuffleRoundTripsRandomCodes) {
  std::mt19937 rng(12345);
  std::vector<std::uint16_t> codes(7 * 640);
  for (auto& c : codes) c = static_cast<std::uint16_t>(rng());
  const std::string bytes = DeltaShuffle(codes, 640);
  ASSERT_EQ(bytes.size(), codes.size() * 2);
  EXPECT_EQ(InverseDeltaShuffle(bytes, 640), codes);
  EXPECT_TRUE(InverseDeltaShuffle(std::string(7, 'x'), 640).empty());
  EXPECT_TRUE(InverseDeltaShuffle(std::string(2 * 640 + 2, 'x'), 640).empty());
}

TEST(ScanBlock, EncodeRoundTripsThroughZstdAndTheInverse) {
  std::vector<LaserScanData> scans;
  for (int s = 0; s < 5; ++s) {
    LaserScanData scan = ScanOf(64);
    scan.intensities.assign(64, 0.0f);
    for (int i = 0; i < 64; ++i) {
      scan.ranges[i] = 0.5f + 0.137f * static_cast<float>(i) + 0.01f * static_cast<float>(s);
      scan.intensities[i] = static_cast<float>(i * 7 + s);
    }
    scan.ranges[3] = kInf;
    scan.ranges[4] = kNaN;
    scan.ranges[5] = -kInf;
    scan.intensities[6] = 70000.0f;
    scans.push_back(scan);
  }
  const EncodedScanBlock block = Encode(scans, true, 3);
  ASSERT_TRUE(block.ok) << block.reason;
  EXPECT_EQ(block.scans, 5u);
  EXPECT_EQ(block.unit, RangeUnit::kMillimetre);
  EXPECT_EQ(block.raw_bytes, 5u * 64 * 2 * 2);

  const std::size_t plane = 5u * 64 * 2;
  const std::vector<std::uint16_t> ranges =
      InverseDeltaShuffle(Decompress(block.ranges, plane), 64);
  const std::vector<std::uint16_t> intensities =
      InverseDeltaShuffle(Decompress(block.intensities, plane), 64);
  ASSERT_EQ(ranges.size(), 5u * 64);
  ASSERT_EQ(intensities.size(), 5u * 64);
  for (std::size_t s = 0; s < 5; ++s) {
    for (std::size_t i = 0; i < 64; ++i) {
      const float want = scans[s].ranges[i];
      const float got = DequantiseRange(ranges[s * 64 + i], block.unit);
      if (std::isnan(want)) {
        EXPECT_TRUE(std::isnan(got));
      } else if (std::isinf(want)) {
        EXPECT_EQ(got, want);
      } else {
        EXPECT_LE(std::fabs(got - want), 0.0005 * 1.001 + 1e-6) << s << "," << i;
      }
      const float want_i = std::min(scans[s].intensities[i], 65535.0f);
      EXPECT_EQ(intensities[s * 64 + i], static_cast<std::uint16_t>(want_i));
    }
  }

  const EncodedScanBlock without = Encode(scans, false, 3);
  ASSERT_TRUE(without.ok);
  EXPECT_TRUE(without.intensities.empty());
  EXPECT_EQ(without.raw_bytes, 5u * 64 * 2);
}

TEST(ScanBlock, EncodeUsesACentimetreUnitWhenTheFirstScanReachesBeyond65Metres) {
  LaserScanData first = ScanOf(32, 100.0f);
  LaserScanData second = ScanOf(32, 12.0f);
  for (int i = 0; i < 32; ++i) {
    first.ranges[i] = 70.0f + 0.9371f * static_cast<float>(i);
    second.ranges[i] = 1.0f + 2.7313f * static_cast<float>(i);
  }
  first.ranges[0] = kInf;
  first.ranges[1] = kNaN;
  first.ranges[2] = -kInf;
  const std::vector<LaserScanData> scans = {first, second};
  const EncodedScanBlock block = Encode(scans, false, 3);
  ASSERT_TRUE(block.ok) << block.reason;
  EXPECT_EQ(block.unit, RangeUnit::kCentimetre);

  const std::vector<std::uint16_t> codes =
      InverseDeltaShuffle(Decompress(block.ranges, 2u * 32 * 2), 32);
  ASSERT_EQ(codes.size(), 2u * 32);
  for (std::size_t s = 0; s < 2; ++s) {
    for (std::size_t i = 0; i < 32; ++i) {
      const float want = scans[s].ranges[i];
      const float got = DequantiseRange(codes[s * 32 + i], block.unit);
      if (std::isnan(want)) {
        EXPECT_TRUE(std::isnan(got));
      } else if (std::isinf(want)) {
        EXPECT_EQ(got, want);
      } else {
        EXPECT_LE(std::fabs(got - want), 0.005f * 1.001f) << s << "," << i;
      }
    }
  }
}

TEST(ScanBlock, EncodeRefusesWhatItCannotLayOut) {
  EXPECT_EQ(Encode({}, false, 3).reason, "empty");
  EXPECT_FALSE(Encode({}, false, 3).ok);

  EXPECT_EQ(Encode({ScanOf(640), ScanOf(639)}, false, 3).reason, "rays_differ");
  EXPECT_EQ(Encode({ScanOf(0)}, false, 3).reason, "zero_rays");

  EXPECT_EQ(Encode({ScanOf(65536)}, false, 3).reason, "rays_over_uint16");
  const EncodedScanBlock biggest = Encode({ScanOf(65535)}, false, 3);
  EXPECT_TRUE(biggest.ok) << biggest.reason;

  LaserScanData with = ScanOf(640);
  with.intensities.assign(640, 1.0f);
  LaserScanData none = ScanOf(640);
  LaserScanData short_one = ScanOf(640);
  short_one.intensities.assign(639, 1.0f);
  EXPECT_EQ(Encode({with, none}, true, 3).reason, "intensities_differ");
  EXPECT_EQ(Encode({with, short_one}, true, 3).reason, "intensities_differ");
  EXPECT_TRUE(Encode({with, with}, true, 3).ok);
  EXPECT_TRUE(Encode({with, none}, false, 3).ok);
}

TEST(ScanBlock, ALevelBelowOneIsEncodedAtLevelOne) {
  std::mt19937 rng(7);
  std::vector<LaserScanData> scans;
  for (int s = 0; s < 20; ++s) {
    LaserScanData scan = ScanOf(640);
    for (auto& r : scan.ranges) r = 0.5f + static_cast<float>(rng() % 4000) * 0.003f;
    scans.push_back(scan);
  }
  const EncodedScanBlock one = Encode(scans, false, 1);
  ASSERT_TRUE(one.ok);
  for (int level : {0, -5}) {
    const EncodedScanBlock low = Encode(scans, false, level);
    ASSERT_TRUE(low.ok) << level;
    EXPECT_EQ(low.ranges, one.ranges) << level;
  }
  EXPECT_TRUE(Encode(scans, false, 100000).ok);
}

TEST(ScanBlock, TheIntensityCodeIsClampedAndNonFiniteIsZero) {
  EXPECT_EQ(QuantiseIntensity(0.0f), 0);
  EXPECT_EQ(QuantiseIntensity(-5.0f), 0);
  EXPECT_EQ(QuantiseIntensity(200.4f), 200);
  EXPECT_EQ(QuantiseIntensity(200.6f), 201);
  EXPECT_EQ(QuantiseIntensity(65535.0f), 65535);
  EXPECT_EQ(QuantiseIntensity(1e9f), 65535);
  EXPECT_EQ(QuantiseIntensity(kInf), 0);
  EXPECT_EQ(QuantiseIntensity(-kInf), 0);
  EXPECT_EQ(QuantiseIntensity(kNaN), 0);
}
