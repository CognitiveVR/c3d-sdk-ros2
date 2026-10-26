#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "c3d/media.hpp"

namespace c3d::scanblock {

// The schema's ordinals.
enum class RangeUnit : std::uint8_t { kMillimetre = 0, kCentimetre = 1 };

// Codes 0xFFFF, 0xFFFE and 0x0000 are the three non-finite returns; finite ranges use 0..kCodeMax
// after clipping, and a finite range that rounds to 0 is therefore indistinguishable from -inf.
constexpr std::uint16_t kCodePosInf = 0xFFFF, kCodeNaN = 0xFFFE, kCodeNegInf = 0x0000,
                        kCodeMax = 0xFFFD;
constexpr float kMillimetreUnitMaxRangeMetres = 65.0f;
// The schema's `rays` is UInt16.
constexpr std::size_t kMaxRaysPerScan = 65535;

RangeUnit ChooseRangeUnit(float range_max_metres);
double UnitMetres(RangeUnit unit);
std::uint16_t QuantiseRange(float metres, RangeUnit unit);
float DequantiseRange(std::uint16_t code, RangeUnit unit);
std::uint16_t QuantiseIntensity(float value);

// Per scan: code[i] - code[i-1] as uint16 (wrapping), ray 0 against 0. Then every value's low
// byte, then every value's high byte, over the whole block in scan order.
// `codes.size()` must be a multiple of `rays`. The inverse returns an empty vector for input
// that is not a whole number of scans.
std::string DeltaShuffle(const std::vector<std::uint16_t>& codes, std::size_t rays);
std::vector<std::uint16_t> InverseDeltaShuffle(const std::string& bytes, std::size_t rays);

// The fields that make scans members of one block. `time_increment` and `scan_time` are not
// here: real drivers measure them per sweep, so each scan carries its own pair.
struct ScanGeometry {
  float angle_min, angle_max, angle_increment, range_min, range_max;
  std::uint32_t rays;
};
ScanGeometry GeometryOf(const LaserScanData& scan);
bool operator==(const ScanGeometry& a, const ScanGeometry& b);

struct EncodedScanBlock {
  bool ok = false;
  // "empty", "rays_differ", "zero_rays", "rays_over_uint16", "intensities_differ", "zstd_failed"
  std::string reason;
  RangeUnit unit = RangeUnit::kMillimetre;
  std::uint32_t scans = 0;
  std::string ranges;           // zstd(DeltaShuffle(codes))
  std::string intensities;      // same layout; empty unless asked for
  std::uint64_t raw_bytes = 0;  // scans * rays * 2, doubled when intensities are encoded
};

EncodedScanBlock Encode(const std::vector<LaserScanData>& scans, bool intensities,
                        int zstd_level);

}  // namespace c3d::scanblock
