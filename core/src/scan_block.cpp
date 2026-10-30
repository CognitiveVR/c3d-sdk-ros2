#include "c3d/scan_block.hpp"

#include <zstd.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace c3d::scanblock {
namespace {

std::string Compress(const std::string& raw, int level) {
  std::string out(ZSTD_compressBound(raw.size()), '\0');
  const std::size_t n = ZSTD_compress(out.data(), out.size(), raw.data(), raw.size(), level);
  if (ZSTD_isError(n)) return {};
  out.resize(n);
  return out;
}

}  // namespace

RangeUnit ChooseRangeUnit(float range_max_metres) {
  return range_max_metres <= kMillimetreUnitMaxRangeMetres ? RangeUnit::kMillimetre
                                                           : RangeUnit::kCentimetre;
}

double UnitMetres(RangeUnit unit) { return unit == RangeUnit::kMillimetre ? 0.001 : 0.01; }

std::uint16_t QuantiseRange(float metres, RangeUnit unit) {
  if (std::isnan(metres)) return kCodeNaN;
  if (std::isinf(metres)) return metres > 0 ? kCodePosInf : kCodeNegInf;
  const double code = std::nearbyint(static_cast<double>(metres) / UnitMetres(unit));
  return static_cast<std::uint16_t>(std::clamp(code, 0.0, static_cast<double>(kCodeMax)));
}

float DequantiseRange(std::uint16_t code, RangeUnit unit) {
  if (code == kCodePosInf) return std::numeric_limits<float>::infinity();
  if (code == kCodeNaN) return std::numeric_limits<float>::quiet_NaN();
  if (code == kCodeNegInf) return -std::numeric_limits<float>::infinity();
  return static_cast<float>(code * UnitMetres(unit));
}

std::uint16_t QuantiseIntensity(float value) {
  if (!std::isfinite(value)) return 0;
  return static_cast<std::uint16_t>(std::nearbyint(std::clamp(value, 0.0f, 65535.0f)));
}

std::string DeltaShuffle(const std::vector<std::uint16_t>& codes, std::size_t rays) {
  const std::size_t n = codes.size();
  std::string out(n * 2, '\0');
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint16_t previous = (rays == 0 || i % rays == 0) ? 0 : codes[i - 1];
    const std::uint16_t delta = static_cast<std::uint16_t>(codes[i] - previous);
    out[i] = static_cast<char>(delta & 0xFF);
    out[n + i] = static_cast<char>(delta >> 8);
  }
  return out;
}

std::vector<std::uint16_t> InverseDeltaShuffle(const std::string& bytes, std::size_t rays) {
  if (rays == 0 || bytes.size() % (2 * rays) != 0) return {};
  const std::size_t n = bytes.size() / 2;
  std::vector<std::uint16_t> codes(n);
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint16_t delta = static_cast<std::uint16_t>(
        static_cast<unsigned char>(bytes[i]) | (static_cast<unsigned char>(bytes[n + i]) << 8));
    const std::uint16_t previous = (i % rays == 0) ? 0 : codes[i - 1];
    codes[i] = static_cast<std::uint16_t>(previous + delta);
  }
  return codes;
}

ScanGeometry GeometryOf(const LaserScanData& scan) {
  return {scan.angle_min, scan.angle_max, scan.angle_increment,
          scan.range_min, scan.range_max, static_cast<std::uint32_t>(scan.ranges.size())};
}

namespace {

// NaN is the same field as NaN: a driver that publishes one would otherwise start a block per scan.
bool Same(float a, float b) { return a == b || (std::isnan(a) && std::isnan(b)); }

}  // namespace

bool operator==(const ScanGeometry& a, const ScanGeometry& b) {
  return Same(a.angle_min, b.angle_min) && Same(a.angle_max, b.angle_max) &&
         Same(a.angle_increment, b.angle_increment) && Same(a.range_min, b.range_min) &&
         Same(a.range_max, b.range_max) && a.rays == b.rays;
}

EncodedScanBlock Encode(const std::vector<LaserScanData>& scans, bool intensities,
                        int zstd_level) {
  EncodedScanBlock block;
  if (scans.empty()) {
    block.reason = "empty";
    return block;
  }
  const std::size_t rays = scans[0].ranges.size();
  if (rays == 0) {
    block.reason = "zero_rays";
    return block;
  }
  if (rays > kMaxRaysPerScan) {
    block.reason = "rays_over_uint16";
    return block;
  }
  for (const LaserScanData& scan : scans) {
    if (scan.ranges.size() != rays) {
      block.reason = "rays_differ";
      return block;
    }
    if (intensities && scan.intensities.size() != rays) {
      block.reason = "intensities_differ";
      return block;
    }
  }

  block.unit = ChooseRangeUnit(scans[0].range_max);
  const int level = std::clamp(zstd_level, 1, ZSTD_maxCLevel());
  std::vector<std::uint16_t> range_codes;
  std::vector<std::uint16_t> intensity_codes;
  range_codes.reserve(scans.size() * rays);
  for (const LaserScanData& scan : scans) {
    for (float r : scan.ranges) range_codes.push_back(QuantiseRange(r, block.unit));
    if (intensities) {
      for (float v : scan.intensities) intensity_codes.push_back(QuantiseIntensity(v));
    }
  }

  block.ranges = Compress(DeltaShuffle(range_codes, rays), level);
  if (intensities) block.intensities = Compress(DeltaShuffle(intensity_codes, rays), level);
  if (block.ranges.empty() || (intensities && block.intensities.empty())) {
    block.ranges.clear();
    block.intensities.clear();
    block.reason = "zstd_failed";
    return block;
  }
  block.scans = static_cast<std::uint32_t>(scans.size());
  block.raw_bytes = static_cast<std::uint64_t>(scans.size()) * rays * 2 * (intensities ? 2 : 1);
  block.ok = true;
  return block;
}

}  // namespace c3d::scanblock
