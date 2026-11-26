#include "c3d/media.hpp"

#include <zstd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace c3d {

const char* PointCloudEncodingName(PointCloudEncoding e) {
  return e == PointCloudEncoding::kPlanarDeltaShuffleZstd ? "planarDeltaShuffleZstd"
                                                          : "planarZstd";
}

const char* PointPositionName(PointPosition p) {
  return p == PointPosition::kFloat32 ? "float32" : "int16_mm";
}

const char* PointColorName(PointColor c) {
  switch (c) {
    case PointColor::kNone: return "none";
    case PointColor::kRgb565: return "rgb565";
    case PointColor::kRgb332: return "rgb332";
    case PointColor::kRgb888: return "rgb888";
  }
  return "none";
}

const char* PointIntensityName(PointIntensity i) {
  return i == PointIntensity::kUint8 ? "uint8" : "none";
}

bool ParsePointPosition(const std::string& text, PointPosition* out) {
  if (text == "int16_mm") { *out = PointPosition::kInt16Mm; return true; }
  if (text == "float32") { *out = PointPosition::kFloat32; return true; }
  return false;
}

bool ParsePointColor(const std::string& text, PointColor* out) {
  if (text == "none") { *out = PointColor::kNone; return true; }
  if (text == "rgb565") { *out = PointColor::kRgb565; return true; }
  if (text == "rgb332") { *out = PointColor::kRgb332; return true; }
  if (text == "rgb888") { *out = PointColor::kRgb888; return true; }
  return false;
}

bool ParsePointIntensity(const std::string& text, PointIntensity* out) {
  if (text == "none") { *out = PointIntensity::kNone; return true; }
  if (text == "uint8") { *out = PointIntensity::kUint8; return true; }
  return false;
}

namespace pointcloud {
namespace {

struct VoxelKey {
  std::int64_t x = 0, y = 0, z = 0;
  bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const {
    // A cheap 64-bit mix per axis. The hash quality matters here: point cloud
    // keys are dense along one axis and a naive shift-xor collides whole scan
    // lines into one bucket, which turns the accumulation pass quadratic.
    auto mix = [](std::uint64_t v) {
      v ^= v >> 33;
      v *= 0xff51afd7ed558ccdULL;
      v ^= v >> 33;
      return v;
    };
    return static_cast<std::size_t>(mix(static_cast<std::uint64_t>(k.x)) ^
                                    (mix(static_cast<std::uint64_t>(k.y)) << 1) ^
                                    (mix(static_cast<std::uint64_t>(k.z)) << 2));
  }
};

struct Accumulator {
  double x = 0.0, y = 0.0, z = 0.0;
  double r = 0.0, g = 0.0, b = 0.0;
  double intensity = 0.0;
  std::uint64_t count = 0;
  // Insertion order, kept so the emitted vector can be built once at the end
  // rather than by walking an unordered_map, whose iteration order is not
  // reproducible across libstdc++ versions. The sort below imposes the final
  // order, but only over a container the encoder itself filled deterministically.
  std::size_t slot = 0;
};

struct Point {
  VoxelKey key;
  double x = 0.0, y = 0.0, z = 0.0;
  double r = 0.0, g = 0.0, b = 0.0;
  double intensity = 0.0;
};

// Lexicographic by (iz, iy, ix), then by everything else. The stated rule is the
// key; the tiebreak exists because without downsampling two distinct points can
// share a key, and a comparator that called them equal would make the output
// depend on the order the points arrived in -- which is exactly the property the
// "shuffled input, identical bytes" test pins.
bool Before(const Point& a, const Point& b) {
  if (a.key.z != b.key.z) return a.key.z < b.key.z;
  if (a.key.y != b.key.y) return a.key.y < b.key.y;
  if (a.key.x != b.key.x) return a.key.x < b.key.x;
  if (a.x != b.x) return a.x < b.x;
  if (a.y != b.y) return a.y < b.y;
  if (a.z != b.z) return a.z < b.z;
  if (a.r != b.r) return a.r < b.r;
  if (a.g != b.g) return a.g < b.g;
  if (a.b != b.b) return a.b < b.b;
  return a.intensity < b.intensity;
}

void PutU16(std::string* out, std::uint16_t v) {
  out->push_back(static_cast<char>(v & 0xFF));
  out->push_back(static_cast<char>((v >> 8) & 0xFF));
}

void PutI16(std::string* out, std::int16_t v) {
  PutU16(out, static_cast<std::uint16_t>(v));
}

void PutF32(std::string* out, float v) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  out->push_back(static_cast<char>(bits & 0xFF));
  out->push_back(static_cast<char>((bits >> 8) & 0xFF));
  out->push_back(static_cast<char>((bits >> 16) & 0xFF));
  out->push_back(static_cast<char>((bits >> 24) & 0xFF));
}

std::uint8_t ToByte(double v) {
  const double rounded = std::round(v);
  if (rounded <= 0.0) return 0;
  if (rounded >= 255.0) return 255;
  return static_cast<std::uint8_t>(rounded);
}

// Written out rather than derived so the encoder and docs/data-format.md's decoder
// can be read against each other line by line.
std::uint16_t PackRgb565(std::uint8_t r, std::uint8_t g, std::uint8_t b) {
  return static_cast<std::uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

std::uint8_t PackRgb332(std::uint8_t r, std::uint8_t g, std::uint8_t b) {
  return static_cast<std::uint8_t>((r & 0xE0) | ((g & 0xE0) >> 3) | (b >> 6));
}

std::size_t BytesPerColor(PointColor c) {
  switch (c) {
    case PointColor::kNone: return 0;
    case PointColor::kRgb565: return 2;
    case PointColor::kRgb332: return 1;
    case PointColor::kRgb888: return 3;
  }
  return 0;
}

}  // namespace

std::size_t MaxPointsWithin(std::uint64_t bytes, const PointCloudOptions& options) {
  const std::uint64_t per_point =
      (options.position == PointPosition::kFloat32 ? 12u : 6u) + BytesPerColor(options.color) +
      (options.intensity == PointIntensity::kUint8 ? 1u : 0u);
  if (options.zstd_level <= 0) return static_cast<std::size_t>(bytes / per_point);
  // The largest n whose block's bound fits; the bound grows with n.
  std::uint64_t low = 0, high = bytes / per_point;
  while (low < high) {
    const std::uint64_t mid = low + (high - low + 1) / 2;
    if (ZSTD_compressBound(static_cast<std::size_t>(mid * per_point)) <= bytes) {
      low = mid;
    } else {
      high = mid - 1;
    }
  }
  return static_cast<std::size_t>(low);
}

std::string DeltaShufflePositionPlanes(const std::string& planar, std::size_t n,
                                       std::size_t position_bytes) {
  if (position_bytes != 2 || planar.size() < 3 * n * 2) return planar;
  std::string out = planar;
  for (std::size_t plane = 0; plane < 3; ++plane) {
    const std::size_t base = plane * n * 2;
    std::uint16_t previous = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint16_t value =
          static_cast<std::uint16_t>(static_cast<unsigned char>(planar[base + i * 2]) |
                                     (static_cast<unsigned char>(planar[base + i * 2 + 1]) << 8));
      const std::uint16_t delta = static_cast<std::uint16_t>(value - previous);
      previous = value;
      out[base + i] = static_cast<char>(delta & 0xFF);
      out[base + n + i] = static_cast<char>(delta >> 8);
    }
  }
  return out;
}

std::string InverseDeltaShufflePositionPlanes(const std::string& transformed, std::size_t n,
                                              std::size_t position_bytes) {
  if (position_bytes != 2) return transformed;
  if (transformed.size() < 3 * n * 2) return std::string();
  std::string out = transformed;
  for (std::size_t plane = 0; plane < 3; ++plane) {
    const std::size_t base = plane * n * 2;
    std::uint16_t value = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint16_t delta =
          static_cast<std::uint16_t>(static_cast<unsigned char>(transformed[base + i]) |
                                     (static_cast<unsigned char>(transformed[base + n + i]) << 8));
      value = static_cast<std::uint16_t>(value + delta);
      out[base + i * 2] = static_cast<char>(value & 0xFF);
      out[base + i * 2 + 1] = static_cast<char>(value >> 8);
    }
  }
  return out;
}

PointCloudEncoding EffectiveEncoding(const PointCloudOptions& options) {
  return options.encoding == PointCloudEncoding::kPlanarDeltaShuffleZstd &&
                 options.position == PointPosition::kInt16Mm && options.zstd_level > 0
             ? PointCloudEncoding::kPlanarDeltaShuffleZstd
             : PointCloudEncoding::kPlanarZstd;
}

EncodedPointCloud Encode(const PointCloudInput& input, const PointCloudOptions& options) {
  EncodedPointCloud out;
  out.position = options.position;
  out.color = options.color;
  out.intensity = options.intensity;

  const std::size_t n = input.xyz.size() / 3;
  if (n == 0) {
    out.reason = "empty";
    return out;
  }
  // An attribute the caller did not actually supply is not encoded, and the
  // RESULT says so rather than the options: every field on the wire describes
  // the block that was written, so a reader never has to reconcile a `color`
  // that says rgb565 with a block that has no colour plane in it.
  const bool have_rgb = input.rgb.size() == n * 3;
  const bool have_intensity = input.intensity.size() == n;
  if (!have_rgb) out.color = PointColor::kNone;
  if (!have_intensity) out.intensity = PointIntensity::kNone;

  const bool downsample = options.voxel_leaf_m > 0.0;
  const double leaf = downsample ? options.voxel_leaf_m : 0.0;

  // The intensity scale is decided over the SOURCE samples, before voxel
  // averaging, because averaging can only pull the maximum down: a cloud whose
  // one saturated return is merged with a dark neighbour would otherwise flip
  // from the 0..1 rule to the 0..255 one and come out 255 times too dark.
  double observed_max_intensity = 0.0;

  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> voxels;
  std::vector<Point> points;
  if (downsample) {
    voxels.reserve(n);
  } else {
    points.reserve(n);
  }

  for (std::size_t i = 0; i < n; ++i) {
    const double x = input.xyz[i * 3 + 0];
    const double y = input.xyz[i * 3 + 1];
    const double z = input.xyz[i * 3 + 2];
    // The magnitude bound is part of the finiteness test, not a separate policy:
    // a coordinate large enough to overflow the int64 voxel index is not a
    // measurement, and casting it would be undefined rather than merely wrong.
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        std::abs(x) > 1e12 || std::abs(y) > 1e12 || std::abs(z) > 1e12) {
      ++out.dropped_non_finite;
      continue;
    }
    const double r = have_rgb ? input.rgb[i * 3 + 0] : 0.0;
    const double g = have_rgb ? input.rgb[i * 3 + 1] : 0.0;
    const double b = have_rgb ? input.rgb[i * 3 + 2] : 0.0;
    double intensity = 0.0;
    if (have_intensity) {
      intensity = input.intensity[i];
      if (!std::isfinite(intensity)) intensity = 0.0;
      observed_max_intensity = std::max(observed_max_intensity, intensity);
    }

    if (downsample) {
      VoxelKey key;
      key.x = static_cast<std::int64_t>(std::floor(x / leaf));
      key.y = static_cast<std::int64_t>(std::floor(y / leaf));
      key.z = static_cast<std::int64_t>(std::floor(z / leaf));
      auto& accumulator = voxels[key];
      if (accumulator.count == 0) accumulator.slot = voxels.size() - 1;
      accumulator.x += x;
      accumulator.y += y;
      accumulator.z += z;
      accumulator.r += r;
      accumulator.g += g;
      accumulator.b += b;
      accumulator.intensity += intensity;
      ++accumulator.count;
    } else {
      Point point;
      // Not downsampled, so the sort key is the quantised position itself: the
      // same millimetre grid the int16 format uses, so the byte stream is
      // ordered the same way whichever position format is in force.
      point.key.x = static_cast<std::int64_t>(std::llround(x * 1000.0));
      point.key.y = static_cast<std::int64_t>(std::llround(y * 1000.0));
      point.key.z = static_cast<std::int64_t>(std::llround(z * 1000.0));
      point.x = x;
      point.y = y;
      point.z = z;
      point.r = r;
      point.g = g;
      point.b = b;
      point.intensity = intensity;
      points.push_back(point);
    }
  }

  if (downsample) {
    points.resize(voxels.size());
    for (const auto& kv : voxels) {
      const Accumulator& a = kv.second;
      const double inverse = 1.0 / static_cast<double>(a.count);
      Point& point = points[a.slot];
      point.key = kv.first;
      point.x = a.x * inverse;
      point.y = a.y * inverse;
      point.z = a.z * inverse;
      point.r = a.r * inverse;
      point.g = a.g * inverse;
      point.b = a.b * inverse;
      point.intensity = a.intensity * inverse;
    }
    out.voxel_leaf_mm = static_cast<std::uint16_t>(
        std::min(65535.0, std::max(0.0, std::round(leaf * 1000.0))));
  }

  if (points.empty()) {
    out.reason = "no_finite_points";
    return out;
  }
  // The cap is checked AFTER downsampling and drops the whole cloud. Truncating
  // to the cap would send a cloud with a hole in it and nothing on the wire
  // would say which part is missing.
  if (points.size() > options.max_points) {
    out.reason = "oversize";
    return out;
  }

  std::sort(points.begin(), points.end(), Before);

  // The intensity rule, stated once here because the wire deliberately does not
  // carry it: a reader needs the 0..255 number, not its provenance.
  const bool scale_unit_intensity =
      !input.intensity_is_integral && observed_max_intensity <= 1.0;

  const std::size_t position_bytes = options.position == PointPosition::kFloat32 ? 4 : 2;
  const std::size_t color_bytes = BytesPerColor(out.color);
  const std::size_t intensity_bytes = out.intensity == PointIntensity::kUint8 ? 1 : 0;

  std::string block;
  block.reserve(points.size() * (position_bytes * 3 + color_bytes + intensity_bytes));

  // int16 range is checked across ALL THREE axes before any of them is written,
  // so a point outside it is dropped whole. Writing x and then discovering z is
  // out of range would leave the planes different lengths, which no reader could
  // detect and every reader would misparse from that point on.
  std::vector<const Point*> kept;
  kept.reserve(points.size());
  if (options.position == PointPosition::kInt16Mm) {
    for (const Point& point : points) {
      const double mx = std::round(point.x * 1000.0);
      const double my = std::round(point.y * 1000.0);
      const double mz = std::round(point.z * 1000.0);
      if (std::abs(mx) > 32767.0 || std::abs(my) > 32767.0 || std::abs(mz) > 32767.0) {
        ++out.dropped_out_of_range;
        continue;
      }
      kept.push_back(&point);
    }
  } else {
    for (const Point& point : points) kept.push_back(&point);
  }
  if (kept.empty()) {
    out.reason = "no_points_in_range";
    return out;
  }

  for (int axis = 0; axis < 3; ++axis) {
    for (const Point* point : kept) {
      const double v = axis == 0 ? point->x : (axis == 1 ? point->y : point->z);
      if (options.position == PointPosition::kInt16Mm) {
        PutI16(&block, static_cast<std::int16_t>(std::round(v * 1000.0)));
      } else {
        PutF32(&block, static_cast<float>(v));
      }
    }
  }

  if (out.color != PointColor::kNone) {
    for (const Point* point : kept) {
      const std::uint8_t r = ToByte(point->r);
      const std::uint8_t g = ToByte(point->g);
      const std::uint8_t b = ToByte(point->b);
      switch (out.color) {
        case PointColor::kRgb565: PutU16(&block, PackRgb565(r, g, b)); break;
        case PointColor::kRgb332: block.push_back(static_cast<char>(PackRgb332(r, g, b))); break;
        case PointColor::kRgb888:
          block.push_back(static_cast<char>(r));
          block.push_back(static_cast<char>(g));
          block.push_back(static_cast<char>(b));
          break;
        case PointColor::kNone: break;
      }
    }
  }

  if (out.intensity == PointIntensity::kUint8) {
    for (const Point* point : kept) {
      block.push_back(static_cast<char>(
          scale_unit_intensity ? ToByte(point->intensity * 255.0) : ToByte(point->intensity)));
    }
  }

  out.point_count = static_cast<std::uint32_t>(kept.size());
  out.raw_bytes = block.size();

  const int level = std::min(ZSTD_maxCLevel(), std::max(0, options.zstd_level));
  out.encoding = PointCloudEncoding::kPlanarZstd;
  if (level == 0) {
    out.zstd_level = 0;
    out.data = std::move(block);
    out.ok = true;
    return out;
  }

  if (EffectiveEncoding(options) == PointCloudEncoding::kPlanarDeltaShuffleZstd) {
    block = DeltaShufflePositionPlanes(block, kept.size(), position_bytes);
    out.encoding = PointCloudEncoding::kPlanarDeltaShuffleZstd;
  }

  std::string compressed;
  compressed.resize(ZSTD_compressBound(block.size()));
  const std::size_t written =
      ZSTD_compress(&compressed[0], compressed.size(), block.data(), block.size(), level);
  if (ZSTD_isError(written)) {
    out.reason = "zstd_failed";
    return out;
  }
  compressed.resize(written);
  out.zstd_level = static_cast<std::uint8_t>(level);
  out.data = std::move(compressed);
  out.ok = true;
  return out;
}

}  // namespace pointcloud
}  // namespace c3d
