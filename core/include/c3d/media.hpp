// Camera frames, lidar scans and the point cloud encoder.
//
// ROS-free like the rest of `core/`: a point cloud arrives here as three plain
// arrays, already in the scene axis convention, and the caller is whatever
// converted them. That boundary is what makes the encoder a pure function with
// byte-exact tests, and what keeps `sensor_msgs` out of this directory.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "c3d/types.hpp"

namespace c3d {

// A sensor's pose in the session's reference frame, with the session's
// `pose_convention` already applied. It is the only converted thing in a camera
// frame or a lidar scan, and it is what makes the payload readable without any
// knowledge of REP-103: `world = rotate(rotation, p) + position`.
//
// `rotation` answers to the coordinates the record it sits in carries, so two
// sensors sharing one physical frame do not share these numbers. A camera frame
// and a point cloud take the sensor's own rotation through the scene map; a
// LaserScan, whose rays reach the wire unconverted, takes the rotation of the scan
// PLANE instead -- see LaserScanData.
struct Pose {
  Vec3 position;
  Quat rotation;
};

// A camera's field of view when one frame was taken: two FULL angles in degrees,
// edge to edge across the image, horizontal and vertical. It travels with every
// video chunk, taken from the chunk's first frame, rather than once per session
// because it is not a constant of the camera: a zoom lens changes it.
//
// 0 means NOT RECORDED, and it is the default -- no real lens has a 0 degree field
// of view, so the wire spends no flag on it. A reader that finds 0 draws no
// frustum rather than a guessed one.
//
// The angles describe the image as published, and scaling it does not change them:
// pass the same field of view whatever size the stored frame is. From a calibrated
// pinhole camera, CameraFovFromPinhole computes them.
struct CameraFov {
  float hfov_deg = 0.0f;
  float vfov_deg = 0.0f;
};

// Whether `fov` is one a frame carries: BOTH angles finite and strictly between 0
// and 180. Anything else -- 0 for unknown, one angle without the other, NaN or
// infinity, a negative or reflex angle -- is not, and Session::RecordCameraAccessUnit
// records such a frame with 0 for both, so a NaN never reaches the wire and a
// frustum is never drawn from half a measurement.
bool IsKnownCameraFov(const CameraFov& fov);

// The field of view of a pinhole camera from its intrinsics, all in pixels:
// focal lengths `fx` and `fy`, principal point `cx` and `cy` measured from the
// image's top-left corner, and the image's `width` and `height` in the same pixel
// units. The principal point need not be centred, which is why each angle is the
// sum of its two halves:
//
//   hfov = atan(cx / fx) + atan((width - cx) / fx)
//   vfov = atan(cy / fy) + atan((height - cy) / fy)
//
// Not recorded (0 by 0) unless the inputs describe a camera: a focal length of 0 or
// less, which is what an uncalibrated driver publishes, a size of 0 or less, any
// non-finite input, or a result IsKnownCameraFov refuses. It is the field of view of
// the undistorted pinhole model, so a wide lens with strong barrel distortion sees
// further at its edges than these angles say.
CameraFov CameraFovFromPinhole(double fx, double fy, double cx, double cy, double width,
                               double height);

// A LaserScan as published. Angles, times and ranges stay in the sensor's own
// units and its own frame, because converting 640 rays throws away what the sensor
// said and the reader can rebuild each one from the pose:
//
//   world = rotate(pose.rotation, (r cos t, r sin t, 0)) + pose.position
//
// with `t` walked from `angle_min` by `angle_increment`. That rule is what
// `pose.rotation` means for a scan, and it is NOT the sensor frame's own rotation:
// a scene map that mirrors handedness is not a rotation at all, so the caller
// sends the proper rotation agreeing with it on the scan plane and the mirror
// lands on the axis no ray uses. The reader needs nothing but this record.
struct LaserScanData {
  float angle_min = 0.0f;
  float angle_max = 0.0f;
  float angle_increment = 0.0f;
  float time_increment = 0.0f;
  float scan_time = 0.0f;
  float range_min = 0.0f;
  float range_max = 0.0f;
  // Infinities and NaNs are preserved: a return outside the sensor's own limits
  // is what the driver means by "nothing there", and rewriting it to a number
  // puts an obstacle in the record of an empty room.
  std::vector<float> ranges;
  std::vector<float> intensities;  // empty unless the sensor is configured for them
};

// The element type of the position planes. `kFloat32` exists for fidelity
// debugging; `kInt16Mm` is millimetres and clamps at +/-32.767 m.
enum class PointPosition { kInt16Mm, kFloat32 };
enum class PointColor { kNone, kRgb565, kRgb332, kRgb888 };
enum class PointIntensity { kNone, kUint8 };

// The lowercase wire spellings. These four fields are `Text` on the wire and the
// gateway answers 400 to anything not in these lists, so the spelling here is a
// wire contract rather than a local naming choice.
const char* PointPositionName(PointPosition p);
const char* PointColorName(PointColor c);
const char* PointIntensityName(PointIntensity i);
bool ParsePointPosition(const std::string& text, PointPosition* out);
bool ParsePointColor(const std::string& text, PointColor* out);
bool ParsePointIntensity(const std::string& text, PointIntensity* out);

enum class PointCloudEncoding { kPlanarZstd, kPlanarDeltaShuffleZstd };

// The schema enumerant spellings, for the session property.
const char* PointCloudEncodingName(PointCloudEncoding e);

struct PointCloudOptions {
  double voxel_leaf_m = 0.05;  // 0 keeps every point
  PointPosition position = PointPosition::kInt16Mm;
  PointColor color = PointColor::kNone;
  PointIntensity intensity = PointIntensity::kNone;
  // A cloud still larger than this AFTER downsampling is dropped whole. Never
  // truncated: half a cloud is a cloud with a hole in it, and nothing on the
  // wire would say which half.
  std::size_t max_points = 100000;
  int zstd_level = 3;  // 0 sends the raw planar block
  // Honoured for int16 positions only; float32 is always written as kPlanarZstd.
  PointCloudEncoding encoding = PointCloudEncoding::kPlanarDeltaShuffleZstd;
};

struct PointCloudInput {
  // 3*N floats, x y z interleaved, ALREADY in the scene axis convention.
  std::vector<float> xyz;
  // 3*N bytes, r g b interleaved, or empty. Ignored unless `color` asks for it.
  std::vector<std::uint8_t> rgb;
  // N floats, or empty. Ignored unless `intensity` asks for it.
  std::vector<float> intensity;
  // Whether the source field was integer-typed. It decides the intensity scaling
  // rule and cannot be recovered from the values: a float field whose samples
  // happen to be 0..1 is scaled by 255, while a uint8 field whose samples happen
  // to be 0..1 is a nearly black cloud and must not be.
  bool intensity_is_integral = false;
};

struct EncodedPointCloud {
  // False means nothing should be sent. `reason` says why, for the counter the
  // caller increments -- an encoder that returned an empty block instead would
  // put a zero-point scan on the wire and call it data.
  bool ok = false;
  std::string reason;

  std::uint32_t point_count = 0;
  PointPosition position = PointPosition::kInt16Mm;
  PointColor color = PointColor::kNone;
  PointIntensity intensity = PointIntensity::kNone;
  std::uint16_t voxel_leaf_mm = 0;  // 0 means the cloud was not downsampled
  std::uint8_t zstd_level = 0;      // 0 means `data` IS the raw planar block
  PointCloudEncoding encoding = PointCloudEncoding::kPlanarZstd;  // what was written
  std::string data;

  // Points the filter and the int16 range check removed, reported separately
  // because they mean different things: a non-finite point is a sensor with
  // nothing to return there, and an out-of-range one is a sensor seeing further
  // than the chosen position format can carry.
  std::uint64_t dropped_non_finite = 0;
  std::uint64_t dropped_out_of_range = 0;
  // The size of the planar block before compression, so a caller can report a
  // compression ratio without decompressing what it just built.
  std::uint64_t raw_bytes = 0;
};

namespace pointcloud {

// Voxel-downsample, quantise, sort and compress a cloud into the planar block
// documented in docs/data-format.md.
//
// Deterministic by construction, and that is a requirement rather than a
// property: the sort by voxel key is what puts spatial neighbours next to each
// other in the byte stream, which is the only reason zstd earns its place, and
// it is also what makes the output testable byte for byte.
EncodedPointCloud Encode(const PointCloudInput& input, const PointCloudOptions& options);

// The most points a cloud may have for Encode's output under `options` never to pass `bytes`,
// whatever the points are: zstd's worst case over the planar block of 2 or 4 bytes a position
// axis, the colour's bytes and the intensity's, or the block itself at zstd level 0.
std::size_t MaxPointsWithin(std::uint64_t bytes, const PointCloudOptions& options);

// The encoding Encode writes under `options`: planarDeltaShuffleZstd only for int16_mm positions
// compressed at a zstd level above 0, whatever `options.encoding` asks for; planarZstd otherwise.
PointCloudEncoding EffectiveEncoding(const PointCloudOptions& options);

// Position planes only, int16 only: per plane, value[i] - value[i-1] (wrapping,
// the first against 0), then that plane's low bytes, then its high bytes. The
// colour and intensity planes follow unchanged. Any other position_bytes returns
// the input as it is; the inverse returns an empty string for a block too short
// to hold the position planes.
std::string DeltaShufflePositionPlanes(const std::string& planar, std::size_t n,
                                       std::size_t position_bytes);
std::string InverseDeltaShufflePositionPlanes(const std::string& transformed, std::size_t n,
                                              std::size_t position_bytes);

}  // namespace pointcloud
}  // namespace c3d
