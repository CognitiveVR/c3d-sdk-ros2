#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include "c3d/media.hpp"
#include "cognitive3d_ros/coordinate_transform.hpp"

namespace cognitive3d_ros {

// Where x, y, z and the optional attributes live in one point of a PointCloud2.
//
// A PointCloud2 is a byte buffer plus a self-description, and every driver
// describes it differently: colour arrives as a float32 with a uint32 bit-punned
// into it (the PCL convention), as a plain uint32, or as three separate uint8
// fields; intensity as float32, uint8 or uint16. Resolving that ONCE per message
// into offsets, rather than branching per point, is both faster and the only way
// the variants are testable without a cloud.
struct PointCloudLayout {
  bool ok = false;
  // Why not. Named so the node's once-per-topic warning can say which field it
  // could not find rather than "unsupported cloud".
  std::string error;

  std::uint32_t x_offset = 0;
  std::uint32_t y_offset = 0;
  std::uint32_t z_offset = 0;

  enum class ColorKind {
    kNone,
    kPackedFloat32,  // one `rgb`/`rgba` field, FLOAT32, holding a uint32 bit pattern
    kPackedUint32,   // one `rgb`/`rgba` field, UINT32
    kThreeBytes,     // separate `r`, `g`, `b` fields, UINT8 each
  };
  ColorKind color = ColorKind::kNone;
  std::uint32_t color_offset = 0;  // kThreeBytes: the offset of `r`
  std::uint32_t g_offset = 0;
  std::uint32_t b_offset = 0;

  enum class IntensityKind { kNone, kFloat32, kUint8, kUint16 };
  IntensityKind intensity = IntensityKind::kNone;
  std::uint32_t intensity_offset = 0;
};

// `x`, `y` and `z` are REQUIRED and must be FLOAT32 -- every ROS point cloud that
// carries positions at all carries them that way, and a cloud that does not is
// one this SDK has never seen and must not guess at.
//
// Takes the whole message, not just the fields: byte order, `point_step` and the
// fields are one self-description and a field only makes sense against the step
// it is an offset into. Every refusal is a layout error so it reaches the
// once-per-topic warning with a reason; one that arrives at the encoder as an
// empty cloud is indistinguishable from a cloud that was genuinely empty.
PointCloudLayout ResolvePointCloudLayout(const sensor_msgs::msg::PointCloud2& cloud);

// Walk the cloud into the encoder's plain arrays, converting each point into the
// scene axis convention on the way.
//
// The conversion happens HERE, per point, and not in the encoder: `core/` is told
// its input is already in scene axes, which is what keeps it free of any opinion
// about REP-103. `want_color` and `want_intensity` follow the configuration, so a
// cloud whose colour is never going to be encoded is never read for it.
c3d::PointCloudInput ExtractPointCloud(const sensor_msgs::msg::PointCloud2& cloud,
                                       const PointCloudLayout& layout, bool want_color,
                                       bool want_intensity, PoseConvention convention);

// How many of a cloud's points are finite, and how many of those lie on the
// positive side of x and of z -- in the SENSOR's own axes, before any conversion,
// because the question this answers is which ROS frame the publisher put them in
// and the scene map would hide that.
//
// It is the only thing that can catch a cloud whose header names a frame its
// points are not in: a real optical-frame cloud has depth on +Z, so essentially
// every finite point has z > 0 while x, the horizontal axis, is mixed; a cloud
// left in the body frame the sensor is attached to has it the other way round.
// Neither count alone separates them. doctor::CheckOpticalCloud reads both.
struct PointCloudAxisSigns {
  std::uint64_t finite = 0;
  std::uint64_t x_positive = 0;
  std::uint64_t z_positive = 0;
};
PointCloudAxisSigns CountPointCloudAxisSigns(const sensor_msgs::msg::PointCloud2& cloud,
                                             const PointCloudLayout& layout);

}  // namespace cognitive3d_ros
