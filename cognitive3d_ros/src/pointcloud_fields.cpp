#include "cognitive3d_ros/pointcloud_fields.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace cognitive3d_ros {
namespace {

using sensor_msgs::msg::PointField;

const PointField* Find(const std::vector<PointField>& fields, const std::string& name) {
  for (const auto& field : fields) {
    if (field.name == name) return &field;
  }
  return nullptr;
}

std::size_t DatatypeSize(std::uint8_t datatype) {
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

// A field whose last byte lies past `point_step` is read out of the NEXT point,
// or off the end of the buffer on the last one. The self-description is wrong
// either way and the values that come back are plausible numbers, so the cloud is
// refused rather than read.
bool FieldFits(const PointField& field, std::uint32_t point_step, std::string* error) {
  const std::size_t size = DatatypeSize(field.datatype);
  if (size != 0 && field.offset + size <= point_step) return true;
  *error = "field '" + field.name + "' at offset " + std::to_string(field.offset) +
           " does not fit in a point_step of " + std::to_string(point_step);
  return false;
}

template <typename T>
T Read(const std::uint8_t* base, std::uint32_t offset) {
  T value;
  std::memcpy(&value, base + offset, sizeof(T));
  return value;
}

// A row-major cloud may PAD each row, and `row_step` is the only place that
// padding is declared. Walked as one flat array it is read as points. A row_step
// smaller than the row's own points is a self-description that cannot be honoured,
// so the flat stride stands.
std::size_t RowStride(const sensor_msgs::msg::PointCloud2& cloud) {
  const std::size_t flat = static_cast<std::size_t>(cloud.width) * cloud.point_step;
  return cloud.row_step > flat ? static_cast<std::size_t>(cloud.row_step) : flat;
}

// `width * height`, less whatever the buffer is short by. NOT `data.size() /
// point_step`: a buffer longer than the cloud it describes -- a reused allocation,
// a driver that rounds up -- would otherwise contribute whatever is past the end
// as points.
std::size_t ReadablePoints(const sensor_msgs::msg::PointCloud2& cloud, std::size_t row_stride) {
  std::size_t count = 0;
  for (std::size_t row = 0; row < cloud.height; ++row) {
    const std::size_t base = row * row_stride;
    if (base >= cloud.data.size()) break;
    count += std::min<std::size_t>(cloud.width, (cloud.data.size() - base) / cloud.point_step);
  }
  return count;
}

// Only the LAST counted row can be short, so every earlier one contributed a full
// `width` and the division recovers the row.
const std::uint8_t* PointAt(const sensor_msgs::msg::PointCloud2& cloud, std::size_t row_stride,
                            std::size_t i) {
  return cloud.data.data() + (i / cloud.width) * row_stride +
         (i % cloud.width) * cloud.point_step;
}

}  // namespace

PointCloudLayout ResolvePointCloudLayout(const sensor_msgs::msg::PointCloud2& cloud) {
  PointCloudLayout layout;
  // Every ROS 2 platform this ships on is little-endian, and a big-endian cloud
  // read as one would be silently wrong rather than obviously so.
  if (cloud.is_bigendian) {
    layout.error = "the cloud is big-endian and this SDK reads little-endian only";
    return layout;
  }
  if (cloud.point_step == 0) {
    layout.error = "point_step is 0, so the cloud describes no points";
    return layout;
  }

  const std::vector<PointField>& fields = cloud.fields;
  const PointField* x = Find(fields, "x");
  const PointField* y = Find(fields, "y");
  const PointField* z = Find(fields, "z");
  if (x == nullptr || y == nullptr || z == nullptr) {
    layout.error = "no x/y/z fields";
    return layout;
  }
  if (x->datatype != PointField::FLOAT32 || y->datatype != PointField::FLOAT32 ||
      z->datatype != PointField::FLOAT32) {
    layout.error = "x/y/z are not FLOAT32";
    return layout;
  }
  if (!FieldFits(*x, cloud.point_step, &layout.error) ||
      !FieldFits(*y, cloud.point_step, &layout.error) ||
      !FieldFits(*z, cloud.point_step, &layout.error)) {
    return layout;
  }
  layout.x_offset = x->offset;
  layout.y_offset = y->offset;
  layout.z_offset = z->offset;

  // `rgb` first, then `rgba`: a cloud that carries both is carrying the same
  // colour twice and the opaque one is the one a viewer wants.
  const PointField* rgb = Find(fields, "rgb");
  if (rgb == nullptr) rgb = Find(fields, "rgba");
  if (rgb != nullptr) {
    if (!FieldFits(*rgb, cloud.point_step, &layout.error)) return layout;
    if (rgb->datatype == PointField::FLOAT32) {
      // PCL's convention: a uint32 0x00RRGGBB bit-punned into a float, which is
      // why the value is meaningless as a number and must never be read as one.
      layout.color = PointCloudLayout::ColorKind::kPackedFloat32;
      layout.color_offset = rgb->offset;
    } else if (rgb->datatype == PointField::UINT32) {
      layout.color = PointCloudLayout::ColorKind::kPackedUint32;
      layout.color_offset = rgb->offset;
    }
  }
  if (layout.color == PointCloudLayout::ColorKind::kNone) {
    const PointField* r = Find(fields, "r");
    const PointField* g = Find(fields, "g");
    const PointField* b = Find(fields, "b");
    if (r != nullptr && g != nullptr && b != nullptr && r->datatype == PointField::UINT8 &&
        g->datatype == PointField::UINT8 && b->datatype == PointField::UINT8) {
      if (!FieldFits(*r, cloud.point_step, &layout.error) ||
          !FieldFits(*g, cloud.point_step, &layout.error) ||
          !FieldFits(*b, cloud.point_step, &layout.error)) {
        return layout;
      }
      layout.color = PointCloudLayout::ColorKind::kThreeBytes;
      layout.color_offset = r->offset;
      layout.g_offset = g->offset;
      layout.b_offset = b->offset;
    }
  }

  const PointField* intensity = Find(fields, "intensity");
  if (intensity != nullptr) {
    switch (intensity->datatype) {
      case PointField::FLOAT32:
        layout.intensity = PointCloudLayout::IntensityKind::kFloat32;
        break;
      case PointField::UINT8:
        layout.intensity = PointCloudLayout::IntensityKind::kUint8;
        break;
      case PointField::UINT16:
        layout.intensity = PointCloudLayout::IntensityKind::kUint16;
        break;
      default:
        break;
    }
    if (layout.intensity != PointCloudLayout::IntensityKind::kNone) {
      if (!FieldFits(*intensity, cloud.point_step, &layout.error)) return layout;
      layout.intensity_offset = intensity->offset;
    }
  }

  layout.ok = true;
  return layout;
}

c3d::PointCloudInput ExtractPointCloud(const sensor_msgs::msg::PointCloud2& cloud,
                                       const PointCloudLayout& layout, bool want_color,
                                       bool want_intensity, PoseConvention convention) {
  c3d::PointCloudInput input;
  if (!layout.ok) return input;

  const std::size_t row_stride = RowStride(cloud);
  const std::size_t count = ReadablePoints(cloud, row_stride);
  if (count == 0) return input;

  const bool take_color = want_color && layout.color != PointCloudLayout::ColorKind::kNone;
  const bool take_intensity =
      want_intensity && layout.intensity != PointCloudLayout::IntensityKind::kNone;

  input.xyz.resize(count * 3);
  if (take_color) input.rgb.resize(count * 3);
  if (take_intensity) input.intensity.resize(count);
  // Anything but FLOAT32 came out of an integer field, which decides the encoder's
  // scaling rule -- and cannot be recovered from the values themselves.
  input.intensity_is_integral =
      take_intensity && layout.intensity != PointCloudLayout::IntensityKind::kFloat32;

  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t* point = PointAt(cloud, row_stride, i);
    const c3d::Vec3 ros{Read<float>(point, layout.x_offset), Read<float>(point, layout.y_offset),
                        Read<float>(point, layout.z_offset)};
    // The SAME map the pose's position channel gets, so a reader reconstructs a
    // world point as rotate(pose.q, p) + pose.position and never has to know
    // which convention the robot published in.
    const c3d::Vec3 scene = ConvertPosition(ros, convention);
    input.xyz[i * 3 + 0] = static_cast<float>(scene.x);
    input.xyz[i * 3 + 1] = static_cast<float>(scene.y);
    input.xyz[i * 3 + 2] = static_cast<float>(scene.z);

    if (take_color) {
      std::uint32_t packed = 0;
      switch (layout.color) {
        case PointCloudLayout::ColorKind::kPackedFloat32: {
          const float as_float = Read<float>(point, layout.color_offset);
          std::memcpy(&packed, &as_float, sizeof(packed));
          break;
        }
        case PointCloudLayout::ColorKind::kPackedUint32:
          packed = Read<std::uint32_t>(point, layout.color_offset);
          break;
        case PointCloudLayout::ColorKind::kThreeBytes:
          input.rgb[i * 3 + 0] = point[layout.color_offset];
          input.rgb[i * 3 + 1] = point[layout.g_offset];
          input.rgb[i * 3 + 2] = point[layout.b_offset];
          break;
        case PointCloudLayout::ColorKind::kNone:
          break;
      }
      if (layout.color != PointCloudLayout::ColorKind::kThreeBytes) {
        input.rgb[i * 3 + 0] = static_cast<std::uint8_t>((packed >> 16) & 0xFF);
        input.rgb[i * 3 + 1] = static_cast<std::uint8_t>((packed >> 8) & 0xFF);
        input.rgb[i * 3 + 2] = static_cast<std::uint8_t>(packed & 0xFF);
      }
    }

    if (take_intensity) {
      switch (layout.intensity) {
        case PointCloudLayout::IntensityKind::kFloat32:
          input.intensity[i] = Read<float>(point, layout.intensity_offset);
          break;
        case PointCloudLayout::IntensityKind::kUint8:
          input.intensity[i] = static_cast<float>(point[layout.intensity_offset]);
          break;
        case PointCloudLayout::IntensityKind::kUint16:
          input.intensity[i] =
              static_cast<float>(Read<std::uint16_t>(point, layout.intensity_offset));
          break;
        case PointCloudLayout::IntensityKind::kNone:
          break;
      }
    }
  }
  return input;
}

PointCloudAxisSigns CountPointCloudAxisSigns(const sensor_msgs::msg::PointCloud2& cloud,
                                             const PointCloudLayout& layout) {
  PointCloudAxisSigns signs;
  if (!layout.ok) return signs;

  const std::size_t row_stride = RowStride(cloud);
  const std::size_t count = ReadablePoints(cloud, row_stride);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t* point = PointAt(cloud, row_stride, i);
    const float x = Read<float>(point, layout.x_offset);
    const float y = Read<float>(point, layout.y_offset);
    const float z = Read<float>(point, layout.z_offset);
    // A depth camera returns a NaN or an infinity wherever it saw nothing, and an
    // organised cloud is mostly those. Counting them would put the fractions
    // wherever the scene happened to be empty.
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    ++signs.finite;
    if (x > 0.0f) ++signs.x_positive;
    if (z > 0.0f) ++signs.z_positive;
  }
  return signs;
}

}  // namespace cognitive3d_ros
