#include <cmath>

#include "c3d/media.hpp"

namespace c3d {
namespace {

// Strictly inside (0, 180): 0 is the schema's "not recorded", and a full angle of
// 180 or more is not a pinhole camera's field of view, whatever the lens. NaN fails
// both comparisons and infinity the second, so neither needs a test of its own.
bool UsableAngle(float degrees) { return degrees > 0.0f && degrees < 180.0f; }

}  // namespace

bool IsKnownCameraFov(const CameraFov& fov) {
  return UsableAngle(fov.hfov_deg) && UsableAngle(fov.vfov_deg);
}

CameraFov CameraFovFromPinhole(double fx, double fy, double cx, double cy, double width,
                               double height) {
  for (const double v : {fx, fy, cx, cy, width, height}) {
    if (!std::isfinite(v)) return CameraFov{};
  }
  if (fx <= 0.0 || fy <= 0.0 || width <= 0.0 || height <= 0.0) return CameraFov{};
  constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;
  const CameraFov fov{
      static_cast<float>((std::atan(cx / fx) + std::atan((width - cx) / fx)) * kDegreesPerRadian),
      static_cast<float>((std::atan(cy / fy) + std::atan((height - cy) / fy)) *
                         kDegreesPerRadian)};
  return IsKnownCameraFov(fov) ? fov : CameraFov{};
}

}  // namespace c3d
