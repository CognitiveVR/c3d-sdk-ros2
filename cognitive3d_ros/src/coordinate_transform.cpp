#include "cognitive3d_ros/coordinate_transform.hpp"

#include <cmath>

namespace cognitive3d_ros {
namespace {

// sqrt(2)/2, the half-angle sine and cosine of the fixed 90 degree turn the
// gaze channel needs under kGltfAuthored.
constexpr double kSqrtHalf = 0.70710678118654752440;

// A rotation given by its COLUMNS, as a quaternion. Shepperd's branches: each one
// divides by a different component, and the branch taken is the one whose
// component is largest, so the division is never near zero -- the single-formula
// version loses all precision at a half turn, where w is 0.
c3d::Quat QuaternionFromColumns(const c3d::Vec3& c0, const c3d::Vec3& c1, const c3d::Vec3& c2) {
  const double trace = c0.x + c1.y + c2.z;
  c3d::Quat q;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q = c3d::Quat{(c1.z - c2.y) / s, (c2.x - c0.z) / s, (c0.y - c1.x) / s, 0.25 * s};
  } else if (c0.x > c1.y && c0.x > c2.z) {
    const double s = std::sqrt(1.0 + c0.x - c1.y - c2.z) * 2.0;
    q = c3d::Quat{0.25 * s, (c1.x + c0.y) / s, (c2.x + c0.z) / s, (c1.z - c2.y) / s};
  } else if (c1.y > c2.z) {
    const double s = std::sqrt(1.0 + c1.y - c0.x - c2.z) * 2.0;
    q = c3d::Quat{(c1.x + c0.y) / s, 0.25 * s, (c2.y + c1.z) / s, (c2.x - c0.z) / s};
  } else {
    const double s = std::sqrt(1.0 + c2.z - c0.x - c1.y) * 2.0;
    q = c3d::Quat{(c2.x + c0.z) / s, (c2.y + c1.z) / s, 0.25 * s, (c0.y - c1.x) / s};
  }
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm == 0.0) return c3d::Quat{};
  return c3d::Quat{q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

}  // namespace

bool ParsePoseConvention(const std::string& text, PoseConvention* out) {
  if (text == "rep103" || text == "ros") { *out = PoseConvention::kRep103; return true; }
  if (text == "unity") { *out = PoseConvention::kUnity; return true; }
  if (text == "gltf_authored") { *out = PoseConvention::kGltfAuthored; return true; }
  return false;
}

const char* PoseConventionName(PoseConvention c) {
  switch (c) {
    case PoseConvention::kUnity: return "unity";
    case PoseConvention::kGltfAuthored: return "gltf_authored";
    case PoseConvention::kRep103: break;
  }
  return "rep103";
}

c3d::Vec3 RosToUnityPosition(const c3d::Vec3& r) {
  return c3d::Vec3{-r.y, r.z, r.x};
}

c3d::Quat RosToUnityRotation(const c3d::Quat& r) {
  return c3d::Quat{-r.y, r.z, r.x, -r.w};
}

c3d::Vec3 RosToUnityAngularVelocity(const c3d::Vec3& r) {
  return c3d::Vec3{r.y, -r.z, -r.x};
}

c3d::Vec3 RosToGltfAuthoredPosition(const c3d::Vec3& r) {
  return c3d::Vec3{-r.x, r.z, -r.y};
}

c3d::Quat RosToGltfAuthoredGazeRotation(const c3d::Quat& r) {
  // basis(q) * Ry(-90deg) with basis(q) = (q.x, -q.z, q.y, q.w), multiplied out.
  // Ry(-90) fixes the body +Y axis and carries +Z onto -X, which is precisely
  // the axis the position map leaves 90 degrees out.
  return c3d::Quat{kSqrtHalf * (r.x + r.y),
                   -kSqrtHalf * (r.z + r.w),
                   kSqrtHalf * (r.y - r.x),
                   kSqrtHalf * (r.w - r.z)};
}

c3d::Quat RosToGltfAuthoredDynamicRotation(const c3d::Quat& r) {
  // rosToGltf(r) = (r.x, r.z, -r.y, r.w), then conjugated by the X mirror that
  // RosToGltfAuthoredPosition applies. That position map has determinant -1, and
  // no quaternion can express a reflection: a rotation carried across a mirror S
  // becomes S R S^-1, which for S = diag(-1, 1, 1) is (x, y, z, w) ->
  // (x, -y, -z, w). Drop it and the model's heading tracks the mirror image of
  // the path its own positions follow.
  //
  // The consumer is the replay viewer, which applies exactly that conjugation
  // back (it negates qy and qz to match its X-axis reflection), so the rendered
  // orientation is the mesh's glTF-frame orientation. The derivation is in
  // docs/coordinates.md.
  return c3d::Quat{r.x, -r.z, r.y, r.w};
}

c3d::Vec3 ConvertPosition(const c3d::Vec3& ros, PoseConvention convention) {
  switch (convention) {
    case PoseConvention::kUnity: return RosToUnityPosition(ros);
    case PoseConvention::kGltfAuthored: return RosToGltfAuthoredPosition(ros);
    case PoseConvention::kRep103: break;
  }
  return ros;
}

c3d::Quat ConvertGazeRotation(const c3d::Quat& ros, PoseConvention convention) {
  switch (convention) {
    case PoseConvention::kUnity: return RosToUnityRotation(ros);
    case PoseConvention::kGltfAuthored: return RosToGltfAuthoredGazeRotation(ros);
    case PoseConvention::kRep103: break;
  }
  return ros;
}

c3d::Quat ConvertDynamicRotation(const c3d::Quat& ros, PoseConvention convention) {
  switch (convention) {
    case PoseConvention::kUnity: return RosToUnityRotation(ros);
    case PoseConvention::kGltfAuthored: return RosToGltfAuthoredDynamicRotation(ros);
    case PoseConvention::kRep103: break;
  }
  return ros;
}

c3d::Quat ConvertScanPlaneRotation(const c3d::Quat& ros, PoseConvention convention) {
  // The position map is the identity here, so the scan plane's rotation IS the
  // sensor's and the construction below could only round it.
  if (convention == PoseConvention::kRep103) return ros;

  const double norm = std::sqrt(ros.x * ros.x + ros.y * ros.y + ros.z * ros.z + ros.w * ros.w);
  if (norm == 0.0) return c3d::Quat{};
  const double x = ros.x / norm, y = ros.y / norm, z = ros.z / norm, w = ros.w / norm;

  // R's own first two columns: where the sensor's x and y axes point in ROS axes.
  // The third is never built -- it is the one the construction replaces.
  const c3d::Vec3 rx{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w)};
  const c3d::Vec3 ry{2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w)};

  // M R, column by column, through the position map itself rather than a second
  // copy of it: a scan endpoint reaches the scene as ConvertPosition(R p + t), so
  // anything that re-derived M here could drift from what the points get.
  const c3d::Vec3 c0 = ConvertPosition(rx, convention);
  const c3d::Vec3 c1 = ConvertPosition(ry, convention);
  const c3d::Vec3 c2{c0.y * c1.z - c0.z * c1.y, c0.z * c1.x - c0.x * c1.z,
                     c0.x * c1.y - c0.y * c1.x};
  return QuaternionFromColumns(c0, c1, c2);
}

c3d::Vec3 ConvertAngularVelocity(const c3d::Vec3& ros, PoseConvention convention) {
  // Pseudovector, so it takes the extra overall sign relative to position. Under
  // kGltfAuthored the position map is a reflection too, so the same rule holds
  // with that map's own axes.
  switch (convention) {
    case PoseConvention::kUnity: return RosToUnityAngularVelocity(ros);
    case PoseConvention::kGltfAuthored: return c3d::Vec3{ros.x, -ros.z, ros.y};
    case PoseConvention::kRep103: break;
  }
  return ros;
}

}  // namespace cognitive3d_ros
