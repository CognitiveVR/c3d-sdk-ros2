#pragma once

#include <string>

#include "c3d/types.hpp"

namespace cognitive3d_ros {

// Which convention the poses on the wire are expressed in.
//
// The platform stores poses verbatim and its viewer is Y-up, so a Z-up pose
// renders on its side. Read back from the API, the same physical path stored two
// ways:
//
//   rep103:  (0,0,0.3) -> (5,0,0.3) -> (5,5,0.3)
//   unity:   (0,0.3,0) -> (0,0.3,5) -> (-5,0.3,5)
//
// In a Y-up viewer the rep103 path sends its second leg 5 metres straight up.
// kRep103 is kept because it is the honest debugging view and matches what
// `ros2 topic echo` shows, but it is not what the dashboard wants.
enum class PoseConvention {
  // Emit REP-103 native: right-handed, Z up, X forward, exactly what
  // `ros2 topic echo` shows a field engineer. Useful for debugging, but the
  // platform will render it lying on its side.
  kRep103,
  // Emit Unity convention: left-handed, Y up, Z forward, the frame the platform's
  // viewer renders upright.
  kUnity,
  // For a scene and dynamic-object mesh authored by a ROS -> glTF exporter that
  // maps `gltf.xyz = (ros.x, ros.z, -ros.y)`.
  //
  // kUnity is the canonical ROS -> Unity map and is correct whenever the scene
  // was authored to match it. It is NOT interchangeable with this one: the two
  // position maps differ by a 90 degree yaw, so telemetry sent under kUnity
  // against a scene authored the other way renders a trajectory that does not
  // lie on the floor plan, and a wheeled robot's pure yaw renders as a roll.
  // Nothing in the payload reveals it -- the platform stores poses verbatim, and
  // every gaze rotation is read the way the platform reads one under either
  // value, because that reading relates a payload to itself and is invariant
  // under a rotation of the whole world frame.
  //
  // Because the exporter's map sends ROS forward (+X) to -X rather than to the
  // +Z the platform reads a gaze rotation's forward along, this convention is the reason the
  // rotation conversion is per-CHANNEL rather than single: see
  // ConvertGazeRotation and ConvertDynamicRotation.
  kGltfAuthored,
};

bool ParsePoseConvention(const std::string& text, PoseConvention* out);
const char* PoseConventionName(PoseConvention c);

// ROS REP-103 (X forward, Y left, Z up, right-handed) to Unity
// (X right, Y up, Z forward, left-handed).
//
// From Unity's own ROS-TCP-Connector CoordinateSpaces.cs, cross-checked against
// ros-sharp:
//
//   position:         unity.xyz  = ( -ros.y,  ros.z,  ros.x )
//   quaternion:       unity.xyzw = ( -ros.y,  ros.z,  ros.x, -ros.w )
//   angular velocity: unity      = (  ros.y, -ros.z, -ros.x )
//
// The `-w` is coupled to the axis sign choice above it. Overall quaternion
// negation is a rotation no-op, so (-y, z, x, -w) and (y, -z, -x, +w) are the
// same rotation; ros-sharp publishes the second form. Either is correct. Mixing
// halves of the two is not, and the result looks almost right and survives
// casual testing, which is why there is a 90 degree yaw test.
c3d::Vec3 RosToUnityPosition(const c3d::Vec3& ros);
c3d::Quat RosToUnityRotation(const c3d::Quat& ros);

// Angular velocity, torque and magnetic field are pseudovectors and take an
// extra overall sign relative to the position transform. Linear velocity,
// acceleration and force do not.
//
// NOTE: nothing in the node calls this today. The only angular-velocity consumer,
// translators/odometry_translator.cpp, reads twist.angular.z as a raw scalar
// sensor and never converts a vector. Its tests therefore prove the formula, not
// that anything ships it.
c3d::Vec3 RosToUnityAngularVelocity(const c3d::Vec3& ros);

// kGltfAuthored. Position is the exporter's ROS -> glTF map with X mirrored,
// because the replay viewer negates X on every telemetry position at load while
// drawing the scene mesh unchanged:
//
//   position: gltf_authored.xyz = ( -ros.x, ros.z, -ros.y )
//
// The two rotations do NOT agree, and that is the point of splitting them.
//
// GAZE answers to how the platform reads a gaze rotation, as a headset's: local
// +Z forward and +Y up, which is how Session Replay turns the robot it draws from
// the pose timeline. The position map above sends ROS forward (+X) to
// -X and ROS up (+Z) to +Y, so the up axis already lands correctly and the
// forward axis is 90 degrees out. The fix is one fixed turn about the body Y
// axis, composed onto the position map's own basis change:
//
//   gaze.xyzw = basis(ros) * Ry(-90deg),  basis(q) = ( q.x, -q.z, q.y, q.w )
//
// written out in closed form below so it costs no trigonometry.
//
// DYNAMICS answers to the mesh's authoring frame and to the fact that the
// position map above is a REFLECTION. The mesh is authored in glTF axes, and the
// position map's determinant is -1, so the rotation must be carried across the
// same mirror -- S R S^-1 with S = diag(-1, 1, 1), i.e. (x, y, z, w) ->
// (x, -y, -z, w), which no component permutation of `ros` alone can express:
//
//   dynamics.xyzw = mirrorX( rosToGltf(ros) ) = ( q.x, -q.z, q.y, q.w )
//
// The replay viewer applies that conjugation back, so the rendered orientation
// is the mesh's glTF-frame orientation at its glTF-frame position -- one frame,
// both channels. For a pure ROS yaw (0, 0, s, c) this is (0, -s, 0, c), a turn
// about glTF +Y, which is yaw in a Y-up viewer, so the mesh stays parallel to
// the floor at every heading.
//
// Two neighbouring values are wrong, and both look plausible. swizzle(rosToGltf(ros)) =
// ( q.x, -q.y, q.z, q.w ) renders a wheeled robot's yaw as a ROLL. Plain
// rosToGltf(ros) = ( q.x, q.z, -q.y, q.w ) renders level but 180 degrees off its
// own path, because it leaves the rotation unmirrored while the position is
// mirrored. The derivation is in docs/coordinates.md.
//
// This is not the position map's basis change and not the gaze rule. A single
// rule cannot serve both channels here.
//
// WHY kUnity GETS AWAY WITH ONE RULE IS ONLY HALF ANSWERED, and the honest
// statement is narrower than it looks. For the GAZE channel the reason is solid:
// the canonical map sends ROS forward to +Z and ROS up to +Y, which is exactly
// how a gaze rotation is read, so no extra turn is needed. For the DYNAMICS
// channel there is no such argument, because the dynamics rule is decided by the
// mesh's authoring frame and kUnity's presumed authoring frame is not specified
// anywhere. kUnity is verified on POSITIONS read back from the API, not on
// dynamic-object rotations.
//
// kUnity's dynamics rule is UNVERIFIED, and the kGltfAuthored derivation above
// makes it more suspect rather than less. kUnity sends an unswizzled
// RosToUnityRotation, which the viewer conjugates by its X mirror and renders;
// whether that lands level depends entirely on the authoring frame of whatever
// mesh a kUnity consumer uploaded. The regression test that pins kUnity's
// dynamics rule pins the current behaviour, not its correctness.
c3d::Vec3 RosToGltfAuthoredPosition(const c3d::Vec3& ros);
c3d::Quat RosToGltfAuthoredGazeRotation(const c3d::Quat& ros);
c3d::Quat RosToGltfAuthoredDynamicRotation(const c3d::Quat& ros);

// Applied at the single point where poses enter the session model, so a
// convention change is one configuration value rather than an audit.
//
// The rotation conversion is per-channel because the channels answer to different
// contracts -- the platform's wire contract for gaze, the viewer's render frame
// plus the mesh's authoring for dynamics, and a reader holding unconverted rays
// for a scan plane. They coincide under kUnity and kRep103 except for the scan
// plane, which differs wherever the position map is a reflection.
c3d::Vec3 ConvertPosition(const c3d::Vec3& ros, PoseConvention convention);
c3d::Quat ConvertGazeRotation(const c3d::Quat& ros, PoseConvention convention);
c3d::Quat ConvertDynamicRotation(const c3d::Quat& ros, PoseConvention convention);
c3d::Vec3 ConvertAngularVelocity(const c3d::Vec3& ros, PoseConvention convention);

// The rotation a 2D LaserScan record carries, which is NOT the dynamic rule.
//
// A scan's rays stay on the wire as the sensor published them -- an angle and a
// range -- so a reader builds every endpoint as `p = (r cos t, r sin t, 0)` in the
// sensor's own XY plane. What it needs from the record is a rotation carrying THAT
// naive plane frame into the scene, and neither rotation above is one: under
// kUnity and kGltfAuthored the position map M is a reflection, so the honest
// sensor-to-scene map `M R` has determinant -1 and is not a rotation at all.
// ConvertDynamicRotation sends `M R M^-1`, which lands the rays correctly only for
// a reader that has already applied M to each of them itself -- and M is knowable
// from the session property `c3d.ros.pose_convention`, never from the record.
//
// A 2D scan lies in one plane, so the axis it does not use can absorb the mirror.
// Q is the proper rotation that agrees with `M R` on that plane:
//
//   Q e_x = M R e_x,   Q e_y = M R e_y,   Q e_z = (M R e_x) x (M R e_y)
//
// M is orthogonal, so those two columns stay orthonormal and Q is a rotation; it
// differs from `M R` in the third column's sign alone, which is the component
// every ray carries as zero. `world = rotate(Q, (r cos t, r sin t, 0)) + M t` then
// holds under every convention with nothing to look up.
//
// The cost is that a laserscan pose and a camera pose on the SAME tf frame are
// different numbers wherever M is a reflection, and that is the right way round:
// the pose is what makes a record self-describing, and the laserscan is the one
// record carrying coordinates this SDK never converted.
c3d::Quat ConvertScanPlaneRotation(const c3d::Quat& ros, PoseConvention convention);

}  // namespace cognitive3d_ros
