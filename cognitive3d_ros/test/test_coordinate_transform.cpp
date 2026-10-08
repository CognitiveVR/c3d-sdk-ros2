// The coordinate conversion is easy to get almost right, and "almost right"
// survives casual testing. These tests are the ones that catch it.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "cognitive3d_ros/coordinate_transform.hpp"

using namespace cognitive3d_ros;
using c3d::Quat;
using c3d::Vec3;

namespace {

// Rotate a vector by a quaternion, so we can assert on what the rotation
// actually does rather than on its components.
Vec3 Rotate(const Quat& q, const Vec3& v) {
  const double x = q.x, y = q.y, z = q.z, w = q.w;
  const double tx = 2.0 * (y * v.z - z * v.y);
  const double ty = 2.0 * (z * v.x - x * v.z);
  const double tz = 2.0 * (x * v.y - y * v.x);
  return Vec3{v.x + w * tx + (y * tz - z * ty),
              v.y + w * ty + (z * tx - x * tz),
              v.z + w * tz + (x * ty - y * tx)};
}

Quat YawRos(double radians) {
  return Quat{0.0, 0.0, std::sin(radians / 2.0), std::cos(radians / 2.0)};
}

Quat Normalized(double x, double y, double z, double w) {
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  return Quat{x / n, y / n, z / n, w / n};
}

// A mixed set: the identity, three planar rotations, and four genuinely
// non-planar ones. The non-planar entries are the point -- a wheeled robot on a
// flat floor only ever produces yaw, and a rotation rule that is wrong off that
// plane passes every planar fixture, so only the non-planar entries can tell it
// from a correct one. The planar entries stay because some assertions want both.
std::vector<Quat> Rotations() {
  return {Quat{0.0, 0.0, 0.0, 1.0},
          YawRos(30.0 * M_PI / 180.0),
          YawRos(-120.0 * M_PI / 180.0),
          Normalized(0.1, 0.2, 0.3, 0.927),
          Normalized(0.0, 0.3826, 0.0, 0.9239),
          Normalized(0.2, -0.3, 0.1, 0.927),
          Normalized(-0.5, 0.5, -0.5, 0.5),
          Normalized(0.37, -0.11, 0.62, 0.68)};
}

// What Session Replay DRAWS, from what we send. EXTERNAL FACTS about the viewer,
// not rules this SDK chooses, which is why the dynamics rule is pinned to them.
// Both are their own inverse.
//   every position                   p: [-p.x, p.y, p.z]
//   a dynamic object's rotation      negate qy and qz to match the X-axis reflection
Vec3 ViewerRenderPosition(const Vec3& p) { return Vec3{-p.x, p.y, p.z}; }
Quat ViewerRenderRotation(const Quat& q) { return Quat{q.x, -q.y, -q.z, q.w}; }

// The exporter's map, as a rotation: gltf.xyz = (ros.x, ros.z, -ros.y).
Quat RosToGltfRotation(const Quat& q) { return Quat{q.x, q.z, -q.y, q.w}; }

}  // namespace

TEST(CoordinateTransform, PositionAxesMatchUnityReference) {
  // unity.xyz = (-ros.y, ros.z, ros.x)
  const Vec3 unity = RosToUnityPosition(Vec3{1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(unity.x, -2.0);
  EXPECT_DOUBLE_EQ(unity.y, 3.0);
  EXPECT_DOUBLE_EQ(unity.z, 1.0);
}

TEST(CoordinateTransform, RosForwardBecomesUnityForward) {
  // ROS +X is forward. Unity +Z is forward. This must survive the mapping.
  const Vec3 unity = RosToUnityPosition(Vec3{1.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(unity.z, 1.0);
  EXPECT_DOUBLE_EQ(unity.x, 0.0);
  EXPECT_DOUBLE_EQ(unity.y, 0.0);
}

TEST(CoordinateTransform, RosLeftBecomesUnityLeft) {
  // ROS +Y is left. Unity +X is right, so left must be -X.
  const Vec3 unity = RosToUnityPosition(Vec3{0.0, 1.0, 0.0});
  EXPECT_DOUBLE_EQ(unity.x, -1.0);
}

TEST(CoordinateTransform, RosUpBecomesUnityUp) {
  const Vec3 unity = RosToUnityPosition(Vec3{0.0, 0.0, 1.0});
  EXPECT_DOUBLE_EQ(unity.y, 1.0);
}

TEST(CoordinateTransform, NinetyDegreeYawSurvivesTheHandednessFlip) {
  // THE test. A robot yawing 90 degrees to its left in ROS must, in Unity's
  // left-handed frame, end up pointing along Unity -X, which is Unity's left.
  //
  // Getting the axes right and the `w` sign wrong produces a mirrored rotation
  // that points 90 degrees the WRONG way while every component still looks
  // plausible. That is precisely what this catches.
  const Quat ros_yaw = YawRos(M_PI / 2.0);
  const Vec3 ros_forward_after = Rotate(ros_yaw, Vec3{1.0, 0.0, 0.0});
  // In ROS, yawing left turns +X into +Y.
  EXPECT_NEAR(ros_forward_after.x, 0.0, 1e-9);
  EXPECT_NEAR(ros_forward_after.y, 1.0, 1e-9);

  const Quat unity_yaw = RosToUnityRotation(ros_yaw);
  // Unity forward is +Z.
  const Vec3 unity_forward_after = Rotate(unity_yaw, Vec3{0.0, 0.0, 1.0});
  // The same physical heading, expressed in Unity: robot-left is Unity -X.
  const Vec3 expected = RosToUnityPosition(ros_forward_after);
  EXPECT_NEAR(unity_forward_after.x, expected.x, 1e-9);
  EXPECT_NEAR(unity_forward_after.y, expected.y, 1e-9);
  EXPECT_NEAR(unity_forward_after.z, expected.z, 1e-9);
  EXPECT_NEAR(unity_forward_after.x, -1.0, 1e-9) << "yaw came out mirrored";
}

TEST(CoordinateTransform, RotationIsConsistentWithPositionForManyAngles) {
  // Generalises the 90 degree case: for any yaw, rotating the ROS forward axis
  // and then converting must equal converting and then rotating.
  for (int deg = -180; deg <= 180; deg += 15) {
    const double rad = deg * M_PI / 180.0;
    const Quat ros_q = YawRos(rad);
    const Vec3 ros_result = Rotate(ros_q, Vec3{1.0, 0.0, 0.0});
    const Vec3 via_position = RosToUnityPosition(ros_result);
    const Vec3 via_rotation = Rotate(RosToUnityRotation(ros_q), Vec3{0.0, 0.0, 1.0});
    EXPECT_NEAR(via_rotation.x, via_position.x, 1e-9) << "yaw " << deg;
    EXPECT_NEAR(via_rotation.y, via_position.y, 1e-9) << "yaw " << deg;
    EXPECT_NEAR(via_rotation.z, via_position.z, 1e-9) << "yaw " << deg;
  }
}

TEST(CoordinateTransform, IdentityStaysIdentity) {
  const Quat unity = RosToUnityRotation(Quat{0, 0, 0, 1});
  // (0,0,0,-1) is the same rotation as identity; assert on behaviour, not bits.
  const Vec3 v = Rotate(unity, Vec3{1.0, 2.0, 3.0});
  EXPECT_NEAR(v.x, 1.0, 1e-9);
  EXPECT_NEAR(v.y, 2.0, 1e-9);
  EXPECT_NEAR(v.z, 3.0, 1e-9);
}

TEST(CoordinateTransform, AngularVelocityTakesTheExtraPseudovectorSign) {
  // Angular velocity, torque and magnetic field are pseudovectors and flip
  // relative to the position transform. Linear quantities do not.
  const Vec3 ros{1.0, 2.0, 3.0};
  const Vec3 linear = RosToUnityPosition(ros);
  const Vec3 angular = RosToUnityAngularVelocity(ros);
  EXPECT_DOUBLE_EQ(angular.x, -linear.x);
  EXPECT_DOUBLE_EQ(angular.y, -linear.y);
  EXPECT_DOUBLE_EQ(angular.z, -linear.z);
}

TEST(CoordinateTransform, Rep103ConventionIsAPassthrough) {
  const Vec3 p{1.0, 2.0, 3.0};
  const Quat q{0.1, 0.2, 0.3, 0.9};
  const Vec3 cp = ConvertPosition(p, PoseConvention::kRep103);
  const Quat gaze = ConvertGazeRotation(q, PoseConvention::kRep103);
  const Quat dyn = ConvertDynamicRotation(q, PoseConvention::kRep103);
  EXPECT_DOUBLE_EQ(cp.x, p.x);
  EXPECT_DOUBLE_EQ(cp.y, p.y);
  EXPECT_DOUBLE_EQ(cp.z, p.z);
  // All four components on BOTH channels. Asserting .w alone lets a mutant that
  // returns the gltf dynamic rule (x, -y, z, w) survive, which would ship
  // y-negated quaternions on the one setting an engineer picks BECAUSE they want
  // the wire to match `ros2 topic echo`.
  EXPECT_DOUBLE_EQ(gaze.x, q.x);
  EXPECT_DOUBLE_EQ(gaze.y, q.y);
  EXPECT_DOUBLE_EQ(gaze.z, q.z);
  EXPECT_DOUBLE_EQ(gaze.w, q.w);
  EXPECT_DOUBLE_EQ(dyn.x, q.x);
  EXPECT_DOUBLE_EQ(dyn.y, q.y);
  EXPECT_DOUBLE_EQ(dyn.z, q.z);
  EXPECT_DOUBLE_EQ(dyn.w, q.w);
}

TEST(CoordinateTransform, ParseRejectsGarbage) {
  PoseConvention out;
  EXPECT_TRUE(ParsePoseConvention("rep103", &out));
  EXPECT_EQ(out, PoseConvention::kRep103);
  EXPECT_TRUE(ParsePoseConvention("unity", &out));
  EXPECT_EQ(out, PoseConvention::kUnity);
  EXPECT_TRUE(ParsePoseConvention("gltf_authored", &out));
  EXPECT_EQ(out, PoseConvention::kGltfAuthored);
  EXPECT_FALSE(ParsePoseConvention("left_handed_ish", &out));
  EXPECT_STREQ(PoseConventionName(PoseConvention::kGltfAuthored), "gltf_authored");
  EXPECT_STREQ(PoseConventionName(PoseConvention::kUnity), "unity");
  EXPECT_STREQ(PoseConventionName(PoseConvention::kRep103), "rep103");
}

// --- kGltfAuthored -----------------------------------------------------------

TEST(GltfAuthoredConvention, PositionMirrorsXOfTheExportersGltfPoint) {
  // The viewer negates X on every telemetry position at load and draws the
  // scene mesh unchanged, so a telemetry position has to be pre-negated.
  const Vec3 p = RosToGltfAuthoredPosition(Vec3{1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(p.x, -1.0);
  EXPECT_DOUBLE_EQ(p.y, 3.0);
  EXPECT_DOUBLE_EQ(p.z, -2.0);
}

TEST(GltfAuthoredConvention, TheGazeContractHoldsOffThePlanarCase) {
  // The platform reads a gaze rotation's local +Z as the direction the robot faces.
  // The gaze pose the sampler produces is body-style: forward +X, up +Z.
  for (const Quat& ros : Rotations()) {
    const Quat sent = ConvertGazeRotation(ros, PoseConvention::kGltfAuthored);
    const Vec3 want = ConvertPosition(Rotate(ros, Vec3{1.0, 0.0, 0.0}),
                                      PoseConvention::kGltfAuthored);
    const Vec3 got = Rotate(sent, Vec3{0.0, 0.0, 1.0});
    EXPECT_NEAR(got.x, want.x, 1e-9);
    EXPECT_NEAR(got.y, want.y, 1e-9);
    EXPECT_NEAR(got.z, want.z, 1e-9);
  }
}

TEST(GltfAuthoredConvention, TheGazeUpAxisIsAlsoCorrect) {
  // +Y up is the other half of the contract, and the half the forward check
  // cannot see: a roll about the forward axis satisfies the forward check
  // exactly while rendering the session upside down.
  for (const Quat& ros : Rotations()) {
    const Quat sent = ConvertGazeRotation(ros, PoseConvention::kGltfAuthored);
    const Vec3 want = ConvertPosition(Rotate(ros, Vec3{0.0, 0.0, 1.0}),
                                      PoseConvention::kGltfAuthored);
    const Vec3 got = Rotate(sent, Vec3{0.0, 1.0, 0.0});
    EXPECT_NEAR(got.x, want.x, 1e-9);
    EXPECT_NEAR(got.y, want.y, 1e-9);
    EXPECT_NEAR(got.z, want.z, 1e-9);
  }
}

TEST(GltfAuthoredConvention, GazeRotationStaysAUnitQuaternion) {
  for (const Quat& ros : Rotations()) {
    const Quat q = RosToGltfAuthoredGazeRotation(ros);
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    EXPECT_NEAR(n, 1.0, 1e-12);
  }
}

TEST(GltfAuthoredConvention, DynamicRotationIsTheMeshOrientationMirroredLikeThePositions) {
  // The viewer does NOT swizzle a dynamic object's rotation, so the sent value is
  // the mesh's own orientation -- CONJUGATED BY THE POSITION MAP'S X MIRROR,
  // because RosToGltfAuthoredPosition is a reflection (determinant -1) and a
  // rotation that ignores it turns the opposite way to the path it accompanies.
  //
  // Two neighbouring values are wrong and must not be asserted here: that the
  // viewer's y/z swizzle of the sent value equals the mesh orientation, and that
  // the sent value equals it unmirrored. See the header.
  //
  // Goes through ConvertDynamicRotation, not the free function: calling the free
  // function proves the formula and leaves the dispatch untested, and deleting
  // the kGltfAuthored case from that switch would then keep the suite green
  // while every dynamic-object rotation went on the wire as raw ROS.
  for (const Quat& ros : Rotations()) {
    const Quat sent = ConvertDynamicRotation(ros, PoseConvention::kGltfAuthored);
    const Quat mesh = RosToGltfRotation(ros);
    const Quat want{mesh.x, -mesh.y, -mesh.z, mesh.w};   // conjugated by the X mirror
    EXPECT_NEAR(sent.x, want.x, 1e-12);
    EXPECT_NEAR(sent.y, want.y, 1e-12);
    EXPECT_NEAR(sent.z, want.z, 1e-12);
    EXPECT_NEAR(sent.w, want.w, 1e-12);
  }
}

TEST(GltfAuthoredConvention, TheDynamicHeadingTurnsWITHThePathNotAgainstIt) {
  // The defect no fixed yaw offset can fix. The position map
  // mirrors X, so a rotation not conjugated by the same mirror makes the model's
  // heading track the MIRROR IMAGE of the robot's, and a mirror is not a
  // rotation: no constant offset composes with it.
  //
  // Asserted as "the two turn the same way": as ROS yaw increases, the rendered
  // heading must move consistently in ONE direction. A mirrored rotation moves
  // it the other way, which no constant offset can repair.
  double previous = 0.0;
  bool have_previous = false;
  int increasing = 0, decreasing = 0;
  for (int degrees = -170; degrees <= 170; degrees += 10) {
    const double half = degrees * M_PI / 180.0 / 2.0;
    const Quat ros{0.0, 0.0, std::sin(half), std::cos(half)};
    const Vec3 forward = Rotate(ConvertDynamicRotation(ros, PoseConvention::kGltfAuthored),
                                Vec3{1.0, 0.0, 0.0});
    const double heading = std::atan2(forward.z, forward.x);
    if (have_previous) {
      double step = heading - previous;
      while (step > M_PI) step -= 2 * M_PI;
      while (step < -M_PI) step += 2 * M_PI;
      if (step > 0) ++increasing; else if (step < 0) ++decreasing;
    }
    previous = heading;
    have_previous = true;
  }
  // Monotone in one direction. Mixed signs would mean the heading is not a
  // consistent function of yaw at all; a uniformly opposite sign is the mirror.
  EXPECT_TRUE(increasing == 0 || decreasing == 0)
      << "heading is not monotone in yaw: " << increasing << " up, " << decreasing << " down";
  // For a pure ROS yaw t the sent value is (0, -sin(t/2), 0, cos(t/2)), a turn of
  // -t about glTF +Y, and R_y(-t) . (1,0,0) = (cos t, 0, sin t) -- so the heading
  // atan2(z, x) equals t and INCREASES with yaw. Derived, not read off a run: the
  // direction is what the test is for, so taking it from a measurement would make
  // the test agree with whatever ships.
  EXPECT_GT(increasing, 0) << "the rendered heading must advance with ROS yaw, not against it";
}

TEST(GltfAuthoredConvention, AWheeledRobotsYawRendersParallelToTheFloor) {
  // THE PROPERTY A MIRRORED RULE VIOLATES, asserted directly rather than as a
  // formula. A robot on a flat floor produces pure yaw about ROS +Z; whatever
  // rotation we send must leave the mesh's up axis pointing at world up, or the
  // model tips -- and a tipped cylinder whose origin is at floor level also
  // pushes part of itself under the floor.
  //
  // Stated as an invariant on the rendered up axis, not as an Euler triple:
  // decomposing these quaternions to roll/pitch/yaw reports 180 and 89.988
  // degrees at gimbal lock for rotations that are in fact perfectly level, so an
  // Euler-based assertion would fail on correct data and pass on a lie.
  for (int degrees = -180; degrees <= 180; degrees += 5) {
    const double half = degrees * M_PI / 180.0 / 2.0;
    const Quat ros{0.0, 0.0, std::sin(half), std::cos(half)};   // yaw about ROS +Z
    const Vec3 up = Rotate(ConvertDynamicRotation(ros, PoseConvention::kGltfAuthored),
                           Vec3{0.0, 1.0, 0.0});                // glTF up
    EXPECT_NEAR(up.x, 0.0, 1e-12) << "tipped at " << degrees << " degrees of yaw";
    EXPECT_NEAR(up.y, 1.0, 1e-12) << "tipped at " << degrees << " degrees of yaw";
    EXPECT_NEAR(up.z, 0.0, 1e-12) << "tipped at " << degrees << " degrees of yaw";
  }
}

TEST(GltfAuthoredConvention, TheRenderedNoseFollowsTheRenderedPath) {
  // THE WHOLE POINT, stated once in the frame a viewer actually sees, with BOTH
  // halves of the viewer's transform applied. Drive one metre along the chassis's
  // own forward axis from an arbitrary orientation: the displacement the viewer
  // DRAWS must be the direction it draws the nose in.
  //
  // Every other dynamics test here compares a rotation against a rotation. None
  // of them ever puts a rendered POSITION and a rendered ROTATION in the same
  // equation, so a rule that mirrors one and not the other satisfies all of them
  // -- and that is exactly the defect this one exists to catch.
  //
  // What this test CANNOT catch, measured rather than assumed: both viewer
  // transforms are involutions, so putting a y/z swizzle in the wire rule and in
  // ViewerRenderRotation together leaves this green. The property below,
  // and the two absolute assertions on the wire value, are what fire on that.
  for (const Quat& ros : Rotations()) {
    const Vec3 start{2.5, -1.25, 0.0};
    const Vec3 chassis_forward = Rotate(ros, Vec3{1.0, 0.0, 0.0});
    const Vec3 end{start.x + chassis_forward.x, start.y + chassis_forward.y,
                   start.z + chassis_forward.z};

    const Vec3 a = ViewerRenderPosition(ConvertPosition(start, PoseConvention::kGltfAuthored));
    const Vec3 b = ViewerRenderPosition(ConvertPosition(end, PoseConvention::kGltfAuthored));
    const Vec3 travel{b.x - a.x, b.y - a.y, b.z - a.z};

    const Quat sent = ConvertDynamicRotation(ros, PoseConvention::kGltfAuthored);
    const Vec3 nose = Rotate(ViewerRenderRotation(sent), Vec3{1.0, 0.0, 0.0});

    EXPECT_NEAR(nose.x, travel.x, 1e-9);
    EXPECT_NEAR(nose.y, travel.y, 1e-9);
    EXPECT_NEAR(nose.z, travel.z, 1e-9);
  }
}

TEST(GltfAuthoredConvention, TheViewersTwoHalvesDescribeOneFrameChange) {
  // What ties the rotation transform to the position transform, and the only
  // property here that pins either to something other than a literal: whatever
  // the viewer does to points, rotating a transformed point by the transformed
  // rotation must equal transforming the rotated point.
  //
  // A y/z swap models a DIFFERENT reflection and fails this. Verified by
  // mutation: swapping both the wire rule and ViewerRenderRotation to it fails
  // this test, AWheeledRobotsYawRendersParallelToTheFloor and
  // TheDynamicHeadingTurnsWITHThePathNotAgainstIt, and nothing else.
  const Vec3 probes[] = {Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0},
                         Vec3{0.4, -0.7, 0.2}};
  for (const Quat& q : Rotations()) {
    for (const Vec3& v : probes) {
      const Vec3 got = Rotate(ViewerRenderRotation(q), ViewerRenderPosition(v));
      const Vec3 want = ViewerRenderPosition(Rotate(q, v));
      EXPECT_NEAR(got.x, want.x, 1e-12);
      EXPECT_NEAR(got.y, want.y, 1e-12);
      EXPECT_NEAR(got.z, want.z, 1e-12);
    }
  }
}

TEST(GltfAuthoredConvention, TheTwoChannelsAreGenuinelyDifferentRules) {
  // A guard against unifying them. One rule cannot serve both here, and the
  // failure is silent: the gaze contract is satisfied either way because it
  // relates a payload to itself.
  int differ = 0;
  for (const Quat& ros : Rotations()) {
    const Quat gaze = ConvertGazeRotation(ros, PoseConvention::kGltfAuthored);
    const Quat dyn = ConvertDynamicRotation(ros, PoseConvention::kGltfAuthored);
    const Vec3 a = Rotate(gaze, Vec3{1.0, 2.0, 3.0});
    const Vec3 b = Rotate(dyn, Vec3{1.0, 2.0, 3.0});
    if (std::abs(a.x - b.x) > 1e-9 || std::abs(a.y - b.y) > 1e-9 ||
        std::abs(a.z - b.z) > 1e-9) {
      ++differ;
    }
  }
  EXPECT_EQ(differ, static_cast<int>(Rotations().size()));
}

TEST(GltfAuthoredConvention, ItIsNotInterchangeableWithUnity) {
  // The two position maps differ by a 90 degree yaw. Sending one against a
  // scene authored for the other puts the trajectory off the floor plan, and
  // nothing in the payload says so.
  const Vec3 ros{1.0, 0.0, 0.0};
  const Vec3 unity = ConvertPosition(ros, PoseConvention::kUnity);
  const Vec3 gltf = ConvertPosition(ros, PoseConvention::kGltfAuthored);
  EXPECT_DOUBLE_EQ(unity.x, 0.0);
  EXPECT_DOUBLE_EQ(unity.z, 1.0);
  EXPECT_DOUBLE_EQ(gltf.x, -1.0);
  EXPECT_DOUBLE_EQ(gltf.z, 0.0);
  // Up is the one axis they agree on, which is why "it renders Y-up" is no
  // evidence at all about the horizontal plane.
  EXPECT_DOUBLE_EQ(ConvertPosition(Vec3{0.0, 0.0, 1.0}, PoseConvention::kUnity).y, 1.0);
  EXPECT_DOUBLE_EQ(ConvertPosition(Vec3{0.0, 0.0, 1.0}, PoseConvention::kGltfAuthored).y, 1.0);
}

TEST(GltfAuthoredConvention, AngularVelocityTakesThePseudovectorSign) {
  // `angular == -linear` is the pseudovector rule `v' = det(P) * P * v` ONLY
  // because det(P) = -1 for this map, as it is for unity's. Copying this shape
  // to a future det = +1 map would encode the wrong rule under a reassuring
  // name, so the axes are also asserted literally below.
  //
  // Several vectors, not one: on a single vector with distinct magnitudes a
  // scaling mutant such as {ros.x, -1.5 * ros.y, ros.y} still agrees.
  for (const Vec3& ros : {Vec3{1.0, 2.0, 3.0}, Vec3{1.0, 1.0, 1.0},
                          Vec3{-2.5, 0.0, 4.0}, Vec3{0.0, 7.0, -1.0}}) {
    const Vec3 linear = ConvertPosition(ros, PoseConvention::kGltfAuthored);
    const Vec3 angular = ConvertAngularVelocity(ros, PoseConvention::kGltfAuthored);
    EXPECT_DOUBLE_EQ(angular.x, -linear.x);
    EXPECT_DOUBLE_EQ(angular.y, -linear.y);
    EXPECT_DOUBLE_EQ(angular.z, -linear.z);
    // det(P) = -1, so v' = -P v = (x, -z, y).
    EXPECT_DOUBLE_EQ(angular.x, ros.x);
    EXPECT_DOUBLE_EQ(angular.y, -ros.z);
    EXPECT_DOUBLE_EQ(angular.z, ros.y);
  }
}

TEST(CoordinateTransform, UnityRotationAgreesWithItsPositionMapOffThePlane) {
  // The one test that can actually SEE RosToUnityRotation being wrong.
  //
  // Every other unity rotation test uses pure yaw or the identity, where
  // r.x == r.y == 0 -- so the two components that carry roll and pitch are never
  // exercised, and (-y, z, -x, -w), (+y, z, x, -w) and (-x, z, y, -w) are all
  // exact on yaw while being ~175 degrees wrong off it. And
  // UnityBehaviourIsUnchanged below compares the dispatch against
  // RosToUnityRotation itself, so it cannot see a change to that function.
  //
  // This asserts the rotation against the POSITION map instead, which is
  // independent of it, on both axes the platform contract names.
  for (const Quat& ros : Rotations()) {
    const Quat sent = ConvertGazeRotation(ros, PoseConvention::kUnity);
    const Vec3 want_fwd = ConvertPosition(Rotate(ros, Vec3{1.0, 0.0, 0.0}),
                                          PoseConvention::kUnity);
    const Vec3 got_fwd = Rotate(sent, Vec3{0.0, 0.0, 1.0});
    EXPECT_NEAR(got_fwd.x, want_fwd.x, 1e-9);
    EXPECT_NEAR(got_fwd.y, want_fwd.y, 1e-9);
    EXPECT_NEAR(got_fwd.z, want_fwd.z, 1e-9);
    const Vec3 want_up = ConvertPosition(Rotate(ros, Vec3{0.0, 0.0, 1.0}),
                                         PoseConvention::kUnity);
    const Vec3 got_up = Rotate(sent, Vec3{0.0, 1.0, 0.0});
    EXPECT_NEAR(got_up.x, want_up.x, 1e-9);
    EXPECT_NEAR(got_up.y, want_up.y, 1e-9);
    EXPECT_NEAR(got_up.z, want_up.z, 1e-9);
  }
}

TEST(GltfAuthoredConvention, UnityBehaviourIsUnchanged) {
  // Pins the DISPATCH: both channels must route to RosToUnityRotation, so no
  // existing consumer can be altered by re-routing one of them. It deliberately
  // does NOT constrain that function's own behaviour --
  // UnityRotationAgreesWithItsPositionMapOffThePlane above does that.
  for (const Quat& ros : Rotations()) {
    const Quat want = RosToUnityRotation(ros);
    const Quat gaze = ConvertGazeRotation(ros, PoseConvention::kUnity);
    const Quat dyn = ConvertDynamicRotation(ros, PoseConvention::kUnity);
    EXPECT_DOUBLE_EQ(gaze.x, want.x);
    EXPECT_DOUBLE_EQ(gaze.y, want.y);
    EXPECT_DOUBLE_EQ(gaze.z, want.z);
    EXPECT_DOUBLE_EQ(gaze.w, want.w);
    EXPECT_DOUBLE_EQ(dyn.x, want.x);
    EXPECT_DOUBLE_EQ(dyn.y, want.y);
    EXPECT_DOUBLE_EQ(dyn.z, want.z);
    EXPECT_DOUBLE_EQ(dyn.w, want.w);
  }
  const Vec3 p = ConvertPosition(Vec3{1.0, 2.0, 3.0}, PoseConvention::kUnity);
  EXPECT_DOUBLE_EQ(p.x, -2.0);
  EXPECT_DOUBLE_EQ(p.y, 3.0);
  EXPECT_DOUBLE_EQ(p.z, 1.0);
}
