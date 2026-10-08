// The clock policy is the most likely silent failure in the whole build:
// Cognitive3D timestamps are Unix epoch seconds and the session id embeds one,
// while Gazebo's /clock starts near zero.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <functional>

#include "cognitive3d_ros/clock_policy.hpp"

using namespace cognitive3d_ros;

namespace {
constexpr double kYear2020 = 1577836800.0;

double WallNow() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

// Whether `read()` lands `lead` seconds past the wall clock, judged against wall reads taken on
// either side of it rather than one taken after it: the gap between two reads is however long
// this thread was descheduled, which no fixed tolerance bounds.
::testing::AssertionResult LeadsTheWallClockBy(const std::function<double()>& read, double lead) {
  const double before = WallNow();
  const double value = read();
  const double after = WallNow();
  if (value - before >= lead - 1e-3 && value - after <= lead + 1e-3) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << value - before << " s past the wall clock read before it and " << value - after
         << " s past the one read after it, not " << lead;
}
}  // namespace

TEST(ClockPolicy, WireTimeIsAlwaysWallClockEvenUnderSimTime) {
  auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ClockPolicy policy(sim_clock, /*use_sim_time=*/true);
  // A sim clock that nobody drives reads zero. The wire timestamp must not.
  EXPECT_GT(policy.WireTime(), kYear2020)
      << "ROS time reached the wire; the session would land in 1970";
}

TEST(ClockPolicy, RosTimeZeroMeansUninitialisedNotEpoch) {
  auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ClockPolicy policy(sim_clock, /*use_sim_time=*/true);
  if (policy.RosTime() == 0.0) {
    EXPECT_FALSE(policy.RosTimeReady());
  }
  ClockPolicy wall_policy(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), false);
  EXPECT_TRUE(wall_policy.RosTimeReady());
}

TEST(ClockPolicy, HeaderStampMapsIntoTheWireDomain) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  ClockPolicy policy(clock, false);
  double wire = 0.0;
  EXPECT_TRUE(LeadsTheWallClockBy(
      [&] {
        const double stamp = policy.RosTime() - 0.25;  // a quarter second old
        wire = policy.HeaderStampToWireTime(stamp);
        return wire;
      },
      -0.25));
  EXPECT_GT(wire, kYear2020);
}

TEST(ClockPolicy, AbsurdHeaderStampFallsBackToNow) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  ClockPolicy policy(clock, false);
  // A stamp from a machine with a badly skewed clock must not drag the record
  // into a different decade.
  EXPECT_NEAR(policy.HeaderStampToWireTime(1.0), policy.WireTime(), 1.0);
  EXPECT_NEAR(policy.HeaderStampToWireTime(0.0), policy.WireTime(), 1.0);
}

TEST(ClockPolicy, BackwardJumpIsDetected) {
  // A sim reset or a bag loop moves ROS time backwards. Without detecting it,
  // durations go negative and two runs interleave in one session.
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ClockPolicy policy(clock, true);
  bool fired = false;
  double observed = 0.0;
  policy.OnBackwardJump([&](double delta) {
    fired = true;
    observed = delta;
  });

  policy.backward_jump_threshold_s = 0.5;
  policy.Poll();  // primes last_ros_time_

  // Drive the jump directly rather than depending on a live /clock publisher.
  clock->ros_time_is_active();
  auto ret = rcl_enable_ros_time_override(clock->get_clock_handle());
  ASSERT_EQ(ret, RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), RCL_S_TO_NS(1000)), RCL_RET_OK);
  policy.Poll();
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), RCL_S_TO_NS(10)), RCL_RET_OK);
  policy.Poll();

  EXPECT_TRUE(fired) << "a backward ROS time jump must be reported";
  EXPECT_LT(observed, 0.0);
}

// TWO callers poll this object and only one of them acts on a jump. `Poll` is edge-triggered, so
// with the return value as the only signal the caller that polls FIRST swallows the edge: the node
// would log "the current session must be closed and a new one opened" while the session stayed
// open, because the discovery timer had already consumed the transition the sample timer acts on.
// Each caller's own logic is correct in that failure; only the hand-off between them is wrong.
TEST(ClockPolicy, ABackwardJumpBelongsToWhoeverActsOnItNotToWhoeverSawItFirst) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ClockPolicy policy(clock, true);
  policy.backward_jump_threshold_s = 0.5;

  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), RCL_S_TO_NS(1000)), RCL_RET_OK);
  policy.Poll();  // primes last_ros_time_
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), RCL_S_TO_NS(10)), RCL_RET_OK);

  EXPECT_TRUE(policy.Poll()) << "the poll that observes the jump reports it";
  EXPECT_FALSE(policy.Poll()) << "a second poll sees no new edge -- this is the swallowing";
  EXPECT_TRUE(policy.ConsumeBackwardJump())
      << "the jump is still outstanding for the caller that acts on it";
  EXPECT_FALSE(policy.ConsumeBackwardJump()) << "and it is delivered exactly once";
}

TEST(ClockPolicy, NoJumpMeansNothingToConsume) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ClockPolicy policy(clock, true);
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), RCL_S_TO_NS(10)), RCL_RET_OK);
  policy.Poll();
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), RCL_S_TO_NS(20)), RCL_RET_OK);
  policy.Poll();
  EXPECT_FALSE(policy.ConsumeBackwardJump());
}

namespace {

// A ROS clock whose time is whatever the test says, which is what /clock does to a sim node.
std::shared_ptr<rclcpp::Clock> OverriddenRosClock(double seconds) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(),
                                      static_cast<rcl_time_point_value_t>(seconds * 1e9)),
            RCL_RET_OK);
  return clock;
}

void SetRosTime(const std::shared_ptr<rclcpp::Clock>& clock, double seconds) {
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(),
                                      static_cast<rcl_time_point_value_t>(seconds * 1e9)),
            RCL_RET_OK);
}

constexpr double kWallAnchor = 1900000000.0;

}  // namespace

// The stretched session: below real time, a wall-clock wire makes a five-minute simulated run as
// long as the wall time it took. Anchored, the wire advances exactly as far as ROS time did.
TEST(ClockPolicySimTimeOnWire, WireTimeAdvancesWithRosTimeFromTheAnchor) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, /*use_sim_time=*/true, /*sim_time_on_wire=*/true);
  policy.Anchor(kWallAnchor, policy.RosTime());
  ASSERT_TRUE(policy.anchored());
  EXPECT_DOUBLE_EQ(policy.WireTime(), kWallAnchor);
  SetRosTime(clock, 1300.0);
  EXPECT_NEAR(policy.WireTime(), kWallAnchor + 300.0, 1e-6)
      << "five simulated minutes must be five minutes on the wire, whatever the wall took";
}

TEST(ClockPolicySimTimeOnWire, HeaderStampIsPlacedAgainstTheAnchor) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, true, true);
  policy.Anchor(kWallAnchor, 1000.0);
  SetRosTime(clock, 1060.0);
  EXPECT_NEAR(policy.HeaderStampToWireTime(1059.75), kWallAnchor + 59.75, 1e-6);
  // A stamp from before the anchor is still placed by the same rule, not clamped to it.
  EXPECT_NEAR(policy.HeaderStampToWireTime(999.5), kWallAnchor - 0.5, 1e-6);
}

TEST(ClockPolicySimTimeOnWire, AnAbsurdHeaderStampFallsBackToTheAnchoredNow) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, true, true);
  policy.Anchor(kWallAnchor, 1000.0);
  SetRosTime(clock, 5000.0);
  const double now_on_wire = kWallAnchor + 4000.0;
  EXPECT_NEAR(policy.HeaderStampToWireTime(1.0), now_on_wire, 1e-6) << "more than 3600 s old";
  EXPECT_NEAR(policy.HeaderStampToWireTime(5002.0), now_on_wire, 1e-6) << "from the future";
  EXPECT_NEAR(policy.HeaderStampToWireTime(0.0), now_on_wire, 1e-6) << "never stamped";
  EXPECT_NEAR(policy.HeaderStampToWireTime(std::nan("")), now_on_wire, 1e-6);
}

// Before /clock is published ROS time reads zero, and an anchor taken against it would put every
// later record ROS-time seconds into the future. The anchor waits, and the wire stays on wall
// clock until it is taken.
TEST(ClockPolicySimTimeOnWire, AZeroRosTimeIsNeverCapturedAsTheAnchor) {
  auto clock = OverriddenRosClock(0.0);
  ClockPolicy policy(clock, true, true);
  policy.Anchor(kWallAnchor, policy.RosTime());
  EXPECT_FALSE(policy.anchored());
  const auto wall = [] {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  };
  EXPECT_NEAR(policy.WireTime(), wall(), 1.0) << "unanchored, the wire is wall clock";

  SetRosTime(clock, 50.0);
  const double taken = policy.WireTime();
  ASSERT_TRUE(policy.anchored()) << "the first wire time after /clock takes the anchor";
  EXPECT_NEAR(taken, wall(), 1.0);
  SetRosTime(clock, 60.0);
  EXPECT_NEAR(policy.WireTime(), taken + 10.0, 1e-6);
}

TEST(ClockPolicySimTimeOnWire, AReanchorAfterABackwardJumpStartsFresh) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, true, true);
  policy.Anchor(kWallAnchor, 1000.0);
  SetRosTime(clock, 1100.0);
  EXPECT_NEAR(policy.WireTime(), kWallAnchor + 100.0, 1e-6);

  // A world reset. The next session anchors at its own, later, wall start.
  SetRosTime(clock, 2.0);
  policy.Anchor(kWallAnchor + 400.0, 2.0);
  EXPECT_NEAR(policy.WireTime(), kWallAnchor + 400.0, 1e-6);
  SetRosTime(clock, 12.0);
  EXPECT_NEAR(policy.WireTime(), kWallAnchor + 410.0, 1e-6);
}

// Faster than real time, the previous session's records run past wall now. A reopen anchored at
// wall now would then put the new session's first records before the old one's last.
TEST(ClockPolicySimTimeOnWire, AReanchorNeverLandsBeforeAWireTimeAlreadyIssued) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, true, true);
  policy.Anchor(kWallAnchor, 1000.0);
  SetRosTime(clock, 1600.0);
  const double last_of_previous = policy.WireTime();
  ASSERT_NEAR(last_of_previous, kWallAnchor + 600.0, 1e-6);

  SetRosTime(clock, 1.0);
  policy.Anchor(kWallAnchor + 300.0, 1.0);
  EXPECT_GE(policy.WireTime(), last_of_previous);
}

// Off is the default: an anchor is ignored and the wire is wall.
TEST(ClockPolicySimTimeOnWire, OffTheAnchorChangesNothing) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, /*use_sim_time=*/true);
  EXPECT_FALSE(policy.sim_time_on_wire());
  policy.Anchor(kWallAnchor, 1000.0);
  EXPECT_FALSE(policy.anchored());
  SetRosTime(clock, 1300.0);
  const double wall = std::chrono::duration<double>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  EXPECT_NEAR(policy.WireTime(), wall, 1.0);
  EXPECT_NEAR(policy.HeaderStampToWireTime(1299.75), wall - 0.25, 1.0);
}

// A backward ROS jump inside a session, before the sample tick that restarts it or below the
// restart threshold altogether, must not stamp records before ones already issued: the platform
// takes the earliest record as the session's start.
TEST(ClockPolicySimTimeOnWire, WireTimeNeverDecreasesAcrossABackwardJump) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, true, true);
  policy.Anchor(kWallAnchor, 1000.0);
  SetRosTime(clock, 1100.0);
  const double before = policy.WireTime();
  ASSERT_NEAR(before, kWallAnchor + 100.0, 1e-6);

  SetRosTime(clock, 1099.8);
  EXPECT_GE(policy.WireTime(), before) << "a jump under the restart threshold";
  SetRosTime(clock, 3.0);
  EXPECT_GE(policy.WireTime(), before) << "a world reset, before the session is restarted";
  EXPECT_GE(policy.HeaderStampToWireTime(3.0), before)
      << "a header stamp is placed against the held now, not the reset one";
  SetRosTime(clock, 1100.5);
  EXPECT_NEAR(policy.WireTime(), kWallAnchor + 100.5, 1e-6)
      << "the hold ends once ROS time passes it";
}

// A header stamp may run ahead of now, and the floor is kept on now alone: a stamp half a second
// ahead is not a reason to hold every later now at it.
TEST(ClockPolicySimTimeOnWire, AStampAheadOfNowRaisesTheLatestButNotTheFloor) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, true, true);
  policy.Anchor(kWallAnchor, 1000.0);
  EXPECT_NEAR(policy.HeaderStampToWireTime(1000.5), kWallAnchor + 0.5, 1e-6);
  EXPECT_NEAR(policy.LatestWireTime(), kWallAnchor + 0.5, 1e-6);
  EXPECT_NEAR(policy.WireTime(), kWallAnchor, 1e-6);
}

// While the anchor waits for /clock, a header stamp is placed at wall now, and the first
// placement after /clock arrives is what takes the anchor.
TEST(ClockPolicySimTimeOnWire, AHeaderStampWhileTheAnchorIsPendingIsWallNow) {
  auto clock = OverriddenRosClock(0.0);
  ClockPolicy policy(clock, true, true);
  policy.Anchor(kWallAnchor, 0.0);
  const auto wall = [] {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  };
  EXPECT_NEAR(policy.HeaderStampToWireTime(5.0), wall(), 1.0);
  EXPECT_FALSE(policy.anchored()) << "ROS time is still zero";

  SetRosTime(clock, 50.0);
  const double placed = policy.HeaderStampToWireTime(49.5);
  ASSERT_TRUE(policy.anchored());
  EXPECT_NEAR(placed, wall() - 0.5, 1.0);
  EXPECT_NEAR(policy.WireTime() - placed, 0.5, 1e-3);
}

// On the wall clock the wire can lead it, for a session whose start is later than wall now: every
// time, a header placement included, moves by the lead and keeps the wall clock's spacing. No
// lead is none, a negative one is none, and under sim_time_on_wire the anchor does this instead.
TEST(ClockPolicy, TheWireLeadsTheWallClockByTheLeadItIsGiven) {
  auto clock = OverriddenRosClock(1000.0);
  ClockPolicy policy(clock, /*use_sim_time=*/true);
  policy.LeadWallClock(1.75);
  EXPECT_DOUBLE_EQ(policy.wall_clock_lead(), 1.75);
  EXPECT_TRUE(LeadsTheWallClockBy([&] { return policy.WireTime(); }, 1.75));
  EXPECT_TRUE(LeadsTheWallClockBy([&] { return policy.HeaderStampToWireTime(999.75); }, 1.5));
  for (const double none : {0.0, -2.0, std::nan("")}) {
    policy.LeadWallClock(none);
    EXPECT_DOUBLE_EQ(policy.wall_clock_lead(), 0.0) << none;
    EXPECT_TRUE(LeadsTheWallClockBy([&] { return policy.WireTime(); }, 0.0)) << none;
  }

  ClockPolicy anchored(clock, /*use_sim_time=*/true, /*sim_time_on_wire=*/true);
  anchored.Anchor(kWallAnchor, 1000.0);
  anchored.LeadWallClock(1.75);
  EXPECT_DOUBLE_EQ(anchored.wall_clock_lead(), 0.0);
  EXPECT_NEAR(anchored.WireTime(), kWallAnchor, 1e-6);
}
