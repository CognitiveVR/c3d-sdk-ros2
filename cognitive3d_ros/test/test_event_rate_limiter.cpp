// The per-source event budget.
//
// Driven on an explicit clock rather than on wall time, because every property
// worth pinning here is about what happens per second and a test that waits for a
// real second is a test nobody runs.

#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <string>

#include "cognitive3d_ros/event_rate_limiter.hpp"

using namespace cognitive3d_ros;

TEST(EventRateLimiter, ZeroDisablesTheCap) {
  EventRateLimiter limiter;
  limiter.set_limit_hz(0.0);
  for (int i = 0; i < 1000; ++i) EXPECT_TRUE(limiter.Allow("/a", 100.0));
  EXPECT_EQ(limiter.suppressed(), 0u);
}

TEST(EventRateLimiter, ABurstTheSizeOfTheLimitPassesAndTheRestIsSuppressed) {
  // One second of headroom, so an action server publishing several transitions in
  // one tick is not clipped, and the twenty-first event in the same instant is.
  EventRateLimiter limiter;
  limiter.set_limit_hz(5.0);
  int allowed = 0;
  for (int i = 0; i < 20; ++i) {
    if (limiter.Allow("/a", 100.0)) ++allowed;
  }
  EXPECT_EQ(allowed, 5);
  EXPECT_EQ(limiter.suppressed(), 15u);
  EXPECT_EQ(limiter.suppressed_for("/a"), 15u);
}

TEST(EventRateLimiter, TheBucketRefillsWithTime) {
  EventRateLimiter limiter;
  limiter.set_limit_hz(5.0);
  for (int i = 0; i < 20; ++i) limiter.Allow("/a", 100.0);
  EXPECT_FALSE(limiter.Allow("/a", 100.0));
  // Two seconds at 5 Hz is ten tokens, capped at the one-second capacity of five.
  int allowed = 0;
  for (int i = 0; i < 20; ++i) {
    if (limiter.Allow("/a", 102.0)) ++allowed;
  }
  EXPECT_EQ(allowed, 5);
}

TEST(EventRateLimiter, OneSourceCannotSpendAnothersBudget) {
  // The whole reason the budget is per source: a translator stuck in a loop must
  // not delete the events a different topic is producing.
  EventRateLimiter limiter;
  limiter.set_limit_hz(2.0);
  for (int i = 0; i < 50; ++i) limiter.Allow("/chatty", 100.0);
  EXPECT_TRUE(limiter.Allow("/quiet", 100.0));
  EXPECT_EQ(limiter.suppressed_for("/quiet"), 0u);
}

TEST(EventRateLimiter, ASubOneHertzLimitStillPassesTheFirstEvent) {
  // Capacity never drops below one token, or a deliberately tiny cap would record
  // nothing at all rather than a little.
  EventRateLimiter limiter;
  limiter.set_limit_hz(0.1);
  EXPECT_TRUE(limiter.Allow("/a", 100.0));
  EXPECT_FALSE(limiter.Allow("/a", 100.0));
  EXPECT_TRUE(limiter.Allow("/a", 110.0));
}

TEST(EventRateLimiter, TimeGoingBackwardsDoesNotDrainTheBucket) {
  EventRateLimiter limiter;
  limiter.set_limit_hz(5.0);
  EXPECT_TRUE(limiter.Allow("/a", 100.0));
  EXPECT_TRUE(limiter.Allow("/a", 99.0));
  EXPECT_EQ(limiter.suppressed(), 0u);
}

TEST(EventRateLimiter, ANaNLimitDisablesTheCapRatherThanRefusingEverything) {
  EventRateLimiter limiter;
  limiter.set_limit_hz(std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < 100; ++i) EXPECT_TRUE(limiter.Allow("/a", 100.0));
}

TEST(EventRateLimiter, ASourceIsReportedOnceAndOnlyAfterItSuppressesSomething) {
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  EXPECT_FALSE(limiter.TakeUnreportedSuppression().has_value())
      << "a source that has suppressed nothing must not produce a warning";
  for (int i = 0; i < 10; ++i) limiter.Allow("/a", 100.0);
  EXPECT_EQ(limiter.TakeUnreportedSuppression(), "/a");
  EXPECT_FALSE(limiter.TakeUnreportedSuppression().has_value())
      << "one warning per source, not per event";
}

TEST(EventRateLimiter, HavingNothingToReportIsNotTheSameAsAnEmptySourceName) {
  // A source key is opaque here -- a topic name for a translator, a fixed key for
  // the pose and remote-variable channels -- so an empty one is a source like any
  // other. Answering "none" with an empty string makes the two indistinguishable,
  // and the caller drains this in a loop until it is told there is nothing left.
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  for (int i = 0; i < 4; ++i) limiter.Allow("", 100.0);
  for (int i = 0; i < 4; ++i) limiter.Allow("/a", 100.0);

  const std::optional<std::string> first = limiter.TakeUnreportedSuppression();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, "");
  EXPECT_EQ(limiter.TakeUnreportedSuppression(), "/a")
      << "an unnamed source ended the drain, taking every source after it with it";
  EXPECT_FALSE(limiter.TakeUnreportedSuppression().has_value());
}

TEST(EventRateLimiter, TheSourceReportedIsTheOneThatWasCharged) {
  // What an operator is handed is the source whose budget went, which is not
  // always the caller that produced the event: an objective sweep runs on
  // whichever action published last and charges the goal's own topic.
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  for (int i = 0; i < 4; ++i) limiter.Allow("/dock/_action/status", 100.0);
  EXPECT_EQ(limiter.TakeUnreportedSuppression(), "/dock/_action/status");

  limiter.Allow("/navigate_to_pose/_action/status", 100.0);
  EXPECT_FALSE(limiter.TakeUnreportedSuppression().has_value())
      << "a source that has spent nothing over its budget was reported";
  for (int i = 0; i < 4; ++i) limiter.Allow("/navigate_to_pose/_action/status", 100.0);
  EXPECT_EQ(limiter.TakeUnreportedSuppression(), "/navigate_to_pose/_action/status");
}

TEST(EventRateLimiter, ResetClearsTheCountersAndTheWarning) {
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  for (int i = 0; i < 10; ++i) limiter.Allow("/a", 100.0);
  EXPECT_EQ(limiter.TakeUnreportedSuppression(), "/a");
  limiter.Reset();
  EXPECT_EQ(limiter.suppressed(), 0u);
  EXPECT_EQ(limiter.suppressed_for("/a"), 0u);
  for (int i = 0; i < 10; ++i) limiter.Allow("/a", 100.0);
  EXPECT_EQ(limiter.TakeUnreportedSuppression(), "/a")
      << "the next session gets its own warning";
}

// A clock, once set, is what refills the bucket, and the `now` a caller passes is ignored: a
// caller stamping records on a frozen simulated clock passes the same `now` forever, and the
// cap must still refill as real time passes.
TEST(EventRateLimiter, ASetClockRefillsTheBucketWhateverNowTheCallerPasses) {
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  double real = 100.0;
  limiter.set_clock([&real] { return real; });
  EXPECT_TRUE(limiter.Allow("/a", 5.0));
  EXPECT_FALSE(limiter.Allow("/a", 5.0));
  real += 1.0;
  EXPECT_TRUE(limiter.Allow("/a", 5.0)) << "a second of the set clock refills one token";
  EXPECT_FALSE(limiter.Allow("/a", 500.0)) << "the caller's now refills nothing";
}
