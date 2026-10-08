#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>

namespace cognitive3d_ros {

// A cap on how many events each source may put into the session.
//
// The session model has no rate limit of its own, and past its record bound it
// drops the NEWEST event. So one source stuck in a loop does not merely crowd the
// others out -- it discards whatever happens after it, which is the half of the
// session an incident is in. A budget per source keeps the damage inside the
// source that caused it.
//
// A token bucket rather than a fixed window: an action server publishing four
// transitions in one tick is normal and must not be clipped, while the same
// source at a hundred a second must be.
class EventRateLimiter {
 public:
  // Events per second, per source. Zero or less removes the cap entirely.
  void set_limit_hz(double hz) { limit_hz_ = hz; }
  double limit_hz() const { return limit_hz_; }

  // Whether this event may be recorded. `now` is wall-clock seconds; a `now`
  // behind the last one refills nothing rather than draining the bucket.
  bool Allow(const std::string& source, double now);

  // Where `now` comes from once set, in place of the argument. The cap protects the
  // platform's ingestion, which runs on wall time whatever timeline a record carries,
  // so a caller whose records are stamped on a simulated one sets a real clock here.
  void set_clock(std::function<double()> clock) { clock_ = std::move(clock); }

  // The next source that has had something suppressed and has not been named yet,
  // or nullopt when there is none. One line per source rather than one per event.
  //
  // Optional rather than an empty string, because a source key is opaque here --
  // it is a topic name for a translator and a fixed key for the pose and
  // remote-variable channels -- and nothing stops a future one from being empty,
  // which under a sentinel would end the caller's drain loop early and silence
  // every source after it.
  //
  // It hands back the SOURCE rather than answering about one, because the caller
  // that produced an event is not always the source whose budget it spent: the
  // vanished-goal sweep runs on whichever action published last and charges the
  // goal's own topic, and the pose channel charges a key that is no topic at all.
  std::optional<std::string> TakeUnreportedSuppression();

  std::uint64_t suppressed() const { return suppressed_; }
  std::uint64_t suppressed_for(const std::string& source) const;

  // Per session, like every other counter the node reports.
  void Reset();

 private:
  struct Bucket {
    double tokens = 0.0;
    double last = 0.0;
    std::uint64_t suppressed = 0;
    bool reported = false;
  };
  double limit_hz_ = 0.0;
  std::function<double()> clock_;
  std::uint64_t suppressed_ = 0;
  std::map<std::string, Bucket> buckets_;
};

}  // namespace cognitive3d_ros
