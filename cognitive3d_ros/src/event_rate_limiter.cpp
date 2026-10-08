#include "cognitive3d_ros/event_rate_limiter.hpp"

#include <algorithm>

namespace cognitive3d_ros {
namespace {

// One second of headroom, so a burst the size of the limit passes and only a
// SUSTAINED rate above it is cut. Never below one token, or a sub-1 Hz limit
// would refuse the first event of a session outright.
double Capacity(double limit_hz) { return std::max(1.0, limit_hz); }

}  // namespace

bool EventRateLimiter::Allow(const std::string& source, double now) {
  // Written as a negation so a NaN limit disables the cap rather than refusing
  // every event.
  if (!(limit_hz_ > 0.0)) return true;
  if (clock_) now = clock_();

  const double capacity = Capacity(limit_hz_);
  auto it = buckets_.find(source);
  if (it == buckets_.end()) {
    Bucket fresh;
    fresh.tokens = capacity;
    fresh.last = now;
    it = buckets_.emplace(source, fresh).first;
  } else {
    const double elapsed = std::max(0.0, now - it->second.last);
    it->second.tokens = std::min(capacity, it->second.tokens + elapsed * limit_hz_);
    it->second.last = now;
  }

  if (it->second.tokens >= 1.0) {
    it->second.tokens -= 1.0;
    return true;
  }
  ++it->second.suppressed;
  ++suppressed_;
  return false;
}

std::optional<std::string> EventRateLimiter::TakeUnreportedSuppression() {
  for (auto& kv : buckets_) {
    if (kv.second.suppressed == 0 || kv.second.reported) continue;
    kv.second.reported = true;
    return kv.first;
  }
  return std::nullopt;
}

std::uint64_t EventRateLimiter::suppressed_for(const std::string& source) const {
  const auto it = buckets_.find(source);
  return it == buckets_.end() ? 0 : it->second.suppressed;
}

void EventRateLimiter::Reset() {
  buckets_.clear();
  suppressed_ = 0;
}

}  // namespace cognitive3d_ros
