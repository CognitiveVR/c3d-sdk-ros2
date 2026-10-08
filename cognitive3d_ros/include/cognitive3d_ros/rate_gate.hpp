#pragma once

namespace cognitive3d_ros {

// The rate limit every camera, lidar and translator applies: at most `rate_hz` messages per second
// of the times it is given.
//
// Those are ARRIVAL times on one clock that never runs backwards, never a message's own stamp. A
// stamp is the publisher's: a driver may leave it at zero on some frames, two publishers on one
// topic carry two clocks, and a simulator resets it, and each of those mixed into one gate either
// holds it shut or lets every message through.
//
// The next deadline is one period after the previous deadline, not after the accepted time: a
// source at the gate's own rate arrives a hair early about half the time, and scheduling from the
// time itself would reject each of those and deliver half the configured rate. It is never earlier
// than half a period after the accepted time, so a gap banks no credit for a burst. Given times
// that do run backwards, it stays shut until they pass the deadline again: it never admits more
// than its rate, whatever it is fed.
class RateGate {
 public:
  // Whether a message arriving at `time_seconds` passes. A rate that is not positive passes all.
  bool Accept(double time_seconds, double rate_hz);
  // Forgets where it was, so the next message passes.
  void Reset() { *this = RateGate{}; }

 private:
  bool primed_ = false;
  double next_ = 0.0;
};

}  // namespace cognitive3d_ros
