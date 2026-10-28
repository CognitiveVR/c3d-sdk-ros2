// The pose stream's upload queue: which encoded blocks go out together, and when.
//
// The rules are the platform's: §10.1 rules 3 to 7 of c3d-stream-formatter's
// docs/pose/spec.md, decided case by case as its vectors/pose/queue/ files require (their
// README is the long form of every rule below). This class decides sends and numbers parts;
// it builds no envelope, reads no clock and holds no lock. Every call carries `t`, the
// session's monotonic clock in microseconds, so a test drives a whole session in
// microseconds of real time, and the session reads that clock in one place.
//
//   * A block's entry time is the `t` of the Arrive that adds it, never a time carried on
//     the block: a deadline counted from a block's close or its samples fires exactly as
//     the next block closes, and the time that block takes to encode makes it miss every
//     upload.
//   * The queue goes out at the first Arrive or Tick at or after its first block's entry
//     time + kSendTimerUs, and nowhere else: not on a manifest send, not on a flush. There
//     is no separate timer object. One started when a block enters an empty queue can be
//     lost when a block arrives while another thread sends.
//   * Arrive checks the timer, then the byte cap, then adds the block, so a block arriving
//     at or after the deadline starts the next queue, and a send both would make is a timer
//     send.
//   * A flush sends the queue as it is (blocks already encoded never wait behind the
//     encoding of open ones), then the blocks it closed, in pieces split by the cap, the
//     last at FinishFlush. Every flush part carries the flush's reason and number. Flushes
//     may overlap; each owns the blocks it closed.
//   * Part numbers start at 1 and count every send, manifest parts included, in the order
//     the sends are decided.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace c3d {
namespace pose {

// Rule 3: the queue goes out this long after its first block entered it.
constexpr std::int64_t kSendTimerUs = 11'000'000;
// Rule 4: the blocks' `data` bytes in one part, at most. Exactly this many fits.
constexpr std::uint64_t kPartCapBytes = 5'242'880;

enum class FlushTrigger { kEnd, kScene, kPause };

// The send log's `reason` for a flush: "flush:end", "flush:scene", "flush:pause".
const char* FlushReason(FlushTrigger trigger);

// One encoded block as the envelope carries it. Only `bytes` decides a send; `start_us` and
// `last_us` (the block's first and last sample times, epoch microseconds) set the part's
// envelope times.
struct QueuedBlock {
  std::string manifest_id;
  std::int64_t seq = 0;
  std::int64_t start_us = 0;
  std::int64_t last_us = 0;
  std::int32_t sample_count = 0;
  bool reset_before = false;
  // Counted against kPartCapBytes: data.size() for a real block.
  std::uint64_t bytes = 0;
  std::string data;
};

struct Send {
  enum class Kind { kBlocks, kManifests };
  std::uint32_t part = 0;
  Kind kind = Kind::kBlocks;
  // A block part's blocks, in arrival order. Empty on a manifest part, whose entries are
  // the caller's.
  std::vector<QueuedBlock> blocks;
  // A block part's send-log fields (spec §10.4): when its first block entered the empty
  // queue or the flush's batch, the clock reading that decided the send, why, and on a
  // flush part the flush's number (0 on any other part).
  std::int64_t queue_start_us = 0;
  std::int64_t sent_us = 0;
  std::string reason;
  int flush = 0;
  // Rule 7: the earliest block start and the latest sample time in the part; both 0 on a
  // manifest part.
  std::int64_t start_us = 0;
  std::int64_t end_us = 0;
};

class UploadQueue {
 public:
  explicit UploadQueue(std::int64_t timer_us = kSendTimerUs,
                       std::uint64_t cap_bytes = kPartCapBytes);

  // An encoded block enters at `t`. `flush` > 0: a block flush `flush` closed (or one still
  // encoding when it started), which joins that flush's own batch; a number that is not a
  // running flush is treated as 0. Returns the sends this decided.
  std::vector<Send> Arrive(std::int64_t t, QueuedBlock block, int flush = 0);
  // The periodic check, at least once a second.
  std::vector<Send> Tick(std::int64_t t);
  // A manifest part is ready: it takes the next part number and touches nothing else. The
  // caller puts the entries in it.
  Send ManifestPart(std::int64_t t);
  // Flush `k` starts: sends the queue as it is, if it holds anything. `k` counts 1, 2, 3, ...
  // per session in start order; a number already used is refused (false, nothing changes).
  bool StartFlush(std::int64_t t, FlushTrigger trigger, int k, std::vector<Send>* sends);
  // Every pipeline has finished flush `k`'s blocks: sends what is left of its batch.
  std::vector<Send> FinishFlush(std::int64_t t, int k);

  bool empty() const { return queue_.blocks.empty(); }
  std::uint32_t parts_sent() const { return next_part_ - 1; }

 private:
  struct Batch {
    std::vector<QueuedBlock> blocks;
    std::uint64_t bytes = 0;
    std::int64_t started = 0;
  };
  struct Flush {
    FlushTrigger trigger = FlushTrigger::kEnd;
    Batch batch;
  };

  // `t` never runs backwards here: a caller's clock that did is held at its latest reading.
  std::int64_t At(std::int64_t t);
  void Timer(std::int64_t t, std::vector<Send>* sends);
  void SendBatch(Batch* batch, std::int64_t t, const std::string& reason, int flush,
                 std::vector<Send>* sends);

  std::int64_t timer_us_;
  std::uint64_t cap_bytes_;
  std::uint32_t next_part_ = 1;
  std::int64_t last_t_ = 0;
  bool seen_t_ = false;
  Batch queue_;
  std::map<int, Flush> flushes_;
  std::vector<int> used_flushes_;
};

}  // namespace pose
}  // namespace c3d
