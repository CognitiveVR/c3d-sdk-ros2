#include "pose_queue.hpp"

#include <algorithm>
#include <utility>

namespace c3d {
namespace pose {

const char* FlushReason(FlushTrigger trigger) {
  switch (trigger) {
    case FlushTrigger::kEnd: return "flush:end";
    case FlushTrigger::kScene: return "flush:scene";
    case FlushTrigger::kPause: return "flush:pause";
  }
  return "flush:end";
}

UploadQueue::UploadQueue(std::int64_t timer_us, std::uint64_t cap_bytes)
    : timer_us_(timer_us), cap_bytes_(cap_bytes) {}

std::int64_t UploadQueue::At(std::int64_t t) {
  if (seen_t_ && t < last_t_) t = last_t_;
  last_t_ = t;
  seen_t_ = true;
  return t;
}

void UploadQueue::Timer(std::int64_t t, std::vector<Send>* sends) {
  // Inclusive: a queue is due AT its deadline.
  if (!queue_.blocks.empty() && t >= queue_.started + timer_us_) {
    SendBatch(&queue_, t, "timer", 0, sends);
  }
}

void UploadQueue::SendBatch(Batch* batch, std::int64_t t, const std::string& reason, int flush,
                            std::vector<Send>* sends) {
  Send send;
  send.part = next_part_++;
  send.kind = Send::Kind::kBlocks;
  send.queue_start_us = batch->started;
  send.sent_us = t;
  send.reason = reason;
  send.flush = flush;
  send.start_us = batch->blocks.front().start_us;
  send.end_us = batch->blocks.front().last_us;
  for (const QueuedBlock& b : batch->blocks) {
    send.start_us = std::min(send.start_us, b.start_us);
    send.end_us = std::max(send.end_us, b.last_us);
  }
  send.blocks = std::move(batch->blocks);
  *batch = Batch{};
  sends->push_back(std::move(send));
}

std::vector<Send> UploadQueue::Arrive(std::int64_t t, QueuedBlock block, int flush) {
  t = At(t);
  std::vector<Send> sends;
  // Rule 3's order: the timer, then the cap, then the block. A flush's block still runs the
  // queue's timer check: an arrival is a check whatever it carries.
  Timer(t, &sends);
  const auto running = flush > 0 ? flushes_.find(flush) : flushes_.end();
  Batch* batch = &queue_;
  std::string cap_reason = "cap";
  int part_flush = 0;
  if (running != flushes_.end()) {
    // A flush's batch is never timer-checked, and a piece the cap splits off it is still a
    // flush part.
    batch = &running->second.batch;
    cap_reason = FlushReason(running->second.trigger);
    part_flush = flush;
  }
  if (!batch->blocks.empty() && batch->bytes + block.bytes > cap_bytes_) {
    SendBatch(batch, t, cap_reason, part_flush, &sends);
  }
  if (batch->blocks.empty()) batch->started = t;
  batch->bytes += block.bytes;
  batch->blocks.push_back(std::move(block));
  return sends;
}

std::vector<Send> UploadQueue::Tick(std::int64_t t) {
  t = At(t);
  std::vector<Send> sends;
  Timer(t, &sends);
  return sends;
}

Send UploadQueue::ManifestPart(std::int64_t t) {
  At(t);
  Send send;
  send.part = next_part_++;
  send.kind = Send::Kind::kManifests;
  return send;
}

bool UploadQueue::StartFlush(std::int64_t t, FlushTrigger trigger, int k,
                             std::vector<Send>* sends) {
  t = At(t);
  if (k <= 0 || std::find(used_flushes_.begin(), used_flushes_.end(), k) != used_flushes_.end()) {
    return false;
  }
  used_flushes_.push_back(k);
  flushes_[k].trigger = trigger;
  // The queue as it is, even when it is already due: a flush part, never a timer send.
  if (!queue_.blocks.empty()) SendBatch(&queue_, t, FlushReason(trigger), k, sends);
  return true;
}

std::vector<Send> UploadQueue::FinishFlush(std::int64_t t, int k) {
  t = At(t);
  std::vector<Send> sends;
  const auto it = flushes_.find(k);
  if (it == flushes_.end()) return sends;
  if (!it->second.batch.blocks.empty()) {
    SendBatch(&it->second.batch, t, FlushReason(it->second.trigger), k, &sends);
  }
  flushes_.erase(it);
  return sends;
}

}  // namespace pose
}  // namespace c3d
