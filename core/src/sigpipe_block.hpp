#pragma once

#include <signal.h>

namespace c3d {

// Holds SIGPIPE off the calling thread for as long as it lives, so a write to a socket whose peer
// has gone fails with EPIPE instead of ending the process. Only this thread's mask changes: the
// process-wide disposition belongs to whatever process this library is loaded into.
//
// On release it discards a SIGPIPE raised while it was held and restores the thread's previous
// mask exactly. A SIGPIPE already pending when it began is left pending, and so is one raised on
// top of it, since standard signals do not queue and the two cannot be told apart.
//
// Released on the thread that made it, which a scope guarantees.
class ScopedSigpipeBlock {
 public:
  ScopedSigpipeBlock();
  ~ScopedSigpipeBlock();
  ScopedSigpipeBlock(const ScopedSigpipeBlock&) = delete;
  ScopedSigpipeBlock& operator=(const ScopedSigpipeBlock&) = delete;

 private:
  sigset_t previous_mask_;
  bool was_pending_ = false;
};

}  // namespace c3d
