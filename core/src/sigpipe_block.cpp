#include "sigpipe_block.hpp"

#include <pthread.h>

#include <cerrno>
#include <ctime>

namespace c3d {
namespace {

sigset_t OnlySigpipe() {
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGPIPE);
  return set;
}

// This thread's pending set and the process's, together.
bool SigpipePending() {
  sigset_t pending;
  sigemptyset(&pending);
  return sigpending(&pending) == 0 && sigismember(&pending, SIGPIPE) == 1;
}

}  // namespace

ScopedSigpipeBlock::ScopedSigpipeBlock() {
  const sigset_t only = OnlySigpipe();
  pthread_sigmask(SIG_BLOCK, &only, &previous_mask_);
  was_pending_ = SigpipePending();
}

ScopedSigpipeBlock::~ScopedSigpipeBlock() {
  if (!was_pending_ && SigpipePending()) {
    const sigset_t only = OnlySigpipe();
#if defined(__linux__)
    // A zero timeout: a SIGPIPE sent to the whole process can be taken by another thread
    // between the check above and this call, and the wait must not then block.
    const timespec now{0, 0};
    while (sigtimedwait(&only, nullptr, &now) < 0 && errno == EINTR) {
    }
#else
    int taken = 0;
    sigwait(&only, &taken);
#endif
  }
  pthread_sigmask(SIG_SETMASK, &previous_mask_, nullptr);
}

}  // namespace c3d
