// ScopedSigpipeBlock on its own: a write to a socket whose peer is gone, made while one is held,
// and the thread's signal state once it is released. Each case runs in a child of its own, so a
// SIGPIPE that gets through is a result rather than the end of the suite, and so the mask and the
// pending set a case leaves behind are nobody else's. The child starts with SIGPIPE at its default
// action and unblocked, whatever the process running the suite handed down.

#include <gtest/gtest.h>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <functional>
#include <string>

#include "sigpipe_block.hpp"

using c3d::ScopedSigpipeBlock;

namespace {

// How a child running `body` ended: "exited 0" is the pass, any other exit code names the step
// that failed, and a signal is a death.
std::string InChild(const std::function<int()>& body) {
  const pid_t child = ::fork();
  if (child == 0) {
    ::signal(SIGPIPE, SIG_DFL);
    sigset_t none;
    sigemptyset(&none);
    ::pthread_sigmask(SIG_SETMASK, &none, nullptr);
    ::_exit(body());
  }
  int status = 0;
  ::waitpid(child, &status, 0);
  if (WIFSIGNALED(status)) return "killed by signal " + std::to_string(WTERMSIG(status));
  return "exited " + std::to_string(WEXITSTATUS(status));
}

// A write to a stream socket whose peer has closed: EPIPE, and SIGPIPE sent to this thread.
// Returns the errno, or 0 when the write did not fail.
int WriteToAClosedPeer() {
  int fds[2];
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) return -1;
  ::close(fds[1]);
  const ssize_t written = ::write(fds[0], "x", 1);
  const int error = written < 0 ? errno : 0;
  ::close(fds[0]);
  return error;
}

bool SigpipePending() {
  sigset_t pending;
  sigemptyset(&pending);
  return ::sigpending(&pending) == 0 && sigismember(&pending, SIGPIPE) == 1;
}

sigset_t CurrentMask() {
  sigset_t mask;
  sigemptyset(&mask);
  ::pthread_sigmask(SIG_SETMASK, nullptr, &mask);
  return mask;
}

bool SameMask(const sigset_t& a, const sigset_t& b) {
  for (int signal = 1; signal < NSIG; ++signal) {
    if (sigismember(&a, signal) != sigismember(&b, signal)) return false;
  }
  return true;
}

void Block(std::initializer_list<int> signals) {
  sigset_t set;
  sigemptyset(&set);
  for (int signal : signals) sigaddset(&set, signal);
  ::pthread_sigmask(SIG_BLOCK, &set, nullptr);
}

}  // namespace

// Without this, the cases below would pass on a platform where the write raises nothing at all.
TEST(ScopedSigpipeBlock, WithoutItTheWriteKillsTheProcess) {
  EXPECT_EQ(InChild([] {
              WriteToAClosedPeer();
              return 0;
            }),
            "killed by signal " + std::to_string(SIGPIPE));
}

TEST(ScopedSigpipeBlock, TheWriteFailsWithEpipeAndTheProcessLives) {
  EXPECT_EQ(InChild([] {
              const sigset_t before = CurrentMask();
              {
                const ScopedSigpipeBlock block;
                if (WriteToAClosedPeer() != EPIPE) return 2;
              }
              if (SigpipePending()) return 3;
              if (!SameMask(CurrentMask(), before)) return 4;
              struct sigaction action {};
              ::sigaction(SIGPIPE, nullptr, &action);
              if (action.sa_handler != SIG_DFL) return 5;
              return 0;
            }),
            "exited 0");
}

TEST(ScopedSigpipeBlock, AMaskThatAlreadyBlockedSigpipeIsRestoredExactly) {
  EXPECT_EQ(InChild([] {
              Block({SIGPIPE, SIGUSR1});
              const sigset_t before = CurrentMask();
              {
                const ScopedSigpipeBlock block;
                if (WriteToAClosedPeer() != EPIPE) return 2;
              }
              if (!SameMask(CurrentMask(), before)) return 3;
              if (SigpipePending()) return 4;
              return 0;
            }),
            "exited 0");
  EXPECT_EQ(InChild([] {
              Block({SIGUSR1});
              const sigset_t before = CurrentMask();
              {
                const ScopedSigpipeBlock block;
                if (WriteToAClosedPeer() != EPIPE) return 2;
              }
              if (!SameMask(CurrentMask(), before)) return 3;
              return 0;
            }),
            "exited 0");
}

// Standard signals do not queue, so the guard cannot tell a SIGPIPE it caused from one that was
// already waiting, and must leave both to whoever blocked it.
TEST(ScopedSigpipeBlock, ASigpipePendingBeforeIsStillPendingAfter) {
  EXPECT_EQ(InChild([] {
              Block({SIGPIPE});
              ::pthread_kill(::pthread_self(), SIGPIPE);
              if (!SigpipePending()) return 2;
              {
                const ScopedSigpipeBlock block;
                if (WriteToAClosedPeer() != EPIPE) return 3;
              }
              if (!SigpipePending()) return 4;
              return 0;
            }),
            "exited 0");
  EXPECT_EQ(InChild([] {
              Block({SIGPIPE});
              ::pthread_kill(::pthread_self(), SIGPIPE);
              { const ScopedSigpipeBlock block; }
              if (!SigpipePending()) return 4;
              return 0;
            }),
            "exited 0");
}
