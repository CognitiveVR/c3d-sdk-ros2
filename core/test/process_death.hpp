#pragma once

#include <sys/wait.h>
#include <unistd.h>

#include <functional>
#include <string>

namespace c3d {
namespace testing_support {

// Runs `body` in a child process that then exits on the spot -- no destructor, no
// orderly anything -- and returns the string `body` produced. A process dying, which a
// leaked object in this one cannot imitate: its threads would keep running, and it would
// keep the spool directory locked, where the kernel releases a dead process's lock.
inline std::string RunAndDieWithoutEnding(const std::function<std::string()>& body) {
  int fds[2];
  if (::pipe(fds) != 0) return {};
  const pid_t child = ::fork();
  if (child == 0) {
    ::close(fds[0]);
    const std::string result = body();
    (void)!::write(fds[1], result.data(), result.size());
    ::_exit(0);
  }
  ::close(fds[1]);
  std::string result;
  char buf[256];
  ssize_t n;
  while ((n = ::read(fds[0], buf, sizeof(buf))) > 0) result.append(buf, static_cast<std::size_t>(n));
  ::close(fds[0]);
  int status = 0;
  ::waitpid(child, &status, 0);
  return result;
}

}  // namespace testing_support
}  // namespace c3d
