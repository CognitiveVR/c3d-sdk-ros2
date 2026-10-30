#include "c3d/h264_encoder.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>

#include "sigpipe_block.hpp"

extern char** environ;

namespace c3d {
namespace {

using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

constexpr std::size_t kStderrTailBytes = 2048;
constexpr int kStdinPipeBytes = 1 << 20;
constexpr std::chrono::milliseconds kPollStep(5);
constexpr Seconds kProbeSeconds(10.0);
constexpr std::uint8_t kNalIdr = 5, kNalSps = 7, kNalPps = 8, kNalAud = 9;

// An integral chunk prints as an integer ("3"), any other with one decimal ("0.5").
std::string ChunkText(double chunk_s) {
  if (chunk_s == std::floor(chunk_s)) return std::to_string(static_cast<long long>(chunk_s));
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f", chunk_s);
  return text;
}

std::string FixedText(double value, int decimals) {
  char text[64];
  std::snprintf(text, sizeof(text), "%.*f", decimals, value);
  return text;
}

std::string SettingsText(const H264EncoderSettings& s) {
  std::string text = "x264 " + s.preset + " crf=" + std::to_string(s.crf) +
                     " threads=" + std::to_string(s.threads) + " gop=" + ChunkText(s.chunk_s) +
                     "s";
  if (s.max_bitrate_kbps > 0) text += " maxrate=" + std::to_string(s.max_bitrate_kbps) + "k";
  return text;
}

void CloseFd(int* fd) {
  if (*fd >= 0) {
    ::close(*fd);
    *fd = -1;
  }
}

// Close-on-exec from birth: a child another encoder spawns in the same window must not inherit an
// end. An inherited stdin write end keeps the first child from ever seeing EOF.
bool MakePipe(int fds[2]) {
#if defined(__linux__)
  return ::pipe2(fds, O_CLOEXEC) == 0;
#else
  if (::pipe(fds) != 0) return false;
  ::fcntl(fds[0], F_SETFD, FD_CLOEXEC);
  ::fcntl(fds[1], F_SETFD, FD_CLOEXEC);
  return true;
#endif
}

void SetNonBlocking(int fd) {
  const int flags = ::fcntl(fd, F_GETFL);
  if (flags >= 0) ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// Wakes whatever polls the other end. The pipe is private and never read, so a full one is as
// good as a written one.
void Poke(int fd) {
  if (fd < 0) return;
  while (::write(fd, "x", 1) < 0 && errno == EINTR) {
  }
}

// Starts `path` with `args` in a process group of its own, with `in`, `out` and `err` as its
// descriptors 0, 1 and 2 and none of this process's others. "" on success, else why not.
std::string Spawn(const std::string& path, const std::vector<std::string>& args, int in, int out,
                  int err, pid_t* pid) {
  std::vector<std::string> words;
  words.reserve(args.size() + 1);
  words.push_back(path);
  words.insert(words.end(), args.begin(), args.end());
  std::vector<char*> argv;
  argv.reserve(words.size() + 1);
  for (std::string& word : words) argv.push_back(&word[0]);
  argv.push_back(nullptr);

  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, in, 0);
  posix_spawn_file_actions_adddup2(&actions, out, 1);
  posix_spawn_file_actions_adddup2(&actions, err, 2);
#ifdef C3D_HAVE_ADDCLOSEFROM_NP
  // Everything else this process holds open without close-on-exec: sockets, spool files, DDS.
  posix_spawn_file_actions_addclosefrom_np(&actions, 3);
#endif
  // Its own group, so a terminal's Ctrl-C reaches this process and not the child, and so a kill
  // can take the whole group, a wrapper script's grandchildren included. An empty signal mask,
  // whatever the spawning thread holds blocked.
  posix_spawnattr_t attributes;
  posix_spawnattr_init(&attributes);
  sigset_t none;
  sigemptyset(&none);
  posix_spawnattr_setsigmask(&attributes, &none);
  posix_spawnattr_setpgroup(&attributes, 0);
  posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK);
  const int rc = ::posix_spawnp(pid, path.c_str(), &actions, &attributes, argv.data(), environ);
  posix_spawnattr_destroy(&attributes);
  posix_spawn_file_actions_destroy(&actions);
  if (rc != 0) {
    *pid = -1;
    return "cannot run " + path + ": " + std::strerror(rc);
  }
  return std::string();
}

enum class ChildState { kRunning, kExited, kGone };

// How `pid` ended, observed WITHOUT reaping it: `code` is the exit status, or the signal when
// `signaled`. kGone is ECHILD: already reaped, by a host that ignores SIGCHLD or by Finish.
ChildState Observe(pid_t pid, int* code, bool* signaled) {
  siginfo_t info;
  std::memset(&info, 0, sizeof(info));
  for (;;) {
    if (::waitid(P_PID, pid, &info, WEXITED | WNOHANG | WNOWAIT) == 0) {
      if (info.si_pid == 0) return ChildState::kRunning;
      *signaled = info.si_code != CLD_EXITED;
      *code = info.si_status;
      return ChildState::kExited;
    }
    if (errno != EINTR) return ChildState::kGone;
  }
}

// One non-blocking reap of `pid` alone, never of any other child.
ChildState Reap(pid_t pid, int* status) {
  for (;;) {
    const pid_t r = ::waitpid(pid, status, WNOHANG);
    if (r == pid) return ChildState::kExited;
    if (r == 0) return ChildState::kRunning;
    if (errno != EINTR) return ChildState::kGone;
  }
}

// Reaps `pid` within `budget`, polling; kRunning when it is still alive at the end.
ChildState ReapWithin(pid_t pid, Seconds budget, int* status) {
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(budget);
  for (;;) {
    const ChildState state = Reap(pid, status);
    if (state != ChildState::kRunning) return state;
    const auto now = Clock::now();
    if (now >= deadline) return ChildState::kRunning;
    std::this_thread::sleep_for(std::min<Clock::duration>(kPollStep, deadline - now));
  }
}

struct Leftovers {
  std::mutex mutex;
  std::vector<pid_t> pids;
};

Leftovers& leftovers() {
  static Leftovers instance;
  return instance;
}

// The reap after a kill: within kH264ReapSeconds, or the pid goes on the leftover list and the
// caller forgets it, never signalling or waiting for it again, which is what keeps a recycled pid
// safe.
ChildState ReapOrLeave(pid_t pid, int* status) {
  const ChildState state = ReapWithin(pid, Seconds(kH264ReapSeconds), status);
  if (state == ChildState::kRunning) {
    std::lock_guard<std::mutex> lock(leftovers().mutex);
    leftovers().pids.push_back(pid);
  }
  return state;
}

// Runs `path` with `args`, stdin and stderr on /dev/null, and collects its stdout. Bounded: a
// probe must never hang its caller. `exit_code` is -1 when the status could not be known.
std::string RunForOutput(const std::string& path, const std::vector<std::string>& args,
                         std::string* out, int* exit_code) {
  *exit_code = -1;
  int devnull = ::open("/dev/null", O_RDWR | O_CLOEXEC);
  int pipe_fds[2] = {-1, -1};
  if (devnull < 0 || !MakePipe(pipe_fds)) {
    const std::string why = "cannot run " + path + ": " + std::strerror(errno);
    CloseFd(&devnull);
    return why;
  }
  pid_t pid = -1;
  const std::string why = Spawn(path, args, devnull, pipe_fds[1], devnull, &pid);
  CloseFd(&devnull);
  CloseFd(&pipe_fds[1]);
  if (!why.empty()) {
    CloseFd(&pipe_fds[0]);
    return why;
  }
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(kProbeSeconds);
  bool timed_out = false;
  char buffer[4096];
  for (;;) {
    const auto now = Clock::now();
    if (now >= deadline) {
      timed_out = true;
      break;
    }
    pollfd readable = {pipe_fds[0], POLLIN, 0};
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const int r = ::poll(&readable, 1, static_cast<int>(left.count()) + 1);
    if (r < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (r == 0) {
      timed_out = true;
      break;
    }
    const ssize_t n = ::read(pipe_fds[0], buffer, sizeof(buffer));
    if (n > 0) {
      out->append(buffer, static_cast<std::size_t>(n));
    } else if (n == 0 || (errno != EINTR && errno != EAGAIN)) {
      break;
    }
  }
  CloseFd(&pipe_fds[0]);
  int status = 0;
  ChildState state = timed_out ? ChildState::kRunning : ReapWithin(pid, Seconds(1.0), &status);
  if (state == ChildState::kRunning) {
    ::kill(-pid, SIGKILL);
    state = ReapOrLeave(pid, &status);
  }
  if (timed_out) return path + " did not answer within " + ChunkText(kProbeSeconds.count()) + " s";
  if (state == ChildState::kExited) {
    *exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  } else if (state == ChildState::kGone) {
    *exit_code = 0;  // reaped by the host: the status is lost and the output stands
  }
  return std::string();
}

class FfmpegH264Encoder final : public IH264Encoder {
 public:
  FfmpegH264Encoder(H264EncoderSettings settings, OnAccessUnit on_access_unit,
                    OnEncoderFailure on_failure)
      : settings_(std::move(settings)),
        on_access_unit_(std::move(on_access_unit)),
        on_failure_(std::move(on_failure)),
        settings_text_(SettingsText(settings_)) {}
  ~FfmpegH264Encoder() override { Finish(0.0); }

  bool Start(std::string* error) override;
  bool Feed(std::string bgr24, std::uint64_t token) override;
  bool Refeed(std::string bgr24, std::uint64_t token) override;
  void Finish(double timeout_s) override;
  std::string codec_params() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return avcc_;
  }
  std::string encoder_settings() const override { return settings_text_; }
  H264EncoderStats stats() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }

 private:
  void WriterLoop();
  bool WriteFrame(const std::string& frame);
  void ReaderLoop();
  void AppendStderr(const char* data, std::size_t n);
  void EmitCompleteUnits(std::string* buffer);
  void EmitUnit(std::string bytes);
  void ReportChildDeath();
  void ReportFailure(std::string why, int exit_code, bool killed, std::uint64_t abandoned,
                     const std::string& stderr_tail);
  void CloseEverything();

  const H264EncoderSettings settings_;
  const OnAccessUnit on_access_unit_;
  const OnEncoderFailure on_failure_;
  const std::string settings_text_;

  pid_t pid_ = -1;
  int stdin_w_ = -1, stdout_r_ = -1, stderr_r_ = -1;
  int writer_stop_[2] = {-1, -1};
  int reader_stop_[2] = {-1, -1};
  std::thread writer_, reader_;

  std::atomic<bool> finishing_{false};
  std::atomic<bool> failure_reported_{false};

  mutable std::mutex mutex_;
  std::condition_variable frames_cv_;  // the writer: a frame, closing, or abort
  std::condition_variable eof_cv_;     // Finish: the reader saw EOF
  bool started_ = false;
  bool closing_ = false;       // drain the deque, then close stdin
  bool abort_writer_ = false;  // leave whatever is queued
  bool saw_eof_ = false;
  bool child_failed_ = false;
  std::deque<std::string> queue_;
  std::deque<std::uint64_t> tokens_;
  std::string sps_, pps_, avcc_;
  std::string stderr_tail_;
  std::string pending_;  // the reader's open unit at EOF: the tail Finish may deliver
  H264EncoderStats stats_;
};

bool FfmpegH264Encoder::Start(std::string* error) {
  // Finish is final, even before a Start: nothing would ever stop a child started after it.
  if (finishing_) {
    if (error) *error = "already finished";
    return false;
  }
  SweepLeftoverChildren();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_) {
      if (error) *error = "already started";
      return false;
    }
  }
  if (settings_.width == 0 || settings_.height == 0 || !(settings_.fps > 0.0)) {
    if (error) *error = "width, height and fps must be positive";
    return false;
  }
  int in[2] = {-1, -1}, out[2] = {-1, -1}, err[2] = {-1, -1};
  const auto close_all = [&] {
    for (int* fd : {&in[0], &in[1], &out[0], &out[1], &err[0], &err[1], &writer_stop_[0],
                    &writer_stop_[1], &reader_stop_[0], &reader_stop_[1]}) {
      CloseFd(fd);
    }
  };
  if (!MakePipe(in) || !MakePipe(out) || !MakePipe(err) || !MakePipe(writer_stop_) ||
      !MakePipe(reader_stop_)) {
    if (error) *error = std::string("pipe: ") + std::strerror(errno);
    close_all();
    return false;
  }
#ifdef F_SETPIPE_SZ
  // Best effort: room for a frame in flight without the writer blocking on it.
  ::fcntl(in[1], F_SETPIPE_SZ, kStdinPipeBytes);
#endif
  const std::string why =
      Spawn(settings_.ffmpeg_path, FfmpegArguments(settings_), in[0], out[1], err[1], &pid_);
  CloseFd(&in[0]);
  CloseFd(&out[1]);
  CloseFd(&err[1]);
  if (!why.empty()) {
    if (error) *error = why;
    close_all();
    return false;
  }
  stdin_w_ = in[1];
  stdout_r_ = out[0];
  stderr_r_ = err[0];
  SetNonBlocking(stdin_w_);
  SetNonBlocking(stdout_r_);
  SetNonBlocking(stderr_r_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = true;
  }
  writer_ = std::thread(&FfmpegH264Encoder::WriterLoop, this);
  reader_ = std::thread(&FfmpegH264Encoder::ReaderLoop, this);
  return true;
}

bool FfmpegH264Encoder::Feed(std::string bgr24, std::uint64_t token) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || finishing_ || child_failed_) return false;
  if (queue_.size() >= kH264QueueFrames) {
    ++stats_.dropped_queue_full;
    return false;
  }
  queue_.push_back(std::move(bgr24));
  tokens_.push_back(token);
  ++stats_.fed;
  frames_cv_.notify_one();
  return true;
}

bool FfmpegH264Encoder::Refeed(std::string bgr24, std::uint64_t token) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || finishing_ || child_failed_) return false;
  queue_.push_back(std::move(bgr24));
  tokens_.push_back(token);
  ++stats_.fed;
  frames_cv_.notify_one();
  return true;
}

void FfmpegH264Encoder::WriterLoop() {
  // EPIPE, not death, when the child has gone before its input has.
  ScopedSigpipeBlock sigpipe;
  for (;;) {
    std::string frame;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      frames_cv_.wait(lock, [this] { return abort_writer_ || closing_ || !queue_.empty(); });
      if (abort_writer_) break;
      if (queue_.empty()) break;  // closing and drained: EOF is what makes the child flush and exit
      frame = std::move(queue_.front());
      queue_.pop_front();
    }
    if (!WriteFrame(frame)) break;
  }
  CloseFd(&stdin_w_);
}

bool FfmpegH264Encoder::WriteFrame(const std::string& frame) {
  std::size_t written = 0;
  while (written < frame.size()) {
    pollfd fds[2] = {{stdin_w_, POLLOUT, 0}, {writer_stop_[0], POLLIN, 0}};
    const int r = ::poll(fds, 2, -1);
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (fds[1].revents != 0) return false;  // told to stop: the rest of this frame is abandoned
    if (fds[0].revents == 0) continue;
    // Written even when poll reports an error on the pipe: the write's EPIPE is what says the
    // child's end is gone, and SIGPIPE is held off this thread.
    const ssize_t n = ::write(stdin_w_, frame.data() + written, frame.size() - written);
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
}

void FfmpegH264Encoder::ReaderLoop() {
  std::string buffer;
  char chunk[65536];
  bool stdout_eof = false, stderr_eof = false;
  while (!stdout_eof) {
    pollfd fds[3] = {{stdout_r_, POLLIN, 0},
                     {stderr_eof ? -1 : stderr_r_, POLLIN, 0},
                     {reader_stop_[0], POLLIN, 0}};
    const int r = ::poll(fds, 3, -1);
    if (r < 0) {
      if (errno == EINTR) continue;
      return;
    }
    if (fds[2].revents != 0) return;  // told to stop: nothing more is delivered, EOF or not
    if (fds[0].revents != 0) {
      const ssize_t n = ::read(stdout_r_, chunk, sizeof(chunk));
      if (n > 0) {
        buffer.append(chunk, static_cast<std::size_t>(n));
        EmitCompleteUnits(&buffer);
      } else if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
        stdout_eof = true;
      }
    }
    if (!stderr_eof && fds[1].revents != 0) {
      const ssize_t n = ::read(stderr_r_, chunk, sizeof(chunk));
      if (n > 0) {
        AppendStderr(chunk, static_cast<std::size_t>(n));
      } else if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
        stderr_eof = true;
      }
    }
  }
  // Whatever stderr still holds, without waiting for more of it.
  if (!stderr_eof) {
    ssize_t n = 0;
    while ((n = ::read(stderr_r_, chunk, sizeof(chunk))) > 0) {
      AppendStderr(chunk, static_cast<std::size_t>(n));
    }
  }
  // The child closed its output before it was asked to.
  if (!finishing_) ReportChildDeath();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = std::move(buffer);
    saw_eof_ = true;
  }
  eof_cv_.notify_all();
}

void FfmpegH264Encoder::AppendStderr(const char* data, std::size_t n) {
  std::lock_guard<std::mutex> lock(mutex_);
  stderr_tail_.append(data, n);
  if (stderr_tail_.size() > kStderrTailBytes) {
    stderr_tail_.erase(0, stderr_tail_.size() - kStderrTailBytes);
  }
}

// Delivers every access unit `buffer` completes, a delimiter up to the next one, and keeps the
// open unit from its delimiter on.
void FfmpegH264Encoder::EmitCompleteUnits(std::string* buffer) {
  const std::vector<NalUnit> nals = SplitNalUnits(*buffer);
  std::size_t open = std::string::npos;
  for (const NalUnit& nal : nals) {
    if (nal.type != kNalAud) continue;
    if (open != std::string::npos) EmitUnit(buffer->substr(open, nal.offset - open));
    open = nal.offset;
  }
  if (open == std::string::npos) {
    // Nothing before the first delimiter is a frame; the last three bytes may begin a start code.
    if (buffer->size() > 3) buffer->erase(0, buffer->size() - 3);
    return;
  }
  buffer->erase(0, open);
}

void FfmpegH264Encoder::EmitUnit(std::string bytes) {
  const std::vector<NalUnit> nals = SplitNalUnits(bytes);
  EncodedAccessUnit unit;
  for (const NalUnit& nal : nals) {
    if (nal.type == kNalIdr) unit.keyframe = true;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const NalUnit& nal : nals) {
      if (nal.type != kNalSps && nal.type != kNalPps) continue;
      const std::size_t start_code = bytes[nal.offset + 2] == '\x01' ? 3 : 4;
      if (nal.length <= start_code) continue;
      std::string& slot = nal.type == kNalSps ? sps_ : pps_;
      if (slot.empty()) slot = bytes.substr(nal.offset + start_code, nal.length - start_code);
    }
    if (avcc_.empty() && !sps_.empty() && !pps_.empty()) avcc_ = BuildAvcC(sps_, pps_);
    if (!tokens_.empty()) {
      unit.token = tokens_.front();
      tokens_.pop_front();
    } else {
      unit.paired = false;
    }
    ++stats_.access_units;
    unit.codec_params = avcc_;
  }
  unit.bytes = std::move(bytes);
  unit.encoder_settings = settings_text_;
  if (on_access_unit_) on_access_unit_(std::move(unit));
}

// The reader's path: the child closed its output on its own. Its exit is observed, never reaped,
// and reported once. From here no Feed is accepted, so the count of frames that never came back is
// final.
void FfmpegH264Encoder::ReportChildDeath() {
  int code = 0;
  bool signaled = false;
  ChildState state = ChildState::kRunning;
  while (state == ChildState::kRunning) {
    if (finishing_) return;  // Finish owns the ending from here
    state = Observe(pid_, &code, &signaled);
    if (state != ChildState::kRunning) break;
    pollfd stop = {reader_stop_[0], POLLIN, 0};
    if (::poll(&stop, 1, 10) > 0) return;
  }
  if (finishing_) return;
  std::uint64_t abandoned = 0;
  std::string stderr_tail;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    child_failed_ = true;
    abandoned = stats_.fed - stats_.access_units;
    stderr_tail = stderr_tail_;
  }
  std::string why;
  int exit_code = -1;
  if (state == ChildState::kGone) {
    why = "ffmpeg exited before its input was closed, and the host reaped it first";
  } else if (signaled) {
    why = "ffmpeg died from signal " + std::to_string(code);
  } else {
    exit_code = code;
    why = code == 0 ? "ffmpeg exited 0 before its input was closed"
                    : "ffmpeg exited " + std::to_string(code);
  }
  ReportFailure(why, exit_code, false, abandoned, stderr_tail);
}

void FfmpegH264Encoder::ReportFailure(std::string why, int exit_code, bool killed,
                                      std::uint64_t abandoned, const std::string& stderr_tail) {
  if (abandoned > 0) {
    why += "; " + std::to_string(abandoned) + " accepted frames never came back as access units";
  }
  if (!stderr_tail.empty()) why += "; stderr: " + stderr_tail;
  if (failure_reported_.exchange(true)) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.failed = true;
    stats_.failure = why;
  }
  if (on_failure_) {
    EncoderFailure failure;
    failure.why = why;
    failure.exit_code = exit_code;
    failure.killed = killed;
    failure.abandoned = abandoned;
    on_failure_(failure);
  }
}

void FfmpegH264Encoder::Finish(double timeout_s) {
  if (finishing_.exchange(true)) return;
  SweepLeftoverChildren();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_) return;
    closing_ = true;
  }
  frames_cv_.notify_all();
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                           Seconds(std::max(0.0, timeout_s)));

  // The child's exit and the reader's EOF, in either order, up to the deadline. The exit is
  // observed, not reaped: the leader's pid, and with it the group id the kill names, stays
  // reserved until the kill has been sent.
  int observed_code = 0;
  bool observed_signaled = false;
  ChildState child = ChildState::kRunning;
  bool eof = false;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      if (child == ChildState::kRunning) child = Observe(pid_, &observed_code, &observed_signaled);
      eof = saw_eof_;
      if (child != ChildState::kRunning && eof) break;
      const auto now = Clock::now();
      if (now >= deadline) break;
      eof_cv_.wait_until(lock, std::min(now + kPollStep, deadline));
    }
  }
  const bool drained = child != ChildState::kRunning && eof;
  bool killed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    abort_writer_ = true;
  }
  frames_cv_.notify_all();
  // Drained, nothing the writer still holds could be encoded, so stopping it only makes the join
  // unconditional. At the deadline it abandons what is left, the reader returns without EOF, and
  // the whole group goes, grandchildren holding the pipes included.
  Poke(writer_stop_[1]);
  if (!drained) {
    Poke(reader_stop_[1]);
    if (settings_.kill_fn) {
      settings_.kill_fn(-pid_, SIGKILL);
    } else {
      ::kill(-pid_, SIGKILL);
    }
    killed = true;
  }
  int status = 0;
  const ChildState reaped = ReapOrLeave(pid_, &status);
  if (writer_.joinable()) writer_.join();
  if (reader_.joinable()) reader_.join();

  // The exit status: from the reap when it happened here, else from the observation. A host that
  // reaped the child itself left none, and absent a kill that reads as a clean exit.
  bool exited_zero = false;
  int exit_code = -1;
  if (reaped == ChildState::kExited) {
    if (WIFEXITED(status)) {
      exit_code = WEXITSTATUS(status);
      exited_zero = exit_code == 0;
    }
  } else if (reaped == ChildState::kGone) {
    if (child == ChildState::kExited) {
      if (!observed_signaled) exit_code = observed_code;
      exited_zero = !observed_signaled && observed_code == 0;
    } else {
      exited_zero = !killed;
    }
  }

  // The EOF-terminated last unit, on this thread, only when the stream is complete: a child that
  // died mid-packet leaves a truncated slice, which is not a frame.
  if (drained && exited_zero) {
    std::string tail;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      tail.swap(pending_);
    }
    const std::vector<NalUnit> nals = SplitNalUnits(tail);
    if (!nals.empty() && nals.front().offset == 0 && nals.front().type == kNalAud) {
      EmitUnit(std::move(tail));
    }
  }

  std::uint64_t abandoned = 0;
  std::string stderr_tail;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    abandoned = stats_.fed - stats_.access_units;
    stats_.abandoned = abandoned;
    stderr_tail = stderr_tail_;
  }
  CloseEverything();
  if (killed || !exited_zero || abandoned > 0) {
    std::string why;
    if (killed) {
      why = "ffmpeg did not stop within " + FixedText(timeout_s, 2) +
            " s and its process group was killed";
      if (reaped == ChildState::kRunning) {
        why += "; it has not died yet and is left to a later sweep";
      }
    } else if (exit_code >= 0) {
      why = "ffmpeg exited " + std::to_string(exit_code);
    } else {
      why = "ffmpeg died from a signal";
    }
    ReportFailure(why, exit_code, killed, abandoned, stderr_tail);
  }
}

void FfmpegH264Encoder::CloseEverything() {
  CloseFd(&stdin_w_);
  CloseFd(&stdout_r_);
  CloseFd(&stderr_r_);
  CloseFd(&writer_stop_[0]);
  CloseFd(&writer_stop_[1]);
  CloseFd(&reader_stop_[0]);
  CloseFd(&reader_stop_[1]);
}

}  // namespace

std::unique_ptr<IH264Encoder> NewFfmpegH264Encoder(const H264EncoderSettings& settings,
                                                   OnAccessUnit on_access_unit,
                                                   OnEncoderFailure on_failure) {
  return std::make_unique<FfmpegH264Encoder>(settings, std::move(on_access_unit),
                                             std::move(on_failure));
}

std::string ProbeFfmpeg(const std::string& ffmpeg_path, std::string* version_out) {
  std::string output;
  int exit_code = -1;
  std::string why = RunForOutput(ffmpeg_path, {"-hide_banner", "-version"}, &output, &exit_code);
  if (!why.empty()) return why;
  if (version_out) *version_out = output.substr(0, output.find('\n'));
  if (exit_code != 0) {
    return ffmpeg_path + " -version " +
           (exit_code < 0 ? std::string("died") : "exited " + std::to_string(exit_code));
  }
  output.clear();
  why = RunForOutput(ffmpeg_path, {"-hide_banner", "-encoders"}, &output, &exit_code);
  if (!why.empty()) return why;
  if (output.find("libx264") == std::string::npos) {
    return ffmpeg_path + " lists no libx264 encoder";
  }
  return std::string();
}

std::string ProbeFfmpegEncode(H264EncoderSettings settings) {
  settings.width = 16;
  settings.height = 16;
  settings.fps = 30.0;
  std::mutex mutex;
  std::size_t units = 0;
  std::string failure;
  auto encoder = NewFfmpegH264Encoder(
      settings,
      [&](EncodedAccessUnit) {
        std::lock_guard<std::mutex> lock(mutex);
        ++units;
      },
      [&](const EncoderFailure& reported) {
        std::lock_guard<std::mutex> lock(mutex);
        failure = reported.why;
      });
  std::string error;
  if (!encoder->Start(&error)) return error;
  const std::string black(16 * 16 * 3, '\0');
  for (std::uint64_t token = 0; token < 2; ++token) encoder->Feed(black, token);
  encoder->Finish(kProbeSeconds.count());
  std::lock_guard<std::mutex> lock(mutex);
  if (!failure.empty()) return failure;
  if (units != 2) {
    return settings.ffmpeg_path + " returned " + std::to_string(units) +
           " access units for 2 frames";
  }
  return std::string();
}

std::vector<std::string> FfmpegArguments(const H264EncoderSettings& s) {
  std::vector<std::string> args = {
      "-hide_banner", "-loglevel", "error", "-nostats",
      "-f", "rawvideo", "-pix_fmt", "bgr24",
      "-video_size", std::to_string(s.width) + "x" + std::to_string(s.height),
      "-framerate", FixedText(s.fps, 6),
      "-thread_queue_size", "2",
      "-use_wallclock_as_timestamps", "1",
      // The input is fully declared; probing its frame rate holds the first frames for seconds.
      "-fpsprobesize", "0",
      "-i", "pipe:0",
      "-an", "-vf", "setpts=PTS-STARTPTS", "-vsync", "passthrough",
      "-c:v", "libx264", "-preset", s.preset, "-crf", std::to_string(s.crf),
      "-g", "1000000", "-keyint_min", "1000000", "-sc_threshold", "0",
      "-threads", std::to_string(s.threads),
      // Counted from the last forced keyframe, not from 0: after a gap in the input, one keyframe
      // is forced rather than one for every chunk length the gap spanned.
      "-force_key_frames",
      "expr:if(isnan(prev_forced_t),1,gte(t,prev_forced_t+" + ChunkText(s.chunk_s) + "))",
      "-x264-params", "aud=1:repeat-headers=1:bframes=0",
      "-pix_fmt", "yuv420p"};
  if (s.max_bitrate_kbps > 0) {
    args.insert(args.end(), {"-maxrate", std::to_string(s.max_bitrate_kbps) + "k",
                             "-bufsize", std::to_string(2LL * s.max_bitrate_kbps) + "k"});
  }
  args.insert(args.end(), {"-flush_packets", "1", "-f", "h264", "pipe:1"});
  return args;
}

std::vector<NalUnit> SplitNalUnits(const std::string& annexb) {
  std::vector<NalUnit> nals;
  const std::size_t n = annexb.size();
  std::size_t i = 0;
  while (i + 2 < n) {
    if (annexb[i] != '\0' || annexb[i + 1] != '\0' || annexb[i + 2] != '\x01') {
      ++i;
      continue;
    }
    NalUnit nal;
    nal.offset = (i > 0 && annexb[i - 1] == '\0') ? i - 1 : i;
    nal.type = i + 3 < n ? static_cast<std::uint8_t>(annexb[i + 3] & 0x1F) : 0;
    if (!nals.empty()) nals.back().length = nal.offset - nals.back().offset;
    nals.push_back(nal);
    i += 3;
  }
  if (!nals.empty()) nals.back().length = n - nals.back().offset;
  return nals;
}

std::string BuildAvcC(const std::string& sps, const std::string& pps) {
  if (sps.size() < 4 || pps.empty() || sps.size() > 0xFFFF || pps.size() > 0xFFFF) return {};
  const auto put16 = [](std::string* out, std::size_t v) {
    out->push_back(static_cast<char>((v >> 8) & 0xFF));
    out->push_back(static_cast<char>(v & 0xFF));
  };
  std::string record;
  record.push_back('\x01');
  record.append(sps, 1, 3);  // profile_idc, constraint flags, level_idc
  record.push_back('\xFF');  // 6 reserved bits, lengthSizeMinusOne 3
  record.push_back('\xE1');  // 3 reserved bits, one SPS
  put16(&record, sps.size());
  record += sps;
  record.push_back('\x01');  // one PPS
  put16(&record, pps.size());
  record += pps;
  const unsigned profile_idc = static_cast<unsigned char>(sps[1]);
  if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 || profile_idc == 144) {
    record.push_back('\xFD');  // 6 reserved bits, chroma_format 1
    record.push_back('\xF8');  // 5 reserved bits, bit_depth_luma_minus8 0
    record.push_back('\xF8');  // 5 reserved bits, bit_depth_chroma_minus8 0
    record.push_back('\x00');  // no SPS extensions
  }
  return record;
}

void SweepLeftoverChildren() {
  Leftovers& list = leftovers();
  std::lock_guard<std::mutex> lock(list.mutex);
  for (auto it = list.pids.begin(); it != list.pids.end();) {
    int status = 0;
    if (Reap(*it, &status) == ChildState::kRunning) {
      ++it;
    } else {
      it = list.pids.erase(it);
    }
  }
}

std::size_t LeftoverChildCount() {
  Leftovers& list = leftovers();
  std::lock_guard<std::mutex> lock(list.mutex);
  return list.pids.size();
}

}  // namespace c3d
