// The H.264 encoder: the host's `ffmpeg`, run as a child process. x264 is GPL and is never linked;
// the SDK writes bgr24 frames into the child's stdin and splits the Annex-B stream it writes back
// into access units, one callback per frame. ROS-free like the rest of core/.
//
// The stop path is clock-bounded by design: Finish(timeout_s) has one deadline and always returns
// with both of its threads joined and every pipe end closed, whatever the child does.
#pragma once

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "c3d/sensor_blocks.hpp"

namespace c3d {

struct H264EncoderSettings {
  std::string ffmpeg_path = "ffmpeg";  // searched on PATH unless it contains a slash
  std::string preset = "ultrafast";
  int crf = 28;
  int max_bitrate_kbps = 0;  // 0 is uncapped; otherwise -maxrate K -bufsize 2K
  int threads = 1;
  // The keyframe interval, and so a chunk's length, in REAL time: ffmpeg stamps each frame with the
  // wall clock as it reads it, whatever `fps` declares.
  double chunk_s = kVideoChunkSeconds;
  std::uint16_t width = 0, height = 0;
  double fps = 30.0;  // x264's nominal rate and the stamp resolution, 1 / fps
  // Test seam: the kill Finish sends at its deadline. Nothing in an unprivileged container survives
  // SIGKILL on purpose, so the leftover-list path can only be exercised by a kill that does
  // nothing. Production never sets it.
  std::function<int(pid_t pgid_or_pid, int signal)> kill_fn;  // empty = ::kill
};

// What on_failure receives: enough to say what happened without touching the instance from the
// callback.
struct EncoderFailure {
  std::string why;
  // -1 when the child died from a signal, was never reaped here, or the host auto-reaped it.
  int exit_code = -1;
  bool killed = false;
  std::uint64_t abandoned = 0;
};

// codec_params and encoder_settings ride on every unit, copied from the encoder that produced it,
// so a consumer never reads them through a pointer that a restart may have reassigned.
struct EncodedAccessUnit {
  std::uint64_t token = 0;
  // False for a unit that came back with no fed frame left to pair it with: ffmpeg returned more
  // units than frames, so the pairing of every later unit with its frame is in doubt. `token` is
  // then meaningless.
  bool paired = true;
  std::string bytes;  // Annex B, from the access unit delimiter through the frame's last NAL
  bool keyframe = false;
  std::string codec_params;
  std::string encoder_settings;
};

// `abandoned` = frames Feed accepted that never came back as access units because Finish reached
// its deadline or the child died: fed - access_units at the end. Those frames are in no other
// counter.
struct H264EncoderStats {
  std::uint64_t fed = 0, access_units = 0, dropped_queue_full = 0, abandoned = 0;
  bool failed = false;
  std::string failure;
};

// Called on the encoder's reader thread, in feed order. The EOF-terminated last unit, whose fate
// depends on the exit code, is delivered on the thread that called Finish, after waitpid.
using OnAccessUnit = std::function<void(EncodedAccessUnit)>;
// At most once per encoder (an atomic flag): from the reader thread when the child dies, from the
// Finish caller's thread otherwise.
using OnEncoderFailure = std::function<void(const EncoderFailure& failure)>;

// Start, Finish and destruction belong to one owner: never two of them at once, and neither Finish
// nor destruction from inside a callback, which runs on the reader thread Finish joins or inside
// Finish itself. Feed, stats, codec_params and encoder_settings may be called from any thread,
// callbacks included, concurrently with each other and with Start and Finish, until destruction
// begins; no lock is held while a callback runs.
class IH264Encoder {
 public:
  virtual ~IH264Encoder() = default;
  // False, with the reason in `error`, when already started, after any Finish (one before Start
  // included: Finish is final), when width, height or fps is not positive, or when the child
  // cannot be run.
  virtual bool Start(std::string* error) = 0;
  // Never blocks. `bgr24` must be exactly width * height * 3 bytes: the size is not checked, and a
  // frame of any other size misaligns every frame after it in the raw stream. False = queue full,
  // the frame dropped and counted; also false, and not counted, before Start, after Finish and
  // once the child has died.
  virtual bool Feed(std::string bgr24, std::uint64_t token) = 0;
  // As Feed, without the kH264QueueFrames bound: for the frames a replacement encoder re-encodes,
  // which can be more than the queue holds. False only when Feed would be false and not count.
  virtual bool Refeed(std::string bgr24, std::uint64_t token) = 0;
  // One deadline, now + timeout_s, over the whole shutdown: no new Feed is accepted; the writer
  // keeps draining the frames already accepted until the deque is empty, then stdin is closed; the
  // reader keeps delivering access units until the child exits; at the deadline whatever is left is
  // abandoned (counted in stats().abandoned), the child's whole PROCESS GROUP is SIGKILLed and the
  // reader is told to stop through its stop pipe (it never needs EOF). Always returns with both
  // threads joined and every pipe end closed; the child is reaped here when it dies within
  // kH264ReapSeconds of the kill, else its pid goes on the leftover list (below) and this instance
  // forgets it. Reported failed on a non-zero exit, a kill or a non-zero `abandoned`. Finish(0)
  // kills at once and delivers nothing. Idempotent; a second call does nothing. The destructor
  // calls Finish(0).
  virtual void Finish(double timeout_s) = 0;
  virtual std::string codec_params() const = 0;      // avcC, empty until the first SPS/PPS
  virtual std::string encoder_settings() const = 0;  // "x264 ultrafast crf=28 threads=1 gop=3s"
  virtual H264EncoderStats stats() const = 0;
};

std::unique_ptr<IH264Encoder> NewFfmpegH264Encoder(const H264EncoderSettings& settings,
                                                   OnAccessUnit on_access_unit,
                                                   OnEncoderFailure on_failure);
// "" when `ffmpeg_path` runs and its `-encoders` lists libx264; otherwise why not.
std::string ProbeFfmpeg(const std::string& ffmpeg_path, std::string* version_out);
// "" when `settings.ffmpeg_path` encodes two 16x16 frames through FfmpegArguments with the rest of
// `settings` -- the preset, crf and bitrate cap a camera runs with -- into two access units;
// otherwise why not. An ffmpeg that lists libx264 can still refuse one of the arguments, and would
// then fail every start. Bounded like the other probes.
std::string ProbeFfmpegEncode(H264EncoderSettings settings);
// argv after the program name.
std::vector<std::string> FfmpegArguments(const H264EncoderSettings& settings);

// `offset` is the start code's, `length` runs through to the next start code.
struct NalUnit {
  std::uint8_t type = 0;
  std::size_t offset = 0, length = 0;
};
// Both 3- and 4-byte start codes.
std::vector<NalUnit> SplitNalUnits(const std::string& annexb);
// Version 1, profile/compat/level from sps[1..3], lengthSizeMinusOne 3, one SPS, one PPS; for
// profile_idc 100, 110, 122 or 144 (x264's High profiles) also the ISO 14496-15 extension:
// chroma_format 1, bit_depth_luma_minus8 0, bit_depth_chroma_minus8 0, 0 SPS extensions. `sps` and
// `pps` are the NAL units without their start codes, header byte included.
std::string BuildAvcC(const std::string& sps, const std::string& pps);

constexpr std::size_t kH264QueueFrames = 4;
// The stop budget. A healthy ultrafast drain is milliseconds, and veryfast at 640x480 on a Pi 5
// drains its 10-frame lookahead plus the up to 8 frames between the SDK and x264 (deque 4, writer
// 1, pipe 1, ffmpeg's input queue 2) in about 0.25 s. It is NOT sized for a saturated encoder: at
// 1080p on a Pi 5 veryfast's queue is always full and a stop abandons up to 18 frames, counted in
// `abandoned` and said once. 1.0 s plus kH264ReapSeconds keeps a node's deactivate (1.25 s plus
// End's spool writes) inside the 2.0 s shutdown-timing budget and the SIGINT path (1.25 s +
// shutdown_drain_s 2.0 + curl's 1 s interrupt poll + End) inside launch's 5 s escalation.
constexpr double kH264FinishTimeoutSeconds = 1.0;
// waitpid(WNOHANG) polling after the kill before the pid goes on the leftover list.
constexpr double kH264ReapSeconds = 0.25;

// Children that did not die within kH264ReapSeconds of their kill (uninterruptible sleep, a
// cgroup-v1 freezer). No thread waits on them: every later Start and Finish sweeps the list with
// waitpid(pid, WNOHANG) for each entry, never waitpid(-1), so no exit status of any other child
// (ProbeFfmpeg's, another camera's, another component's) is ever stolen, nothing holds a reference
// to an encoder or a capture, no thread can outlive a component unload or hang static destruction,
// and a pid stays reserved by the kernel until it is reaped, so no later spawn can receive it. A
// child still in that state at process exit is reparented to init and reaped when it leaves it.
// The list is a plain mutex-guarded vector of pids; a sweep is ONE critical section: the lock
// covers the iteration, every waitpid and the erase of what it reaped. The reader thread never
// reaps: a child that died is observed with waitid(WNOWAIT) only, and the one reap is Finish's.
// ECHILD from any wait reads as "gone" (a host that inherited SIGCHLD = SIG_IGN auto-reaps its
// children; Finish must not take ECHILD for "still running" and pay the timeout), and the sweep
// erases such an entry.
void SweepLeftoverChildren();
std::size_t LeftoverChildCount();

}  // namespace c3d
