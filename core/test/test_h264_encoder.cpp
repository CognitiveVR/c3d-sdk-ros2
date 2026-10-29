// The ffmpeg H.264 encoder, against the real binary and against shell scripts standing in for it.
//
// The real-binary tests take `ffmpeg` from PATH. A host without one FAILS them, unless
// C3D_TEST_WITHOUT_FFMPEG=1 is set, in which case they skip with a message: the test image always
// has the binary, so CI can never skip them silently, and a developer's host opts out explicitly.
// The scripts stand in where the behaviour under test is a child that misbehaves: one that never
// reads, one that dies, one that leaves a grandchild holding the pipes, one that will not die.

#include <gtest/gtest.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "c3d/h264_encoder.hpp"

using namespace c3d;
using namespace std::chrono_literals;

namespace {

using Clock = std::chrono::steady_clock;

double SecondsSince(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

bool WaitUntil(const std::function<bool()>& ready, std::chrono::milliseconds budget) {
  const auto deadline = Clock::now() + budget;
  while (!ready()) {
    if (Clock::now() >= deadline) return false;
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

// FAILs the test on a host without a usable ffmpeg, unless C3D_TEST_WITHOUT_FFMPEG=1 opts out.
#define REQUIRE_FFMPEG()                                                                  \
  do {                                                                                    \
    std::string version;                                                                  \
    const std::string why = ProbeFfmpeg("ffmpeg", &version);                              \
    if (!why.empty()) {                                                                   \
      const char* opt_out = std::getenv("C3D_TEST_WITHOUT_FFMPEG");                       \
      if (opt_out != nullptr && std::string(opt_out) == "1") GTEST_SKIP() << why;         \
      FAIL() << why << " (set C3D_TEST_WITHOUT_FFMPEG=1 to skip the real-ffmpeg tests)";  \
    }                                                                                     \
  } while (0)

// A directory for the scripts that stand in for ffmpeg and for the files they write.
class ScratchDir {
 public:
  ScratchDir() {
    char tmpl[] = "/tmp/c3d_h264_XXXXXX";
    const char* made = ::mkdtemp(tmpl);
    path_ = made != nullptr ? made : "";
  }
  ~ScratchDir() {
    for (const std::string& file : files_) ::unlink(file.c_str());
    if (!path_.empty()) ::rmdir(path_.c_str());
  }
  std::string Path(const std::string& name) {
    const std::string path = path_ + "/" + name;
    files_.push_back(path);
    return path;
  }
  // A `#!/bin/sh` script, executable, standing in for ffmpeg.
  std::string Script(const std::string& name, const std::string& body) {
    const std::string path = Path(name);
    {
      std::ofstream out(path);
      out << "#!/bin/sh\n" << body << "\n";
    }
    ::chmod(path.c_str(), 0755);
    return path;
  }

 private:
  std::string path_;
  std::vector<std::string> files_;
};

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

// Everything the callbacks deliver, from whichever thread they run on.
struct Collector {
  std::mutex mutex;
  std::vector<EncodedAccessUnit> units;
  std::vector<EncoderFailure> failures;

  OnAccessUnit OnUnit() {
    return [this](EncodedAccessUnit unit) {
      std::lock_guard<std::mutex> lock(mutex);
      units.push_back(std::move(unit));
    };
  }
  OnEncoderFailure OnFailure() {
    return [this](const EncoderFailure& failure) {
      std::lock_guard<std::mutex> lock(mutex);
      failures.push_back(failure);
    };
  }
  std::size_t UnitCount() {
    std::lock_guard<std::mutex> lock(mutex);
    return units.size();
  }
  std::size_t FailureCount() {
    std::lock_guard<std::mutex> lock(mutex);
    return failures.size();
  }
  std::string FirstFailure() {
    std::lock_guard<std::mutex> lock(mutex);
    return failures.empty() ? std::string() : failures.front().why;
  }
  std::vector<std::uint64_t> Tokens() {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<std::uint64_t> tokens;
    for (const EncodedAccessUnit& unit : units) tokens.push_back(unit.token);
    return tokens;
  }
  std::vector<std::size_t> KeyframeIndices() {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<std::size_t> indices;
    for (std::size_t i = 0; i < units.size(); ++i) {
      if (units[i].keyframe) indices.push_back(i);
    }
    return indices;
  }
};

std::unique_ptr<IH264Encoder> NewEncoder(const H264EncoderSettings& settings, Collector* c) {
  return NewFfmpegH264Encoder(settings, c->OnUnit(), c->OnFailure());
}

H264EncoderSettings Small(double fps, double chunk_s) {
  H264EncoderSettings settings;
  settings.width = 64;
  settings.height = 48;
  settings.fps = fps;
  settings.chunk_s = chunk_s;
  return settings;
}

// A bgr24 gradient that moves with `i`, so consecutive frames are not identical.
std::string Frame(int width, int height, int i) {
  std::string frame(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3, '\0');
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t at = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                              static_cast<std::size_t>(x)) * 3;
      frame[at] = static_cast<char>((x * 4 + i * 8) & 0xFF);
      frame[at + 1] = static_cast<char>((y * 5 + i * 3) & 0xFF);
      frame[at + 2] = static_cast<char>((x + y + i * 2) & 0xFF);
    }
  }
  return frame;
}

// ffmpeg stamps each frame with the wall clock as it reads it, and the first frame's stamp is the
// zero every keyframe mark counts from. Fed while ffmpeg is still starting, frame 0 waits in the
// pipe and every mark lands late by the start-up time; fed this long after Start, it arrives while
// ffmpeg is already blocked on an empty pipe.
constexpr std::chrono::milliseconds kFfmpegStartUp(1000);

// Feeds `count` 64x48 frames `period` apart on the wall clock, each at its own slot from the first
// rather than `period` after the previous call returned, so sleep latency does not accumulate.
void FeedPaced(IH264Encoder& encoder, int count, std::chrono::milliseconds period) {
  const auto start = Clock::now();
  for (int i = 0; i < count; ++i) {
    const auto slot = start + period * i;
    const auto now = Clock::now();
    if (slot > now) std::this_thread::sleep_for(slot - now);
    EXPECT_TRUE(encoder.Feed(Frame(64, 48, i), static_cast<std::uint64_t>(i))) << "frame " << i;
  }
}

bool BeginsWithAud(const std::string& bytes) {
  const std::string four("\x00\x00\x00\x01\x09", 5);
  const std::string three("\x00\x00\x01\x09", 4);
  return bytes.compare(0, four.size(), four) == 0 || bytes.compare(0, three.size(), three) == 0;
}

std::string Bytes(std::initializer_list<int> values) {
  std::string out;
  for (int v : values) out.push_back(static_cast<char>(v));
  return out;
}

// Finishes an encoder whose kill is a no-op, so its child lands on the leftover list; returns the
// child's pid (what the kill was asked to signal, negated). The caller kills it for real.
pid_t LeaveAChildOnTheList(ScratchDir* dir, int* kills) {
  std::atomic<pid_t> target{0};
  H264EncoderSettings settings = Small(30.0, 1.0);
  settings.ffmpeg_path = dir->Script("sleeper.sh", "exec sleep 30");
  settings.kill_fn = [&target, kills](pid_t pid, int) {
    target = pid;
    ++*kills;
    return 0;
  };
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  EXPECT_TRUE(encoder->Start(&error)) << error;
  encoder->Finish(0.1);
  EXPECT_LT(target.load(), 0);
  return -target.load();
}

void KillForRealAndWaitForTheZombie(pid_t pid) {
  ::kill(-pid, SIGKILL);
  siginfo_t info;
  std::memset(&info, 0, sizeof(info));
  ASSERT_EQ(::waitid(P_PID, pid, &info, WEXITED | WNOWAIT), 0) << std::strerror(errno);
}

}  // namespace

TEST(H264Encoder, TheArgumentsPinTheMeasuredSettings) {
  H264EncoderSettings settings;
  settings.width = 640;
  settings.height = 480;
  settings.fps = 30.0;
  const std::vector<std::string> expected = {
      "-hide_banner", "-loglevel", "error", "-nostats",
      "-f", "rawvideo", "-pix_fmt", "bgr24", "-video_size", "640x480",
      "-framerate", "30.000000", "-thread_queue_size", "2",
      "-use_wallclock_as_timestamps", "1", "-fpsprobesize", "0", "-i", "pipe:0",
      "-an", "-vf", "setpts=PTS-STARTPTS", "-vsync", "passthrough",
      "-c:v", "libx264", "-preset", "ultrafast", "-crf", "28",
      "-g", "1000000", "-keyint_min", "1000000", "-sc_threshold", "0", "-threads", "1",
      "-force_key_frames", "expr:gte(t,n_forced*3)",
      "-x264-params", "aud=1:repeat-headers=1:bframes=0", "-pix_fmt", "yuv420p",
      "-flush_packets", "1", "-f", "h264", "pipe:1"};
  EXPECT_EQ(FfmpegArguments(settings), expected);

  settings.max_bitrate_kbps = 800;
  std::vector<std::string> capped = expected;
  const auto after_pix_fmt = std::find(capped.begin(), capped.end(), "yuv420p") + 1;
  capped.insert(after_pix_fmt, {"-maxrate", "800k", "-bufsize", "1600k"});
  EXPECT_EQ(FfmpegArguments(settings), capped);
  {
    Collector capped_settings;
    EXPECT_EQ(NewEncoder(settings, &capped_settings)->encoder_settings(),
              "x264 ultrafast crf=28 threads=1 gop=3s maxrate=800k");
  }
  // The buffer is twice the cap, which an int cannot hold for every cap an int can.
  settings.max_bitrate_kbps = 2147483647;
  const std::vector<std::string> widest = FfmpegArguments(settings);
  const auto bufsize = std::find(widest.begin(), widest.end(), "-bufsize");
  ASSERT_NE(bufsize, widest.end());
  EXPECT_EQ(*(bufsize + 1), "4294967294k");

  settings.max_bitrate_kbps = 0;
  settings.chunk_s = 0.5;
  const std::vector<std::string> half = FfmpegArguments(settings);
  const auto forced = std::find(half.begin(), half.end(), "-force_key_frames");
  ASSERT_NE(forced, half.end());
  EXPECT_EQ(*(forced + 1), "expr:gte(t,n_forced*0.5)");

  settings.chunk_s = 3.0;
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  EXPECT_EQ(encoder->encoder_settings(), "x264 ultrafast crf=28 threads=1 gop=3s");
  EXPECT_TRUE(encoder->codec_params().empty());
}

TEST(H264Encoder, SplitNalUnitsFindsBothStartCodeLengths) {
  const std::string buffer = Bytes({0x00, 0x00, 0x00, 0x01, 0x67, 0xAA, 0xBB,
                                    0x00, 0x00, 0x01, 0x68, 0xCC,
                                    0x00, 0x00, 0x01, 0x65, 0xDD, 0xEE});
  const std::vector<NalUnit> nals = SplitNalUnits(buffer);
  ASSERT_EQ(nals.size(), 3u);
  EXPECT_EQ(nals[0].type, 7);
  EXPECT_EQ(nals[0].offset, 0u);
  EXPECT_EQ(nals[0].length, 7u);
  EXPECT_EQ(nals[1].type, 8);
  EXPECT_EQ(nals[1].offset, 7u);
  EXPECT_EQ(nals[1].length, 5u);
  EXPECT_EQ(nals[2].type, 5);
  EXPECT_EQ(nals[2].offset, 12u);
  EXPECT_EQ(nals[2].length, 6u);
  EXPECT_TRUE(SplitNalUnits("").empty());
  EXPECT_TRUE(SplitNalUnits(Bytes({0x00, 0x00})).empty());
}

TEST(H264Encoder, BuildAvcCFromKnownSpsPps) {
  const std::string baseline_sps = Bytes({0x67, 0x42, 0xC0, 0x1E, 0xDA, 0x02, 0x80, 0xF6, 0x80});
  const std::string pps = Bytes({0x68, 0xCE, 0x38, 0x80});
  const std::string baseline = BuildAvcC(baseline_sps, pps);
  const std::string expected_baseline =
      Bytes({0x01, 0x42, 0xC0, 0x1E, 0xFF, 0xE1, 0x00, 0x09}) + baseline_sps +
      Bytes({0x01, 0x00, 0x04}) + pps;
  EXPECT_EQ(baseline, expected_baseline);

  const std::string high_sps = Bytes({0x67, 0x64, 0x00, 0x1F, 0xAC, 0xD9, 0x40, 0x50, 0x05, 0xBB});
  const std::string high = BuildAvcC(high_sps, pps);
  const std::string expected_high =
      Bytes({0x01, 0x64, 0x00, 0x1F, 0xFF, 0xE1, 0x00, 0x0A}) + high_sps +
      Bytes({0x01, 0x00, 0x04}) + pps + Bytes({0xFD, 0xF8, 0xF8, 0x00});
  EXPECT_EQ(high, expected_high);
}

TEST(H264Encoder, EncodesEveryFrameInOrderWithAKeyframeEveryChunk) {
  REQUIRE_FFMPEG();
  Collector c;
  auto encoder = NewEncoder(Small(10.0, 1.0), &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  std::this_thread::sleep_for(kFfmpegStartUp);
  FeedPaced(*encoder, 25, 100ms);
  encoder->Finish(1.0);

  EXPECT_EQ(c.FailureCount(), 0u) << c.FirstFailure();
  ASSERT_EQ(c.UnitCount(), 25u);
  std::vector<std::uint64_t> expected_tokens;
  for (std::uint64_t i = 0; i < 25; ++i) expected_tokens.push_back(i);
  EXPECT_EQ(c.Tokens(), expected_tokens);

  const std::vector<std::size_t> keyframes = c.KeyframeIndices();
  ASSERT_EQ(keyframes.size(), 3u) << "keyframes at " << ::testing::PrintToString(keyframes);
  EXPECT_EQ(keyframes[0], 0u);
  EXPECT_TRUE(keyframes[1] == 10 || keyframes[1] == 11) << keyframes[1];
  EXPECT_TRUE(keyframes[2] == 20 || keyframes[2] == 21) << keyframes[2];

  const std::string codec_params = encoder->codec_params();
  ASSERT_FALSE(codec_params.empty());
  EXPECT_EQ(static_cast<unsigned char>(codec_params[0]), 0x01);
  {
    std::lock_guard<std::mutex> lock(c.mutex);
    for (const EncodedAccessUnit& unit : c.units) {
      EXPECT_TRUE(BeginsWithAud(unit.bytes)) << "token " << unit.token;
      EXPECT_EQ(unit.codec_params, codec_params);
      EXPECT_EQ(unit.encoder_settings, "x264 ultrafast crf=28 threads=1 gop=1s");
    }
  }
  const H264EncoderStats stats = encoder->stats();
  EXPECT_EQ(stats.fed, 25u);
  EXPECT_EQ(stats.access_units, 25u);
  EXPECT_EQ(stats.dropped_queue_full, 0u);
  EXPECT_EQ(stats.abandoned, 0u);
  EXPECT_FALSE(stats.failed);
}

TEST(H264Encoder, AChunkIsOneSecondOfRealTimeWhateverTheDeclaredRate) {
  REQUIRE_FFMPEG();
  Collector c;
  auto encoder = NewEncoder(Small(30.0, 1.0), &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  std::this_thread::sleep_for(kFfmpegStartUp);
  // 14 frames 0.25 s apart: 3.25 s of real time against 0.47 s of the declared 30 fps clock. The
  // keyframe at each real second may land on the frame after the mark, when the quantised stamp of
  // the one at the mark falls just short; the 14th frame exists so the third mark has that slack.
  FeedPaced(*encoder, 14, 250ms);
  encoder->Finish(1.0);

  EXPECT_EQ(c.FailureCount(), 0u) << c.FirstFailure();
  ASSERT_EQ(c.UnitCount(), 14u);
  const std::vector<std::size_t> keyframes = c.KeyframeIndices();
  ASSERT_EQ(keyframes.size(), 4u) << "keyframes at " << ::testing::PrintToString(keyframes);
  EXPECT_EQ(keyframes[0], 0u);
  for (std::size_t k = 1; k <= 3; ++k) {
    EXPECT_TRUE(keyframes[k] == 4 * k || keyframes[k] == 4 * k + 1)
        << "keyframe " << k << " at " << keyframes[k];
  }
}

TEST(H264Encoder, EachChunkDecodesOnItsOwn) {
  REQUIRE_FFMPEG();
  ScratchDir dir;
  Collector c;
  auto encoder = NewEncoder(Small(10.0, 0.5), &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  std::this_thread::sleep_for(kFfmpegStartUp);
  FeedPaced(*encoder, 16, 100ms);
  encoder->Finish(1.0);
  EXPECT_EQ(c.FailureCount(), 0u) << c.FirstFailure();
  ASSERT_EQ(c.UnitCount(), 16u);
  const std::vector<std::size_t> keyframes = c.KeyframeIndices();
  ASSERT_GE(keyframes.size(), 3u) << "keyframes at " << ::testing::PrintToString(keyframes);

  // The second chunk: from the second keyframe up to the third, and nothing before it.
  const std::string chunk_path = dir.Path("chunk.h264");
  std::size_t frames = 0;
  std::string shape;  // each unit's NAL types, for the message when the count is off
  {
    std::lock_guard<std::mutex> lock(c.mutex);
    std::ofstream out(chunk_path, std::ios::binary);
    for (std::size_t i = keyframes[1]; i < keyframes[2]; ++i) {
      out.write(c.units[i].bytes.data(), static_cast<std::streamsize>(c.units[i].bytes.size()));
      ++frames;
      shape += "[";
      for (const NalUnit& nal : SplitNalUnits(c.units[i].bytes)) {
        shape += " " + std::to_string(nal.type);
      }
      shape += " ]";
    }
  }
  ASSERT_GE(frames, 2u);

  // Every decoded frame, as decoded: without passthrough the decoder's rate conversion may drop
  // or repeat one, and what is being counted is the frames the chunk carries.
  const std::string command = "ffmpeg -hide_banner -loglevel error -f h264 -i " + chunk_path +
                              " -vsync passthrough -f rawvideo -pix_fmt gray -";
  FILE* decoder = ::popen(command.c_str(), "r");
  ASSERT_NE(decoder, nullptr);
  std::size_t decoded_bytes = 0;
  char buffer[4096];
  std::size_t n = 0;
  while ((n = std::fread(buffer, 1, sizeof(buffer), decoder)) > 0) decoded_bytes += n;
  const int status = ::pclose(decoder);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  EXPECT_EQ(decoded_bytes, frames * 64 * 48) << "NAL types per unit: " << shape;
}

// At 2 Hz the first unit is due as the third frame is fed, 1 s after the first: x264 holds a frame
// until the next one's stamp gives it a duration, and a unit is complete only when the next one's
// delimiter arrives. A probe of the input's frame rate would hold the first frames for seconds.
TEST(H264Encoder, TheFirstAccessUnitArrivesWithoutWaitingForAProbe) {
  REQUIRE_FFMPEG();
  Collector c;
  std::mutex mutex;
  std::vector<Clock::time_point> arrivals;
  const OnAccessUnit collect = c.OnUnit();
  auto encoder = NewFfmpegH264Encoder(
      Small(2.0, 3.0),
      [&](EncodedAccessUnit unit) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          arrivals.push_back(Clock::now());
        }
        collect(std::move(unit));
      },
      c.OnFailure());
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  std::this_thread::sleep_for(kFfmpegStartUp);
  const auto first_feed = Clock::now();
  FeedPaced(*encoder, 6, 500ms);
  encoder->Finish(1.0);

  EXPECT_EQ(c.FailureCount(), 0u) << c.FirstFailure();
  EXPECT_EQ(c.UnitCount(), 6u);
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_FALSE(arrivals.empty());
  std::vector<double> seconds;
  for (const Clock::time_point& at : arrivals) {
    seconds.push_back(std::chrono::duration<double>(at - first_feed).count());
  }
  EXPECT_LT(seconds.front(), 1.5) << "units arrived " << ::testing::PrintToString(seconds)
                                  << " s after the first Feed";
}

TEST(H264Encoder, AFullQueueDropsTheFrameAndCounts) {
  ScratchDir dir;
  H264EncoderSettings settings;
  settings.width = 640;
  settings.height = 480;
  settings.ffmpeg_path = dir.Script("never_reads.sh", "exec sleep 30");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;

  const std::string frame = Frame(640, 480, 0);
  std::uint64_t refused = 0;
  for (std::uint64_t i = 0; i < 10; ++i) {
    if (!encoder->Feed(frame, i)) ++refused;
  }
  H264EncoderStats stats = encoder->stats();
  EXPECT_GE(stats.dropped_queue_full, 3u);
  EXPECT_EQ(stats.dropped_queue_full, refused);
  EXPECT_EQ(stats.fed + refused, 10u);

  const auto start = Clock::now();
  encoder->Finish(0.5);
  EXPECT_LT(SecondsSince(start), 1.0);
  stats = encoder->stats();
  EXPECT_TRUE(stats.failed);
  EXPECT_EQ(stats.abandoned, stats.fed);
  EXPECT_EQ(c.UnitCount(), 0u);
  EXPECT_EQ(c.FailureCount(), 1u);
}

// A replacement encoder re-encodes every frame its predecessor's chunk refused, which can be more
// than the queue holds: Refeed takes them all, and none is counted as dropped.
TEST(H264Encoder, RefeedTakesMoreFramesThanTheQueueHolds) {
  ScratchDir dir;
  H264EncoderSettings settings = Small(30.0, 3.0);
  settings.ffmpeg_path = dir.Script("never_reads.sh", "exec sleep 30");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  for (std::uint64_t i = 0; i < 3 * kH264QueueFrames; ++i) {
    EXPECT_TRUE(encoder->Refeed(Frame(64, 48, static_cast<int>(i)), i)) << "frame " << i;
  }
  const H264EncoderStats stats = encoder->stats();
  EXPECT_EQ(stats.fed, 3 * kH264QueueFrames);
  EXPECT_EQ(stats.dropped_queue_full, 0u);
  encoder->Finish(0.0);
  EXPECT_FALSE(encoder->Refeed(Frame(64, 48, 0), 99)) << "taken after Finish";
}

TEST(H264Encoder, RefedFramesComeBackInOrderBeforeTheNextFed) {
  REQUIRE_FFMPEG();
  Collector c;
  auto encoder = NewEncoder(Small(30.0, 3.0), &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  const std::uint64_t refed = 3 * kH264QueueFrames;
  for (std::uint64_t i = 0; i < refed; ++i) {
    ASSERT_TRUE(encoder->Refeed(Frame(64, 48, static_cast<int>(i)), i)) << "frame " << i;
  }
  ASSERT_TRUE(WaitUntil([&] { return c.UnitCount() + 2 >= refed; }, 10000ms));
  ASSERT_TRUE(encoder->Feed(Frame(64, 48, 99), refed));
  encoder->Finish(5.0);
  EXPECT_EQ(c.FailureCount(), 0u) << c.FirstFailure();
  std::vector<std::uint64_t> expected;
  for (std::uint64_t i = 0; i <= refed; ++i) expected.push_back(i);
  EXPECT_EQ(c.Tokens(), expected);
  ASSERT_FALSE(c.KeyframeIndices().empty());
  EXPECT_EQ(c.KeyframeIndices().front(), 0u) << "the first refed frame is not a keyframe";
}

TEST(H264Encoder, AMissingBinaryFailsStartWithTheReason) {
  H264EncoderSettings settings = Small(30.0, 3.0);
  settings.ffmpeg_path = "/nonexistent/ffmpeg";
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  EXPECT_FALSE(encoder->Start(&error));
  EXPECT_NE(error.find("/nonexistent/ffmpeg"), std::string::npos) << error;
  EXPECT_FALSE(encoder->Feed(Frame(64, 48, 0), 0));
  encoder->Finish(1.0);
  EXPECT_EQ(c.FailureCount(), 0u);
}

TEST(H264Encoder, AStartAfterFinishIsRefusedAndRunsNothing) {
  ScratchDir dir;
  const std::string ran = dir.Path("ran");
  H264EncoderSettings settings = Small(30.0, 3.0);
  settings.ffmpeg_path = dir.Script("marks.sh", "touch " + ran + "\nexec cat >/dev/null");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  // A stop that ran ahead of the start: the encoder is final from here.
  encoder->Finish(0.0);
  std::string error;
  if (encoder->Start(&error)) {
    // No Finish is left to stop what that Start began, and destroying the encoder would block
    // forever on its writer's wait, so it is leaked: the test fails instead of hanging.
    static_cast<void>(encoder.release());
    FAIL() << "Start after Finish started a child";
  }
  EXPECT_NE(error.find("finished"), std::string::npos) << error;
  EXPECT_FALSE(encoder->Feed(Frame(64, 48, 0), 0));
  encoder.reset();
  std::this_thread::sleep_for(100ms);
  EXPECT_NE(::access(ran.c_str(), F_OK), 0) << "the child ran";
  EXPECT_EQ(c.FailureCount(), 0u);
  EXPECT_EQ(c.UnitCount(), 0u);
}

TEST(H264Encoder, ProbeNamesWhatIsWrong) {
  std::string version;
  const std::string no_encoder = ProbeFfmpeg("/bin/true", &version);
  EXPECT_NE(no_encoder.find("lists no libx264 encoder"), std::string::npos) << no_encoder;
  EXPECT_NE(no_encoder.find("/bin/true"), std::string::npos) << no_encoder;

  const std::string missing = ProbeFfmpeg("/nonexistent", &version);
  EXPECT_NE(missing.find("cannot run"), std::string::npos) << missing;
  EXPECT_NE(missing.find("/nonexistent"), std::string::npos) << missing;
}

TEST(H264Encoder, AChildThatDiesReportsFailureOnce) {
  ScratchDir dir;
  H264EncoderSettings settings = Small(30.0, 3.0);
  // One read, then its input is closed while it still lives, then exit 1. The frames written to a
  // closed input in that window are what SIGPIPE would answer, were it not held off the writer.
  settings.ffmpeg_path =
      dir.Script("dies.sh", "head -c 16 >/dev/null\nexec 0<&-\nsleep 0.2\nexit 1");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  ASSERT_TRUE(encoder->Feed(Frame(64, 48, 0), 0));

  std::uint64_t next = 1;
  ASSERT_TRUE(WaitUntil(
      [&] {
        encoder->Feed(Frame(64, 48, next), next);
        ++next;
        return c.FailureCount() >= 1;
      },
      3000ms));
  EXPECT_EQ(c.FailureCount(), 1u);
  {
    std::lock_guard<std::mutex> lock(c.mutex);
    EXPECT_EQ(c.failures[0].exit_code, 1);
    EXPECT_FALSE(c.failures[0].killed);
    EXPECT_NE(c.failures[0].why.find("exited 1"), std::string::npos) << c.failures[0].why;
  }
  EXPECT_FALSE(encoder->Feed(Frame(64, 48, 1), 1));
  H264EncoderStats stats = encoder->stats();
  EXPECT_TRUE(stats.failed);
  EXPECT_FALSE(stats.failure.empty());

  encoder->Finish(1.0);
  EXPECT_EQ(c.FailureCount(), 1u);
  stats = encoder->stats();
  EXPECT_TRUE(stats.failed);
  EXPECT_GE(stats.fed, 1u);
  EXPECT_EQ(stats.abandoned, stats.fed);
  EXPECT_EQ(c.UnitCount(), 0u);
}

TEST(H264Encoder, ATruncatedTailAfterACrashIsNotAFrame) {
  ScratchDir dir;
  H264EncoderSettings settings = Small(30.0, 3.0);
  // An AUD and the first bytes of an IDR slice, then death: no following AUD ever ends the unit.
  settings.ffmpeg_path = dir.Script(
      "truncates.sh",
      "printf '\\000\\000\\000\\001\\011\\360\\000\\000\\000\\001\\145\\210\\204'\nexit 1");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  ASSERT_TRUE(encoder->Feed(Frame(64, 48, 0), 0));

  ASSERT_TRUE(WaitUntil([&] { return c.FailureCount() >= 1; }, 3000ms));
  EXPECT_EQ(c.UnitCount(), 0u);
  encoder->Finish(1.0);
  EXPECT_EQ(c.UnitCount(), 0u);
  EXPECT_EQ(c.FailureCount(), 1u);
  EXPECT_TRUE(encoder->stats().failed);
  EXPECT_TRUE(encoder->codec_params().empty());
}

TEST(H264Encoder, FinishKillsAStuckChildWithinTheTimeout) {
  ScratchDir dir;
  const std::string pid_file = dir.Path("grandchild.pid");
  H264EncoderSettings settings = Small(30.0, 3.0);
  // The grandchild inherits stdout and holds it open; the script itself just waits.
  settings.ffmpeg_path = dir.Script("wrapper.sh",
                                    "sleep 30 &\necho $! > " + pid_file + "\nwait");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  ASSERT_TRUE(WaitUntil([&] { return !ReadFile(pid_file).empty(); }, 3000ms));
  const pid_t grandchild = static_cast<pid_t>(std::stol(ReadFile(pid_file)));
  ASSERT_GT(grandchild, 1);
  ASSERT_EQ(::kill(grandchild, 0), 0) << "the grandchild should be alive before Finish";

  const auto start = Clock::now();
  encoder->Finish(0.5);
  EXPECT_LT(SecondsSince(start), 1.0);
  EXPECT_TRUE(encoder->stats().failed);
  EXPECT_EQ(c.FailureCount(), 1u);

  // Reparented to the container's init and reaped asynchronously: either gone or a zombie.
  const auto gone = [&] {
    if (::kill(grandchild, 0) != 0) return errno == ESRCH;
    const std::string stat = ReadFile("/proc/" + std::to_string(grandchild) + "/stat");
    const std::size_t close = stat.rfind(')');
    return close != std::string::npos && close + 2 < stat.size() && stat[close + 2] == 'Z';
  };
  EXPECT_TRUE(WaitUntil(gone, 1000ms)) << "the grandchild is still running";
}

TEST(H264Encoder, TheChildHoldsOnlyItsThreeDescriptors) {
#if !defined(__linux__)
  GTEST_SKIP() << "reads /proc";
#else
  ScratchDir dir;
  const std::string listing = dir.Path("fds.txt");
  // Descriptors without close-on-exec in this process, which a child must still not inherit.
  int spare[2] = {-1, -1};
  ASSERT_EQ(::pipe(spare), 0);
  const int file = ::open(dir.Path("open.txt").c_str(), O_RDWR | O_CREAT, 0644);
  ASSERT_GE(file, 0);

  H264EncoderSettings settings = Small(30.0, 3.0);
  settings.ffmpeg_path = dir.Script("lists.sh", "ls /proc/self/fd > " + listing);
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  ASSERT_TRUE(WaitUntil([&] { return c.FailureCount() >= 1; }, 3000ms));
  encoder->Finish(1.0);
  ::close(spare[0]);
  ::close(spare[1]);
  ::close(file);

  std::vector<int> fds;
  std::istringstream lines(ReadFile(listing));
  std::string line;
  while (std::getline(lines, line)) {
    if (!line.empty()) fds.push_back(std::stoi(line));
  }
  std::sort(fds.begin(), fds.end());
  // 0, 1 and 2, plus the directory descriptor `ls` holds on /proc/self/fd while listing it.
  EXPECT_EQ(fds, (std::vector<int>{0, 1, 2, 3})) << ReadFile(listing);
#endif
}

TEST(H264Encoder, AChildThatWillNotDieGoesOnTheLeftoverListAndIsSweptLater) {
  SweepLeftoverChildren();
  ASSERT_EQ(LeftoverChildCount(), 0u);
  ScratchDir dir;
  std::atomic<int> kills{0};
  std::atomic<pid_t> target{0};
  std::atomic<int> signal_sent{0};
  H264EncoderSettings settings = Small(30.0, 1.0);
  settings.ffmpeg_path = dir.Script("sleeper.sh", "exec sleep 30");
  settings.kill_fn = [&](pid_t pid, int signal) {
    target = pid;
    signal_sent = signal;
    ++kills;
    return 0;
  };
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;

  const auto start = Clock::now();
  encoder->Finish(0.3);
  EXPECT_LT(SecondsSince(start), 0.3 + kH264ReapSeconds + 0.1);
  EXPECT_EQ(kills.load(), 1);
  EXPECT_EQ(signal_sent.load(), SIGKILL);
  ASSERT_LT(target.load(), 0) << "the kill names the process group, negated";
  const pid_t pid = -target.load();
  EXPECT_EQ(LeftoverChildCount(), 1u);
  EXPECT_EQ(::kill(pid, 0), 0) << "the child is still alive";
  EXPECT_TRUE(encoder->stats().failed);
  EXPECT_EQ(c.FailureCount(), 1u);

  SweepLeftoverChildren();
  EXPECT_EQ(LeftoverChildCount(), 1u);

  KillForRealAndWaitForTheZombie(pid);
  SweepLeftoverChildren();
  EXPECT_EQ(LeftoverChildCount(), 0u);

  encoder->Finish(0.3);
  EXPECT_EQ(kills.load(), 1);
  EXPECT_EQ(c.FailureCount(), 1u);
}

TEST(H264Encoder, TheSweepNeverStealsASiblingsExitStatus) {
  SweepLeftoverChildren();
  ASSERT_EQ(LeftoverChildCount(), 0u);
  ScratchDir dir;
  int kills = 0;
  const pid_t leftover = LeaveAChildOnTheList(&dir, &kills);
  ASSERT_EQ(LeftoverChildCount(), 1u);

  const pid_t sibling = ::fork();
  if (sibling == 0) ::_exit(7);
  ASSERT_GT(sibling, 0);
  siginfo_t info;
  std::memset(&info, 0, sizeof(info));
  ASSERT_EQ(::waitid(P_PID, sibling, &info, WEXITED | WNOWAIT), 0);

  SweepLeftoverChildren();

  int status = 0;
  EXPECT_EQ(::waitpid(sibling, &status, 0), sibling) << std::strerror(errno);
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 7);

  KillForRealAndWaitForTheZombie(leftover);
  SweepLeftoverChildren();
  EXPECT_EQ(LeftoverChildCount(), 0u);
}

TEST(H264Encoder, StartSweepsTheLeftoverList) {
  SweepLeftoverChildren();
  ASSERT_EQ(LeftoverChildCount(), 0u);
  ScratchDir dir;
  int kills = 0;
  const pid_t leftover = LeaveAChildOnTheList(&dir, &kills);
  ASSERT_EQ(LeftoverChildCount(), 1u);
  KillForRealAndWaitForTheZombie(leftover);
  ASSERT_EQ(LeftoverChildCount(), 1u) << "dead, but nothing has swept yet";

  H264EncoderSettings settings = Small(30.0, 1.0);
  settings.ffmpeg_path = dir.Script("another_sleeper.sh", "exec sleep 30");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  EXPECT_EQ(LeftoverChildCount(), 0u);
  encoder->Finish(0.1);
  EXPECT_EQ(LeftoverChildCount(), 0u);
}

TEST(H264Encoder, FinishSweepsTheLeftoverList) {
  SweepLeftoverChildren();
  ASSERT_EQ(LeftoverChildCount(), 0u);
  ScratchDir dir;
  int kills = 0;
  const pid_t leftover = LeaveAChildOnTheList(&dir, &kills);
  ASSERT_EQ(LeftoverChildCount(), 1u);
  KillForRealAndWaitForTheZombie(leftover);
  ASSERT_EQ(LeftoverChildCount(), 1u) << "dead, but nothing has swept yet";

  // Never started, so the sweep is all this Finish can do.
  Collector c;
  auto encoder = NewEncoder(Small(30.0, 1.0), &c);
  encoder->Finish(0.1);
  EXPECT_EQ(LeftoverChildCount(), 0u);
  EXPECT_EQ(c.FailureCount(), 0u);
}

TEST(H264Encoder, EchildReadsAsGone) {
  REQUIRE_FFMPEG();
  // A host that ignores SIGCHLD auto-reaps its children: every wait answers ECHILD.
  struct sigaction ignore;
  std::memset(&ignore, 0, sizeof(ignore));
  ignore.sa_handler = SIG_IGN;
  sigemptyset(&ignore.sa_mask);
  struct sigaction previous;
  std::memset(&previous, 0, sizeof(previous));
  ASSERT_EQ(::sigaction(SIGCHLD, &ignore, &previous), 0);
  struct Restore {
    struct sigaction* previous;
    ~Restore() { ::sigaction(SIGCHLD, previous, nullptr); }
  } restore{&previous};

  Collector c;
  auto encoder = NewEncoder(Small(30.0, 3.0), &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  for (int i = 0; i < 3; ++i) ASSERT_TRUE(encoder->Feed(Frame(64, 48, i), i));
  const auto start = Clock::now();
  encoder->Finish(1.0);
  EXPECT_LT(SecondsSince(start), 0.5) << "Finish waited for a deadline it should not have";
  EXPECT_EQ(c.FailureCount(), 0u) << c.FirstFailure();
  EXPECT_EQ(c.UnitCount(), 3u);
  EXPECT_FALSE(encoder->stats().failed);
}

TEST(H264Encoder, AnAbandonedTailIsCountedAndReportedOnce) {
  ScratchDir dir;
  H264EncoderSettings settings = Small(30.0, 3.0);
  settings.ffmpeg_path = dir.Script("swallows.sh", "cat >/dev/null\nexec sleep 30");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  for (int i = 0; i < 3; ++i) ASSERT_TRUE(encoder->Feed(Frame(64, 48, i), i));

  const auto start = Clock::now();
  encoder->Finish(0.3);
  EXPECT_LT(SecondsSince(start), 1.0);
  const H264EncoderStats stats = encoder->stats();
  EXPECT_EQ(stats.fed, 3u);
  EXPECT_EQ(stats.access_units, 0u);
  EXPECT_EQ(stats.abandoned, 3u);
  EXPECT_TRUE(stats.failed);
  ASSERT_EQ(c.FailureCount(), 1u);
  {
    std::lock_guard<std::mutex> lock(c.mutex);
    EXPECT_EQ(c.failures[0].abandoned, 3u);
    EXPECT_NE(c.failures[0].why.find("3 accepted frames never came back"), std::string::npos)
        << c.failures[0].why;
  }
  encoder->Finish(0.3);
  EXPECT_EQ(c.FailureCount(), 1u);
}

TEST(H264Encoder, FinishDeliversEveryAcceptedFrameBeforeClosingTheInput) {
  REQUIRE_FFMPEG();
  H264EncoderSettings settings;
  settings.width = 1280;
  settings.height = 720;
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  // 2.76 MB a frame, larger than the pipe: frames are still queued or mid-write when Finish starts.
  for (int i = 0; i < 4; ++i) ASSERT_TRUE(encoder->Feed(Frame(1280, 720, i), i));
  encoder->Finish(1.5);
  EXPECT_EQ(c.FailureCount(), 0u) << c.FirstFailure();
  EXPECT_EQ(c.UnitCount(), 4u);
  EXPECT_EQ(c.Tokens(), (std::vector<std::uint64_t>{0, 1, 2, 3}));
  EXPECT_EQ(encoder->stats().abandoned, 0u);
}

TEST(H264Encoder, TwoEncodersDoNotShareEachOthersPipes) {
  REQUIRE_FFMPEG();
  Collector a_units, b_units;
  auto a = NewEncoder(Small(30.0, 3.0), &a_units);
  auto b = NewEncoder(Small(30.0, 3.0), &b_units);
  std::string error;
  ASSERT_TRUE(a->Start(&error)) << error;
  ASSERT_TRUE(b->Start(&error)) << error;
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(a->Feed(Frame(64, 48, i), i));
    ASSERT_TRUE(b->Feed(Frame(64, 48, i), i));
  }
  // An inherited stdin write end in B would keep A's child from ever seeing EOF.
  const auto start = Clock::now();
  a->Finish(1.0);
  EXPECT_LT(SecondsSince(start), 0.9) << "A's Finish ran to its deadline";
  EXPECT_EQ(a_units.FailureCount(), 0u) << a_units.FirstFailure();
  EXPECT_EQ(a_units.UnitCount(), 3u);
  EXPECT_FALSE(a->stats().failed);

  b->Finish(1.0);
  EXPECT_EQ(b_units.FailureCount(), 0u) << b_units.FirstFailure();
  EXPECT_EQ(b_units.UnitCount(), 3u);
}

TEST(H264Encoder, TheChildIsInItsOwnProcessGroup) {
#if !defined(__linux__)
  GTEST_SKIP() << "reads /proc";
#else
  ScratchDir dir;
  const std::string stat_copy = dir.Path("stat.txt");
  H264EncoderSettings settings = Small(30.0, 3.0);
  settings.ffmpeg_path =
      dir.Script("stat.sh", "cat /proc/self/stat > " + stat_copy + "\nexec sleep 30");
  Collector c;
  auto encoder = NewEncoder(settings, &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  ASSERT_TRUE(WaitUntil([&] { return ReadFile(stat_copy).find(')') != std::string::npos; },
                        3000ms));
  std::this_thread::sleep_for(20ms);
  const std::string stat = ReadFile(stat_copy);
  encoder->Finish(0.2);

  // Field 2 is the command in parentheses; the state, ppid and pgrp follow the closing one.
  const std::size_t close = stat.rfind(')');
  ASSERT_NE(close, std::string::npos) << stat;
  std::istringstream rest(stat.substr(close + 1));
  std::string state;
  long ppid = 0, pgrp = 0;
  rest >> state >> ppid >> pgrp;
  // `cat` is the script's child, so its parent is the encoder's child, and that child leads the
  // group: a group of its own, not this process's.
  EXPECT_EQ(pgrp, ppid) << stat;
  EXPECT_NE(pgrp, static_cast<long>(::getpgrp())) << stat;
  EXPECT_GT(pgrp, 1) << stat;
#endif
}

TEST(H264Encoder, FinishIsIdempotentAndDeliversTheTail) {
  REQUIRE_FFMPEG();
  Collector c;
  auto encoder = NewEncoder(Small(30.0, 3.0), &c);
  std::string error;
  ASSERT_TRUE(encoder->Start(&error)) << error;
  // kH264QueueFrames frames at once: every one fits the deque whether or not the writer has
  // taken the first yet.
  for (int i = 0; i < 4; ++i) ASSERT_TRUE(encoder->Feed(Frame(64, 48, i), i));
  encoder->Finish(1.0);
  // Every access unit, the EOF-terminated last one included, arrived before Finish returned.
  EXPECT_EQ(c.UnitCount(), 4u);
  EXPECT_EQ(c.Tokens(), (std::vector<std::uint64_t>{0, 1, 2, 3}));
  EXPECT_EQ(c.FailureCount(), 0u) << c.FirstFailure();
  const H264EncoderStats first = encoder->stats();
  EXPECT_EQ(first.access_units, 4u);
  EXPECT_FALSE(first.failed);

  encoder->Finish(1.0);
  EXPECT_EQ(c.UnitCount(), 4u);
  EXPECT_EQ(c.FailureCount(), 0u);
  const H264EncoderStats second = encoder->stats();
  EXPECT_EQ(second.access_units, first.access_units);
  EXPECT_EQ(second.abandoned, 0u);
  EXPECT_FALSE(encoder->Feed(Frame(64, 48, 4), 4));
}
