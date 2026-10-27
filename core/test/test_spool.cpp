#include <gtest/gtest.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <fstream>

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "c3d/spool.hpp"

using namespace c3d;
namespace fs = std::filesystem;

namespace {

class SpoolTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("c3d_spool_" + std::to_string(::getpid()) + "_" +
                                        std::to_string(counter_++));
    fs::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  Chunk MakeChunk(Stream s, const std::string& body) {
    Chunk c;
    c.url = "https://example.invalid/v0/x";
    c.body = body;
    c.session_id = "sess";
    c.stream = s;
    c.created_at = 1.0;
    return c;
  }
  fs::path dir_;
  static int counter_;
};
int SpoolTest::counter_ = 0;

}  // namespace

TEST_F(SpoolTest, WriteThenPeekRoundTrips) {
  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kEvents, R"({"hello":"world"})")));
  EXPECT_EQ(spool.Count(), 1u);

  Chunk out;
  ASSERT_TRUE(spool.PeekOldest(&out));
  EXPECT_EQ(out.body, R"({"hello":"world"})");
  EXPECT_EQ(out.session_id, "sess");
  EXPECT_EQ(out.stream, Stream::kEvents);

  spool.Remove(out.id);
  EXPECT_EQ(spool.Count(), 0u);
  EXPECT_FALSE(spool.PeekOldest(&out));
}

TEST_F(SpoolTest, ReplaysOldestFirst) {
  // Unity's cache replays newest-first, which reorders a recovered session.
  // Ours must not.
  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(spool.Write(MakeChunk(Stream::kEvents, std::to_string(i))));
  }
  for (int i = 0; i < 5; ++i) {
    Chunk out;
    ASSERT_TRUE(spool.PeekOldest(&out));
    EXPECT_EQ(out.body, std::to_string(i)) << "FIFO order violated at " << i;
    spool.Remove(out.id);
  }
}

TEST_F(SpoolTest, SurvivesProcessRestart) {
  {
    DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
    for (int i = 0; i < 3; ++i) spool.Write(MakeChunk(Stream::kEvents, std::to_string(i)));
  }
  // New instance, same directory: this is the reboot path.
  DiskSpool reopened(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  EXPECT_EQ(reopened.Count(), 3u);
  Chunk out;
  ASSERT_TRUE(reopened.PeekOldest(&out));
  EXPECT_EQ(out.body, "0") << "ordering must survive a restart";

  // And new writes must sort after the recovered ones, not collide with them.
  reopened.Write(MakeChunk(Stream::kEvents, "3"));
  EXPECT_EQ(reopened.Count(), 4u);
}

TEST_F(SpoolTest, EnforcesTheCapAndShedsLowPriorityFirst) {
  const std::string big(4096, 'x');
  DiskSpool spool(dir_.string(), 40 * 1024, std::make_shared<NullLogger>());

  for (int i = 0; i < 4; ++i) spool.Write(MakeChunk(Stream::kEvents, big));
  const std::size_t events_before = spool.Count();

  // Push well past the cap with gaze, which is lower priority than events.
  for (int i = 0; i < 40; ++i) spool.Write(MakeChunk(Stream::kGaze, big));

  EXPECT_LE(spool.TotalBytes(), 40u * 1024u) << "cap must be enforced, not hoped for";
  EXPECT_GT(spool.evicted(), 0u);

  // Count the surviving events. Eviction should have preferred gaze.
  std::size_t events_after = 0;
  Chunk out;
  while (spool.PeekOldest(&out)) {
    if (out.stream == Stream::kEvents) ++events_after;
    spool.Remove(out.id);
  }
  EXPECT_EQ(events_after, events_before)
      << "events must outlive gaze when the spool sheds load";
}

// A gaze part carrying session properties is evicted with the events, not with the poses: it is
// what makes the platform keep the session at all. The tier is in the chunk's id, so it holds
// across a restart without a file being opened.
TEST_F(SpoolTest, AGazePartCarryingPropertiesIsEvictedWithTheEvents) {
  const std::string big(2048, 'x');
  Chunk properties = MakeChunk(Stream::kGaze, R"({"properties":{"c3d.app.version":"1"}})");
  properties.carries_properties = true;
  {
    DiskSpool spool(dir_.string(), 24 * 1024, std::make_shared<NullLogger>());
    ASSERT_TRUE(spool.Write(properties));
    for (int i = 0; i < 40; ++i) ASSERT_TRUE(spool.Write(MakeChunk(Stream::kGaze, big)));
    ASSERT_GT(spool.evicted(), 0u);
    Chunk oldest;
    ASSERT_TRUE(spool.PeekOldest(&oldest));
    EXPECT_TRUE(oldest.carries_properties) << "the property part was evicted with the poses";
    EXPECT_EQ(oldest.stream, Stream::kGaze);
  }
  DiskSpool reopened(dir_.string(), 24 * 1024, std::make_shared<NullLogger>());
  for (int i = 0; i < 40; ++i) ASSERT_TRUE(reopened.Write(MakeChunk(Stream::kGaze, big)));
  ASSERT_GT(reopened.evicted(), 0u);
  Chunk oldest;
  ASSERT_TRUE(reopened.PeekOldest(&oldest));
  EXPECT_TRUE(oldest.carries_properties) << "the restart forgot the tier: " << oldest.id;
  EXPECT_EQ(oldest.stream, Stream::kGaze) << oldest.id;
  EXPECT_EQ(oldest.body, properties.body);
}

// A spool holding nothing but events refuses a plain gaze part, which may evict only gaze and
// below. A part carrying properties is not refused: it evicts the oldest event, as an event would.
TEST_F(SpoolTest, APropertyPartIsNotRefusedWhereOnlyEventsAreLeft) {
  DiskSpool spool(dir_.string(), 8 * 1024, std::make_shared<NullLogger>());
  while (spool.evicted() == 0) {
    ASSERT_TRUE(spool.Write(MakeChunk(Stream::kEvents, std::string(1000, 'e'))));
  }
  const std::uint64_t evicted = spool.evicted();
  EXPECT_FALSE(spool.Write(MakeChunk(Stream::kGaze, std::string(1000, 'g'))));
  EXPECT_EQ(spool.evicted(), evicted);
  Chunk properties = MakeChunk(Stream::kGaze, std::string(1000, 'p'));
  properties.carries_properties = true;
  EXPECT_TRUE(spool.Write(properties));
  EXPECT_EQ(spool.evicted(), evicted + 1);
}

// A chunk that cannot be read is discarded, and the one behind it is offered by the same call.
TEST_F(SpoolTest, CorruptChunkIsDiscardedRatherThanWedgingTheQueue) {
  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  spool.Write(MakeChunk(Stream::kEvents, "good"));

  // Corrupt the chunk on disk behind the spool's back.
  for (const auto& e : fs::directory_iterator(dir_ / "chunks")) {
    std::ofstream out(e.path(), std::ios::trunc);
    out << "{ this is not json";
  }
  spool.Write(MakeChunk(Stream::kEvents, "behind"));

  Chunk out;
  ASSERT_TRUE(spool.PeekOldest(&out));
  EXPECT_EQ(out.body, "behind");
  EXPECT_EQ(spool.Count(), 1u) << "a poisoned chunk must not block the queue forever";
  spool.Remove(out.id);
  EXPECT_FALSE(spool.PeekOldest(&out));
}

TEST_F(SpoolTest, PartialWritesAreNeverVisible) {
  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  spool.Write(MakeChunk(Stream::kEvents, "body"));
  // Chunks are renamed into place, so no .tmp file should ever be indexed.
  for (const auto& e : fs::directory_iterator(dir_ / "chunks")) {
    EXPECT_NE(e.path().extension(), ".tmp");
  }
}

TEST_F(SpoolTest, MarkersTrackOpenSessions) {
  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  SessionMarker m;
  m.session_id = "100_robot";
  m.scene_id = "SCENE";
  m.scene_version = "2";
  m.device_id = "robot";
  m.started_at = 100.0;
  m.last_seen = 160.0;
  spool.WriteMarker(m);

  auto open = spool.OpenMarkers();
  ASSERT_EQ(open.size(), 1u);
  EXPECT_EQ(open[0].session_id, "100_robot");
  EXPECT_DOUBLE_EQ(open[0].last_seen, 160.0);

  spool.CloseMarker("100_robot");
  EXPECT_TRUE(spool.OpenMarkers().empty());
}

TEST_F(SpoolTest, ABodyContainingEveryByteValueSurvivesTheRoundTrip) {
  // A packed Cap'n Proto payload is arbitrary bytes, and the chunk envelope is
  // JSON. Putting one in a JSON string field does not mangle it, it THROWS -- so
  // this is the test that the binary streams can be spooled at all.
  std::string body;
  for (int i = 0; i < 256; ++i) body.push_back(static_cast<char>(i));

  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kCamera, body)));

  Chunk out;
  ASSERT_TRUE(spool.PeekOldest(&out));
  EXPECT_EQ(out.body, body);
  EXPECT_EQ(out.body.size(), 256u);
  EXPECT_EQ(out.stream, Stream::kCamera);
}

TEST_F(SpoolTest, ABinaryChunkSurvivesAProcessRestart) {
  std::string body;
  for (int i = 0; i < 256; ++i) body.push_back(static_cast<char>(255 - i));
  {
    DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
    ASSERT_TRUE(spool.Write(MakeChunk(Stream::kLidar, body)));
    ASSERT_TRUE(spool.Write(MakeChunk(Stream::kPose, body)));
  }
  DiskSpool reopened(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  EXPECT_EQ(reopened.Count(), 2u);
  // The stream is read back from the id suffix, which recovery and eviction both
  // key on, so a new stream name that Rescan does not know reads as `events`.
  for (Stream s : {Stream::kLidar, Stream::kPose}) {
    Chunk out;
    ASSERT_TRUE(reopened.PeekOldest(&out));
    EXPECT_EQ(out.body, body);
    EXPECT_EQ(out.stream, s);
    reopened.Remove(out.id);
  }
}

TEST_F(SpoolTest, TextBodiesStayReadableInTheChunkFile) {
  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kEvents, R"({"name":"c3d.sessionStart"})")));
  for (const auto& e : fs::directory_iterator(dir_ / "chunks")) {
    std::ifstream in(e.path(), std::ios::binary);
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("c3d.sessionStart"), std::string::npos)
        << "a JSON body must not be base64'd: a spooled chunk is diagnosed with `cat`";
    EXPECT_EQ(content.find("body_b64"), std::string::npos);
  }
}

TEST_F(SpoolTest, CameraChunksAreShedBeforeThePoseTimeline) {
  // The spool is sized so one camera chunk plus one gaze chunk does not fit.
  DiskSpool spool(dir_.string(), 6 * 1024, std::make_shared<NullLogger>());
  std::string picture(4 * 1024, '\x80');  // not valid UTF-8, so it takes the base64 path
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kCamera, picture)));
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kGaze, std::string(512, 'g'))));
  EXPECT_GE(spool.evicted(), 1u);

  Chunk out;
  ASSERT_TRUE(spool.PeekOldest(&out));
  EXPECT_EQ(out.stream, Stream::kGaze)
      << "a camera part must be shed before the spatial timeline it indexes";
}

namespace {
class LinesLogger : public ILogger {
 public:
  void Log(LogLevel, const std::string& message) override {
    std::lock_guard<std::mutex> lock(mu_);
    lines_.push_back(message);
  }
  std::vector<std::string> lines() const {
    std::lock_guard<std::mutex> lock(mu_);
    return lines_;
  }

 private:
  mutable std::mutex mu_;
  std::vector<std::string> lines_;
};
}  // namespace

TEST_F(SpoolTest, AnEvictionIsSaidInTheLogBecauseThoseChunksWillNeverBeSent) {
  auto log = std::make_shared<LinesLogger>();
  DiskSpool spool(dir_.string(), 6 * 1024, log);
  const std::string picture(4 * 1024, 'p');
  for (int i = 0; i < 3; ++i) ASSERT_TRUE(spool.Write(MakeChunk(Stream::kCamera, picture)));
  ASSERT_GE(spool.evicted(), 1u);
  std::size_t said = 0;
  for (const auto& line : log->lines()) {
    if (line.find("evicted") != std::string::npos && line.find("camera") != std::string::npos) {
      ++said;
    }
  }
  EXPECT_EQ(said, 1u) << "the first eviction must be logged, and once, not per write";
}

// A spool directory can hold a chunk another build wrote, of a stream this one does not send: such
// a chunk is discarded, said in the log, and never sent -- read back as the events chunk its
// unknown stream name would map to, it would be. The chunk behind it is offered by the same call,
// since a caller reads no chunk as nothing to send.
TEST_F(SpoolTest, AChunkOfAStreamThisBuildDoesNotSendIsDiscardedUnsentAndSaid) {
  {
    DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
    ASSERT_TRUE(spool.Write(MakeChunk(Stream::kGaze, R"({"data":[]})")));
  }
  // Older than the gaze chunk, so it is read first.
  {
    std::ofstream foreign(dir_ / "chunks" / "000000000000-foreignStream.c3dchunk");
    foreign << R"({"schema":4,"url":"https://example.invalid/v0/foreignStream/S?version=1",)"
               R"("scene_id":"S","scene_version":"1","key_fingerprint":"","session_id":"sess",)"
               R"("stream":"foreignStream","created_at":1.0,"body":"{\"foreignRecord\":[]}"})";
  }
  auto log = std::make_shared<LinesLogger>();
  DiskSpool reopened(dir_.string(), 10 * 1024 * 1024, log);
  ASSERT_EQ(reopened.Count(), 2u);
  Chunk out;
  ASSERT_TRUE(reopened.PeekOldest(&out)) << "the chunk behind the foreign one was not offered";
  EXPECT_EQ(out.stream, Stream::kGaze) << "the foreign chunk was read back as " << out.id;
  EXPECT_EQ(reopened.Count(), 1u);
  EXPECT_FALSE(fs::exists(dir_ / "chunks" / "000000000000-foreignStream.c3dchunk"));
  std::size_t said = 0;
  for (const auto& line : log->lines()) {
    if (line.find("foreignStream") != std::string::npos &&
        line.find("discarding") != std::string::npos) {
      ++said;
    }
  }
  EXPECT_EQ(said, 1u);
}

// ---------------------------------------------------------------------------
// Private, locked, and honest about failure
// ---------------------------------------------------------------------------

namespace {
// The process umask for one scope, restored however the scope ends, so a failed assertion does not
// leave the tests after it running under another.
class UmaskGuard {
 public:
  explicit UmaskGuard(mode_t mask) : previous_(::umask(mask)) {}
  ~UmaskGuard() { ::umask(previous_); }
  UmaskGuard(const UmaskGuard&) = delete;
  UmaskGuard& operator=(const UmaskGuard&) = delete;

 private:
  mode_t previous_;
};

unsigned Mode(const fs::path& p) {
  struct stat st {};
  ::stat(p.c_str(), &st);
  return st.st_mode & 0777;
}

std::uintmax_t BytesOnDisk(const fs::path& chunks) {
  std::uintmax_t total = 0;
  for (const auto& e : fs::directory_iterator(chunks)) total += e.file_size();
  return total;
}

std::size_t FilesEndingIn(const fs::path& dir, const std::string& suffix) {
  std::size_t n = 0;
  for (const auto& e : fs::directory_iterator(dir)) {
    const std::string name = e.path().filename().string();
    if (name.size() >= suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
      ++n;
    }
  }
  return n;
}
}  // namespace

// Chunks hold poses, events and camera frames. Under the default umask anything created without
// an explicit mode is readable by every account on the robot.
TEST_F(SpoolTest, EverythingTheSpoolCreatesIsOwnerOnly) {
  const UmaskGuard guard(022);
  const fs::path dir = dir_ / "fresh";
  {
    DiskSpool spool(dir.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
    ASSERT_TRUE(spool.Write(MakeChunk(Stream::kGaze, R"({"pose":1})")));
    SessionMarker m;
    m.session_id = "1_robot";
    spool.WriteMarker(m);
    EXPECT_EQ(Mode(dir), 0700u);
    EXPECT_EQ(Mode(dir / "chunks"), 0700u);
    EXPECT_EQ(Mode(dir / "sessions"), 0700u);
    EXPECT_EQ(Mode(dir / ".lock"), 0600u);
    EXPECT_EQ(Mode(dir / "sessions" / "1_robot.c3dsession"), 0600u);
    for (const auto& e : fs::directory_iterator(dir / "chunks")) EXPECT_EQ(Mode(e.path()), 0600u);
  }
  // A spool_dir its owner made group-readable keeps that; the spool's own directories do not.
  const fs::path chosen = dir_ / "chosen";
  fs::create_directories(chosen / "chunks");
  ::chmod(chosen.c_str(), 0750);
  ::chmod((chosen / "chunks").c_str(), 0755);
  DiskSpool spool(chosen.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  EXPECT_EQ(Mode(chosen), 0750u);
  EXPECT_EQ(Mode(chosen / "chunks"), 0700u);
}

// The default spool_dir is $HOME/.local/state/cognitive3d, and on a fresh account only $HOME
// exists. Every directory the spool has to create on the way is as private as the spool; the
// directories that were already there are their owner's.
TEST_F(SpoolTest, EveryParentTheSpoolCreatesIsOwnerOnlyAndNoExistingOneIsChanged) {
  const UmaskGuard guard(022);
  const fs::path home = dir_ / "home";
  fs::create_directories(home);
  ::chmod(home.c_str(), 0751);
  const fs::path spool_dir = home / ".local" / "state" / "cognitive3d";
  {
    DiskSpool spool(spool_dir.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
    ASSERT_TRUE(spool.error().empty()) << spool.error();
  }
  EXPECT_EQ(Mode(home), 0751u);
  EXPECT_EQ(Mode(home / ".local"), 0700u);
  EXPECT_EQ(Mode(home / ".local" / "state"), 0700u);
  EXPECT_EQ(Mode(spool_dir), 0700u);
}

// doctor creates spool_dir before any spool exists, and the node then keeps the directory it
// finds, so the two must create it alike. Under umask 0 a directory made without a mode is 0777.
TEST_F(SpoolTest, MakeSpoolDirectoryCreatesWhatTheSpoolWouldAndLeavesAnExistingOneAlone) {
  {
    const UmaskGuard guard(0);
    const fs::path made = dir_ / "made" / "spool";
    EXPECT_TRUE(MakeSpoolDirectory(made.string()));
    EXPECT_EQ(Mode(dir_ / "made"), 0700u);
    EXPECT_EQ(Mode(made), 0700u);

    const fs::path chosen = dir_ / "chosen";
    fs::create_directories(chosen);
    ::chmod(chosen.c_str(), 0755);
    EXPECT_TRUE(MakeSpoolDirectory(chosen.string()));
    EXPECT_EQ(Mode(chosen), 0755u);
  }

  std::ofstream(dir_ / "a_file") << "x";
  EXPECT_FALSE(MakeSpoolDirectory((dir_ / "a_file" / "spool").string()));
  EXPECT_FALSE(MakeSpoolDirectory((dir_ / "a_file").string()));
  EXPECT_FALSE(MakeSpoolDirectory(""));
}

// A umask can strip the owner's own bits, and mkdir alone then leaves each directory 0500: one the
// spool cannot write in, under which a non-root process cannot create the next directory of the
// path. Root is not stopped by 0500, and the tests may run as root, so the modes are asserted.
TEST_F(SpoolTest, MakeSpoolDirectoryIsOwnerOnlyUnderAUmaskThatStripsTheOwnersBits) {
  const UmaskGuard guard(0277);
  const fs::path made = dir_ / "stripped" / "spool";
  EXPECT_TRUE(MakeSpoolDirectory(made.string()));
  EXPECT_EQ(Mode(dir_ / "stripped"), 0700u);
  EXPECT_EQ(Mode(made), 0700u);
}

// Two spools on one directory issue the same chunk ids from two counters, and each rename
// replaces the other's file: half of both nodes' data, gone under a dead uplink.
TEST_F(SpoolTest, ASecondSpoolOnOneDirectoryIsRefusedAndSaysWhy) {
  auto first = std::make_shared<DiskSpool>(dir_.string(), 10 * 1024 * 1024,
                                           std::make_shared<NullLogger>());
  ASSERT_TRUE(first->error().empty()) << first->error();
  DiskSpool second(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  EXPECT_NE(second.error().find(dir_.string()), std::string::npos) << second.error();
  EXPECT_NE(second.error().find("already in use by process " + std::to_string(::getpid())),
            std::string::npos)
      << second.error();
  EXPECT_FALSE(second.Write(MakeChunk(Stream::kEvents, "from the second")));
  ASSERT_TRUE(first->Write(MakeChunk(Stream::kEvents, "from the first")));
  Chunk out;
  EXPECT_FALSE(second.PeekOldest(&out)) << "an unusable spool must not hand out another's chunks";
  ASSERT_TRUE(first->PeekOldest(&out));
  EXPECT_EQ(out.body, "from the first");
  EXPECT_EQ(FilesEndingIn(dir_ / "chunks", ".c3dchunk"), 1u);

  first.reset();
  DiskSpool third(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  EXPECT_TRUE(third.error().empty()) << "the lock must go with the spool that held it";
  EXPECT_EQ(third.Count(), 1u);
}

TEST_F(SpoolTest, AnotherProcessHoldsTheDirectoryUntilItDies) {
  int ready[2], release[2];
  ASSERT_EQ(::pipe(ready), 0);
  ASSERT_EQ(::pipe(release), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    DiskSpool held(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
    (void)!::write(ready[1], "x", 1);
    char c;
    (void)!::read(release[0], &c, 1);
    ::_exit(0);  // dies holding it: no destructor releases the lock, the kernel does
  }
  char c;
  ASSERT_EQ(::read(ready[0], &c, 1), 1);
  {
    DiskSpool refused(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
    EXPECT_NE(refused.error().find("by process " + std::to_string(child)), std::string::npos)
        << refused.error();
  }
  (void)!::write(release[1], "x", 1);
  int status = 0;
  ::waitpid(child, &status, 0);
  DiskSpool after(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  EXPECT_TRUE(after.error().empty()) << after.error();
}

// A write cut short -- a full disk, a quota, a file-size limit -- must not leave its partial
// .tmp behind, uncounted against the cap and never removed.
TEST_F(SpoolTest, AWriteThatFailsPartWayLeavesNothingBehindAndIsCounted) {
  DiskSpool spool(dir_.string(), 512ull << 20, std::make_shared<NullLogger>());
  struct rlimit old_limit {};
  ::getrlimit(RLIMIT_FSIZE, &old_limit);
  auto previous = std::signal(SIGXFSZ, SIG_IGN);
  struct rlimit small = old_limit;
  small.rlim_cur = 64 * 1024;
  ::setrlimit(RLIMIT_FSIZE, &small);
  const bool written = spool.Write(MakeChunk(Stream::kEvents, std::string(300 * 1024, 'a')));
  ::setrlimit(RLIMIT_FSIZE, &old_limit);
  std::signal(SIGXFSZ, previous);

  EXPECT_FALSE(written);
  EXPECT_EQ(spool.write_failures(), 1u);
  EXPECT_EQ(FilesEndingIn(dir_ / "chunks", ".tmp"), 0u) << "the partial file stayed behind";
  EXPECT_EQ(spool.Count(), 0u);
  EXPECT_EQ(spool.TotalBytes(), 0u);
  EXPECT_TRUE(spool.Write(MakeChunk(Stream::kEvents, "after"))) << "the spool must still work";
}

TEST_F(SpoolTest, ATmpLeftByAProcessThatDiedMidWriteIsRemovedAtStart) {
  fs::create_directories(dir_ / "chunks");
  fs::create_directories(dir_ / "sessions");
  std::ofstream(dir_ / "chunks" / "000000000007-events.c3dchunk.tmp") << std::string(4096, 'x');
  std::ofstream(dir_ / "sessions" / "1_robot.c3dsession.tmp") << "{";
  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  EXPECT_EQ(FilesEndingIn(dir_ / "chunks", ".tmp"), 0u);
  EXPECT_EQ(FilesEndingIn(dir_ / "sessions", ".tmp"), 0u);
  EXPECT_EQ(spool.Count(), 0u);
}

// A binary body is stored base64'd at four thirds of its size, so a cap checked against the body
// alone lets a spool of camera parts sit above spool_max_bytes by a third of a chunk.
TEST_F(SpoolTest, TheCapHoldsToTheBytesOnDiskForBinaryChunksToo) {
  const std::uint64_t cap = 16 * 1024;
  DiskSpool spool(dir_.string(), cap, std::make_shared<NullLogger>());
  const std::string picture(3 * 1024, '\x80');  // not UTF-8, so stored as base64
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(spool.Write(MakeChunk(Stream::kCamera, picture)));
    const std::uintmax_t on_disk = BytesOnDisk(dir_ / "chunks");
    EXPECT_LE(on_disk, cap) << "after write " << i;
    EXPECT_EQ(spool.TotalBytes(), on_disk) << "the count must be the files' real size";
  }
  EXPECT_GT(spool.evicted(), 0u);
}

TEST_F(SpoolTest, AnOrderlyCloseRemovesTheMarker) {
  DiskSpool spool(dir_.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  for (int i = 0; i < 3; ++i) {
    SessionMarker m;
    m.session_id = std::to_string(100 + i) + "_robot";
    spool.WriteMarker(m);
    spool.CloseMarker(m.session_id);
  }
  EXPECT_EQ(FilesEndingIn(dir_ / "sessions", ".c3dsession"), 0u)
      << "ended markers accumulated: one file per session, for ever";
}

// An operator clearing a backlog with `rm -rf`, or a cleaner, must not leave the spool dropping
// every chunk for the rest of the process's life.
TEST_F(SpoolTest, AVanishedSpoolDirectoryIsRecreatedAndRelocked) {
  const fs::path dir = dir_ / "spool";
  DiskSpool spool(dir.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kEvents, "before")));
  fs::remove_all(dir);

  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kEvents, "after"))) << "the chunk was dropped";
  EXPECT_EQ(spool.Count(), 1u) << "the chunk that went with the directory is forgotten";
  Chunk out;
  ASSERT_TRUE(spool.PeekOldest(&out));
  EXPECT_EQ(out.body, "after");
  EXPECT_EQ(Mode(dir), 0700u);
  DiskSpool second(dir.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  EXPECT_FALSE(second.error().empty()) << "the recreated directory must be locked again";

  // Only the chunks directory, with the lock still in place.
  fs::remove_all(dir / "chunks");
  EXPECT_TRUE(spool.Write(MakeChunk(Stream::kEvents, "after the chunks went")));
  EXPECT_EQ(spool.Count(), 1u);

  SessionMarker m;
  m.session_id = "3_robot";
  fs::remove_all(dir / "sessions");
  spool.WriteMarker(m);
  EXPECT_EQ(spool.OpenMarkers().size(), 1u) << "a marker write must recreate its directory too";
}

TEST_F(SpoolTest, ADirectoryAnotherSpoolRecreatedFirstIsLeftToIt) {
  const fs::path dir = dir_ / "spool";
  DiskSpool first(dir.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  fs::remove_all(dir);
  DiskSpool second(dir.string(), 10 * 1024 * 1024, std::make_shared<NullLogger>());
  ASSERT_TRUE(second.error().empty()) << second.error();

  EXPECT_FALSE(first.Write(MakeChunk(Stream::kEvents, "first")));
  EXPECT_NE(first.error().find("recreated by another process"), std::string::npos)
      << first.error();
  EXPECT_TRUE(second.Write(MakeChunk(Stream::kEvents, "second")));
  EXPECT_EQ(FilesEndingIn(dir / "chunks", ".c3dchunk"), 1u);
}

// A sessions/ that cannot be stat'd -- here a symlink to itself; a failing card or a stale NFS
// handle leaves it the same -- costs the marker write and never throws: every marker call runs
// on the worker or the heartbeat, where an escaped exception ends the process.
TEST_F(SpoolTest, AnUnstatableSessionsDirectoryCostsTheWriteNotTheProcess) {
  {
    DiskSpool spool(dir_.string(), 1 << 20, std::make_shared<NullLogger>());
    fs::remove_all(dir_ / "sessions");
    fs::create_symlink(dir_ / "sessions", dir_ / "sessions");
    SessionMarker m;
    m.session_id = "1_robot";
    EXPECT_NO_THROW(spool.WriteMarker(m));
    std::vector<SessionMarker> open;
    EXPECT_NO_THROW(open = spool.OpenMarkers());
    EXPECT_TRUE(open.empty());
    EXPECT_NO_THROW(spool.CloseMarker("1_robot"));
  }
  // An entry that cannot be stat'd is passed over, at startup and when markers are read, and
  // the markers beside it are still read.
  fs::remove(dir_ / "sessions");
  fs::create_directories(dir_ / "sessions");
  fs::create_symlink(dir_ / "sessions" / "loop.c3dsession", dir_ / "sessions" / "loop.c3dsession");
  fs::create_symlink(dir_ / "chunks" / "loop.c3dchunk", dir_ / "chunks" / "loop.c3dchunk");
  std::unique_ptr<DiskSpool> spool;
  ASSERT_NO_THROW(spool = std::make_unique<DiskSpool>(dir_.string(), 1 << 20,
                                                      std::make_shared<NullLogger>()));
  SessionMarker m;
  m.session_id = "2_robot";
  m.started_at = 1.0;
  spool->WriteMarker(m);
  std::vector<SessionMarker> open;
  ASSERT_NO_THROW(open = spool->OpenMarkers());
  ASSERT_EQ(open.size(), 1u);
  EXPECT_EQ(open[0].session_id, "2_robot");
}

namespace {

std::string ReadWhole(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void WriteWhole(const fs::path& p, const std::string& text) {
  std::ofstream(p, std::ios::binary) << text;
}

}  // namespace

// A spool_dir another account can write to may hold a link planted where the lock goes. Opening
// it would truncate what it names and write a pid into it -- an authorized_keys, say. The spool
// refuses instead, naming the path.
TEST_F(SpoolTest, ALockPlantedAsALinkIsRefusedAndWhatItNamesIsUntouched) {
  const fs::path victim = dir_.string() + "_victim";
  for (const bool hard : {false, true}) {
    SCOPED_TRACE(hard ? "a hard link" : "a symbolic link");
    WriteWhole(victim, "precious contents\n");
    fs::remove(dir_ / ".lock");
    if (hard) {
      fs::create_hard_link(victim, dir_ / ".lock");
    } else {
      fs::create_symlink(victim, dir_ / ".lock");
    }
    DiskSpool spool(dir_.string(), 1 << 20, std::make_shared<NullLogger>());
    EXPECT_NE(spool.error().find((dir_ / ".lock").string()), std::string::npos) << spool.error();
    EXPECT_NE(spool.error().find(hard ? "hard link" : "symbolic link"), std::string::npos)
        << spool.error();
    EXPECT_FALSE(spool.Write(MakeChunk(Stream::kEvents, "{}")));
    EXPECT_EQ(ReadWhole(victim), "precious contents\n");
  }
  fs::remove(victim);
}

// A chunk or marker is written to a temporary name first. A link planted at that name must be
// replaced, never written through.
TEST_F(SpoolTest, ATemporaryFilePlantedAsALinkIsReplacedNotWrittenThrough) {
  const fs::path victim = dir_.string() + "_victim";
  WriteWhole(victim, "precious contents\n");
  DiskSpool spool(dir_.string(), 1 << 20, std::make_shared<NullLogger>());
  // The first chunk of an empty spool, and the marker of a session named below.
  fs::create_symlink(victim, dir_ / "chunks" / "000000000000-events.c3dchunk.tmp");
  fs::create_symlink(victim, dir_ / "sessions" / "7_robot.c3dsession.tmp");

  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kEvents, "{\"data\":[]}")));
  Chunk read;
  ASSERT_TRUE(spool.PeekOldest(&read));
  EXPECT_EQ(read.id, "000000000000-events") << "the planted name was not the one written";
  EXPECT_EQ(read.body, "{\"data\":[]}");
  SessionMarker m;
  m.session_id = "7_robot";
  spool.WriteMarker(m);
  const auto open = spool.OpenMarkers();
  ASSERT_EQ(open.size(), 1u);
  EXPECT_EQ(open[0].session_id, "7_robot");
  EXPECT_EQ(ReadWhole(victim), "precious contents\n");
  fs::remove(victim);
}

// chunks/ and sessions/ are tightened to 0700 on every start, but never through a symbolic
// link: its target can be anything the node's user may chmod, which for root is anything.
TEST_F(SpoolTest, ALinkedSubdirectoryIsNeverTightenedThrough) {
  const fs::path target = dir_.string() + "_elsewhere";
  fs::create_directories(target);
  fs::permissions(target, fs::perms::owner_all | fs::perms::group_read | fs::perms::group_exec |
                              fs::perms::others_read | fs::perms::others_exec);
  fs::create_symlink(target, dir_ / "chunks");
  { DiskSpool spool(dir_.string(), 1 << 20, std::make_shared<NullLogger>()); }
  EXPECT_EQ(Mode(target), 0755u) << "the link's target was chmodded";
  fs::remove(dir_ / "chunks");
  fs::remove_all(target);
}

// A chunk that cannot fit even once everything it may displace is gone is refused without
// evicting anything. Evicting and then refusing it anyway would lose the evicted chunks for nothing.
TEST_F(SpoolTest, AChunkThatCannotFitEvictsNothing) {
  DiskSpool spool(dir_.string(), 4096, std::make_shared<NullLogger>());
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kCamera, std::string(500, 'c'))));
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kCamera, std::string(500, 'c'))));
  ASSERT_TRUE(spool.Write(MakeChunk(Stream::kEvents, std::string(1500, 'e'))));
  const std::size_t count = spool.Count();

  // A gaze chunk may displace the camera chunks but never the events one, and this one would
  // not fit even then. Nor does one larger than the whole cap.
  EXPECT_FALSE(spool.Write(MakeChunk(Stream::kGaze, std::string(3500, 'g'))));
  EXPECT_FALSE(spool.Write(MakeChunk(Stream::kEvents, std::string(5000, 'e'))));
  EXPECT_EQ(spool.evicted(), 0u);
  EXPECT_EQ(spool.Count(), count);

  // One that fits once the camera chunks are gone still displaces them.
  EXPECT_TRUE(spool.Write(MakeChunk(Stream::kGaze, std::string(1800, 'g'))));
  EXPECT_EQ(spool.evicted(), 2u);
  EXPECT_EQ(spool.Count(), 2u);
}
