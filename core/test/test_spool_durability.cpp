// What reaches the device, and in what order, when a chunk is spooled or a marker is written or
// removed. A power cut can leave a renamed-but-empty file behind unless the data is flushed
// before the rename and the directory after it -- and the spool then discards that chunk as
// unparseable -- and can bring back an unlinked file whose directory was never flushed. Nothing
// short of a power cut shows this behaviourally, so this executable replaces fsync, fdatasync,
// rename and unlink with wrappers that record each call and then make the real one, and the
// test reads the record.

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "c3d/spool.hpp"

namespace fs = std::filesystem;

namespace {

std::mutex g_mu;
bool g_recording = false;
std::vector<std::string> g_calls;

std::string PathOf(int fd) {
  char buf[4096];
  const ssize_t n =
      ::readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), buf, sizeof(buf) - 1);
  return n > 0 ? std::string(buf, static_cast<std::size_t>(n)) : std::string("?");
}

void Record(const std::string& call) {
  std::lock_guard<std::mutex> lock(g_mu);
  if (g_recording) g_calls.push_back(call);
}

template <typename F>
F Real(const char* name) {
  return reinterpret_cast<F>(::dlsym(RTLD_NEXT, name));
}

}  // namespace

extern "C" int fsync(int fd) {
  static const auto real = Real<int (*)(int)>("fsync");
  Record("fsync " + PathOf(fd));
  return real(fd);
}

extern "C" int fdatasync(int fd) {
  static const auto real = Real<int (*)(int)>("fdatasync");
  Record("fdatasync " + PathOf(fd));
  return real(fd);
}

extern "C" int rename(const char* from, const char* to) noexcept {
  static const auto real = Real<int (*)(const char*, const char*)>("rename");
  Record(std::string("rename ") + from + " " + to);
  return real(from, to);
}

extern "C" int unlink(const char* path) noexcept {
  static const auto real = Real<int (*)(const char*)>("unlink");
  Record(std::string("unlink ") + path);
  return real(path);
}

namespace {

std::vector<std::string> RecordedWhile(const std::function<void()>& action) {
  {
    std::lock_guard<std::mutex> lock(g_mu);
    g_calls.clear();
    g_recording = true;
  }
  action();
  std::lock_guard<std::mutex> lock(g_mu);
  g_recording = false;
  return g_calls;
}

std::ptrdiff_t IndexOf(const std::vector<std::string>& calls, const std::string& call) {
  const auto it = std::find(calls.begin(), calls.end(), call);
  return it == calls.end() ? -1 : it - calls.begin();
}

}  // namespace

TEST(SpoolDurability, AChunkReachesTheDeviceBeforeItIsRenamedAndTheDirectoryAfter) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_durable_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  c3d::DiskSpool spool(dir.string(), 10 * 1024 * 1024, std::make_shared<c3d::NullLogger>());
  c3d::Chunk chunk;
  chunk.url = "https://example.invalid/v0/events/S?version=1";
  chunk.body = R"({"data":[]})";
  chunk.stream = c3d::Stream::kEvents;
  const auto calls = RecordedWhile([&] { ASSERT_TRUE(spool.Write(chunk)); });

  c3d::Chunk written;
  ASSERT_TRUE(spool.PeekOldest(&written));
  const std::string chunks = fs::canonical(dir / "chunks").string();
  const std::string final_path = chunks + "/" + written.id + ".c3dchunk";
  const std::string tmp_path = final_path + ".tmp";

  const auto data = IndexOf(calls, "fdatasync " + tmp_path);
  const auto renamed = IndexOf(calls, "rename " + (dir / "chunks").string() + "/" + written.id +
                                          ".c3dchunk.tmp " + (dir / "chunks").string() + "/" +
                                          written.id + ".c3dchunk");
  const auto directory = IndexOf(calls, "fsync " + chunks);
  std::string seen;
  for (const auto& c : calls) seen += "\n  " + c;
  EXPECT_GE(data, 0) << "the chunk's data was never flushed:" << seen;
  EXPECT_GE(renamed, 0) << "the chunk was never renamed into place:" << seen;
  EXPECT_GE(directory, 0) << "the chunks directory was never flushed:" << seen;
  EXPECT_LT(data, renamed) << "flushed after the rename made it visible:" << seen;
  EXPECT_LT(renamed, directory) << "the directory was flushed before the rename:" << seen;
  fs::remove_all(dir);
}

// A session's marker must be on the device before anything of the session is: durable chunks
// with no marker belong to a session no recovery would close. Its removal must be too, or a
// power cut brings back the marker of a session that ended in order. The heartbeat rewrites in
// between are not flushed.
TEST(SpoolDurability, AMarkerReachesTheDeviceWhenFirstWrittenAndWhenRemoved) {
  const fs::path dir =
      fs::temp_directory_path() / ("c3d_durable_marker_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  c3d::DiskSpool spool(dir.string(), 10 * 1024 * 1024, std::make_shared<c3d::NullLogger>());
  c3d::SessionMarker marker;
  marker.session_id = "1_robot";
  const std::string sessions = fs::canonical(dir / "sessions").string();
  const std::string named = (dir / "sessions" / "1_robot.c3dsession").string();
  auto seen = [](const std::vector<std::string>& calls) {
    std::string out;
    for (const auto& c : calls) out += "\n  " + c;
    return out;
  };

  const auto first = RecordedWhile([&] { spool.WriteMarker(marker); });
  const auto data = IndexOf(first, "fdatasync " + sessions + "/1_robot.c3dsession.tmp");
  const auto renamed = IndexOf(first, "rename " + named + ".tmp " + named);
  const auto directory = IndexOf(first, "fsync " + sessions);
  EXPECT_GE(data, 0) << "the first marker's data was never flushed:" << seen(first);
  EXPECT_LT(data, renamed) << seen(first);
  EXPECT_LT(renamed, directory) << "the sessions directory was not flushed after:" << seen(first);

  const auto beat = RecordedWhile([&] { spool.WriteMarker(marker); });
  for (const auto& call : beat) {
    EXPECT_NE(call.rfind("fsync", 0), 0u) << "a heartbeat rewrite was flushed:" << seen(beat);
    EXPECT_NE(call.rfind("fdatasync", 0), 0u) << "a heartbeat rewrite was flushed:" << seen(beat);
  }

  const auto closed = RecordedWhile([&] { spool.CloseMarker("1_robot"); });
  const auto removed = IndexOf(closed, "unlink " + named);
  const auto flushed = IndexOf(closed, "fsync " + sessions);
  EXPECT_GE(removed, 0) << seen(closed);
  EXPECT_LT(removed, flushed) << "the removal was not flushed after it:" << seen(closed);
  fs::remove_all(dir);
}
