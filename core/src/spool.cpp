#include "c3d/spool.hpp"

#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>

#include "json_dump.hpp"
#include "serialize.hpp"
#include "util.hpp"

namespace fs = std::filesystem;
using json = nlohmann::ordered_json;

namespace c3d {
namespace {

constexpr const char* kChunkExt = ".c3dchunk";
constexpr const char* kMarkerExt = ".c3dsession";
constexpr const char* kTmpExt = ".tmp";
constexpr const char* kLockName = ".lock";
// The envelope's format, recorded in every chunk so a spooled file says which format it is in when
// someone inspects it. Nothing in this SDK reads it back.
constexpr int kChunkSchema = 4;

Stream StreamFromName(const std::string& n) {
  if (n == "gaze") return Stream::kGaze;
  if (n == "sensors") return Stream::kSensors;
  if (n == "dynamics") return Stream::kDynamics;
  if (n == "camera") return Stream::kCamera;
  if (n == "lidar") return Stream::kLidar;
  if (n == "pose") return Stream::kPose;
  return Stream::kEvents;
}

// Whether a chunk's stream is one this build sends. A spool directory can hold a chunk another
// build wrote, of a stream this one does not have. StreamFromName would read that back as an
// events chunk, evicted and sent as one.
bool IsSentStream(const std::string& n) {
  for (Stream s : {Stream::kGaze, Stream::kEvents, Stream::kSensors, Stream::kDynamics,
                   Stream::kCamera, Stream::kLidar, Stream::kPose}) {
    if (n == StreamName(s)) return true;
  }
  return false;
}

// The id suffix of a chunk that carries session properties: `<seq>-<stream>-properties`.
// After the stream name, so a build that reads the stream from the id's last segment
// evicts it with the events, which is where it belongs.
constexpr const char* kPropertiesSuffix = "-properties";

// The tier a chunk is evicted at. See Chunk::carries_properties.
int EvictionTier(Stream stream, bool carries_properties) {
  return carries_properties ? StreamPriority(Stream::kEvents) : StreamPriority(stream);
}

std::string ErrnoText() { return std::strerror(errno); }

bool EndsWith(const std::string& text, const std::string& suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Data first, then the rename: a chunk's bytes must be on the device before the
// rename that makes it visible, or a power cut leaves a complete-looking empty file.
int SyncData(int fd) {
#if defined(__linux__)
  return ::fdatasync(fd);
#else
  return ::fsync(fd);
#endif
}

// The rename into a directory is durable only once the directory itself is.
bool SyncDirectory(const std::string& dir) {
  const int fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) return false;
  const bool ok = ::fsync(fd) == 0;
  ::close(fd);
  return ok;
}

// Owner-only whatever the umask left of it. A symbolic link is never tightened
// through: its target may be anything the node's user can chmod.
void Tighten(const fs::path& dir) {
  const int fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  if (fd < 0) return;
  (void)::fchmod(fd, S_IRWXU);
  ::close(fd);
}

// The spool's own subdirectories are owner-only even when they already existed.
void MakePrivateDirectory(const std::string& dir, bool always_tighten) {
  MakeSpoolDirectory(dir);
  if (always_tighten) Tighten(dir);
}

// Calls `visit` with each regular file in `dir`, never throwing: this runs on the worker
// and the heartbeat, where an escaped exception ends the process. A directory that cannot
// be read visits nothing, and an entry that cannot be stat'd -- a symlink loop, a failing
// card -- is passed over.
template <typename Visit>
void ForEachRegularFile(const std::string& dir, Visit visit) {
  std::error_code ec;
  for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
    std::error_code type_ec;
    if (it->is_regular_file(type_ec) && !type_ec) visit(it->path());
  }
}

}  // namespace

bool MakeSpoolDirectory(const std::string& dir) {
  if (dir.empty()) return false;
  fs::path built;
  for (const fs::path& part : fs::path(dir)) {
    built /= part;
    if (part.empty()) continue;
    // Created owner-only from the start, never widened for a moment and narrowed after.
    if (::mkdir(built.c_str(), S_IRWXU) == 0) {
      Tighten(built);
    } else if (errno != EEXIST) {
      return false;
    }
  }
  std::error_code ec;
  return fs::is_directory(dir, ec);
}

struct DiskSpool::Impl {
  // Lines to log once this spool's locks are released: the logger may be a consumer's that
  // calls back into a Session, whose stats() reads this spool under `mu`.
  using Said = std::vector<std::pair<LogLevel, std::string>>;

  std::string dir;
  std::string chunk_dir;
  std::string marker_dir;
  std::string lock_path;
  std::uint64_t max_bytes = 0;
  std::shared_ptr<ILogger> log;

  // Guards the index and everything below it. Never held across file I/O, so a slow
  // disk delays only the call doing the I/O and never a reader of the index.
  mutable std::mutex mu;
  std::uint64_t seq = 0;
  std::uint64_t total_bytes = 0;
  // Bytes of chunks being written right now, counted against the cap so two writers
  // cannot both fit under it with the same headroom.
  std::uint64_t reserved_bytes = 0;
  std::size_t count = 0;
  std::string error;
  std::atomic<int> lock_fd{-1};
  std::atomic<std::uint64_t> evicted{0};
  std::atomic<std::uint64_t> write_failures{0};

  // Serialises marker files, which nothing else touches. Kept apart from `mu` so a
  // heartbeat never waits behind a chunk being written or unlinked.
  mutable std::mutex marker_mu;
  // Markers whose first write reached the device. Guarded by marker_mu.
  std::set<std::string> durable_markers;

  // What eviction needs to know of a chunk, all of it read from the id.
  struct Entry {
    std::uint64_t size = 0;
    Stream stream = Stream::kEvents;
    bool carries_properties = false;
    int tier() const { return EvictionTier(stream, carries_properties); }
  };
  // Ordered by chunk id, which is zero-padded and monotonic, so map order is
  // FIFO by construction.
  std::map<std::string, Entry> index;

  std::string ChunkPath(const std::string& id) const { return chunk_dir + "/" + id + kChunkExt; }
  std::string MarkerPath(const std::string& session_id) const {
    return marker_dir + "/" + session_id + kMarkerExt;
  }

  // Takes the directory lock. "Held by another" makes the spool unusable, and so does a
  // lock that is not a plain file of its own: a spool_dir another account can write to
  // could hold a link planted at .lock, and the pid write below would truncate whatever
  // it names. A filesystem that cannot lock at all is said and tolerated, because
  // refusing it would refuse every robot whose spool lives on one. With `said`, what it
  // has to say is left there for a caller holding a lock to log after it.
  void Lock(Said* said = nullptr) {
    auto say = [this, said](LogLevel level, std::string message) {
      if (said) {
        said->emplace_back(level, std::move(message));
      } else {
        log->Log(level, message);
      }
    };
    const int fd = ::open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW,
                          S_IRUSR | S_IWUSR);
    if (fd < 0) {
      error = errno == ELOOP || errno == EMLINK
                  ? "the spool lock " + lock_path + " is a symbolic link, which this process "
                    "will not follow. Remove it, and give the node a spool_dir no other account "
                    "can write to"
                  : "cannot open the spool lock " + lock_path + ": " + ErrnoText();
      return;
    }
    struct stat st {};
    if (::fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_nlink != 1) {
      error = "the spool lock " + lock_path + " is not a plain file of its own (a hard link "
              "to another file, or not a regular file at all). Remove it, and give the node a "
              "spool_dir no other account can write to";
      ::close(fd);
      return;
    }
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      if (errno == EWOULDBLOCK) {
        std::string holder;
        char buf[32] = {0};
        if (::pread(fd, buf, sizeof(buf) - 1, 0) > 0) holder = std::string(buf);
        while (!holder.empty() && (holder.back() == '\n' || holder.back() == ' ')) holder.pop_back();
        error = "spool_dir " + dir + " is already in use" +
                (holder.empty() ? std::string() : " by process " + holder) +
                ". Two writers on one spool directory overwrite each other's chunks: give each "
                "node its own spool_dir";
      } else {
        say(LogLevel::kWarn, "spool: cannot lock " + lock_path + " (" + ErrnoText() +
                                 "); nothing stops a second process from sharing this directory");
      }
      ::close(fd);
      return;
    }
    const std::string pid = std::to_string(::getpid()) + "\n";
    // Best effort: the pid only makes a refusal more helpful.
    if (::pwrite(fd, pid.data(), pid.size(), 0) < 0 ||
        ::ftruncate(fd, static_cast<off_t>(pid.size())) != 0) {
      say(LogLevel::kDebug, "spool: could not record this process in " + lock_path);
    }
    lock_fd.store(fd);
  }

  // Whether the lock file this spool holds is still the one at lock_path. False
  // when the directory was removed and recreated under it, by anything.
  bool LockIsStillTheDirectorys() const {
    const int fd = lock_fd.load();
    if (fd < 0) return true;
    struct stat held {}, named {};
    if (::fstat(fd, &held) != 0 || ::stat(lock_path.c_str(), &named) != 0) return false;
    return held.st_dev == named.st_dev && held.st_ino == named.st_ino;
  }

  // The spool directory, or part of it, was removed while running -- by an
  // operator clearing a backlog, or a cleaner. Recreate what is missing, take the
  // lock again if it went too, and forget the chunks that went with it. If another
  // process has already claimed a recreated directory, this one stops writing to
  // it. Caller holds no lock of this spool's.
  bool RecoverDirectory() {
    Said said;
    bool recovered = false;
    {
      std::lock_guard<std::mutex> markers(marker_mu);
      std::lock_guard<std::mutex> lock(mu);
      recovered = RecoverDirectoryLocked(&said);
    }
    for (const auto& [level, message] : said) log->Log(level, message);
    return recovered;
  }

  // Caller holds marker_mu and `mu`.
  bool RecoverDirectoryLocked(Said* said) {
    if (!error.empty()) return false;
    const bool lock_ours = LockIsStillTheDirectorys();
    std::error_code ec;
    const bool chunks_there = fs::exists(chunk_dir, ec);
    const bool markers_there = fs::exists(marker_dir, ec);
    if (lock_ours && chunks_there && markers_there) return true;
    if (!lock_ours) {
      struct stat named {};
      if (::stat(lock_path.c_str(), &named) == 0) {
        error = "spool_dir " + dir + " was removed and recreated by another process; this one "
                "has stopped writing to it";
        said->emplace_back(LogLevel::kError, "spool: " + error);
        return false;
      }
      MakePrivateDirectory(dir, false);
      ::close(lock_fd.exchange(-1));
      Lock(said);
      if (!error.empty()) {
        said->emplace_back(LogLevel::kError, "spool: " + error);
        return false;
      }
    }
    MakePrivateDirectory(chunk_dir, true);
    MakePrivateDirectory(marker_dir, true);
    if (!lock_ours || !chunks_there) {
      said->emplace_back(LogLevel::kWarn, "spool: " + (lock_ours ? chunk_dir : dir) +
                                              " had been removed; recreated it. The " +
                                              std::to_string(count) +
                                              " chunk(s) it held went with it");
      index.clear();
      total_bytes = 0;
      count = 0;
    }
    return true;
  }

  // Writes `text` to `path` through a temporary file, mode 0600. With `durable`,
  // the data reaches the device before the rename and the directory after it.
  // Leaves nothing behind on failure. Caller holds no lock of this spool's except,
  // for a marker, marker_mu -- which RecoverDirectory takes, so a marker write
  // does not try to recover.
  //
  // The temporary file is always created anew, never opened: whatever already stands
  // at its name -- one a crash left, or a link planted to make this write truncate its
  // target -- is unlinked, which removes a link and not what it names.
  bool WriteFileAtomically(const std::string& path, const std::string& parent,
                           const std::string& text, bool durable, bool may_recover) {
    const std::string tmp = path + kTmpExt;
    bool cleared = false;
    bool recovered = false;
    while (true) {
      const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
                            S_IRUSR | S_IWUSR);
      if (fd < 0) {
        if (errno == EEXIST && !cleared) {
          cleared = true;
          ::unlink(tmp.c_str());
          continue;
        }
        if (errno == ENOENT && !recovered && may_recover && RecoverDirectory()) {
          recovered = true;
          continue;
        }
        log->Error("spool: cannot open " + tmp + ": " + ErrnoText());
        return false;
      }
      std::size_t written = 0;
      std::string what;
      while (written < text.size()) {
        const ssize_t n = ::write(fd, text.data() + written, text.size() - written);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) {
          what = "write failed: " + ErrnoText();
          break;
        }
        written += static_cast<std::size_t>(n);
      }
      bool ok = written == text.size();
      if (ok && durable && SyncData(fd) != 0) {
        ok = false;
        what = "flush to the device failed: " + ErrnoText();
      }
      if (::close(fd) != 0 && ok) {
        ok = false;
        what = "close failed: " + ErrnoText();
      }
      if (ok && ::rename(tmp.c_str(), path.c_str()) != 0) {
        ok = false;
        what = "rename failed: " + ErrnoText();
      }
      if (!ok) {
        ::unlink(tmp.c_str());
        log->Error("spool: " + what + " for " + path);
        return false;
      }
      if (durable && !SyncDirectory(parent)) {
        log->Warn("spool: could not flush " + parent + " (" + ErrnoText() +
                  "); what was just written there may not survive a power cut");
      }
      return true;
    }
  }

  // Caller holds `mu`.
  void Rescan() {
    total_bytes = 0;
    count = 0;
    index.clear();
    ForEachRegularFile(chunk_dir, [this](const fs::path& path) {
      std::error_code ec;
      // A write the process died inside. The lock says no other process is
      // writing here, so it is nobody's.
      if (EndsWith(path.string(), kTmpExt)) {
        fs::remove(path, ec);
        return;
      }
      if (path.extension() != kChunkExt) return;
      const std::string id = path.stem().string();
      const std::uintmax_t file_size = fs::file_size(path, ec);
      Entry entry;
      entry.size = ec ? 0 : static_cast<std::uint64_t>(file_size);
      // The stream and the properties flag are encoded in the id so eviction can
      // pick by priority without opening every file.
      const auto dash = id.find('-');
      if (dash != std::string::npos) {
        std::string suffix = id.substr(dash + 1);
        entry.carries_properties = EndsWith(suffix, kPropertiesSuffix);
        if (entry.carries_properties) {
          suffix.resize(suffix.size() - std::strlen(kPropertiesSuffix));
        }
        entry.stream = StreamFromName(suffix);
      }
      index[id] = entry;
      total_bytes += entry.size;
      ++count;
      // Reseed the sequence counter so ordering survives a restart.
      try {
        std::uint64_t n = std::stoull(id.substr(0, dash == std::string::npos ? id.size() : dash));
        if (n >= seq) seq = n + 1;
      } catch (...) {
      }
    });
  }

  // Markers half-written when a process died. Nothing reads them.
  void PruneMarkers() {
    ForEachRegularFile(marker_dir, [](const fs::path& path) {
      std::error_code remove_ec;
      if (EndsWith(path.string(), kTmpExt)) fs::remove(path, remove_ec);
    });
  }
};

DiskSpool::DiskSpool(std::string dir, std::uint64_t max_bytes, std::shared_ptr<ILogger> log)
    : impl_(new Impl) {
  impl_->dir = std::move(dir);
  impl_->chunk_dir = impl_->dir + "/chunks";
  impl_->marker_dir = impl_->dir + "/sessions";
  impl_->lock_path = impl_->dir + "/" + kLockName;
  impl_->max_bytes = max_bytes;
  impl_->log = log ? log : DefaultLogger();
  MakePrivateDirectory(impl_->dir, false);
  impl_->Lock();
  if (!impl_->error.empty()) {
    impl_->log->Error("spool: " + impl_->error);
    return;
  }
  MakePrivateDirectory(impl_->chunk_dir, true);
  MakePrivateDirectory(impl_->marker_dir, true);
  impl_->PruneMarkers();
  std::lock_guard<std::mutex> lock(impl_->mu);
  impl_->Rescan();
  if (impl_->count > 0) {
    impl_->log->Info("spool: recovered " + std::to_string(impl_->count) +
                     " chunk(s), " + std::to_string(impl_->total_bytes) + " bytes");
  }
}

DiskSpool::~DiskSpool() {
  if (impl_ && impl_->lock_fd.load() >= 0) ::close(impl_->lock_fd.load());
}

bool DiskSpool::Write(const Chunk& chunk) { return WriteNamed(chunk, nullptr); }

bool DiskSpool::WriteNamed(const Chunk& chunk, std::string* id_out) {
  if (id_out) id_out->clear();
  json envelope = json::object();
  envelope["schema"] = kChunkSchema;
  envelope["url"] = chunk.url;
  envelope["scene_id"] = chunk.scene_id;
  envelope["scene_version"] = chunk.scene_version;
  envelope["key_fingerprint"] = chunk.key_fingerprint;
  envelope["session_id"] = chunk.session_id;
  envelope["stream"] = StreamName(chunk.stream);
  envelope["created_at"] = chunk.created_at;
  // The body is opaque to the spool and is never re-parsed -- but the ENVELOPE is
  // JSON, and a JSON string holds only valid UTF-8: DumpJson replaces every
  // invalid sequence with U+FFFD. A packed Cap'n Proto payload is arbitrary bytes,
  // so putting one in `body` would corrupt it beyond decoding.
  //
  // The test is the CONTENT, not the stream: a rule keyed on `camera` or `lidar`
  // would leave the JSON streams one malformed UTF-8 property away from the same
  // corruption, and a body that happens to be valid text round-trips byte for byte
  // through a JSON string either way (embedded NULs and control characters
  // included, as \u0000 escapes). So text stays readable in the chunk file, which
  // is what makes a spooled chunk diagnosable with `cat`, and only what cannot be
  // text pays for base64.
  if (IsValidUtf8(chunk.body)) {
    envelope["body"] = chunk.body;
  } else {
    envelope["body_b64"] = Base64Encode(chunk.body);
  }
  // Serialised before the cap is checked, so the cap is held to the file's exact
  // size -- base64 included -- rather than to an estimate of it.
  const std::string text = DumpJson(envelope);
  const std::uint64_t incoming = text.size();

  if (!impl_->LockIsStillTheDirectorys() && !impl_->RecoverDirectory()) return false;

  std::string id;
  std::vector<std::string> victims;
  Stream last_victim = Stream::kEvents;
  std::uint64_t evicted_before = 0;
  std::uint64_t evicted_after = 0;
  bool refused = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (!impl_->error.empty()) return false;
    // Enforce the cap ourselves rather than hoping. Evict the lowest-priority
    // stream's oldest chunks first: an unbounded spool is how a telemetry agent
    // fills a robot's disk. Whether the chunk fits at all is decided first, against
    // what may be evicted for it -- its own tier and below -- so a chunk refused
    // anyway never costs the chunks that would have made room for it.
    const int incoming_tier = EvictionTier(chunk.stream, chunk.carries_properties);
    if (impl_->total_bytes + impl_->reserved_bytes + incoming > impl_->max_bytes) {
      std::uint64_t evictable = 0;
      for (const auto& kv : impl_->index) {
        if (kv.second.tier() <= incoming_tier) evictable += kv.second.size;
      }
      refused = impl_->total_bytes - std::min(impl_->total_bytes, evictable) +
                    impl_->reserved_bytes + incoming >
                impl_->max_bytes;
    }
    evicted_before = impl_->evicted.load();
    while (!refused && impl_->total_bytes + impl_->reserved_bytes + incoming > impl_->max_bytes &&
           !impl_->index.empty()) {
      auto victim = impl_->index.end();
      int best_priority = 1 << 30;
      for (auto it = impl_->index.begin(); it != impl_->index.end(); ++it) {
        const int p = it->second.tier();
        if (p < best_priority) {
          best_priority = p;
          victim = it;
        }
      }
      if (victim == impl_->index.end()) break;
      // Never evict to make room for something we would immediately evict again.
      if (best_priority > incoming_tier) break;
      victims.push_back(impl_->ChunkPath(victim->first));
      impl_->total_bytes -= std::min(impl_->total_bytes, victim->second.size);
      if (impl_->count) --impl_->count;
      last_victim = victim->second.stream;
      impl_->index.erase(victim);
      ++impl_->evicted;
    }
    evicted_after = impl_->evicted.load();
    if (!refused) {
      id = PadNumber(impl_->seq++, 12) + "-" + StreamName(chunk.stream) +
           (chunk.carries_properties ? kPropertiesSuffix : "");
      impl_->reserved_bytes += incoming;
    }
  }
  if (refused) {
    impl_->log->Warn("spool: at capacity, dropping a " + std::string(StreamName(chunk.stream)) +
                     " chunk");
    return false;
  }
  for (const auto& path : victims) ::unlink(path.c_str());
  // Said, because an eviction is data that will never reach the platform, and
  // nothing else in the log would show it: the part-number gap on the platform
  // is the only other trace. Once at the first and then every 50th, since a
  // spool at its cap evicts on nearly every write.
  if (evicted_after > evicted_before &&
      (evicted_before == 0 || evicted_before / 50 != evicted_after / 50)) {
    impl_->log->Warn("spool: at its " + std::to_string(impl_->max_bytes / (1024 * 1024)) +
                     " MiB cap, evicted the oldest " + std::string(StreamName(last_victim)) +
                     " chunk(s) to make room; " + std::to_string(evicted_after) +
                     " evicted so far, and they will not be uploaded");
  }

  const bool written = impl_->WriteFileAtomically(impl_->ChunkPath(id), impl_->chunk_dir, text,
                                                  /*durable=*/true, /*may_recover=*/true);
  std::lock_guard<std::mutex> lock(impl_->mu);
  impl_->reserved_bytes -= std::min(impl_->reserved_bytes, incoming);
  if (!written) {
    ++impl_->write_failures;
    return false;
  }
  Impl::Entry entry;
  entry.size = incoming;
  entry.stream = chunk.stream;
  entry.carries_properties = chunk.carries_properties;
  impl_->index[id] = entry;
  impl_->total_bytes += incoming;
  ++impl_->count;
  if (id_out) *id_out = id;
  return true;
}

bool DiskSpool::PeekOldest(Chunk* out, const std::set<std::string>& skip) {
  // A chunk discarded below leaves the index, and the scan goes on to the one behind it: a caller
  // reads false as "nothing to send", so a discard that returned it would end a pass early.
  while (true) {
    std::string id;
    Impl::Entry entry;
    {
      std::lock_guard<std::mutex> lock(impl_->mu);
      auto found = impl_->index.begin();
      while (found != impl_->index.end() && skip.count(found->first) > 0) ++found;
      if (found == impl_->index.end()) return false;
      id = found->first;
      entry = found->second;
    }
    const std::string path = impl_->ChunkPath(id);
    // Forgets `id` if it is still indexed: a chunk that cannot be read must not
    // wedge the queue behind it forever.
    auto forget = [this, &id] {
      std::lock_guard<std::mutex> lock(impl_->mu);
      auto it = impl_->index.find(id);
      if (it == impl_->index.end()) return;
      impl_->total_bytes -= std::min(impl_->total_bytes, it->second.size);
      if (impl_->count) --impl_->count;
      impl_->index.erase(it);
    };

    std::ifstream in(path, std::ios::binary);
    if (!in) {
      impl_->log->Warn("spool: chunk " + id + " vanished, dropping from index");
      forget();
      continue;
    }
    std::ostringstream ss;
    ss << in.rdbuf();

    json envelope;
    try {
      envelope = json::parse(ss.str());
    } catch (const std::exception& e) {
      impl_->log->Error("spool: chunk " + id + " is unparseable, discarding: " + e.what());
      ::unlink(path.c_str());
      forget();
      continue;
    }

    out->id = id;
    out->url = envelope.value("url", "");
    out->scene_id = envelope.value("scene_id", "");
    out->scene_version = envelope.value("scene_version", "");
    out->key_fingerprint = envelope.value("key_fingerprint", "");
    // `body_b64` when present, else `body`: Write puts a text body in `body` and anything else in
    // `body_b64`.
    if (envelope.contains("body_b64")) {
      if (!Base64Decode(envelope.value("body_b64", ""), &out->body)) {
        impl_->log->Error("spool: chunk " + id + " has an unreadable body_b64, discarding");
        ::unlink(path.c_str());
        forget();
        continue;
      }
    } else {
      out->body = envelope.value("body", "");
    }
    out->session_id = envelope.value("session_id", "");
    const std::string stream = envelope.value("stream", "events");
    if (!IsSentStream(stream)) {
      impl_->log->Warn("spool: chunk " + id + " is of the '" + stream +
                       "' stream, which this SDK does not send; discarding it unsent");
      ::unlink(path.c_str());
      forget();
      continue;
    }
    out->stream = StreamFromName(stream);
    out->carries_properties = entry.carries_properties;
    out->created_at = envelope.value("created_at", 0.0);
    out->size_bytes = entry.size;
    return true;
  }
}

void DiskSpool::Remove(const std::string& chunk_id) {
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    auto it = impl_->index.find(chunk_id);
    if (it == impl_->index.end()) return;
    impl_->total_bytes -= std::min(impl_->total_bytes, it->second.size);
    if (impl_->count) --impl_->count;
    impl_->index.erase(it);
  }
  ::unlink(impl_->ChunkPath(chunk_id).c_str());
}

std::uint64_t DiskSpool::TotalBytes() const {
  std::lock_guard<std::mutex> lock(impl_->mu);
  return impl_->total_bytes;
}

std::size_t DiskSpool::Count() const {
  std::lock_guard<std::mutex> lock(impl_->mu);
  return impl_->count;
}

std::string DiskSpool::error() const {
  std::lock_guard<std::mutex> lock(impl_->mu);
  return impl_->error;
}

bool DiskSpool::Contains(const std::string& chunk_id) const {
  std::lock_guard<std::mutex> lock(impl_->mu);
  return impl_->index.count(chunk_id) != 0;
}

std::uint64_t DiskSpool::evicted() const { return impl_->evicted.load(); }

std::uint64_t DiskSpool::write_failures() const { return impl_->write_failures.load(); }

// A marker's first write reaches the device, data and directory entry, before the
// session spools anything: a power cut must not leave durable chunks with no marker to
// close their session. Every later write is a heartbeat and is not flushed: losing the
// newest only makes the marker read older.
void DiskSpool::WriteMarker(const SessionMarker& m) {
  if (!error().empty()) return;
  json j = json::object();
  j["session_id"] = m.session_id;
  j["scene_id"] = m.scene_id;
  j["scene_version"] = m.scene_version;
  j["device_id"] = m.device_id;
  j["gateway"] = m.gateway;
  j["key_fingerprint"] = m.key_fingerprint;
  j["started_at"] = m.started_at;
  j["last_seen"] = m.last_seen;
  j["max_silence_s"] = m.max_silence_s;
  j["next_events_part"] = m.next_events_part;
  const std::string text = DumpJson(j);
  // A sessions/ that cannot be stat'd is not a missing one: the write below fails and says
  // why, where recreating it could only fail the same way.
  std::error_code ec;
  const bool markers_missing = !fs::exists(impl_->marker_dir, ec) && !ec;
  if (!impl_->LockIsStillTheDirectorys() || markers_missing) {
    if (!impl_->RecoverDirectory()) return;
  }
  std::lock_guard<std::mutex> lock(impl_->marker_mu);
  const bool first = impl_->durable_markers.count(m.session_id) == 0;
  if (impl_->WriteFileAtomically(impl_->MarkerPath(m.session_id), impl_->marker_dir, text,
                                 /*durable=*/first, /*may_recover=*/false) &&
      first) {
    impl_->durable_markers.insert(m.session_id);
  }
}

std::vector<SessionMarker> DiskSpool::OpenMarkers() const {
  std::vector<SessionMarker> result;
  if (!error().empty()) return result;
  std::lock_guard<std::mutex> lock(impl_->marker_mu);
  ForEachRegularFile(impl_->marker_dir, [&result](const fs::path& path) {
    if (path.extension() != kMarkerExt) return;
    std::ifstream in(path, std::ios::binary);
    if (!in) return;
    std::ostringstream ss;
    ss << in.rdbuf();
    try {
      json j = json::parse(ss.str());
      SessionMarker m;
      m.session_id = j.value("session_id", "");
      m.scene_id = j.value("scene_id", "");
      m.scene_version = j.value("scene_version", "");
      m.device_id = j.value("device_id", "");
      m.gateway = j.value("gateway", "");
      m.key_fingerprint = j.value("key_fingerprint", "");
      m.started_at = j.value("started_at", 0.0);
      m.last_seen = j.value("last_seen", 0.0);
      m.max_silence_s = j.value("max_silence_s", 0.0);
      m.next_events_part = j.value("next_events_part", static_cast<std::uint64_t>(0));
      if (!m.session_id.empty()) result.push_back(std::move(m));
    } catch (...) {
    }
  });
  std::sort(result.begin(), result.end(),
            [](const SessionMarker& a, const SessionMarker& b) {
              return a.started_at < b.started_at;
            });
  return result;
}

// The removal reaches the device too: a marker a power cut brought back would have
// recovery add a second, false end to a session that ended in order.
void DiskSpool::CloseMarker(const std::string& session_id) {
  if (!error().empty()) return;
  std::lock_guard<std::mutex> lock(impl_->marker_mu);
  impl_->durable_markers.erase(session_id);
  if (::unlink(impl_->MarkerPath(session_id).c_str()) == 0) SyncDirectory(impl_->marker_dir);
}

}  // namespace c3d
