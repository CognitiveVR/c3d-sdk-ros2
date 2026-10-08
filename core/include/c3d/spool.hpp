#pragma once

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "c3d/logger.hpp"
#include "c3d/types.hpp"

namespace c3d {

// One serialized, self-contained upload. A chunk recovered from disk months
// later must be interpretable on its own, so it carries its destination, its
// session identity and its schema version alongside the body.
struct Chunk {
  // Filename stem. Monotonic, so FIFO ordering is lexicographic, and REQUIRED
  // to be unique for the lifetime of the ISpool instance -- an implementation
  // must never reissue an id, including one whose chunk has been removed or
  // evicted. Session keys what it remembers about a chunk on it -- held, refused,
  // deferred, unremovable, and the 401 allowance -- so a recycled id would hand the
  // next chunk another's state: held or skipped when it should go, or a spent
  // allowance that pauses uploads at its first refusal.
  // DiskSpool satisfies this with a counter that only ever increases; a
  // content-hash or recycling spool would not, which is why this is stated in
  // the contract rather than left to the one implementation that happens to
  // meet it.
  std::string id;
  // Where the chunk was recorded for. Upload compares its host with the current
  // configuration and rebuilds the URL it sends to; this one is never sent to.
  std::string url;
  // The scene the chunk was recorded for. A JSON stream's rebuilt URL carries it; a binary
  // stream's URL names no scene, and its body does.
  std::string scene_id;
  std::string scene_version;
  // The application key the chunk was recorded under, as a fingerprint and never the key.
  // Upload holds a chunk recorded under another key FOR ANOTHER SCENE, as it holds one recorded
  // for another host: a scene id of one project posted under another project's key is not
  // refused, it lands in a new, empty scene of the second project. Another key for the same
  // scene is a rotated key, and is sent under the current one. Empty means not declared: the
  // current key.
  std::string key_fingerprint;
  std::string body;
  std::string session_id;
  Stream stream = Stream::kEvents;
  // A gaze part that carries session properties, which is evicted at the events tier rather
  // than its stream's: the platform accepts and then discards a session none of whose parts
  // carries `c3d.app.version`, so evicting this one small chunk loses the whole session.
  // DiskSpool records it in the chunk's id, where eviction reads it without opening a file.
  bool carries_properties = false;
  double created_at = 0.0;
  std::uint64_t size_bytes = 0;
};

// Marker for one session's lifetime. Written at Begin, touched as data flows,
// closed at End. A marker that is still open on the next process start means
// the previous run died without an orderly shutdown.
struct SessionMarker {
  std::string session_id;
  std::string scene_id;
  std::string scene_version;
  std::string device_id;
  // The data host the session was recorded against. Recovery writes the session's
  // abnormalEnd for this host, so it is held with the rest of the session's data until
  // a process configured for that host drains it, and never closes the session on
  // another stack. Empty means not declared: the current host.
  std::string gateway;
  // The application key the session was recorded under, as Chunk::key_fingerprint. Recovery
  // records the abnormalEnd under it, so the end goes where the rest of the session's data
  // goes: held with it when both the key and the scene have changed, sent with it when only
  // the key has. Empty means not declared: the current key.
  std::string key_fingerprint;
  double started_at = 0.0;
  double last_seen = 0.0;
  // The writer's own bound on how long it can go quiet while perfectly healthy:
  // its configured threshold, or its heartbeat plus one worker tick, whichever is
  // larger. A reader must not adopt this marker before it.
  //
  // A reader judging staleness with its OWN threshold adopts a slower writer's
  // live session even though both configs pass Validate(). The writer knows its
  // numbers and the reader does not, so the writer states its budget and the
  // reader honours it. Uploads are not in the budget: they run on a thread of
  // their own and cannot delay the heartbeat.
  //
  // 0 means "not declared" (a foreign writer), and the reader falls back to its
  // own threshold.
  double max_silence_s = 0.0;
  // The part number the session's NEXT events payload would carry, so the
  // abnormalEnd that recovery writes for it takes a number the session never
  // sent. The gateway keys every event on its part and its index in the part, so
  // a recovered end reusing an early part number lands on an event the session
  // already delivered -- its own sessionStart. Rewritten before every events part
  // is spooled, so no spooled or uploaded events part can be at or above it.
  //
  // 0 means "not declared" (a marker whose writer did not say).
  std::uint64_t next_events_part = 0;
};

class ISpool {
 public:
  virtual ~ISpool() = default;

  virtual bool Write(const Chunk& chunk) = 0;
  // Write, and on success the id the chunk was spooled under in `*id`, or empty when this
  // spool does not say. Session uses it to tell a session's end still spooled from one the
  // cap evicted. The default says nothing.
  virtual bool WriteNamed(const Chunk& chunk, std::string* id) {
    if (id) id->clear();
    return Write(chunk);
  }
  // Oldest first, passing over every chunk whose id is in `skip`. Returns false when no chunk is
  // left to offer. A chunk that vanished or cannot be read or sent is dropped and passed over, so
  // it neither wedges the queue nor ends a pass while a chunk behind it could go.
  virtual bool PeekOldest(Chunk* out, const std::set<std::string>& skip) = 0;
  bool PeekOldest(Chunk* out) { return PeekOldest(out, {}); }
  virtual void Remove(const std::string& chunk_id) = 0;
  virtual std::uint64_t TotalBytes() const = 0;
  virtual std::size_t Count() const = 0;

  virtual void WriteMarker(const SessionMarker& marker) = 0;
  virtual std::vector<SessionMarker> OpenMarkers() const = 0;
  // The session ended in order: its marker is removed.
  virtual void CloseMarker(const std::string& session_id) = 0;

  // Why this spool cannot be used, or empty when it can. Session::Create refuses a
  // spool that names a reason.
  virtual std::string error() const { return {}; }

  // Whether a chunk is still spooled. Session forgets what it remembers about a chunk
  // that is gone -- evicted by the cap, removed by another user of the spool -- so its
  // held and deferred counts stay true on a long-running node. The default never
  // forgets.
  virtual bool Contains(const std::string& chunk_id) const {
    (void)chunk_id;
    return true;
  }
  // Chunks evicted to stay under the cap. Reported in Session's Stats.
  virtual std::uint64_t evicted() const { return 0; }
};

// Disk-backed spool.
//
// Three deliberate differences from the Unity SDK's cache, all of which are
// robot-motivated:
//   1. FIFO, not LIFO. Unity replays newest-first, which reorders a recovered
//      session. Ours replays in the order the data happened.
//   2. One self-describing JSON file per chunk, not two lines of a shared text
//      file whose format depends on line pairing surviving every write.
//   3. A hard failure on one chunk does not abort the replay of the rest.
//
// One process at a time. Construction takes an exclusive lock on `<dir>/.lock`,
// which the kernel drops when the process dies however it dies, and a second
// DiskSpool on the same directory -- in this process or another -- is unusable
// and says why in error(): two writers on one directory issue the same chunk ids
// and overwrite each other's data. The directory, any parent it has to create, and its
// subdirectories are created 0700 and every file 0600, because chunks hold poses and
// camera frames; a spool_dir that already exists keeps its owner's mode. A chunk is
// written to a temporary file, flushed to the device, renamed into place and the
// directory flushed, so after a power cut it is either complete or absent.
class DiskSpool : public ISpool {
 public:
  DiskSpool(std::string dir, std::uint64_t max_bytes, std::shared_ptr<ILogger> log);
  ~DiskSpool() override;

  bool Write(const Chunk& chunk) override;
  bool WriteNamed(const Chunk& chunk, std::string* id) override;
  using ISpool::PeekOldest;
  bool PeekOldest(Chunk* out, const std::set<std::string>& skip) override;
  void Remove(const std::string& chunk_id) override;
  std::uint64_t TotalBytes() const override;
  std::size_t Count() const override;

  void WriteMarker(const SessionMarker& marker) override;
  std::vector<SessionMarker> OpenMarkers() const override;
  void CloseMarker(const std::string& session_id) override;

  std::string error() const override;
  bool Contains(const std::string& chunk_id) const override;
  std::uint64_t evicted() const override;
  // Number of chunk writes that failed, each cleaned up after itself.
  std::uint64_t write_failures() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Creates `dir` as DiskSpool creates its spool_dir: every directory missing from the path, the
// leaf and any parent, owner-only (0700) whatever the umask, and every directory that already
// existed left as its owner made it. For a caller that has to create the directory before a
// spool exists. False when `dir` is not a directory afterwards.
bool MakeSpoolDirectory(const std::string& dir);

}  // namespace c3d
