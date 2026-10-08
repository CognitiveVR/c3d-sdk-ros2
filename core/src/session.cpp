#include "c3d/session.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

#include "c3d/version.hpp"
#include "c3d/wall_clock.hpp"
#include "capnp_payloads.hpp"
#include "serialize.hpp"
#include "util.hpp"

namespace c3d {
namespace {

constexpr const char* kSessionStart = "c3d.sessionStart";
constexpr const char* kSessionEnd = "c3d.sessionEnd";

// The events part a recovered abnormalEnd takes when the orphan's marker does not
// say which part comes next. Any number the session actually sent would put the
// end on top of an event it delivered (part 1 is always its sessionStart), so this
// is one no session reaches: parts are cut at most once per auto-send interval or
// per full batch, and this is more than 11 days of the former at the default 10 s.
constexpr std::uint64_t kUndeclaredNextEventsPart = 100000;

// Consecutive chunk-shaped failures before a chunk becomes a candidate for deferral
// behind the rest of the spool, and the ceiling on a deferred chunk's own doubling
// retry interval.
constexpr int kStrikesBeforeDeferral = 3;
constexpr double kMaxChunkDeferralS = 3600.0;

// How many chunks a pass reads looking for one of another stream before it settles for
// the oldest. Bounded because every chunk looked at is a file read.
constexpr int kPreferenceScan = 16;

// How often the uploader forgets what it remembers about chunks no longer spooled.
constexpr double kPruneIntervalS = 1.0;

// The longest Retry-After honoured. A server is trusted to ask for a longer pause, not
// to stop a robot's uploads for days, and never for a shorter one: 0, or a date the
// robot's clock has already passed, would otherwise retry at every worker tick.
constexpr double kMaxRetryAfterS = 3600.0;

// An answer an outage gives every chunk alike, so it never counts against the one that
// met it: no answer, a captive portal or a redirect, a timeout, a rate limit, and the
// unavailable family a load balancer answers with when nothing is behind it.
bool OutageShaped(long status) {
  return status < 400 || status == 408 || status == 429 || status == 502 || status == 503 ||
         status == 504;
}

std::string LowerCase(std::string text) {
  for (auto& c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return text;
}

struct SensorGate {
  double last_value = 0.0;
  double last_time = -1e18;
  bool primed = false;
};

// A rejection body, made safe to put in a log line. The status code alone names
// the shape of the problem and never the cause: the gateway answers 400 to an
// unknown `color`, a scene version it cannot resolve for the key, and a
// timestamp below its floor, and only the body distinguishes them -- while the
// chunk is discarded either way, so nobody gets a second look at it.
//
// Bounded and sanitised because this is a body from the network: an intercepting
// proxy answers with an HTML page, and a gateway echoing part of a packed Cap'n
// Proto request would otherwise put control bytes on an operator's terminal.
std::string LogSafeBody(const std::string& body) {
  constexpr std::size_t kMaxLogged = 200;
  const std::size_t taken = std::min(body.size(), kMaxLogged);
  std::string out;
  out.reserve(taken + 24);
  for (std::size_t i = 0; i < taken; ++i) {
    const unsigned char c = static_cast<unsigned char>(body[i]);
    out.push_back(c >= 0x20 && c < 0x7F ? static_cast<char>(c) : '.');
  }
  if (body.size() > taken) {
    out += "... (truncated from " + std::to_string(body.size()) + " bytes)";
  }
  return out;
}

}  // namespace

struct Session::Impl {
  Config cfg;
  // Of cfg.application_key, set once at construction. See Chunk::key_fingerprint.
  std::string key_fingerprint;
  std::shared_ptr<ITransport> transport;
  std::shared_ptr<ISpool> spool;
  std::shared_ptr<ILogger> log;

  // The locks, and the only orders they are taken in:
  //
  //   lifecycle_mu -> write turn -> marker_mu -> mu -> the spool's own
  //   pump_mu -> marker_mu -> mu
  //   recovery_mu -> marker_mu -> mu
  //
  // `mu` guards the batches, the properties, the part counters and the stats, and
  // is never held across file or network I/O. That is the whole of the "Record*
  // never blocks" contract: a Record* call takes only `mu`, so it can wait behind a
  // batch being moved out, never behind a part being serialised or written.
  mutable std::mutex mu;
  std::condition_variable cv;
  // Serializes the upload pump. The upload thread drains continuously and
  // callers may also drain explicitly (shutdown, tests). Without this, both can
  // peek the same chunk and POST it twice, which turns at-least-once delivery
  // into visible duplicates on the dashboard.
  std::mutex pump_mu;
  // Serialises Begin() and End() with each other, across their I/O.
  std::mutex lifecycle_mu;
  // Serialises recovery passes, a consumer's and the worker's recheck alike, across their
  // I/O: two passes reading the same markers would both adopt an orphan and spool two ends.
  std::mutex recovery_mu;

  // Serialises every marker write with every other and with CloseMarker. A marker
  // is snapshotted under `mu` and written under this, so markers reach the disk in
  // the order their contents were taken, and a heartbeat in flight can never land
  // after End() removed the marker -- which would resurrect an orderly-ended
  // session as an open one for the next recovery to adopt. It is not `mu`, so a
  // slow marker write delays only the heartbeat, and not `mu` either way round, so
  // a heartbeat never waits behind a chunk being written.
  std::mutex marker_mu;
  // The session whose marker this Session owns on disk, empty when none. Guarded by
  // marker_mu, with the time it was last written.
  std::string marker_id;
  double last_marker_touch = 0.0;

  // Spooling order. A batch of parts is taken out under `mu` together with a
  // ticket, and written outside it once every earlier ticket has been, so parts
  // reach the spool -- and the uploader -- in the order they were cut whichever
  // thread cut them. `next_ticket` is guarded by `mu`, `now_serving` by write_mu.
  std::mutex write_mu;
  std::condition_variable write_cv;
  std::uint64_t next_ticket = 0;
  std::uint64_t now_serving = 0;

  bool active = false;
  bool worker_stop = false;
  // Cuts parts, spools them, heartbeats the marker. Never uploads.
  std::thread worker;
  // Drains the spool. Never cuts a part.
  std::thread uploader;
  // Read by the pump between chunks, so a pause or a shutdown takes effect
  // without waiting out a whole 64-chunk pass. Atomic rather than under `mu`
  // because the pump deliberately holds no lock across a POST.
  std::atomic<bool> uploads_paused{false};
  std::atomic<bool> stopping{false};
  // Whether the last pass found nothing it could send, which the destructor's wait for
  // the tail stops on. A pass records the generation it saw before looking, and every
  // chunk spooled bumps it, so a pass that looked just before a write landed can never
  // report that chunk unsendable.
  static constexpr std::uint64_t kNoEmptyPass = ~std::uint64_t{0};
  std::atomic<std::uint64_t> spooled_generation{0};
  std::atomic<std::uint64_t> empty_pass_generation{kNoEmptyPass};
  bool NothingToSend() const { return empty_pass_generation.load() == spooled_generation.load(); }

  // What threw on the worker or the uploader, each kind said once. A consumer-supplied
  // ISpool may throw where DiskSpool never does, and an exception escaping either thread
  // is std::terminate.
  enum class Thrown { kMarkerWrite, kRemove, kUploadPass, kWorkerPass, kCount };
  std::array<std::atomic<bool>, static_cast<std::size_t>(Thrown::kCount)> thrown_said{};

  // Takes `mu`.
  void Threw(Thrown kind, const std::string& what) {
    {
      std::lock_guard<std::mutex> lock(mu);
      ++stats.thread_exceptions;
    }
    if (thrown_said[static_cast<std::size_t>(kind)].exchange(true)) return;
    static constexpr const char* kWhat[] = {"a session marker write",
                                            "the removal of a chunk that left the queue",
                                            "an upload pass", "a pass of the worker"};
    log->Error(std::string(kWhat[static_cast<std::size_t>(kind)]) + " threw (" + what +
               "); the thread carries on, and this is said once");
  }

  // Runs `call`, and returns false when it threw, which Threw has counted and said.
  template <typename Call>
  bool Guarded(Thrown kind, Call&& call) {
    try {
      call();
      return true;
    } catch (const std::exception& e) {
      Threw(kind, e.what());
    } catch (...) {
      Threw(kind, "a non-standard exception");
    }
    return false;
  }

  std::string session_id;
  // The whole second the last session id was issued for, so Begin can issue the
  // next one strictly later. See Begin.
  std::int64_t last_issued_second = 0;
  bool issued_any = false;
  double session_timestamp = 0.0;
  double session_started_steady = 0.0;

  // Batches
  std::vector<GazeRecord> gaze;
  std::vector<EventRecord> events;
  std::map<std::string, SensorSeries> sensors;
  std::size_t sensor_sample_count = 0;
  std::vector<DynamicSnapshot> dynamics;
  std::vector<ManifestEntry> manifest_pending;
  std::unordered_map<std::string, SensorGate> sensor_gates;

  // The binary buffers, bounded by bytes rather than by count. `*_opened_at` is
  // when the FIRST record of the current part arrived, so the time bound measures
  // the age of the oldest frame in the part and not the age of the newest.
  std::vector<CameraFrameRecord> camera_frames;
  std::uint64_t camera_buffered_bytes = 0;
  double camera_opened_at = 0.0;
  // The cameras whose unusable field of view has been warned about this session:
  // at 30 Hz a per-frame warning is the log.
  std::set<std::string> camera_fov_warned;
  std::vector<LidarScanRecord> lidar_scans;
  std::uint64_t lidar_buffered_bytes = 0;
  double lidar_opened_at = 0.0;
  // Epoch milliseconds, resolved once at Begin. Every binary payload carries it,
  // and recomputing it per part would let two parts of one session disagree about
  // when the session started.
  std::uint64_t session_start_ms = 0;

  // Session properties. `pending` is the delta that has not yet ridden a gaze
  // payload; `known` is everything set so far, kept for a future scene change.
  Properties pending_properties;
  Properties known_properties;
  // What `known_properties` held before the current session opened. Restored on
  // End, so a property set DURING a session cannot outlive it. See Begin.
  Properties presession_properties;

  // Part counters. These reset on End: carried across sessions, they would have a
  // long-lived process send part: 500 on a fresh session's first upload.
  std::map<Stream, std::uint64_t> part;

  double last_auto_send = 0.0;
  double next_attempt_steady = 0.0;
  double current_backoff = 0.0;
  // When the back-off after a retryable failure runs out. Stats::backing_off is read off
  // it, so it reads false once the wait is over, whatever came after the failure: a
  // success, a discard, an eviction, or nothing left but held chunks. Guarded by `mu`.
  double backoff_until_steady = 0.0;
  bool BackingOffLocked() const { return SteadySeconds() < backoff_until_steady; }

  // Steady-clock deadline at which deferred markers get a second look, or 0 for
  // "nothing was deferred". Guarded by `mu`.
  double recovery_recheck_steady = 0.0;

  // Read and written only inside PumpUploadsInternal(), which holds pump_mu
  // for its whole body, and reset by ForgetUnauthorizedAllowance().
  std::string unauthorized_chunk_id;
  int unauthorized_attempts = 0;

  // Uploads paused because the gateway or a proxy refused the credentials. Atomic
  // for the same reason as `uploads_paused`; the probe time is pump_mu's.
  std::atomic<bool> auth_paused{false};
  double next_auth_probe_steady = 0.0;
  // A refusal can be about one route rather than the key -- a stream the project is not
  // entitled to, a proxy rule on one path -- so the streams refused are remembered one by
  // one. While paused, probes go to a stream not yet probed in this round, and the first
  // one accepted resumes every stream but those refused. Their chunks stay spooled,
  // skipped without being read again, and one refused stream at a time is probed every
  // retry_max_s. All guarded by pump_mu.
  std::set<Stream> refused_streams;
  std::map<std::string, Stream> refused_ids;
  std::set<Stream> probed_streams;
  std::optional<Stream> last_refused_probe;

  // Per-chunk failure history for the chunk-shaped failures. Guarded by pump_mu.
  struct ChunkRetry {
    Stream stream = Stream::kEvents;
    int strikes = 0;
    double deferral_s = 0.0;  // 0 until the chunk is deferred
    double next_try_steady = 0.0;
  };
  std::map<std::string, ChunkRetry> chunk_retries;
  // A chunk with kStrikesBeforeDeferral strikes is not deferred on its strikes alone, since
  // a gateway failing everything strikes every chunk alike. The next attempt goes to
  // another chunk, the witness, instead; the candidate is deferred only if a witness is
  // accepted and the candidate then fails again. Guarded by pump_mu.
  std::string deferral_candidate;
  bool candidate_vouched = false;
  // The same evidence, applied to a stream. A stream with a chunk deferred is suspect until
  // one of its chunks is accepted, and the next of its chunks to fail on its own becomes the
  // candidate at its first strike. Deferred too, it defers its whole stream: every chunk of
  // it is passed over, remembered so it is not read again, and one is let through on the
  // stream's own doubling interval until the gateway accepts it. A gateway failing
  // everything fails the witness as well, so an outage defers no stream. Guarded by pump_mu.
  struct StreamDeferral {
    double deferral_s = 0.0;
    double next_try_steady = 0.0;
  };
  std::set<Stream> suspect_streams;
  std::map<Stream, StreamDeferral> deferred_streams;
  std::map<std::string, Stream> stream_deferred_ids;
  // Chunks kept but never sent by this process, and what they were held for -- a host, or
  // another key -- each said once. Guarded by pump_mu.
  std::set<std::string> held_ids;
  std::set<std::string> held_reasons_logged;
  // Chunks delivered or discarded whose removal threw: never sent again by this process,
  // and forgotten once the spool no longer holds them. Guarded by pump_mu.
  std::set<std::string> unremovable_ids;
  // When the per-chunk state above is next checked against the spool. Guarded by pump_mu.
  double next_prune_steady = 0.0;

  void ForgetUnauthorizedAllowance() {
    unauthorized_chunk_id.clear();
    unauthorized_attempts = 0;
  }

  // Sessions whose end this process spooled -- End's or recovery's -- and has not yet
  // seen leave the spool, for what the destructor says it leaves behind, each with the id
  // of the chunk carrying the end, or empty while it is being written or when the spool
  // does not name it. An end the cap evicted is told apart by that id. Guarded by `mu`.
  std::map<std::string, std::string> unsent_ends;

  // Chunks uploaded since the spool was last seen empty. Guarded by pump_mu. Only
  // reported when no session is open -- the configure-only drain of a backlog,
  // where nothing else in the log says how far it has got.
  std::uint64_t drain_uploaded = 0;

  Stats stats;

  // ---- helpers, all called with `mu` held unless noted --------------------

  EnvelopeContext MakeContext(Stream s) {
    EnvelopeContext ctx;
    ctx.session_id = session_id;
    ctx.device_id = cfg.device_id;
    ctx.session_timestamp = session_timestamp;
    ctx.device_type = cfg.device_type;
    ctx.gaze_interval = cfg.gaze_interval_s;
    auto it = part.find(s);
    if (it == part.end()) {
      part[s] = PartBase(s);
      it = part.find(s);
    }
    ctx.part = it->second++;
    return ctx;
  }

  BinaryEnvelope MakeBinaryEnvelope(Stream s) {
    BinaryEnvelope envelope;
    envelope.user_id = cfg.device_id;
    envelope.session_id = session_id;
    envelope.scene_id = cfg.scene_id;
    // Validated at Create: Config::Validate refuses a scene_version that is not
    // a number, precisely so this cannot quietly send 0 and collect a 400 per
    // part for the life of the run.
    SceneVersionNumber(cfg.scene_version, &envelope.scene_version);
    envelope.format_version = FormatVersion(s);
    envelope.session_start_ms = session_start_ms;
    envelope.sdk_version = kSdkVersion;
    auto it = part.find(s);
    if (it == part.end()) {
      part[s] = PartBase(s);
      it = part.find(s);
    }
    envelope.part = static_cast<std::uint32_t>(it->second++);
    return envelope;
  }

  void SetPropertyLocked(const std::string& key, const PropertyValue& value) {
    for (auto& kv : known_properties) {
      if (kv.first == key) {
        if (kv.second == value) return;  // no-op, matches Unity delta semantics
        kv.second = value;
        for (auto& p : pending_properties) {
          if (p.first == key) {
            p.second = value;
            return;
          }
        }
        pending_properties.emplace_back(key, value);
        return;
      }
    }
    known_properties.emplace_back(key, value);
    pending_properties.emplace_back(key, value);
  }

  // One part taken out of the batches, still to be serialised and spooled.
  struct PendingPart {
    Stream stream = Stream::kEvents;
    std::size_t records = 0;
    std::string session_id;
    bool carries_end = false;
    bool carries_properties = false;
    std::function<std::string()> serialize;
  };
  struct PendingBatch {
    std::vector<PendingPart> parts;
    std::uint64_t ticket = 0;
    bool ticketed = false;
  };

  // Moves every batch that is due out of the session, allocating each part its
  // number, and takes a ticket for writing them. Caller holds `mu`; this does no
  // I/O and no serialisation, only moves, so it is the only spooling work a
  // Record* caller can ever wait behind.
  PendingBatch TakeDue(bool force) {
    PendingBatch batch;
    if (session_id.empty()) return batch;
    auto add = [&](Stream stream, std::size_t records, std::function<std::string()> serialize) {
      PendingPart part;
      part.stream = stream;
      part.records = records;
      part.session_id = session_id;
      part.serialize = std::move(serialize);
      batch.parts.push_back(std::move(part));
    };

    if (!events.empty() && (force || events.size() >= cfg.event_batch)) {
      const EnvelopeContext ctx = MakeContext(Stream::kEvents);
      const std::size_t n = events.size();
      add(Stream::kEvents, n,
          [ctx, records = std::move(events)] { return SerializeEvents(ctx, records); });
      events.clear();
    }

    // Gaze also carries session properties, so a properties-only payload with an
    // empty data array is legal and is the only way metadata reaches the platform.
    const bool gaze_due = !gaze.empty() && (force || gaze.size() >= cfg.gaze_batch);
    const bool props_due = !pending_properties.empty() && force;
    if (gaze_due || props_due) {
      const EnvelopeContext ctx = MakeContext(Stream::kGaze);
      const std::size_t n = gaze.size();
      const bool carries_properties = !pending_properties.empty();
      add(Stream::kGaze, n,
          [ctx, records = std::move(gaze), properties = std::move(pending_properties)] {
            return SerializeGaze(ctx, records, properties);
          });
      batch.parts.back().carries_properties = carries_properties;
      gaze.clear();
      pending_properties.clear();
    }

    if (sensor_sample_count > 0 && (force || sensor_sample_count >= cfg.sensor_batch)) {
      std::vector<SensorSeries> series;
      series.reserve(sensors.size());
      for (auto& kv : sensors) series.push_back(std::move(kv.second));
      const EnvelopeContext ctx = MakeContext(Stream::kSensors);
      add(Stream::kSensors, sensor_sample_count,
          [ctx, records = std::move(series)] { return SerializeSensors(ctx, records); });
      sensors.clear();
      sensor_sample_count = 0;
    }

    const std::size_t dyn_total = dynamics.size() + manifest_pending.size();
    if (dyn_total > 0 && (force || dyn_total >= cfg.dynamic_batch)) {
      const EnvelopeContext ctx = MakeContext(Stream::kDynamics);
      add(Stream::kDynamics, dyn_total,
          [ctx, manifest = std::move(manifest_pending), records = std::move(dynamics)] {
            return SerializeDynamics(ctx, manifest, records);
          });
      dynamics.clear();
      manifest_pending.clear();
    }

    // Either bound cuts, and `force` cuts regardless, so End() and Flush() never
    // leave a partial payload buffered in a process that is about to exit.
    const double now = EpochSeconds();
    if (!camera_frames.empty() &&
        (force || camera_buffered_bytes >= cfg.camera_flush_bytes ||
         now - camera_opened_at >= cfg.camera_flush_s)) {
      const BinaryEnvelope envelope = MakeBinaryEnvelope(Stream::kCamera);
      const std::size_t n = camera_frames.size();
      add(Stream::kCamera, n, [envelope, records = std::move(camera_frames)] {
        return SerializeCameraPayload(envelope, records);
      });
      camera_frames.clear();
      camera_buffered_bytes = 0;
    }
    if (!lidar_scans.empty() &&
        (force || lidar_buffered_bytes >= cfg.lidar_flush_bytes ||
         now - lidar_opened_at >= cfg.lidar_flush_s)) {
      const BinaryEnvelope envelope = MakeBinaryEnvelope(Stream::kLidar);
      const std::size_t n = lidar_scans.size();
      add(Stream::kLidar, n, [envelope, records = std::move(lidar_scans)] {
        return SerializeLidarPayload(envelope, records);
      });
      lidar_scans.clear();
      lidar_buffered_bytes = 0;
    }

    if (!batch.parts.empty()) {
      batch.ticket = next_ticket++;
      batch.ticketed = true;
    }
    return batch;
  }

  // Holds the write turn for one ticket: waits for every earlier ticket to have
  // been written, and passes the turn on when it goes, exception or not.
  class WriteTurn {
   public:
    WriteTurn(Impl& impl, std::uint64_t ticket) : impl_(impl) {
      std::unique_lock<std::mutex> lock(impl_.write_mu);
      impl_.write_cv.wait(lock, [&] { return impl_.now_serving == ticket; });
    }
    ~WriteTurn() {
      {
        std::lock_guard<std::mutex> lock(impl_.write_mu);
        ++impl_.now_serving;
      }
      impl_.write_cv.notify_all();
    }
    WriteTurn(const WriteTurn&) = delete;
    WriteTurn& operator=(const WriteTurn&) = delete;

   private:
    Impl& impl_;
  };

  // Serialises and spools a batch TakeDue produced, in its turn. Caller holds no
  // lock. Returns whether every events part in it was spooled.
  //
  // Each part is written behind its own guard. Whatever throws -- a serialiser, an
  // allocation, a consumer-supplied spool -- costs that one part, counted and
  // logged, and never the process: this runs on the worker thread among others,
  // where an escaped exception is std::terminate.
  bool SpoolBatch(PendingBatch& batch) {
    if (!batch.ticketed) return true;
    WriteTurn turn(*this, batch.ticket);
    bool events_spooled = true;
    for (PendingPart& part : batch.parts) {
      // The marker learns the next events part BEFORE this one reaches the spool,
      // so recovery's abnormalEnd can never reuse a number that was sent. A crash
      // between the two leaves a gap in the part numbers, which loses nothing; the
      // other order would leave a repeat, which overwrites an event.
      if (part.stream == Stream::kEvents) WriteMarkerNow(part.session_id);
      // Noted before the write, so an uploader that sends the end at once finds it noted.
      if (part.carries_end) NoteEndSpooled(part.session_id);
      bool spooled = false;
      bool threw = false;
      std::string chunk_id;
      try {
        Chunk chunk;
        chunk.url = BuildUrl(cfg.protocol, cfg.gateway, cfg.network_version, part.stream,
                             cfg.scene_id, cfg.scene_version);
        chunk.scene_id = cfg.scene_id;
        chunk.scene_version = cfg.scene_version;
        chunk.key_fingerprint = key_fingerprint;
        chunk.body = part.serialize();
        chunk.session_id = part.session_id;
        chunk.stream = part.stream;
        chunk.carries_properties = part.carries_properties;
        chunk.created_at = EpochSeconds();
        spooled = spool->WriteNamed(chunk, &chunk_id);
      } catch (const std::exception& e) {
        threw = true;
        log->Error(std::string("could not spool a ") + StreamName(part.stream) + " part (" +
                   e.what() + "); " + std::to_string(part.records) + " record(s) dropped");
      } catch (...) {
        threw = true;
        log->Error(std::string("could not spool a ") + StreamName(part.stream) +
                   " part (a non-standard exception); " + std::to_string(part.records) +
                   " record(s) dropped");
      }
      part.serialize = nullptr;  // the records go now, not when the batch does
      {
        std::lock_guard<std::mutex> lock(mu);
        if (spooled) {
          ++stats.chunks_spooled;
          if (part.carries_end) NameEndChunkLocked(part.session_id, chunk_id);
        } else {
          if (part.carries_end) unsent_ends.erase(part.session_id);
          stats.records_dropped += part.records;
          if (threw) ++stats.serialization_failures;
          else ++stats.spool_write_failures;
        }
      }
      if (spooled) ++spooled_generation;
      if (part.stream == Stream::kEvents && !spooled) events_spooled = false;
      // Durable from its end on: the marker goes before the parts behind the end are written, or
      // a kill while they are leaves it for recovery, which appends a second, abnormal end.
      if (spooled && part.carries_end) CloseMarkerOf(part.session_id);
    }
    return events_spooled;
  }

  // Why a pass sends the chunk it sends.
  enum class Attempt {
    kUpload,   // the oldest chunk that may go
    kWitness,  // another chunk, sent ahead of the deferral candidate to test the gateway
    kProbe,    // a chunk of a stream whose credentials were refused, asking again
    kStreamRetry,  // a chunk of a deferred stream, on the stream's own interval
  };

  // The chunks the pump passes over this time: those held for naming another host or key,
  // those deferred after repeated chunk-shaped failures whose turn has not come round,
  // those of a deferred stream, and, unless asked otherwise, those of a refused stream.
  // Caller holds pump_mu.
  std::set<std::string> SkipSet(double now_steady, bool with_refused = true) const {
    std::set<std::string> skip = held_ids;
    skip.insert(unremovable_ids.begin(), unremovable_ids.end());
    for (const auto& kv : stream_deferred_ids) skip.insert(kv.first);
    for (const auto& kv : chunk_retries) {
      if (kv.second.deferral_s > 0.0 && now_steady < kv.second.next_try_steady) skip.insert(kv.first);
    }
    if (with_refused) {
      for (const auto& kv : refused_ids) skip.insert(kv.first);
    }
    return skip;
  }

  // Caller holds pump_mu. The oldest chunk outside `skip`, preferring one whose stream is
  // not in `avoid` among the first kPreferenceScan it reads. A chunk of a refused or a
  // deferred stream not in `probing` is passed over and remembered, so it is skipped from
  // then on without being read. False when there is none.
  bool PeekPreferring(std::set<std::string> skip, const std::set<Stream>& avoid,
                      const std::set<Stream>& probing, Chunk* out) {
    Chunk first;
    bool have_first = false;
    int scanned = 0;
    while (scanned < kPreferenceScan) {
      Chunk chunk;
      if (!spool->PeekOldest(&chunk, skip)) break;
      skip.insert(chunk.id);
      if (refused_streams.count(chunk.stream) != 0 && probing.count(chunk.stream) == 0) {
        refused_ids.emplace(chunk.id, chunk.stream);
        continue;
      }
      if (deferred_streams.count(chunk.stream) != 0 && probing.count(chunk.stream) == 0) {
        stream_deferred_ids.emplace(chunk.id, chunk.stream);
        continue;
      }
      if (avoid.count(chunk.stream) == 0) {
        *out = std::move(chunk);
        return true;
      }
      if (!have_first) {
        first = std::move(chunk);
        have_first = true;
      }
      ++scanned;
    }
    if (!have_first) return false;
    *out = std::move(first);
    return true;
  }

  // Caller holds pump_mu. The refused stream whose turn it is to be probed.
  Stream NextRefusedStream() const {
    auto it = last_refused_probe ? refused_streams.upper_bound(*last_refused_probe)
                                 : refused_streams.begin();
    if (it == refused_streams.end()) it = refused_streams.begin();
    return *it;
  }

  // Caller holds pump_mu, and not `mu`.
  void PublishRefusedStreams() {
    std::vector<Stream> refused(refused_streams.begin(), refused_streams.end());
    std::lock_guard<std::mutex> lock(mu);
    stats.auth_refused_streams = std::move(refused);
  }

  // Caller holds pump_mu, and not `mu`.
  void ForgetRefusedStream(Stream stream) {
    refused_streams.erase(stream);
    for (auto it = refused_ids.begin(); it != refused_ids.end();) {
      it = it->second == stream ? refused_ids.erase(it) : std::next(it);
    }
    PublishRefusedStreams();
  }

  // Caller holds pump_mu, and not `mu`.
  void PublishDeferredStreams() {
    std::vector<Stream> deferred;
    for (const auto& kv : deferred_streams) deferred.push_back(kv.first);
    std::lock_guard<std::mutex> lock(mu);
    stats.deferred_streams = std::move(deferred);
  }

  // Caller holds pump_mu, and not `mu`.
  void ForgetDeferredStream(Stream stream) {
    deferred_streams.erase(stream);
    for (auto it = stream_deferred_ids.begin(); it != stream_deferred_ids.end();) {
      it = it->second == stream ? stream_deferred_ids.erase(it) : std::next(it);
    }
    PublishDeferredStreams();
  }

  // Caller holds pump_mu. The deferred stream whose turn it is to be tried, if any.
  std::optional<Stream> DueDeferredStream(double now_steady) const {
    for (const auto& kv : deferred_streams) {
      if (now_steady >= kv.second.next_try_steady) return kv.first;
    }
    return std::nullopt;
  }

  // Caller holds pump_mu. The chunk this pass sends next, and why.
  bool NextChunk(double now_steady, Attempt* attempt, Chunk* out) {
    *attempt = Attempt::kUpload;
    if (auth_paused.load()) {
      // A probe, aimed at a stream not yet probed in this round, so one route refusing
      // cannot keep every other paused.
      if (!PeekPreferring(SkipSet(now_steady, false), probed_streams, refused_streams, out)) {
        return false;
      }
      if (probed_streams.count(out->stream) != 0) probed_streams.clear();
      *attempt = Attempt::kProbe;
      return true;
    }
    std::set<std::string> skip = SkipSet(now_steady);
    if (!refused_streams.empty() && now_steady >= next_auth_probe_steady) {
      // One refused stream is let back into the queue for one probe. An older chunk of a
      // stream that is not refused still goes first.
      const Stream target = NextRefusedStream();
      for (const auto& kv : refused_ids) {
        if (kv.second == target) skip.erase(kv.first);
      }
      if (!PeekPreferring(skip, {}, {target}, out)) {
        ForgetRefusedStream(target);  // nothing of it is spooled any more
        return false;
      }
      if (out->stream == target) {
        *attempt = Attempt::kProbe;
        last_refused_probe = target;
      }
      return true;
    }
    if (!deferral_candidate.empty() && !candidate_vouched) {
      // Another stream's chunk first, so a witness accepted ahead of the candidate never
      // reorders the candidate's own stream when it can be helped.
      std::set<std::string> around = skip;
      around.insert(deferral_candidate);
      std::set<Stream> candidate_stream;
      if (const auto it = chunk_retries.find(deferral_candidate); it != chunk_retries.end()) {
        candidate_stream.insert(it->second.stream);
      }
      if (PeekPreferring(around, candidate_stream, {}, out)) {
        *attempt = Attempt::kWitness;
        return true;
      }
      // Nothing else to send: the candidate goes as it would have.
    }
    if (const auto due = DueDeferredStream(now_steady)) {
      // One chunk of the stream is let back into the queue, never one deferred on its own
      // account, so a single chunk that always fails cannot keep its stream deferred. An
      // older chunk of a stream that is not deferred still goes first. After the witness,
      // so a candidate waiting for one is never sent in its place.
      for (const auto& kv : stream_deferred_ids) {
        if (kv.second == *due) skip.erase(kv.first);
      }
      for (const auto& kv : chunk_retries) {
        if (kv.second.stream == *due && kv.second.deferral_s > 0.0) skip.insert(kv.first);
      }
      if (!PeekPreferring(skip, {}, {*due}, out)) {
        ForgetDeferredStream(*due);  // nothing of it can be sent now
        return false;
      }
      if (out->stream == *due) *attempt = Attempt::kStreamRetry;
      return true;
    }
    return PeekPreferring(skip, {}, {}, out);
  }

  // Caller holds pump_mu. Everything this Session remembers about a chunk that has left
  // the spool.
  void ForgetChunk(const Chunk& chunk) {
    if (chunk.stream == Stream::kEvents &&
        chunk.body.find(std::string("\"") + kSessionEnd + "\"") != std::string::npos) {
      std::lock_guard<std::mutex> lock(mu);
      unsent_ends.erase(chunk.session_id);
    }
    const std::string& id = chunk.id;
    chunk_retries.erase(id);
    refused_ids.erase(id);
    stream_deferred_ids.erase(id);
    if (id == deferral_candidate) {
      deferral_candidate.clear();
      candidate_vouched = false;
    }
  }

  // Caller holds pump_mu. Forgets every chunk this Session remembers that the spool no
  // longer holds -- evicted at the cap, typically, which is the one data-loss path a
  // refused key leaves -- so the held and deferred counts stay true and the skip set
  // does not grow for the life of the process.
  void PruneForgottenChunks(double now_steady) {
    if (now_steady < next_prune_steady) return;
    next_prune_steady = now_steady + kPruneIntervalS;
    for (auto it = held_ids.begin(); it != held_ids.end();) {
      it = spool->Contains(*it) ? std::next(it) : held_ids.erase(it);
    }
    for (auto it = unremovable_ids.begin(); it != unremovable_ids.end();) {
      it = spool->Contains(*it) ? std::next(it) : unremovable_ids.erase(it);
    }
    for (auto it = refused_ids.begin(); it != refused_ids.end();) {
      it = spool->Contains(it->first) ? std::next(it) : refused_ids.erase(it);
    }
    for (auto it = stream_deferred_ids.begin(); it != stream_deferred_ids.end();) {
      it = spool->Contains(it->first) ? std::next(it) : stream_deferred_ids.erase(it);
    }
    for (auto it = chunk_retries.begin(); it != chunk_retries.end();) {
      it = spool->Contains(it->first) ? std::next(it) : chunk_retries.erase(it);
    }
    if (!deferral_candidate.empty() && chunk_retries.count(deferral_candidate) == 0) {
      deferral_candidate.clear();
      candidate_vouched = false;
    }
    EndsStillSpooled();
    std::lock_guard<std::mutex> lock(mu);
    stats.chunks_held = held_ids.size();
    stats.chunks_deferred = CountDeferred();
  }

  // Drains the spool. Called without `mu` held: uploading must never block a
  // Record* caller.
  void PumpUploadsInternal() {
    if (uploads_paused.load()) return;
    std::lock_guard<std::mutex> pump_lock(pump_mu);
    const double now_steady = SteadySeconds();
    PruneForgottenChunks(now_steady);
    // While paused for authentication a pass is one probe, and only a success resumes
    // the rest.
    if (auth_paused.load() && now_steady < next_auth_probe_steady) return;
    {
      std::lock_guard<std::mutex> lock(mu);
      if (now_steady < next_attempt_steady) return;
    }

    // Bound the work per pass, so a caller draining explicitly gets control back.
    for (int i = 0; i < 64; ++i) {
      if (uploads_paused.load() || stopping.load()) return;
      // Redundant while the worker runs, since it keeps the heartbeat whatever
      // this thread is doing; kept so a pump never depends on that.
      TouchMarkerIfDue();

      Chunk chunk;
      Attempt attempt = Attempt::kUpload;
      const std::uint64_t generation = spooled_generation.load();
      if (!NextChunk(SteadySeconds(), &attempt, &chunk)) {
        empty_pass_generation.store(generation);
        ReportDrainedIfIdle();
        return;
      }
      empty_pass_generation.store(kNoEmptyPass);

      std::string url;
      if (!UploadUrlFor(chunk, &url)) continue;
      const HttpResponse response =
          transport->Post(url, chunk.body, BuildHeaders(cfg.application_key, chunk.stream));
      const bool paused_probe = attempt == Attempt::kProbe && auth_paused.load();
      if (attempt == Attempt::kProbe) next_auth_probe_steady = SteadySeconds() + cfg.retry_max_s;

      if (response.IsSuccess()) {
        RemoveChunk(chunk.id);
        // Any chunk accepted while a candidate waits says the gateway is up, so the
        // candidate's next failure is its own.
        if (!deferral_candidate.empty() && chunk.id != deferral_candidate) candidate_vouched = true;
        ForgetChunk(chunk);
        ForgetUnauthorizedAllowance();
        ++drain_uploaded;
        const bool resumed = auth_paused.exchange(false);
        const bool stream_was_refused = refused_streams.count(chunk.stream) != 0;
        if (stream_was_refused) ForgetRefusedStream(chunk.stream);
        if (resumed) probed_streams.clear();
        suspect_streams.erase(chunk.stream);
        const bool stream_was_deferred = deferred_streams.count(chunk.stream) != 0;
        if (stream_was_deferred) ForgetDeferredStream(chunk.stream);
        bool session_open = false;
        {
          std::lock_guard<std::mutex> lock(mu);
          ++stats.chunks_uploaded;
          current_backoff = 0.0;
          next_attempt_steady = 0.0;
          backoff_until_steady = 0.0;
          stats.uploads_auth_paused = false;
          stats.chunks_deferred = CountDeferred();
          session_open = !session_id.empty();
        }
        if (resumed && refused_streams.empty()) {
          log->Info("upload authentication accepted again: uploads resume, " +
                    std::to_string(spool->Count()) + " chunk(s) spooled");
        } else if (resumed) {
          std::string refused;
          for (const Stream s : refused_streams) {
            refused += (refused.empty() ? "" : ", ") + std::string(StreamName(s));
          }
          log->Warn("upload authentication accepted again for stream " +
                    std::string(StreamName(chunk.stream)) + ": uploads resume for every stream "
                    "but " + refused + ", whose credentials are still refused. Their chunks stay "
                    "spooled and one of them is tried again every " +
                    std::to_string(static_cast<int>(cfg.retry_max_s)) + "s");
        } else if (stream_was_refused) {
          log->Info("upload authentication accepted again for stream " +
                    std::string(StreamName(chunk.stream)) + ": its chunks upload again");
        }
        if (stream_was_deferred) {
          log->Info("deferred stream " + std::string(StreamName(chunk.stream)) +
                    " accepted again: its chunks upload again");
        }
        if (!session_open && drain_uploaded % 25 == 0) {
          log->Info("draining spool: " + std::to_string(drain_uploaded) + " chunk(s) uploaded, " +
                    std::to_string(spool->Count()) + " remaining");
        }
        continue;
      }

      if (response.IsAuthenticationRejection()) {
        if (!HandleAuthenticationRejection(chunk, response, attempt)) return;
        continue;
      }

      bool go_on = false;
      if (response.IsPayloadRejection()) {
        // The payload itself is wrong. Discard this chunk and keep going: a single
        // poisoned chunk must not wedge the queue behind it.
        RemoveChunk(chunk.id);
        ForgetChunk(chunk);
        // Cleared for EVERY removal: a chunk that answered 401 once and is then
        // discarded must not leave an allowance keyed on an id that is gone.
        ForgetUnauthorizedAllowance();
        {
          std::lock_guard<std::mutex> lock(mu);
          ++stats.chunks_discarded;
          stats.chunks_deferred = CountDeferred();
        }
        std::string message = "upload rejected with status " + std::to_string(response.status) +
                              " for stream " + StreamName(chunk.stream) + "; chunk discarded";
        if (!response.body.empty()) message += "; gateway said: " + LogSafeBody(response.body);
        log->Error(message);
        go_on = true;
      } else {
        go_on = HandleRetryableFailure(chunk, response, attempt);
      }
      if (!go_on || paused_probe) return;
    }
  }

  // Caller holds pump_mu, and not `mu`.
  void RemoveChunk(const std::string& id) {
    if (!Guarded(Thrown::kRemove, [&] { spool->Remove(id); })) unremovable_ids.insert(id);
  }

  // Caller holds pump_mu. The URL a chunk is sent to is built from the CURRENT
  // configuration, never read from the spool: the application key goes only where
  // this process was configured to send it, and a file in the spool directory is
  // not configuration. A chunk recorded for another host -- another environment, a
  // capture stub, or a planted file -- is held, not sent, until a process
  // configured for that host drains it, and so is one recorded for another project --
  // see RecordedForAnotherProject -- until a process with its key does. Returns false
  // for a held chunk.
  bool UploadUrlFor(const Chunk& chunk, std::string* url) {
    std::string host;
    const std::string current = LowerCase(cfg.gateway);
    std::string reason;
    std::string why;
    if (!ParseChunkUrl(chunk.url, &host, nullptr, nullptr)) {
      reason = "(unreadable)";
      why = "its recorded destination cannot be read";
    } else if (host != current) {
      reason = host;
      why = "it was recorded for " + host + " and this process sends to " + current +
            ". The application key goes only where the current configuration names";
    } else if (!IsBinaryStream(chunk.stream) &&
               (chunk.scene_id.empty() || !SceneVersionNumber(chunk.scene_version, nullptr))) {
      reason = "(no scene)";
      why = "it names no scene version to send it under";
    } else if (RecordedForAnotherProject(chunk)) {
      reason = "key " + chunk.key_fingerprint;
      why = "it was recorded under another application key than this process has, for another "
            "scene than this process records to, and the platform files a scene id posted under "
            "another project's key in a new, empty scene of that project";
    }
    if (why.empty()) {
      *url = BuildUrl(cfg.protocol, cfg.gateway, cfg.network_version, chunk.stream,
                      chunk.scene_id, chunk.scene_version);
      return true;
    }
    held_ids.insert(chunk.id);
    {
      std::lock_guard<std::mutex> lock(mu);
      stats.chunks_held = held_ids.size();
    }
    if (held_reasons_logged.insert(reason).second) {
      log->Warn("spool: a " + std::string(StreamName(chunk.stream)) + " chunk is kept and not "
                "sent: " + why + ". It and every other chunk like it stay spooled for a "
                "process configured to send them");
    }
    return false;
  }

  // Whether a chunk was recorded for another project: under another application key AND for
  // another scene than this process records to. Both, because each alone is ordinary -- a key
  // rotated within a project keeps its scene, and a project records to several scenes under
  // one key -- while a scene id posted under another project's key is not refused but lands
  // in a new, empty scene of that project. A chunk that names no scene cannot be told to be
  // the same project's, and counts as another's.
  bool RecordedForAnotherProject(const Chunk& chunk) const {
    return !chunk.key_fingerprint.empty() && chunk.key_fingerprint != key_fingerprint &&
           chunk.scene_id != cfg.scene_id;
  }

  // Caller holds pump_mu. Returns whether the pass may go on to the next chunk.
  //
  // A rejected credential is not the chunk's fault and would meet every other chunk of
  // its stream the same way, so nothing is discarded. After a short, bounded allowance
  // for a transient rejection, ALL uploads pause with everything kept: the key is the
  // likely cause. Probes then go out one per retry_max_s, a stream at a time, and the
  // first accepted resumes every stream not refused. The spool cap bounds the disk
  // meanwhile. current_backoff is never touched here, so a credential hiccup cannot
  // escalate into the transient-outage curve.
  bool HandleAuthenticationRejection(const Chunk& chunk, const HttpResponse& response,
                                     Attempt attempt) {
    const double now = SteadySeconds();
    {
      std::lock_guard<std::mutex> lock(mu);
      ++stats.auth_rejections;
    }
    const std::string status = std::to_string(response.status);
    if (attempt == Attempt::kProbe) {
      refused_streams.insert(chunk.stream);
      refused_ids.emplace(chunk.id, chunk.stream);
      PublishRefusedStreams();
      const bool paused = auth_paused.load();
      if (paused) probed_streams.insert(chunk.stream);
      log->Debug("upload authentication still rejected for stream " +
                 std::string(StreamName(chunk.stream)) + " (status " + status +
                 "); the next probe goes in " +
                 std::to_string(static_cast<int>(cfg.retry_max_s)) + "s");
      return !paused;
    }
    if (chunk.id != unauthorized_chunk_id) {
      unauthorized_chunk_id = chunk.id;
      unauthorized_attempts = 1;
    } else {
      ++unauthorized_attempts;
    }
    if (unauthorized_attempts < cfg.unauthorized_attempts) {
      {
        std::lock_guard<std::mutex> lock(mu);
        next_attempt_steady = now + cfg.unauthorized_retry_s;
        ++stats.unauthorized_retries;
      }
      log->Warn("upload rejected with status " + status + " for stream " +
                StreamName(chunk.stream) + "; retrying (attempt " +
                std::to_string(unauthorized_attempts) + " of " +
                std::to_string(cfg.unauthorized_attempts) + ")");
      return false;
    }
    // Read before the clear, or the message reports the count it just zeroed.
    const int attempts_made = unauthorized_attempts;
    ForgetUnauthorizedAllowance();
    refused_streams.insert(chunk.stream);
    refused_ids.emplace(chunk.id, chunk.stream);
    PublishRefusedStreams();
    probed_streams = {chunk.stream};
    auth_paused.store(true);
    next_auth_probe_steady = now + cfg.retry_max_s;
    {
      std::lock_guard<std::mutex> lock(mu);
      stats.uploads_auth_paused = true;
    }
    log->Error("upload rejected with status " + status + " for stream " +
               StreamName(chunk.stream) + " after " + std::to_string(attempts_made) +
               " attempt(s): the " +
               (response.status == 407 ? std::string("proxy refused the credentials")
                                       : std::string("application key was refused")) +
               ". ALL uploads are paused and everything stays spooled; authentication is tried "
               "again every " +
               std::to_string(static_cast<int>(cfg.retry_max_s)) + "s, a stream at a time");
    return false;
  }

  // Caller holds pump_mu. Returns whether the pass may go on to the next chunk.
  //
  // Every retryable failure backs off globally -- nothing else is likely to succeed
  // either -- except a retry of a chunk already deferred, and the deferral of a chunk the
  // gateway has just been seen to fail on its own.
  //
  // A chunk-shaped failure (404, any other unexpected 4xx, a 5xx other than 502, 503 and
  // 504) is a strike against that one chunk: a payload the gateway answers 500 to every
  // time would otherwise sit at the head of the FIFO for ever, and every chunk behind it
  // with it. Strikes alone cannot tell that chunk from a gateway answering 500 to
  // everything, though, and deferring on them would reorder the backlog through an
  // outage. So a chunk with kStrikesBeforeDeferral strikes becomes the candidate, the
  // next attempt sends a witness instead, and the candidate is deferred behind the rest
  // of the spool only once a witness was accepted and the candidate failed again. A
  // witness that fails as well means an outage: the candidate's strikes start over and
  // the FIFO stands. A deferred chunk is retried on its own doubling interval. It is
  // never deleted for this; only the spool cap can evict it. The same evidence against a
  // second chunk of a stream defers the stream: see suspect_streams.
  bool HandleRetryableFailure(const Chunk& chunk, const HttpResponse& response, Attempt attempt) {
    const double now = SteadySeconds();
    const long status = response.status;
    const bool chunk_shaped = !OutageShaped(status);
    // An outage starting withdraws what an earlier witness said about the gateway.
    if (!chunk_shaped) candidate_vouched = false;

    std::string what = "status " + std::to_string(status);
    if (!response.error.empty()) what += ", " + response.error;
    if (status == 404) {
      what += "; the gateway has no such route, which is a configuration problem: check gateway "
              "and the platform this key belongs to";
    }

    if (attempt == Attempt::kStreamRetry) {
      const auto deferral = deferred_streams.find(chunk.stream);
      if (deferral != deferred_streams.end() && chunk_shaped) {
        deferral->second.deferral_s = std::min(deferral->second.deferral_s * 2.0, kMaxChunkDeferralS);
        deferral->second.next_try_steady = now + deferral->second.deferral_s;
        {
          std::lock_guard<std::mutex> lock(mu);
          ++stats.upload_failures;
        }
        log->Warn("deferred stream " + std::string(StreamName(chunk.stream)) + " failed again (" +
                  what + "); next try in " +
                  std::to_string(static_cast<int>(deferral->second.deferral_s)) + "s");
        return true;
      }
      // An outage: the stream keeps its interval, and the global back-off below applies.
      if (deferral != deferred_streams.end()) {
        deferral->second.next_try_steady = now + deferral->second.deferral_s;
      }
    }

    if (chunk_shaped) {
      ChunkRetry& retry = chunk_retries[chunk.id];
      retry.stream = chunk.stream;
      ++retry.strikes;
      if (retry.deferral_s > 0.0) {
        retry.deferral_s = std::min(retry.deferral_s * 2.0, kMaxChunkDeferralS);
        retry.next_try_steady = now + retry.deferral_s;
        {
          std::lock_guard<std::mutex> lock(mu);
          ++stats.upload_failures;
          stats.chunks_deferred = CountDeferred();
        }
        log->Warn("deferred " + std::string(StreamName(chunk.stream)) + " chunk " + chunk.id +
                  " failed again (" + what + "); next try in " +
                  std::to_string(static_cast<int>(retry.deferral_s)) + "s");
        return true;
      }
      if (chunk.id == deferral_candidate && candidate_vouched) {
        deferral_candidate.clear();
        candidate_vouched = false;
        retry.deferral_s = cfg.retry_max_s;
        retry.next_try_steady = now + retry.deferral_s;
        const bool defer_stream = suspect_streams.count(chunk.stream) != 0;
        if (defer_stream) {
          StreamDeferral& deferral = deferred_streams[chunk.stream];
          deferral.deferral_s = cfg.retry_max_s;
          deferral.next_try_steady = now + deferral.deferral_s;
          PublishDeferredStreams();
        } else {
          suspect_streams.insert(chunk.stream);
        }
        {
          std::lock_guard<std::mutex> lock(mu);
          ++stats.upload_failures;
          stats.chunks_deferred = CountDeferred();
        }
        if (defer_stream) {
          log->Warn("a second " + std::string(StreamName(chunk.stream)) + " chunk has failed (" +
                    what + ") while the gateway accepted another stream's: the whole " +
                    StreamName(chunk.stream) + " stream now waits behind the rest of the spool "
                    "and is retried on its own, starting in " +
                    std::to_string(static_cast<int>(cfg.retry_max_s)) +
                    "s. Its chunks are kept, not deleted");
        } else {
          log->Warn(std::string(StreamName(chunk.stream)) + " chunk " + chunk.id + " has failed " +
                    std::to_string(retry.strikes) + " times (" + what +
                    ") while the gateway accepted another; it now waits behind the rest of the "
                    "spool and is retried on its own, starting in " +
                    std::to_string(static_cast<int>(retry.deferral_s)) +
                    "s. It is kept, not deleted");
        }
        return true;
      }
      const int strikes_needed =
          suspect_streams.count(chunk.stream) != 0 ? 1 : kStrikesBeforeDeferral;
      if (retry.strikes >= strikes_needed && deferral_candidate.empty()) {
        deferral_candidate = chunk.id;
        candidate_vouched = false;
      }
    }
    if (attempt == Attempt::kWitness) {
      if (const auto it = chunk_retries.find(deferral_candidate); it != chunk_retries.end()) {
        it->second.strikes = 0;
      }
      deferral_candidate.clear();
      candidate_vouched = false;
    }

    double wait = 0.0;
    bool lengthened = false;
    {
      std::lock_guard<std::mutex> lock(mu);
      ++stats.upload_failures;
      current_backoff = current_backoff <= 0.0 ? cfg.retry_min_s
                                               : std::min(current_backoff * 2.0, cfg.retry_max_s);
      const double asked = std::min(response.retry_after_s, kMaxRetryAfterS);
      lengthened = asked > current_backoff;
      wait = lengthened ? asked : current_backoff;
      next_attempt_steady = now + wait;
      backoff_until_steady = next_attempt_steady;
    }
    log->Warn("upload failed (" + what + "), backing off " + std::to_string(static_cast<int>(wait)) +
              "s" + (lengthened ? " as the server's Retry-After asked" : ""));
    return false;
  }

  // Caller holds pump_mu.
  std::size_t CountDeferred() const {
    std::size_t n = 0;
    for (const auto& kv : chunk_retries) {
      if (kv.second.deferral_s > 0.0) ++n;
    }
    return n;
  }

  // Caller holds no lock but may hold the write turn.
  void NoteEndSpooled(const std::string& session_id) {
    std::lock_guard<std::mutex> lock(mu);
    unsent_ends.emplace(session_id, std::string());
  }

  // Caller holds `mu`. Once the end's chunk is spooled, under the id the spool gave it --
  // unless the uploader has already sent it and forgotten the session.
  void NameEndChunkLocked(const std::string& session_id, const std::string& chunk_id) {
    const auto it = unsent_ends.find(session_id);
    if (it != unsent_ends.end() && it->second.empty()) it->second = chunk_id;
  }

  // Caller holds no lock of this Session's. Forgets every end whose chunk the spool no
  // longer holds -- evicted at the cap, or removed by another user of the spool -- and
  // returns the sessions whose end is still spooled.
  std::vector<std::string> EndsStillSpooled() {
    std::map<std::string, std::string> ends;
    {
      std::lock_guard<std::mutex> lock(mu);
      ends = unsent_ends;
    }
    std::vector<std::string> spooled;
    std::vector<std::pair<std::string, std::string>> gone;
    for (const auto& [session, chunk_id] : ends) {
      if (chunk_id.empty() || spool->Contains(chunk_id)) {
        spooled.push_back(session);
      } else {
        gone.emplace_back(session, chunk_id);
      }
    }
    std::lock_guard<std::mutex> lock(mu);
    for (const auto& [session, chunk_id] : gone) {
      const auto it = unsent_ends.find(session);
      if (it != unsent_ends.end() && it->second == chunk_id) unsent_ends.erase(it);
    }
    return spooled;
  }

  // What a shutdown leaves in the spool, said once: a one-shot process, or a robot
  // being retired, may never start again, and a session whose end is still spooled
  // stays open on the platform until something drains it.
  void SayWhatIsLeft(const std::string& why) {
    const std::size_t left = spool->Count();
    if (left == 0) return;
    std::string ends;
    for (const auto& id : EndsStillSpooled()) ends += (ends.empty() ? "" : ", ") + id;
    std::string message = "shutting down with " + std::to_string(left) +
                          " chunk(s) still spooled in " + cfg.spool_dir + " (" + why + "), ";
    message += ends.empty() ? "none of them the end of a session this process ran"
                            : "the end of session " + ends +
                                  " among them, which stays open on the platform until then";
    message += ". They upload at the next start that uses this spool_dir";
    if (ends.empty()) {
      log->Info(message);
    } else {
      log->Warn(message);
    }
  }

  // Caller holds pump_mu. The spool reading empty once is not a drain -- a chunk
  // that vanished or would not parse also ends a pass -- so the count decides.
  void ReportDrainedIfIdle() {
    if (drain_uploaded == 0 || spool->Count() != 0) return;
    bool session_open = false;
    {
      std::lock_guard<std::mutex> lock(mu);
      session_open = !session_id.empty();
    }
    if (!session_open) {
      log->Info("spool drained: " + std::to_string(drain_uploaded) +
                " chunk(s) uploaded, none remaining");
    }
    drain_uploaded = 0;
  }

  // The marker of session `id`, from the configuration alone, so Begin can build one
  // before the session is installed. The marker records the interval this writer
  // PROMISES to refresh at, because the reader cannot otherwise know it, and judging a
  // writer by the reader's own threshold lets two configs that each pass Validate()
  // have one adopt the other's live session.
  SessionMarker MarkerFor(const std::string& id, double started_at, double now,
                          std::uint64_t next_events_part) const {
    SessionMarker m;
    m.session_id = id;
    m.scene_id = cfg.scene_id;
    m.scene_version = cfg.scene_version;
    m.device_id = cfg.device_id;
    m.gateway = cfg.gateway;
    m.key_fingerprint = key_fingerprint;
    m.started_at = started_at;
    m.last_seen = now;
    m.max_silence_s = std::max(cfg.orphan_stale_after_s,
                               cfg.marker_heartbeat_s + cfg.worker_tick_s);
    m.next_events_part = next_events_part;
    return m;
  }

  // Caller holds `mu`.
  SessionMarker BuildMarkerLocked(double now) const {
    const auto events_part = part.find(Stream::kEvents);
    return MarkerFor(session_id, session_timestamp, now,
                     events_part != part.end() ? events_part->second
                                               : PartBase(Stream::kEvents));
  }

  // Rewrites the marker of `id` now, if this Session still owns it. Caller holds
  // marker_mu. The snapshot is taken under `mu` and the write made outside it.
  void WriteMarkerLocked(const std::string& id) {
    if (marker_id.empty() || marker_id != id) return;
    const double now = EpochSeconds();
    SessionMarker m;
    {
      std::lock_guard<std::mutex> lock(mu);
      if (session_id != id) return;
      m = BuildMarkerLocked(now);
    }
    Guarded(Thrown::kMarkerWrite, [&] { spool->WriteMarker(m); });
    last_marker_touch = now;
  }

  void WriteMarkerNow(const std::string& id) {
    std::lock_guard<std::mutex> lock(marker_mu);
    WriteMarkerLocked(id);
  }

  // Cleared in the same hold of marker_mu, so no heartbeat can write the marker back, and
  // cleared whether or not the close threw.
  void CloseMarkerOf(const std::string& id) {
    std::lock_guard<std::mutex> lock(marker_mu);
    Guarded(Thrown::kMarkerWrite, [&] { spool->CloseMarker(id); });
    if (marker_id == id) marker_id.clear();
  }

  // The heartbeat, callable from anywhere that might run long. This is what lets the
  // next process start tell an orderly shutdown from a robot that lost power mid-run.
  void TouchMarkerIfDue() {
    std::lock_guard<std::mutex> lock(marker_mu);
    if (marker_id.empty()) return;
    if (EpochSeconds() - last_marker_touch <= cfg.marker_heartbeat_s) return;
    WriteMarkerLocked(marker_id);
  }

  // A marker is judged against the silence budget ITS WRITER declared, never less
  // than our own threshold. Judging a slow writer by a fast reader's threshold
  // adopts the live session of a co-tenant whose config is merely different, not
  // wrong. A marker that declares nothing gets our threshold.
  double StaleAfterFor(const SessionMarker& m) const {
    return std::max(cfg.orphan_stale_after_s, m.max_silence_s);
  }

  int RecoverOrphansInternal() {
    std::lock_guard<std::mutex> recovering(recovery_mu);
    const auto markers = spool->OpenMarkers();
    std::string own_marker;
    {
      std::lock_guard<std::mutex> lock(marker_mu);
      own_marker = marker_id;
    }
    int recovered = 0;
    bool deferred_any = false;
    const double now = EpochSeconds();
    // Logging and spool I/O happen OUTSIDE `mu`. `Session::active()`,
    // `session_id()` and `session_timestamp()` all take that same non-recursive
    // mutex, so a consumer-supplied Logger that calls back into Session would
    // otherwise self-deadlock -- and the spool writes below are file I/O the
    // worker's serialisation path should not be blocked behind.
    std::vector<std::string> left_alone;
    std::vector<SessionMarker> adopt;
    {
      std::lock_guard<std::mutex> lock(mu);
      for (const auto& m : markers) {
        if (m.session_id == session_id || m.session_id == own_marker) continue;

        // Is anything still refreshing this marker? A live session touches it
        // every marker_heartbeat_s, and that heartbeat exists for precisely this
        // decision: adopting a running session -- another Session's on this spool,
        // or a process's sharing a directory that cannot be locked -- declares it
        // abnormally ended and deletes its marker, leaving a genuine later crash
        // unrecoverable.
        //
        // `now - last_seen` is NEGATIVE when the writer's clock is ahead of ours,
        // which must read as live rather than as infinitely stale; `last_seen ==
        // 0` is a session that died before its first heartbeat and must still be
        // recovered, which the epoch-scale difference gives for free.
        const double silent_for = now - m.last_seen;
        if (m.last_seen > 0.0 && silent_for < StaleAfterFor(m)) {
          left_alone.push_back(m.session_id + " (refreshed " +
                               std::to_string(static_cast<int>(silent_for < 0.0 ? 0.0 : silent_for)) +
                               "s ago)");
          deferred_any = true;
          continue;
        }
        adopt.push_back(m);
      }
      // Deferring is not deciding. Schedule the second look before releasing the
      // lock, so a crash inside the window is picked up by THIS process rather
      // than waiting for a restart that may never come.
      if (deferred_any) recovery_recheck_steady = SteadySeconds() + cfg.orphan_stale_after_s;
    }

    for (const auto& name : left_alone)
      log->Info("leaving session " + name + " alone: it is probably still running");

    for (const auto& m : adopt) {
      // Reconstruct the ending we never got to write. Without this every power
      // loss leaves a session that is open forever, and power loss is the exact
      // scenario the spool exists to survive.
      EnvelopeContext ctx;
      ctx.session_id = m.session_id;
      ctx.device_id = m.device_id;
      ctx.session_timestamp = m.started_at;
      ctx.device_type = cfg.device_type;
      // Never a part the session sent: the gateway keys an event on its part and
      // its index, so an end filed under part 1 lands on the sessionStart.
      ctx.part = m.next_events_part > 0 ? m.next_events_part : kUndeclaredNextEventsPart;

      EventRecord end;
      end.name = kSessionEnd;
      end.t = m.last_seen > m.started_at ? m.last_seen : m.started_at;
      end.properties.emplace_back("Reason", std::string("abnormalEnd"));
      end.properties.emplace_back("sessionlength", std::max(0.0, m.last_seen - m.started_at));
      end.properties.emplace_back("c3d.recovered", true);

      // Recorded for the session's own host and under its own key, so a session from
      // another stack or another project is closed there, held alongside its data, and
      // never here.
      const std::string& host = m.gateway.empty() ? cfg.gateway : m.gateway;
      Chunk chunk;
      chunk.key_fingerprint = m.key_fingerprint.empty() ? key_fingerprint : m.key_fingerprint;
      chunk.scene_id = m.scene_id.empty() ? cfg.scene_id : m.scene_id;
      chunk.scene_version = m.scene_version.empty() ? cfg.scene_version : m.scene_version;
      chunk.url = BuildUrl(cfg.protocol, host, cfg.network_version, Stream::kEvents,
                           chunk.scene_id, chunk.scene_version);
      chunk.body = SerializeEvents(ctx, {end});
      chunk.session_id = m.session_id;
      chunk.stream = Stream::kEvents;
      chunk.created_at = EpochSeconds();
      NoteEndSpooled(m.session_id);
      // The marker is closed only once its end is in the spool. A full disk -- a
      // likely cause of the crash being recovered from -- would otherwise close
      // the marker over an end that was never written, leaving the session open on
      // the platform for good. Left open, it gets another look.
      bool written = false;
      std::string end_id;
      try {
        written = spool->WriteNamed(chunk, &end_id);
      } catch (...) {
        std::lock_guard<std::mutex> lock(mu);
        unsent_ends.erase(m.session_id);
        throw;
      }
      {
        std::lock_guard<std::mutex> lock(mu);
        if (written) {
          NameEndChunkLocked(m.session_id, end_id);
        } else {
          unsent_ends.erase(m.session_id);
          recovery_recheck_steady = SteadySeconds() + cfg.orphan_stale_after_s;
        }
      }
      if (!written) {
        log->Error("could not spool the abnormalEnd for orphaned session " + m.session_id +
                   "; its marker is kept and recovery tries again");
        continue;
      }
      ++spooled_generation;
      spool->CloseMarker(m.session_id);
      ++recovered;
      std::string held;
      if (LowerCase(host) != LowerCase(cfg.gateway)) {
        held = "; it was recorded against " + host +
               ", and its end is kept with its data for a process sending there";
      } else if (RecordedForAnotherProject(chunk)) {
        held = "; it was recorded under another application key for another scene, and its end "
               "is kept with its data for a process that has that key";
      }
      log->Warn("recovered orphaned session " + m.session_id + " (ended abnormally after " +
                std::to_string(static_cast<int>(m.last_seen - m.started_at)) + "s)" + held);
    }
    return recovered;
  }

  // Idempotent. Called by Create() so deferred-recovery rechecks happen even when
  // Begin() never does, and still by Begin() so a session restarted after a
  // destructor-less stop gets a worker back.
  void StartWorker() {
    if (worker.joinable()) return;
    worker_stop = false;
    worker = std::thread([this] { WorkerLoop(); });
    uploader = std::thread([this] { UploaderLoop(); });
  }

  void UploaderLoop() {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mu);
        cv.wait_for(lock, std::chrono::duration<double>(cfg.worker_tick_s),
                    [this] { return worker_stop; });
        if (worker_stop) return;
      }
      Guarded(Thrown::kUploadPass, [this] { PumpUploadsInternal(); });
    }
  }

  void WorkerLoop() {
    bool recheck_due = false;
    while (true) {
      PendingBatch batch;
      {
        std::unique_lock<std::mutex> lock(mu);
        cv.wait_for(lock, std::chrono::duration<double>(cfg.worker_tick_s),
                    [this] { return worker_stop; });
        if (worker_stop) return;

        const double now = EpochSeconds();
        const bool timer_due = (now - last_auto_send) >= cfg.auto_send_interval_s;
        batch = TakeDue(timer_due);
        if (timer_due) last_auto_send = now;

        recheck_due = recovery_recheck_steady > 0.0 &&
                      SteadySeconds() >= recovery_recheck_steady;
        // Cleared here, under the lock that guards it. Clearing it after the
        // locked block races a consumer-thread RecoverOrphanedSessions(): that
        // call schedules a recheck under `mu`, and an unlocked zero landing
        // afterwards silently discards the schedule, so the deferred marker
        // waits for some unrelated deferral to reschedule it.
        if (recheck_due) recovery_recheck_steady = 0.0;
      }

      const bool passed = Guarded(Thrown::kWorkerPass, [&] {
        SpoolBatch(batch);

        // Heartbeat, with no lock of the batches held: the pump beats again between
        // uploads.
        TouchMarkerIfDue();

        // A marker left alone for being too fresh must be looked at AGAIN. Recovery
        // runs once at startup, so without this a node that crashes and respawns
        // inside the staleness window defers its own crashed session -- forever,
        // for anything with a restart policy. The staleness gate is safe only with
        // this recheck behind it.
        if (recheck_due) RecoverOrphansInternal();
      });
      if (!passed && recheck_due) {
        std::lock_guard<std::mutex> lock(mu);
        recovery_recheck_steady = SteadySeconds() + cfg.orphan_stale_after_s;
      }

      // No upload here, ever. See UploaderLoop.
    }
  }
};

Session::Session(const Config& config, std::shared_ptr<ITransport> transport,
                 std::shared_ptr<ISpool> spool, std::shared_ptr<ILogger> logger)
    : impl_(new Impl) {
  impl_->cfg = config;
  impl_->key_fingerprint = KeyFingerprint(config.application_key);
  impl_->uploads_paused.store(config.uploads_paused);
  impl_->transport = std::move(transport);
  impl_->spool = std::move(spool);
  impl_->log = logger ? std::move(logger) : DefaultLogger();
}

Session::~Session() {
  if (!impl_) return;
  if (active()) End("destructor");
  // The tail -- usually the session just ended, its end included -- gets a bounded
  // chance to go before the process does. The marker is already closed, so being
  // killed during this wait loses nothing and adds no abnormalEnd: what is left
  // uploads at the next start.
  impl_->cv.notify_all();
  const double deadline = SteadySeconds() + impl_->cfg.shutdown_drain_s;
  std::string stopped = "shutdown_drain_s ran out";
  while (impl_->spool->Count() > 0 && SteadySeconds() < deadline) {
    if (impl_->uploads_paused.load()) {
      stopped = "uploads are paused";
      break;
    }
    if (impl_->auth_paused.load()) {
      stopped = "the credentials are refused";
      break;
    }
    if (impl_->NothingToSend()) {
      stopped = "none of them can be sent by this process now";
      break;
    }
    {
      std::lock_guard<std::mutex> lock(impl_->mu);
      if (impl_->BackingOffLocked()) {
        stopped = "uploads are backing off after a failure";
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  impl_->SayWhatIsLeft(stopped);
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    impl_->worker_stop = true;
  }
  impl_->stopping.store(true);
  impl_->cv.notify_all();
  // An upload in flight can legitimately run for minutes on a slow link; the
  // uploader is not joined behind it. The chunk stays in the spool for next time.
  impl_->transport->Interrupt();
  if (impl_->worker.joinable()) impl_->worker.join();
  if (impl_->uploader.joinable()) impl_->uploader.join();
}

std::unique_ptr<Session> Session::Create(const Config& config,
                                         std::shared_ptr<ITransport> transport,
                                         std::shared_ptr<ISpool> spool,
                                         std::shared_ptr<ILogger> logger,
                                         std::string* error_out) {
  const std::string problem = config.Validate();
  if (!problem.empty()) {
    if (error_out) *error_out = problem;
    if (logger) logger->Error("invalid config: " + problem);
    return nullptr;
  }
  if (!transport || !spool) {
    if (error_out) *error_out = "transport and spool are required";
    return nullptr;
  }
  const std::string spool_problem = spool->error();
  if (!spool_problem.empty()) {
    if (error_out) *error_out = spool_problem;
    if (logger) logger->Error("spool unusable: " + spool_problem);
    return nullptr;
  }
  auto session = std::unique_ptr<Session>(
      new Session(config, std::move(transport), std::move(spool), std::move(logger)));
  // The worker starts HERE, not at Begin(). It is what re-examines a marker that
  // recovery deferred for being too fresh, and a node configured with
  // `auto_start_session: false` may never call Begin() at all -- so a session
  // that crashed and respawned inside the staleness window would stay open on the
  // platform indefinitely, waiting for a Begin() that is not coming. With no
  // session open the worker serialises nothing and touches no marker, and reruns
  // deferred recovery; the uploader drains the spool. Both are wanted before
  // Begin() -- a node configured and never activated drains a backlog this way.
  if (config.uploads_paused) {
    session->impl_->log->Info(
        "uploads paused: recording and spooling continue, nothing is sent until they are "
        "resumed or a process with uploads unpaused drains the spool");
  }
  session->impl_->StartWorker();
  return session;
}

int Session::RecoverOrphanedSessions() { return impl_->RecoverOrphansInternal(); }

bool Session::Begin() {
  std::lock_guard<std::mutex> lifecycle(impl_->lifecycle_mu);
  const double now = EpochSeconds();
  const std::string clock_problem = WallClockProblem(now, impl_->cfg.wall_clock_floor_epoch_s);
  if (!clock_problem.empty()) {
    impl_->log->Error("session not begun: " + clock_problem);
    return false;
  }

  std::string id;
  double timestamp = now;
  bool already_active = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    already_active = impl_->active;
  }
  // Logged with `mu` released, as every line is: a consumer's ILogger may call back into
  // this Session, and `mu` is not recursive.
  if (already_active) {
    impl_->log->Warn("Begin called while a session is already active");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    // `<unix seconds>_<device id>`, matching the Unity SDK, and the format is not
    // ours to change. But it is truncated to WHOLE seconds, so sessions begun in
    // the same second would share an id -- and the platform merges them, losing a
    // whole session. A task orchestrator ending one trial and starting the next
    // does it in milliseconds, so every id this Session issues is a second later
    // than the last one it issued. The timestamp then runs up to a few seconds
    // ahead of the wall clock, which is the cheaper error: it is a session label,
    // and durations come from the steady clock.
    std::int64_t second = static_cast<std::int64_t>(std::floor(timestamp));
    if (impl_->issued_any && second <= impl_->last_issued_second) {
      timestamp += static_cast<double>(impl_->last_issued_second + 1 - second);
      second = impl_->last_issued_second + 1;
    }
    impl_->last_issued_second = second;
    impl_->issued_any = true;
    id = std::to_string(second) + "_" + impl_->cfg.device_id;
  }

  // The marker is written before anything of the session is spooled: a chunk spooled
  // with no marker behind it belongs to a session no recovery would close. A first
  // write that fails is said by the spool and made again, durably, by the next
  // heartbeat; a crash in between leaves chunks that still upload and a session no
  // recovery closes. Until the session is installed below, nothing else in this
  // Session knows of it.
  {
    const SessionMarker marker =
        impl_->MarkerFor(id, timestamp, now, PartBase(Stream::kEvents));
    std::lock_guard<std::mutex> lock(impl_->marker_mu);
    impl_->spool->WriteMarker(marker);
    impl_->marker_id = id;
    impl_->last_marker_touch = now;
  }

  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    impl_->session_id = id;
    impl_->session_timestamp = timestamp;
    impl_->session_started_steady = SteadySeconds();
    impl_->active = true;
    impl_->last_auto_send = impl_->session_timestamp;
    impl_->current_backoff = 0.0;
    impl_->next_attempt_steady = 0.0;
    impl_->backoff_until_steady = 0.0;
    // NOT ForgetUnauthorizedAllowance() here, and it is worth saying why,
    // because it looks like an omission. That state belongs to `pump_mu`, which
    // Begin cannot take: the worker acquires `pump_mu` and only then `mu`, so a
    // Begin that holds `mu` and reaches for `pump_mu` inverts the order and
    // deadlocks. Touching the fields unguarded instead is a data race with a
    // worker that is already running on a second Begin. It costs nothing to
    // leave them: the allowance is per chunk, chunks outlive sessions in the
    // spool by design, and a stale id can only ever be overwritten because the
    // pump clears on every removal.

    // Part counters start fresh every session.
    impl_->part.clear();
    for (Stream s : {Stream::kGaze, Stream::kEvents, Stream::kSensors, Stream::kDynamics,
                     Stream::kCamera, Stream::kLidar}) {
      impl_->part[s] = PartBase(s);
    }
    // Milliseconds, from the same timestamp the session id embeds, so the binary
    // payloads name the same start the JSON streams do.
    impl_->session_start_ms =
        static_cast<std::uint64_t>(impl_->session_timestamp * 1000.0);
    // Anything still buffered belongs to the session that just ended. End()
    // flushes, so this is a defence rather than a routine clear -- but a frame
    // carried across a session boundary would be filed under the wrong session id
    // with a seq from the old one's counter.
    impl_->camera_frames.clear();
    impl_->camera_buffered_bytes = 0;
    impl_->camera_fov_warned.clear();
    impl_->lidar_scans.clear();
    impl_->lidar_buffered_bytes = 0;

    // Do not clear known_properties: anything the caller set before Begin is
    // legitimate session metadata. Re-mark the whole set as pending so a new
    // session resends everything rather than inheriting a stale delta.
    //
    // What IS cleared is anything set during the previous session, and End does
    // it by restoring this snapshot. Without that, `known_properties` accumulates
    // across every session a long-lived process ever runs, and because the Flush
    // at the bottom of Begin happens BEFORE the caller can apply this session's
    // identity or properties, session N+1's opening gaze payload goes out
    // carrying session N's c3d.sessionname and N's values. The platform keeps
    // the last write, so the stored session ends up correct -- but the opening
    // payload is wrong on the wire, and anything reading the timeline rather
    // than the final set sees one session's facts attributed to the next.
    impl_->presession_properties = impl_->known_properties;
    impl_->pending_properties = impl_->known_properties;
    impl_->SetPropertyLocked("c3d.app.engine", std::string(kAppEngine));
    impl_->SetPropertyLocked("c3d.app.sdktype", std::string(kSdkType));
    impl_->SetPropertyLocked("c3d.version", std::string(kSdkVersion));
    impl_->SetPropertyLocked("c3d.deviceid", impl_->cfg.device_id);
    impl_->SetPropertyLocked("c3d.app.name", impl_->cfg.app_name);
    // Load-bearing. Omitting this makes the platform accept every payload with a
    // 200 and then discard the whole session with no error anywhere. See the
    // note in config.hpp.
    impl_->SetPropertyLocked("c3d.app.version", impl_->cfg.app_version);

    EventRecord start;
    start.name = kSessionStart;
    start.t = impl_->session_timestamp;
    impl_->events.push_back(std::move(start));

    impl_->StartWorker();
  }
  impl_->log->Info("session " + id + " begun");
  Flush();
  return true;
}

void Session::End(const std::string& reason, std::optional<double> end_time) {
  std::lock_guard<std::mutex> lifecycle(impl_->lifecycle_mu);
  Impl::PendingBatch batch;
  std::string id;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (!impl_->active) return;

    EventRecord end;
    end.name = kSessionEnd;
    end.properties.emplace_back("Reason", reason);
    if (end_time) {
      end.t = std::max(*end_time, impl_->session_timestamp);
      end.properties.emplace_back("sessionlength", end.t - impl_->session_timestamp);
    } else {
      end.t = EpochSeconds();
      // Measured on the steady clock so an NTP step mid-session cannot produce a
      // negative or absurd session length.
      end.properties.emplace_back("sessionlength",
                                  SteadySeconds() - impl_->session_started_steady);
    }
    impl_->events.push_back(std::move(end));

    // The whole set rides the session's last gaze part as well as its first, so a session
    // whose opening part the spool cap evicted still delivers c3d.app.version.
    impl_->pending_properties = impl_->known_properties;
    batch = impl_->TakeDue(true);
    for (auto& part : batch.parts) part.carries_end = part.stream == Stream::kEvents;
    impl_->active = false;
    id = impl_->session_id;
  }

  // Once its end is in the spool the session is durable and nothing needs
  // recovering, so the marker goes at once -- SpoolBatch closes it as soon as the
  // part carrying the end is written, before any upload, and nothing here waits for
  // one. A session ends in the time it takes to write its last parts; they upload in
  // the background, or at the next start.
  const bool end_spooled = impl_->SpoolBatch(batch);
  {
    std::lock_guard<std::mutex> lock(impl_->marker_mu);
    impl_->marker_id.clear();
  }

  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    // An end that never reached the spool leaves the marker for recovery, which this
    // process runs again rather than leaving it to the next start.
    if (!end_spooled) {
      impl_->recovery_recheck_steady = SteadySeconds() + impl_->cfg.orphan_stale_after_s;
    }
    impl_->session_id.clear();
    impl_->session_timestamp = 0.0;
    impl_->sensor_gates.clear();
    impl_->part.clear();
    // Session-scoped properties end with the session; pre-session metadata does
    // not. The snapshot is taken in Begin, so this restores exactly what the
    // caller had established before the session opened.
    impl_->known_properties = impl_->presession_properties;
    impl_->presession_properties.clear();
  }
  if (!end_spooled) {
    impl_->log->Error("the end of session " + id +
                      " could not be spooled; its marker is left open for recovery to close");
  }
  impl_->log->Info("session " + id + " ended (" + reason + ")");
  impl_->cv.notify_all();
}

bool Session::active() const {
  std::lock_guard<std::mutex> lock(impl_->mu);
  return impl_->active;
}

std::string Session::session_id() const {
  std::lock_guard<std::mutex> lock(impl_->mu);
  return impl_->session_id;
}

double Session::session_timestamp() const {
  std::lock_guard<std::mutex> lock(impl_->mu);
  return impl_->session_timestamp;
}

void Session::SetProperty(const std::string& key, const PropertyValue& value) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  impl_->SetPropertyLocked(key, value);
}

void Session::SetParticipantId(const std::string& id) {
  std::string value = id;
  if (value.size() > kParticipantIdMax) {
    value = value.substr(0, kParticipantIdMax);
    // Back the cut off to a code-point boundary, and only when the cut actually SPLIT one. A
    // byte-exact cut can land inside a multi-byte sequence, which the JSON writer would send as
    // U+FFFD at the end of the id. A sequence that ends exactly at the cap is complete and is
    // kept.
    std::size_t lead_index = 0;
    bool found_lead = false;
    for (std::size_t back = 0; back <= 3 && back < value.size(); ++back) {
      const std::size_t index = value.size() - 1 - back;
      if ((static_cast<unsigned char>(value[index]) & 0xC0) == 0x80) continue;
      lead_index = index;
      found_lead = true;
      break;
    }
    if (found_lead) {
      const unsigned char lead = static_cast<unsigned char>(value[lead_index]);
      std::size_t declared = 0;
      if (lead >= 0xF0 && lead <= 0xF7) declared = 4;
      else if (lead >= 0xE0 && lead <= 0xEF) declared = 3;
      else if (lead >= 0xC0 && lead <= 0xDF) declared = 2;
      // declared == 0 is ASCII, or a byte no lead can be: either way nothing was split.
      if (declared > 0 && value.size() - lead_index < declared) value.erase(lead_index);
    }
    impl_->log->Error("participant id exceeds " + std::to_string(kParticipantIdMax) +
                      " characters and was truncated");
  }
  std::lock_guard<std::mutex> lock(impl_->mu);
  impl_->SetPropertyLocked("c3d.participant.id", value);
}

void Session::SetSessionName(const std::string& name) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  impl_->SetPropertyLocked("c3d.sessionname", name);
}

void Session::RecordGaze(double t, const Vec3& position, const Quat& rotation,
                         const Vec3& gaze_point) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  if (!impl_->active) return;
  if (impl_->gaze.size() >= impl_->cfg.max_records_per_batch) {
    // Drop-oldest for metric-shaped streams: recent poses are more useful than
    // the tail of a backlog.
    impl_->gaze.erase(impl_->gaze.begin());
    ++impl_->stats.records_dropped;
  }
  GazeRecord r;
  r.t = t;
  r.p = position;
  r.r = rotation;
  r.has_gaze = true;
  r.g = gaze_point;
  impl_->gaze.push_back(r);
  ++impl_->stats.records_recorded;
}

void Session::RecordGaze(double t, const Vec3& position, const Quat& rotation) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  if (!impl_->active) return;
  if (impl_->gaze.size() >= impl_->cfg.max_records_per_batch) {
    impl_->gaze.erase(impl_->gaze.begin());
    ++impl_->stats.records_dropped;
  }
  GazeRecord r;
  r.t = t;
  r.p = position;
  r.r = rotation;
  r.has_gaze = false;
  impl_->gaze.push_back(r);
  ++impl_->stats.records_recorded;
}

void Session::RecordEvent(const std::string& name, double t, const Vec3& point,
                          const Properties& properties, const std::string& dynamic_id) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  if (!impl_->active) return;
  if (impl_->events.size() >= impl_->cfg.max_records_per_batch) {
    // Drop-newest for events: an objective outcome or an E-stop recorded an
    // hour ago matters more than the one arriving into a full queue, and losing
    // the older one would silently rewrite history.
    ++impl_->stats.records_dropped;
    return;
  }
  EventRecord r;
  r.name = name;
  r.t = t;
  r.point = point;
  r.properties = properties;
  r.dynamic_id = dynamic_id;
  impl_->events.push_back(std::move(r));
  ++impl_->stats.records_recorded;
}

void Session::RecordSensor(const std::string& name, double t, double value) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  if (!impl_->active) return;

  // Change gating. A robot publishes battery state at 1 Hz whether or not it
  // changed; storing every identical sample is pure cost.
  auto& gate = impl_->sensor_gates[name];
  if (gate.primed) {
    const bool moved = std::abs(value - gate.last_value) > impl_->cfg.sensor_change_threshold;
    const bool keepalive = (t - gate.last_time) >= impl_->cfg.sensor_keepalive_s;
    if (!moved && !keepalive) return;
  }
  gate.primed = true;
  gate.last_value = value;
  gate.last_time = t;

  auto& series = impl_->sensors[name];
  if (series.name.empty()) series.name = name;
  if (series.samples.size() >= impl_->cfg.max_records_per_batch) {
    series.samples.erase(series.samples.begin());
    ++impl_->stats.records_dropped;
    if (impl_->sensor_sample_count) --impl_->sensor_sample_count;
  }
  series.samples.push_back(SensorSample{t, value});
  ++impl_->sensor_sample_count;
  ++impl_->stats.records_recorded;
}

void Session::RegisterDynamic(const std::string& id, const std::string& name,
                              const std::string& mesh, double t, const Vec3& position,
                              const Quat& rotation, const Vec3& scale) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  if (!impl_->active) return;
  ManifestEntry entry;
  entry.id = id;
  entry.name = name;
  entry.mesh = mesh;
  impl_->manifest_pending.push_back(std::move(entry));

  DynamicSnapshot snapshot;
  snapshot.id = id;
  snapshot.t = t;
  snapshot.p = position;
  snapshot.r = rotation;
  snapshot.scale = scale;
  // Lifecycle is expressed as a property rather than a separate message.
  snapshot.properties.emplace_back("enabled", true);
  impl_->dynamics.push_back(std::move(snapshot));
  ++impl_->stats.records_recorded;
}

void Session::RecordDynamic(const std::string& id, double t, const Vec3& position,
                            const Quat& rotation, const std::optional<Vec3>& scale,
                            const Properties& properties) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  if (!impl_->active) return;
  if (impl_->dynamics.size() >= impl_->cfg.max_records_per_batch) {
    impl_->dynamics.erase(impl_->dynamics.begin());
    ++impl_->stats.records_dropped;
  }
  DynamicSnapshot snapshot;
  snapshot.id = id;
  snapshot.t = t;
  snapshot.p = position;
  snapshot.r = rotation;
  snapshot.scale = scale;
  snapshot.properties = properties;
  impl_->dynamics.push_back(std::move(snapshot));
  ++impl_->stats.records_recorded;
}

void Session::RemoveDynamic(const std::string& id, double t, const Vec3& position,
                            const Quat& rotation) {
  Properties props;
  props.emplace_back("enabled", false);
  RecordDynamic(id, t, position, rotation, std::nullopt, props);
}

void Session::RecordCameraFrame(const std::string& camera_id, std::uint64_t time_ms,
                                std::uint32_t seq, const std::string& frame_id,
                                const Pose& pose, const std::string& format,
                                std::uint16_t width, std::uint16_t height,
                                const std::string& bytes, const CameraFov& fov) {
  const bool fov_known = IsKnownCameraFov(fov);
  // "Not recorded" is 0 by 0, the default, and is no fault; anything else refused is.
  const bool fov_refused = !fov_known && !(fov.hfov_deg == 0.0f && fov.vfov_deg == 0.0f);
  bool warn_fov = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (!impl_->active) return;
    if (bytes.empty()) {
      ++impl_->stats.records_dropped;
      return;
    }
    if (fov_refused) warn_fov = impl_->camera_fov_warned.insert(camera_id).second;
    if (impl_->camera_frames.empty()) impl_->camera_opened_at = EpochSeconds();
    CameraFrameRecord record;
    record.camera_id = camera_id;
    record.time_ms = time_ms;
    record.seq = seq;
    record.frame_id = frame_id;
    record.pose = pose;
    record.format = format;
    record.width = width;
    record.height = height;
    record.data = bytes;
    if (fov_known) {
      record.hfov_deg = fov.hfov_deg;
      record.vfov_deg = fov.vfov_deg;
    }
    impl_->camera_buffered_bytes += bytes.size();
    impl_->camera_frames.push_back(std::move(record));
    ++impl_->stats.camera_frames;
    impl_->stats.camera_bytes += bytes.size();
  }
  // Outside the lock: a host's logger is not this file's to hold `mu` across.
  if (warn_fov) {
    impl_->log->Warn("camera '" + camera_id + "' was given a field of view of " +
                     std::to_string(fov.hfov_deg) + " x " + std::to_string(fov.vfov_deg) +
                     " degrees, which is not two angles between 0 and 180; its frames are "
                     "recorded with none (0 x 0) until it is");
  }
}

void Session::RecordLaserScan(const std::string& sensor_id, std::uint64_t time_ms,
                              std::uint32_t seq, const std::string& frame_id,
                              const Pose& pose, const LaserScanData& scan) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  if (!impl_->active) return;
  if (scan.ranges.empty()) {
    ++impl_->stats.records_dropped;
    return;
  }
  if (impl_->lidar_scans.empty()) impl_->lidar_opened_at = EpochSeconds();
  LidarScanRecord record;
  record.sensor_id = sensor_id;
  record.time_ms = time_ms;
  record.seq = seq;
  record.frame_id = frame_id;
  record.pose = pose;
  record.is_pointcloud = false;
  record.scan = scan;
  const std::uint64_t size =
      static_cast<std::uint64_t>((scan.ranges.size() + scan.intensities.size()) * sizeof(float));
  impl_->lidar_buffered_bytes += size;
  impl_->lidar_scans.push_back(std::move(record));
  ++impl_->stats.lidar_scans;
  impl_->stats.lidar_bytes += size;
}

void Session::RecordPointCloud(const std::string& sensor_id, std::uint64_t time_ms,
                               std::uint32_t seq, const std::string& frame_id,
                               const Pose& pose, const EncodedPointCloud& cloud) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  if (!impl_->active) return;
  // A cloud the encoder refused has a reason and a counter at the caller. Sending
  // it anyway would put a scan with zero points on the wire, which reads as a
  // sensor that saw nothing rather than as a cloud that was never encoded.
  if (!cloud.ok || cloud.point_count == 0) {
    ++impl_->stats.records_dropped;
    return;
  }
  if (impl_->lidar_scans.empty()) impl_->lidar_opened_at = EpochSeconds();
  LidarScanRecord record;
  record.sensor_id = sensor_id;
  record.time_ms = time_ms;
  record.seq = seq;
  record.frame_id = frame_id;
  record.pose = pose;
  record.is_pointcloud = true;
  record.cloud = cloud;
  const std::uint64_t size = static_cast<std::uint64_t>(cloud.data.size());
  impl_->lidar_buffered_bytes += size;
  impl_->lidar_scans.push_back(std::move(record));
  ++impl_->stats.lidar_scans;
  impl_->stats.lidar_bytes += size;
}

void Session::Flush() {
  Impl::PendingBatch batch;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    batch = impl_->TakeDue(true);
    impl_->last_auto_send = EpochSeconds();
  }
  impl_->SpoolBatch(batch);
}

void Session::PumpUploads() { impl_->PumpUploadsInternal(); }

void Session::SetUploadsPaused(bool paused) {
  const bool was = impl_->uploads_paused.exchange(paused);
  if (was == paused) return;
  impl_->log->Info(paused ? "uploads paused: recording and spooling continue, nothing is sent"
                          : "uploads resumed: draining " +
                                std::to_string(impl_->spool->Count()) + " spooled chunk(s)");
}

bool Session::uploads_paused() const { return impl_->uploads_paused.load(); }

Stats Session::stats() const {
  Stats copy;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    copy = impl_->stats;
    copy.backing_off = impl_->BackingOffLocked();
  }
  copy.uploads_paused = impl_->uploads_paused.load();
  copy.spool_bytes = impl_->spool->TotalBytes();
  copy.spool_count = impl_->spool->Count();
  copy.chunks_evicted = impl_->spool->evicted();
  return copy;
}

}  // namespace c3d
