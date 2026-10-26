#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "c3d/types.hpp"

namespace c3d {

struct GazeRecord {
  double t = 0.0;
  Vec3 p;
  Quat r;
  bool has_gaze = false;
  Vec3 g;
};

struct EventRecord {
  std::string name;
  double t = 0.0;
  Vec3 point;
  Properties properties;
  std::string dynamic_id;
};

struct SensorSample {
  double t = 0.0;
  double value = 0.0;
};

struct SensorSeries {
  std::string name;
  double hz_limit = 0.0;  // 0 means unlimited; drives sensorHzLimit* fields
  std::vector<SensorSample> samples;
};

struct DynamicSnapshot {
  std::string id;
  double t = 0.0;
  Vec3 p;
  Quat r;
  std::optional<Vec3> scale;
  Properties properties;
};

struct ManifestEntry {
  std::string id;
  std::string name;
  std::string mesh;
  std::string file_type = "gltf";
};

// Everything the envelope needs that is not per-record.
struct EnvelopeContext {
  std::string session_id;
  std::string device_id;
  double session_timestamp = 0.0;
  std::uint64_t part = 0;
  std::string device_type = "ros2";
  double gaze_interval = 0.1;
};

// The `part` counter base of a JSON stream, as the Unity SDK's own source sends it: gaze 0,
// dynamics 0, events 1, sensors 1. The binary streams carry no part: each of their payloads is
// numbered by its own sensor's seq, so this answers nothing for them.
std::uint64_t PartBase(Stream stream);

// Sensors are the odd one out on two counts: the device id key is "name" rather
// than "userid", and formatversion is "2.0" where every other JSON stream sends
// "1.0". Both verified in Unity SharedCore.cs. The camera and lidar payloads are
// "2.0" as well, the version their schemas' header names.
const char* FormatVersion(Stream stream);

std::string SerializeGaze(const EnvelopeContext& ctx,
                          const std::vector<GazeRecord>& records,
                          const Properties& session_properties);
std::string SerializeEvents(const EnvelopeContext& ctx,
                            const std::vector<EventRecord>& records);
std::string SerializeSensors(const EnvelopeContext& ctx,
                             const std::vector<SensorSeries>& series);
std::string SerializeDynamics(const EnvelopeContext& ctx,
                              const std::vector<ManifestEntry>& manifest,
                              const std::vector<DynamicSnapshot>& snapshots);

// The JSON streams are scene-keyed: `/v0/<stream>/<sceneId>?version=<n>`. The two binary ones are
// SESSION-keyed -- `/v0/camera` with no scene id in the path at all -- because
// everything identifying the upload is read out of the Cap'n Proto body.
// `scene_id` and `scene_version` are ignored for those two rather than refused:
// one call site builds every chunk's url.
std::string BuildUrl(const std::string& protocol, const std::string& gateway,
                     const std::string& network_version, Stream stream,
                     const std::string& scene_id, const std::string& scene_version);

// What a chunk's recorded URL says that an upload needs back: the host it was recorded for,
// lowercased, and for a JSON stream the scene id and version. Either output may be null. False
// when `url` is not the shape BuildUrl writes. An upload never sends to the recorded URL itself;
// it rebuilds one from the current configuration.
bool ParseChunkUrl(const std::string& url, std::string* host, std::string* scene_id,
                   std::string* scene_version);

// Per stream, because the content type is. A JSON body announced as octet-stream is
// the kind of mismatch a gateway answers 400 to and a sender never sees.
std::vector<std::string> BuildHeaders(const std::string& application_key, Stream stream);

// A name for an application key that is not the key: the first 16 hex digits of a SHA-256
// over a fixed label and the key. Every chunk and marker records the one it was spooled under,
// and the spool must never hold the key itself.
std::string KeyFingerprint(const std::string& application_key);

}  // namespace c3d
