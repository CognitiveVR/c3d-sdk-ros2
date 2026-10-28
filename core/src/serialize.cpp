#include "serialize.hpp"

#include <nlohmann/json.hpp>

#include <cctype>
#include <cmath>
#include <sstream>

#include "json_dump.hpp"
#include "c3d/sha256.hpp"
#include "util.hpp"

namespace c3d {
namespace {

using json = nlohmann::ordered_json;

json V3(const Vec3& v) {
  return json::array({Round5(v.x), Round5(v.y), Round5(v.z)});
}

// Quaternion component order is [x, y, z, w], w last. Matches
// geometry_msgs/Quaternion field order, so no reordering is needed on ingest,
// only the handedness conversion documented in docs/coordinates.md.
json Q4(const Quat& q) {
  return json::array({Round5(q.x), Round5(q.y), Round5(q.z), Round5(q.w)});
}

void PutProperty(json& obj, const std::string& key, const PropertyValue& value) {
  switch (value.kind()) {
    case PropertyValue::Kind::kString: obj[key] = value.as_string(); break;
    case PropertyValue::Kind::kDouble: obj[key] = Round5(value.as_double()); break;
    case PropertyValue::Kind::kInt: obj[key] = value.as_int(); break;
    case PropertyValue::Kind::kBool: obj[key] = value.as_bool(); break;
  }
}

json PropertiesObject(const Properties& props) {
  json obj = json::object();
  for (const auto& kv : props) PutProperty(obj, kv.first, kv.second);
  return obj;
}

// Dynamic object snapshot properties are an ARRAY of single-key objects, while
// custom event properties are a plain OBJECT. That asymmetry is real in both
// reference implementations. Getting it wrong on dynamics does not error, it
// silently drops the properties.
json PropertiesArray(const Properties& props) {
  json arr = json::array();
  for (const auto& kv : props) {
    json one = json::object();
    PutProperty(one, kv.first, kv.second);
    arr.push_back(std::move(one));
  }
  return arr;
}

std::string HzLimitString(double hz) {
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(2);
  os << hz;
  return os.str();
}

}  // namespace

std::uint64_t PartBase(Stream stream) {
  // docs/data-format.md fixes the binary streams' part at 1, and the platform
  // stores it with every record, which is what makes a lost or duplicated part
  // visible from the stored data alone.
  if (IsBinaryStream(stream)) return 1;
  switch (stream) {
    case Stream::kGaze:
    case Stream::kDynamics:
      return 0;
    case Stream::kEvents:
    case Stream::kSensors:
      return 1;
    case Stream::kCamera:
    case Stream::kLidar:
    case Stream::kPose:
      return 1;
  }
  return 1;
}

const char* FormatVersion(Stream stream) {
  if (stream == Stream::kSensors) return "2.0";
  return "1.0";
}

std::string BuildUrl(const std::string& protocol, const std::string& gateway,
                     const std::string& network_version, Stream stream,
                     const std::string& scene_id, const std::string& scene_version) {
  std::ostringstream os;
  os << protocol << "://" << gateway << "/v" << network_version << "/" << StreamName(stream);
  if (!IsBinaryStream(stream)) os << "/" << scene_id << "?version=" << scene_version;
  return os.str();
}

bool ParseChunkUrl(const std::string& url, std::string* host, std::string* scene_id,
                   std::string* scene_version) {
  const std::size_t scheme_end = url.find("://");
  if (scheme_end == std::string::npos) return false;
  const std::size_t authority_start = scheme_end + 3;
  const std::size_t path_start = url.find_first_of("/?#", authority_start);
  if (path_start == std::string::npos || url[path_start] != '/') return false;
  std::string authority = url.substr(authority_start, path_start - authority_start);
  if (authority.empty() || authority.find('@') != std::string::npos) return false;
  for (auto& c : authority) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  const std::size_t query_start = url.find('?', path_start);
  const std::string path = url.substr(path_start, query_start == std::string::npos
                                                      ? std::string::npos
                                                      : query_start - path_start);
  std::vector<std::string> segments;
  std::size_t at = 1;
  while (at <= path.size()) {
    const std::size_t next = path.find('/', at);
    segments.push_back(path.substr(at, next == std::string::npos ? std::string::npos : next - at));
    if (next == std::string::npos) break;
    at = next + 1;
  }
  if (segments.size() < 2 || segments[0].size() < 2 || segments[0][0] != 'v') return false;

  std::string scene;
  std::string version;
  if (segments.size() >= 3) scene = segments[2];
  if (query_start != std::string::npos) {
    const std::string query = url.substr(query_start + 1);
    const std::size_t key = query.find("version=");
    if (key == 0 || (key != std::string::npos && query[key - 1] == '&')) {
      const std::size_t value_start = key + 8;
      version = query.substr(value_start, query.find('&', value_start) - value_start);
    }
  }
  if (host) *host = authority;
  if (scene_id) *scene_id = scene;
  if (scene_version) *scene_version = version;
  return true;
}

std::vector<std::string> BuildHeaders(const std::string& application_key, Stream stream) {
  if (IsBinaryStream(stream)) {
    // No X-HTTP-Method-Override. That header mirrors what Unity sends on the
    // routes Unity has, and these two are not among them.
    return {
        "Content-Type: application/octet-stream",
        "Authorization: APIKEY:DATA " + application_key,
    };
  }
  return {
      "Content-Type: application/json",
      "Authorization: APIKEY:DATA " + application_key,
      // Unity sends this on every data POST. Harmless if the gateway ignores it,
      // and it keeps us behaviourally identical to the reference implementation.
      "X-HTTP-Method-Override: POST",
  };
}

std::string KeyFingerprint(const std::string& application_key) {
  return Sha256Hex("c3d.spool.application_key\n" + application_key).substr(0, 16);
}

std::string SerializeGaze(const EnvelopeContext& ctx,
                          const std::vector<GazeRecord>& records,
                          const Properties& session_properties) {
  json root = json::object();
  json data = json::array();
  for (const auto& r : records) {
    json rec = json::object();
    rec["time"] = Round5(r.t);
    rec["p"] = V3(r.p);
    rec["r"] = Q4(r.r);
    if (r.has_gaze) rec["g"] = V3(r.g);
    data.push_back(std::move(rec));
  }
  root["data"] = std::move(data);
  root["userid"] = ctx.device_id;
  root["timestamp"] = static_cast<std::int64_t>(ctx.session_timestamp);
  root["sessionid"] = ctx.session_id;
  root["part"] = ctx.part;
  root["hmdtype"] = ctx.device_type;
  root["interval"] = Round5(ctx.gaze_interval);
  root["formatversion"] = FormatVersion(Stream::kGaze);
  // Session properties ride the gaze stream and only the gaze stream. There is
  // no /sessions or /participants endpoint. A payload with an empty data array
  // and a populated properties object is legal and is how metadata registers.
  if (!session_properties.empty()) {
    root["properties"] = PropertiesObject(session_properties);
  }
  return DumpJson(root);
}

std::string SerializeEvents(const EnvelopeContext& ctx,
                            const std::vector<EventRecord>& records) {
  json root = json::object();
  root["userid"] = ctx.device_id;
  // Events carry the full fractional session timestamp; gaze, sensors and
  // dynamics truncate it to whole seconds. Verified, not a typo.
  root["timestamp"] = Round5(ctx.session_timestamp);
  root["sessionid"] = ctx.session_id;
  root["part"] = ctx.part;
  root["formatversion"] = FormatVersion(Stream::kEvents);
  json data = json::array();
  for (const auto& r : records) {
    json rec = json::object();
    rec["name"] = r.name;
    rec["time"] = Round5(r.t);
    if (!r.dynamic_id.empty()) rec["dynamicId"] = r.dynamic_id;
    rec["point"] = V3(r.point);
    if (!r.properties.empty()) rec["properties"] = PropertiesObject(r.properties);
    data.push_back(std::move(rec));
  }
  root["data"] = std::move(data);
  return DumpJson(root);
}

std::string SerializeSensors(const EnvelopeContext& ctx,
                             const std::vector<SensorSeries>& series) {
  json root = json::object();
  // "name", not "userid". Sensors are the only stream that does this.
  root["name"] = ctx.device_id;
  root["sessionid"] = ctx.session_id;
  root["timestamp"] = static_cast<std::int64_t>(ctx.session_timestamp);
  root["part"] = ctx.part;
  root["formatversion"] = FormatVersion(Stream::kSensors);
  json data = json::array();
  for (const auto& s : series) {
    json entry = json::object();
    entry["name"] = s.name;
    if (s.hz_limit > 0.0 && s.hz_limit <= 10.0) {
      // Both of these are strings on the wire, including the boolean. Verified.
      entry["sensorHzLimitType"] = HzLimitString(s.hz_limit);
      entry["sensorHzLimited"] = "true";
    }
    json points = json::array();
    for (const auto& sample : s.samples) {
      points.push_back(json::array({Round5(sample.t), Round5(sample.value)}));
    }
    entry["data"] = std::move(points);
    data.push_back(std::move(entry));
  }
  root["data"] = std::move(data);
  return DumpJson(root);
}

std::string SerializeDynamics(const EnvelopeContext& ctx,
                              const std::vector<ManifestEntry>& manifest,
                              const std::vector<DynamicSnapshot>& snapshots) {
  json root = json::object();
  root["userid"] = ctx.device_id;
  root["timestamp"] = static_cast<std::int64_t>(ctx.session_timestamp);
  root["sessionid"] = ctx.session_id;
  root["part"] = ctx.part;
  root["formatversion"] = FormatVersion(Stream::kDynamics);
  if (!manifest.empty()) {
    // The manifest is a map keyed by object id, while data is an array.
    json m = json::object();
    for (const auto& e : manifest) {
      json entry = json::object();
      entry["name"] = e.name;
      entry["mesh"] = e.mesh;
      entry["fileType"] = e.file_type;
      m[e.id] = std::move(entry);
    }
    root["manifest"] = std::move(m);
  }
  if (!snapshots.empty()) {
    json data = json::array();
    for (const auto& s : snapshots) {
      json rec = json::object();
      rec["id"] = s.id;
      rec["time"] = Round5(s.t);
      rec["p"] = V3(s.p);
      rec["r"] = Q4(s.r);
      if (s.scale.has_value()) rec["s"] = V3(*s.scale);
      if (!s.properties.empty()) rec["properties"] = PropertiesArray(s.properties);
      data.push_back(std::move(rec));
    }
    root["data"] = std::move(data);
  }
  return DumpJson(root);
}

}  // namespace c3d
