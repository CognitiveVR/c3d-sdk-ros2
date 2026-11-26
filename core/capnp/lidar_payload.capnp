@0xb5602509f3dd9e1f;
using Java = import "java.capnp";
using Cxx = import "/capnp/c++.capnp";
using Common = import "common_payload.capnp";
$Java.package("cvr.common.models.capnp");
$Java.outerClassname("LidarSerialization");
$Cxx.namespace("cvr::capnp");

struct LidarPayload {
  # one uploaded object: one sensor, 3 s of scans, or clouds until ~2 MB or 5 s, whichever comes first
  userId          @0  :Text;      # deviceId, same as the JSON streams' "userid"
  sessionId       @1  :Text;      # "<epochSeconds>_<deviceId>"
  sceneId         @2  :Text;      # the scene UUID the session runs in
  sceneVersion    @3  :UInt32;    # the version NUMBER (?version=N); ingestion resolves the id
  formatVersion   @4  :Text;      # "2.0"
  sessionStartMs  @5  :UInt64;    # wall clock, epoch ms
  sdkVersion      @6  :Text;      # the value that reaches the JSON streams as c3d.version
  sensorId        @7  :Text;      # the configured lidar name
  seq             @8  :UInt32;    # block index per sensor from 0
  startMs         @9  :UInt64;
  endMs           @10 :UInt64;
  frameId         @11 :Text;      # ROS frame the poses are for, constant per block
  union {
    scans  @12 :ScanBlock;
    clouds @13 :CloudBlock;
  }
}

struct ScanBlock {
  # the LaserScan geometry, constant across the block; a change in any of these starts a new block.
  # Timing is per scan (ScanEntry): RPLIDAR drivers change scanTime and timeIncrement on every sweep
  angleMin       @0  :Float32;
  angleMax       @1  :Float32;
  angleIncrement @2  :Float32;
  rangeMin       @3  :Float32;
  rangeMax       @4  :Float32;
  rays           @5  :UInt16;
  rangeUnit      @6  :RangeUnit;      # millimetre when rangeMax <= 65 m, else centimetre; always written
  encoding       @7  :ScanEncoding;
  entries        @8  :List(ScanEntry); # one per scan, in time order
  ranges         @9  :Data;            # entries.size x rays uint16 codes, laid out per `encoding`
  intensities    @10 :Data;            # empty unless configured; same layout as ranges, uint16
}

enum RangeUnit { millimetre @0; centimetre @1; }

enum ScanEncoding {
  uint16 @0;            # scan-major, little-endian, no transform; sentinels 0xFFFF = +inf (no return), 0xFFFE = NaN, 0x0000 = -inf
  deltaShuffleZstd @1;  # per scan, each code minus the previous ray's (uint16 wrap); then all low bytes, then all high bytes; then zstd
}

struct ScanEntry {
  time @0 :UInt64;      # epoch ms
  seq  @1 :UInt32;      # per-sensor counter from session start
  pose @2 :Common.Pose;
  timeIncrement @3 :Float32;   # seconds between rays, this scan's own
  scanTime      @4 :Float32;   # seconds this sweep took
}

struct CloudBlock {
  # the cloud fields, constant across the block
  position     @0 :Text;      # "int16_mm" | "float32"
  color        @1 :Text;      # "none" | "rgb565" | "rgb332" | "rgb888"
  intensity    @2 :Text;      # "none" | "uint8"
  voxelLeafMm  @3 :UInt16;
  encoding     @4 :CloudEncoding;
  zstdLevel    @5 :UInt8;     # 3 recommended; 0 = raw planar blocks
  entries      @6 :List(CloudEntry);
  data         @7 :Data;      # the per-cloud compressed blocks back to back; each decodes on its own
}

enum CloudEncoding {
  planarZstd @0;              # X[N] Y[N] Z[N] C[N] I[N] little-endian, zstd
  planarDeltaShuffleZstd @1;  # each position plane as differences between consecutive points (int16 wrap), low bytes then high bytes; colour and intensity planes unchanged; then zstd
}

struct CloudEntry {
  time       @0 :UInt64;
  seq        @1 :UInt32;
  pose       @2 :Common.Pose;
  pointCount @3 :UInt32;
  byteOffset @4 :UInt32;      # this cloud's compressed block inside data
  byteLength @5 :UInt32;
}
