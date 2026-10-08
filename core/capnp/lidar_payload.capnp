@0xb5602509f3dd9e1f;
using Java = import "java.capnp";
using Cxx = import "/capnp/c++.capnp";
using Common = import "common_payload.capnp";
$Java.package("cvr.common.models.capnp");
$Java.outerClassname("LidarSerialization");
$Cxx.namespace("cvr::capnp");

struct LidarPayload {
  # header fields @0..@7 are identical to CameraPayload's; Cap'n Proto has no inheritance
  userId         @0 :Text;      # deviceId, same as the JSON streams' "userid"
  sessionId      @1 :Text;      # "<epochSeconds>_<deviceId>"
  sceneId        @2 :Text;      # the scene UUID the session runs in
  sceneVersion   @3 :UInt32;    # the version NUMBER (?version=N); ingestion resolves the id
  formatVersion  @4 :Text;      # "1.0"
  sessionStartMs @5 :UInt64;    # wall clock, epoch ms
  sdkVersion     @6 :Text;      # the value that reaches the JSON streams as c3d.version
  part           @7 :UInt32;    # monotonic per stream from 1, like the JSON envelopes
  scans          @8 :List(Scan);
}

struct Scan {
  sensorId @0 :Text;            # the configured lidar name, e.g. "scan"
  time     @1 :UInt64;          # epoch ms, wall clock
  seq      @2 :UInt32;          # per-sensor monotonic counter from session start
  frameId  @3 :Text;            # the ROS frame the pose is for
  pose     @4 :Common.Pose;
  union {
    laserscan  @5 :LaserScan;
    pointcloud @6 :PointCloud;
  }
}

struct LaserScan {
  # angles, times and ranges are left in the sensor's own units and frame; only pose is scene
  angleMin       @0 :Float32;   # radians, as published
  angleMax       @1 :Float32;   # radians, as published
  angleIncrement @2 :Float32;   # radians between rays
  timeIncrement  @3 :Float32;   # seconds between rays
  scanTime       @4 :Float32;   # seconds between scans
  rangeMin       @5 :Float32;   # metres
  rangeMax       @6 :Float32;   # metres
  ranges         @7 :List(Float32);   # metres, as published; inf/nan preserved
  intensities    @8 :List(Float32);   # empty unless configured
}

struct PointCloud {
  pointCount      @0 :UInt32;   # points in the planar block
  position        @1 :Text;     # "int16_mm" | "float32"; lowercase, the gateway rejects unknown values
  color           @2 :Text;     # "none" | "rgb565" | "rgb332" | "rgb888"; lowercase, the gateway rejects unknown values
  intensity       @3 :Text;     # "none" | "uint8"; lowercase, the gateway rejects unknown values
  voxelLeafMm     @4 :UInt16;   # downsample leaf in mm; 0 = not downsampled. Java reads it as short -- widen with & 0xFFFF
  zstdLevel       @5 :UInt8;    # 0 = data is the raw planar block. Java reads it as byte -- widen with & 0xFF
  data            @6 :Data;     # planar block X[N] Y[N] Z[N] C[N] I[N], little-endian, zstd when zstdLevel > 0
}
