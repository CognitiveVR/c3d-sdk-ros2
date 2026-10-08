@0xace75ba0f6e475c8;
using Java = import "java.capnp";
using Cxx = import "/capnp/c++.capnp";
using Common = import "common_payload.capnp";
$Java.package("cvr.common.models.capnp");
$Java.outerClassname("CameraSerialization");
$Cxx.namespace("cvr::capnp");

struct CameraPayload {
  userId         @0 :Text;      # deviceId, same as the JSON streams' "userid"
  sessionId      @1 :Text;      # "<epochSeconds>_<deviceId>"
  sceneId        @2 :Text;      # the scene UUID the session runs in
  sceneVersion   @3 :UInt32;    # the version NUMBER (?version=N); ingestion resolves the id
  formatVersion  @4 :Text;      # "1.0"
  sessionStartMs @5 :UInt64;    # wall clock, epoch ms
  sdkVersion     @6 :Text;      # the value that reaches the JSON streams as c3d.version
  part           @7 :UInt32;    # monotonic per stream from 1, like the JSON envelopes
  frames         @8 :List(Frame);
}

struct Frame {
  cameraId @0 :Text;            # the configured camera name, e.g. "front"
  time     @1 :UInt64;          # epoch ms, wall clock
  seq      @2 :UInt32;          # per-sensor monotonic counter from session start
  frameId  @3 :Text;            # the ROS frame the pose is for
  pose     @4 :Common.Pose;
  format   @5 :Text;            # "jpeg" ("jpegxl" reserved); lowercase, the gateway rejects unknown values
  width    @6 :UInt16;          # pixels; unsigned, but the Java binding reads it as short -- widen with & 0xFFFF
  height   @7 :UInt16;          # pixels; same narrowing as width
  data     @8 :Data;            # the encoded image bytes
  hfovDegrees @9 :Float32;      # horizontal field of view, full angle in degrees; 0 = not recorded
  vfovDegrees @10 :Float32;     # vertical field of view, full angle in degrees; 0 = not recorded
}
