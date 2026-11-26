@0xace75ba0f6e475c8;
using Java = import "java.capnp";
using Cxx = import "/capnp/c++.capnp";
using Common = import "common_payload.capnp";
$Java.package("cvr.common.models.capnp");
$Java.outerClassname("CameraSerialization");
$Cxx.namespace("cvr::capnp");

struct CameraPayload {
  # one uploaded object: one camera, one closed GOP (3 s by default; a config change ends a chunk early)
  userId          @0  :Text;      # deviceId, same as the JSON streams' "userid"
  sessionId       @1  :Text;      # "<epochSeconds>_<deviceId>"
  sceneId         @2  :Text;      # the scene UUID the session runs in
  sceneVersion    @3  :UInt32;    # the version NUMBER (?version=N); ingestion resolves the id
  formatVersion   @4  :Text;      # "2.0"
  sessionStartMs  @5  :UInt64;    # wall clock, epoch ms
  sdkVersion      @6  :Text;      # the value that reaches the JSON streams as c3d.version
  cameraId        @7  :Text;      # the configured camera name
  seq             @8  :UInt32;    # chunk index per camera from 0; also in the object key
  startMs         @9  :UInt64;    # epoch ms of the first frame
  endMs           @10 :UInt64;    # epoch ms of the last frame
  codec           @11 :Text;      # "h264" (Annex-B, closed GOP, IDR first); "h265" reserved
  codecParams     @12 :Data;      # SPS/PPS as an avcC record, for the codec string and profile; the stream itself is Annex-B with in-band parameter sets
  width           @13 :UInt16;
  height          @14 :UInt16;
  nominalFps      @15 :Float32;
  frameId         @16 :Text;      # ROS frame the poses are for, constant per chunk
  hfovDegrees     @17 :Float32;   # 0 = not recorded
  vfovDegrees     @18 :Float32;
  encoderSettings @19 :Text;      # diagnostic: "x264 ultrafast crf=28 threads=1 gop=3s"
  frames          @20 :List(FrameMeta);   # one per frame, in decode order
  data            @21 :Data;      # the bitstream, every frame's access unit back to back
}

struct FrameMeta {
  time       @0 :UInt64;          # epoch ms, wall clock
  seq        @1 :UInt32;          # per-camera counter from session start
  pose       @2 :Common.Pose;
  byteOffset @3 :UInt32;          # this frame's access unit inside data
  byteLength @4 :UInt32;
  keyframe   @5 :Bool;            # true for the first frame of every chunk
}
