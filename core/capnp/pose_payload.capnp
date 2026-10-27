@0xe3de72e0639dc503;
using Java = import "java.capnp";
using Cxx = import "/capnp/c++.capnp";
$Java.package("cvr.common.models.capnp");
$Java.outerClassname("PoseSerialization");
$Cxx.namespace("cvr::capnp");

# Envelope for the pose stream (POST /v0/pose). Cap'n Proto carries only the envelope: each block's
# bytes are the pose block format of c3d-stream-formatter docs/pose/spec.md, which nothing on the
# ingest path decodes. Generated Java ships to se-upload and to the Unity SDK's Android plugin, so a
# change adds fields with the next number only -- never renumber, retype or delete a field. A field
# added after SDK builds have shipped reads as 0 / "" / false on uploads from those builds, so its
# zero value must mean "not sent" and must never be rejected. (The fields below were all present in
# the first shipped version; seq = 0 really is the first block.)

struct PosePayload {
  # header fields @0..@7 are identical to CameraPayload's and LidarPayload's; Cap'n Proto has no inheritance
  userId         @0 :Text;      # deviceId, same as the JSON streams' "userid"
  sessionId      @1 :Text;      # "<epochSeconds>_<deviceId>"
  sceneId        @2 :Text;      # the scene UUID the session runs in
  sceneVersion   @3 :UInt32;    # the version NUMBER (?version=N); se-upload resolves the id
  formatVersion  @4 :Text;      # "1.0"; the envelope's version, not the block's (that is byte 1 of each block)
  sessionStartMs @5 :UInt64;    # wall clock, epoch ms
  sdkVersion     @6 :Text;      # the value that reaches the JSON streams as c3d.version
  part           @7 :UInt32;    # monotonic per stream from 1, like the JSON envelopes
  manifests      @8 :List(ObjectManifest);   # only the objects whose manifest is sent in this part
  blocks         @9 :List(Block);
  startUs        @10 :Int64;    # the earliest Block.startUs in this part; 0 = not sent (spec §10.2)
  endUs          @11 :Int64;    # the latest sample time in this part, inclusive (a block's last sample, not
                                # its window end); 0 = not sent. Both 0 or both set: the gateway rejects a
                                # part with only one set, either set on a part with no blocks, a startUs
                                # other than the earliest block start, and an endUs before any block start
}

struct ObjectManifest {
  # today's free-form manifest entry, field for field; an unset Text reads as ""
  manifestId         @0 :Text;  # the object's GUID, the key its blocks carry
  name               @1 :Text;  # required
  mesh               @2 :Text;  # "" = no model (nothing to draw from this stream); both or neither
  fileType           @3 :Text;  # "" = no model; set exactly when mesh is set
  riggingMappingType @4 :Text;  # "jointID" | "meshName"; "" = not sent
  joints             @5 :List(Joint);   # empty when root is set
  root               @6 :Text;  # "" = none. Set = this entry is a FOLLOWER: drawn from the blocks and
                                # joints of the object it names (e.g. a clothing mesh on an avatar's
                                # armature), with no joints and no blocks of its own. The gateway rejects
                                # a follower that lists joints, a block carrying a follower's manifestId
                                # in the same part, and a root that names itself or another follower when
                                # both entries are in the part. An object is a follower or an owner for
                                # the whole session, never both: the gateway rejects a part listing one
                                # manifestId both ways, and cannot check across parts. Not the root JOINT (j = 0)
  properties         @7 :List(Property);  # optional, about the object (e.g. c3d.profile, hand.side, ros.robotName)
  firstSeq           @8 :UInt32;  # the seq of the first block that uses this send, counted in the object's
                                  # own seq, or in its root object's seq for a follower; 0 = from the first
                                  # block of the session. Each send is stored beside the others, keyed
                                  # (firstSeq, part): a block decodes with the send of the largest
                                  # (firstSeq, part) whose firstSeq <= its seq. Two sends with no block
                                  # between them share a firstSeq and the later part wins, so the gateway
                                  # rejects a part that lists one (manifestId, firstSeq) twice; two sends
                                  # of one object with blocks between them may share a part (the gateway
                                  # accepts it; SDKs send each in a part of its own, spec §10.1 rule 6).
                                  # Java reads it as int
}

struct Joint {
  j          @0 :UInt32;        # 0 is the root joint; indices run 0..J-1 with no gaps. Java reads it as int
  meshName   @1 :Text;          # node / skin joint name
  properties @2 :List(Property);  # optional, about the joint (e.g. ros.parent, ros.jointType, c3d.kind)
}

struct Property {
  # One free-form fact, like a custom event property. The schema fixes the container only: a key
  # and a value of one of three types, stored as the props_str / props_num / props_bool maps. Which
  # keys exist is a convention documented in the spec ("Manifest properties"): c3d.* is reserved, a
  # key's meaning never changes once published. The gateway checks shape only, and rejects the part
  # otherwise: <= 64 properties per entry; key 1..64 UTF-8 bytes and unique within the entry; a value
  # is set (the union tag defaults to str, so a property whose writer set only the key reads as str
  # with hasStr() false: never stored as ""); str <= 1,024 UTF-8 bytes; num finite and |num| <= 3.4028235e38
  # (stored as a 32-bit float). Nothing on the backend interprets a key.
  key @0 :Text;
  union {
    str  @1 :Text;
    num  @2 :Float64;
    bool @3 :Bool;
  }
}

struct Block {
  # one object's block. startUs and sampleCount copy the block header so the ingest path can key the
  # row without decompressing; the decoder checks both against the header and rejects a mismatch
  manifestId  @0 :Text;         # which object; must not be ""
  startUs     @1 :Int64;        # header start: epoch microseconds of the block's first sample
  sampleCount @2 :UInt32;       # header N, >= 1. Java reads it as int
  resetBefore @3 :Bool;         # a manifest was sent for this object right before this block: the
                                # decoder clears its carry-over state and enabled before reading it.
                                # When this part's manifests list the object and the part carries its
                                # block with seq == that manifest's firstSeq, that block has
                                # resetBefore = true; the gateway rejects the part otherwise. Other
                                # blocks are not checked: the block a re-send closed was built under
                                # the previous send, so it need not have resetBefore = true (it has it
                                # only if it was also the first block after that previous send).
                                # Follower entries are exempt: their sends do not touch the root's
                                # blocks. The reverse is not required: resetBefore = true with no
                                # manifest in this part is valid, the manifest may have gone earlier
  data        @4 :Data;         # 4-byte prefix + compressed body, exactly as the encoder produced it;
                                # at most 983,616 bytes (spec rule 1: split a larger block in two)
  seq         @5 :UInt32;       # per object: 0 at its first block of the session, +1 per block, never
                                # restarts. A gap tells the decoder a block was lost, as opposed to a
                                # pause in sampling, which only leaves a gap in time. Java reads it as int
}
