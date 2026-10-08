@0xee25719780260474;
using Java = import "java.capnp";
using Cxx = import "/capnp/c++.capnp";
$Java.package("cvr.common.models.capnp");
$Java.outerClassname("CommonSerialization");
$Cxx.namespace("cvr::capnp");

struct Pose {
  # sensor pose in the session's reference frame, pose_convention applied
  px @0 :Float32; py @1 :Float32; pz @2 :Float32;
  qx @3 :Float32; qy @4 :Float32; qz @5 :Float32;
  qw @6 :Float32 = 1;           # the identity rotation, so an unwritten pose reads as no rotation
                                # rather than as the zero quaternion; a sent pose never has qw == 0
                                # together with a zero vector
}
