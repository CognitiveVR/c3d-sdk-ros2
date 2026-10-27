# Vendored Cap'n Proto schemas

These five files are **copies** of the schemas the Cognitive3D platform uses to read the camera,
lidar and pose bodies this SDK writes. Cognitive3D maintains the canonical copies. They are vendored
here because `core/` takes no dependency it could not ship to a non-ROS consumer, so a schema
is a file in this tree rather than a package.

`camera_payload.capnp`, `lidar_payload.capnp` and `pose_payload.capnp` are the three bodies.
The pose body is an envelope only: each block it carries is in the pose block format, which no
schema describes (`c3d/pose.hpp`). `common_payload.capnp` holds `Pose`, the one struct the camera
and lidar bodies both need: shared rather than duplicated because the generated C++ headers have
to be includable in a single translation unit, and two files each defining a global `Pose` is a
redefinition error there. `java.capnp` is the annotation schema the other four import, unchanged
from [capnproto-java](https://github.com/capnproto/capnproto-java).

Never edit a `.capnp` file here by hand. `tools/test_capnp_vendored.py` runs in CI and fails on
any byte that `CHECKSUMS` does not record. The platform reads what this SDK writes, so a field
added, renamed or renumbered on one side alone is a wire break that neither side's own tests can
see. A schema change starts in the canonical copy and reaches this directory through
`tools/sync_capnp.sh <schema-dir>`, which copies the five files in and rewrites `CHECKSUMS`;
`tools/sync_capnp.sh --check <schema-dir>` reports drift in either direction and exits non-zero.
If you need a schema change, open an issue describing it.
