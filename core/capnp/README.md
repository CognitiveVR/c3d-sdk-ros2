# Vendored Cap'n Proto schemas

These four files are **copies** of the schemas the Cognitive3D platform uses to read the camera
and lidar bodies this SDK writes. Cognitive3D maintains the canonical copies. They are vendored
here because `core/` takes no dependency it could not ship to a non-ROS consumer, so a schema
is a file in this tree rather than a package.

`camera_payload.capnp` and `lidar_payload.capnp` are the two bodies. `common_payload.capnp`
holds `Pose`, the one struct both of them need: shared rather than duplicated because the
generated C++ headers have to be includable in a single translation unit, and two files each
defining a global `Pose` is a redefinition error there. `java.capnp` is the annotation schema
the other three import, unchanged from [capnproto-java](https://github.com/capnproto/capnproto-java).

Never edit a `.capnp` file here by hand. `tools/test_capnp_vendored.py` runs in CI and fails on
any byte that `CHECKSUMS` does not record. The platform reads what this SDK writes, so a field
added, renamed or renumbered on one side alone is a wire break that neither side's own tests can
see. A schema change starts in the canonical copy and reaches this directory through
`tools/sync_capnp.sh <schema-dir>`, which copies the four files in and rewrites `CHECKSUMS`;
`tools/sync_capnp.sh --check <schema-dir>` reports drift in either direction and exits non-zero.
If you need a schema change, open an issue describing it.
