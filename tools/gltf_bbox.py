#!/usr/bin/env python3
"""Report a glTF's extents, orientation and upload readiness, before it is uploaded.

  ros2 run cognitive3d_ros gltf_bbox.py [--scene | --object] <file.gltf|file.glb>...

The bounding box is computed THROUGH the node hierarchy: accessor min/max are taken before any
node transform, so a raw bounding box can report a correctly repaired scene as still lying on its
side.

glTF is Y-up by definition, so the up-axis repair is checkable: for a world whose HEIGHT is its
smallest extent -- a room, a warehouse floor -- an upright export has its shortest extent on Y.

That assumption is the limit of this check. A corridor 100 m long, 10 m deep and 20 m tall is
upright with Y NOT shortest, and no rule on the extents clears it without also clearing a
warehouse whose repair did not take. So a non-Y shortest axis is reported as SOMETHING TO LOOK AT
with both explanations named, never as a verdict of "on its side". A near-cubic world is
inconclusive.
"""
import json, struct, sys, pathlib, urllib.parse

# Below this ratio between the largest and smallest extent, "which axis is
# shortest" is not evidence about orientation.
DECISIVE_RATIO = 1.5


GLB_MAGIC = b"glTF"


def load(path):
    """The glTF JSON, from either a .glb container or a plain .gltf.

    Only the JSON is ever read -- bounds come from accessor `min`/`max`, never from the buffer --
    so a `.gltf` needs no special handling beyond not unwrapping a container it does not have.
    Sniffed by magic rather than by extension, because the extension is the thing most likely to
    be wrong on a file someone just converted.
    """
    data = pathlib.Path(path).read_bytes()
    if not data[:4] == GLB_MAGIC:
        try:
            return json.loads(data)
        except json.JSONDecodeError as error:
            raise SystemExit(f"{path}: neither a .glb container nor glTF JSON ({error})")
    off = 12
    # `off < len(data)` is not enough: a chunk header is 8 bytes, and a .glb truncated after its
    # 12-byte header (a disk-full export, a killed converter, a partial copy) then raises
    # struct.error instead of refusing. A traceback is not a verdict.
    while off + 8 <= len(data):
        clen, ctype = struct.unpack_from("<II", data, off)
        payload = data[off + 8: off + 8 + clen]
        if ctype == 0x4E4F534A:
            # The header can fit while its declared payload does not -- the same truncation, one
            # step later -- and a short slice is invalid JSON rather than a short read.
            if len(payload) < clen:
                raise SystemExit(
                    "%s: .glb JSON chunk declares %d bytes and only %d are present; the file is "
                    "truncated" % (path, clen, len(payload)))
            try:
                return json.loads(payload)
            except json.JSONDecodeError as error:
                raise SystemExit("%s: .glb JSON chunk is not valid JSON (%s)" % (path, error))
        off += 8 + clen + ((4 - clen % 4) % 4)
    raise SystemExit(f"{path}: .glb container with no JSON chunk")


def mat_of(node):
    if "matrix" in node:
        m = node["matrix"]           # column-major
        return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
    M = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    if "rotation" in node:
        x, y, z, w = node["rotation"]
        M = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), 0],
             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), 0],
             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), 0],
             [0, 0, 0, 1]]
    if "scale" in node:
        s = node["scale"]
        for r in range(3):
            for c in range(3):
                M[r][c] *= s[c]
    if "translation" in node:
        t = node["translation"]
        for r in range(3):
            M[r][3] += t[r]
    return M


def mul(A, B):
    return [[sum(A[r][k] * B[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def apply(M, p):
    return [sum(M[r][c] * (p[c] if c < 3 else 1.0) for c in range(4)) for r in range(3)]


# The Z-up correction can sit on a node's matrix or be baked into the vertices -- one toolchain
# does the first for a world and the second for a single mesh -- so the walk applies every node's
# transform rather than assuming either.
def bbox(path):
    g = load(path)
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    ident = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]

    def walk(ni, parent):
        node = g["nodes"][ni]
        M = mul(parent, mat_of(node))
        if "mesh" in node:
            for prim in g["meshes"][node["mesh"]].get("primitives", []):
                ai = prim.get("attributes", {}).get("POSITION")
                if ai is None:
                    continue
                acc = g["accessors"][ai]
                if "min" not in acc:
                    continue
                mn, mx = acc["min"], acc["max"]
                # every corner of the local AABB, through the accumulated transform
                for bx in (mn[0], mx[0]):
                    for by in (mn[1], mx[1]):
                        for bz in (mn[2], mx[2]):
                            w = apply(M, [bx, by, bz])
                            for k in range(3):
                                lo[k] = min(lo[k], w[k])
                                hi[k] = max(hi[k], w[k])
        for ch in node.get("children", []):
            walk(ch, M)

    for root in g.get("scenes", [{}])[0].get("nodes", []):
        walk(root, ident)
    if lo[0] == float("inf"):
        raise SystemExit(f"{path}: no positioned mesh geometry found")
    return lo, hi


UP_AXIS = 1          # glTF is Y-up by definition


def orientation(ext, decisive_ratio=DECISIVE_RATIO):
    """(verdict, lines) for these extents. Assumes the world's height is its smallest extent."""
    smallest, largest = min(ext), max(ext)
    ratio = largest / smallest if smallest > 0 else float("inf")
    if ratio < decisive_ratio:
        return "INCONCLUSIVE", [
            "extents are within %.2fx of each other, so the shortest axis says" % ratio,
            "nothing about orientation. Check by eye."]
    # `ext.index(smallest) == UP_AXIS` would be wrong on a tie: extents [10, 10, 30] have Y tied
    # for shortest, and index() returns X. A square floor plan ties within float error routinely.
    if ext[UP_AXIS] == smallest:
        return "consistent with UPRIGHT", [
            "Y is the shortest extent, as an upright world with a floor plan is."]
    return "SOMETHING TO LOOK AT", [
        "Y is not the shortest extent. Two things produce that and this check",
        "cannot separate them: the COLLADA up_axis repair did not take, OR your",
        "world is taller than it is narrow (a corridor, a tower), for which a",
        "shortest-axis test says nothing at all. Open it and look."]


def external_files(spec, base):
    """The files a `.gltf` REFERENCES and cannot be uploaded without, as (uri, path) pairs.

    A `.glb` embeds its buffers, so this is empty for one. A `.gltf` does not: geometry lives in a
    sibling `.bin`, and the platform accepts the pair and rejects a lone file. `assimp export`
    writes them in two steps, so a failure between the two leaves a valid-looking `.gltf` with no
    geometry behind it -- and bounds come from accessor min/max, so even the orientation check
    passes. That artefact then uploads with a 200 and is discarded.
    """
    referenced = []
    for key in ("buffers", "images"):
        for entry in spec.get(key) or []:
            uri = entry.get("uri")
            if not uri or uri.startswith("data:"):
                continue
            declared = entry.get("byteLength") if key == "buffers" else None
            referenced.append((uri, base / urllib.parse.unquote(uri), declared))
    return referenced


SCENE_BUFFER = "scene.bin"


def unresolvable_references(spec, gltf_path, is_container, kind):
    """What the VIEWER will fail to fetch, as (what, why) pairs.

    The two upload paths are addressed differently, and one rule for both is wrong for one of them:

      SCENE   the upload renames the glTF and its buffer to the literals `scene.gltf` and
              `scene.bin`, and does not rewrite the glTF body, so `buffers[].uri` has to say
              `scene.bin` ALREADY. A pair named after the world -- what `export_world_gltf.sh`
              writes -- references a file stored under a different name, which 404s.

      OBJECT  the viewer builds the glTF's URL from the MESH NAME, `objects/{mesh}/{mesh}.gltf`,
              and resolves `buffers[].uri` RELATIVE TO THAT URL. So any bare filename works,
              provided that file was uploaded beside the glTF -- which completeness answers.

    IMAGES follow the object rule under BOTH kinds: the upload renames only the glTF and the
    buffer, so a texture keeps its own name and is fetched relative to whichever glTF referenced
    it. What breaks either kind is a uri carrying a PATH: the upload stores basenames only, so
    `textures/wall.png` resolves on your disk, satisfies every other check here, and 404s on the
    platform -- as an untextured model rather than an empty one, which is why it is easy to miss.
    """
    unresolvable = []

    def check(uri, expected_literal):
        if uri is None:
            return  # genuinely embedded: a .glb body, or a bufferView-backed image
        decoded = urllib.parse.unquote(uri)
        if decoded.startswith("data:"):
            return
        if decoded == "":
            unresolvable.append(("a uri is the empty string",
                                 "only an ABSENT uri means embedded; an empty one is fetched, and "
                                 "resolves to the page rather than to an asset"))
            return
        # A single leading `./` is dropped by URL parsing before the request is made, so it
        # resolves exactly as the bare name would. Refusing it would be a false refusal.
        if decoded.startswith("./"):
            decoded = decoded[2:]
        if expected_literal is not None:
            if decoded != expected_literal:
                unresolvable.append((
                    "buffers[].uri is %r" % uri,
                    "a scene's buffer is stored as %r whatever you call it, and this reference is "
                    "not rewritten" % expected_literal))
            return
        if "/" in decoded or "\\" in decoded:
            unresolvable.append((
                "a uri is %r" % uri,
                "the upload stores basenames only, so this resolves here and 404s on the "
                "platform; flatten it beside the .gltf"))

    if not is_container:
        for entry in spec.get("buffers") or []:
            check(entry.get("uri"), SCENE_BUFFER if kind == "scene" else None)
        # Images are addressed relative to their glTF under BOTH kinds -- the upload renames only
        # the glTF and the buffer -- so the bare-filename rule is theirs too.
        for entry in spec.get("images") or []:
            check(entry.get("uri"), None)
    return unresolvable


def shortfall(path, declared):
    """Why this referenced file is not usable, or None when it is.

    A buffer states its own `byteLength`, so "the file exists and is not empty" is not the same
    question as "the file is all there". A `.bin` truncated by a full disk still satisfies the
    weaker test, and bounds come from accessor min/max rather than from buffer contents, so every
    other check in this tool passes over it and the upload is accepted and discarded.
    """
    if not path.exists():
        return "missing"
    size = path.stat().st_size
    if size == 0:
        return "empty"
    if declared is None:
        return None
    # A malformed glTF can declare anything. Comparing a size against a string raises, which is
    # the traceback-instead-of-a-verdict this tool exists to avoid.
    if not isinstance(declared, int) or isinstance(declared, bool) or declared < 0:
        return "declares a byteLength of %r, which is not a length" % (declared,)
    if size < declared:
        return "truncated: declares %d bytes, has %d" % (declared, size)
    return None


def report(path, kind="scene"):
    """Print the report. Returns False when the asset is not fit to upload as `kind`."""
    spec = load(path)
    missing = []
    for uri, referenced, declared in external_files(spec, pathlib.Path(path).parent):
        why = shortfall(referenced, declared)
        if why is not None:
            missing.append((uri, why))
    # Only meaningful for a `.gltf`: a `.glb` embeds its buffer and names nothing.
    is_container = pathlib.Path(path).read_bytes()[:4] == GLB_MAGIC
    unresolvable = unresolvable_references(spec, path, is_container, kind)
    lo, hi = bbox(path)
    ext = [hi[k] - lo[k] for k in range(3)]
    print(pathlib.Path(path).name)
    for k, n in enumerate("XYZ"):
        print("   %s extent %8.3f   (%9.3f .. %9.3f)" % (n, ext[k], lo[k], hi[k]))
    print("   shortest = %s" % "XYZ"[ext.index(min(ext))])
    verdict, lines = orientation(ext)
    print("   %s" % verdict)
    for line in lines:
        print("      %s" % line)
    if missing:
        print("   INCOMPLETE -- this file references %d thing(s) it cannot be uploaded without:"
              % len(missing))
        for uri, why in missing:
            print("      %s (%s)" % (uri, why))
        print("      Upload needs the .gltf AND its .bin together. A lone .gltf is accepted with")
        print("      HTTP 200 and discarded, so re-export before uploading.")
    if unresolvable:
        print("   WILL UPLOAD EMPTY -- the viewer cannot fetch what this file points at:")
        for what, why in unresolvable:
            print("      %s" % what)
            print("        %s" % why)
        print("      Every other signal reads healthy -- 2xx on upload, textures serve, the scene")
        print("      type is set -- and the result is empty.")
    print()
    return not missing and not unresolvable


if __name__ == "__main__":
    # Every path is reported before the exit code is decided, INCLUDING one that refuses. Two
    # things would silence the remaining arguments: `all(...)` over a generator stopping at the
    # first failure, and a `SystemExit` from load() propagating out of the loop. A tool whose
    # whole job is to say what is wrong must not go quiet the moment it finds one thing.
    arguments = list(sys.argv[1:])
    # A scene and a dynamic object are stored by different rules and cannot be told apart from the
    # file, so the caller says which. Scene is the default: it is the first upload any integration
    # does, and the one whose rename is destructive.
    kind = "object" if "--object" in arguments else "scene"
    arguments = [a for a in arguments if a not in ("--object", "--scene")]
    usage = ("usage: gltf_bbox.py [--scene | --object] <file.gltf|file.glb>...\n"
             "Reports each file's extents, orientation and upload readiness, and exits 1 when any "
             "file is not fit to upload as a scene (the default) or as a dynamic object.")
    if any(a in ("-h", "--help") for a in arguments):
        print(usage)
        sys.exit(0)
    # With nothing to check every check passes vacuously, and exit 0 would read as a verdict.
    if not arguments:
        print(usage, file=sys.stderr)
        sys.exit(2)
    results = []
    for path in arguments:
        try:
            results.append(report(path, kind))
        except SystemExit as refusal:
            print("%s\n" % refusal)
            results.append(False)
    sys.exit(0 if all(results) else 1)
