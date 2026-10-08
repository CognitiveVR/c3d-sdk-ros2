#!/usr/bin/env bash
# Gazebo SDF world -> glTF, ready for upload.cognitive3d.com.
#
# Uses only upstream Gazebo and assimp -- no Cognitive3D code -- so it runs in any
# container that can already run your world. Its one non-obvious step is the
# COLLADA repair in the middle.
#
# THE REPAIR, and why it is not optional:
#   Gazebo's ColladaWorldExporter writes a .dae with NO <asset> block. COLLADA
#   1.4.1 requires <asset> and its default for an unspecified up_axis is Y_UP;
#   Gazebo is Z-up. So the exported file hands Z-up vertices to every importer
#   that will read them as Y-up, and the scene arrives lying on its side with no
#   error from any stage. Injecting <up_axis>Z_UP</up_axis> makes assimp emit the
#   -90 degree X rotation on the root node and the scene stands up.
#   Upstream: https://github.com/gazebosim/gz-common/issues/930
#   When that lands, the repair below detects the declaration and copies
#   unchanged, so this script needs no edit.
#
# Usage, inside a container that has Gazebo and the world's resource paths:
#   tools/export_world_gltf.sh husarion_office /tmp/out
#   tools/export_world_gltf.sh /path/to/my_world.sdf /tmp/out
#   tools/export_world_gltf.sh my_world /tmp/out glb      # one self-contained .glb
#
# The third argument is the output format, `gltf` (default) or `glb`.
#
# The DEFAULT IS gltf because that is what the platform accepts: upload takes a
# .gltf together with its .bin, and rejects a .glb. `glb` remains available
# because a single self-contained file cannot arrive with a dangling texture URI,
# which is a real advantage everywhere except the one place these assets are going.
#
# A gltf export is written as scene.gltf and scene.bin, with every texture copied
# beside them under its bare name. A scene upload stores the pair under exactly
# those two names and every texture under its basename, and rewrites no
# reference, so a glTF that still points at <world>.bin or at a texture path
# uploads without an error and renders empty or untextured.
set -eo pipefail

USAGE="usage: export_world_gltf.sh <world-name-or-sdf-path> <output-dir> [glb|gltf]"
case "${1:-}" in
  -h|--help) echo "$USAGE"; exit 0 ;;
esac
[ $# -ge 2 ] || { echo "$USAGE" >&2; exit 2; }
WORLD="$1"
OUTDIR="$2"
FORMAT="${3:-gltf}"
case "$FORMAT" in
  glb|gltf) ;;
  # An unrecognised format must not fall through to a default -- that is how you
  # get an artefact in a shape you did not ask for and do not notice.
  *) echo "unknown format '$FORMAT' (want: glb | gltf)" >&2; exit 2 ;;
esac

command -v gz      >/dev/null || { echo "gz not on PATH; source the ROS/Gazebo setup" >&2; exit 1; }
command -v assimp  >/dev/null || { echo "assimp not found; apt install assimp-utils" >&2; exit 1; }

# A bare name is resolved against the world search path the same way Gazebo does.
if [ -f "$WORLD" ]; then
  # ABSOLUTE, resolved before anything changes directory. `dirname` of a relative path stays
  # relative, and step 2 runs from the staging directory -- so a relative argument (the natural
  # way to invoke this from a repo root) would put a relative entry on GZ_SIM_RESOURCE_PATH and
  # resolve it against the staging directory instead. With the resource check below fatal,
  # that would turn a correct world into a hard failure.
  SDF="$(cd "$(dirname "$WORLD")" && pwd)/$(basename "$WORLD")"
  NAME="$(basename "${WORLD%.sdf}")"
else
  # Worlds are NOT generally on GZ_SIM_RESOURCE_PATH -- that variable points at
  # model directories. Search the ament prefixes' share trees as well.
  SDF="$(python3 -c "
import os, sys, glob
name = sys.argv[1]
roots = []
for var in ('GZ_SIM_RESOURCE_PATH', 'AMENT_PREFIX_PATH'):
    for entry in os.environ.get(var, '').split(':'):
        if not entry:
            continue
        roots.append(entry)
        roots.append(os.path.dirname(entry.rstrip('/')))
        roots.append(os.path.join(entry, 'share'))
seen = set()
for root in roots:
    if root in seen or not os.path.isdir(root):
        continue
    seen.add(root)
    for candidate in glob.glob(os.path.join(root, '**', name + '.sdf'), recursive=True):
        print(candidate); sys.exit(0)
sys.exit('cannot find ' + name + '.sdf under GZ_SIM_RESOURCE_PATH or AMENT_PREFIX_PATH')
" "$WORLD")"
  NAME="$WORLD"
fi
echo "world SDF : $SDF"

mkdir -p "$OUTDIR"
STAGE="$OUTDIR/.stage"
rm -rf "$STAGE" && mkdir -p "$STAGE"

# 1. Inject the exporter system plugin into a COPY of the world.
python3 - "$SDF" "$STAGE/world_with_exporter.sdf" <<'PY'
import sys, pathlib
src = pathlib.Path(sys.argv[1]).read_text()

# The first `<world` OUTSIDE an XML comment. A plain search matches a commented-out
# `<!-- <world name="old"> -->` just as happily, and then the real world gets no exporter -- which
# surfaces only as "exporter produced no .dae" several steps later.
def comment_spans(text):
    spans, at = [], 0
    while True:
        start = text.find('<!--', at)
        if start < 0:
            return spans
        end = text.find('-->', start + 4)
        if end < 0:
            spans.append((start, len(text)))
            return spans
        spans.append((start, end + 3))
        at = end + 3

spans = comment_spans(src)
at = 0
while True:
    start = src.find('<world', at)
    if start < 0:
        sys.exit('no <world> element outside a comment in ' + sys.argv[1])
    # `<worldly>` is not `<world>`
    following = src[start + 6: start + 7]
    if following in (' ', '\t', '\r', '\n', '>') and \
            not any(a <= start < b for a, b in spans):
        break
    at = start + 6

# The closing `>` OUTSIDE any attribute value: `<world name="a>b">` is legal XML, and taking
# the first `>` would splice the plugin into the middle of the name.
close, quote = -1, None
for index in range(start, len(src)):
    character = src[index]
    if quote:
        if character == quote:
            quote = None
    elif character in ('"', "'"):
        quote = character
    elif character == '>':
        close = index
        break
if close < 0:
    sys.exit('unterminated <world> start tag in ' + sys.argv[1])
plugin = ('\n    <plugin filename="gz-sim-collada-world-exporter-system"\n'
          '      name="gz::sim::systems::ColladaWorldExporter" />\n')
pathlib.Path(sys.argv[2]).write_text(src[:close + 1] + plugin + src[close + 1:])
PY

# 2. Run the server just long enough for the exporter to fire. It writes into the CWD, so run
#    from the staging directory -- which MOVES THE BASE for resolving the world's own resource
#    URIs. The source directory is prepended to GZ_SIM_RESOURCE_PATH to compensate, and the log
#    is kept rather than discarded: a world whose models do not resolve still exports, just
#    without them, and "an incomplete scene, exit 0" is the worst outcome this script has.
GZLOG="$STAGE/gz.log"
( cd "$STAGE" && GZ_SIM_RESOURCE_PATH="$(dirname "$SDF")${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" \
    gz sim -s -r -v 1 --iterations 20 "$STAGE/world_with_exporter.sdf" >"$GZLOG" 2>&1 ) || true

DAE="$(find "$STAGE" -name '*.dae' -print -quit)"
if [ -z "$DAE" ]; then
  echo "exporter produced no .dae. Gazebo said:" >&2
  tail -20 "$GZLOG" >&2 || true
  exit 1
fi

# An unresolvable mesh is a warning to Gazebo and a missing wall to you. Two rules, because
# matching log text is unreliable in BOTH directions and pretending otherwise is how this check
# would earn its own false confidence:
#
#   * anything Gazebo complained about is PRINTED, always. At -v 1 a clean run is quiet, so if
#     there is output here it is worth your eyes even when the gate below does not fire. This is
#     the half that does not depend on guessing Gazebo's phrasing.
#   * only lines that are specifically about RESOURCE resolution are fatal, and a plugin that
#     failed to load is explicitly not one of them -- a missing sensor plugin says nothing about
#     whether the walls are in the .dae, and refusing a complete export is the same class of
#     wrong answer in the other direction.
if [ -s "$GZLOG" ]; then
  COMPLAINTS="$(grep -iE '\[Err|\[Wrn|error|warn|unable|failed|no such' "$GZLOG" || true)"
  if [ -n "$COMPLAINTS" ]; then
    echo "gazebo said (full log: $GZLOG):" >&2
    printf '%s\n' "$COMPLAINTS" | head -15 >&2
    echo >&2
  fi
fi

RESOURCE_ERRORS="$(grep -iE 'model://|file://|\.dae|\.stl|\.obj|mesh' "$GZLOG" 2>/dev/null \
  | grep -iE 'unable to find|could not resolve|unable to download|error finding|no such file|cannot find' \
  | grep -viE 'plugin|\.so' || true)"
if [ -n "$RESOURCE_ERRORS" ]; then
  echo "the exporter ran but could not resolve some of the world's GEOMETRY:" >&2
  printf '%s\n' "$RESOURCE_ERRORS" | head -10 >&2
  echo >&2
  echo "The export would be missing that geometry, so this is fatal. Full log: $GZLOG" >&2
  exit 1
fi

# Honest limitation: the gate above matches known phrasings, and Gazebo's wording is not stable
# across versions. An incomplete export is possible with no match -- which is why the log is
# printed unconditionally and why step 5 reports the result's extents rather than asserting them.
echo "exported  : $DAE"

# 3. Repair the missing <asset>/<up_axis>. Streamed, because the .dae is large.
#    The logic lives in a module rather than inline here so it can be tested
#    without Gazebo: uv run tools/test_collada_zup_repair.py
REPAIRED="${DAE%.dae}_zup.dae"
python3 "$(dirname "$0")/collada_zup_repair.py" "$DAE" "$REPAIRED"

# 4. Convert to glTF. assimp names the buffer after the output file, so writing
#    scene.gltf is what makes the buffer scene.bin and its reference say so.
if [ "$FORMAT" = gltf ]; then
  OUT="$OUTDIR/scene.gltf"
else
  OUT="$OUTDIR/$NAME.glb"
fi
assimp export "$REPAIRED" "$OUT" >/dev/null
[ -s "$OUT" ] || { echo "assimp produced no $FORMAT at $OUT" >&2; exit 1; }
echo "glTF      : $OUT ($(du -h "$OUT" | cut -f1))"

# 4b. Put every texture beside the glTF. assimp keeps the COLLADA document's references, which
#     are relative to the .dae in the staging directory (`../materials/textures/<name>`), so they
#     resolve against the wrong directory here and carry a path the upload does not keep. A
#     reference that resolves nowhere is left as it is, for step 5 to report.
if [ "$FORMAT" = gltf ]; then
  python3 - "$OUT" "$(dirname "$REPAIRED")" <<'PY'
import json, os, shutil, sys, urllib.parse
gltf, source_dir = sys.argv[1], sys.argv[2]
out_dir = os.path.dirname(os.path.abspath(gltf))
with open(gltf) as f:
    spec = json.load(f)
copied = {}
for image in spec.get('images') or []:
    uri = image.get('uri')
    if not uri or uri.startswith('data:'):
        continue
    reference = urllib.parse.unquote(uri)
    name = os.path.basename(reference.replace('\\', '/'))
    source = os.path.normpath(os.path.join(source_dir, reference))
    if not name or not os.path.isfile(source):
        continue
    if copied.get(name, source) != source:
        sys.exit(f'two textures are both called {name} ({copied[name]} and {source}); the upload '
                 f'keeps basenames only, so one would replace the other. Rename one and re-export.')
    copied[name] = source
    shutil.copyfile(source, os.path.join(out_dir, name))
    image['uri'] = urllib.parse.quote(name)
with open(gltf, 'w') as f:
    json.dump(spec, f)
print(f'textures  : {len(copied)} copied beside {os.path.basename(gltf)}')
PY
fi

# 5. Report the result's orientation rather than trusting it. Bounds are computed
#    THROUGH the node hierarchy: accessor min/max are pre-transform, so a raw
#    bbox reports "on its side" even after a correct repair.
python3 "$(dirname "$0")/gltf_bbox.py" "$OUT"
