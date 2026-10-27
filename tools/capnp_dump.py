#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["pycapnp", "zstandard"]
# ///
"""Decode camera, lidar and pose payload captures into one JSON document.

The three binary streams are packed Cap'n Proto bodies (docs/data-format.md), so a capture of
one is the only captured stream that `json.load` cannot read. A pose capture's blocks are decoded
too, from the platform's pose block format, into times, joint values and events.
This turns any number of them into a single document on stdout, `{"files": [...]}` with one
entry per input, so a shell assertion reads the lot in one `json.load` rather than per file.

This is the one tool in tools/ that carries a third-party dependency. The repo's
standard-library-only rule holds everywhere else; a hand-written packed Cap'n Proto reader does
not survive a schema change, and the schemas are vendored from another repository. The
dependencies are declared inline (PEP 723), so

    uv run tools/capnp_dump.py capture/000012-camera.bin

resolves them into a throwaway environment and the interpreter that runs every other tool stays
clean.

Everything that does not need the schemas -- the planar point decoder, the two colour decoders,
the name-to-stream rule -- is a plain function here, and `import capnp` happens inside
`load_schemas`. tools/test_capnp_dump.py covers that half unconditionally, which is what makes
it worth running on a machine with no pycapnp.
"""
import argparse
import io
import itertools
import json
import math
import os
import re
import struct
import sys
from pathlib import Path

SCHEMA_DIR = Path(__file__).resolve().parent.parent / 'core' / 'capnp'

# A capture's NAME is the only thing that says which schema decodes it: the payload structs
# share their first eight field ordinals, so a body gives no discriminator to sniff.
STREAM_SUFFIXES = (('-camera.bin', 'camera'), ('-lidar.bin', 'lidar'), ('-pose.bin', 'pose'))
STREAMS = tuple(stream for _, stream in STREAM_SUFFIXES)

# A pose block: a 4-byte prefix (magic, format version, codec, flags), then the body, whose header
# is N, J, startUs and the three quantisation steps in 1e-9 units, little-endian.
POSE_MAGIC = 0xC3
POSE_FORMAT_VERSION = 1
POSE_CODEC_NONE = 0
POSE_CODEC_ZSTD = 1
POSE_HEADER = '<iiqiii'
POSE_HEADER_BYTES = 28
# Each joint's mask byte names the groups the block writes for it, in this order.
POSE_GROUPS = (('p', 0x01, 3), ('r', 0x02, 4), ('s', 0x04, 3))

# The three enum-like PointCloud fields, and what each value costs per point. They are Text on
# the wire and the gateway answers 400 to anything not spelled exactly like this.
POSITION_ELEMENT = {'int16_mm': ('h', 2), 'float32': ('f', 4)}
COLOR_BYTES = {'none': 0, 'rgb565': 2, 'rgb332': 1, 'rgb888': 3}
INTENSITY_BYTES = {'none': 0, 'uint8': 1}

# int16 position planes are millimetres.
MILLIMETRES_PER_METRE = 1000.0

# pycapnp's default traversal limit is a guard against hostile messages. A camera part is a
# batch of multi-megabyte JPEGs from a sender we wrote, so the guard has to be lifted or a
# legitimate capture fails to read.
TRAVERSAL_LIMIT_WORDS = 2 ** 30

JPEG_SOI = b'\xff\xd8'

# Anything outside this becomes '_' in a filename component -- three of the four components of
# an extracted JPEG's name are payload text, and a path separator in one of them would write
# outside the directory the caller named.
UNSAFE_IN_FILENAME = re.compile(r'[^A-Za-z0-9._-]')


def stream_for_filename(name):
    """The stream a capture's name declares, or None when it declares neither."""
    for suffix, stream in STREAM_SUFFIXES:
        if name.endswith(suffix):
            return stream
    return None


def resolve_stream(name, override=None):
    """Which schema decodes `name`. Never guesses: an undecidable name is an error naming it."""
    if override is not None:
        return override
    stream = stream_for_filename(name)
    if stream is None:
        raise ValueError(
            f'{name}: cannot tell whether this is a camera, lidar or pose capture. A capture is '
            f'named "...-camera.bin", "...-lidar.bin" or "...-pose.bin"; pass --stream to decode '
            f'anything else.')
    return stream


def decode_rgb565(value):
    """One uint16 to (r, g, b), by the bit replication docs/data-format.md specifies.

    Each channel's low bits are refilled from its own high bits, which is what keeps a mid grey
    mid grey instead of drifting dark. 0x0000 and 0xFFFF decode to exactly black and exactly
    white, and every code survives a decode and re-encode unchanged.
    """
    red = ((value >> 11) << 3) | (value >> 13)
    green = (((value >> 5) & 63) << 2) | ((value >> 9) & 3)
    blue = ((value & 31) << 3) | ((value >> 2) & 7)
    return red, green, blue


def decode_rgb332(value):
    """One uint8 to (r, g, b): r in bits 7-5, g in 4-2, b in 1-0, same replication.

    A three-bit channel `210` replicates to `210 210 21`, so red and green are the bits, the
    bits again three places down, and the top two again six down. Blue has two bits and tiles
    them four times, which is why its line runs to four terms.
    """
    red = (value & 0xE0) | ((value & 0xE0) >> 3) | ((value & 0xE0) >> 6)
    green = ((value << 3) & 0xE0) | (value & 0x1C) | ((value >> 3) & 0x03)
    blue = ((value & 0x03) << 6) | ((value & 0x03) << 4) | ((value & 0x03) << 2) | (value & 0x03)
    return red, green, blue


def point_stride_bytes(position, color, intensity):
    """Bytes one point contributes across all five planes."""
    if position not in POSITION_ELEMENT:
        raise ValueError(f'position: {position!r} is not one of {sorted(POSITION_ELEMENT)}')
    if color not in COLOR_BYTES:
        raise ValueError(f'color: {color!r} is not one of {sorted(COLOR_BYTES)}')
    if intensity not in INTENSITY_BYTES:
        raise ValueError(f'intensity: {intensity!r} is not one of {sorted(INTENSITY_BYTES)}')
    return (3 * POSITION_ELEMENT[position][1]
            + COLOR_BYTES[color] + INTENSITY_BYTES[intensity])


def decompress_block(data, zstd_level):
    """The raw planar block. `zstdLevel` 0 means `data` already is it."""
    if zstd_level <= 0:
        return data
    try:
        import zstandard
    except ImportError as exc:
        raise RuntimeError(
            f'this block is zstd-compressed (zstdLevel={zstd_level}) and the `zstandard` '
            f'package is not importable. Run this tool as `uv run tools/capnp_dump.py`, which '
            f'installs the PEP 723 dependencies declared at the top of this file.') from exc
    # A frame written without a content-size header is legal and `decompress()` refuses it for
    # want of an output size. The streaming route reads both kinds.
    return zstandard.ZstdDecompressor().stream_reader(io.BytesIO(data)).read()


def decode_planar_block(data, point_count, position, color, intensity):
    """Decode `X[N] Y[N] Z[N] C[N] I[N]` and say whether the block was the size it claims.

    Returns `(points, consistent)`. A point is `(x, y, z, rgb, intensity)`: metres for the
    position whatever the wire element was, `rgb` an (r, g, b) triple or None, `intensity` an
    int or None.

    `consistent` is whether `len(data)` is exactly what `pointCount` and the three format fields
    predict. When it is not, the planes are read for as many whole points as the block can hold
    rather than for `pointCount` -- the layout is planar, so a short block moves every plane
    boundary and the numbers that come back are not the ones the sender wrote. The flag is the
    only thing that says so.
    """
    stride = point_stride_bytes(position, color, intensity)
    expected = point_count * stride
    consistent = len(data) == expected
    count = point_count if len(data) >= expected else len(data) // stride

    typecode, element_bytes = POSITION_ELEMENT[position]
    divisor = MILLIMETRES_PER_METRE if position == 'int16_mm' else 1.0
    offset = 0
    axes = []
    for _ in range(3):
        axes.append(struct.unpack_from(f'<{count}{typecode}', data, offset))
        offset += count * element_bytes
    colors = _decode_color_plane(data, offset, count, color)
    offset += count * COLOR_BYTES[color]
    intensities = _decode_intensity_plane(data, offset, count, intensity)

    xs, ys, zs = axes
    points = [(xs[i] / divisor, ys[i] / divisor, zs[i] / divisor,
               None if colors is None else colors[i],
               None if intensities is None else intensities[i])
              for i in range(count)]
    return points, consistent


def _decode_color_plane(data, offset, count, color):
    if color == 'none':
        return None
    if color == 'rgb565':
        return [decode_rgb565(v) for v in struct.unpack_from(f'<{count}H', data, offset)]
    if color == 'rgb332':
        return [decode_rgb332(v) for v in struct.unpack_from(f'<{count}B', data, offset)]
    raw = struct.unpack_from(f'<{3 * count}B', data, offset)   # rgb888: the bytes as they were
    return [tuple(raw[3 * i:3 * i + 3]) for i in range(count)]


def _decode_intensity_plane(data, offset, count, intensity):
    if intensity == 'none':
        return None
    return list(struct.unpack_from(f'<{count}B', data, offset))


def parse_pose_body(body):
    """One decompressed pose block body: its header, sample times, masks, groups and events.

    Times are absolute epoch microseconds. A group the block writes comes back per sample in
    meters (`p`), as a quaternion `[x, y, z, w]` (`r`) or as a scale (`s`), keyed by joint and then
    group; a group the mask leaves out is carried over from an earlier block and is absent here.
    `consistent` is whether the body ended exactly where its own counts say it does.
    """
    if len(body) < POSE_HEADER_BYTES:
        raise ValueError(f'a pose block body of {len(body)} bytes has no 28-byte header')
    n, j, start_us, pos_step, rot_step, scale_step = struct.unpack_from(POSE_HEADER, body, 0)
    if n < 1 or j < 1:
        raise ValueError(f'a pose block of {n} samples and {j} joints')
    steps = {'p': pos_step * 1e-9, 'r': rot_step * 1e-9, 's': scale_step * 1e-9}
    offset = POSE_HEADER_BYTES
    deltas = struct.unpack_from(f'<{n}i', body, offset)
    offset += 4 * n
    times = [start_us + total for total in itertools.accumulate(deltas)]
    masks = list(body[offset:offset + j])
    offset += j
    joints = {}
    for joint in range(j):
        for group, bit, channels in POSE_GROUPS:
            if not masks[joint] & bit:
                continue
            columns = []
            for _ in range(channels):
                # The first value is absolute and every later one a delta from the one before.
                raw = struct.unpack_from(f'<{n}i', body, offset)
                offset += 4 * n
                columns.append(list(itertools.accumulate(raw)))
            joints.setdefault(str(joint), {})[group] = [
                [columns[c][i] * steps[group] for c in range(channels)] for i in range(n)]
    (event_count,) = struct.unpack_from('<i', body, offset)
    offset += 4
    events = [list(struct.unpack_from('<ii', body, offset + 8 * e)) for e in range(event_count)]
    offset += 8 * event_count
    return {
        'sampleCount': n,
        'jointCount': j,
        'startUs': start_us,
        'steps': {'pos': pos_step, 'rot': rot_step, 'scale': scale_step},
        'times': times,
        'masks': masks,
        'joints': joints,
        'events': events,
        'consistent': offset == len(body),
    }


def decode_pose_block(data):
    """One block as the envelope carries it: the prefix read, the body decompressed and parsed."""
    if len(data) < 4 or data[0] != POSE_MAGIC or data[1] != POSE_FORMAT_VERSION:
        raise ValueError(f'not a version {POSE_FORMAT_VERSION} pose block: {bytes(data[:4]).hex()}')
    codec, flags = data[2], data[3]
    if codec == POSE_CODEC_NONE:
        body = bytes(data[4:])
    elif codec == POSE_CODEC_ZSTD:
        body = decompress_block(bytes(data[4:]), 1)
    else:
        raise ValueError(f'pose block codec {codec} is not one this tool reads')
    decoded = parse_pose_body(body)
    decoded['keyframe'] = bool(flags & 0x01)
    decoded['codec'] = codec
    return decoded


def point_json(point):
    """`[x, y, z]`, then the (r, g, b) triple if there is colour, then intensity if there is."""
    x, y, z, rgb, intensity = point
    row = [x, y, z]
    if rgb is not None:
        row.append(list(rgb))
    if intensity is not None:
        row.append(intensity)
    return row


def safe_component(text):
    """One filename component, with everything that could steer a path replaced."""
    return UNSAFE_IN_FILENAME.sub('_', text) or '_'


def jpeg_filename(session_id, camera_id, part, seq):
    """`<sessionId>-<cameraId>-<part>-<seq>.jpg`."""
    return '-'.join((safe_component(session_id), safe_component(camera_id),
                     str(part), str(seq))) + '.jpg'


def load_pose_schema(schema_dir):
    """The pose envelope's schema module. Needs pycapnp, like load_schemas."""
    import capnp
    capnp.remove_import_hook()
    bundled = os.path.dirname(os.path.dirname(os.path.abspath(capnp.__file__)))
    schema_dir = Path(schema_dir)
    return capnp.load(str(schema_dir / 'pose_payload.capnp'), imports=[str(schema_dir), bundled])


def load_schemas(schema_dir):
    """The camera and lidar schema modules. The only function here that needs pycapnp."""
    import capnp
    capnp.remove_import_hook()
    # `imports=` REPLACES pycapnp's default search path, so the schemas' own
    # `using Cxx = import "/capnp/c++.capnp"` fails with `Import failed: /capnp/c++.capnp`
    # unless the bundled copy is reachable. It sits at `<site-packages>/capnp/c++.capnp`, so
    # the entry that must be on the path is site-packages itself, not the capnp package.
    bundled = os.path.dirname(os.path.dirname(os.path.abspath(capnp.__file__)))
    schema_dir = Path(schema_dir)
    imports = [str(schema_dir), bundled]
    camera = capnp.load(str(schema_dir / 'camera_payload.capnp'), imports=imports)
    lidar = capnp.load(str(schema_dir / 'lidar_payload.capnp'), imports=imports)
    return camera, lidar


def header_json(payload):
    """The eight header fields, which the three payload structs share at the same ordinals."""
    return {
        'userId': payload.userId,
        'sessionId': payload.sessionId,
        'sceneId': payload.sceneId,
        'sceneVersion': payload.sceneVersion,
        'formatVersion': payload.formatVersion,
        'sessionStartMs': payload.sessionStartMs,
        'sdkVersion': payload.sdkVersion,
        'part': payload.part,
    }


def pose_json(pose):
    return [pose.px, pose.py, pose.pz, pose.qx, pose.qy, pose.qz, pose.qw]


def describe_camera(payload, extract_dir=None):
    frames = []
    extracted = 0
    for frame in payload.frames:
        data = bytes(frame.data)
        frames.append({
            'cameraId': frame.cameraId,
            'time': frame.time,
            'seq': frame.seq,
            'frameId': frame.frameId,
            'pose': pose_json(frame.pose),
            'format': frame.format,
            'width': frame.width,
            'height': frame.height,
            # Full angles in degrees; 0 is the schema's "not recorded".
            'hfovDegrees': frame.hfovDegrees,
            'vfovDegrees': frame.vfovDegrees,
            'dataBytes': len(data),
            # Whether the bytes open with a JPEG start-of-image marker. Cheap proof they are a
            # picture and not a truncated or empty buffer; decoding one is not this tool's job.
            'jpegSoi': data[:2] == JPEG_SOI,
        })
        if extract_dir is not None:
            name = jpeg_filename(payload.sessionId, frame.cameraId, payload.part, frame.seq)
            (Path(extract_dir) / name).write_bytes(data)
            extracted += 1
    entry = {'header': header_json(payload), 'frames': frames}
    if extract_dir is not None:
        entry['extracted'] = extracted
    return entry


def describe_lidar(payload, scan_ranges=False):
    scans = []
    for scan in payload.scans:
        kind = str(scan.which())
        record = {
            'sensorId': scan.sensorId,
            'time': scan.time,
            'seq': scan.seq,
            'frameId': scan.frameId,
            'pose': pose_json(scan.pose),
            'kind': kind,
        }
        if kind == 'laserscan':
            record.update(laserscan_json(scan.laserscan, scan_ranges))
        else:
            record.update(pointcloud_json(scan.pointcloud))
        scans.append(record)
    return {'header': header_json(payload), 'scans': scans}


def laserscan_json(scan, ranges_verbatim=False):
    ranges = list(scan.ranges)
    record = {
        'angleMin': scan.angleMin,
        'angleMax': scan.angleMax,
        'angleIncrement': scan.angleIncrement,
        'timeIncrement': scan.timeIncrement,
        'scanTime': scan.scanTime,
        'rangeMin': scan.rangeMin,
        'rangeMax': scan.rangeMax,
        'rangeCount': len(ranges),
        'intensityCount': len(scan.intensities),
        # A scan is copied to the wire as published, infinities and NaNs included, so the count
        # of ranges that are actually measurements is separate from the count of rays.
        'finiteRangeCount': sum(1 for value in ranges if math.isfinite(value)),
    }
    if ranges_verbatim:
        # The arrays themselves, for a reader placing ray i by the rule in docs/data-format.md: its
        # angle is angleMin + i * angleIncrement and its range is ranges[i]. Off by default
        # because a 640-ray sweep is most of the document and almost no assertion needs it.
        # The non-finite ranges reach the document as JSON's non-standard `Infinity` and `NaN`
        # tokens, which Python writes and reads back but a strict parser rejects.
        record['ranges'] = ranges
        record['intensities'] = list(scan.intensities)
    return record


def pointcloud_json(cloud):
    data = bytes(cloud.data)
    block = decompress_block(data, cloud.zstdLevel)
    points, consistent = decode_planar_block(
        block, cloud.pointCount, cloud.position, cloud.color, cloud.intensity)
    return {
        'pointCount': cloud.pointCount,
        'position': cloud.position,
        'color': cloud.color,
        'intensity': cloud.intensity,
        'voxelLeafMm': cloud.voxelLeafMm,
        'zstdLevel': cloud.zstdLevel,
        'dataBytes': len(data),
        'decoded': {
            'points': len(points),
            'first': [point_json(point) for point in points[:3]],
            'consistent': consistent,
        },
    }


def properties_json(properties):
    """A manifest's property list as a dict: each value as its union arm holds it."""
    out = {}
    for prop in properties:
        arm = str(prop.which())
        out[prop.key] = getattr(prop, arm)
    return out


def describe_pose(payload):
    manifests = []
    for manifest in payload.manifests:
        manifests.append({
            'manifestId': manifest.manifestId,
            'name': manifest.name,
            'mesh': manifest.mesh,
            'fileType': manifest.fileType,
            'riggingMappingType': manifest.riggingMappingType,
            'root': manifest.root,
            'firstSeq': manifest.firstSeq,
            'properties': properties_json(manifest.properties),
            'joints': [{'j': joint.j, 'meshName': joint.meshName,
                        'properties': properties_json(joint.properties)}
                       for joint in manifest.joints],
        })
    blocks = []
    for block in payload.blocks:
        data = bytes(block.data)
        blocks.append({
            'manifestId': block.manifestId,
            'startUs': block.startUs,
            'sampleCount': block.sampleCount,
            'resetBefore': block.resetBefore,
            'seq': block.seq,
            'dataBytes': len(data),
            'decoded': decode_pose_block(data),
        })
    return {'header': header_json(payload), 'startUs': payload.startUs, 'endUs': payload.endUs,
            'manifests': manifests, 'blocks': blocks}


def decode_file(path, stream, schemas, extract_dir=None, scan_ranges=False, pose_schema=None):
    camera, lidar = schemas
    raw = Path(path).read_bytes()
    entry = {'file': str(path), 'stream': stream, 'bytes': len(raw)}
    if stream == 'pose':
        payload = pose_schema.PosePayload.from_bytes_packed(
            raw, traversal_limit_in_words=TRAVERSAL_LIMIT_WORDS)
        entry.update(describe_pose(payload))
    elif stream == 'camera':
        payload = camera.CameraPayload.from_bytes_packed(
            raw, traversal_limit_in_words=TRAVERSAL_LIMIT_WORDS)
        entry.update(describe_camera(payload, extract_dir))
    else:
        payload = lidar.LidarPayload.from_bytes_packed(
            raw, traversal_limit_in_words=TRAVERSAL_LIMIT_WORDS)
        entry.update(describe_lidar(payload, scan_ranges))
    return entry


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Decode camera, lidar and pose payload captures into one JSON document.')
    parser.add_argument('--stream', choices=STREAMS,
                        help='decode every file as this stream instead of reading its name')
    parser.add_argument('--schema-dir', default=str(SCHEMA_DIR),
                        help='where the vendored .capnp schemas live (default: %(default)s)')
    parser.add_argument('--extract-jpeg', metavar='DIR',
                        help='also write every camera frame to DIR as a .jpg')
    parser.add_argument('--scan-ranges', action='store_true',
                        help='include each LaserScan\'s ranges and intensities verbatim')
    parser.add_argument('--indent', type=int, default=2, help='json indent (default: %(default)s)')
    parser.add_argument('files', nargs='+', metavar='FILE')
    args = parser.parse_args(argv)

    # Every name is resolved before the first file is read, so an undecidable name in a long
    # list fails before any of the list has been decoded.
    try:
        streams = [resolve_stream(name, args.stream) for name in args.files]
    except ValueError as exc:
        parser.error(str(exc))

    if args.extract_jpeg:
        os.makedirs(args.extract_jpeg, exist_ok=True)
    schemas = load_schemas(args.schema_dir)
    pose_schema = load_pose_schema(args.schema_dir) if 'pose' in streams else None
    decoded = [decode_file(name, stream, schemas, args.extract_jpeg, args.scan_ranges, pose_schema)
               for name, stream in zip(args.files, streams)]
    json.dump({'files': decoded}, sys.stdout, indent=args.indent)
    sys.stdout.write('\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
