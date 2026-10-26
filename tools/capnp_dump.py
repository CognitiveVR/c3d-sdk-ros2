#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["pycapnp", "zstandard"]
# ///
"""Decode camera and lidar payload captures into one JSON document.

The two binary streams are packed Cap'n Proto bodies (docs/data-format.md), so a capture of
one is the only captured stream that `json.load` cannot read.
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
the inverse scan and cloud transforms, the NAL unit splitter, the name-to-stream rule -- is a plain
function here, and `import capnp` happens inside `load_schemas`. tools/test_capnp_dump.py covers
that half unconditionally, which is what makes it worth running on a machine with no pycapnp.
"""
import argparse
import io
import json
import math
import os
import re
import struct
import sys
from pathlib import Path

SCHEMA_DIR = Path(__file__).resolve().parent.parent / 'core' / 'capnp'

# A capture's NAME is the only thing that says which schema decodes it: the two payload structs
# open with the same eleven fields, ordinal for ordinal, so a body gives no discriminator to sniff.
STREAM_SUFFIXES = (('-camera.bin', 'camera'), ('-lidar.bin', 'lidar'))

# The three enum-like PointCloud fields, and what each value costs per point. They are Text on
# the wire and the gateway answers 400 to anything not spelled exactly like this.
POSITION_ELEMENT = {'int16_mm': ('h', 2), 'float32': ('f', 4)}
COLOR_BYTES = {'none': 0, 'rgb565': 2, 'rgb332': 1, 'rgb888': 3}
INTENSITY_BYTES = {'none': 0, 'uint8': 1}

# int16 position planes are millimetres.
MILLIMETRES_PER_METRE = 1000.0

# pycapnp's default traversal limit is a guard against hostile messages. A camera chunk is
# seconds of video and a cloud object up to 2 MiB, from a sender we wrote, so the guard has to be
# lifted or a legitimate capture fails to read.
TRAVERSAL_LIMIT_WORDS = 2 ** 30

# The ScanEncoding and CloudEncoding enumerants, by name.
SCAN_ENCODINGS = ('uint16', 'deltaShuffleZstd')
CLOUD_ENCODINGS = ('planarZstd', 'planarDeltaShuffleZstd')

# H.264 NAL unit types (ITU-T H.264 table 7-1) that a chunk's own fields are checked against.
NAL_IDR = 5
NAL_ACCESS_UNIT_DELIMITER = 9

# Anything outside this becomes '_' in a filename component -- two of the three components of an
# extracted stream's name are payload text, and a path separator in one of them would write
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
            f'{name}: cannot tell whether this is a camera or a lidar capture. A capture is '
            f'named "...-camera.bin" or "...-lidar.bin"; pass --stream to decode anything else.')
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


def zstd_decompress(data):
    try:
        import zstandard
    except ImportError as exc:
        raise RuntimeError(
            'this block is zstd-compressed and the `zstandard` package is not importable. Run '
            'this tool as `uv run tools/capnp_dump.py`, which installs the PEP 723 dependencies '
            'declared at the top of this file.') from exc
    # A frame written without a content-size header is legal and `decompress()` refuses it for
    # want of an output size. The streaming route reads both kinds.
    return zstandard.ZstdDecompressor().stream_reader(io.BytesIO(data)).read()


def decompress_block(data, zstd_level):
    """A cloud's block as stored before compression. `zstdLevel` 0 means `data` already is it."""
    if zstd_level <= 0:
        return data
    return zstd_decompress(data)


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


def inverse_delta_shuffle(data, rays):
    """Undo core/src/scan_block.cpp: byte-unshuffle the whole block, then a per-scan running sum."""
    count = len(data) // 2
    low, high = data[:count], data[count:2 * count]
    codes = []
    for start in range(0, count, rays):
        value = 0
        for i in range(start, min(start + rays, count)):
            value = (value + (low[i] | (high[i] << 8))) & 0xFFFF
            codes.append(value)
    return codes


def dequantise_range(code, unit):
    if code == 0xFFFF:
        return math.inf
    if code == 0xFFFE:
        return math.nan
    if code == 0:
        return -math.inf
    if unit == 'millimetre':
        return code * 0.001
    if unit == 'centimetre':
        return code * 0.01
    raise ValueError(f'unit: {unit!r} is not millimetre or centimetre')


def inverse_delta_shuffle_planes(block, n, position_bytes):
    """Undo pointcloud::DeltaShufflePositionPlanes; only int16 positions are transformed."""
    if position_bytes != 2:
        return block
    out = bytearray(block)
    for plane in range(3):
        base = plane * n * 2
        value = 0
        for i in range(n):
            value = (value + (block[base + i] | (block[base + n + i] << 8))) & 0xFFFF
            out[base + 2 * i] = value & 0xFF
            out[base + 2 * i + 1] = value >> 8
    return bytes(out)


def split_nal_units(data):
    """(type, start-code offset, length to the next start code) per Annex-B NAL unit.

    A unit that ends where its start code does has no header byte, so its type is None: a stream
    cut short is reported as such rather than raised on.
    """
    starts = []
    i = data.find(b'\x00\x00\x01')
    while i != -1:
        starts.append(i - 1 if i > 0 and data[i - 1] == 0 else i)
        i = data.find(b'\x00\x00\x01', i + 3)
    units = []
    for k, start in enumerate(starts):
        end = starts[k + 1] if k + 1 < len(starts) else len(data)
        header = start + (4 if data[start] == 0 and data[start + 2] == 0 else 3)
        units.append((data[header] & 0x1F if header < end else None, start, end - start))
    return units


def parse_avcc(data):
    """An AVCDecoderConfigurationRecord's profile, level and first SPS and PPS."""
    sps_length = int.from_bytes(data[6:8], 'big')
    sps = bytes(data[8:8 + sps_length])
    at = 8 + sps_length + 1
    pps_length = int.from_bytes(data[at:at + 2], 'big')
    return {'profile': data[1], 'level': data[3], 'sps': sps,
            'pps': bytes(data[at + 2:at + 2 + pps_length])}


def decode_scan_codes(data, rays, encoding):
    """A ScanBlock's `ranges` or `intensities` as its uint16 codes, scan after scan."""
    if encoding not in SCAN_ENCODINGS:
        raise ValueError(f'encoding: {encoding!r} is not one of {list(SCAN_ENCODINGS)}')
    if not data or rays == 0:
        return []
    if encoding == 'uint16':
        return list(struct.unpack_from(f'<{len(data) // 2}H', data))
    return inverse_delta_shuffle(zstd_decompress(data), rays)


def decode_cloud(data, offset, length, point_count, position, color, intensity, encoding,
                 zstd_level):
    """One CloudEntry's points out of its object's `data`: `(points, consistent)`.

    `consistent` is decode_planar_block's, and false as well when the entry's slice runs past the
    end of `data`. The position planes are transformed back only when the block holds all three:
    a shorter one is already inconsistent, and is read as it stands.
    """
    if encoding not in CLOUD_ENCODINGS:
        raise ValueError(f'encoding: {encoding!r} is not one of {list(CLOUD_ENCODINGS)}')
    point_stride_bytes(position, color, intensity)
    end = offset + length
    block = decompress_block(data[offset:end], zstd_level)
    position_bytes = POSITION_ELEMENT[position][1]
    if encoding == 'planarDeltaShuffleZstd' and len(block) >= 3 * point_count * position_bytes:
        block = inverse_delta_shuffle_planes(block, point_count, position_bytes)
    points, consistent = decode_planar_block(block, point_count, position, color, intensity)
    return points, consistent and end <= len(data)


def access_unit_json(unit):
    """One frame's access unit: its NAL unit types in order, and whether it opens on a delimiter.

    The delimiter has to be at byte 0. An access unit that opens with anything else was sliced at
    the wrong offset, or written without the delimiter that marks where it begins.
    """
    nals = split_nal_units(unit)
    return {
        'nalTypes': [nal_type for nal_type, _offset, _length in nals],
        'startsWithAud': bool(nals) and nals[0][0] == NAL_ACCESS_UNIT_DELIMITER and nals[0][1] == 0,
    }


def offsets_tile(spans, total):
    """Whether `(offset, length)` spans run back to back from 0 and end exactly at `total`."""
    at = 0
    for offset, length in spans:
        if offset != at:
            return False
        at += length
    return at == total


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


def h264_filename(session_id, camera_id, seq):
    """`<sessionId>-<cameraId>-<seq>.h264`: one chunk's `data`, a raw Annex-B stream."""
    return '-'.join((safe_component(session_id), safe_component(camera_id), str(seq))) + '.h264'


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
    """The seven session fields, which CameraPayload and LidarPayload share at the same ordinals."""
    return {
        'userId': payload.userId,
        'sessionId': payload.sessionId,
        'sceneId': payload.sceneId,
        'sceneVersion': payload.sceneVersion,
        'formatVersion': payload.formatVersion,
        'sessionStartMs': payload.sessionStartMs,
        'sdkVersion': payload.sdkVersion,
    }


def pose_json(pose):
    return [pose.px, pose.py, pose.pz, pose.qx, pose.qy, pose.qz, pose.qw]


def describe_camera(payload, extract_dir=None):
    data = bytes(payload.data)
    # Copied onto every frame, so a reader that walks frames rather than chunks -- the scan-plane
    # verifier filters frames on cameraId and frameId -- needs nothing from the chunk around them.
    shared = {
        'cameraId': payload.cameraId,
        'frameId': payload.frameId,
        'width': payload.width,
        'height': payload.height,
        # Full angles in degrees at the chunk's first frame; 0 is the schema's "not recorded".
        'hfovDegrees': payload.hfovDegrees,
        'vfovDegrees': payload.vfovDegrees,
    }
    frames = []
    for frame in payload.frames:
        record = {
            'time': frame.time,
            'seq': frame.seq,
            'pose': pose_json(frame.pose),
            'byteOffset': frame.byteOffset,
            'byteLength': frame.byteLength,
            'keyframe': frame.keyframe,
        }
        record.update(access_unit_json(data[frame.byteOffset:frame.byteOffset + frame.byteLength]))
        record.update(shared)
        frames.append(record)
    entry = {
        'header': header_json(payload),
        'cameraId': payload.cameraId,
        'seq': payload.seq,
        'startMs': payload.startMs,
        'endMs': payload.endMs,
        'codec': payload.codec,
        'width': payload.width,
        'height': payload.height,
        'nominalFps': payload.nominalFps,
        'hfovDegrees': payload.hfovDegrees,
        'vfovDegrees': payload.vfovDegrees,
        'frameId': payload.frameId,
        'dataBytes': len(data),
        # Every frame's access unit back to back from byte 0 and nothing after the last, so each
        # frame's slice is exactly its own access unit and no byte of `data` belongs to no frame.
        'offsetsTile': offsets_tile(
            ((frame['byteOffset'], frame['byteLength']) for frame in frames), len(data)),
        # The flag a reader seeks by agrees with the bitstream: an IDR slice in the frame's access
        # unit and `keyframe`, both or neither.
        'keyframesMatchNalTypes': all(
            frame['keyframe'] == (NAL_IDR in frame['nalTypes']) for frame in frames),
        'frames': frames,
    }
    if extract_dir is not None:
        path = Path(extract_dir) / h264_filename(payload.sessionId, payload.cameraId, payload.seq)
        path.write_bytes(data)
        entry['extracted'] = str(path)
    return entry


def describe_lidar(payload, scan_ranges=False):
    kind = str(payload.which())
    entry = {
        'header': header_json(payload),
        'sensorId': payload.sensorId,
        'seq': payload.seq,
        'startMs': payload.startMs,
        'endMs': payload.endMs,
        'frameId': payload.frameId,
        'kind': kind,
    }
    if kind == 'scans':
        entry.update(scan_block_json(payload, scan_ranges))
    else:
        entry.update(cloud_block_json(payload.clouds))
    return entry


def scan_block_json(payload, scan_ranges=False):
    block = payload.scans
    rays = block.rays
    unit = str(block.rangeUnit)
    encoding = str(block.encoding)
    ranges = [dequantise_range(code, unit)
              for code in decode_scan_codes(bytes(block.ranges), rays, encoding)]
    intensities = decode_scan_codes(bytes(block.intensities), rays, encoding)
    # Copied onto every entry, so a reader that walks scans rather than blocks -- placing ray i at
    # angleMin + i * angleIncrement, or filtering on sensorId and frameId -- needs nothing from
    # the block around them.
    shared = {
        'sensorId': payload.sensorId,
        'frameId': payload.frameId,
        'angleMin': block.angleMin,
        'angleIncrement': block.angleIncrement,
        'kind': 'scans',
    }
    entries = []
    for index, scan in enumerate(block.entries):
        record = {
            'time': scan.time,
            'seq': scan.seq,
            'pose': pose_json(scan.pose),
            'timeIncrement': scan.timeIncrement,
            'scanTime': scan.scanTime,
        }
        record.update(shared)
        if scan_ranges:
            # Ranges in metres with the three sentinels restored, intensities as their codes. Off
            # by default because a 640-ray sweep is most of the document and almost no assertion
            # needs it. The non-finite ranges reach the document as JSON's non-standard
            # `Infinity` and `NaN` tokens, which Python writes and reads back but a strict parser
            # rejects.
            record['ranges'] = ranges[index * rays:(index + 1) * rays]
            record['intensities'] = intensities[index * rays:(index + 1) * rays]
        entries.append(record)
    return {
        'angleMin': block.angleMin,
        'angleMax': block.angleMax,
        'angleIncrement': block.angleIncrement,
        'rangeMin': block.rangeMin,
        'rangeMax': block.rangeMax,
        'rays': rays,
        'rangeUnit': unit,
        'encoding': encoding,
        'entries': entries,
        # A scan is stored with its infinities and NaNs, each as a code of its own, so the count of
        # ranges that are actually measurements is separate from the count of rays.
        'finiteRangeCount': sum(1 for value in ranges if math.isfinite(value)),
    }


def cloud_block_json(block):
    data = bytes(block.data)
    encoding = str(block.encoding)
    entries = []
    for cloud in block.entries:
        points, consistent = decode_cloud(
            data, cloud.byteOffset, cloud.byteLength, cloud.pointCount, block.position,
            block.color, block.intensity, encoding, block.zstdLevel)
        entries.append({
            'time': cloud.time,
            'seq': cloud.seq,
            'pose': pose_json(cloud.pose),
            'pointCount': cloud.pointCount,
            'byteOffset': cloud.byteOffset,
            'byteLength': cloud.byteLength,
            'decoded': {
                'points': len(points),
                'first': [point_json(point) for point in points[:3]],
                'consistent': consistent,
            },
        })
    return {
        'position': block.position,
        'color': block.color,
        'intensity': block.intensity,
        'voxelLeafMm': block.voxelLeafMm,
        'encoding': encoding,
        'zstdLevel': block.zstdLevel,
        'entries': entries,
    }


def decode_file(path, stream, schemas, extract_dir=None, scan_ranges=False):
    camera, lidar = schemas
    raw = Path(path).read_bytes()
    entry = {'file': str(path), 'stream': stream, 'bytes': len(raw)}
    if stream == 'camera':
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
        description='Decode camera and lidar payload captures into one JSON document.')
    parser.add_argument('--stream', choices=('camera', 'lidar'),
                        help='decode every file as this stream instead of reading its name')
    parser.add_argument('--schema-dir', default=str(SCHEMA_DIR),
                        help='where the vendored .capnp schemas live (default: %(default)s)')
    parser.add_argument('--extract-h264', metavar='DIR',
                        help='also write every camera chunk\'s stream to DIR as a .h264')
    parser.add_argument('--scan-ranges', action='store_true',
                        help='include each scan\'s decoded ranges and intensities')
    parser.add_argument('--indent', type=int, default=2, help='json indent (default: %(default)s)')
    parser.add_argument('files', nargs='+', metavar='FILE')
    args = parser.parse_args(argv)

    # Every name is resolved before the first file is read, so an undecidable name in a long
    # list fails before any of the list has been decoded.
    try:
        streams = [resolve_stream(name, args.stream) for name in args.files]
    except ValueError as exc:
        parser.error(str(exc))

    if args.extract_h264:
        os.makedirs(args.extract_h264, exist_ok=True)
    schemas = load_schemas(args.schema_dir)
    decoded = [decode_file(name, stream, schemas, args.extract_h264, args.scan_ranges)
               for name, stream in zip(args.files, streams)]
    json.dump({'files': decoded}, sys.stdout, indent=args.indent)
    sys.stdout.write('\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
