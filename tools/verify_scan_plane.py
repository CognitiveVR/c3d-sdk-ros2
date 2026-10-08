#!/usr/bin/env python3
"""Check a captured session's LaserScan poses against the reader rule in docs/data-format.md.

    world = rotate(pose.rotation, (r cos t, r sin t, 0)) + pose.position,  t = angleMin + i * angleIncrement

That rule is what a consumer of the lidar stream has, and it holds only if the record carries the
SCAN-PLANE rotation. A camera frame on the same tf frame carries a different one -- the dynamic
rule, conjugated by the position map -- and reusing it draws the fan somewhere else while the row
it was drawn from looks perfectly well formed.

This runs against a capture directory rather than against the SDK: the pose that reaches the wire
is observable nowhere else, because the generated Cap'n Proto readers are PRIVATE to core/ and the
ament tests cannot decode a payload. It is the only thing in the tree that pins the CALL SITE,
`SessionRunner::RecordLaserScan`, rather than the conversion the call site is supposed to use.

Nothing here calls SDK code, by design. The expected world position of a ray is built from the tf
the harness published -- a rotation matrix written out from roll, pitch and yaw -- and mapped into
scene axes by the convention's position map, written out below. A verifier that asked the SDK
where the ray went would agree with it however wrong both were.

    python3 tools/verify_scan_plane.py --capture DIR --convention gltf_authored \\
        --frame sensor_mount --lidar scan --camera mount \\
        --mount-xyz 0.35,-0.12,0.47 --mount-rpy-deg 20,35,55

Standard library only. The decode is a subprocess call to tools/capnp_dump.py, which is where the
schemas and their one third-party dependency live.
"""
import argparse
import glob
import json
import math
import os
import subprocess
import sys

# ROS axes to scene axes, per pose_convention. Both of these are REFLECTIONS -- each has
# determinant -1 -- which is the whole reason a scan plane needs its own rotation: no quaternion
# expresses a mirror, so the honest sensor-to-scene map is not a rotation at all and the record
# carries the proper rotation that agrees with it on the scan plane.
POSITION_MAPS = {
    'rep103': lambda x, y, z: (x, y, z),
    'unity': lambda x, y, z: (-y, z, x),
    'gltf_authored': lambda x, y, z: (-x, z, -y),
}

# Under rep103 the position map is the identity, the two rules coincide exactly, and a pass that
# runs it proves nothing about which one the call site uses. That is not a limitation to work
# around, it is the reason this tool exists, so it refuses rather than passing vacuously.
REFLECTING_CONVENTIONS = ('unity', 'gltf_authored')

CAPNP_DUMP = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'capnp_dump.py')


def rotation_matrix_from_rpy_degrees(roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll), the ROS composition, multiplied out.

    A matrix rather than a quaternion on purpose: the harness published the transform as a
    quaternion, so an expectation built from a quaternion could share a mistake with it.
    """
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def apply_matrix(matrix, vector):
    return tuple(sum(row[i] * vector[i] for i in range(3)) for row in matrix)


def rotate(quaternion, vector):
    """`q v q*` for q = (x, y, z, w). What a reader of the record does, and nothing more."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # t = 2 (q_vec x v), then v + w t + q_vec x t.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def reader_endpoint(pose, ray_range, angle):
    """The reader rule: the ray in the sensor's own XY plane, through the record's own pose."""
    px, py, pz, qx, qy, qz, qw = pose
    naive = (ray_range * math.cos(angle), ray_range * math.sin(angle), 0.0)
    rx, ry, rz = rotate((qx, qy, qz, qw), naive)
    return (rx + px, ry + py, rz + pz)


def published_endpoint(matrix, translation, convention, ray_range, angle):
    """Where that ray's endpoint is, from the tf that was published and the axis map alone."""
    naive = (ray_range * math.cos(angle), ray_range * math.sin(angle), 0.0)
    turned = apply_matrix(matrix, naive)
    ros = tuple(turned[i] + translation[i] for i in range(3))
    return POSITION_MAPS[convention](*ros)


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def normalized(quaternion):
    """A unit quaternion. A wire quaternion is four float32s and is only nearly one."""
    norm = math.sqrt(sum(value * value for value in quaternion))
    return quaternion if norm == 0.0 else tuple(value / norm for value in quaternion)


def rotation_gap_degrees(first, second):
    """The angle between two rotations, in degrees.

    Through |dot|, so q and -q count as the same rotation: two rules that differ only in the
    sign of every component are the same rotation and must not be reported as a difference.
    Both are normalised first, because float32 rounding leaves a norm about 1e-7 off unity and
    the arc cosine turns that into a fiftieth of a degree -- enough for a record compared with
    ITSELF to read as a different rotation.
    """
    first, second = normalized(first), normalized(second)
    dot = abs(sum(first[i] * second[i] for i in range(4)))
    return 2.0 * math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def finite_rays(scan, wanted):
    """Up to `wanted` (index, range, angle) triples spread across one sweep, non-finite skipped.

    A driver publishes inf for a ray that returned nothing and NaN for one whose measurement is
    invalid, and the wire copies both as published, so a verifier that took ray 0 on faith would
    be reconstructing an infinity.
    """
    ranges = scan.get('ranges')
    if not ranges:
        return []
    # The finite rays first, THEN spread over those. Striding over every ray and dropping the
    # non-finite ones instead makes the count returned depend on where the sensor's dropouts
    # happen to land -- and this publisher's land every 128 rays, in step with any stride that
    # divides the sweep.
    finite = [index for index, value in enumerate(ranges)
              if math.isfinite(value) and value > 0.0]
    if not finite:
        return []
    stride = max(1, len(finite) // max(wanted, 1))
    return [(index, ranges[index], scan['angleMin'] + index * scan['angleIncrement'])
            for index in finite[::stride]][:wanted]


def decode(capture, capnp_dump=CAPNP_DUMP):
    """Every camera and lidar capture in `capture`, as one document, ranges included."""
    files = sorted(glob.glob(os.path.join(capture, '*-camera.bin')))
    files += sorted(glob.glob(os.path.join(capture, '*-lidar.bin')))
    if not files:
        raise SystemExit(f'{capture}: no *-camera.bin or *-lidar.bin captures to decode')
    output = subprocess.check_output(
        [sys.executable, capnp_dump, '--scan-ranges', *files])
    return json.loads(output)


def wire_convention(capture):
    """`c3d.ros.pose_convention` as the session actually reported it, or None.

    Session properties ride the gaze stream and only the gaze stream, so this is the one place a
    capture says which convention was in force -- and it is what makes a pass driven by a
    command-line override honest rather than merely configured.
    """
    for path in sorted(glob.glob(os.path.join(capture, '*-gaze.json'))):
        with open(path) as handle:
            body = json.load(handle).get('body', {})
        properties = body.get('properties') or {}
        if 'c3d.ros.pose_convention' in properties:
            return properties['c3d.ros.pose_convention']
    return None


def check_document(document, options, reported_convention=None):
    """Every assertion, as `(ok, message)` pairs. No I/O, so a unit test can call it directly."""
    results = []

    def check(condition, message):
        results.append((bool(condition), message))
        return bool(condition)

    convention = options.convention
    matrix = rotation_matrix_from_rpy_degrees(*options.mount_rpy_deg)
    translation = options.mount_xyz
    scene_origin = POSITION_MAPS[convention](*translation)

    entries = document['files']
    scans = [scan
             for entry in entries if entry['stream'] == 'lidar'
             for scan in entry['scans']
             if scan['kind'] == 'laserscan' and scan['frameId'] == options.frame
             and scan['sensorId'] == options.lidar]
    frames = [frame
              for entry in entries if entry['stream'] == 'camera'
              for frame in entry['frames']
              if frame['frameId'] == options.frame and frame['cameraId'] == options.camera]

    if reported_convention is not None:
        check(reported_convention == convention,
              f'the session reports pose_convention {reported_convention!r}, '
              f'and this pass is {convention!r}')
    if not check(scans, f"laserscan records for '{options.lidar}' on {options.frame}: {len(scans)}"):
        return results
    if not check(frames, f"camera frames for '{options.camera}' on {options.frame}: {len(frames)}"):
        return results

    # The mount is STATIC against the reference frame, so every record's position is the same
    # number and no record has to be paired with another by stamp for any of this.
    worst_position = max(distance(record['pose'][:3], scene_origin)
                         for record in scans + frames)
    check(worst_position <= options.tolerance_m,
          f'every record places the sensor at the mapped mount position '
          f'{tuple(round(v, 4) for v in scene_origin)} (worst off by {worst_position:.3e} m)')

    # (a) The two rules are different rotations. Necessary, and nowhere near sufficient: "not
    # equal" also passes for a rule that is wrong in some third way, which is what (b) is for.
    scan_rotation = scans[0]['pose'][3:]
    camera_rotation = frames[0]['pose'][3:]
    gap = rotation_gap_degrees(scan_rotation, camera_rotation)
    check(gap >= options.min_rotation_gap_deg,
          f'the laserscan rotation is not the camera rotation on the same frame: '
          f'{gap:.3f} degrees apart (want at least {options.min_rotation_gap_deg})')

    check(all(rotation_gap_degrees(record['pose'][3:], scan_rotation) <= 1e-2 for record in scans),
          'every laserscan record carries the same rotation, as a static mount must')

    # (b) The assertion that matters: the rule holds for real rays, against a world position
    # computed from the published tf and the axis map, with no SDK code in the expectation.
    worst_ray = 0.0
    checked = 0
    for scan in scans[:options.scans]:
        for _index, ray_range, angle in finite_rays(scan, options.rays):
            got = reader_endpoint(scan['pose'], ray_range, angle)
            want = published_endpoint(matrix, translation, convention, ray_range, angle)
            worst_ray = max(worst_ray, distance(got, want))
            checked += 1
    if not check(checked >= options.rays,
                 f'rays reconstructed: {checked} (want at least {options.rays})'):
        return results
    check(worst_ray <= options.tolerance_m,
          f'every ray lands where the published tf puts it: worst {worst_ray:.3e} m over '
          f'{checked} rays (tolerance {options.tolerance_m} m)')

    # The control. It fails only if the two rules have stopped disagreeing, in which case the
    # assertion above passes for a reason that has nothing to do with the call site.
    worst_wrong = 0.0
    for scan in scans[:options.scans]:
        wrong_pose = list(scan['pose'][:3]) + list(camera_rotation)
        for _index, ray_range, angle in finite_rays(scan, options.rays):
            got = reader_endpoint(wrong_pose, ray_range, angle)
            want = published_endpoint(matrix, translation, convention, ray_range, angle)
            worst_wrong = max(worst_wrong, distance(got, want))
    check(worst_wrong >= options.min_wrong_ray_m,
          f'the camera rotation would put those rays somewhere else: up to {worst_wrong:.3f} m '
          f'away (want at least {options.min_wrong_ray_m})')

    return results


def triple(text):
    parts = text.split(',')
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f'want three comma-separated numbers, got {text!r}')
    return tuple(float(part) for part in parts)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--capture', required=True, help='a stub_gateway capture directory')
    parser.add_argument('--convention', required=True, choices=REFLECTING_CONVENTIONS,
                        help='the pose_convention the session ran under. rep103 is not accepted: '
                             'its position map is the identity, so the two rules coincide and '
                             'the run could not tell them apart')
    parser.add_argument('--frame', required=True, help='the tf frame both sensors are on')
    parser.add_argument('--lidar', required=True, help='the configured LaserScan lidar name')
    parser.add_argument('--camera', required=True, help='the configured camera name')
    parser.add_argument('--mount-xyz', type=triple, required=True,
                        help='the published mount translation, metres, as x,y,z')
    parser.add_argument('--mount-rpy-deg', type=triple, required=True,
                        help='the published mount rotation, degrees, as roll,pitch,yaw')
    parser.add_argument('--rays', type=int, default=8, help='rays per scan (default: %(default)s)')
    parser.add_argument('--scans', type=int, default=3, help='scans (default: %(default)s)')
    # A pose is seven float32s on the wire and a range is another, so a metre of reconstruction
    # carries about a micrometre of representation error. A millimetre is loose enough to never
    # be the thing that fails and tight enough that a wrong rotation -- which misses by metres --
    # cannot hide under it.
    parser.add_argument('--tolerance-m', type=float, default=1e-3,
                        help='metres (default: %(default)s)')
    parser.add_argument('--min-rotation-gap-deg', type=float, default=5.0,
                        help='degrees the two rules must differ by (default: %(default)s)')
    parser.add_argument('--min-wrong-ray-m', type=float, default=0.1,
                        help='metres the camera rule must miss by (default: %(default)s)')
    parser.add_argument('--capnp-dump', default=CAPNP_DUMP,
                        help='tools/capnp_dump.py (default: %(default)s)')
    return parser.parse_args(argv)


def main(argv=None):
    options = parse_args(argv)
    document = decode(options.capture, options.capnp_dump)
    results = check_document(document, options, wire_convention(options.capture))
    failures = 0
    for ok, message in results:
        print(('  ok   ' if ok else '  FAIL ') + message)
        failures += 0 if ok else 1
    if failures:
        print(f'{failures} scan-plane assertion(s) FAILED under {options.convention}')
        return 1
    print(f'scan plane: every assertion passed under {options.convention}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
