#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# ///
"""Check that a session's stored Dynamic Object rotations render the robot level and facing its path.

Only for a session recorded with `robot_dynamic_object: true`. That parameter defaults to false,
and Session Replay then draws the robot from the pose timeline, so a default session stores no
Dynamic Object samples and this tool exits `FAIL no dynamics samples stored`. That failure reports
the configuration, not the session. `verify_gaze_contract.py` checks a default session.

Two checks, both in the frame Session Replay draws in, because either alone passes a real defect:

  Level    every stored rotation keeps the mesh's up axis within --max-degrees of world up.
  Heading  the mesh faces along the robot's own path. The level check cannot see a turn about
           the up axis, so a robot rendered backwards or sideways is still level. The heading
           check needs --minimum-forward-segments segments of forward motion, and a drive with
           fewer FAILS rather than passing with the heading unchecked.

The mesh's forward axis is glTF +X once `mesh_yaw_offset_deg` is applied. A mesh authored to face
another axis needs that parameter, and this check then measures the corrected result.

  ros2 run cognitive3d_ros verify_dynamics_level.py <session-id> [--project <id>]
  uv run tools/verify_dynamics_level.py <session-id> [--project <id>]     # from a source checkout

What a PASS does not establish:

  - Both checks compare the session with itself, so a planar pose source passes them: it renders
    level and along its own path because nothing in it leaves the plane. `doctor`'s pose channel
    check says whether the pose can carry height and tilt.
  - Heights are never checked. Only rotations are read.

What a FAIL does not establish:

  - A drive that mostly reverses fails like a mesh turned around (--max-reversing-share). Give the
    drive a turn-around rather than raising the bound, which would also let a mirrored mesh
    through.
  - The level check has no notion of terrain, so a correct climb up a slope and a broken up axis
    fail alike. On a robot that leaves the horizontal plane, a FAIL is a prompt to look in Session
    Replay.

It reads the session through an API the Cognitive3D dashboard uses, which may change without
notice. Credentials and hosts come from the environment: C3D_PROJECT_ID, C3D_ORG_API_KEY, and
C3D_DATA_HOST / C3D_API_HOST for a stack other than production.
"""
from __future__ import annotations

import argparse
import math

from c3d_api import env as c3denv, http as c3dhttp
# The sibling verifier's helper, not a second copy of it. An arbitrary non-unit quaternion does
# not name a rotation, and `rotate` would SCALE rather than turn -- so a genuine 3 degree tilt
# stored at norm 0.1 reads 0.03 and passes, as do [0,0,0,0] and all-NaN.
from verify_gaze_contract import normalized_rotation

GLTF_UP = (0.0, 1.0, 0.0)
MESH_FORWARD = (1.0, 0.0, 0.0)


def is_registration_record(row):
    """`c3d::Session::RegisterDynamic`'s one-off object record, which is NOT a pose measurement.

    The SDK announces the object once per session with a default-constructed Vec3 and Quat, so that
    entry carries a HARD-CODED origin and identity rotation next to `s` and `properties`. Its
    position is the map origin rather than wherever the robot spawned, so a segment drawn from it
    measures the session's distance from the origin -- which is metres on any run that does not
    start at the origin, and reads as a heading error rather than as an artefact.

    Identified by `properties`, NOT by `s`. `Session::RegisterDynamic` always emplaces an
    `enabled` property, and `Session::RecordDynamic` takes both scale and properties as OPTIONAL
    arguments -- so on a consumer that passes neither, `properties` is present on exactly the
    announcement. Keying on `s` as well looks equivalent and is not: a consumer that records the
    object's scale on every snapshot (a legal `RecordDynamic` call) would have EVERY pose discarded
    as an announcement, and this tool would report that a valid session measured nothing.
    """
    return 'properties' in row


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def angle_degrees(a, b):
    dot = sum(i * j for i, j in zip(a, b))
    na = math.sqrt(sum(i * i for i in a))
    nb = math.sqrt(sum(i * i for i in b))
    if na == 0.0 or nb == 0.0:
        return float('nan')
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / (na * nb)))))


# Session Replay transforms what it is sent before drawing it, and both checks are computed after
# that transform: compared in the WIRE frame, whose X the position map has already negated, a robot
# driving backwards reads 0.000 degrees. These copies of the viewer's transform must change with it,
# because this tool is standard-library only and cannot import it.
def render_rotation(q):
    """What Session Replay draws with, from what we sent."""
    return (q[0], -q[1], -q[2], q[3])


def render_position(p):
    """What Session Replay draws at, from what we sent."""
    return (-p[0], p[1], p[2])


def fetch(environment, project_id, session_id, kind):
    url = (f'{environment.api_host}/v0/projects/{project_id}'
           f'/sessions/{session_id}/jsonRequests')
    status, body = c3dhttp.post_json(
        url, c3denv.auth_header(environment.org_key), {'jsonType': kind})
    # The platform's answer for a session that stored none of `kind`.
    if status == 404:
        return []
    if status == 0:
        raise SystemExit(f'FAIL {session_id}: no response from jsonRequests -- {body}')
    # Described by its shape, never echoed: an error body can quote the request it refused.
    if status != 200:
        raise SystemExit(f'FAIL {session_id}: HTTP {status} from jsonRequests '
                         f'-- got {c3dhttp.shape(body)}')
    if body is not None and not isinstance(body, dict):
        raise SystemExit(f'FAIL {session_id}: HTTP 200 from jsonRequests but the body is not a JSON '
                         f'object -- got {c3dhttp.shape(body)}. A captive portal or intercepting '
                         f'proxy answers exactly like this')
    rows = []
    for entries in ((body or {}).get('data') or {}).values():
        if isinstance(entries, list):
            rows.extend(e for e in entries if isinstance(e, dict))
    return rows


def heading_errors(rows, minimum_travel_meters):
    """Classify every segment by the angle between the rendered mesh forward and the
    rendered direction of travel. Returns (forward_degrees, reversing_degrees).

    Only over segments long enough to have a direction: a stationary robot has no
    heading, and normalising a near-zero displacement amplifies float noise into
    arbitrary angles. Consecutive samples at 10 Hz barely move, so segments are
    grown until they clear the bound.

    REVERSING IS CLASSIFIED, NOT FOLDED AND NOT AVERAGED IN. A TurtleBot 4 undocks
    by reversing and Nav2 backs up during a recovery, so a correctly oriented robot
    really does produce ~180 degree segments, and a correct session can pass with a
    worst segment of 180. Putting them in one distribution makes a
    recovery-heavy session fail as an inverted render, and folding the angle
    (`min(t, 180 - t)`) would score a genuinely inverted render as perfect, which is
    the defect this check exists to catch.

    SEGMENTS ARE GROWN PER OBJECT. A session may carry several dynamic objects and the API returns
    them interleaved in one stream, so a flat trajectory reads the jump BETWEEN two objects as
    motion: two stationary, correctly oriented objects a metre apart yield half their segments
    along +X and half along -X, a 50% reversing share on a session where nothing moved. A session
    read back from the platform carrying several objects is ordinary, not an error.
    """
    by_object: dict[str, list] = {}
    for row in rows:
        if 'p' not in row or 'time' not in row or is_registration_record(row):
            continue
        quaternion = normalized_rotation(row.get('r'))
        if quaternion is None:
            continue
        by_object.setdefault(str(row.get('id', '')), []).append(
            (float(row['time']), render_position(row['p']), quaternion))

    forward_degrees, reversing_degrees = [], []
    for usable in by_object.values():
        usable.sort(key=lambda u: u[0])
        anchor = 0
        for index in range(1, len(usable)):
            a, b = usable[anchor][1], usable[index][1]
            travel = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            if math.sqrt(sum(c * c for c in travel)) < minimum_travel_meters:
                continue
            middle = usable[(anchor + index) // 2]
            nose = rotate(render_rotation(middle[2]), MESH_FORWARD)
            angle = angle_degrees(nose, travel)
            (reversing_degrees if angle > 90.0 else forward_degrees).append(angle)
            anchor = index
    return forward_degrees, reversing_degrees


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('session_id')
    parser.add_argument('--project', type=int, default=None, help='defaults to C3D_PROJECT_ID')
    parser.add_argument('--max-degrees', type=float, default=1.0,
                        help='worst tolerated tilt of the up axis (default 1.0)')
    parser.add_argument('--max-heading-degrees', type=float, default=25.0,
                        help='p95 tolerated angle between the mesh forward and the path. Loose on '
                             'purpose: a real turn puts the chord off the instantaneous heading, '
                             'and the defects this catches are 90 and 180 degrees (default 25)')
    parser.add_argument('--minimum-travel-meters', type=float, default=0.10,
                        help='shortest segment that counts as having a direction (default 0.10)')
    parser.add_argument('--max-unusable-fraction', type=float, default=0.01,
                        help='fraction of samples allowed to carry no usable rotation. A sample that '
                             'cannot be checked is not a sample that passed (default 0.01)')
    parser.add_argument('--minimum-forward-segments', type=int, default=20,
                        help='forward-motion segments the heading check needs to mean anything. '
                             'Below this the run FAILS rather than passing with the heading '
                             'unchecked -- a contract check that examined nothing is not a pass '
                             '(default 20)')
    parser.add_argument('--max-reversing-share', type=float, default=0.25,
                        help='fraction of classified segments allowed to render the mesh travelling '
                             'backwards. Reversing is legitimate (undocking, Nav2 recovery) so it is '
                             'kept out of the angle threshold, but an inverted render makes nearly '
                             'every segment reverse, so it cannot be ungated (default 0.25)')
    arguments = parser.parse_args(argv)

    environment = c3denv.require_environment()
    project_id = arguments.project if arguments.project is not None else environment.project_id
    rows = fetch(environment, project_id, arguments.session_id, 'DYNAMICS')
    print(f'session {arguments.session_id} on project {project_id}')
    if not rows:
        raise SystemExit(
            'FAIL no dynamics samples stored -- nothing to verify (a run with robot_dynamic_object '
            'off, the default, records none; this verifier is for runs that switch it on, so for '
            'one that did, check the session id and the project)')

    tilts = []
    unusable = 0
    registrations = 0
    for row in rows:
        if is_registration_record(row):
            registrations += 1
            continue
        quaternion = normalized_rotation(row.get('r'))
        if quaternion is None:
            unusable += 1
            continue
        tilts.append(angle_degrees(rotate(render_rotation(quaternion), GLTF_UP), GLTF_UP))
    measurements = len(rows) - registrations
    if measurements <= 0:
        raise SystemExit(f'FAIL {len(rows)} row(s), all of them object registrations rather than '
                         f'pose measurements -- nothing to verify')
    if not tilts:
        raise SystemExit(f'FAIL {measurements} samples, none carried a usable rotation')

    unusable_fraction = unusable / measurements
    tilts.sort()
    n = len(tilts)
    print(f'  samples : {len(rows)} total, {registrations} object registration(s) excluded, '
          f'{n} checked, {unusable} unusable ({100.0 * unusable_fraction:.2f}%, threshold '
          f'{100.0 * arguments.max_unusable_fraction:.2f}%)')
    print('  1. LEVEL   angle from the rendered up axis to world up:')
    print(f'       median {tilts[n // 2]:.4f}   p95 {tilts[int(0.95 * n)]:.4f}   '
          f'worst {tilts[-1]:.4f} deg   (threshold {arguments.max_degrees})')

    failures = []
    if unusable_fraction > arguments.max_unusable_fraction:
        failures.append(f'{unusable} of {measurements} samples carry no usable rotation '
                        f'({100.0 * unusable_fraction:.2f}%)')
    if tilts[-1] > arguments.max_degrees:
        over = sum(1 for t in tilts if t > arguments.max_degrees)
        failures.append(f'{over} of {n} samples tip the mesh past {arguments.max_degrees} deg, so '
                        f'it renders on its side and partly under the floor')

    forward, reversing = heading_errors(rows, arguments.minimum_travel_meters)
    classified = len(forward) + len(reversing)
    reversing_share = len(reversing) / classified if classified else 0.0
    print(f'  2. HEADING angle from the rendered mesh forward to the path, '
          f'{classified} segment(s) of at least {arguments.minimum_travel_meters} m '
          f'({len(forward)} forward, {len(reversing)} reversing, share '
          f'{reversing_share:.1%}, bound {arguments.max_reversing_share:.0%}):')
    forward.sort()
    reversing.sort()
    if forward:
        f = len(forward)
        forward_p95 = forward[int(0.95 * f)]
        print(f'       forward   median {forward[f // 2]:.3f}   p95 {forward_p95:.3f}   '
              f'worst {forward[-1]:.3f} deg   (p95 threshold {arguments.max_heading_degrees})')
    if reversing:
        r = len(reversing)
        print(f'       reversing median {reversing[r // 2]:.3f}   worst {reversing[-1]:.3f} deg   '
              f'(reported, not thresholded: undocking and Nav2 recovery really do reverse)')

    # A HEADING CHECK THAT EXAMINED NOTHING IS NOT A PASS. Without this the tool would print
    # "HEADING skipped" and then "PASS ... and it faces its path", certifying the very defect
    # it exists to catch: a robot that moves 0.09 m with its level mesh turned 180 degrees
    # away clears the tilt check, produces no segment, and exits 0.
    if len(forward) < arguments.minimum_forward_segments:
        failures.append(f'only {len(forward)} forward-motion segment(s) of at least '
                        f'{arguments.minimum_travel_meters} m, need '
                        f'{arguments.minimum_forward_segments}. The heading was NOT checked, so '
                        f'this session says nothing about whether the mesh faces its path -- did '
                        f'the robot actually drive?')
    elif forward[int(0.95 * len(forward))] > arguments.max_heading_degrees:
        failures.append(f'the mesh faces {forward[int(0.95 * len(forward))]:.1f} deg off its own '
                        f'path at the 95th percentile of {len(forward)} forward segments (median '
                        f'{forward[len(forward) // 2]:.1f}). Near 180 means it renders driving '
                        f'backwards; near 90 means a crab. Check mesh_yaw_offset_deg, and that '
                        f'the dynamics rotation is conjugated by the position map mirror')
    if reversing_share > arguments.max_reversing_share:
        failures.append(f'{len(reversing)} of {classified} segments render the mesh travelling '
                        f'BACKWARDS ({reversing_share:.1%}, bound '
                        f'{arguments.max_reversing_share:.0%}). Reversing that pervasive is an '
                        f'inverted render, not a manoeuvre')

    if failures:
        for reason in failures:
            print(f'  FAIL: {reason}')
        return 1
    print(f'  PASS: {n} rotations render the mesh level, and {len(forward)} forward segments '
          f'face their path')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
