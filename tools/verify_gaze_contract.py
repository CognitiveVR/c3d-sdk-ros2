#!/usr/bin/env python3
"""Check that a stored session's gaze samples are what this SDK sends, and read its rotations.

A gaze sample this SDK sends is a time, a position p and a rotation r, and nothing else. It carries
no gaze point g: a robot has no eyes, and the platform counts every gaze point it is sent in its
gaze analytics. Session Replay places the robot at p and turns it by r, read as a headset's
rotation: local forward and up, per --pose-convention. This reads what the platform STORED, not
what the node believes it sent, so it also catches a payload changed on the way in.

What it checks, on every sample:

  Shape    p is three finite numbers and r a unit quaternion. A sample that is not is unusable, and
           more than --max-unusable-fraction of them FAILS the run.
  No g     a sample carrying a gaze point FAILS the run.
  Level    the angle from r's up axis to world up. Reported; FAILS the run only with
           --require-level, for a gaze frame that stays level, such as a ground robot's body. A
           `gaze_frame` that legitimately pitches or rolls fails it.
  Heading  the angle from r's forward axis to the direction p travels, on the horizontal, over
           segments of at least --minimum-travel-meters. Reported; FAILS the run only with
           --require-heading, and then also when the robot drove fewer than
           --minimum-forward-segments segments forward. A gaze frame that turns away from the path,
           such as a camera on a pan/tilt head, fails it.

What a PASS does not prove. Every check compares the session with itself, so a session recorded
under the wrong pose convention passes: only Session Replay shows a path off the floor plan.

Usage:
    ros2 run cognitive3d_ros verify_gaze_contract.py <session-id>
        --pose-convention unity|gltf_authored|rep103 [--project <id>]
        [--require-level] [--require-heading]

From a source checkout, `uv run tools/verify_gaze_contract.py` takes the same arguments.

It reads the session through an API the Cognitive3D dashboard uses, which may change without
notice. Credentials and hosts come from the environment: C3D_PROJECT_ID, C3D_ORG_API_KEY, and
C3D_DATA_HOST / C3D_API_HOST for a stack other than production. The organization key leaves this
process only as an HTTP header.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

from c3d_api import env as c3denv, http as c3dhttp


def rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    """q * (v, 0) * q^-1 -- the ordinary rotation of a vector by a quaternion.

    The caller MUST pass a unit quaternion: this is the optimised form, which is only a rotation when
    |q| == 1. Feeding it a non-unit q silently scales the result, and the scaled vector then dots to
    more than 1 against a unit ray -- which the caller's clamp turns into a reported 0 degrees, so a
    real 1-degree error passes. `normalized_rotation` is the guard.
    """
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty,
            v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx)


def normalized_rotation(r) -> tuple[float, float, float, float] | None:
    """`r` as a unit quaternion, or None if it is not a usable rotation.

    A stored `r` is float32, so it is never exactly unit -- normal float32 rounding deviates by up
    to about 4e-8, which is fine. What is NOT fine is accepting an arbitrary non-unit quaternion: it
    does not name a rotation, and `rotate` would scale rather than rotate. The 1e-3 tolerance admits
    float32 rounding by five orders of magnitude and rejects everything else.
    """
    # isinstance BEFORE len(), the same guard vector3 has. Without it a stored
    # `"r": 3.5` passes the truthiness test and len() raises TypeError, killing
    # the run after thousands of samples instead of counting one unusable one.
    if not isinstance(r, (list, tuple)) or len(r) != 4:
        return None
    try:
        values = [float(c) for c in r]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(c) for c in values):
        return None
    norm = math.sqrt(sum(c * c for c in values))
    if norm < 1e-9 or abs(norm - 1.0) > 1e-3:
        return None
    return tuple(c / norm for c in values)


def vector3(v) -> tuple[float, float, float] | None:
    """`v` as three finite floats, or None if it is not a usable point.

    A truthiness check alone lets a stored `{"p": [0, 0]}` reach `p[2]` and
    kill the whole run with an IndexError instead of being counted as one
    unusable sample. The platform accepts malformed payloads with HTTP 200, so
    malformed samples coming back is the expected case, not the exceptional one.
    """
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        return None
    try:
        values = [float(c) for c in v]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(c) for c in values):
        return None
    return tuple(values)


# The local forward axis and the world up axis, per wire convention. These are NOT
# universal: `unity` is the platform's own basis, where the change of basis has
# mapped ROS +X forward onto +Z and ROS +Z up onto +Y. On the `rep103` wire no
# change of basis is applied at all, so REP-103's own axes survive -- forward is
# +X and up is +Z.
#
# Hardcoding the unity pair would report EVERY sample of a correct rep103 session
# as exactly 90 degrees wrong, under the verdict "the session renders pointing the
# wrong way" -- the precise catastrophe this tool exists to detect, on a session
# that is right. `pose_convention: rep103` is a supported, documented option, and
# the C++ gaze tests parameterise the axis for the same reason.
AXES = {
    'unity': {'forward': (0.0, 0.0, 1.0), 'up': (0.0, 1.0, 0.0), 'up_component': 1},
    'rep103': {'forward': (1.0, 0.0, 0.0), 'up': (0.0, 0.0, 1.0), 'up_component': 2},
    # Identical to unity, and that is the substantive claim rather than a
    # copy-paste: gltf_authored differs from unity in its POSITION map, and its
    # gaze rotation carries a fixed turn chosen precisely so the sent basis
    # reads as the platform reads a gaze rotation -- local +Z forward, +Y up. If
    # this row ever needs to differ from unity's, the C++ gaze rule is wrong.
    'gltf_authored': {'forward': (0.0, 0.0, 1.0), 'up': (0.0, 1.0, 0.0), 'up_component': 1},
}


@dataclass(frozen=True)
class Checked:
    """One usable gaze sample, read in its wire basis."""
    time: float | None
    position: tuple[float, float, float]
    forward: tuple[float, float, float]
    up_angle: float
    carries_gaze_point: bool


def angle_degrees(a, b) -> float | None:
    """The angle between two vectors in degrees, or None when either has no length."""
    la = math.sqrt(sum(c * c for c in a))
    lb = math.sqrt(sum(c * c for c in b))
    if la < 1e-9 or lb < 1e-9:
        return None
    # The clamp only absorbs floating-point overshoot on parallel vectors, never a real error.
    dot = sum(a[i] * b[i] for i in range(3)) / (la * lb)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def check_sample(sample: dict, convention: str) -> tuple[Checked | None, str | None]:
    """(the sample read, None) or (None, why it cannot be checked).

    A sample that cannot be checked contributes nothing to any verdict, so it can never silently
    count towards a PASS. `convention` selects the wire basis; see AXES.
    """
    axes = AXES[convention]
    raw_p, r = sample.get('p'), sample.get('r')
    if raw_p is None or r is None:
        return None, 'missing ' + ','.join(n for n, v in (('p', raw_p), ('r', r)) if v is None)
    p = vector3(raw_p)
    if p is None:
        return None, f'p={raw_p} is not a usable 3-vector'
    rotation = normalized_rotation(r)
    if rotation is None:
        return None, f'r={r} is not a usable unit quaternion'
    forward = rotate(rotation, axes['forward'])
    up = rotate(rotation, axes['up'])
    # The component of the rotated local up that lies along WORLD up.
    up_angle = math.degrees(math.acos(max(-1.0, min(1.0, up[axes['up_component']]))))
    try:
        time = float(sample['time'])
        if not math.isfinite(time):
            time = None
    except (KeyError, TypeError, ValueError):
        time = None
    return Checked(time, p, forward, up_angle, sample.get('g') is not None), None


def heading_errors(samples: list[Checked], convention: str,
                   minimum_travel_meters: float) -> tuple[list[float], list[float]]:
    """(forward degrees, reversing degrees): the horizontal angle from r's forward axis to the
    direction of travel, per segment of at least `minimum_travel_meters`.

    Segments are grown from consecutive samples, in time order, until they clear the bound: a
    robot standing still has no direction, and normalising a near-zero step turns float noise into
    arbitrary angles. Reversing is classified rather than averaged in, because a correctly posed
    robot really does reverse -- undocking, a Nav2 recovery -- while a rotation turned 180 degrees
    reverses on every segment. A sample with no usable time cannot be ordered and is left out.
    """
    up = AXES[convention]['up_component']

    def horizontal(v):
        return [0.0 if i == up else v[i] for i in range(3)]

    timed = sorted((s for s in samples if s.time is not None), key=lambda s: s.time)
    forward_degrees, reversing_degrees = [], []
    anchor = 0
    for index in range(1, len(timed)):
        a, b = timed[anchor].position, timed[index].position
        travel = horizontal([b[i] - a[i] for i in range(3)])
        if math.sqrt(sum(c * c for c in travel)) < minimum_travel_meters:
            continue
        nose = horizontal(timed[(anchor + index) // 2].forward)
        angle = angle_degrees(nose, travel)
        anchor = index
        if angle is None:
            continue  # facing straight up or down: no heading to compare
        (reversing_degrees if angle > 90.0 else forward_degrees).append(angle)
    return forward_degrees, reversing_degrees


def fetch_gaze(environment: c3denv.C3dEnvironment, session_id: str, project_id: int) -> list[dict]:
    """The stored gaze samples. `data` comes back keyed by scene versionId, not as a flat list."""
    status, body = c3dhttp.post_json(
        f'{environment.api_host}/v0/projects/{project_id}/sessions/{session_id}/jsonRequests',
        c3denv.auth_header(environment.org_key), {'jsonType': 'GAZE'})
    if status == 0:
        raise SystemExit(f'FAIL {session_id}: no response from jsonRequests -- {body}')
    # Described by its shape, never echoed: an error body can quote the request it refused.
    if status != 200:
        raise SystemExit(f'FAIL {session_id}: HTTP {status} from jsonRequests -- got '
                         f'{c3dhttp.shape(body)}')
    # A 200 carrying something other than a JSON object is not this API answering.
    # It is a captive portal or an intercepting proxy, both routine on site wifi,
    # and calling .get() on the string it returns would raise AttributeError -- a
    # traceback where the diagnosis belongs.
    if not isinstance(body, dict):
        raise SystemExit(f'FAIL {session_id}: HTTP 200 from jsonRequests but the body is not a JSON '
                         f'object -- got {c3dhttp.shape(body)}. A captive portal or intercepting '
                         f'proxy answers exactly like this')
    samples: list[dict] = []
    for value in (body.get('data') or {}).values():
        if isinstance(value, list):
            samples.extend(value)
    return samples


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('session_id')
    parser.add_argument('--project', type=int, default=None, help='defaults to C3D_PROJECT_ID')
    parser.add_argument('--pose-convention', choices=sorted(AXES), required=True,
                        help="the wire basis the session was recorded in: the node's "
                             "pose_convention, which has no default, so neither does this. A rep103 "
                             "session has no change of basis applied, so its forward axis is +X and "
                             "its up axis is +Z; checking it as unity reads its up axis as forward, "
                             "and every heading segment 90 degrees off. The session records which in "
                             "the property `c3d.ros.pose_convention`.")
    parser.add_argument('--max-up-degrees', type=float, default=5.0,
                        help='the angle between r\'s up axis and world up above which the level '
                             'line is called out (default 5.0). Only FAILS the run with '
                             '--require-level.')
    parser.add_argument('--require-level', action='store_true',
                        help='make the up-axis angle fatal above --max-up-degrees. It is only sound '
                             'for a gaze frame that stays level: a `gaze_frame` pitched 15 degrees '
                             'reads as 15 degrees of error, which is a supported configuration and '
                             'not a defect. Pass this for a ground robot whose gaze frame is its '
                             'body.')
    parser.add_argument('--require-heading', action='store_true',
                        help='make the heading fatal: the 95th percentile of the forward segments '
                             'above --max-heading-degrees, more than --max-reversing-share of the '
                             'segments reversing, or fewer than --minimum-forward-segments forward '
                             'segments. Only sound for a gaze frame that faces the way the robot '
                             'drives, such as a ground robot\'s body.')
    parser.add_argument('--max-heading-degrees', type=float, default=25.0,
                        help='p95 tolerated angle between r\'s forward axis and the path. Loose on '
                             'purpose: a real turn puts the chord off the instantaneous heading, and '
                             'the defects this catches are 90 and 180 degrees (default 25)')
    parser.add_argument('--minimum-travel-meters', type=float, default=0.10,
                        help='shortest segment that counts as having a direction (default 0.10)')
    parser.add_argument('--minimum-forward-segments', type=int, default=20,
                        help='forward segments the heading check needs to mean anything; with '
                             '--require-heading a drive with fewer FAILS (default 20)')
    parser.add_argument('--max-reversing-share', type=float, default=0.25,
                        help='fraction of segments allowed to reverse. Reversing is legitimate '
                             '(undocking, Nav2 recovery), but a rotation turned 180 degrees '
                             'reverses on nearly every segment (default 0.25)')
    parser.add_argument('--max-unusable-fraction', type=float, default=0.01,
                        help='largest tolerated fraction of samples that cannot be checked '
                             '(default 0.01). A sample that cannot be checked is not a sample that '
                             'passed.')
    args = parser.parse_args(argv)

    environment = c3denv.require_environment()
    project_id = args.project if args.project is not None else environment.project_id
    samples = fetch_gaze(environment, args.session_id, project_id)
    if not samples:
        raise SystemExit(f'FAIL {args.session_id}: the platform returned no gaze samples')

    checked: list[Checked] = []
    unusable: list[str] = []
    for index, sample in enumerate(samples):
        result, reason = check_sample(sample if isinstance(sample, dict) else {},
                                      args.pose_convention)
        if reason is not None:
            unusable.append(f'sample {index}: {reason}')
            continue
        checked.append(result)

    if not checked:
        raise SystemExit(f'FAIL {args.session_id}: no sample carried a usable p and r')

    unusable_fraction = len(unusable) / len(samples)
    axes = AXES[args.pose_convention]
    print(f'session {args.session_id} on project {project_id}, checked as '
          f'{args.pose_convention} (local forward {axes["forward"]}, up {axes["up"]})')
    print(f'  samples          : {len(samples)} total, {len(checked)} checked, {len(unusable)} '
          f'unusable ({100.0 * unusable_fraction:.2f}%, threshold '
          f'{100.0 * args.max_unusable_fraction:.2f}%)')
    for reason in unusable[:3]:
        print(f'    - {reason}')
    if len(unusable) > 3:
        print(f'    - ... and {len(unusable) - 3} more')

    failures: list[str] = []
    # An unusable sample is not a sample that passed. Counting only the checkable ones would let a
    # session of 4999 malformed samples plus one good one PASS -- precisely what this tool exists to
    # catch, given the collector answers 200 to malformed payloads.
    if unusable_fraction > args.max_unusable_fraction:
        failures.append(f'{len(unusable)} of {len(samples)} samples ({100.0 * unusable_fraction:.2f}%) '
                        f'cannot be checked; the verdict covers only the {len(checked)} that can')

    with_point = [c for c in checked if c.carries_gaze_point]
    print(f'  gaze point g     : on {len(with_point)} of {len(checked)} samples (none expected)')
    if with_point:
        failures.append(f'{len(with_point)} of {len(checked)} samples carry a gaze point g, which '
                        f'this SDK never sends')

    up_angles = [c.up_angle for c in checked]
    up_worst = max(up_angles)
    print(f'  level            : angle from r\'s up axis to world up, median '
          f'{percentile(up_angles, 0.5):.4f}   worst {up_worst:.4f} deg   '
          f'({"fatal" if args.require_level else "reported only"} above {args.max_up_degrees})')
    if up_worst > args.max_up_degrees:
        if args.require_level:
            over = sum(1 for a in up_angles if a > args.max_up_degrees)
            failures.append(f'{over} of {len(up_angles)} samples have r\'s up axis more than '
                            f'{args.max_up_degrees} deg from world up (worst {up_worst:.4f}). '
                            f'--require-level was passed, so this is fatal; drop it if the gaze '
                            f'frame legitimately tilts')
        else:
            print('    NOTE the gaze frame is not level. Expected when `gaze_frame` names a pitched '
                  'or rolled frame (see the session property `c3d.ros.gaze_frame`). If that '
                  'does not apply, re-run with --require-level to make it fatal.')

    forward, reversing = heading_errors(checked, args.pose_convention, args.minimum_travel_meters)
    classified = len(forward) + len(reversing)
    reversing_share = len(reversing) / classified if classified else 0.0
    print(f'  heading          : angle from r\'s forward axis to the path, {classified} segment(s) '
          f'of at least {args.minimum_travel_meters} m ({len(forward)} forward, {len(reversing)} '
          f'reversing, share {reversing_share:.1%}, bound {args.max_reversing_share:.0%}; '
          f'{"fatal" if args.require_heading else "reported only"})')
    if forward:
        print(f'    forward   median {percentile(forward, 0.5):.3f}   '
              f'p95 {percentile(forward, 0.95):.3f}   worst {max(forward):.3f} deg   '
              f'(p95 threshold {args.max_heading_degrees})')
    if reversing:
        print(f'    reversing median {percentile(reversing, 0.5):.3f}   worst {max(reversing):.3f} deg')
    heading_problems: list[str] = []
    # A heading check that examined nothing is not a pass.
    if len(forward) < args.minimum_forward_segments:
        heading_problems.append(f'only {len(forward)} forward segment(s) of at least '
                                f'{args.minimum_travel_meters} m, need '
                                f'{args.minimum_forward_segments}: the heading was not checked. '
                                f'Did the robot drive?')
    elif forward and percentile(forward, 0.95) > args.max_heading_degrees:
        heading_problems.append(f'r faces {percentile(forward, 0.95):.1f} deg off the path at the '
                                f'95th percentile of {len(forward)} forward segments. Near 90 '
                                f'means a sideways rotation; check pose_convention and '
                                f'gaze_frame_convention')
    if reversing_share > args.max_reversing_share:
        heading_problems.append(f'{len(reversing)} of {classified} segments travel against r '
                                f'({reversing_share:.1%}, bound {args.max_reversing_share:.0%}): '
                                f'reversing that pervasive is a rotation turned 180 degrees')
    if args.require_heading:
        failures.extend(heading_problems)
    else:
        for problem in heading_problems:
            print(f'    NOTE {problem}')

    if failures:
        for failure in failures:
            print(f'  FAIL: {failure}')
        return 1
    print(f'  PASS: {len(checked)} samples carry a usable position and rotation and no gaze point'
          f'{", level" if args.require_level else ""}'
          f'{", facing the path" if args.require_heading else ""}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
