"""Unit tests for tools/verify_gaze_contract.py -- no network, no credentials.

Run: uv run tools/test_verify_gaze_contract.py

The fixtures build the WIRE form from ROS inputs locally rather than calling the tool's own
transforms, so a change to those transforms moves the code under test and not the expectation.
"""
import contextlib
import io
import math
import unittest
from unittest import mock

import verify_gaze_contract as VGC
from verify_gaze_contract import (AXES, check_sample, heading_errors, normalized_rotation, rotate,
                                  vector3)

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def quat_about_y(degrees: float) -> tuple[float, float, float, float]:
    """A unit quaternion for a rotation of `degrees` about the +Y (up) axis."""
    half = math.radians(degrees) / 2.0
    return (0.0, math.sin(half), 0.0, math.cos(half))


def quat_about_x(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) / 2.0
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def ros_yaw(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def unity_rotation(q):
    """The unity gaze rule, `(-y, z, x, -w)`, written out."""
    return [-q[1], q[2], q[0], -q[3]]


def unity_position(v):
    return [-v[1], v[2], v[0]]


def drive(count=60, step=0.05, heading_degrees=0.0, facing_degrees=None, convention='unity'):
    """A level robot driving a straight line along `heading_degrees` (ROS yaw) at `step` m per
    sample, its rotation facing `facing_degrees` (the heading, unless given), as stored."""
    facing = heading_degrees if facing_degrees is None else facing_degrees
    q = ros_yaw(facing)
    direction = (math.cos(math.radians(heading_degrees)), math.sin(math.radians(heading_degrees)))
    rows = []
    for i in range(count):
        ros_p = (direction[0] * step * i, direction[1] * step * i, 0.0)
        if convention == 'unity':
            rows.append({'time': 1790000000.0 + 0.1 * i, 'p': unity_position(ros_p),
                         'r': unity_rotation(q)})
        else:
            rows.append({'time': 1790000000.0 + 0.1 * i, 'p': list(ros_p), 'r': list(q)})
    return rows


def checked_rows(rows, convention='unity'):
    return [check_sample(row, convention)[0] for row in rows]


class TestRotate(unittest.TestCase):
    def test_identity_leaves_vector_unchanged(self):
        result = rotate(IDENTITY, (0.0, 0.0, 1.0))
        for actual, expected in zip(result, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected)

    def test_ninety_degrees_about_up_sends_forward_to_local_x(self):
        result = rotate(quat_about_y(90.0), (0.0, 0.0, 1.0))
        for actual, expected in zip(result, (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=6)


class TestNormalizedRotation(unittest.TestCase):
    def test_unit_quaternion_is_returned_normalized(self):
        result = normalized_rotation(IDENTITY)
        self.assertIsNotNone(result)
        for actual, expected in zip(result, IDENTITY):
            self.assertAlmostEqual(actual, expected)

    def test_float32_rounded_unit_quaternion_is_accepted(self):
        # A component nudged by 1e-4 is well inside the 1e-3 tolerance float32 storage needs.
        self.assertIsNotNone(normalized_rotation((1e-4, 0.0, 0.0, 1.0)))

    def test_a_quaternion_inside_the_tolerance_is_actually_normalized(self):
        # Accepting a slightly-off quaternion is only safe because it is then SCALED to unit
        # length. Skip that and `rotate` returns a vector 5e-4 too long, and the level check, which
        # reads one component of it as a cosine, reads a real tilt as less than it is.
        nudged = tuple(c * 1.0005 for c in IDENTITY)
        self.assertAlmostEqual(math.sqrt(sum(c * c for c in nudged)), 1.0005, places=6)
        result = normalized_rotation(nudged)
        self.assertIsNotNone(result, 'must be inside the tolerance, or this tests nothing')
        self.assertAlmostEqual(math.sqrt(sum(c * c for c in result)), 1.0, places=9)

    def test_a_real_tilt_survives_a_quaternion_inside_the_tolerance(self):
        # End to end: a genuine 1 degree roll must still read as 1 degree off level when the
        # stored rotation is 5e-4 off unit, not get clamped to zero.
        nudged = tuple(c * 1.0005 for c in quat_about_x(1.0))
        checked, reason = check_sample({'p': [0.0, 0.0, 0.0], 'r': list(nudged)}, 'unity')
        self.assertIsNone(reason)
        self.assertAlmostEqual(checked.up_angle, 1.0, places=3)

    def test_non_unit_quaternion_is_rejected(self):
        # `rotate` on a non-unit quaternion silently scales its result rather than rotating.
        self.assertIsNone(normalized_rotation((0.0, 0.0, 0.0, 2.0)))

    def test_zero_quaternion_is_rejected(self):
        self.assertIsNone(normalized_rotation((0.0, 0.0, 0.0, 0.0)))

    def test_wrong_arity_is_rejected(self):
        self.assertIsNone(normalized_rotation((0.0, 0.0, 1.0)))

    def test_missing_is_rejected(self):
        self.assertIsNone(normalized_rotation(None))

    def test_non_numeric_component_is_rejected(self):
        self.assertIsNone(normalized_rotation((0.0, 0.0, 0.0, 'w')))


class TestConventions(unittest.TestCase):
    """The AXES rows, held to rotations built the way the SDK builds them. Without these, giving
    gltf_authored rep103's axes, or unity the wrong forward, leaves the suite green."""

    def gltf_authored_sample(self, yaw_degrees: float) -> dict:
        # The wire values the SDK's gltf_authored rule produces for a level robot at this yaw:
        # gaze rotation basis(q) * Ry(-90deg) in closed form.
        q = ros_yaw(yaw_degrees)
        s = math.sqrt(2.0) / 2.0
        r = (s * (q[0] + q[1]), -s * (q[2] + q[3]), s * (q[1] - q[0]), s * (q[3] - q[2]))
        return {'p': [-1.0, 0.0, -2.0], 'r': list(r)}

    def test_a_level_robot_reads_level_and_faces_its_heading_under_every_convention(self):
        for yaw in (0.0, 37.0, 90.0, -150.0):
            ros_forward = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.0)
            cases = {
                'gltf_authored': (self.gltf_authored_sample(yaw),
                                  [-ros_forward[0], ros_forward[2], -ros_forward[1]]),
                'unity': ({'p': [0.0, 0.0, 0.0], 'r': unity_rotation(ros_yaw(yaw))},
                          unity_position(ros_forward)),
                'rep103': ({'p': [0.0, 0.0, 0.0], 'r': list(ros_yaw(yaw))}, list(ros_forward)),
            }
            for convention, (sample, want_forward) in cases.items():
                with self.subTest(convention=convention, yaw=yaw):
                    checked, reason = check_sample(sample, convention)
                    self.assertIsNone(reason)
                    self.assertLess(checked.up_angle, 1e-6)
                    for got, want in zip(checked.forward, want_forward):
                        self.assertAlmostEqual(got, want, places=6)

    def test_a_rep103_session_checked_as_unity_faces_ninety_degrees_off_its_path(self):
        # The defect, pinned: checking a rep103 session with the unity axes reads its forward
        # axis as world up, so every segment is 90 degrees off whatever the heading -- which is
        # why it reads like a real finding. Its level angle is the heading, not a tilt.
        for heading in (0.0, 30.0, 180.0):
            with self.subTest(heading=heading):
                checked = checked_rows(drive(heading_degrees=heading, convention='rep103'), 'unity')
                forward, reversing = heading_errors(checked, 'unity', 0.10)
                self.assertTrue(forward + reversing)
                for angle in forward + reversing:
                    self.assertAlmostEqual(angle, 90.0, places=3)
                for sample in checked:
                    self.assertAlmostEqual(sample.up_angle, heading, places=3)

    def test_a_unity_session_checked_as_rep103_reads_by_the_direction_it_drove(self):
        # The reverse is not symmetric. Its up axis is read as forward, so the level angle is the
        # heading rather than any tilt, and its segments read as a correct heading, as reversing,
        # or not at all, by the direction driven -- never as 90 degrees off.
        expected = {0.0: (None, None), 30.0: (None, 180.0), -60.0: (0.0, None),
                    90.0: (None, None)}
        for heading, (forward_reads, reversing_reads) in expected.items():
            with self.subTest(heading=heading):
                checked = checked_rows(drive(heading_degrees=heading, convention='unity'), 'rep103')
                for sample in checked:
                    self.assertAlmostEqual(sample.up_angle, abs(heading), places=3)
                forward, reversing = heading_errors(checked, 'rep103', 0.10)
                for got, reads in ((forward, forward_reads), (reversing, reversing_reads)):
                    if reads is None:
                        self.assertEqual(got, [])
                        continue
                    self.assertTrue(got)
                    for angle in got:
                        self.assertAlmostEqual(angle, reads, places=3)

    def test_there_is_no_default_convention(self):
        with self.assertRaises(TypeError):
            check_sample({'p': [0.0, 0.0, 0.0], 'r': list(IDENTITY)})

    def test_both_conventions_name_a_distinct_up_component(self):
        self.assertEqual(AXES['unity']['up_component'], 1)
        self.assertEqual(AXES['rep103']['up_component'], 2)


class TestVector3(unittest.TestCase):
    """A truthiness check alone lets a short `p` array crash the whole run. The platform accepts
    malformed payloads with HTTP 200, so getting one back is the expected case."""

    def test_a_three_vector_is_accepted(self):
        self.assertEqual(vector3([1, 2, 3]), (1.0, 2.0, 3.0))

    def test_a_short_vector_is_rejected_not_crashed(self):
        self.assertIsNone(vector3([0, 0]))

    def test_a_long_vector_is_rejected(self):
        self.assertIsNone(vector3([0, 0, 0, 0]))

    def test_a_non_numeric_component_is_rejected(self):
        self.assertIsNone(vector3([0, 'x', 0]))

    def test_a_non_finite_component_is_rejected(self):
        self.assertIsNone(vector3([0, float('nan'), 0]))
        self.assertIsNone(vector3([0, float('inf'), 0]))

    def test_a_scalar_is_rejected(self):
        self.assertIsNone(vector3(3.0))


class TestCheckSample(unittest.TestCase):
    def test_a_sample_without_a_gaze_point_is_checked(self):
        checked, reason = check_sample({'time': 5.0, 'p': [1.0, 2.0, 3.0], 'r': list(IDENTITY)},
                                       'unity')
        self.assertIsNone(reason)
        self.assertFalse(checked.carries_gaze_point)
        self.assertEqual(checked.position, (1.0, 2.0, 3.0))
        self.assertEqual(checked.time, 5.0)
        self.assertAlmostEqual(checked.up_angle, 0.0, places=6)

    def test_a_gaze_point_of_any_shape_is_carried(self):
        for g in ([0.0, 0.0, 5.0], [0.0, None, 1.0], [0.0, 0.0], 'far'):
            with self.subTest(g=g):
                checked, reason = check_sample({'p': [1.0, 2.0, 3.0], 'g': g, 'r': list(IDENTITY)},
                                               'unity')
                self.assertIsNone(reason)
                self.assertTrue(checked.carries_gaze_point)

    def test_a_null_gaze_point_is_no_gaze_point(self):
        checked, reason = check_sample({'p': [0.0, 0.0, 0.0], 'g': None, 'r': list(IDENTITY)},
                                       'unity')
        self.assertIsNone(reason)
        self.assertFalse(checked.carries_gaze_point)

    def test_missing_position_or_rotation_is_unusable(self):
        for missing in ('p', 'r'):
            sample = {'p': [0.0, 0.0, 0.0], 'r': list(IDENTITY)}
            del sample[missing]
            with self.subTest(missing=missing):
                checked, reason = check_sample(sample, 'unity')
                self.assertIsNone(checked)
                self.assertIn(missing, reason)

    def test_non_unit_rotation_is_unusable_not_a_pass(self):
        checked, reason = check_sample({'p': [0.0, 0.0, 0.0], 'r': [0.0, 0.0, 0.0, 2.0]}, 'unity')
        self.assertIsNone(checked)
        self.assertIsNotNone(reason)

    def test_a_short_position_is_unusable_not_a_crash(self):
        checked, reason = check_sample({'p': [0.0, 0.0], 'r': list(IDENTITY)}, 'unity')
        self.assertIsNone(checked)
        self.assertIn('3-vector', reason)

    def test_a_sample_with_no_usable_time_is_still_checked(self):
        for time in (None, 'soon', float('nan')):
            with self.subTest(time=time):
                checked, reason = check_sample({'time': time, 'p': [0, 0, 0], 'r': list(IDENTITY)},
                                               'unity')
                self.assertIsNone(reason)
                self.assertIsNone(checked.time)


class TestHeading(unittest.TestCase):
    def test_a_robot_facing_its_path_reads_zero_under_both_bases(self):
        for convention in ('unity', 'rep103'):
            with self.subTest(convention=convention):
                forward, reversing = heading_errors(
                    checked_rows(drive(heading_degrees=30.0, convention=convention), convention),
                    convention, 0.10)
                self.assertGreater(len(forward), 10)
                self.assertEqual(reversing, [])
                self.assertLess(max(forward), 1e-3)

    def test_a_rotation_turned_ninety_degrees_reads_ninety(self):
        forward, reversing = heading_errors(
            checked_rows(drive(heading_degrees=0.0, facing_degrees=90.0)), 'unity', 0.10)
        self.assertTrue(forward)
        for angle in forward + reversing:
            self.assertAlmostEqual(angle, 90.0, places=3)

    def test_a_rotation_turned_around_reads_as_reversing(self):
        forward, reversing = heading_errors(
            checked_rows(drive(heading_degrees=0.0, facing_degrees=180.0)), 'unity', 0.10)
        self.assertEqual(forward, [])
        self.assertGreater(len(reversing), 10)

    def test_a_pitched_frame_facing_its_path_reads_zero(self):
        # Heading is read on the horizontal, so a camera pitched down at the floor ahead still faces
        # its path.
        rows = drive(heading_degrees=0.0, convention='rep103')
        pitched = (0.0, math.sin(math.radians(10)), 0.0, math.cos(math.radians(10)))
        for row in rows:
            row['r'] = list(pitched)
        forward, _ = heading_errors(checked_rows(rows, 'rep103'), 'rep103', 0.10)
        self.assertTrue(forward)
        self.assertLess(max(forward), 1e-3)

    def test_segments_are_grown_past_the_bound_and_ordered_by_time(self):
        rows = drive(count=11, step=0.06)
        rows.reverse()  # stored order is not time order
        forward, reversing = heading_errors(checked_rows(rows), 'unity', 0.10)
        self.assertEqual(len(forward), 5, 'eleven samples 0.06 m apart are five segments of 0.12 m')
        self.assertEqual(reversing, [])

    def test_a_robot_standing_still_has_no_heading(self):
        forward, reversing = heading_errors(checked_rows(drive(step=0.0)), 'unity', 0.10)
        self.assertEqual((forward, reversing), ([], []))


class TestTheFetch(unittest.TestCase):
    """What jsonRequests answers is described by its shape, never echoed: an error body can quote
    the request it refused, and that request carried the organization key."""

    QUOTED = 'orgkey-QUOTED-BACK-NOT-A-KEY'

    def failure(self, status, body):
        environment = type('E', (), {'api_host': 'https://api', 'project_id': 1, 'org_key': 'k'})()
        with mock.patch.object(VGC.c3denv, 'auth_header', lambda key: ''), \
                mock.patch.object(VGC.c3dhttp, 'post_json', lambda *a, **k: (status, body)), \
                self.assertRaises(SystemExit) as raised:
            VGC.fetch_gaze(environment, 'session-under-test', 1)
        return str(raised.exception)

    def test_an_error_body_is_named_by_its_shape(self):
        body = f'refused: Authorization: {self.QUOTED}'
        message = self.failure(500, body)
        self.assertIn('HTTP 500 from jsonRequests', message)
        self.assertIn(f'str, length {len(body)}', message)
        self.assertNotIn(self.QUOTED, message)

    def test_a_json_error_body_is_named_by_its_shape(self):
        message = self.failure(403, {'error': self.QUOTED})
        self.assertIn('dict, length 1', message)
        self.assertNotIn(self.QUOTED, message)

    def test_a_200_that_is_not_a_json_object_is_named_by_its_shape(self):
        message = self.failure(200, f'<html>{self.QUOTED}</html>')
        self.assertIn('not a JSON object', message)
        self.assertNotIn(self.QUOTED, message)

    def test_no_response_says_what_the_transport_said(self):
        self.assertIn('Connection refused',
                      self.failure(0, 'URLError: [Errno 111] Connection refused'))


class TestTheVerdict(unittest.TestCase):
    def run_main(self, rows, *extra):
        environment = type('E', (), {'api_host': 'x', 'project_id': 1, 'org_key': 'k'})()
        with mock.patch.object(VGC.c3denv, 'require_environment', lambda: environment), \
                mock.patch.object(VGC, 'fetch_gaze', lambda *a, **k: rows), \
                contextlib.redirect_stdout(io.StringIO()):
            return VGC.main(['session-under-test', '--pose-convention', 'unity', *extra])

    def test_a_session_without_gaze_points_passes(self):
        self.assertEqual(self.run_main(drive()), 0)

    def test_a_gaze_point_on_one_sample_fails_the_run(self):
        for g in ([0.0, 0.0, 5.0], [0.0, 0.0]):
            with self.subTest(g=g):
                rows = drive()
                rows[7]['g'] = g
                self.assertEqual(self.run_main(rows), 1)

    def test_a_tilted_frame_fails_only_when_level_is_required(self):
        rows = drive()
        for row in rows:
            row['r'] = list(quat_about_x(20.0))
        self.assertEqual(self.run_main(rows), 0)
        self.assertEqual(self.run_main(rows, '--require-level'), 1)

    def test_a_sideways_rotation_fails_only_when_heading_is_required(self):
        rows = drive(heading_degrees=0.0, facing_degrees=90.0)
        self.assertEqual(self.run_main(rows), 0)
        self.assertEqual(self.run_main(rows, '--require-heading'), 1)
        self.assertEqual(self.run_main(drive(), '--require-heading'), 0)

    def test_a_turned_around_rotation_fails_when_heading_is_required(self):
        rows = drive(heading_degrees=0.0, facing_degrees=180.0, count=100)
        self.assertEqual(self.run_main(rows, '--require-heading'), 1)
        self.assertEqual(self.run_main(rows, '--require-heading', '--max-reversing-share', '1.0',
                                       '--minimum-forward-segments', '0'), 0)

    def test_a_heading_check_that_examined_nothing_is_not_a_pass(self):
        self.assertEqual(self.run_main(drive(step=0.0), '--require-heading'), 1)

    def test_malformed_samples_are_gated_and_not_merely_counted(self):
        rows = drive(count=400)
        for row in rows[:100]:
            row['r'] = [0.0, 0.0, 0.0, 0.0]
        self.assertEqual(self.run_main(rows), 1)
        # Relaxing ONLY the unusable bound must make the same run pass, so the failure above is
        # attributable to that gate.
        self.assertEqual(self.run_main(rows, '--max-unusable-fraction', '1.0'), 0)

    def test_no_usable_sample_is_a_failure_not_a_crash(self):
        with self.assertRaises(SystemExit) as raised:
            self.run_main([{'p': [0, 0], 'r': [0, 0, 0, 0]}, 'not a sample'])
        self.assertIn('no sample carried a usable p and r', str(raised.exception))


if __name__ == '__main__':
    unittest.main()
