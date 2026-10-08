"""Tests for tools/verify_dynamics_level.py.

Two easy ways for the tool to be wrong are passing a robot rendering backwards, and certifying a
run in which it measured nothing. The next edit to
`is_registration_record` or `heading_errors` needs something to catch either.

    Run: uv run tools/test_verify_dynamics_level.py

The fixtures build the WIRE form from ROS inputs locally rather than calling the tool's own
transforms, so a change to those transforms moves the code under test and not the expectation.
"""
from __future__ import annotations

import importlib.util
import math
import pathlib
import unittest
from unittest import mock

MODULE_PATH = pathlib.Path(__file__).parent / 'verify_dynamics_level.py'
_SPEC = importlib.util.spec_from_file_location('verify_dynamics_level', MODULE_PATH)
VDL = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(VDL)


def wire_position(ros):
    """`ros_to_c3d_point`: the glTF point with X mirrored."""
    x, y, z = ros
    return [-x, z, -y]


def wire_rotation(q):
    """The dynamics wire rule, `(qx, -qz, qy, qw)`."""
    return [q[0], -q[2], q[1], q[3]]


def yaw(degrees):
    half = math.radians(degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def drive(positions, orientation=yaw(0), t0=1000.0, interval=0.1):
    return [{'time': t0 + interval * i, 'p': wire_position(p), 'r': wire_rotation(orientation)}
            for i, p in enumerate(positions)]


def straight(count=200, step=0.02, start=0.0):
    return [(start + step * i, 0.0, 0.0) for i in range(count)]


REGISTRATION = {'time': 999.0, 'p': [0.0, 0.0, 0.0], 'r': [0.0, 0.0, 0.0, 1.0],
                's': [1.0, 1.0, 1.0], 'properties': [{'name': 'robot'}]}


class Heading(unittest.TestCase):
    def test_a_correct_forward_drive_reads_zero_and_reverses_nothing(self):
        forward, reversing = VDL.heading_errors(drive(straight()), 0.10)
        self.assertGreater(len(forward), 20)
        self.assertEqual(reversing, [])
        self.assertLess(max(forward), 1e-6)

    def test_a_mesh_turned_180_degrees_reads_as_reversing_not_as_forward(self):
        """It must land in the reversing bucket, not be averaged into the forward distribution
        and not be folded to look perfect."""
        forward, reversing = VDL.heading_errors(drive(straight(), orientation=yaw(180)), 0.10)
        self.assertEqual(forward, [])
        self.assertGreater(len(reversing), 20)
        self.assertGreater(min(reversing), 179.0)

    def test_real_reversing_is_kept_out_of_the_forward_distribution(self):
        """A TurtleBot 4 undocks by reversing, so ~180 degree segments are legitimate. One
        distribution holding both pushes the p95 over the threshold and fails a correct session."""
        positions = [(-0.02 * i, 0.0, 0.0) for i in range(20)]
        positions += [(positions[-1][0] + 0.02 * i, 0.0, 0.0) for i in range(1, 181)]
        forward, reversing = VDL.heading_errors(drive(positions), 0.10)
        self.assertGreater(len(reversing), 0)
        self.assertGreater(len(forward), 20)
        self.assertLess(max(forward), 1e-6, 'a reversing segment leaked into the forward set')

    def test_a_segment_shorter_than_the_bound_is_not_a_direction(self):
        """Normalising a near-zero displacement turns float noise into an arbitrary angle, so a
        stationary robot must yield no segment rather than a spurious one."""
        forward, reversing = VDL.heading_errors(drive([(1.0, 0.0, 0.0)] * 200), 0.10)
        self.assertEqual((forward, reversing), ([], []))


class RegistrationRecords(unittest.TestCase):
    def test_the_object_registration_record_is_not_a_pose(self):
        """`Session::RegisterDynamic` announces the object once with a hard-coded origin. Treated
        as a pose it draws a segment from the map origin to wherever the robot spawned, so the
        error IS the session's distance from the origin -- metres on any run not starting there,
        and indistinguishable from a heading defect."""
        rows = drive(straight(start=8.0))
        self.assertTrue(VDL.is_registration_record(REGISTRATION))
        self.assertFalse(VDL.is_registration_record(rows[0]))
        self.assertEqual(VDL.heading_errors([dict(REGISTRATION)] + rows, 0.10),
                         VDL.heading_errors(rows, 0.10),
                         'the registration record entered the heading check')

    def test_a_pose_that_carries_its_own_scale_is_still_a_pose(self):
        """`RecordDynamic` takes scale as an optional argument, so a consumer may legally record
        it on every snapshot. Keying the filter on `s` as well looks equivalent and discards
        EVERY such pose, so a valid session would report it had measured nothing."""
        rows = drive(straight())
        for row in rows:
            row['s'] = [1.0, 1.0, 1.0]
        self.assertFalse(any(VDL.is_registration_record(row) for row in rows))
        forward, reversing = VDL.heading_errors(rows, 0.10)
        self.assertGreater(len(forward), 20)
        self.assertEqual(reversing, [])


class ObjectsAreSeparatePaths(unittest.TestCase):
    def test_two_stationary_objects_are_not_one_moving_object(self):
        """The API returns every dynamic object of a session interleaved in one stream. Grown as a
        flat trajectory, the jump between two objects reads as motion: a 50% reversing share and a
        FAIL on a session where nothing moved and nothing rendered wrongly."""
        rows = []
        for i in range(40):
            for object_id, x in (('robot', 0.0), ('pallet', 1.0)):
                rows.append({'id': object_id, 'time': 1000.0 + 0.1 * i,
                             'p': wire_position((x, 0.0, 0.0)), 'r': wire_rotation(yaw(0))})
        self.assertEqual(VDL.heading_errors(rows, 0.10), ([], []),
                         'the gap between two objects was read as travel')

    def test_each_object_is_checked_on_its_own_path(self):
        """The other half: grouping must separate the objects without losing them."""
        rows = []
        for i in range(200):
            rows.append({'id': 'east', 'time': 1000.0 + 0.1 * i,
                         'p': wire_position((0.02 * i, 0.0, 0.0)), 'r': wire_rotation(yaw(0))})
            rows.append({'id': 'west', 'time': 1000.0 + 0.1 * i,
                         'p': wire_position((-0.02 * i, 5.0, 0.0)), 'r': wire_rotation(yaw(180))})
        forward, reversing = VDL.heading_errors(rows, 0.10)
        self.assertEqual(reversing, [], 'a correctly oriented object read as reversing')
        self.assertGreater(len(forward), 40)
        self.assertLess(max(forward), 1e-6)


class TheVerdict(unittest.TestCase):
    def run_main(self, rows, *extra):
        environment = type('E', (), {'api_host': 'x', 'project_id': 1, 'org_key': 'k'})()
        with mock.patch.object(VDL.c3denv, 'require_environment', lambda: environment), \
                mock.patch.object(VDL, 'fetch', lambda *a, **k: rows):
            return VDL.main(['session-under-test', *extra])

    def test_a_session_with_no_heading_to_check_fails_rather_than_passing(self):
        """A robot that moves 0.09 m with its level mesh turned 180 degrees away clears the tilt
        check (yaw-blind by construction) and produces no segment. Printing `HEADING skipped`
        then `PASS` would certify the exact defect the tool exists to catch."""
        barely_moved = [(0.0005 * i, 0.0, 0.0) for i in range(200)]   # 0.0995 m, bound 0.10
        self.assertEqual(self.run_main(drive(barely_moved, orientation=yaw(180))), 1)

    def test_a_correct_session_passes(self):
        self.assertEqual(self.run_main(drive(straight())), 0)

    def test_a_backwards_session_fails(self):
        self.assertEqual(self.run_main(drive(straight(), orientation=yaw(180))), 1)

    def test_a_tipped_session_fails_on_the_level_check(self):
        """Roll about the chassis's own forward axis: the up axis leaves world up. The threshold
        is on the WORST sample, not a percentile."""
        rolled = (math.sin(math.radians(20)), 0.0, 0.0, math.cos(math.radians(20)))
        self.assertEqual(self.run_main(drive(straight(), orientation=rolled)), 1)

    def test_malformed_rotations_are_gated_and_not_merely_counted(self):
        """4999 malformed samples beside one good one must not report PASS on every sample."""
        rows = drive(straight(count=400))
        for row in rows[:100]:
            row['r'] = [0.0, 0.0, 0.0, 0.0]
        self.assertEqual(self.run_main(rows), 1)
        # Relaxing ONLY the unusable bound must make the same run pass, so the failure above is
        # attributable to that gate and not to a side effect of the malformed rows.
        self.assertEqual(self.run_main(rows, '--max-unusable-fraction', '1.0'), 0)

    def test_all_registration_records_and_nothing_else_is_refused(self):
        """Not a pass, and not a crash: a run whose only rows are announcements measured nothing."""
        with self.assertRaises(SystemExit) as raised:
            self.run_main([dict(REGISTRATION), dict(REGISTRATION)])
        self.assertIn('all of them object registrations', str(raised.exception))


class NothingStored(unittest.TestCase):
    """Through the real `fetch`: what `jsonRequests` answers decides which failure is printed."""

    def failure_message(self, status, body):
        environment = type('E', (), {'api_host': 'https://api', 'project_id': 1, 'org_key': 'k'})()
        with mock.patch.object(VDL.c3denv, 'require_environment', lambda: environment), \
                mock.patch.object(VDL.c3denv, 'auth_header', lambda key: ''), \
                mock.patch.object(VDL.c3dhttp, 'post_json', lambda *a, **k: (status, body)), \
                self.assertRaises(SystemExit) as raised:
            VDL.main(['session-under-test'])
        # A string code is printed to stderr and exits 1.
        self.assertIsInstance(raised.exception.code, str)
        return raised.exception.code

    def test_a_404_reaches_the_documented_failure(self):
        """A session recorded with robot_dynamic_object off stores no DYNAMICS, and the platform
        answers the request for them with a 404 rather than an empty 200."""
        message = self.failure_message(404, 'No data found for the requested json file')
        self.assertTrue(message.startswith('FAIL no dynamics samples stored'), message)

    def test_any_other_error_status_still_fails_naming_it(self):
        message = self.failure_message(500, 'Internal Server Error')
        self.assertIn('HTTP 500 from jsonRequests', message)
        self.assertNotIn('no dynamics samples stored', message)

    def test_a_body_is_named_by_its_shape_and_never_echoed(self):
        """An error body can quote the request it refused, and that request carried the
        organization key."""
        quoted = 'orgkey-QUOTED-BACK-NOT-A-KEY'
        text = f'refused: Authorization: {quoted}'
        for status, body, said in ((500, text, f'str, length {len(text)}'),
                                   (401, {'error': quoted}, 'dict, length 1'),
                                   (200, f'<html>{quoted}</html>', 'not a JSON object')):
            with self.subTest(status=status):
                message = self.failure_message(status, body)
                self.assertIn(said, message)
                self.assertNotIn(quoted, message)

    def test_no_response_says_what_the_transport_said(self):
        message = self.failure_message(0, 'URLError: [Errno 111] Connection refused')
        self.assertIn('Connection refused', message)


class TheProject(unittest.TestCase):
    """`--project` reads another project's session with the same credentials, as the gaze
    verifier's does; without it the project is the environment's."""

    def fetched_project(self, *extra):
        environment = type('E', (), {'api_host': 'x', 'project_id': 1, 'org_key': 'k'})()
        asked = []

        def fetch(environment, project_id, session_id, kind):
            asked.append(project_id)
            return drive(straight())

        with mock.patch.object(VDL.c3denv, 'require_environment', lambda: environment), \
                mock.patch.object(VDL, 'fetch', fetch):
            self.assertEqual(VDL.main(['session-under-test', *extra]), 0)
        return asked

    def test_the_flag_names_the_project_read(self):
        self.assertEqual(self.fetched_project('--project', '42'), [42])

    def test_without_the_flag_it_is_the_environments(self):
        self.assertEqual(self.fetched_project(), [1])

    def test_the_request_names_the_project_it_was_given(self):
        environment = type('E', (), {'api_host': 'https://api', 'project_id': 1, 'org_key': 'k'})()
        urls = []

        def post_json(url, headers, body):
            urls.append(url)
            return 200, {'data': {}}

        with mock.patch.object(VDL.c3dhttp, 'post_json', post_json), \
                mock.patch.object(VDL.c3denv, 'auth_header', lambda key: ''):
            VDL.fetch(environment, 42, 'session-under-test', 'DYNAMICS')
        self.assertEqual(urls, ['https://api/v0/projects/42/sessions/session-under-test/jsonRequests'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
