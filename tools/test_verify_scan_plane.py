"""Unit tests for tools/verify_scan_plane.py.

The verifier runs against a real capture, so nothing in CI can exercise it end to end without a
ROS graph. What CAN be exercised here is the half that decides the verdict: the reader rule, the
axis maps, the rotation matrix and the rays it picks. A synthetic document stands in for the
capture, built two ways -- one carrying the scan-plane rotation and one carrying the rotation a
camera frame carries, which is exactly the swap that the synthetic tier's mutation check makes in
`SessionRunner::RecordLaserScan`.

The expected rotations here are built from the DEFINITION -- the columns of `M R` -- with a
quaternion extraction written out below, not shared with the SDK and not shared with the verifier.
A fixture built by the rule under test agrees with a rule that is wrong.

Run: python3 tools/test_verify_scan_plane.py
"""
import importlib.util
import math
import pathlib
import types
import unittest

MODULE_PATH = pathlib.Path(__file__).resolve().parent / 'verify_scan_plane.py'
_SPEC = importlib.util.spec_from_file_location('verify_scan_plane', MODULE_PATH)
verify = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(verify)

FRAME = 'sensor_mount'
LIDAR = 'scan'
CAMERA = 'mount'
MOUNT_XYZ = (0.35, -0.12, 0.47)
MOUNT_RPY = (20.0, 35.0, 55.0)
RAY_COUNT = 32


def options(convention, **overrides):
    values = dict(convention=convention, frame=FRAME, lidar=LIDAR, camera=CAMERA,
                  mount_xyz=MOUNT_XYZ, mount_rpy_deg=MOUNT_RPY, rays=8, scans=3,
                  tolerance_m=1e-3, min_rotation_gap_deg=5.0, min_wrong_ray_m=0.1)
    values.update(overrides)
    return types.SimpleNamespace(**values)


def quaternion_from_columns(c0, c1, c2):
    """(x, y, z, w) for the rotation whose columns these are. Shepperd's branches.

    Written here rather than imported: this is the definition the record is supposed to satisfy,
    and a test that borrowed the implementation's extraction would pass for an implementation
    that extracts wrongly.
    """
    m = ((c0[0], c1[0], c2[0]), (c0[1], c1[1], c2[1]), (c0[2], c1[2], c2[2]))
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)
    norm = math.sqrt(sum(value * value for value in q))
    return tuple(value / norm for value in q)


def scan_plane_rotation(convention):
    """The rotation the record is supposed to carry, straight from the definition.

    `Q e_x = M R e_x`, `Q e_y = M R e_y`, `Q e_z = (M R e_x) x (M R e_y)`: the third column
    absorbs the mirror, which is the axis every ray carries as zero.
    """
    matrix = verify.rotation_matrix_from_rpy_degrees(*MOUNT_RPY)
    mapped = [verify.POSITION_MAPS[convention](*(matrix[row][column] for row in range(3)))
              for column in range(2)]
    cross = (mapped[0][1] * mapped[1][2] - mapped[0][2] * mapped[1][1],
             mapped[0][2] * mapped[1][0] - mapped[0][0] * mapped[1][2],
             mapped[0][0] * mapped[1][1] - mapped[0][1] * mapped[1][0])
    return quaternion_from_columns(mapped[0], mapped[1], cross)


def ros_rotation_quaternion():
    """The mount's own rotation as a quaternion, from roll/pitch/yaw, for the camera rule."""
    roll, pitch, yaw = (math.radians(angle) for angle in MOUNT_RPY)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def camera_rotation(convention):
    """What `ScenePose` sends: the dynamic rule, which is the rotation conjugated by the map."""
    x, y, z, w = ros_rotation_quaternion()
    if convention == 'unity':
        return (-y, z, x, -w)
    return (x, -z, y, w)               # gltf_authored


def ranges():
    """One sweep with the non-finite rays a driver really publishes."""
    values = []
    for index in range(RAY_COUNT):
        if index % 8 == 0:
            values.append(math.inf)
        elif index % 8 == 4:
            values.append(math.nan)
        else:
            values.append(2.0 + 0.25 * (index % 5))
    return values


def document(convention, scan_rotation=None, scans=3, frames=2):
    """A decoded capture in capnp_dump's shape: one scan block and one camera chunk.

    Each scan entry and each frame carries the sensor, frame and geometry of the block or chunk
    around it, as capnp_dump copies them, and the verifier reads nothing else.
    """
    position = list(verify.POSITION_MAPS[convention](*MOUNT_XYZ))
    scan_rotation = list(scan_rotation or scan_plane_rotation(convention))
    increment = 2.0 * math.pi / RAY_COUNT
    scan_block = {
        'stream': 'lidar', 'sensorId': LIDAR, 'seq': 0, 'frameId': FRAME, 'kind': 'scans',
        'angleMin': -math.pi, 'angleMax': math.pi, 'angleIncrement': increment,
        'rangeMin': 0.1, 'rangeMax': 12.0, 'rays': RAY_COUNT, 'rangeUnit': 'millimetre',
        'encoding': 'deltaShuffleZstd', 'finiteRangeCount': scans * (RAY_COUNT - 8),
        'entries': [{
            'time': 1757000000000 + index, 'seq': index, 'pose': position + scan_rotation,
            'timeIncrement': 0.0001, 'scanTime': 0.25,
            'sensorId': LIDAR, 'frameId': FRAME, 'angleMin': -math.pi,
            'angleIncrement': increment, 'kind': 'scans',
            'ranges': ranges(), 'intensities': [],
        } for index in range(scans)],
    }
    chunk = {
        'stream': 'camera', 'cameraId': CAMERA, 'seq': 0, 'frameId': FRAME,
        'width': 160, 'height': 120, 'hfovDegrees': 0.0, 'vfovDegrees': 0.0,
        'frames': [{
            'time': 1757000000000 + index, 'seq': index,
            'pose': position + list(camera_rotation(convention)),
            'byteOffset': 512 * index, 'byteLength': 512, 'keyframe': index == 0,
            'nalTypes': [9, 7, 8, 5] if index == 0 else [9, 1], 'startsWithAud': True,
            'cameraId': CAMERA, 'frameId': FRAME, 'width': 160, 'height': 120,
            'hfovDegrees': 0.0, 'vfovDegrees': 0.0,
        } for index in range(frames)],
    }
    return {'files': [scan_block, chunk]}


def cloud_object(convention):
    """A point cloud object from the same sensor name on the same frame, posed like a camera."""
    position = list(verify.POSITION_MAPS[convention](*MOUNT_XYZ))
    return {
        'stream': 'lidar', 'sensorId': LIDAR, 'seq': 0, 'frameId': FRAME, 'kind': 'clouds',
        'position': 'int16_mm', 'color': 'none', 'intensity': 'none', 'voxelLeafMm': 50,
        'encoding': 'planarDeltaShuffleZstd', 'zstdLevel': 3,
        'entries': [{
            'time': 1757000000000, 'seq': 0, 'pose': position + list(camera_rotation(convention)),
            'pointCount': 0, 'byteOffset': 0, 'byteLength': 0,
            'decoded': {'points': 0, 'first': [], 'consistent': True},
        }],
    }


def failures(results):
    return [message for ok, message in results if not ok]


class RotationMatrix(unittest.TestCase):
    def test_a_quarter_turn_about_z_carries_x_onto_y(self):
        matrix = verify.rotation_matrix_from_rpy_degrees(0.0, 0.0, 90.0)
        turned = verify.apply_matrix(matrix, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(turned[0], 0.0, places=12)
        self.assertAlmostEqual(turned[1], 1.0, places=12)
        self.assertAlmostEqual(turned[2], 0.0, places=12)

    def test_a_quarter_turn_about_y_carries_x_onto_minus_z(self):
        matrix = verify.rotation_matrix_from_rpy_degrees(0.0, 90.0, 0.0)
        turned = verify.apply_matrix(matrix, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(turned[2], -1.0, places=12)

    def test_a_quarter_turn_about_x_carries_y_onto_z(self):
        matrix = verify.rotation_matrix_from_rpy_degrees(90.0, 0.0, 0.0)
        turned = verify.apply_matrix(matrix, (0.0, 1.0, 0.0))
        self.assertAlmostEqual(turned[2], 1.0, places=12)

    def test_the_composition_is_rz_ry_rx_and_not_the_other_order(self):
        # Rz(90) Ry(90) sends x to -z then... the two orders disagree on this vector, which is
        # the point: rpy composed the other way round is a different rotation.
        matrix = verify.rotation_matrix_from_rpy_degrees(0.0, 90.0, 90.0)
        turned = verify.apply_matrix(matrix, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(turned[0], 0.0, places=12)
        self.assertAlmostEqual(turned[1], 0.0, places=12)
        self.assertAlmostEqual(turned[2], -1.0, places=12)

    def test_the_matrix_is_orthonormal(self):
        matrix = verify.rotation_matrix_from_rpy_degrees(*MOUNT_RPY)
        for column in range(3):
            axis = [matrix[row][column] for row in range(3)]
            self.assertAlmostEqual(math.sqrt(sum(v * v for v in axis)), 1.0, places=12)


class Rotate(unittest.TestCase):
    def test_the_identity_moves_nothing(self):
        self.assertEqual(verify.rotate((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0)), (1.0, 2.0, 3.0))

    def test_a_quarter_turn_about_z(self):
        half = math.sqrt(0.5)
        turned = verify.rotate((0.0, 0.0, half, half), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(turned[0], 0.0, places=12)
        self.assertAlmostEqual(turned[1], 1.0, places=12)

    def test_a_negated_quaternion_is_the_same_rotation(self):
        quaternion = verify.rotate((0.3, -0.2, 0.1, 0.927), (0.5, -1.5, 2.0))
        negated = verify.rotate((-0.3, 0.2, -0.1, -0.927), (0.5, -1.5, 2.0))
        for got, want in zip(negated, quaternion):
            self.assertAlmostEqual(got, want, places=12)


class RotationGap(unittest.TestCase):
    def test_a_rotation_against_itself_is_zero(self):
        self.assertAlmostEqual(verify.rotation_gap_degrees((0.3, -0.2, 0.1, 0.927),
                                                           (0.3, -0.2, 0.1, 0.927)), 0.0, places=6)

    def test_a_quaternion_that_is_not_quite_unit_is_still_the_same_rotation(self):
        # Four float32s land about 1e-7 off unity, and the arc cosine turns that into a
        # fiftieth of a degree -- enough for a record compared with ITSELF to read as a
        # different rotation. This quaternion is one a capture really carries.
        quaternion = (0.3994181156158447, -0.796946108341217,
                      0.44764038920402527, 0.07042819261550903)
        self.assertNotAlmostEqual(sum(v * v for v in quaternion), 1.0, places=9)
        self.assertLess(verify.rotation_gap_degrees(quaternion, quaternion), 1e-3)

    def test_a_negated_quaternion_is_no_gap_at_all(self):
        quaternion = (0.3, -0.2, 0.1, 0.927)
        negated = tuple(-value for value in quaternion)
        self.assertAlmostEqual(verify.rotation_gap_degrees(quaternion, negated), 0.0, places=6)

    def test_a_quarter_turn_reads_as_ninety_degrees(self):
        half = math.sqrt(0.5)
        self.assertAlmostEqual(
            verify.rotation_gap_degrees((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, half, half)),
            90.0, places=6)


class PositionMaps(unittest.TestCase):
    def test_the_two_reflections_and_the_identity(self):
        self.assertEqual(verify.POSITION_MAPS['rep103'](1.0, 2.0, 3.0), (1.0, 2.0, 3.0))
        self.assertEqual(verify.POSITION_MAPS['unity'](1.0, 2.0, 3.0), (-2.0, 3.0, 1.0))
        self.assertEqual(verify.POSITION_MAPS['gltf_authored'](1.0, 2.0, 3.0), (-1.0, 3.0, -2.0))

    def test_both_scene_maps_are_reflections(self):
        # Determinant -1 is the whole reason the scan plane needs its own rotation. If one of
        # these ever became a proper rotation, the rule under test would be unnecessary.
        for convention in verify.REFLECTING_CONVENTIONS:
            axes = [verify.POSITION_MAPS[convention](*axis)
                    for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))]
            determinant = (axes[0][0] * (axes[1][1] * axes[2][2] - axes[1][2] * axes[2][1])
                           - axes[1][0] * (axes[0][1] * axes[2][2] - axes[0][2] * axes[2][1])
                           + axes[2][0] * (axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1]))
            self.assertAlmostEqual(determinant, -1.0, places=12, msg=convention)


class RaySelection(unittest.TestCase):
    def test_non_finite_rays_are_skipped(self):
        scan = document('unity')['files'][0]['entries'][0]
        for _index, ray_range, _angle in verify.finite_rays(scan, 8):
            self.assertTrue(math.isfinite(ray_range))

    def test_the_angle_is_the_index_walked_from_angle_min(self):
        scan = document('unity')['files'][0]['entries'][0]
        index, _range, angle = verify.finite_rays(scan, 1)[0]
        self.assertAlmostEqual(angle, scan['angleMin'] + index * scan['angleIncrement'], places=12)

    def test_a_scan_with_no_ranges_yields_nothing(self):
        # --scan-ranges off is the default, so a document decoded without it must not quietly
        # reconstruct zero rays and pass.
        self.assertEqual(verify.finite_rays({'angleMin': 0.0, 'angleIncrement': 0.1}, 8), [])


class TheRuleHolds(unittest.TestCase):
    def test_the_scan_plane_rotation_passes_under_both_reflecting_conventions(self):
        for convention in verify.REFLECTING_CONVENTIONS:
            results = verify.check_document(document(convention), options(convention), convention)
            self.assertEqual(failures(results), [], msg=convention)
            self.assertGreaterEqual(len(results), 6, msg=convention)

    def test_a_record_carrying_the_camera_rotation_fails(self):
        # The mutation, at this level: RecordLaserScan calling ScenePose instead of
        # ScanPlanePose puts exactly this quaternion on the record.
        for convention in verify.REFLECTING_CONVENTIONS:
            results = verify.check_document(
                document(convention, scan_rotation=camera_rotation(convention)),
                options(convention), convention)
            messages = failures(results)
            self.assertTrue(any('lands where the published tf puts it' in m for m in messages),
                            msg=f'{convention}: {messages}')
            self.assertTrue(any('is not the camera rotation' in m for m in messages),
                            msg=f'{convention}: {messages}')

    def test_a_rotation_wrong_by_a_degree_fails_the_ray_check(self):
        # "Not the camera's" is not the assertion; "where the tf put it" is. A rotation that is
        # neither rule fails too, which is what separates this from an inequality test.
        for convention in verify.REFLECTING_CONVENTIONS:
            x, y, z, w = scan_plane_rotation(convention)
            half = math.radians(1.0) / 2.0
            nudge = (math.sin(half), 0.0, 0.0, math.cos(half))
            tilted = (w * nudge[0] + x * nudge[3] + y * nudge[2] - z * nudge[1],
                      w * nudge[1] - x * nudge[2] + y * nudge[3] + z * nudge[0],
                      w * nudge[2] + x * nudge[1] - y * nudge[0] + z * nudge[3],
                      w * nudge[3] - x * nudge[0] - y * nudge[1] - z * nudge[2])
            results = verify.check_document(
                document(convention, scan_rotation=tilted), options(convention), convention)
            self.assertTrue(
                any('lands where the published tf puts it' in m for m in failures(results)),
                msg=convention)

    def test_a_shifted_position_fails_even_with_the_right_rotation(self):
        for convention in verify.REFLECTING_CONVENTIONS:
            doc = document(convention)
            for scan in doc['files'][0]['entries']:
                scan['pose'][0] += 0.05
            results = verify.check_document(doc, options(convention), convention)
            self.assertTrue(any('mapped mount position' in m for m in failures(results)),
                            msg=convention)

    def test_the_convention_on_the_wire_must_be_the_one_the_pass_claims(self):
        results = verify.check_document(document('unity'), options('unity'), 'rep103')
        self.assertTrue(any('pose_convention' in m for m in failures(results)))

    def test_records_on_another_frame_or_sensor_are_not_counted(self):
        doc = document('unity')
        for scan in doc['files'][0]['entries']:
            scan['frameId'] = 'base_link'
        results = verify.check_document(doc, options('unity'), 'unity')
        self.assertTrue(any('laserscan records' in m for m in failures(results)))

    def test_a_cloud_object_from_the_same_name_and_frame_is_not_a_scan(self):
        # A lidar name can be either kind, and a cloud entry has a pose but no rays and none of a
        # scan entry's copied fields. Only a scan block's entries are scans.
        doc = document('unity')
        doc['files'].insert(0, cloud_object('unity'))
        results = verify.check_document(doc, options('unity'), 'unity')
        self.assertEqual(failures(results), [])
        self.assertIn("laserscan records for 'scan' on sensor_mount: 3",
                      [message for _ok, message in results])

    def test_a_capture_with_no_camera_frames_is_a_failure_not_a_pass(self):
        doc = document('unity', frames=0)
        results = verify.check_document(doc, options('unity'), 'unity')
        self.assertTrue(any('camera frames' in m for m in failures(results)))

    def test_too_few_reconstructable_rays_is_a_failure(self):
        doc = document('unity')
        for scan in doc['files'][0]['entries']:
            scan['ranges'] = [math.inf] * RAY_COUNT
        results = verify.check_document(doc, options('unity'), 'unity')
        self.assertTrue(any('rays reconstructed' in m for m in failures(results)))


class CommandLine(unittest.TestCase):
    def test_rep103_is_refused(self):
        # The pass that cannot tell the two rules apart must not be reachable by typing it.
        with self.assertRaises(SystemExit) as caught:
            verify.parse_args(['--capture', '/tmp', '--convention', 'rep103', '--frame', FRAME,
                               '--lidar', LIDAR, '--camera', CAMERA,
                               '--mount-xyz', '0,0,0', '--mount-rpy-deg', '0,0,0'])
        self.assertEqual(caught.exception.code, 2)

    def test_a_triple_needs_three_numbers(self):
        self.assertEqual(verify.triple('1,-2.5,3'), (1.0, -2.5, 3.0))
        with self.assertRaises(Exception):
            verify.triple('1,2')


if __name__ == '__main__':
    unittest.main(verbosity=2)
