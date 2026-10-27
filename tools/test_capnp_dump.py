"""Unit tests for tools/capnp_dump.py.

Most of what that tool gets wrong, it gets wrong without pycapnp's help: the planar point block
is hand-written against docs/data-format.md, and the colour decoders are bit arithmetic copied
from it. So everything below is unconditional except the schema round trip, which is the only
part that needs the one dependency the tool carries -- CI runs this file with no dependency step,
and a suite that skipped wholesale there would be watching nothing.

The blocks are written out as byte literals rather than packed by `struct`. A fixture built by
the same layout rule the decoder applies agrees with a decoder that has the layout wrong.

Run: python3 tools/test_capnp_dump.py
"""
import contextlib
import importlib.util
import io
import json
import math
import pathlib
import sys
import tempfile
import unittest
import unittest.mock

MODULE_PATH = pathlib.Path(__file__).resolve().parent / 'capnp_dump.py'
_SPEC = importlib.util.spec_from_file_location('capnp_dump', MODULE_PATH)
dump = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(dump)

try:
    import capnp                                                        # noqa: F401
    HAVE_CAPNP = True
except ImportError:
    HAVE_CAPNP = False

try:
    import zstandard                                                    # noqa: F401
    HAVE_ZSTANDARD = True
except ImportError:
    HAVE_ZSTANDARD = False

NEEDS_CAPNP = unittest.skipUnless(HAVE_CAPNP, 'pycapnp is not importable')
NEEDS_ZSTANDARD = unittest.skipUnless(HAVE_ZSTANDARD, 'zstandard is not importable')


def encode_rgb565(red, green, blue):
    """docs/data-format.md's encoder, so a round trip is not the decoder talking to itself."""
    return ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)


def encode_rgb332(red, green, blue):
    return (red & 0xE0) | ((green & 0xE0) >> 3) | (blue >> 6)


# 3 points: int16_mm positions, rgb565 colour, uint8 intensity. Planes in wire order.
BLOCK_INT16 = bytes.fromhex(
    'e803 3cf6 ff7f'      # X: 1000, -2500, 32767 mm
    '0000 f401 ffff'      # Y: 0, 500, -1 mm
    'fa00 0000 0080'      # Z: 250, 0, -32768 mm
    '0000 ffff 1084'      # C: 0x0000, 0xFFFF, 0x8410
    '00 80 ff')           # I: 0, 128, 255

# 2 points: float32 positions, no colour, no intensity.
BLOCK_FLOAT32 = bytes.fromhex(
    '0000c03f 000000c0'   # X: 1.5, -2.0
    '0000003f 0000803e'   # Y: 0.5, 0.25
    '00000000 00004040')  # Z: 0.0, 3.0

# 2 points: int16_mm positions, rgb888 colour (three bytes per point), no intensity.
BLOCK_RGB888 = bytes.fromhex(
    '6400 c800'           # X: 100, 200 mm
    '9cff 0000'           # Y: -100, 0 mm
    '0000 3200'           # Z: 0, 50 mm
    '010203 fafbfc')      # C: (1,2,3), (250,251,252)

# 2 points: int16_mm positions at the origin, rgb332 colour (ONE byte per point), uint8
# intensity. A colour plane mis-sized as two bytes per point moves the intensity plane.
BLOCK_RGB332 = bytes.fromhex(
    '0000 0000'
    '0000 0000'
    '0000 0000'
    '92 ff'               # C: 0x92, 0xFF
    '07 08')              # I: 7, 8


class StreamFromFilename(unittest.TestCase):
    def test_the_three_capture_suffixes(self):
        self.assertEqual(dump.stream_for_filename('000001-camera.bin'), 'camera')
        self.assertEqual(dump.stream_for_filename('000042-lidar.bin'), 'lidar')
        self.assertEqual(dump.stream_for_filename('000043-pose.bin'), 'pose')
        self.assertEqual(dump.resolve_stream('000001-camera.bin'), 'camera')
        self.assertEqual(dump.resolve_stream('000042-lidar.bin'), 'lidar')

    def test_a_name_that_only_mentions_a_stream_declares_nothing(self):
        # The suffix is the rule, not a substring search: a guess here decodes a body with the
        # wrong schema and prints plausible nonsense.
        for name in ('camera.bin', 'run-camera-3.bin', '000001-camera.bin.gz',
                     'lidar', '000001-camera.json'):
            with self.subTest(name):
                self.assertIsNone(dump.stream_for_filename(name))

    def test_an_undecidable_name_is_an_error_naming_the_file(self):
        with self.assertRaises(ValueError) as caught:
            dump.resolve_stream('capture.dat')
        self.assertIn('capture.dat', str(caught.exception))

    def test_an_explicit_stream_wins_over_the_name(self):
        self.assertEqual(dump.resolve_stream('capture.dat', 'lidar'), 'lidar')
        self.assertEqual(dump.resolve_stream('000001-camera.bin', 'lidar'), 'lidar')


class Rgb565(unittest.TestCase):
    def test_black_and_white_are_exact(self):
        self.assertEqual(dump.decode_rgb565(0x0000), (0, 0, 0))
        self.assertEqual(dump.decode_rgb565(0xFFFF), (255, 255, 255))

    def test_a_hand_computed_mid_grey(self):
        # r5=16, g6=32, b5=16 -> 0x8410. Each channel's replication term is non-zero, so a
        # decoder that dropped the replication would answer (128, 128, 128) here.
        self.assertEqual(dump.decode_rgb565(0x8410), (132, 130, 132))

    def test_every_code_survives_a_decode_and_a_re_encode(self):
        for value in range(1 << 16):
            red, green, blue = dump.decode_rgb565(value)
            self.assertEqual(encode_rgb565(red, green, blue), value, hex(value))

    def test_a_round_trip_stays_within_one_quantisation_step(self):
        steps = (8, 4, 8)
        for colour in ((0, 0, 0), (255, 255, 255), (128, 128, 128),
                       (200, 60, 60), (17, 200, 240), (7, 3, 1)):
            with self.subTest(colour):
                decoded = dump.decode_rgb565(encode_rgb565(*colour))
                for channel, (before, after, step) in enumerate(zip(colour, decoded, steps)):
                    self.assertLess(abs(before - after), step, f'channel {channel}')

    def test_channels_are_not_swapped(self):
        # Pure red, green and blue at full strength, which a permuted decode cannot satisfy.
        self.assertEqual(dump.decode_rgb565(encode_rgb565(255, 0, 0)), (255, 0, 0))
        self.assertEqual(dump.decode_rgb565(encode_rgb565(0, 255, 0)), (0, 255, 0))
        self.assertEqual(dump.decode_rgb565(encode_rgb565(0, 0, 255)), (0, 0, 255))


class Rgb332(unittest.TestCase):
    def test_black_and_white_are_exact(self):
        self.assertEqual(dump.decode_rgb332(0x00), (0, 0, 0))
        self.assertEqual(dump.decode_rgb332(0xFF), (255, 255, 255))

    def test_a_hand_computed_code(self):
        # 0x92 is r=100b, g=100b, b=10b. Replicated: 10010010, 10010010, 10101010.
        self.assertEqual(dump.decode_rgb332(0x92), (146, 146, 170))

    def test_every_code_survives_a_decode_and_a_re_encode(self):
        for value in range(1 << 8):
            red, green, blue = dump.decode_rgb332(value)
            self.assertEqual(encode_rgb332(red, green, blue), value, hex(value))

    def test_a_round_trip_stays_within_one_quantisation_step(self):
        steps = (32, 32, 64)
        for colour in ((0, 0, 0), (255, 255, 255), (128, 128, 128), (200, 60, 60)):
            with self.subTest(colour):
                decoded = dump.decode_rgb332(encode_rgb332(*colour))
                for channel, (before, after, step) in enumerate(zip(colour, decoded, steps)):
                    self.assertLess(abs(before - after), step, f'channel {channel}')

    def test_channels_are_not_swapped(self):
        self.assertEqual(dump.decode_rgb332(encode_rgb332(255, 0, 0)), (255, 0, 0))
        self.assertEqual(dump.decode_rgb332(encode_rgb332(0, 255, 0)), (0, 255, 0))
        self.assertEqual(dump.decode_rgb332(encode_rgb332(0, 0, 255)), (0, 0, 255))


class PointStride(unittest.TestCase):
    def test_the_stride_of_every_combination(self):
        expected = {
            ('int16_mm', 'none', 'none'): 6,
            ('int16_mm', 'rgb332', 'none'): 7,
            ('int16_mm', 'rgb565', 'uint8'): 9,
            ('int16_mm', 'rgb888', 'uint8'): 10,
            ('float32', 'none', 'none'): 12,
            ('float32', 'rgb565', 'none'): 14,
            ('float32', 'rgb888', 'uint8'): 16,
        }
        for combination, size in expected.items():
            with self.subTest(combination):
                self.assertEqual(dump.point_stride_bytes(*combination), size)

    def test_a_value_the_gateway_would_reject_is_refused_here_too(self):
        for combination in (('INT16_MM', 'none', 'none'), ('int16_mm', 'RGB565', 'none'),
                            ('int16_mm', 'none', 'float32'), ('int32', 'none', 'none')):
            with self.subTest(combination):
                with self.assertRaises(ValueError):
                    dump.point_stride_bytes(*combination)


class PlanarBlock(unittest.TestCase):
    def assertPointsAlmostEqual(self, points, expected):
        self.assertEqual(len(points), len(expected))
        for index, (got, want) in enumerate(zip(points, expected)):
            with self.subTest(index):
                for axis in range(3):
                    self.assertAlmostEqual(got[axis], want[axis], places=6, msg=f'axis {axis}')
                self.assertEqual(got[3], want[3], 'colour')
                self.assertEqual(got[4], want[4], 'intensity')

    def test_int16_positions_are_reported_in_metres_with_colour_and_intensity(self):
        points, consistent = dump.decode_planar_block(
            BLOCK_INT16, 3, 'int16_mm', 'rgb565', 'uint8')
        self.assertTrue(consistent)
        self.assertPointsAlmostEqual(points, [
            (1.0, 0.0, 0.25, (0, 0, 0), 0),
            (-2.5, 0.5, 0.0, (255, 255, 255), 128),
            (32.767, -0.001, -32.768, (132, 130, 132), 255),
        ])

    def test_float32_positions_pass_through_unscaled(self):
        points, consistent = dump.decode_planar_block(
            BLOCK_FLOAT32, 2, 'float32', 'none', 'none')
        self.assertTrue(consistent)
        self.assertPointsAlmostEqual(points, [
            (1.5, 0.5, 0.0, None, None),
            (-2.0, 0.25, 3.0, None, None),
        ])

    def test_rgb888_is_three_bytes_per_point(self):
        points, consistent = dump.decode_planar_block(
            BLOCK_RGB888, 2, 'int16_mm', 'rgb888', 'none')
        self.assertTrue(consistent)
        self.assertPointsAlmostEqual(points, [
            (0.1, -0.1, 0.0, (1, 2, 3), None),
            (0.2, 0.0, 0.05, (250, 251, 252), None),
        ])

    def test_rgb332_is_one_byte_per_point_so_intensity_lands_where_it_should(self):
        points, consistent = dump.decode_planar_block(
            BLOCK_RGB332, 2, 'int16_mm', 'rgb332', 'uint8')
        self.assertTrue(consistent)
        self.assertPointsAlmostEqual(points, [
            (0.0, 0.0, 0.0, (146, 146, 170), 7),
            (0.0, 0.0, 0.0, (255, 255, 255), 8),
        ])

    def test_a_truncated_block_is_not_consistent_and_yields_fewer_points(self):
        # One byte short of 3 x 9. Planar layout means a short block moves every plane boundary,
        # so the flag is the whole warning -- the numbers that come back are not the sender's.
        points, consistent = dump.decode_planar_block(
            BLOCK_INT16[:-1], 3, 'int16_mm', 'rgb565', 'uint8')
        self.assertFalse(consistent)
        self.assertEqual(len(points), 2)

    def test_a_block_longer_than_the_header_claims_is_not_consistent(self):
        points, consistent = dump.decode_planar_block(
            BLOCK_INT16 + b'\x00' * 4, 3, 'int16_mm', 'rgb565', 'uint8')
        self.assertFalse(consistent)
        self.assertEqual(len(points), 3)

    def test_an_empty_cloud_decodes_to_nothing(self):
        points, consistent = dump.decode_planar_block(b'', 0, 'int16_mm', 'none', 'none')
        self.assertTrue(consistent)
        self.assertEqual(points, [])

    def test_point_json_omits_the_channels_the_format_says_are_absent(self):
        self.assertEqual(dump.point_json((1.0, 2.0, 3.0, None, None)), [1.0, 2.0, 3.0])
        self.assertEqual(dump.point_json((1.0, 2.0, 3.0, (4, 5, 6), None)),
                         [1.0, 2.0, 3.0, [4, 5, 6]])
        self.assertEqual(dump.point_json((1.0, 2.0, 3.0, None, 7)), [1.0, 2.0, 3.0, 7])
        self.assertEqual(dump.point_json((1.0, 2.0, 3.0, (4, 5, 6), 7)),
                         [1.0, 2.0, 3.0, [4, 5, 6], 7])


class Decompression(unittest.TestCase):
    def test_level_zero_means_the_data_already_is_the_block(self):
        self.assertEqual(dump.decompress_block(BLOCK_INT16, 0), BLOCK_INT16)

    def test_a_compressed_block_without_the_dependency_names_the_dependency(self):
        # None in sys.modules makes `import zstandard` raise whether or not it is installed, so
        # the message a caller without it would see is checked on a machine that has it.
        with unittest.mock.patch.dict(sys.modules, {'zstandard': None}):
            with self.assertRaises(RuntimeError) as caught:
                dump.decompress_block(b'irrelevant', 3)
        self.assertIn('zstandard', str(caught.exception))

    @NEEDS_ZSTANDARD
    def test_a_compressed_block_comes_back_byte_for_byte(self):
        compressed = zstandard.ZstdCompressor(level=3).compress(BLOCK_INT16)
        self.assertEqual(dump.decompress_block(compressed, 3), BLOCK_INT16)

    @NEEDS_ZSTANDARD
    def test_a_frame_with_no_content_size_in_its_header_still_reads(self):
        # Legal, and the one-shot `decompress()` refuses it for want of an output size.
        writer = zstandard.ZstdCompressor(level=3, write_content_size=False)
        compressed = writer.stream_reader(io.BytesIO(BLOCK_INT16)).read()
        self.assertEqual(dump.decompress_block(compressed, 3), BLOCK_INT16)


# One pose block body, 2 samples of 2 joints: joint 0 writes position and rotation, joint 1
# position only, one event. Each channel is its first value, then deltas.
POSE_BODY = bytes.fromhex(
    '02000000 02000000'                   # N = 2, J = 2
    '00d0de70fb3d0600'                    # startUs = 1757000000000000
    'a0860100 a0860100 a0860100'          # steps: 1e-4 m, 1e-4, 1e-4
    '00000000 f4010000'                   # times: +0, +500 us
    '03 01'                               # masks: joint 0 p|r, joint 1 p
    '0a000000 05000000'                   # j0 p.x: 10, +5
    '00000000 fdffffff'                   # j0 p.y: 0, -3
    'd0070000 00000000'                   # j0 p.z: 2000, +0
    '00000000 00000000 00000000 00000000 00000000 00000000'  # j0 r.x, r.y, r.z
    '10270000 00000000'                   # j0 r.w: 10000, +0
    'd0070000 00000000 00000000 00000000 00000000 00000000'  # j1 p
    '01000000 00000000 01000000'          # 1 event: sample 0, enabled
    .replace(' ', ''))


class PoseBlock(unittest.TestCase):
    def test_a_body_decodes_to_absolute_times_and_values_in_meters(self):
        decoded = dump.parse_pose_body(POSE_BODY)
        self.assertTrue(decoded['consistent'])
        self.assertEqual(decoded['sampleCount'], 2)
        self.assertEqual(decoded['jointCount'], 2)
        self.assertEqual(decoded['times'], [1757000000000000, 1757000000000500])
        self.assertEqual(decoded['masks'], [3, 1])
        self.assertEqual(sorted(decoded['joints']), ['0', '1'])
        self.assertEqual(sorted(decoded['joints']['0']), ['p', 'r'])
        self.assertEqual(sorted(decoded['joints']['1']), ['p'])
        for got, want in zip(decoded['joints']['0']['p'], [[0.001, 0.0, 0.2],
                                                           [0.0015, -0.0003, 0.2]]):
            for a, b in zip(got, want):
                self.assertAlmostEqual(a, b, places=12)
        for got in decoded['joints']['0']['r']:
            for a, b in zip(got, [0.0, 0.0, 0.0, 1.0]):
                self.assertAlmostEqual(a, b, places=12)
        self.assertAlmostEqual(decoded['joints']['1']['p'][1][0], 0.2, places=12)
        self.assertEqual(decoded['events'], [[0, 1]])

    def test_a_body_with_bytes_past_its_counts_is_not_consistent(self):
        self.assertFalse(dump.parse_pose_body(POSE_BODY + b'\0')['consistent'])

    def test_an_uncompressed_block_reads_its_prefix(self):
        decoded = dump.decode_pose_block(bytes([0xC3, 1, 0, 1]) + POSE_BODY)
        self.assertTrue(decoded['keyframe'])
        self.assertEqual(decoded['codec'], 0)
        self.assertEqual(decoded['sampleCount'], 2)
        self.assertFalse(dump.decode_pose_block(bytes([0xC3, 1, 0, 0]) + POSE_BODY)['keyframe'])

    def test_a_prefix_that_is_not_a_pose_block_is_refused(self):
        for prefix in (bytes([0xC4, 1, 0, 0]), bytes([0xC3, 2, 0, 0]), bytes([0xC3, 1, 9, 0])):
            with self.subTest(prefix.hex()):
                with self.assertRaises(ValueError):
                    dump.decode_pose_block(prefix + POSE_BODY)

    @NEEDS_ZSTANDARD
    def test_a_zstd_block_decompresses_to_the_same_body(self):
        compressed = zstandard.ZstdCompressor(write_checksum=True).compress(POSE_BODY)
        decoded = dump.decode_pose_block(bytes([0xC3, 1, 1, 0]) + compressed)
        self.assertEqual(decoded['times'], [1757000000000000, 1757000000000500])


class JpegFilename(unittest.TestCase):
    def test_the_four_components_in_order(self):
        self.assertEqual(dump.jpeg_filename('1757000000_robot7', 'front', 4, 11),
                         '1757000000_robot7-front-4-11.jpg')

    def test_payload_text_cannot_steer_the_path(self):
        name = dump.jpeg_filename('../../etc', 'a/b', 1, 2)
        self.assertNotIn('/', name)
        self.assertEqual(name, '.._.._etc-a_b-1-2.jpg')


class CommandLine(unittest.TestCase):
    def test_an_undecidable_name_fails_before_anything_is_read(self):
        # Argparse exits 2. This runs with no schemas loaded, which is the point: the whole list
        # is resolved before the first file is opened.
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as caught:
                dump.main(['/nonexistent/capture.dat'])
        self.assertEqual(caught.exception.code, 2)
        self.assertIn('capture.dat', stderr.getvalue())


@NEEDS_CAPNP
class RoundTripThroughTheSchemas(unittest.TestCase):
    """Build a payload with pycapnp, pack it, and read it back through the CLI."""

    def setUp(self):
        self.directory = pathlib.Path(tempfile.mkdtemp())
        self.camera, self.lidar = dump.load_schemas(dump.SCHEMA_DIR)

    def write_header(self, payload):
        payload.userId = 'robot7'
        payload.sessionId = '1757000000_robot7'
        payload.sceneId = 'd3b07384-d9a0-4c9b-8f1e-000000000001'
        payload.sceneVersion = 3
        payload.formatVersion = '1.0'
        payload.sessionStartMs = 1757000000123
        payload.sdkVersion = '1.2.0'
        payload.part = 4

    def camera_capture(self):
        payload = self.camera.CameraPayload.new_message()
        self.write_header(payload)
        frames = payload.init('frames', 2)
        frames[0].cameraId = 'front'
        frames[0].time = 1757000001000
        frames[0].seq = 11
        frames[0].frameId = 'camera_link'
        frames[0].pose.px = 1.5
        frames[0].pose.pz = -0.25
        frames[0].pose.qz = 0.5
        frames[0].pose.qw = 0.8660254
        frames[0].format = 'jpeg'
        frames[0].width = 320
        frames[0].height = 240
        frames[0].hfovDegrees = 61.0
        frames[0].vfovDegrees = 45.5
        frames[0].data = b'\xff\xd8\xff\xe0 pretend jpeg'
        frames[1].cameraId = 'rear'
        frames[1].time = 1757000001200
        frames[1].seq = 12
        frames[1].frameId = 'camera_link'
        frames[1].format = 'jpeg'
        frames[1].width = 64
        frames[1].height = 48
        frames[1].data = b'no marker here'
        path = self.directory / '000007-camera.bin'
        path.write_bytes(payload.to_bytes_packed())
        return path

    def lidar_capture(self):
        payload = self.lidar.LidarPayload.new_message()
        self.write_header(payload)
        scans = payload.init('scans', 2)
        scans[0].sensorId = 'scan'
        scans[0].time = 1757000002000
        scans[0].seq = 5
        scans[0].frameId = 'base_scan'
        laserscan = scans[0].init('laserscan')
        laserscan.angleMin = -math.pi
        laserscan.angleMax = math.pi
        laserscan.angleIncrement = 2 * math.pi / 4
        laserscan.timeIncrement = 0.0001
        laserscan.scanTime = 0.2
        laserscan.rangeMin = 0.1
        laserscan.rangeMax = 12.0
        ranges = laserscan.init('ranges', 4)
        ranges[0] = 1.0
        ranges[1] = math.inf
        ranges[2] = math.nan
        ranges[3] = 2.0
        intensities = laserscan.init('intensities', 2)
        intensities[0] = 7.0
        intensities[1] = 8.0
        scans[1].sensorId = 'points'
        scans[1].time = 1757000002500
        scans[1].seq = 6
        scans[1].frameId = 'base_link'
        cloud = scans[1].init('pointcloud')
        cloud.pointCount = 3
        cloud.position = 'int16_mm'
        cloud.color = 'rgb565'
        cloud.intensity = 'uint8'
        cloud.voxelLeafMm = 50
        cloud.zstdLevel = 0
        cloud.data = BLOCK_INT16
        path = self.directory / '000008-lidar.bin'
        path.write_bytes(payload.to_bytes_packed())
        return path

    def run_cli(self, arguments):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            self.assertEqual(dump.main(arguments), 0)
        return json.loads(stdout.getvalue())

    def test_both_captures_decode_into_one_document(self):
        camera_path = self.camera_capture()
        lidar_path = self.lidar_capture()
        document = self.run_cli([str(camera_path), str(lidar_path)])

        self.assertEqual([entry['stream'] for entry in document['files']], ['camera', 'lidar'])
        camera_entry, lidar_entry = document['files']
        self.assertEqual(camera_entry['file'], str(camera_path))
        self.assertEqual(camera_entry['bytes'], camera_path.stat().st_size)
        self.assertEqual(camera_entry['header'], {
            'userId': 'robot7',
            'sessionId': '1757000000_robot7',
            'sceneId': 'd3b07384-d9a0-4c9b-8f1e-000000000001',
            'sceneVersion': 3,
            'formatVersion': '1.0',
            'sessionStartMs': 1757000000123,
            'sdkVersion': '1.2.0',
            'part': 4,
        })

        first, second = camera_entry['frames']
        self.assertEqual(first['cameraId'], 'front')
        self.assertEqual(first['seq'], 11)
        self.assertEqual(first['frameId'], 'camera_link')
        self.assertEqual((first['width'], first['height']), (320, 240))
        self.assertEqual(first['format'], 'jpeg')
        self.assertEqual(first['dataBytes'], len(b'\xff\xd8\xff\xe0 pretend jpeg'))
        self.assertTrue(first['jpegSoi'])
        self.assertAlmostEqual(first['pose'][0], 1.5, places=6)
        self.assertAlmostEqual(first['pose'][2], -0.25, places=6)
        self.assertAlmostEqual(first['pose'][6], 0.8660254, places=6)
        self.assertFalse(second['jpegSoi'])
        # The field of view, and its absence: a frame that never set it reads 0 by 0.
        self.assertEqual((first['hfovDegrees'], first['vfovDegrees']), (61.0, 45.5))
        self.assertEqual((second['hfovDegrees'], second['vfovDegrees']), (0.0, 0.0))
        # qw defaults to 1, so a pose that was never written reads as the identity rotation at
        # the origin rather than as the zero quaternion, which is not a rotation at all.
        self.assertEqual(second['pose'], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        scan, cloud = lidar_entry['scans']
        self.assertEqual(scan['kind'], 'laserscan')
        self.assertEqual(scan['sensorId'], 'scan')
        self.assertEqual(scan['rangeCount'], 4)
        self.assertEqual(scan['intensityCount'], 2)
        self.assertEqual(scan['finiteRangeCount'], 2)
        self.assertAlmostEqual(scan['rangeMax'], 12.0, places=6)
        self.assertEqual(cloud['kind'], 'pointcloud')
        self.assertEqual(cloud['pointCount'], 3)
        self.assertEqual(cloud['voxelLeafMm'], 50)
        self.assertEqual(cloud['dataBytes'], len(BLOCK_INT16))
        self.assertEqual(cloud['decoded']['points'], 3)
        self.assertTrue(cloud['decoded']['consistent'])
        self.assertEqual(len(cloud['decoded']['first']), 3)
        self.assertEqual(cloud['decoded']['first'][0], [1.0, 0.0, 0.25, [0, 0, 0], 0])

    def test_a_scans_ranges_are_absent_by_default_and_verbatim_with_the_flag(self):
        path = self.lidar_capture()
        scan, cloud = self.run_cli([str(path)])['files'][0]['scans']
        # 640 floats per sweep is most of the document and almost no assertion needs them, so
        # the flag is what a reader placing a ray asks for.
        self.assertNotIn('ranges', scan)
        self.assertNotIn('intensities', scan)

        scan, cloud = self.run_cli(['--scan-ranges', str(path)])['files'][0]['scans']
        self.assertEqual(scan['ranges'][0], 1.0)
        self.assertEqual(scan['ranges'][3], 2.0)
        # As published: the two spellings of "nothing there" survive the round trip through JSON
        # as the non-standard tokens Python writes and reads.
        self.assertTrue(math.isinf(scan['ranges'][1]))
        self.assertTrue(math.isnan(scan['ranges'][2]))
        self.assertEqual(len(scan['ranges']), scan['rangeCount'])
        self.assertEqual(scan['intensities'], [7.0, 8.0])
        self.assertNotIn('ranges', cloud)

    def test_only_the_first_three_points_are_reported(self):
        payload = self.lidar.LidarPayload.new_message()
        self.write_header(payload)
        scan = payload.init('scans', 1)[0]
        cloud = scan.init('pointcloud')
        cloud.pointCount = 8
        cloud.position = 'int16_mm'
        cloud.color = 'none'
        cloud.intensity = 'none'
        cloud.data = bytes(8 * 6)
        path = self.directory / '000009-lidar.bin'
        path.write_bytes(payload.to_bytes_packed())
        decoded = self.run_cli([str(path)])['files'][0]['scans'][0]['decoded']
        self.assertEqual(decoded['points'], 8)
        self.assertEqual(len(decoded['first']), 3)
        self.assertTrue(decoded['consistent'])

    def test_a_short_block_reaches_the_json_as_inconsistent(self):
        payload = self.lidar.LidarPayload.new_message()
        self.write_header(payload)
        scan = payload.init('scans', 1)[0]
        cloud = scan.init('pointcloud')
        cloud.pointCount = 3
        cloud.position = 'int16_mm'
        cloud.color = 'rgb565'
        cloud.intensity = 'uint8'
        cloud.data = BLOCK_INT16[:-1]
        path = self.directory / '000010-lidar.bin'
        path.write_bytes(payload.to_bytes_packed())
        decoded = self.run_cli([str(path)])['files'][0]['scans'][0]['decoded']
        self.assertFalse(decoded['consistent'])
        self.assertEqual(decoded['points'], 2)

    def test_an_explicit_stream_decodes_a_file_named_anything(self):
        capture = self.camera_capture()
        renamed = self.directory / 'capture.dat'
        renamed.write_bytes(capture.read_bytes())
        document = self.run_cli(['--stream', 'camera', str(renamed)])
        self.assertEqual(document['files'][0]['stream'], 'camera')
        self.assertEqual(len(document['files'][0]['frames']), 2)

    def test_extract_jpeg_writes_every_frame_and_counts_them(self):
        capture = self.camera_capture()
        target = self.directory / 'jpegs'
        entry = self.run_cli(['--extract-jpeg', str(target), str(capture)])['files'][0]
        self.assertEqual(entry['extracted'], 2)
        front = target / '1757000000_robot7-front-4-11.jpg'
        rear = target / '1757000000_robot7-rear-4-12.jpg'
        self.assertEqual(front.read_bytes(), b'\xff\xd8\xff\xe0 pretend jpeg')
        self.assertEqual(rear.read_bytes(), b'no marker here')
        self.assertEqual(sorted(p.name for p in target.iterdir()),
                         [front.name, rear.name])

    @NEEDS_ZSTANDARD
    def test_a_compressed_cloud_decodes_through_the_cli(self):
        payload = self.lidar.LidarPayload.new_message()
        self.write_header(payload)
        scan = payload.init('scans', 1)[0]
        cloud = scan.init('pointcloud')
        cloud.pointCount = 3
        cloud.position = 'int16_mm'
        cloud.color = 'rgb565'
        cloud.intensity = 'uint8'
        cloud.zstdLevel = 7
        compressed = zstandard.ZstdCompressor(level=7).compress(BLOCK_INT16)
        cloud.data = compressed
        path = self.directory / '000011-lidar.bin'
        path.write_bytes(payload.to_bytes_packed())
        record = self.run_cli([str(path)])['files'][0]['scans'][0]
        # dataBytes is what was on the wire; the decode is of what came out of it.
        self.assertEqual(record['dataBytes'], len(compressed))
        self.assertNotEqual(record['dataBytes'], len(BLOCK_INT16))
        self.assertTrue(record['decoded']['consistent'])
        self.assertEqual(record['decoded']['first'][2], [32.767, -0.001, -32.768, [132, 130, 132], 255])


if __name__ == '__main__':
    unittest.main(verbosity=2)
