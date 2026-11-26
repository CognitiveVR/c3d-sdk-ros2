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

# BLOCK_INT16 with planarDeltaShuffleZstd's position transform: each plane as the differences
# between consecutive points (int16 wrap, the first against 0), low bytes then high bytes. The
# colour and intensity planes are unchanged.
BLOCK_INT16_DELTA = bytes.fromhex(
    'e854c3 03f289'       # X deltas: 1000, -3500, 35267 wrapping to -30269
    '00f40b 0001fe'       # Y deltas: 0, 500, -501
    'fa0600 00ff80'       # Z deltas: 250, -250, -32768
    '0000 ffff 1084'
    '00 80 ff')

# 1 point, same format: (7, -7, 0) mm, colour 0x8410, intensity 42. One value per plane is its own
# delta, so the transformed block is the plain one.
BLOCK_ONE_POINT = bytes.fromhex('0700 f9ff 0000 1084 2a')

# Two scans of four rays. Range codes: 1000, 0xFFFF (+inf), 0xFFFE (NaN), 2000, then
# 0x0000 (-inf), 1500, 1501, 1499.
SCAN_RANGES_UINT16 = bytes.fromhex('e803 ffff feff d007' '0000 dc05 dd05 db05')
# The same codes as deltaShuffleZstd before the zstd: per scan, each code minus the previous
# ray's (ray 0 against 0), then the low byte of every value, then the high byte of every value.
SCAN_RANGES_SHUFFLED = bytes.fromhex(
    'e817ffd2 00dc01fe'   # low bytes of 0x03E8 0xFC17 0xFFFF 0x07D2 | 0x0000 0x05DC 0x0001 0xFFFE
    '03fcff07 000500ff')  # high bytes
SCAN_RANGE_CODES = [1000, 0xFFFF, 0xFFFE, 2000, 0, 1500, 1501, 1499]
# Intensity codes 7, 8, 9, 10, then 100 four times, in both layouts.
SCAN_INTENSITIES_UINT16 = bytes.fromhex('0700 0800 0900 0a00' '6400 6400 6400 6400')
SCAN_INTENSITIES_SHUFFLED = bytes.fromhex('07010101 64000000' '00000000 00000000')

# One chunk's stream, as x264 writes it with four-byte start codes: an IDR frame's access unit
# (delimiter, SPS, PPS, IDR slice), then a P frame's (delimiter, non-IDR slice).
ACCESS_UNIT_IDR = bytes.fromhex(
    '00000001 0910' '00000001 6742c01eaabb' '00000001 68ce3c80' '00000001 65888421')
ACCESS_UNIT_P = bytes.fromhex('00000001 0910' '00000001 419a02')


class InverseDeltaShuffle(unittest.TestCase):
    def test_inverse_delta_shuffle_recovers_a_hand_worked_block(self):
        data = bytes.fromhex('0A 02 FF 05 00 FA 00 00 FF 00 00 FF')
        self.assertEqual(dump.inverse_delta_shuffle(data, 3), [10, 12, 11, 5, 5, 65535])


class DequantiseRange(unittest.TestCase):
    def test_dequantise_restores_the_sentinels(self):
        self.assertEqual(dump.dequantise_range(0xFFFF, 'millimetre'), math.inf)
        self.assertTrue(math.isnan(dump.dequantise_range(0xFFFE, 'centimetre')))
        self.assertEqual(dump.dequantise_range(0, 'millimetre'), -math.inf)
        self.assertAlmostEqual(dump.dequantise_range(12345, 'millimetre'), 12.345)
        self.assertAlmostEqual(dump.dequantise_range(1234, 'centimetre'), 12.34)


class NalUnits(unittest.TestCase):
    def test_split_nal_units_sees_both_start_codes(self):
        data = bytes.fromhex('00000001 67aabb 000001 68cc 000001 65ddeeff')
        self.assertEqual(dump.split_nal_units(data), [(7, 0, 7), (8, 7, 5), (5, 12, 7)])

    def test_a_unit_cut_off_after_its_start_code_has_no_type(self):
        # At the end of the data, and before the next start code: neither has a header byte.
        self.assertEqual(dump.split_nal_units(bytes.fromhex('00000001 0910 00000001')),
                         [(9, 0, 6), (None, 6, 4)])
        self.assertEqual(dump.split_nal_units(bytes.fromhex('000001 00000001 65aa')),
                         [(None, 0, 3), (5, 3, 6)])


class AvcC(unittest.TestCase):
    def test_parse_avcc_reads_profile_and_parameter_sets(self):
        sps = bytes.fromhex('67 42 C0 1E AA BB')
        pps = bytes.fromhex('68 CE 3C 80')
        record = (bytes([1, 0x42, 0xC0, 0x1E, 0xFF, 0xE1]) + len(sps).to_bytes(2, 'big') + sps
                  + bytes([1]) + len(pps).to_bytes(2, 'big') + pps + b'\xFD\xF8\xF8\x00')
        parsed = dump.parse_avcc(record)
        self.assertEqual((parsed['profile'], parsed['level']), (0x42, 0x1E))
        self.assertEqual((parsed['sps'], parsed['pps']), (sps, pps))


class PlanarDeltaShuffle(unittest.TestCase):
    def test_inverse_delta_shuffle_planes_round_trips(self):
        plain = bytes.fromhex(
            '0100 0400 0700'      # X: 1, 4, 7
            '0200 0500 0800'      # Y: 2, 5, 8
            '0300 0600 0900'      # Z: 3, 6, 9
            'ff0000 00ff00 0000ff'
            '0a141e')
        transformed = bytes.fromhex(
            '010303 000000'       # X deltas 1,3,3: low bytes, then high bytes
            '020303 000000'
            '030303 000000'
            'ff0000 00ff00 0000ff'
            '0a141e')
        self.assertEqual(dump.inverse_delta_shuffle_planes(transformed, 3, 2), plain)
        self.assertEqual(dump.inverse_delta_shuffle_planes(plain, 3, 4), plain)
        wrapping = bytes.fromhex('ff7f 0080 0000' + '0000 0000 0000' * 2)
        wrapped = bytes.fromhex('ff0100 7f0080' + '000000 000000' * 2)
        self.assertEqual(dump.inverse_delta_shuffle_planes(wrapped, 3, 2), wrapping)


class AccessUnits(unittest.TestCase):
    def test_each_access_unit_lists_its_nal_types_after_the_delimiter(self):
        self.assertEqual(dump.access_unit_json(ACCESS_UNIT_IDR),
                         {'nalTypes': [9, 7, 8, 5], 'startsWithAud': True})
        self.assertEqual(dump.access_unit_json(ACCESS_UNIT_P),
                         {'nalTypes': [9, 1], 'startsWithAud': True})

    def test_an_access_unit_that_does_not_open_on_a_delimiter_says_so(self):
        cases = {
            'no delimiter at all': ACCESS_UNIT_P[6:],
            'a stray byte before the delimiter': b'\xab' + ACCESS_UNIT_P,
            # Two bytes late: the delimiter's start code is broken, and the SPS comes first.
            'sliced two bytes late': ACCESS_UNIT_IDR[2:],
            'empty': b'',
        }
        for name, unit in cases.items():
            with self.subTest(name):
                self.assertFalse(dump.access_unit_json(unit)['startsWithAud'])


class OffsetsTile(unittest.TestCase):
    def test_spans_back_to_back_from_zero_to_the_last_byte_tile(self):
        self.assertTrue(dump.offsets_tile([(0, 32), (32, 13)], 45))
        self.assertTrue(dump.offsets_tile([], 0))

    def test_a_gap_an_overlap_a_late_start_or_an_unowned_tail_does_not_tile(self):
        cases = {
            'gap': ([(0, 32), (33, 12)], 45),
            'overlap': ([(0, 32), (31, 14)], 45),
            'late start': ([(1, 31), (32, 13)], 45),
            'unowned tail': ([(0, 32), (32, 13)], 46),
            'no frames over bytes': ([], 45),
        }
        for name, (spans, total) in cases.items():
            with self.subTest(name):
                self.assertFalse(dump.offsets_tile(spans, total))


class ScanCodes(unittest.TestCase):
    def test_uint16_codes_are_read_scan_major_little_endian(self):
        self.assertEqual(dump.decode_scan_codes(SCAN_RANGES_UINT16, 4, 'uint16'), SCAN_RANGE_CODES)

    @NEEDS_ZSTANDARD
    def test_a_delta_shuffled_block_is_decompressed_and_summed_per_scan(self):
        compressed = zstandard.ZstdCompressor(level=1).compress(SCAN_RANGES_SHUFFLED)
        self.assertEqual(dump.decode_scan_codes(compressed, 4, 'deltaShuffleZstd'),
                         SCAN_RANGE_CODES)

    def test_an_empty_field_needs_no_zstandard(self):
        # `intensities` is empty unless configured, and reading a block without them must not
        # depend on the one package a scan needs only to decompress something.
        with unittest.mock.patch.dict(sys.modules, {'zstandard': None}):
            self.assertEqual(dump.decode_scan_codes(b'', 4, 'deltaShuffleZstd'), [])

    def test_an_encoding_that_is_neither_enumerant_is_refused(self):
        with self.assertRaises(ValueError):
            dump.decode_scan_codes(SCAN_RANGES_UINT16, 4, 'zstd')


class CloudDecode(unittest.TestCase):
    FORMAT = ('int16_mm', 'rgb565', 'uint8')

    def plain_points(self):
        points, consistent = dump.decode_planar_block(BLOCK_INT16, 3, *self.FORMAT)
        self.assertTrue(consistent)
        return points

    def test_delta_shuffled_positions_come_back_as_the_plain_block(self):
        points, consistent = dump.decode_cloud(
            BLOCK_INT16_DELTA, 0, len(BLOCK_INT16_DELTA), 3, *self.FORMAT,
            'planarDeltaShuffleZstd', 0)
        self.assertTrue(consistent)
        self.assertEqual(points, self.plain_points())

    def test_planar_zstd_positions_are_not_transformed(self):
        points, consistent = dump.decode_cloud(
            BLOCK_INT16, 0, len(BLOCK_INT16), 3, *self.FORMAT, 'planarZstd', 0)
        self.assertTrue(consistent)
        self.assertEqual(points, self.plain_points())

    def test_each_cloud_is_read_from_its_own_slice(self):
        data = BLOCK_INT16_DELTA + BLOCK_ONE_POINT
        points, consistent = dump.decode_cloud(
            data, len(BLOCK_INT16_DELTA), len(BLOCK_ONE_POINT), 1, *self.FORMAT,
            'planarDeltaShuffleZstd', 0)
        self.assertTrue(consistent)
        self.assertEqual([dump.point_json(point) for point in points],
                         [[0.007, -0.007, 0.0, [132, 130, 132], 42]])

    def test_a_slice_past_the_end_of_data_is_not_consistent(self):
        _points, consistent = dump.decode_cloud(
            BLOCK_INT16, 0, len(BLOCK_INT16) + 4, 3, *self.FORMAT, 'planarZstd', 0)
        self.assertFalse(consistent)

    def test_a_short_delta_block_is_read_as_it_stands_and_is_not_consistent(self):
        points, consistent = dump.decode_cloud(
            BLOCK_INT16_DELTA[:10], 0, 10, 3, *self.FORMAT, 'planarDeltaShuffleZstd', 0)
        self.assertFalse(consistent)
        self.assertEqual(len(points), 1)


class StreamFromFilename(unittest.TestCase):
    def test_the_two_capture_suffixes(self):
        self.assertEqual(dump.stream_for_filename('000001-camera.bin'), 'camera')
        self.assertEqual(dump.stream_for_filename('000042-lidar.bin'), 'lidar')
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


class H264Filename(unittest.TestCase):
    def test_the_three_components_in_order(self):
        self.assertEqual(dump.h264_filename('1757000000_robot7', 'front', 4),
                         '1757000000_robot7-front-4.h264')

    def test_payload_text_cannot_steer_the_path(self):
        name = dump.h264_filename('../../etc', 'a/b', 2)
        self.assertNotIn('/', name)
        self.assertEqual(name, '.._.._etc-a_b-2.h264')


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

    HEADER = {
        'userId': 'robot7',
        'sessionId': '1757000000_robot7',
        'sceneId': 'd3b07384-d9a0-4c9b-8f1e-000000000001',
        'sceneVersion': 3,
        'formatVersion': '2.0',
        'sessionStartMs': 1757000000123,
        'sdkVersion': '2.0.0',
    }

    def setUp(self):
        self.directory = pathlib.Path(tempfile.mkdtemp())
        self.camera, self.lidar = dump.load_schemas(dump.SCHEMA_DIR)

    def write_header(self, payload):
        for field, value in self.HEADER.items():
            setattr(payload, field, value)

    def write_capture(self, payload, name):
        path = self.directory / name
        path.write_bytes(payload.to_bytes_packed())
        return path

    def camera_capture(self, name='000007-camera.bin', keyframes=(True, False), gap=b''):
        """One chunk of an IDR frame then a P frame, flagged `keyframes`, with `gap` between."""
        payload = self.camera.CameraPayload.new_message()
        self.write_header(payload)
        payload.cameraId = 'front'
        payload.seq = 4
        payload.startMs = 1757000001000
        payload.endMs = 1757000001500
        payload.codec = 'h264'
        payload.width = 160
        payload.height = 120
        payload.nominalFps = 2.0
        payload.frameId = 'camera_link'
        payload.hfovDegrees = 61.0
        payload.vfovDegrees = 45.5
        frames = payload.init('frames', 2)
        frames[0].time = 1757000001000
        frames[0].seq = 11
        frames[0].pose.px = 1.5
        frames[0].pose.pz = -0.25
        frames[0].pose.qz = 0.5
        frames[0].pose.qw = 0.8660254
        frames[0].byteOffset = 0
        frames[0].byteLength = len(ACCESS_UNIT_IDR)
        frames[0].keyframe = keyframes[0]
        frames[1].time = 1757000001500
        frames[1].seq = 12
        frames[1].byteOffset = len(ACCESS_UNIT_IDR) + len(gap)
        frames[1].byteLength = len(ACCESS_UNIT_P)
        frames[1].keyframe = keyframes[1]
        payload.data = ACCESS_UNIT_IDR + gap + ACCESS_UNIT_P
        return self.write_capture(payload, name)

    def scan_capture(self, encoding, ranges, intensities, name='000008-lidar.bin'):
        """Two scans of four rays, with their codes already laid out as `encoding` says."""
        payload = self.lidar.LidarPayload.new_message()
        self.write_header(payload)
        payload.sensorId = 'scan'
        payload.seq = 2
        payload.startMs = 1757000002000
        payload.endMs = 1757000002500
        payload.frameId = 'base_scan'
        block = payload.init('scans')
        block.angleMin = -math.pi
        block.angleMax = math.pi
        block.angleIncrement = 2 * math.pi / 4
        block.rangeMin = 0.1
        block.rangeMax = 12.0
        block.rays = 4
        block.rangeUnit = 'millimetre'
        block.encoding = encoding
        entries = block.init('entries', 2)
        for index in range(2):
            entries[index].time = 1757000002000 + 500 * index
            entries[index].seq = 5 + index
            entries[index].pose.px = 0.25 * (index + 1)
            entries[index].timeIncrement = 0.0001 * (index + 1)
            entries[index].scanTime = 0.25 * (index + 1)
        block.ranges = ranges
        block.intensities = intensities
        return self.write_capture(payload, name)

    def cloud_capture(self, clouds, encoding='planarZstd', zstd_level=0, color='rgb565',
                      intensity='uint8', name='000009-lidar.bin'):
        """`clouds` is `(pointCount, block as stored)` per cloud, laid back to back in `data`."""
        payload = self.lidar.LidarPayload.new_message()
        self.write_header(payload)
        payload.sensorId = 'depth'
        payload.startMs = 1757000003000
        payload.endMs = 1757000003000 + 1000 * (len(clouds) - 1)
        payload.frameId = 'base_link'
        block = payload.init('clouds')
        block.position = 'int16_mm'
        block.color = color
        block.intensity = intensity
        block.voxelLeafMm = 50
        block.encoding = encoding
        block.zstdLevel = zstd_level
        entries = block.init('entries', len(clouds))
        offset = 0
        for index, (count, stored) in enumerate(clouds):
            entries[index].time = 1757000003000 + 1000 * index
            entries[index].seq = index
            entries[index].pointCount = count
            entries[index].byteOffset = offset
            entries[index].byteLength = len(stored)
            offset += len(stored)
        block.data = b''.join(stored for _count, stored in clouds)
        return self.write_capture(payload, name)

    def run_cli(self, arguments):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            self.assertEqual(dump.main(arguments), 0)
        return json.loads(stdout.getvalue())

    def assertDecodedScans(self, entries):
        """The two scans' ranges and intensities, from the codes the fixtures carry."""
        first, second = entries
        self.assertEqual(first['ranges'][0], 1.0)
        self.assertEqual(first['ranges'][1], math.inf)
        self.assertTrue(math.isnan(first['ranges'][2]))
        self.assertEqual(first['ranges'][3], 2.0)
        self.assertEqual(second['ranges'][0], -math.inf)
        for got, want in zip(second['ranges'][1:], (1.5, 1.501, 1.499)):
            self.assertAlmostEqual(got, want, places=9)
        self.assertEqual(first['intensities'], [7, 8, 9, 10])
        self.assertEqual(second['intensities'], [100, 100, 100, 100])

    def test_both_captures_decode_into_one_document(self):
        camera_path = self.camera_capture()
        scan_path = self.scan_capture('uint16', SCAN_RANGES_UINT16, SCAN_INTENSITIES_UINT16)
        cloud_path = self.cloud_capture([(3, BLOCK_INT16)])
        document = self.run_cli([str(camera_path), str(scan_path), str(cloud_path)])

        self.assertEqual([entry['stream'] for entry in document['files']],
                         ['camera', 'lidar', 'lidar'])
        chunk, scans, clouds = document['files']
        self.assertEqual(chunk['file'], str(camera_path))
        self.assertEqual(chunk['bytes'], camera_path.stat().st_size)
        for entry in document['files']:
            self.assertEqual(entry['header'], self.HEADER)

        self.assertEqual(
            {key: chunk[key] for key in ('cameraId', 'seq', 'startMs', 'endMs', 'codec', 'width',
                                         'height', 'nominalFps', 'frameId', 'hfovDegrees',
                                         'vfovDegrees', 'dataBytes')},
            {'cameraId': 'front', 'seq': 4, 'startMs': 1757000001000, 'endMs': 1757000001500,
             'codec': 'h264', 'width': 160, 'height': 120, 'nominalFps': 2.0,
             'frameId': 'camera_link', 'hfovDegrees': 61.0, 'vfovDegrees': 45.5,
             'dataBytes': len(ACCESS_UNIT_IDR) + len(ACCESS_UNIT_P)})
        self.assertTrue(chunk['offsetsTile'])
        self.assertTrue(chunk['keyframesMatchNalTypes'])
        first, second = chunk['frames']
        self.assertEqual(
            {key: first[key] for key in ('time', 'seq', 'byteOffset', 'byteLength', 'keyframe',
                                         'nalTypes', 'startsWithAud')},
            {'time': 1757000001000, 'seq': 11, 'byteOffset': 0,
             'byteLength': len(ACCESS_UNIT_IDR), 'keyframe': True, 'nalTypes': [9, 7, 8, 5],
             'startsWithAud': True})
        self.assertEqual((second['byteOffset'], second['keyframe'], second['nalTypes']),
                         (len(ACCESS_UNIT_IDR), False, [9, 1]))
        self.assertAlmostEqual(first['pose'][0], 1.5, places=6)
        self.assertAlmostEqual(first['pose'][2], -0.25, places=6)
        self.assertAlmostEqual(first['pose'][6], 0.8660254, places=6)
        # qw defaults to 1, so a pose that was never written reads as the identity rotation at
        # the origin rather than as the zero quaternion, which is not a rotation at all.
        self.assertEqual(second['pose'], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        # The chunk's fields on every frame, for a reader that walks frames.
        for frame in chunk['frames']:
            self.assertEqual(
                {key: frame[key] for key in ('cameraId', 'frameId', 'width', 'height',
                                             'hfovDegrees', 'vfovDegrees')},
                {'cameraId': 'front', 'frameId': 'camera_link', 'width': 160, 'height': 120,
                 'hfovDegrees': 61.0, 'vfovDegrees': 45.5})

        self.assertEqual(
            {key: scans[key] for key in ('sensorId', 'seq', 'startMs', 'endMs', 'frameId', 'kind',
                                         'rays', 'rangeUnit', 'encoding', 'finiteRangeCount')},
            {'sensorId': 'scan', 'seq': 2, 'startMs': 1757000002000, 'endMs': 1757000002500,
             'frameId': 'base_scan', 'kind': 'scans', 'rays': 4, 'rangeUnit': 'millimetre',
             'encoding': 'uint16', 'finiteRangeCount': 5})
        self.assertAlmostEqual(scans['angleMin'], -math.pi, places=6)
        self.assertAlmostEqual(scans['rangeMax'], 12.0, places=6)
        self.assertEqual([entry['seq'] for entry in scans['entries']], [5, 6])
        for index, entry in enumerate(scans['entries']):
            self.assertEqual(entry['time'], 1757000002000 + 500 * index)
            self.assertAlmostEqual(entry['pose'][0], 0.25 * (index + 1), places=6)
            # Timing is the scan's own, not the block's.
            self.assertAlmostEqual(entry['timeIncrement'], 0.0001 * (index + 1), places=9)
            self.assertAlmostEqual(entry['scanTime'], 0.25 * (index + 1), places=6)
            # The block's fields on every entry, for a reader that walks scans.
            self.assertEqual((entry['sensorId'], entry['frameId'], entry['kind']),
                             ('scan', 'base_scan', 'scans'))
            self.assertAlmostEqual(entry['angleMin'], -math.pi, places=6)
            self.assertAlmostEqual(entry['angleIncrement'], math.pi / 2, places=6)

        self.assertEqual(
            {key: clouds[key] for key in ('sensorId', 'kind', 'position', 'color', 'intensity',
                                          'voxelLeafMm', 'encoding', 'zstdLevel')},
            {'sensorId': 'depth', 'kind': 'clouds', 'position': 'int16_mm', 'color': 'rgb565',
             'intensity': 'uint8', 'voxelLeafMm': 50, 'encoding': 'planarZstd', 'zstdLevel': 0})
        (cloud,) = clouds['entries']
        self.assertEqual((cloud['pointCount'], cloud['byteOffset'], cloud['byteLength']),
                         (3, 0, len(BLOCK_INT16)))
        self.assertEqual(cloud['decoded']['points'], 3)
        self.assertTrue(cloud['decoded']['consistent'])
        self.assertEqual(cloud['decoded']['first'][0], [1.0, 0.0, 0.25, [0, 0, 0], 0])

    def test_a_keyframe_flag_the_bitstream_disagrees_with_is_reported(self):
        # A P frame flagged keyframe, then an IDR frame left unflagged. In each chunk the other
        # frame agrees, so a check of one frame, or of any frame, would call the chunk sound.
        for flags in ((True, True), (False, False)):
            with self.subTest(flags=flags):
                chunk = self.run_cli([str(self.camera_capture(keyframes=flags))])['files'][0]
                self.assertEqual([frame['keyframe'] for frame in chunk['frames']], list(flags))
                self.assertFalse(chunk['keyframesMatchNalTypes'])
                self.assertTrue(chunk['offsetsTile'])

    def test_a_byte_between_two_access_units_does_not_tile(self):
        chunk = self.run_cli([str(self.camera_capture(gap=b'\xff'))])['files'][0]
        self.assertFalse(chunk['offsetsTile'])
        self.assertTrue(chunk['keyframesMatchNalTypes'])
        # Each frame is still read at its own offset, so both access units decode as they are.
        self.assertEqual([frame['nalTypes'] for frame in chunk['frames']], [[9, 7, 8, 5], [9, 1]])

    def test_scan_ranges_are_absent_by_default_and_decoded_with_the_flag(self):
        path = self.scan_capture('uint16', SCAN_RANGES_UINT16, SCAN_INTENSITIES_UINT16)
        # Most of the document and almost no assertion needs them, so the flag is what a reader
        # placing a ray asks for.
        for entry in self.run_cli([str(path)])['files'][0]['entries']:
            self.assertNotIn('ranges', entry)
            self.assertNotIn('intensities', entry)
        # The three sentinels come back as the non-finite values they stand for, and survive the
        # trip through JSON as the non-standard tokens Python writes and reads.
        self.assertDecodedScans(self.run_cli(['--scan-ranges', str(path)])['files'][0]['entries'])

    @NEEDS_ZSTANDARD
    def test_a_delta_shuffled_scan_block_decodes_through_the_cli(self):
        compressor = zstandard.ZstdCompressor(level=1)
        path = self.scan_capture('deltaShuffleZstd', compressor.compress(SCAN_RANGES_SHUFFLED),
                                 compressor.compress(SCAN_INTENSITIES_SHUFFLED))
        block = self.run_cli(['--scan-ranges', str(path)])['files'][0]
        self.assertEqual(block['finiteRangeCount'], 5)
        self.assertDecodedScans(block['entries'])

    def test_only_the_first_three_points_are_reported(self):
        path = self.cloud_capture([(8, bytes(8 * 6))], color='none', intensity='none')
        decoded = self.run_cli([str(path)])['files'][0]['entries'][0]['decoded']
        self.assertEqual(decoded['points'], 8)
        self.assertEqual(len(decoded['first']), 3)
        self.assertTrue(decoded['consistent'])

    def test_a_short_block_reaches_the_json_as_inconsistent(self):
        path = self.cloud_capture([(3, BLOCK_INT16[:-1])])
        decoded = self.run_cli([str(path)])['files'][0]['entries'][0]['decoded']
        self.assertFalse(decoded['consistent'])
        self.assertEqual(decoded['points'], 2)

    @NEEDS_ZSTANDARD
    def test_each_compressed_delta_shuffled_cloud_decodes_from_its_own_slice(self):
        compressor = zstandard.ZstdCompressor(level=1)
        stored = [compressor.compress(BLOCK_INT16_DELTA), compressor.compress(BLOCK_ONE_POINT)]
        path = self.cloud_capture([(3, stored[0]), (1, stored[1])],
                                  encoding='planarDeltaShuffleZstd', zstd_level=1)
        first, second = self.run_cli([str(path)])['files'][0]['entries']
        # The slice is what was on the wire; the decode is of what came out of it.
        self.assertEqual((first['byteOffset'], first['byteLength']), (0, len(stored[0])))
        self.assertEqual((second['byteOffset'], second['byteLength']),
                         (len(stored[0]), len(stored[1])))
        self.assertTrue(first['decoded']['consistent'])
        self.assertTrue(second['decoded']['consistent'])
        self.assertEqual(first['decoded']['first'],
                         [[1.0, 0.0, 0.25, [0, 0, 0], 0],
                          [-2.5, 0.5, 0.0, [255, 255, 255], 128],
                          [32.767, -0.001, -32.768, [132, 130, 132], 255]])
        self.assertEqual(second['decoded']['first'], [[0.007, -0.007, 0.0, [132, 130, 132], 42]])

    def test_an_explicit_stream_decodes_a_file_named_anything(self):
        capture = self.camera_capture()
        renamed = self.directory / 'capture.dat'
        renamed.write_bytes(capture.read_bytes())
        document = self.run_cli(['--stream', 'camera', str(renamed)])
        self.assertEqual(document['files'][0]['stream'], 'camera')
        self.assertEqual(len(document['files'][0]['frames']), 2)

    def test_extract_h264_writes_each_chunk_as_its_own_stream(self):
        capture = self.camera_capture()
        target = self.directory / 'streams'
        entry = self.run_cli(['--extract-h264', str(target), str(capture)])['files'][0]
        written = target / '1757000000_robot7-front-4.h264'
        self.assertEqual(entry['extracted'], str(written))
        # The chunk's `data` exactly: a raw Annex-B stream that `ffmpeg -f h264` reads as it is.
        self.assertEqual(written.read_bytes(), ACCESS_UNIT_IDR + ACCESS_UNIT_P)
        self.assertEqual([path.name for path in target.iterdir()], [written.name])


if __name__ == '__main__':
    unittest.main(verbosity=2)
