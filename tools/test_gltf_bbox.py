"""Unit tests for tools/gltf_bbox.py.

Two properties. First, the bounding box is computed THROUGH the node hierarchy: assimp puts the
Z-up correction on the root node's matrix rather than in vertex data, so a bbox taken from
accessor min/max alone calls a correctly repaired scene "on its side" -- a false negative that
says a working fix did not work, and the reason this file walks the tree at all.

Second, the orientation verdict is a heuristic and must not outrun its assumption. It assumes the
world's HEIGHT is its smallest extent. The three cases below are why the wording is what it is:
no rule over the extents alone clears an upright corridor without also clearing a warehouse whose
up-axis repair failed, so the non-upright branch names both explanations instead of concluding.

Run: uv run tools/test_gltf_bbox.py
"""
import importlib.util
import json
import math
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest

MODULE_PATH = pathlib.Path(__file__).parent / 'gltf_bbox.py'
_SPEC = importlib.util.spec_from_file_location('gltf_bbox', MODULE_PATH)
bbox_module = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(bbox_module)


def glb(gltf: dict) -> pathlib.Path:
    """A .glb carrying only a JSON chunk -- bbox() reads accessor min/max and never the buffer."""
    payload = json.dumps(gltf).encode()
    payload += b' ' * ((4 - len(payload) % 4) % 4)
    chunk = struct.pack('<II', len(payload), 0x4E4F534A) + payload
    data = struct.pack('<III', 0x46546C67, 2, 12 + len(chunk)) + chunk
    path = pathlib.Path(tempfile.mkdtemp()) / 'scene.glb'
    path.write_bytes(data)
    return path


def gltf(spec: dict) -> pathlib.Path:
    """The same content as `glb`, but as plain glTF JSON -- what the exporter emits by default."""
    path = pathlib.Path(tempfile.mkdtemp()) / 'scene.gltf'
    path.write_text(json.dumps(spec))
    return path


def scene(node: dict) -> dict:
    """One mesh whose local AABB is a 14.3 x 13.6 x 2.6 box with height on Z, under `node`."""
    node = dict(node)
    node['mesh'] = 0
    return {
        'scenes': [{'nodes': [0]}],
        'nodes': [node],
        'meshes': [{'primitives': [{'attributes': {'POSITION': 0}}]}],
        'accessors': [{'min': [0.0, 0.0, 0.0], 'max': [14.3, 13.6, 2.6]}],
    }


class TransformWalk(unittest.TestCase):
    def test_an_untransformed_node_reports_the_raw_extents(self):
        lo, hi = bbox_module.bbox(glb(scene({})))
        self.assertAlmostEqual(hi[1] - lo[1], 13.6, places=3)

    def test_a_root_rotation_is_applied_rather_than_ignored(self):
        # -90 degrees about X, which is exactly what assimp emits for a Z_UP COLLADA document.
        # Height moves from Z onto Y. A bbox read from accessor min/max would miss this entirely
        # and still report 13.6 on Y.
        half = math.radians(-90.0) / 2.0
        rotation = [math.sin(half), 0.0, 0.0, math.cos(half)]
        lo, hi = bbox_module.bbox(glb(scene({'rotation': rotation})))
        extents = [hi[k] - lo[k] for k in range(3)]
        self.assertAlmostEqual(extents[1], 2.6, places=3)     # height now on Y
        self.assertAlmostEqual(extents[2], 13.6, places=3)
        self.assertEqual(bbox_module.orientation(extents)[0], 'consistent with UPRIGHT')

    def test_a_nested_transform_composes_with_its_parent(self):
        gltf = scene({})
        gltf['nodes'] = [{'children': [1], 'translation': [100.0, 0.0, 0.0]},
                         {'mesh': 0}]
        lo, hi = bbox_module.bbox(glb(gltf))
        self.assertAlmostEqual(lo[0], 100.0, places=3)

    def test_geometry_with_no_positioned_mesh_is_refused_not_reported_as_a_point(self):
        with self.assertRaises(SystemExit):
            bbox_module.bbox(glb({'scenes': [{'nodes': [0]}], 'nodes': [{}]}))


class Formats(unittest.TestCase):
    def test_a_plain_gltf_reads_the_same_as_a_glb(self):
        # The exporter defaults to .gltf because the platform rejects .glb. If this file could
        # only read the container, that default would silently skip the up-axis check -- which is
        # the one check that catches the COLLADA repair having failed.
        spec = scene({})
        self.assertEqual(bbox_module.bbox(glb(spec)), bbox_module.bbox(gltf(spec)))

    def test_the_format_is_sniffed_not_taken_from_the_extension(self):
        # A file someone just converted is exactly where the extension is wrong.
        spec = scene({})
        mislabelled = pathlib.Path(tempfile.mkdtemp()) / 'actually_json.glb'
        mislabelled.write_text(json.dumps(spec))
        self.assertEqual(bbox_module.bbox(mislabelled), bbox_module.bbox(glb(spec)))

    def test_neither_shape_is_refused_clearly(self):
        junk = pathlib.Path(tempfile.mkdtemp()) / 'junk.gltf'
        junk.write_bytes(b'this is not glTF at all')
        with self.assertRaises(SystemExit):
            bbox_module.bbox(junk)


class Truncation(unittest.TestCase):
    def test_a_glb_truncated_after_its_header_is_refused_not_a_traceback(self):
        # Disk-full export, killed converter, partial copy. `off < len(data)` alone lets the
        # chunk-header unpack run off the end and raise struct.error, which is not a verdict a
        # caller can read.
        path = pathlib.Path(tempfile.mkdtemp()) / 'cut.glb'
        path.write_bytes(struct.pack('<III', 0x46546C67, 2, 13) + b'\x01')
        with self.assertRaises(SystemExit):
            bbox_module.load(path)

    def test_a_glb_with_no_json_chunk_is_refused(self):
        path = pathlib.Path(tempfile.mkdtemp()) / 'nojson.glb'
        path.write_bytes(struct.pack('<III', 0x46546C67, 2, 12))
        with self.assertRaises(SystemExit):
            bbox_module.load(path)


class Completeness(unittest.TestCase):
    def spec_with_buffer(self, uri, name='scene'):
        spec = scene({})
        spec['buffers'] = [{'uri': uri, 'byteLength': 4}]
        spec['scenes'] = [{'name': name, 'nodes': [0]}]
        return spec

    def test_a_gltf_whose_bin_is_missing_is_reported_incomplete(self):
        # THE case: assimp writes .gltf then .bin, so a failure between them leaves a valid-looking
        # .gltf with no geometry. Bounds come from accessor min/max, so orientation still passes --
        # and the platform accepts a lone .gltf with 200 and discards it.
        directory = pathlib.Path(tempfile.mkdtemp())
        (directory / 'scene.gltf').write_text(json.dumps(self.spec_with_buffer('scene.bin')))
        self.assertFalse(bbox_module.report(directory / 'scene.gltf'))

    def test_present_and_non_empty_is_complete(self):
        directory = pathlib.Path(tempfile.mkdtemp())
        (directory / 'scene.gltf').write_text(json.dumps(self.spec_with_buffer('scene.bin')))
        (directory / 'scene.bin').write_bytes(b'\x00\x01\x02\x03')
        self.assertTrue(bbox_module.report(directory / 'scene.gltf'))

    def test_an_empty_bin_counts_as_missing(self):
        directory = pathlib.Path(tempfile.mkdtemp())
        (directory / 'scene.gltf').write_text(json.dumps(self.spec_with_buffer('scene.bin')))
        (directory / 'scene.bin').write_bytes(b'')
        self.assertFalse(bbox_module.report(directory / 'scene.gltf'))

    def test_a_data_uri_needs_no_sibling_file(self):
        spec = self.spec_with_buffer('data:application/octet-stream;base64,AAAAAA==')
        self.assertEqual(bbox_module.external_files(spec, pathlib.Path('/nowhere')), [])

    def test_a_glb_embeds_its_buffer_so_nothing_is_referenced(self):
        spec = scene({})
        spec['buffers'] = [{'byteLength': 4}]
        self.assertEqual(bbox_module.external_files(spec, pathlib.Path('/nowhere')), [])

    def test_a_percent_encoded_uri_resolves(self):
        directory = pathlib.Path(tempfile.mkdtemp())
        # The glTF is named to match the buffer: the canonical-name rule and the percent-encoding
        # this test is about are independent, and the fixture must not fail one to exercise the
        # other.
        (directory / 'my scene.gltf').write_text(
            json.dumps(self.spec_with_buffer('my%20scene.bin', name='my scene')))
        # Four bytes, matching the spec's declared byteLength -- this test is about resolving the
        # percent-encoded name, so the file it resolves to has to be a complete one.
        (directory / 'my scene.bin').write_bytes(b'\x00\x01\x02\x03')
        self.assertTrue(bbox_module.report(directory / 'my scene.gltf', 'object'))


class Orientation(unittest.TestCase):
    def test_the_repaired_warehouse_reads_upright(self):
        self.assertEqual(bbox_module.orientation([14.3, 2.6, 13.6])[0], 'consistent with UPRIGHT')

    def test_the_unrepaired_warehouse_is_flagged(self):
        # The case this tool exists to catch: height left on Y because the repair did not take.
        verdict, lines = bbox_module.orientation([14.3, 13.6, 2.6])
        self.assertEqual(verdict, 'SOMETHING TO LOOK AT')
        self.assertTrue(any('up_axis repair' in line for line in lines))

    def test_an_upright_corridor_is_flagged_but_its_innocent_explanation_is_named(self):
        # 100 long, 10 deep, 20 tall: upright, and Y is NOT its shortest extent. Any rule that
        # cleared this would also clear the unrepaired warehouse above, so the tool must not
        # conclude -- it must say which two things it cannot tell apart.
        verdict, lines = bbox_module.orientation([100.0, 20.0, 10.0])
        self.assertEqual(verdict, 'SOMETHING TO LOOK AT')
        self.assertTrue(any('corridor' in line or 'tower' in line for line in lines))
        self.assertFalse(any('ON ITS SIDE' in line for line in lines),
                         'a confident wrong verdict is the defect this wording replaced')

    def test_y_tied_for_shortest_counts_as_shortest(self):
        # `ext.index(min(ext))` returns X here, so a first-occurrence test calls this scene
        # "something to look at" and blames the up_axis repair. A square floor plan ties within
        # float error routinely, so this is not an exotic input.
        self.assertEqual(bbox_module.orientation([10.0, 10.0, 30.0])[0],
                         'consistent with UPRIGHT')

    def test_a_near_cubic_world_is_inconclusive(self):
        self.assertEqual(bbox_module.orientation([10.0, 9.0, 9.5])[0], 'INCONCLUSIVE')

    def test_a_degenerate_flat_scene_does_not_divide_by_zero(self):
        verdict, _ = bbox_module.orientation([10.0, 0.0, 10.0])
        self.assertEqual(verdict, 'consistent with UPRIGHT')




class TruncationIsNotCompleteness(unittest.TestCase):
    """Existence and non-emptiness are a weaker question than "is this file all there"."""

    def test_a_buffer_shorter_than_it_declares_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / 'geometry.bin').write_bytes(b'x')
            self.assertIsNone(bbox_module.shortfall(root / 'geometry.bin', None),
                              'with nothing declared, a non-empty file is all we can ask')
            why = bbox_module.shortfall(root / 'geometry.bin', 1048576)
            self.assertIsNotNone(why, 'a 1-byte file for a declared 1 MB buffer is not complete')
            self.assertIn('1048576', why)

    def test_complete_empty_and_missing_files_are_told_apart(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / 'full.bin').write_bytes(b'0123456789')
            self.assertIsNone(bbox_module.shortfall(root / 'full.bin', 10))
            self.assertIsNone(bbox_module.shortfall(root / 'full.bin', 4),
                              'a file longer than declared is padded, not truncated')
            (root / 'empty.bin').write_bytes(b'')
            self.assertEqual(bbox_module.shortfall(root / 'empty.bin', 10), 'empty')
            self.assertEqual(bbox_module.shortfall(root / 'absent.bin', 10), 'missing')

    def test_external_files_carries_the_declared_length_through(self):
        spec = {'buffers': [{'uri': 'geometry.bin', 'byteLength': 2048}],
                'images': [{'uri': 'texture.png'}]}
        entries = bbox_module.external_files(spec, pathlib.Path('/nowhere'))
        self.assertEqual([(uri, declared) for uri, _, declared in entries],
                         [('geometry.bin', 2048), ('texture.png', None)])


class ATruncatedGlbRefusesRatherThanTracebacks(unittest.TestCase):
    def test_a_json_chunk_shorter_than_its_header_declares(self):
        # A 12-byte glb header, then a chunk header claiming 4096 bytes of JSON, then 3 bytes.
        blob = bbox_module.GLB_MAGIC + struct.pack('<II', 2, 27)
        blob += struct.pack('<II', 4096, 0x4E4F534A) + b'{"a'
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'truncated.glb'
            path.write_bytes(blob)
            with self.assertRaises(SystemExit) as raised:
                bbox_module.load(str(path))
            self.assertIn('truncated', str(raised.exception))

    def test_a_complete_header_over_invalid_json(self):
        payload = b'not json!!!!'
        blob = bbox_module.GLB_MAGIC + struct.pack('<II', 2, 12 + 8 + len(payload))
        blob += struct.pack('<II', len(payload), 0x4E4F534A) + payload
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'garbage.glb'
            path.write_bytes(blob)
            with self.assertRaises(SystemExit) as raised:
                bbox_module.load(str(path))
            self.assertIn('not valid JSON', str(raised.exception))


class ARefusalDoesNotSilenceTheRemainingArguments(unittest.TestCase):
    """The CLI reports every path before deciding the exit code, including one that refuses."""

    def test_a_bad_byte_length_is_a_reason_not_a_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / 'geometry.bin').write_bytes(b'0123')
            why = bbox_module.shortfall(root / 'geometry.bin', '4')
            self.assertIsNotNone(why, 'a string byteLength must not reach a < comparison')
            self.assertIn('not a length', why)
            self.assertIsNotNone(bbox_module.shortfall(root / 'geometry.bin', -1))
            self.assertIsNotNone(bbox_module.shortfall(root / 'geometry.bin', True))

    def test_a_refusing_file_does_not_hide_the_next_one(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            # A .glb truncated after its 12-byte header: load() refuses with SystemExit.
            (root / 'broken.glb').write_bytes(
                bbox_module.GLB_MAGIC + struct.pack('<II', 2, 27)
                + struct.pack('<II', 4096, 0x4E4F534A) + b'{"a')
            (root / 'healthy.gltf').write_text(json.dumps(scene({})))
            result = subprocess.run(
                [sys.executable, str(MODULE_PATH), str(root / 'broken.glb'),
                 str(root / 'healthy.gltf')],
                capture_output=True, text=True, stdin=subprocess.DEVNULL)
            self.assertIn('truncated', result.stdout + result.stderr)
            self.assertIn('healthy.gltf', result.stdout,
                          'the refusal on the first path silenced the second')
            self.assertEqual(result.returncode, 1)


class NothingToCheckIsNotAPass(unittest.TestCase):
    def run_cli(self, *arguments):
        return subprocess.run([sys.executable, str(MODULE_PATH), *arguments],
                              capture_output=True, text=True, stdin=subprocess.DEVNULL)

    def test_no_file_prints_the_usage_and_exits_2(self):
        for arguments in ((), ('--object',), ('--scene',)):
            result = self.run_cli(*arguments)
            self.assertEqual(result.returncode, 2, arguments)
            self.assertIn('usage: gltf_bbox.py', result.stderr, arguments)

    def test_help_prints_the_usage_and_exits_0(self):
        result = self.run_cli('--help')
        self.assertEqual(result.returncode, 0)
        self.assertIn('usage: gltf_bbox.py', result.stdout)


class WhatTheViewerCanActuallyFetch(unittest.TestCase):
    """Read off the loader each kind ACTUALLY takes, not off one intercept.

    A scene's glTF is fetched with `cdn=true` and goes through a custom buffer intercept that
    rewrites the URL. A dynamic object's is not: the viewer builds `objects/{mesh}/{mesh}.gltf`
    from the runtime manifest's mesh name, and a plain glTF loader resolves `buffers[].uri`
    relative to that URL. Applying the scene rule to objects refuses meshes that render.
    """

    def spec(self, uri=None, name=None):
        out = scene({})
        if uri is not None:
            out['buffers'] = [{'uri': uri, 'byteLength': 4}]
        if name is not None:
            out['scenes'] = [{'name': name, 'nodes': [0]}]
        return out

    def test_a_scene_must_reference_scene_bin_whatever_the_gltf_is_called(self):
        # A rule that ignores the scene rename ACCEPTS this, and it matters because this pair is
        # what our own exporter produces by default.
        wrong = bbox_module.unresolvable_references(
            self.spec(uri='warehouse.bin'), '/tmp/warehouse.gltf', False, 'scene')
        self.assertEqual(len(wrong), 1)
        self.assertIn('warehouse.bin', wrong[0][0])

        # ...and refuses this one, which works: the glTF is renamed too.
        self.assertEqual(bbox_module.unresolvable_references(
            self.spec(uri='scene.bin'), '/tmp/depot_ramp.gltf', False, 'scene'), [])

    def test_the_scene_rule_is_EXACT_and_not_a_substring(self):
        # The viewer sends any buffer uri containing `scene.bin` to the SCENE base URL, so
        # `my_scene.bin` is fetched there -- where storage holds the literal `scene.bin` and it
        # 404s. A substring rule here would accept it, and a substring implementation passes
        # every other test.
        for near_miss in ('my_scene.bin', 'scene.bin.bin', 'a/scene.bin', 'Scene.bin'):
            self.assertNotEqual(
                bbox_module.unresolvable_references(
                    self.spec(uri=near_miss), '/tmp/scene.gltf', False, 'scene'), [],
                'accepted a uri that is not exactly scene.bin: %r' % near_miss)

    def test_an_object_may_name_its_buffer_anything_that_sits_beside_the_gltf(self):
        # The common shape of an object that renders: no scene name, buffer named after the
        # mesh.
        self.assertEqual(bbox_module.unresolvable_references(
            self.spec(uri='turtlebot4.bin'), '/tmp/turtlebot4.gltf', False, 'object'), [])
        # A name unrelated to the glTF is equally fine: the loader appends it to the object's own
        # URL, so only the file's presence matters -- which the completeness check answers.
        self.assertEqual(bbox_module.unresolvable_references(
            self.spec(uri='shell.bin'), '/tmp/turtlebot4.gltf', False, 'object'), [])
        # ...including with no scene name at all, which assimp output has and Blender sets to
        # 'Scene'. Neither is read for objects.
        self.assertEqual(bbox_module.unresolvable_references(
            self.spec(uri='geometry.bin', name='Scene'), '/tmp/turtlebot4.gltf', False, 'object'), [])

    def test_an_object_uri_carrying_a_PATH_is_refused(self):
        # The upload stores basenames only, so this resolves on disk, passes completeness, and
        # 404s as objects/{mesh}/buffers/geometry.bin.
        for pathed in ('buffers/geometry.bin', 'sub/dir/geometry.bin',
                       'buffers%2Fgeometry.bin', 'buffers\\geometry.bin'):
            wrong = bbox_module.unresolvable_references(
                self.spec(uri=pathed), '/tmp/turtlebot4.gltf', False, 'object')
            self.assertEqual(len(wrong), 1, 'accepted a uri with a path: %r' % pathed)
            self.assertIn('basenames only', wrong[0][1])

    def test_a_pathed_IMAGE_is_refused_under_both_kinds(self):
        """The upload renames only the glTF and its buffer, so a texture keeps its own name and is
        fetched relative to whichever glTF referenced it. A pathed image therefore fails the same
        way -- as an untextured model rather than an empty one, which is why it is easy to miss."""
        for kind in ('scene', 'object'):
            spec = self.spec(uri='scene.bin' if kind == 'scene' else 'geometry.bin')
            spec['images'] = [{'uri': 'textures/wall.png'}]
            wrong = bbox_module.unresolvable_references(spec, '/tmp/scene.gltf', False, kind)
            self.assertEqual(len(wrong), 1, 'a pathed image passed under %s' % kind)
            self.assertIn('wall.png', wrong[0][0])

        # A bare image name is fine under both, and an embedded one names nothing at all.
        for kind in ('scene', 'object'):
            spec = self.spec(uri='scene.bin' if kind == 'scene' else 'geometry.bin')
            spec['images'] = [{'uri': 'wall.png'}, {'bufferView': 0}, {'uri': 'data:image/png;base64,AA'}]
            self.assertEqual(
                bbox_module.unresolvable_references(spec, '/tmp/scene.gltf', False, kind), [])

    def test_a_leading_dot_slash_resolves_and_must_not_be_refused(self):
        """URL parsing drops a single-dot segment before the request leaves the client, so
        `./geometry.bin` is fetched as `geometry.bin`. Refusing it would be a false refusal."""
        self.assertEqual(bbox_module.unresolvable_references(
            self.spec(uri='./geometry.bin'), '/tmp/turtlebot4.gltf', False, 'object'), [])
        self.assertEqual(bbox_module.unresolvable_references(
            self.spec(uri='./scene.bin'), '/tmp/scene.gltf', False, 'scene'), [])
        # ...but `..` really does climb out of the object's directory.
        self.assertNotEqual(bbox_module.unresolvable_references(
            self.spec(uri='../geometry.bin'), '/tmp/turtlebot4.gltf', False, 'object'), [])

    def test_an_empty_uri_is_not_the_same_as_an_absent_one(self):
        """Only an ABSENT uri means embedded. An empty string is fetched, and resolves to the page
        rather than to an asset."""
        spec = self.spec()
        spec['buffers'] = [{'uri': '', 'byteLength': 4}]
        self.assertEqual(len(bbox_module.unresolvable_references(
            spec, '/tmp/turtlebot4.gltf', False, 'object')), 1)
        spec['buffers'] = [{'byteLength': 4}]
        self.assertEqual(bbox_module.unresolvable_references(
            spec, '/tmp/turtlebot4.gltf', False, 'object'), [])

    def test_a_real_container_is_skipped_by_its_MAGIC_not_its_extension(self):
        """`load()` sniffs the magic, so a JSON glTF misnamed `.glb` must still be checked --
        deciding the skip on the extension would give it a PASS it had not earned."""
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            misnamed = root / 'warehouse.glb'
            misnamed.write_text(json.dumps(self.spec(uri='warehouse.bin')))
            (root / 'warehouse.bin').write_bytes(b'0123')
            self.assertFalse(bbox_module.report(misnamed, 'scene'),
                             'a JSON glTF wearing a .glb name skipped the check')

    def test_the_two_rules_disagree_which_is_why_the_caller_must_say_which(self):
        plain_object = self.spec(uri='turtlebot4.bin')
        self.assertEqual(bbox_module.unresolvable_references(
            plain_object, '/tmp/turtlebot4.gltf', False, 'object'), [])
        self.assertNotEqual(bbox_module.unresolvable_references(
            plain_object, '/tmp/turtlebot4.gltf', False, 'scene'), [])

    def test_an_embedded_buffer_names_nothing_and_is_not_reported(self):
        spec = scene({})
        spec['buffers'] = [{'byteLength': 4}]
        self.assertEqual(bbox_module.unresolvable_references(spec, '/tmp/scene.gltf', False, 'scene'), [])
        spec['buffers'] = [{'uri': 'data:application/octet-stream;base64,AAAA'}]
        self.assertEqual(bbox_module.unresolvable_references(spec, '/tmp/scene.gltf', False, 'scene'), [])

    def test_report_refuses_an_unfetchable_asset_even_when_every_file_is_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / 'warehouse.gltf').write_text(json.dumps(self.spec(uri='warehouse.bin')))
            (root / 'warehouse.bin').write_bytes(b'0123')
            self.assertIsNone(bbox_module.shortfall(root / 'warehouse.bin', 4),
                              'the file itself is complete, which is what makes this silent')
            self.assertFalse(bbox_module.report(root / 'warehouse.gltf', 'scene'))
            # The same files are a perfectly good OBJECT.
            self.assertTrue(bbox_module.report(root / 'warehouse.gltf', 'object'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
