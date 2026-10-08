"""Unit tests for tools/export_world_gltf.sh, run against stand-ins for Gazebo and assimp.

The stand-ins reproduce what the real pair was measured to write: Gazebo's exporter puts the .dae
at `<world>/meshes/<world>.dae` under its working directory and the textures at
`<world>/materials/textures/`, and `assimp export` names the buffer after its output file and
keeps the COLLADA document's texture reference, `../materials/textures/<name>`. Everything else is
the real script, the real COLLADA repair and the real upload check.

Run: uv run tools/test_export_world_gltf.py
"""
import importlib.util
import json
import os
import pathlib
import stat
import subprocess
import sys
import tempfile
import textwrap
import unittest

TOOLS = pathlib.Path(__file__).parent
SCRIPT = TOOLS / 'export_world_gltf.sh'
_SPEC = importlib.util.spec_from_file_location('gltf_bbox', TOOLS / 'gltf_bbox.py')
gltf_bbox = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gltf_bbox)

TEXTURE = b'\x89PNG\r\n\x1a\nnot really a picture'

FAKE_GZ = '''
import os, pathlib
world = 'tiny_room'
meshes = pathlib.Path(world) / 'meshes'
meshes.mkdir(parents=True)
(meshes / (world + '.dae')).write_text(
    '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">'
    '<library_images/></COLLADA>')
textures = pathlib.Path(world) / 'materials' / 'textures'
textures.mkdir(parents=True)
for name in os.environ.get('FAKE_TEXTURES', 'brick.png').split(','):
    if name:
        target = textures / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(%r)
''' % TEXTURE

FAKE_ASSIMP = '''
import json, os, pathlib, struct, sys
_, verb, source, out = sys.argv
out = pathlib.Path(out)
spec = {
    'asset': {'version': '2.0'},
    'scenes': [{'nodes': [0]}],
    'nodes': [{'mesh': 0}],
    'meshes': [{'primitives': [{'attributes': {'POSITION': 0}}]}],
    'accessors': [{'min': [-5.0, 0.0, -4.0], 'max': [5.0, 2.0, 4.0]}],
}
if out.suffix == '.glb':
    payload = json.dumps(spec).encode()
    payload += b' ' * ((4 - len(payload) % 4) % 4)
    chunk = struct.pack('<II', len(payload), 0x4E4F534A) + payload
    out.write_bytes(struct.pack('<III', 0x46546C67, 2, 12 + len(chunk)) + chunk)
    sys.exit(0)
stem = out.stem
spec['buffers'] = [{'uri': stem + '.bin', 'byteLength': 8}]
spec['images'] = [{'uri': '../materials/textures/' + name}
                  for name in os.environ.get('FAKE_IMAGE_URIS', 'brick.png').split(',') if name]
(out.parent / (stem + '.bin')).write_bytes(b'\\0' * 8)
out.write_text(json.dumps(spec))
'''

WORLD = textwrap.dedent('''\
    <?xml version="1.0"?>
    <sdf version="1.8">
      <world name="tiny_room">
      </world>
    </sdf>
    ''')


class ExportWorldGltf(unittest.TestCase):
    def setUp(self):
        self.root = pathlib.Path(tempfile.mkdtemp())
        bin_dir = self.root / 'bin'
        bin_dir.mkdir()
        for name, body in (('gz', FAKE_GZ), ('assimp', FAKE_ASSIMP)):
            path = bin_dir / name
            path.write_text('#!%s\n%s' % (sys.executable, body))
            path.chmod(path.stat().st_mode | stat.S_IXUSR)
        self.world = self.root / 'tiny_room.sdf'
        self.world.write_text(WORLD)
        self.out = self.root / 'out'
        self.env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ.get('PATH', ''))

    def export(self, *extra, **variables):
        env = dict(self.env, **variables)
        return subprocess.run(['bash', str(SCRIPT), str(self.world), str(self.out), *extra],
                              capture_output=True, text=True, env=env, stdin=subprocess.DEVNULL)

    def test_a_default_export_is_upload_ready_and_exits_zero(self):
        result = self.export()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        spec = json.loads((self.out / 'scene.gltf').read_text())
        self.assertEqual([b['uri'] for b in spec['buffers']], ['scene.bin'],
                         'a scene upload stores its buffer as scene.bin and rewrites no reference')
        self.assertTrue((self.out / 'scene.bin').is_file())
        self.assertEqual([i['uri'] for i in spec['images']], ['brick.png'])
        self.assertEqual((self.out / 'brick.png').read_bytes(), TEXTURE)
        self.assertTrue(gltf_bbox.report(self.out / 'scene.gltf', 'scene'))
        self.assertFalse(list(self.out.glob('tiny_room.*')),
                         'nothing named after the world is left for someone to upload instead')

    def test_a_texture_that_resolves_nowhere_is_left_for_the_check_to_report(self):
        result = self.export(FAKE_TEXTURES='')
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('INCOMPLETE', result.stdout)
        self.assertIn('../materials/textures/brick.png', result.stdout)

    def test_two_textures_with_one_name_are_refused_rather_than_one_replacing_the_other(self):
        result = self.export(FAKE_TEXTURES='brick.png,old/brick.png',
                             FAKE_IMAGE_URIS='brick.png,old/brick.png')
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('both called brick.png', result.stderr)

    def test_a_glb_keeps_the_world_name(self):
        result = self.export('glb')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((self.out / 'tiny_room.glb').is_file())
        self.assertFalse((self.out / 'scene.gltf').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
