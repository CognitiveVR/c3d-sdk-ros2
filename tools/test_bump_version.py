"""tools/bump_version.sh rewrites every version declaration, and nothing else.

Each case runs the script inside a scratch copy of the declarations: the real version.hpp, the
real manifests, and one extra package the script has never heard of, so that "every package.xml
is found" is tested rather than assumed. A file outside the declarations rides along to prove the
script leaves it alone, and so does a dependency constraint that carries the same version string.

Standard library only (repo rule). Run: python3 tools/test_bump_version.py
"""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

REPO = pathlib.Path(__file__).resolve().parent.parent
SCRIPT = REPO / 'tools' / 'bump_version.sh'
HEADER = pathlib.Path('core/include/c3d/version.hpp')
PREFIX = 'constexpr const char* kSdkVersion = "'


def header_version(text):
    for line in text.splitlines():
        if line.startswith(PREFIX):
            return line[len(PREFIX):].split('"', 1)[0]
    return None


def manifest_version(text):
    start = text.find('<version>')
    if start < 0:
        return None
    start += len('<version>')
    return text[start:text.find('</version>', start)]


class BumpVersion(unittest.TestCase):
    def setUp(self):
        self.tree = pathlib.Path(tempfile.mkdtemp(prefix='c3d_bump_'))
        self.addCleanup(shutil.rmtree, self.tree)
        (self.tree / 'tools').mkdir()
        shutil.copy2(SCRIPT, self.tree / 'tools' / SCRIPT.name)
        (self.tree / HEADER).parent.mkdir(parents=True)
        shutil.copy2(REPO / HEADER, self.tree / HEADER)
        self.version = header_version((REPO / HEADER).read_text())
        self.assertIsNotNone(self.version, 'the real version.hpp declares no kSdkVersion')
        self.manifests = []
        for manifest in sorted(REPO.glob('*/package.xml')):
            target = self.tree / manifest.parent.name / 'package.xml'
            target.parent.mkdir()
            shutil.copy2(manifest, target)
            self.manifests.append(target)
        self.assertGreaterEqual(len(self.manifests), 3, 'the real manifests were not found')
        extra = self.tree / 'zz_new_package' / 'package.xml'
        extra.parent.mkdir()
        extra.write_text(
            '<?xml version="1.0"?>\n<package format="3">\n  <name>zz_new_package</name>\n'
            f'  <version>{self.version}</version>\n'
            f'  <depend version_gte="{self.version}">cognitive3d_ros</depend>\n</package>\n')
        self.manifests.append(extra)
        self.bystander = self.tree / 'README.md'
        self.bystander.write_text(f'The tree declares `{self.version}`.\n')

    def run_script(self, *args):
        return subprocess.run(['bash', str(self.tree / 'tools' / SCRIPT.name), *args],
                              capture_output=True, text=True)

    def snapshot(self):
        return {p.relative_to(self.tree): p.read_bytes()
                for p in self.tree.rglob('*') if p.is_file()}

    def test_a_bump_rewrites_every_declaration(self):
        result = self.run_script('9.8.7')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(header_version((self.tree / HEADER).read_text()), '9.8.7')
        for manifest in self.manifests:
            with self.subTest(manifest.parent.name):
                self.assertEqual(manifest_version(manifest.read_text()), '9.8.7')

    def test_a_bump_changes_nothing_but_the_declaration_lines(self):
        before = self.snapshot()
        self.assertEqual(self.run_script('9.8.7').returncode, 0)
        after = self.snapshot()
        self.assertEqual(sorted(after), sorted(before), 'the bump created or removed a file')
        changed = sorted(str(p) for p in before if before[p] != after[p])
        expected = sorted([str(HEADER)] + [str(m.relative_to(self.tree)) for m in self.manifests])
        self.assertEqual(changed, expected)
        for path in before:
            old = before[path].decode().splitlines()
            new = after[path].decode().splitlines()
            self.assertEqual(len(old), len(new), path)
            differing = [(a, b) for a, b in zip(old, new) if a != b]
            self.assertLessEqual(len(differing), 1, f'{path}: more than one line changed')
            for a, b in differing:
                self.assertEqual(a.replace(self.version, '9.8.7'), b, path)
        self.assertIn(f'version_gte="{self.version}"', (self.tree / 'zz_new_package' /
                                                        'package.xml').read_text(),
                      "a dependency's constraint was rewritten as though it were a declaration")

    def test_the_file_mode_survives(self):
        (self.tree / HEADER).chmod(0o644)
        self.assertEqual(self.run_script('9.8.7').returncode, 0)
        self.assertEqual((self.tree / HEADER).stat().st_mode & 0o777, 0o644)

    def test_check_passes_on_agreement_and_fails_on_any_other_version(self):
        self.assertEqual(self.run_script('--check', self.version).returncode, 0)
        result = self.run_script('--check', '9.8.7')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('version.hpp', result.stderr)

    def test_check_names_the_one_declaration_that_disagrees(self):
        extra = self.tree / 'zz_new_package' / 'package.xml'
        extra.write_text(extra.read_text().replace(f'<version>{self.version}<',
                                                   '<version>0.0.1<', 1))
        result = self.run_script('--check', self.version)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('zz_new_package/package.xml declares 0.0.1', result.stderr)
        self.assertNotIn('version.hpp declares', result.stderr)

    def test_a_tree_that_already_disagrees_is_refused_and_left_alone(self):
        extra = self.tree / 'zz_new_package' / 'package.xml'
        extra.write_text(extra.read_text().replace(f'<version>{self.version}<',
                                                   '<version>0.0.1<', 1))
        before = self.snapshot()
        result = self.run_script('9.8.7')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('disagree', result.stderr)
        self.assertEqual(self.snapshot(), before)

    def test_anything_but_three_integers_is_refused_and_changes_nothing(self):
        before = self.snapshot()
        for bad in ('1.2', 'v1.2.3', '1.2.3-rc1', '1.2.3.4', '', 'x.y.z'):
            with self.subTest(bad):
                result = self.run_script(bad)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(self.run_script('--check', bad).returncode, 2)
        self.assertEqual(self.snapshot(), before)

    def test_bumping_to_the_current_version_is_an_error(self):
        self.assertNotEqual(self.run_script(self.version).returncode, 0)


if __name__ == '__main__':
    unittest.main()
