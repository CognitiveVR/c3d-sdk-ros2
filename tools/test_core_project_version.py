"""core/CMakeLists.txt takes project(VERSION) from kSdkVersion rather than declaring its own.

Each case configures a scratch copy of core/CMakeLists.txt next to a version.hpp the case writes,
with CMAKE_PROJECT_INCLUDE pointing at a probe that prints PROJECT_VERSION and stops the configure
right there, before any dependency is looked for. So this needs CMake and a C++ compiler and
nothing else, and it is skipped where either is missing.

The scratch version is deliberately not the real one: a project(VERSION) that still carried a
literal would agree with the real header and pass a check made only against it.

Standard library only (repo rule). Run: python3 tools/test_core_project_version.py
"""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

REPO = pathlib.Path(__file__).resolve().parent.parent
CORE_CMAKE = REPO / 'core' / 'CMakeLists.txt'
REAL_HEADER = REPO / 'core' / 'include' / 'c3d' / 'version.hpp'
PROBE = 'message(STATUS "C3D_PROBE_VERSION=${PROJECT_VERSION}")\nmessage(FATAL_ERROR "probe: stop")\n'
HEADER_TEMPLATE = """#pragma once

namespace c3d {

// A comment that names the constant: constexpr const char* kSdkVersion = "9.9.9";
constexpr const char* kSdkVersion = "%s";

}  // namespace c3d
"""


def have_toolchain():
    return shutil.which('cmake') is not None and (
        shutil.which('c++') is not None or shutil.which('g++') is not None)


@unittest.skipUnless(have_toolchain(), 'needs cmake and a C++ compiler')
class CoreProjectVersion(unittest.TestCase):
    def configure(self, header_text):
        tree = pathlib.Path(tempfile.mkdtemp(prefix='c3d_core_version_'))
        self.addCleanup(shutil.rmtree, tree)
        (tree / 'src' / 'include' / 'c3d').mkdir(parents=True)
        shutil.copy2(CORE_CMAKE, tree / 'src' / 'CMakeLists.txt')
        (tree / 'src' / 'include' / 'c3d' / 'version.hpp').write_text(header_text)
        (tree / 'probe.cmake').write_text(PROBE)
        result = subprocess.run(
            ['cmake', '-S', str(tree / 'src'), '-B', str(tree / 'build'),
             f'-DCMAKE_PROJECT_INCLUDE={tree / "probe.cmake"}'],
            capture_output=True, text=True)
        output = result.stdout + result.stderr
        found = re.search(r'C3D_PROBE_VERSION=(\S*)', output)
        return (found.group(1) if found else None), output

    def test_the_project_version_is_whatever_the_header_declares(self):
        version, output = self.configure(HEADER_TEMPLATE % '7.6.5')
        self.assertEqual(version, '7.6.5', output)

    def test_the_real_tree_agrees_with_its_own_header(self):
        declared = re.search(r'^constexpr const char\* kSdkVersion = "([^"]+)";',
                             REAL_HEADER.read_text(), re.M).group(1)
        version, output = self.configure(REAL_HEADER.read_text())
        self.assertEqual(version, declared, output)

    def test_a_version_that_is_not_three_integers_fails_the_configure(self):
        for bad in ('1.2', '1.2.3-rc1', 'next'):
            with self.subTest(bad):
                version, output = self.configure(HEADER_TEMPLATE % bad)
                self.assertIsNone(version, f'configured as {version} from {bad!r}')
                self.assertIn('three integers', output)


if __name__ == '__main__':
    unittest.main()
