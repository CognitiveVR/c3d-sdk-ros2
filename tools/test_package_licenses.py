"""Every package carries the repository's LICENSE, byte for byte, and its manifest names it.

bloom writes the file a manifest's `<license file="...">` points at into the deb's
`debian/copyright`, so the copy inside each package directory is the licence text a robot
actually receives. The copies exist because a path outside the package does not survive a
release repository that splits the packages apart; the cost is that they can drift from the
root, and this suite is what makes a drifted copy fail rather than ship.

Packages are FOUND, not listed: a package added later is checked without anyone remembering to.

Standard library only (repo rule). Run: python3 tools/test_package_licenses.py
"""
import pathlib
import unittest
import xml.etree.ElementTree as ET

REPO = pathlib.Path(__file__).resolve().parent.parent
ROOT_LICENSE = REPO / 'LICENSE'
LICENSE_ID = 'LicenseRef-Cognitive3D-SDK'


def package_dirs(repo):
    """Every directory directly under the repo root that holds a package.xml."""
    return sorted(p.parent for p in repo.glob('*/package.xml'))


def license_elements(manifest_text):
    """The `<license>` elements of a package.xml, as (text, file attribute or None) pairs."""
    if '<!DOCTYPE' in manifest_text:
        raise ValueError('a package.xml with a DOCTYPE is refused rather than parsed')
    root = ET.fromstring(manifest_text)
    return [((e.text or '').strip(), e.get('file')) for e in root.findall('license')]


class LicenseElementParser(unittest.TestCase):
    def test_reads_the_text_and_the_file_attribute(self):
        self.assertEqual(
            license_elements('<package><license file="LICENSE">X</license></package>'),
            [('X', 'LICENSE')])

    def test_a_license_without_a_file_has_none(self):
        self.assertEqual(license_elements('<package><license>X</license></package>'),
                         [('X', None)])

    def test_refuses_a_doctype(self):
        with self.assertRaises(ValueError):
            license_elements('<!DOCTYPE p [<!ENTITY e "x">]><package/>')


class EveryPackageCarriesTheLicense(unittest.TestCase):
    def setUp(self):
        self.assertTrue(ROOT_LICENSE.is_file(), f'{ROOT_LICENSE} is missing')
        self.root_bytes = ROOT_LICENSE.read_bytes()
        self.packages = package_dirs(REPO)
        # Three ship today. Fewer means the glob broke, and every loop below would pass vacuously.
        self.assertGreaterEqual(len(self.packages), 3, f'found only {self.packages}')

    def test_each_copy_is_byte_identical_to_the_root_license(self):
        for package in self.packages:
            with self.subTest(package.name):
                copy = package / 'LICENSE'
                self.assertTrue(copy.is_file(), f'{package.name}/LICENSE is missing')
                self.assertEqual(
                    copy.read_bytes(), self.root_bytes,
                    f'{package.name}/LICENSE differs from the root LICENSE. The root is the '
                    f'source: copy it over the package copy (cp LICENSE {package.name}/LICENSE).')

    def test_each_manifest_names_the_license_and_points_at_the_copy(self):
        for package in self.packages:
            with self.subTest(package.name):
                elements = license_elements((package / 'package.xml').read_text())
                self.assertEqual(
                    elements, [(LICENSE_ID, 'LICENSE')],
                    f'{package.name}/package.xml should carry exactly '
                    f'<license file="LICENSE">{LICENSE_ID}</license>')


if __name__ == '__main__':
    unittest.main()
