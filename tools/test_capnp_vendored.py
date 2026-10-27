"""The Cap'n Proto schemas under core/capnp/ still are what core/capnp/CHECKSUMS says.

core/capnp/ is a VENDORED copy. Cognitive3D maintains the canonical files, and the platform
reads the bodies this SDK writes with them -- so a field added, renamed or renumbered on one
side and not the other is a wire break that neither side's own tests can see. The checksum
file is the pin, `tools/sync_capnp.sh` is the only thing that should ever rewrite it, and this
suite is what makes a hand edit here fail rather than ship.

The second assertion matters as much as the first: a file present in core/capnp/ but absent
from CHECKSUMS is unpinned, so a schema dropped in without a resync would otherwise be
vendored with nothing watching it.

Standard library only (repo rule). Run: python3 tools/test_capnp_vendored.py
"""
import hashlib
import pathlib
import unittest

VENDOR_DIR = pathlib.Path(__file__).resolve().parent.parent / 'core' / 'capnp'
CHECKSUMS = VENDOR_DIR / 'CHECKSUMS'


def parse_checksums(text):
    """`shasum -a 256` output: "<64 hex>  <name>" per line. Returns {name: hash}."""
    recorded = {}
    for lineno, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        digest, _, name = line.partition('  ')
        if len(digest) != 64 or not name:
            raise ValueError(f'{CHECKSUMS}:{lineno}: not a shasum line: {line!r}')
        recorded[name] = digest
    return recorded


class VendoredSchemasArePinned(unittest.TestCase):
    def setUp(self):
        self.assertTrue(CHECKSUMS.is_file(), f'{CHECKSUMS} is missing; run tools/sync_capnp.sh')
        self.recorded = parse_checksums(CHECKSUMS.read_text())
        # An empty or truncated CHECKSUMS would pass every loop below vacuously.
        self.assertGreater(len(self.recorded), 0, f'{CHECKSUMS} records nothing')

    def test_every_recorded_file_hashes_to_its_recorded_value(self):
        for name, expected in sorted(self.recorded.items()):
            with self.subTest(name):
                path = VENDOR_DIR / name
                self.assertTrue(path.is_file(), f'{name} is recorded but not vendored')
                actual = hashlib.sha256(path.read_bytes()).hexdigest()
                self.assertEqual(
                    actual, expected,
                    f'core/capnp/{name} has been edited in place. These files are copies of '
                    f'the canonical Cognitive3D schemas and are never edited here: a schema '
                    f'change starts in the canonical copy and arrives by tools/sync_capnp.sh.')

    def test_nothing_in_the_directory_is_unpinned(self):
        present = sorted(p.name for p in VENDOR_DIR.iterdir()
                         if p.is_file() and p.name not in ('CHECKSUMS', 'README.md'))
        self.assertEqual(
            present, sorted(self.recorded),
            'core/capnp/ and CHECKSUMS disagree about which files are vendored. '
            'Re-run tools/sync_capnp.sh <schema-dir>.')

    def test_the_schemas_the_streams_need_are_all_here(self):
        # Named rather than counted: a resync that dropped one would still be self-consistent.
        for name in ('camera_payload.capnp', 'lidar_payload.capnp', 'pose_payload.capnp',
                     'common_payload.capnp', 'java.capnp'):
            with self.subTest(name):
                self.assertIn(name, self.recorded)


if __name__ == '__main__':
    unittest.main(verbosity=2)
