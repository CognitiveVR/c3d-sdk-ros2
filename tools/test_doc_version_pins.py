"""The docs pin the release in places tools/bump_version.sh does not reach, and each of them must
name the version the tree declares, `kSdkVersion` in core/include/c3d/version.hpp.

A missed one ships a quickstart that clones, downloads or links the previous release. The pins
are the ones CONTRIBUTING.md lists for a bump: the `--branch v<x.y.z>` clones, `SDK_VERSION=`, the
example deb file name, the example startup and doctor lines, the version the skill targets, its
`/blob/v<x.y.z>/` links, and CHANGELOG.md's newest section and its link at the foot of the file.
Every match anywhere in the docs is checked, and each pin must still be found where it is listed,
so the check cannot pass by matching nothing. CHANGELOG.md's entries are history and are not
scanned.

The release workflow runs this before it builds. Standard library only (repo rule).
Run: python3 tools/test_doc_version_pins.py
"""
import pathlib
import re
import unittest

REPO = pathlib.Path(__file__).resolve().parent.parent
VERSION_HPP = REPO / 'core' / 'include' / 'c3d' / 'version.hpp'
CHANGELOG = REPO / 'CHANGELOG.md'
SKILL = '.claude/skills/integrate-cognitive3d-ros2'
V = r'([0-9]+\.[0-9]+\.[0-9]+)'

# (what it is, the pattern with the version as its one group, the files that must carry it)
PINS = [
    ('a --branch clone', rf'--branch v{V}\b',
     ['README.md', 'docs/install.md', '.claude/skills/README.md']),
    ('a versioned link', rf'/blob/v{V}/',
     [f'{SKILL}/SKILL.md', f'{SKILL}/params.template.yaml']),
    ('SDK_VERSION=', rf'\bSDK_VERSION={V}\b', ['docs/install.md']),
    ('an example deb file name', rf'\bros-[a-z]+-cognitive3d-ros[a-z0-9-]*_{V}-0',
     ['docs/install.md']),
    ('an example startup or doctor line', rf'\bcognitive3d_ros {V} [(d]',
     ['docs/install.md', 'docs/doctor-and-verification.md']),
    ('the version the skill targets', rf'\btargets SDK {V}\b', [f'{SKILL}/SKILL.md']),
]


def declared_version():
    match = re.search(r'kSdkVersion = "([^"]+)"', VERSION_HPP.read_text())
    assert match, f'no kSdkVersion in {VERSION_HPP}'
    return match.group(1)


def scanned_files():
    files = [REPO / 'README.md', REPO / 'CONTRIBUTING.md', REPO / 'tools' / 'README.md']
    files += sorted((REPO / 'docs').glob('*.md'))
    files += sorted(p for p in (REPO / '.claude' / 'skills').rglob('*') if p.is_file())
    return [p for p in files if p.is_file()]


def pins_in(text):
    """[(what, version, line number)] for every pin in `text`."""
    found = []
    for number, line in enumerate(text.splitlines(), 1):
        for what, pattern, _ in PINS:
            found += [(what, m.group(1), number) for m in re.finditer(pattern, line)]
    return found


class ThePatternsMatchWhatTheyDescribe(unittest.TestCase):
    def test_each_pattern_finds_its_pin_and_only_the_version(self):
        sample = '\n'.join([
            'git clone --depth 1 --branch v1.2.3 https://github.com/o/r.git',
            'https://github.com/o/r/blob/v1.2.3/docs/x.md',
            'SDK_VERSION=1.2.3',
            '`ros-humble-cognitive3d-ros-nav2_1.2.3-0jammy_arm64.deb`',
            'cognitive3d_ros 1.2.3 doctor (ROS 2 jazzy)',
            '[cognitive3d]: cognitive3d_ros 1.2.3 (ROS 2 jazzy, rmw) constructed.',
            'It targets SDK 1.2.3, and every link',
        ])
        found = pins_in(sample)
        self.assertEqual({what for what, _, _ in found}, {what for what, _, _ in PINS})
        self.assertEqual({version for _, version, _ in found}, {'1.2.3'})
        self.assertEqual(pins_in('tools/bump_version.sh 1.4.0\n--branch v<x.y.z>'), [])


class TheDocsPinTheDeclaredVersion(unittest.TestCase):
    def test_every_pin_names_the_declared_version(self):
        version = declared_version()
        wrong = []
        for path in scanned_files():
            for what, found, number in pins_in(path.read_text(errors='replace')):
                if found != version:
                    wrong.append(f'{path.relative_to(REPO)}:{number}: {what} names {found}')
        self.assertEqual(wrong, [], f'the tree declares {version}')

    def test_every_listed_pin_is_still_where_it_is_listed(self):
        for what, pattern, files in PINS:
            for name in files:
                with self.subTest(pin=what, file=name):
                    self.assertRegex((REPO / name).read_text(), pattern,
                                     f'{name} no longer carries {what}; if it moved, move it here')

    def test_the_newest_changelog_section_and_its_link_name_the_declared_version(self):
        version = declared_version()
        text = CHANGELOG.read_text()
        sections = re.findall(rf'^## \[{V}\]', text, flags=re.MULTILINE)
        self.assertTrue(sections, 'CHANGELOG.md has no version section')
        self.assertEqual(sections[0], version, 'the newest section of CHANGELOG.md')
        self.assertRegex(text, rf'(?m)^\[{re.escape(version)}\]: \S+/releases/tag/v'
                               rf'{re.escape(version)}$')


if __name__ == '__main__':
    unittest.main(verbosity=2)
