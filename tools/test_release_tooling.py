"""The release scripts that can be checked without Docker.

- `tools/release/changelog_section.sh` prints exactly one version's notes, matches the version as a
  literal rather than as a pattern, and refuses a missing or empty section. With `--release` it
  also refuses an undated heading and an HTML comment, and makes every relative link absolute.
- `tools/release/pin_control.py` pins exactly the named packages in a debian/control's Depends.
- `tools/release/build_debs.sh` and `smoke_test.sh` refuse a distro they do not know before they
  start a container.
- `tools/release/rosdep.yaml` has a key, for every distro the release builds, for each of this
  repository's packages that another of them depends on. Without one bloom cannot generate the
  dependent package, and that is found only in a release build.

Standard library only (repo rule). Run: python3 tools/test_release_tooling.py
"""
import importlib.util
import pathlib
import re
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

REPO = pathlib.Path(__file__).resolve().parent.parent
RELEASE = REPO / 'tools' / 'release'
SECTION = RELEASE / 'changelog_section.sh'
ROSDEP_YAML = RELEASE / 'rosdep.yaml'
VERSION_HPP = REPO / 'core' / 'include' / 'c3d' / 'version.hpp'
REPO_URL = 'https://github.com/CognitiveVR/c3d-sdk-ros2'
# The distros the release builds, and the Ubuntu each one is released for.
DISTROS = {'humble': 'jammy', 'jazzy': 'noble'}

CHANGELOG = """\
# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

- Nothing yet.

## [11.1.0] - 2030-01-01

- The wrong section: 1.1.0 read as a pattern selects this one.

## [1.3.0] - 2027-10-15

### Changed

- First line.

- Second line, after a blank one.

## [1.1.0.1] - 2027-01-02

- Also the wrong section.

## [1.1.0] - 2027-01-01

- The right section.

## [1.0.1] - 2026-12-01

## [1.0.0]

- No date on this heading.

[Unreleased]: https://example.invalid/compare/v1.3.0...HEAD
[1.3.0]: https://example.invalid/releases/tag/v1.3.0
"""


def section(version, text=CHANGELOG, release_url=None, allow_undated=False):
    with tempfile.NamedTemporaryFile('w', suffix='.md', delete=False) as handle:
        handle.write(text)
    flags = [] if release_url is None else ['--release', release_url]
    flags += ['--allow-undated'] if allow_undated else []
    try:
        return subprocess.run(['bash', str(SECTION), *flags, version, handle.name],
                              capture_output=True, text=True)
    finally:
        pathlib.Path(handle.name).unlink()


def declared_version():
    match = re.search(r'kSdkVersion = "([^"]+)"', VERSION_HPP.read_text())
    assert match, f'no kSdkVersion in {VERSION_HPP}'
    return match.group(1)


class ChangelogSection(unittest.TestCase):
    def test_prints_the_body_of_the_section_and_nothing_around_it(self):
        result = section('1.3.0')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout,
                         '### Changed\n\n- First line.\n\n- Second line, after a blank one.\n')

    def test_the_version_is_a_literal_and_not_a_pattern(self):
        result = section('1.1.0')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, '- The right section.\n')

    def test_the_last_section_stops_at_the_link_definitions(self):
        result = section('1.0.0')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, '- No date on this heading.\n')

    def test_a_version_with_no_section_is_an_error(self):
        result = section('1.2.0')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')
        self.assertIn('no section for 1.2.0', result.stderr)

    def test_an_empty_section_is_an_error(self):
        result = section('1.0.1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('empty', result.stderr)

    def test_a_heading_that_only_starts_with_the_version_is_not_it(self):
        result = section('1.3.0', CHANGELOG.replace('## [1.3.0] - 2027-10-15',
                                                    '## [1.3.0]x'))
        self.assertNotEqual(result.returncode, 0)

    def test_a_missing_changelog_is_an_error(self):
        result = subprocess.run(['bash', str(SECTION), '1.3.0', '/nonexistent/CHANGELOG.md'],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)


RELEASE_CHANGELOG = """\
# Changelog

## [Unreleased]

## [1.2.0] - 2027-03-15

- See [Install](docs/install.md), [poses](./docs/coordinates.md#planar-poses) and
  [the root](/README.md), [above](#120---2027-03-15), [the site](https://example.invalid/x)
  and [mail](mailto:support@example.invalid). ![a figure](docs/figure.png)
- Code keeps its text: `[x](docs/a.md)`, and so does a fence:

```text
[y](docs/b.md)
```

- After the fence: [Sessions](docs/sessions.md).

## [1.1.0] - 2027-01-10

- Older.
"""


class ChangelogSectionForARelease(unittest.TestCase):
    def test_relative_links_point_at_the_tagged_tree_and_absolute_ones_are_kept(self):
        result = section('1.2.0', RELEASE_CHANGELOG, REPO_URL)
        self.assertEqual(result.returncode, 0, result.stderr)
        blob = f'{REPO_URL}/blob/v1.2.0'
        self.assertEqual(result.stdout, (
            f'- See [Install]({blob}/docs/install.md), '
            f'[poses]({blob}/docs/coordinates.md#planar-poses) and\n'
            f'  [the root]({blob}/README.md), [above]({blob}/CHANGELOG.md#120---2027-03-15), '
            '[the site](https://example.invalid/x)\n'
            f'  and [mail](mailto:support@example.invalid). ![a figure]({blob}/docs/figure.png)\n'
            '- Code keeps its text: `[x](docs/a.md)`, and so does a fence:\n'
            '\n'
            '```text\n'
            '[y](docs/b.md)\n'
            '```\n'
            '\n'
            f'- After the fence: [Sessions]({blob}/docs/sessions.md).\n'))

    def test_without_release_the_links_are_left_relative(self):
        result = section('1.2.0', RELEASE_CHANGELOG)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('[Install](docs/install.md)', result.stdout)

    def test_a_trailing_slash_on_the_repository_url_is_dropped(self):
        result = section('1.2.0', RELEASE_CHANGELOG, REPO_URL + '/')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f'[Install]({REPO_URL}/blob/v1.2.0/docs/install.md)', result.stdout)

    def test_an_undated_heading_is_refused(self):
        for heading in ('## [1.2.0] - Unreleased', '## [1.2.0]', '## [1.2.0] - 2027-03',
                        '## [1.2.0] - 2027-03-15 (preview)'):
            with self.subTest(heading=heading):
                text = RELEASE_CHANGELOG.replace('## [1.2.0] - 2027-03-15', heading)
                result = section('1.2.0', text, REPO_URL)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(result.stdout, '')
                self.assertIn('not dated', result.stderr)
                self.assertEqual(section('1.2.0', text).returncode, 0, 'only --release refuses')

    def test_a_dry_run_accepts_an_undated_heading_and_a_tag_does_not(self):
        text = RELEASE_CHANGELOG.replace('## [1.2.0] - 2027-03-15', '## [1.2.0] - Unreleased')
        tag = section('1.2.0', text, REPO_URL)
        self.assertNotEqual(tag.returncode, 0)
        self.assertIn('not dated', tag.stderr)
        dry_run = section('1.2.0', text, REPO_URL, allow_undated=True)
        self.assertEqual(dry_run.returncode, 0, dry_run.stderr)
        self.assertEqual(dry_run.stdout, section('1.2.0', RELEASE_CHANGELOG, REPO_URL).stdout)

    def test_a_dry_run_still_refuses_a_comment_and_a_link_out(self):
        for old, new, says in (('- After the fence:', '<!-- note -->\n- After the fence:',
                                'HTML comment'),
                               ('(docs/install.md)', '(../elsewhere.md)', '../elsewhere.md')):
            with self.subTest(says=says):
                text = RELEASE_CHANGELOG.replace(old, new)
                result = section('1.2.0', text, REPO_URL, allow_undated=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(says, result.stderr)

    def test_allow_undated_is_only_an_option_of_release(self):
        result = subprocess.run(['bash', str(SECTION), '--allow-undated', '1.2.0'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('usage', result.stderr)

    def test_an_html_comment_is_refused(self):
        text = RELEASE_CHANGELOG.replace(
            '- After the fence:', '<!-- MERGE THE FRAGMENT HERE -->\n- After the fence:')
        result = section('1.2.0', text, REPO_URL)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')
        self.assertIn('HTML comment', result.stderr)
        self.assertIn('MERGE THE FRAGMENT HERE', result.stderr)

    def test_a_comment_in_another_section_does_not_matter(self):
        text = RELEASE_CHANGELOG.replace('- Older.', '- Older. <!-- note -->')
        self.assertEqual(section('1.2.0', text, REPO_URL).returncode, 0)

    def test_a_link_out_of_the_repository_is_refused(self):
        text = RELEASE_CHANGELOG.replace('(docs/install.md)', '(../elsewhere.md)')
        result = section('1.2.0', text, REPO_URL)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('../elsewhere.md', result.stderr)

    def test_the_release_url_must_be_https(self):
        for url in ('http://github.com/x/y', 'github.com/x/y', 'https://'):
            with self.subTest(url=url):
                result = section('1.2.0', RELEASE_CHANGELOG, url)
                self.assertEqual(result.returncode, 2, result.stderr)


class TheRepositorysChangelog(unittest.TestCase):
    def test_it_has_notes_for_the_declared_version_with_no_html_comment(self):
        version = declared_version()
        result = subprocess.run(['bash', str(SECTION), version], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('<!--', result.stdout)


def load_pin_control():
    spec = importlib.util.spec_from_file_location('pin_control', RELEASE / 'pin_control.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CONTROL = """\
Source: ros-jazzy-cognitive3d-ros-nav2
Maintainer: "Cognitive3D" <support@cognitive3d.com>
Build-Depends: debhelper (>= 9.0.0), ros-jazzy-cognitive3d-ros, ros-jazzy-nav2-msgs
Standards-Version: 3.9.2

Package: ros-jazzy-cognitive3d-ros-nav2
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}, ros-jazzy-cognitive3d-ros-msgs,
 ros-jazzy-cognitive3d-ros (= 1.2.0), ros-jazzy-nav2-msgs
Description: Nav2 translators for cognitive3d_ros.
 A second description line.
"""


class PinControl(unittest.TestCase):
    pin = staticmethod(load_pin_control().pin)

    def test_only_the_named_package_is_pinned_and_a_bare_version_is_replaced(self):
        text, depends = self.pin(CONTROL, {'ros-jazzy-cognitive3d-ros': '1.2.0-0noble'})
        self.assertEqual(depends, '${shlibs:Depends}, ${misc:Depends}, '
                                  'ros-jazzy-cognitive3d-ros-msgs, '
                                  'ros-jazzy-cognitive3d-ros (= 1.2.0-0noble), ros-jazzy-nav2-msgs')
        self.assertIn('Build-Depends: debhelper (>= 9.0.0), ros-jazzy-cognitive3d-ros, '
                      'ros-jazzy-nav2-msgs\n', text)
        self.assertIn(f'Depends: {depends}\nDescription: Nav2 translators for cognitive3d_ros.\n'
                      ' A second description line.\n', text)
        self.assertTrue(text.startswith('Source: ros-jazzy-cognitive3d-ros-nav2\n'))
        self.assertIn('Standards-Version: 3.9.2\n\nPackage:', text)

    def test_several_packages_are_pinned_at_once(self):
        _, depends = self.pin(CONTROL, {'ros-jazzy-cognitive3d-ros': '1-0noble',
                                        'ros-jazzy-cognitive3d-ros-msgs': '2-0noble'})
        self.assertIn('ros-jazzy-cognitive3d-ros-msgs (= 2-0noble)', depends)
        self.assertIn('ros-jazzy-cognitive3d-ros (= 1-0noble)', depends)

    def test_a_package_missing_from_depends_is_an_error(self):
        with self.assertRaisesRegex(ValueError, 'ros-jazzy-other appears 0 time'):
            self.pin(CONTROL, {'ros-jazzy-other': '1'})

    def test_a_package_in_an_alternative_is_an_error(self):
        text = CONTROL.replace('ros-jazzy-nav2-msgs\nDescription',
                               'ros-jazzy-nav2-msgs | ros-jazzy-cognitive3d-ros\nDescription')
        with self.assertRaisesRegex(ValueError, 'alternative'):
            self.pin(text, {'ros-jazzy-cognitive3d-ros': '1'})

    def test_the_command_line_writes_the_file_and_refuses_without_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            control = pathlib.Path(directory) / 'control'
            control.write_text(CONTROL)
            script = str(RELEASE / 'pin_control.py')
            ok = subprocess.run(['python3', script, str(control),
                                 'ros-jazzy-cognitive3d-ros=1.2.0-0noble'],
                                capture_output=True, text=True)
            self.assertEqual(ok.returncode, 0, ok.stderr)
            self.assertIn('ros-jazzy-cognitive3d-ros (= 1.2.0-0noble)', control.read_text())
            before = control.read_text()
            refused = subprocess.run(['python3', script, str(control), 'ros-jazzy-absent=1'],
                                     capture_output=True, text=True)
            self.assertEqual(refused.returncode, 1)
            self.assertEqual(control.read_text(), before)


class ContainerScriptsRefuseBadArguments(unittest.TestCase):
    def test_an_unknown_distro_or_missing_directory_is_a_usage_error(self):
        for script in ('build_debs.sh', 'smoke_test.sh'):
            for args in (['rolling', '/tmp'], ['humble'], [], ['jazzy', '/tmp', 'extra']):
                with self.subTest(script=script, args=args):
                    result = subprocess.run(['bash', str(RELEASE / script), *args],
                                            capture_output=True, text=True)
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertIn('usage', result.stderr)


def parse_rosdep_yaml(text):
    """{key: {codename: [packages]}} for the one shape this file uses, refusing anything else."""
    keys = {}
    key = None
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.split('#', 1)[0].rstrip()
        if not line:
            continue
        if not line.startswith(' '):
            if not line.endswith(':'):
                raise ValueError(f'line {lineno}: expected "<key>:"')
            key = line[:-1]
            keys[key] = {}
        elif line == '  ubuntu:':
            if key is None:
                raise ValueError(f'line {lineno}: ubuntu outside a key')
        elif line.startswith('    ') and ': [' in line and line.endswith(']'):
            codename, _, packages = line.strip().partition(': ')
            keys[key][codename] = [p.strip() for p in packages[1:-1].split(',') if p.strip()]
        else:
            raise ValueError(f'line {lineno}: not in the shape this test reads: {raw!r}')
    return keys


def repo_packages():
    """{package name: set of the names it depends on}, from every package.xml in the repo."""
    packages = {}
    for manifest in sorted(REPO.glob('*/package.xml')):
        root = ET.fromstring(manifest.read_text())
        deps = {e.text.strip() for e in root if e.tag.endswith('depend') and e.text}
        packages[root.findtext('name').strip()] = deps
    return packages


class RosdepKeysForOurOwnPackages(unittest.TestCase):
    def test_the_parser_reads_the_shape_and_refuses_others(self):
        self.assertEqual(parse_rosdep_yaml('a:\n  ubuntu:\n    jammy: [x, y]\n'),
                         {'a': {'jammy': ['x', 'y']}})
        with self.assertRaises(ValueError):
            parse_rosdep_yaml('a:\n  debian: [x]\n')

    def test_every_package_another_one_depends_on_has_a_key_for_every_distro(self):
        packages = repo_packages()
        self.assertGreaterEqual(len(packages), 3, f'found only {sorted(packages)}')
        depended_on = sorted({d for deps in packages.values() for d in deps if d in packages})
        self.assertIn('cognitive3d_ros_msgs', depended_on, 'the dependency scan found nothing')
        keys = parse_rosdep_yaml(ROSDEP_YAML.read_text())
        for name in depended_on:
            for distro, codename in DISTROS.items():
                with self.subTest(package=name, distro=distro):
                    expected = f'ros-{distro}-{name.replace("_", "-")}'
                    self.assertEqual(keys.get(name, {}).get(codename), [expected],
                                     f'{ROSDEP_YAML.name} must map {name} on {codename} to '
                                     f'{expected}')


if __name__ == '__main__':
    unittest.main()
