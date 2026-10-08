"""What a shipped tool says about itself is written for the person running it.

The tools `cognitive3d_ros` installs are read through `--help`, their usage message and their
module docstring or header comment by someone who has the deb and nothing else. A source path in
another SDK, a C++ test name, a test file, a ticket or pull request number, a host they cannot
reach, a path or doc that does not ship, or a date reads to them as notes left
behind. The tools are FOUND in the install rule, not listed, so a tool added there is checked
without anyone remembering to.

Standard library only (repo rule). Run: python3 tools/test_tool_help.py
"""
import ast
import pathlib
import re
import subprocess
import sys
import tempfile
import unittest

REPO = pathlib.Path(__file__).resolve().parent.parent
CMAKE = REPO / 'cognitive3d_ros' / 'CMakeLists.txt'

MONTHS = ('January|February|March|April|May|June|July|August|September|October|November|'
          'December')

INTERNAL = {
    'a source path in another SDK': re.compile(r'\.cs\b|cvr-sdk'),
    "another of the organisation's repositories": re.compile(r'\bcvr-[a-z][a-z-]*', re.I),
    'a C++ source file': re.compile(r'\b\w+\.(?:cpp|hpp)\b'),
    'a test file, which does not ship': re.compile(r'\btest_[a-z0-9_]+\b'),
    'a date': re.compile(rf'\b20\d\d-\d\d-\d\d\b|\b(?:{MONTHS}) 20\d\d\b'),
    'a pull request': re.compile(r'\bPR\s*#?\d+\b|(?<![\w&])#\d+\b'),
}

# A ticket id is a project key and a number. These keys are standards a reader can look up.
TICKET = re.compile(r'\b([A-Z][A-Z0-9]{1,9})-\d+\b')
STANDARDS = {'AES', 'IEC', 'ISO', 'PEP', 'REP', 'RFC', 'SHA', 'UTF'}

# The hosts a reader can reach. An environment variable names no host; its default still does.
PUBLIC_HOSTS = {'cognitive3d.com', 'api.cognitive3d.com', 'data.cognitive3d.com',
                'docs.cognitive3d.com', 'upload.cognitive3d.com', 'github.com', 'localhost'}
URL_HOST = re.compile(r'\b[a-z][a-z0-9+.-]*://([\w.-]+)', re.I)
DOMAIN = re.compile(r'(?<![\w./])[a-z0-9][a-z0-9-]*(?:\.[a-z0-9][a-z0-9-]*)*'
                    r'\.(?:com|net|org|io|dev|ai|cloud|internal|local|lan|corp)'
                    r'(?![\w-]|\.\w)', re.I)

# The top-level directories a release ships. A path under any other is one its reader cannot open.
SHIPPED_ROOTS = {'.claude', '.github', 'cognitive3d_ros', 'cognitive3d_ros_msgs',
                 'cognitive3d_ros_nav2', 'core', 'docker', 'docs', 'tools'}
RELATIVE_PATH = re.compile(r'(?<![\w./:~$-])[\w.-]+(?:/[\w.-]+)+')

# The Markdown files a release ships, and all it ships under `docs/`. Any other `.md` a tool names
# is one its reader cannot open.
SHIPPED_DOCS = {
    'README.md', 'CHANGELOG.md', 'CONTRIBUTING.md', 'SECURITY.md', 'THIRD_PARTY_NOTICES.md',
    'install.md', 'quickstart.md', 'scenes.md', 'configuration.md', 'sessions.md',
    'coordinates.md', 'recording.md', 'camera-and-lidar.md', 'remote-variables.md',
    'doctor-and-verification.md', 'troubleshooting.md', 'data-format.md',
}
MARKDOWN = re.compile(r'[\w./-]*\.md\b')

# A test name reads as a sentence: five or more CamelCase words, or four with a function word.
# Fewer, or four without one, is how a message or class is named (`BehaviorTreeStatusChange`).
CAMEL = re.compile(r'\b[A-Z][A-Za-z]*[a-z][A-Za-z]*\b')
CAMEL_WORD = re.compile(r'[A-Z]+(?![a-z])|[A-Z][a-z]+')
FUNCTION_WORDS = {'A', 'An', 'The', 'Is', 'Are', 'Of', 'To', 'When', 'Without', 'With', 'And',
                  'Or', 'Not', 'No', 'Every', 'Each', 'Its', 'It', 'As', 'By', 'From', 'Into',
                  'Than', 'Before', 'After', 'Only', 'Never', 'Rather'}


def test_names(text):
    for match in CAMEL.finditer(text):
        words = CAMEL_WORD.findall(match.group(0))
        if len(words) >= 5 or (len(words) == 4 and FUNCTION_WORDS & set(words)):
            yield match.group(0)


def hosts(text):
    named = [match.group(1) for match in URL_HOST.finditer(text)]
    named += [match.group(0) for match in DOMAIN.finditer(text)]
    return dict.fromkeys(host.rstrip('.').lower() for host in named)


def unshipped_paths(text, repo):
    """Relative paths into the repository that name nothing a release ships."""
    roots = SHIPPED_ROOTS | {entry.name for entry in repo.iterdir()}
    for match in RELATIVE_PATH.finditer(text):
        path = match.group(0).rstrip('.')
        root, _, rest = path.partition('/')
        if root in roots and (root not in SHIPPED_ROOTS or not (repo / path).exists()
                              or root == 'docs' and rest not in SHIPPED_DOCS):
            yield path


def internal_notes(text, repo=REPO):
    notes = [f'{what}: {match.group(0)!r}'
             for what, pattern in INTERNAL.items() for match in pattern.finditer(text)]
    notes += [f'a ticket id: {match.group(0)!r}' for match in TICKET.finditer(text)
              if match.group(1) not in STANDARDS]
    notes += [f'a host its reader cannot reach: {host!r}' for host in hosts(text)
              if host not in PUBLIC_HOSTS]
    notes += [f'a C++ test name: {name!r}' for name in test_names(text)]
    notes += [f'a path that does not ship: {path!r}' for path in unshipped_paths(text, repo)]
    notes += [f'a doc that does not ship: {match.group(0)!r}' for match in MARKDOWN.finditer(text)
              if pathlib.PurePosixPath(match.group(0)).name not in SHIPPED_DOCS]
    return notes


def shipped_tools():
    """The files the package's `install(PROGRAMS ...)` rule installs."""
    rule = re.search(r'install\(PROGRAMS(.*?)DESTINATION', CMAKE.read_text(), re.S)
    names = re.findall(r'\$\{C3D_TOOLS_DIR\}/([\w.]+)', rule.group(1))
    return [REPO / 'tools' / name for name in names]


def header_comment(script):
    """A shell script's leading comment block, after the shebang."""
    lines = script.read_text().splitlines()[1:]
    block = []
    for line in lines:
        if not line.startswith('#'):
            break
        block.append(line.lstrip('#'))
    return '\n'.join(block)


def run(tool, *arguments):
    command = [sys.executable, str(tool)] if tool.suffix == '.py' else ['bash', str(tool)]
    return subprocess.run([*command, *arguments], cwd=tool.parent, capture_output=True,
                          text=True, timeout=60)


class ShippedToolHelp(unittest.TestCase):
    def test_the_install_rule_names_tools_to_check(self):
        tools = shipped_tools()
        self.assertGreaterEqual(len([t for t in tools if t.suffix == '.py']), 5)
        self.assertIn(REPO / 'tools' / 'export_world_gltf.sh', tools)
        self.assertEqual([t for t in tools if t.suffix not in ('.py', '.sh')], [])

    def test_no_module_docstring_or_header_comment_carries_internal_notes(self):
        for tool in shipped_tools():
            with self.subTest(tool=tool.name):
                if tool.suffix == '.py':
                    text = ast.get_docstring(ast.parse(tool.read_text())) or ''
                else:
                    text = header_comment(tool)
                self.assertTrue(text.strip(), 'a shipped tool says what it is for')
                self.assertEqual(internal_notes(text), [])

    def test_no_help_output_carries_internal_notes(self):
        for tool in shipped_tools():
            with self.subTest(tool=tool.name):
                result = run(tool, '--help')
                output = result.stdout + result.stderr
                self.assertIn('usage', output.lower())
                self.assertEqual(internal_notes(output), [])

    def test_a_shell_tool_run_wrongly_prints_its_usage_and_not_a_shell_error(self):
        for tool in shipped_tools():
            if tool.suffix != '.sh':
                continue
            with self.subTest(tool=tool.name):
                for arguments in ((), ('only-a-world',)):
                    result = run(tool, *arguments)
                    output = result.stdout + result.stderr
                    self.assertEqual(result.returncode, 2, output)
                    self.assertTrue(output.startswith('usage: '), output)
                    self.assertNotRegex(output, r'line \d+:')
                    self.assertEqual(internal_notes(output), [])


class ThePatternsFindWhatTheyName(unittest.TestCase):
    def test_each_banned_item_is_found(self):
        for sample in ('Runtime/Scripts/ExampleGaze.cs:12', 'cvr-sdk-example', 'cvr-example',
                       'core/src/environment.cpp', 'tools/test_gltf_bbox.py', 'test_gltf_bbox',
                       'AnOpticalRayPointsWhereTheCameraActuallyLooks', 'TheVersionIsCompared',
                       'SceneProject.AVersionTheSceneDoesNotHave', 'measured 2026-09-11',
                       'measured in September 2026', 'see ABC-123', 'tracked as X9-7',
                       'merged as PR 12', 'merged as PR#12', 'fixed in #12',
                       'api.example.internal', 'https://DATA.Staging.Example.com/v0',
                       'http://ingest-worker:8080/v1', 'notarealhost.cognitive3d.com',
                       '${C3D_API_HOST:-api.example.internal}', 'cognitive3d.com.example.net',
                       'see docs/NOT_A_REAL_FILE.md',
                       'NOT_A_REAL_FILE.md', 'docs/NOT_A_REAL_FILE.md#step-3',
                       'https://github.com/o/r/blob/main/docs/NOT_A_REAL_FILE.md',
                       'tools/not_a_real_tool.sh', 'see docs/not-a-real-dir/x.',
                       'cognitive3d_ros/config/not_a_real.yaml'):
            with self.subTest(sample=sample):
                self.assertNotEqual(internal_notes(sample), [])

    def test_ordinary_words_and_shipped_docs_are_not(self):
        for sample in ('ColladaWorldExporter writes a .dae; see gltf_bbox.py',
                       'BehaviorTreeStatusChange', 'ROSIDLTypeSupport',
                       'see docs/scenes.md and README.md',
                       'UTF-8, SHA-256 and REP-105', 'May the scene', 'version 2026',
                       'contest_results', 'https://api.cognitive3d.com/v0/',
                       'upload.cognitive3d.com.', '$C3D_API_HOST',
                       '${C3D_DATA_HOST:-data.cognitive3d.com}', 'data.<your-stack>',
                       'https://github.com/gazebosim/gz-common/issues/930', 'c3d.app.version',
                       'scene.gltf and scene.bin', 'params.minimal.yaml', 'docker/Dockerfile.test',
                       'copy params.yaml.example', 'see tools/gltf_bbox.py.', 'c3d_api/env.py',
                       'cognitive3d_ros/config/params.minimal.yaml', '/tmp/out',
                       '~/.ssh/config.local', '/path/to/my_world.sdf', '$HOME/out/scene.gltf',
                       'and/or', 'meters/second',
                       '$(ros2 pkg prefix cognitive3d_ros)/share/cognitive3d_ros/config/x.yaml'):
            with self.subTest(sample=sample):
                self.assertEqual(internal_notes(sample), [])

    def test_a_path_must_exist_under_a_root_the_release_ships(self):
        with tempfile.TemporaryDirectory() as directory:
            repo = pathlib.Path(directory)
            for path in ('tools/real_tool.py', 'docs/scenes.md', 'docs/extra/notes.txt',
                         'example_root/real_script.sh'):
                (repo / path).parent.mkdir(parents=True, exist_ok=True)
                (repo / path).touch()
            self.assertEqual(internal_notes('see tools/real_tool.py and docs/scenes.md', repo), [])
            for sample in ('see example_root/real_script.sh', 'tools/missing.py', 'docs/missing',
                           'docs/extra', 'docs/extra/notes.txt'):
                with self.subTest(sample=sample):
                    self.assertNotEqual(internal_notes(sample, repo), [])


if __name__ == '__main__':
    unittest.main()
