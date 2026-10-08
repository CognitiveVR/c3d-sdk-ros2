"""The Quickstart prints the minimal params file the package installs, without its comments, and
the two must say the same thing: the copy is what a first run is written from.

Standard library only (repo rule). Run: python3 tools/test_quickstart_params.py
"""
import pathlib
import re
import unittest

REPO = pathlib.Path(__file__).resolve().parent.parent
MINIMAL = REPO / 'cognitive3d_ros' / 'config' / 'params.minimal.yaml'
QUICKSTART = REPO / 'docs' / 'quickstart.md'
INTRODUCTION = 'Without its comments, the copy reads:'


def without_comment(line):
    """`line` up to a `#` that starts a YAML comment: outside quotes, at the start or after a space."""
    quote = None
    for index, character in enumerate(line):
        if quote:
            if character == quote:
                quote = None
        elif character in '"\'':
            quote = character
        elif character == '#' and (index == 0 or line[index - 1] in ' \t'):
            return line[:index]
    return line


def uncommented(text):
    stripped = (without_comment(line).rstrip() for line in text.splitlines())
    return [line for line in stripped if line]


def quickstart_copy(text):
    _, found, after = text.partition(INTRODUCTION)
    match = re.match(r'\s*```yaml\n(.*?)\n```', after, re.S) if found else None
    return match.group(1).splitlines() if match else None


class TheQuickstartCopyIsTheShippedFile(unittest.TestCase):
    def test_the_copy_is_the_file_without_its_comments(self):
        copy = quickstart_copy(QUICKSTART.read_text())
        self.assertIsNotNone(copy, f'no yaml block after "{INTRODUCTION}" in {QUICKSTART.name}')
        self.assertEqual(copy, uncommented(MINIMAL.read_text()))

    def test_a_comment_is_cut_only_where_yaml_starts_one(self):
        self.assertEqual(uncommented('# heading\n  key: "a # b"   # why\n\n  url: x#y\n'),
                         ['  key: "a # b"', '  url: x#y'])


if __name__ == '__main__':
    unittest.main()
