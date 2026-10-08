#!/usr/bin/env python3
"""Pin dependencies in a bloom-generated debian/control to exact versions, in place.

    pin_control.py <debian/control> <deb-name>=<version> [<deb-name>=<version>...]

Each named package in the binary paragraph's Depends becomes `<deb-name> (= <version>)`, replacing
any constraint it had. bloom cannot write this itself: `version_eq="1.0.0"` in a package.xml
becomes `(= 1.0.0)`, which no deb versioned `1.0.0-0noble` satisfies. Every named package must
appear in Depends exactly once and outside an alternative, or nothing is written and the exit is 1.
Build-Depends is left alone: it does not reach the built deb.

Standard library only. Prints the resulting Depends.
"""
from __future__ import annotations

import re
import sys

ENTRY = re.compile(r'(?P<name>[^\s(|]+)\s*(?:\([^)]*\))?')


def split_paragraphs(text: str) -> list[list[str]]:
    paragraphs: list[list[str]] = [[]]
    for line in text.splitlines():
        if line.strip() == '':
            if paragraphs[-1]:
                paragraphs.append([])
        else:
            paragraphs[-1].append(line)
    return [p for p in paragraphs if p]


def field_span(paragraph: list[str], name: str) -> tuple[int, int] | None:
    """(first, end) line indexes of field `name`, its continuation lines included."""
    for i, line in enumerate(paragraph):
        if line.split(':', 1)[0] == name and ':' in line:
            end = i + 1
            while end < len(paragraph) and paragraph[end][:1] in (' ', '\t'):
                end += 1
            return i, end
    return None


def pin(text: str, pins: dict[str, str]) -> tuple[str, str]:
    """(new control text, new Depends value). Raises ValueError naming what is wrong."""
    paragraphs = split_paragraphs(text)
    binaries = [p for p in paragraphs if field_span(p, 'Package') is not None]
    if len(binaries) != 1:
        raise ValueError(f'expected one binary paragraph, found {len(binaries)}')
    paragraph = binaries[0]
    span = field_span(paragraph, 'Depends')
    if span is None:
        raise ValueError('the binary paragraph has no Depends field')
    first, end = span
    value = ' '.join([paragraph[first].split(':', 1)[1]] + paragraph[first + 1:end])
    entries = [e.strip() for e in value.split(',') if e.strip()]
    found = {name: 0 for name in pins}
    for index, entry in enumerate(entries):
        for alternative in entry.split('|'):
            match = ENTRY.fullmatch(alternative.strip())
            name = match.group('name') if match else alternative.strip().split()[0]
            if name not in pins:
                continue
            if '|' in entry:
                raise ValueError(f'{name} is in an alternative: {entry!r}')
            if match is None:
                raise ValueError(f'{name} has a form this cannot pin: {entry!r}')
            found[name] += 1
            entries[index] = f'{name} (= {pins[name]})'
    wrong = {name: count for name, count in found.items() if count != 1}
    if wrong:
        raise ValueError('; '.join(f'{name} appears {count} time(s) in Depends'
                                   for name, count in sorted(wrong.items())))
    depends = ', '.join(entries)
    paragraph[first:end] = [f'Depends: {depends}']
    return '\n\n'.join('\n'.join(p) for p in paragraphs) + '\n', depends


def main(argv: list[str]) -> int:
    if len(argv) < 3 or not all('=' in a for a in argv[2:]):
        print(f'usage: {argv[0]} <debian/control> <deb-name>=<version>...', file=sys.stderr)
        return 2
    path = argv[1]
    pins = dict(a.split('=', 1) for a in argv[2:])
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    try:
        new_text, depends = pin(text, pins)
    except ValueError as error:
        print(f'{path}: {error}', file=sys.stderr)
        return 1
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(new_text)
    print(f'Depends: {depends}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
