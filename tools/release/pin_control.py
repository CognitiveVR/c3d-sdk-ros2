#!/usr/bin/env python3
"""Pin dependencies in a bloom-generated debian/control to exact versions, and add Recommends, in place.

    pin_control.py <debian/control> [--recommends <deb-name>]... [<deb-name>=<version>...]

Each named package in the binary paragraph's Depends becomes `<deb-name> (= <version>)`, replacing
any constraint it had. bloom cannot write this itself: `version_eq="1.0.0"` in a package.xml
becomes `(= 1.0.0)`, which no deb versioned `1.0.0-0noble` satisfies. Every named package must
appear in Depends exactly once and outside an alternative, or nothing is written and the exit is 1.
Build-Depends is left alone: it does not reach the built deb.

Each `--recommends` package is added to the binary paragraph's Recommends, which a package.xml
cannot declare. One that is also in Depends is refused, since it would not be optional.

Standard library only. Prints the resulting Depends and Recommends.
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


def recommend(text: str, names: list[str]) -> tuple[str, str]:
    """(new control text, new Recommends value). Raises ValueError naming what is wrong."""
    paragraphs = split_paragraphs(text)
    binaries = [p for p in paragraphs if field_span(p, 'Package') is not None]
    if len(binaries) != 1:
        raise ValueError(f'expected one binary paragraph, found {len(binaries)}')
    paragraph = binaries[0]
    depends_span = field_span(paragraph, 'Depends')
    depended = set()
    if depends_span is not None:
        first, end = depends_span
        value = ' '.join([paragraph[first].split(':', 1)[1]] + paragraph[first + 1:end])
        for entry in value.split(','):
            for alternative in entry.split('|'):
                match = ENTRY.fullmatch(alternative.strip())
                if match:
                    depended.add(match.group('name'))
    both = sorted(set(names) & depended)
    if both:
        raise ValueError(f'{", ".join(both)} is in Depends, so it cannot be only recommended')
    span = field_span(paragraph, 'Recommends')
    entries = []
    if span is not None:
        first, end = span
        value = ' '.join([paragraph[first].split(':', 1)[1]] + paragraph[first + 1:end])
        entries = [e.strip() for e in value.split(',') if e.strip()]
    for name in names:
        if name not in entries:
            entries.append(name)
    recommends = ', '.join(entries)
    if span is not None:
        paragraph[span[0]:span[1]] = [f'Recommends: {recommends}']
    else:
        at = depends_span[1] if depends_span is not None else len(paragraph)
        paragraph.insert(at, f'Recommends: {recommends}')
    return '\n\n'.join('\n'.join(p) for p in paragraphs) + '\n', recommends


def main(argv: list[str]) -> int:
    path = argv[1] if len(argv) > 1 else ''
    pins: dict[str, str] = {}
    recommends: list[str] = []
    rest = argv[2:]
    usable = bool(path) and bool(rest)
    while rest and usable:
        if rest[0] == '--recommends' and len(rest) > 1:
            recommends.append(rest[1])
            rest = rest[2:]
        elif '=' in rest[0] and not rest[0].startswith('-'):
            name, version = rest[0].split('=', 1)
            pins[name] = version
            rest = rest[1:]
        else:
            usable = False
    if not usable:
        print(f'usage: {argv[0]} <debian/control> [--recommends <deb-name>]... '
              '[<deb-name>=<version>...]', file=sys.stderr)
        return 2
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    try:
        depends = None
        if pins:
            text, depends = pin(text, pins)
        recommended = None
        if recommends:
            text, recommended = recommend(text, recommends)
    except ValueError as error:
        print(f'{path}: {error}', file=sys.stderr)
        return 1
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(text)
    if depends is not None:
        print(f'Depends: {depends}')
    if recommended is not None:
        print(f'Recommends: {recommended}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
