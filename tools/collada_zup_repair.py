#!/usr/bin/env python3
"""Inject the COLLADA `<asset>`/`<up_axis>` block that Gazebo's exporter omits.

Gazebo's `ColladaWorldExporter` writes a .dae with no `<asset>` element. COLLADA
1.4.1 requires it and its default for an unspecified up_axis is `Y_UP`, while
Gazebo is Z-up -- so every importer reads Z-up vertices as Y-up and the scene
arrives on its side, with no error from any stage.
Upstream: https://github.com/gazebosim/gz-common/issues/930

  ros2 run cognitive3d_ros collada_zup_repair.py <input.dae> <output.dae>

A file that already declares an up axis is copied unchanged.
"""
import pathlib
import sys

# `<asset>` belongs at the top of the document; no real exporter pushes the root
# element past this. Read as bytes: decoding with errors='replace' can change the
# length of the prefix and so the offset we splice at.
HEAD_BYTES = 8192
CHUNK = 1 << 20

ASSET_BLOCK = (b'\n    <asset>\n'
               b'        <unit meter="1.0" name="meter"/>\n'
               b'        <up_axis>Z_UP</up_axis>\n'
               b'    </asset>')


def _copy_rest(fin, fout):
    while (chunk := fin.read(CHUNK)):
        fout.write(chunk)


def repair(source, destination):
    """Write `source` to `destination` with an `<asset>` block if it lacks one.

    Returns a one-line description of what was done. Raises ValueError when the
    input does not look like a COLLADA document.
    """
    source, destination = pathlib.Path(source), pathlib.Path(destination)
    with source.open('rb') as fin:
        head = fin.read(HEAD_BYTES)

        if b'<up_axis>' in head:
            fin.seek(0)
            with destination.open('wb') as fout:
                _copy_rest(fin, fout)
            return 'up_axis already declared; copied unchanged'

        # `<asset>` must be the first child of `<COLLADA>`, so the anchor is that start tag. The
        # first `>` in the file is the same place only while nothing precedes `<COLLADA`: gz-common
        # drops the XML declaration too, and if it ever writes one, a first-`>` anchor would put
        # `<asset>` outside the root element.
        start = head.find(b'<COLLADA')
        if start < 0:
            raise ValueError(f'no <COLLADA> element in the first {HEAD_BYTES} bytes of {source}')
        close = head.find(b'>', start)
        if close < 0:
            raise ValueError(f'<COLLADA> start tag is not closed within the first '
                             f'{HEAD_BYTES} bytes of {source}')

        fin.seek(0)
        with destination.open('wb') as fout:
            fout.write(fin.read(close + 1) + ASSET_BLOCK)
            _copy_rest(fin, fout)
        return 'injected <up_axis>Z_UP</up_axis>'


def main(argv):
    if len(argv) != 3:
        print(f'usage: {argv[0]} <input.dae> <output.dae>', file=sys.stderr)
        return 2
    try:
        print(repair(argv[1], argv[2]))
    except ValueError as error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
