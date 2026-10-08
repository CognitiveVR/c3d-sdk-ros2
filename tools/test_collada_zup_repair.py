"""Unit tests for tools/collada_zup_repair.py.

The property under test is that the `<asset>` block lands as the FIRST CHILD OF
`<COLLADA>` and the result is well-formed XML -- including when an XML
declaration is present. That case is what separates anchoring on `<COLLADA`
from anchoring on the first `>` in the file: today's Gazebo exporter emits no
declaration, so both anchors agree, and a test that only feeds it today's output
cannot tell a correct implementation from an accidentally-correct one.

Run: uv run tools/test_collada_zup_repair.py
"""
import importlib.util
import pathlib
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree

_SPEC = importlib.util.spec_from_file_location(
    'collada_zup_repair', pathlib.Path(__file__).parent / 'collada_zup_repair.py')
repair_module = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(repair_module)

NAMESPACE = 'http://www.collada.org/2005/11/COLLADASchema'
BODY = (f'<COLLADA xmlns="{NAMESPACE}" version="1.4.1">'
        '<library_geometries/>'
        '<scene/>'
        '</COLLADA>')
DECLARATION = '<?xml version="1.0" encoding="utf-8"?>\n'


def local(tag):
    return tag.rsplit('}', 1)[-1]


def parse(path):
    """Parse, having first refused a DTD.

    `xml.etree.ElementTree` does NOT resolve external entities -- measured on this interpreter, a
    `SYSTEM` entity raises `ParseError: undefined entity` rather than reading the file -- so the
    XXE that static analysis reports against this call does not exist here. What DOES exist is
    internal entity expansion: a nested-entity document expands (billion laughs / quadratic
    blowup). Entity expansion requires a DTD, so refusing one closes the real class rather than
    arguing about the reported one.

    It also strengthens what these tests assert: the repair must never introduce a `<!DOCTYPE`,
    because that would change how every downstream consumer resolves entities in a file we
    rewrote.
    """
    raw = pathlib.Path(path).read_bytes()
    assert b'<!DOCTYPE' not in raw, f'{path} declares a DTD; these fixtures never should'
    return ElementTree.parse(path)


class ColladaZupRepair(unittest.TestCase):
    def setUp(self):
        self.directory = pathlib.Path(tempfile.mkdtemp())

    def write(self, text):
        source = self.directory / 'in.dae'
        source.write_text(text)
        return source

    def repair(self, text):
        source = self.write(text)
        destination = self.directory / 'out.dae'
        note = repair_module.repair(source, destination)
        return note, destination

    def assertAssetIsFirstChildOfCollada(self, destination):
        root = parse(destination).getroot()
        self.assertEqual(local(root.tag), 'COLLADA',
                         'the repaired document must still be rooted at <COLLADA>')
        children = [local(child.tag) for child in root]
        self.assertEqual(children[0], 'asset',
                         f'<asset> must be the first child of <COLLADA>, got {children}')
        up_axis = root.find(f'{{{NAMESPACE}}}asset/{{{NAMESPACE}}}up_axis')
        self.assertIsNotNone(up_axis)
        self.assertEqual(up_axis.text, 'Z_UP')

    def test_injects_into_todays_exporter_output_which_has_no_declaration(self):
        note, destination = self.repair(BODY)
        self.assertIn('injected', note)
        self.assertAssetIsFirstChildOfCollada(destination)

    def test_injects_correctly_when_an_xml_declaration_is_present(self):
        # The discriminating case. A first-'>' anchor would splice <asset>
        # between the declaration and <COLLADA>, which is not well-formed.
        note, destination = self.repair(DECLARATION + BODY)
        self.assertIn('injected', note)
        self.assertAssetIsFirstChildOfCollada(destination)

    def test_a_first_angle_bracket_anchor_would_fail_that_case(self):
        # Pins WHY the anchor is what it is: the same splice at the first '>'
        # produces a document ElementTree refuses. If this ever stops raising,
        # the reasoning beside the anchor in repair() no longer holds.
        text = DECLARATION + BODY
        first_close = text.index('>') + 1
        spliced = text[:first_close] + repair_module.ASSET_BLOCK.decode() + text[first_close:]
        bad = self.directory / 'bad.dae'
        bad.write_text(spliced)
        with self.assertRaises(ElementTree.ParseError):
            parse(bad)

    def test_already_declared_is_copied_byte_for_byte(self):
        declared = BODY.replace('<library_geometries/>',
                                '<asset><up_axis>Z_UP</up_axis></asset><library_geometries/>')
        note, destination = self.repair(declared)
        self.assertIn('already declared', note)
        self.assertEqual(destination.read_bytes(), declared.encode())

    def test_a_non_collada_input_is_refused(self):
        with self.assertRaises(ValueError):
            self.repair('<html><body>not a mesh</body></html>')

    def test_an_unterminated_collada_tag_is_refused(self):
        with self.assertRaises(ValueError):
            self.repair('<COLLADA ' + 'x' * (repair_module.HEAD_BYTES + 10))

    def test_content_past_the_head_survives_streaming(self):
        filler = '<node/>' * 40000            # comfortably past HEAD_BYTES
        text = BODY.replace('<scene/>', filler + '<scene/>')
        note, destination = self.repair(text)
        self.assertIn('injected', note)
        self.assertEqual(destination.read_text().count('<node/>'), 40000)
        self.assertAssetIsFirstChildOfCollada(destination)


if __name__ == '__main__':
    unittest.main(verbosity=2)
