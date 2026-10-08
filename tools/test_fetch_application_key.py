"""Unit tests for tools/fetch_application_key.py -- no network, no real credentials.

The properties under test are the decisions the tool makes on its own: which key it picks when a
project has several; that a masked listing prints a key exactly as doctor does, and never enough
of it to be used; and that a response of an unexpected shape is described, never echoed. Every key
here is a fixed non-secret placeholder.

Run: uv run tools/test_fetch_application_key.py
"""
import contextlib
import importlib.util
import io
import pathlib
import re
import sys
import types
import unittest
from unittest import mock

TOOLS = pathlib.Path(__file__).parent
sys.path.insert(0, str(TOOLS))          # so `from c3d_api import ...` resolves
_SPEC = importlib.util.spec_from_file_location('fetch_application_key',
                                               TOOLS / 'fetch_application_key.py')
tool = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(tool)


def record(identifier, key, valid=True, created=0):
    return {'id': identifier, 'apiKey': key, 'valid': valid, 'createdAt': created}


class Mask(unittest.TestCase):
    """doctor's `application key` line and this listing name a key the same way, so a person
    comparing the two sees the same text. The strings are doctor's MaskSecret's."""

    def test_a_long_key_shows_only_its_last_four(self):
        self.assertEqual(tool.mask('ABCD' + 'X' * 24 + 'WXYZ'), 'ending ...WXYZ')
        self.assertEqual(tool.mask('ABCD' + 'X' * 8 + 'WXYZ'), 'ending ...WXYZ')

    def test_a_short_key_shows_length_only(self):
        # Four characters of a short value are much of the value, so it must not be shown at all.
        self.assertEqual(tool.mask('ABCD' + 'X' * 7 + 'WXYZ'),
                         '<15 characters, too short to mask safely>')
        self.assertEqual(tool.mask('SHORTKEY'), '<8 characters, too short to mask safely>')
        self.assertEqual(tool.mask(''), '<empty>')

    def test_the_short_key_threshold_is_doctors(self):
        doctor = TOOLS.parent / 'cognitive3d_ros' / 'src' / 'doctor.cpp'
        if not doctor.exists():
            self.skipTest('doctor.cpp is not beside this copy of tools/')
        found = re.search(r'kShortestMaskable = (\d+);', doctor.read_text())
        self.assertIsNotNone(found, 'doctor.cpp no longer names kShortestMaskable')
        self.assertEqual(tool._SHORTEST_MASKABLE, int(found.group(1)))

    def test_the_mask_never_contains_the_middle_of_the_key(self):
        key = 'ABCD' + 'SECRETMIDDLE' + 'WXYZ'
        self.assertNotIn('SECRETMIDDLE', tool.mask(key))


class Choose(unittest.TestCase):
    def test_no_keys_is_refused_rather_than_guessed(self):
        chosen, why = tool.choose([])
        self.assertIsNone(chosen)
        self.assertIn('no application keys', why)

    def test_all_invalid_is_refused_and_says_how_many(self):
        chosen, why = tool.choose([record(1, 'A' * 32, valid=False),
                                   record(2, 'B' * 32, valid=False)])
        self.assertIsNone(chosen)
        self.assertIn('2 application key(s), none of them valid', why)

    def test_one_valid_key_is_chosen_and_said_to_be_the_only_one(self):
        chosen, why = tool.choose([record(7, 'A' * 32)])
        self.assertEqual(chosen['id'], 7)
        self.assertIn('exactly one', why)

    def test_an_invalid_key_is_never_chosen_even_when_it_is_newest(self):
        chosen, _ = tool.choose([record(1, 'A' * 32, valid=True, created=100),
                                 record(2, 'B' * 32, valid=False, created=999)])
        self.assertEqual(chosen['id'], 1)

    def test_the_newest_valid_key_wins_and_the_choice_is_reported(self):
        chosen, why = tool.choose([record(1, 'A' * 32, created=100),
                                   record(2, 'B' * 32, created=300),
                                   record(3, 'C' * 32, created=200)])
        self.assertEqual(chosen['id'], 2)
        self.assertIn('2', why)
        self.assertIn('cannot be wrong for routing', why)

    def test_a_missing_createdAt_does_not_crash_the_comparison(self):
        # The API has always sent createdAt; a tool that dies on its absence turns a cosmetic
        # server change into "onboarding is broken".
        chosen, _ = tool.choose([{'id': 1, 'apiKey': 'A' * 32, 'valid': True},
                                 record(2, 'B' * 32, created=5)])
        self.assertEqual(chosen['id'], 2)


class UnexpectedResponse(unittest.TestCase):
    """A 200 that is not a list of key records, and any other status but 401/403, is described by
    its shape alone: the keys may be in it under another wrapping or another status, and stderr
    gets pasted into tickets as readily as stdout."""

    KEY = 'Qx7Zk2Wv9Bj4Mn8Pr3Ty6Lg5'

    def said(self, body, status=200):
        environment = types.SimpleNamespace(api_host='https://api.example.invalid', project_id=7,
                                            environment='test')
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(tool.c3denv, 'require_environment', return_value=environment), \
                mock.patch.object(tool, 'fetch', return_value=(status, body)), \
                contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = tool.main(['fetch_application_key.py'])
        self.assertEqual(code, 1)
        text = out.getvalue() + err.getvalue()
        runs = {self.KEY[at:at + 4] for at in range(len(self.KEY) - 3)}
        self.assertEqual([run for run in runs if run in text], [], text)
        return text

    def test_a_wrapped_list_is_named_by_its_type_and_size(self):
        self.assertIn('got dict, length 1', self.said({'data': [record(1, self.KEY)]}))

    def test_a_text_body_is_named_by_its_type_and_size(self):
        self.assertIn(f'got str, length {len(self.KEY)}', self.said(self.KEY))

    def test_an_empty_body_says_so(self):
        self.assertIn('got an empty body', self.said(None))

    def test_a_record_with_its_key_under_another_name_is_named_by_its_fields(self):
        said = self.said([{'id': 1, 'key': self.KEY, 'valid': True}])
        self.assertIn('its fields: id, key, valid', said)

    def test_another_2xx_carrying_the_key_list_is_named_by_its_shape(self):
        # urllib returns every 2xx as an ordinary response: a caching proxy's 203 is the real list
        said = self.said([record(1, self.KEY)], status=203)
        self.assertIn('FAIL HTTP 203 from apiKeys/data -- got list, length 1', said)

    def test_an_error_body_that_quotes_the_key_is_named_by_its_shape(self):
        # c3d_api.http hands back up to 400 characters of an error body as text
        said = self.said(f'upstream rejected {{"apiKey": "{self.KEY}"}}', status=500)
        self.assertIn('FAIL HTTP 500 from apiKeys/data -- got str, length', said)

    def test_an_error_with_a_json_body_is_named_by_its_shape(self):
        self.assertIn('FAIL HTTP 502 from apiKeys/data -- got dict, length 1',
                      self.said({'data': [record(1, self.KEY)]}, status=502))


if __name__ == '__main__':
    unittest.main(verbosity=2)
