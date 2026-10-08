"""Unit tests for tools/c3d_api/env.py -- no network, no real credentials.

The host decision is tested against core/test/environment_cases.json, the same table the C++
resolver in core/src/environment.cpp is tested against, so the two halves of the toolchain cannot
come to disagree about which hosts a process environment selects.

Run: uv run tools/test_c3d_api_env.py
"""
import json
import os
import pathlib
import sys
import unittest
from unittest import mock

TOOLS = pathlib.Path(__file__).parent
sys.path.insert(0, str(TOOLS))
from c3d_api import env as c3denv  # noqa: E402

CASES = TOOLS.parent / 'core' / 'test' / 'environment_cases.json'


class SharedCaseTable(unittest.TestCase):
    def test_every_case_the_core_resolver_is_tested_against(self):
        table = json.loads(CASES.read_text())
        self.assertGreaterEqual(len(table['cases']), 20)
        for case in table['cases']:
            with self.subTest(env=case['env']):
                if case['accept']:
                    resolved = c3denv.resolve(case['env'])
                    self.assertEqual(resolved.name, case['name'])
                    self.assertEqual(resolved.hosts.data, case['data'])
                    self.assertEqual(resolved.hosts.api, case['api'])
                    self.assertEqual(resolved.custom_hosts, case['custom'])
                else:
                    with self.assertRaises(ValueError) as refused:
                        c3denv.resolve(case['env'])
                    for variable in case['mentions']:
                        self.assertIn(variable, str(refused.exception))
                    if 'says' in case:
                        self.assertIn(case['says'], str(refused.exception))


class RequireEnvironment(unittest.TestCase):
    BASE = {'C3D_PROJECT_ID': '7', 'C3D_ORG_API_KEY': 'orgkey-placeholder'}

    def test_unset_environment_is_production(self):
        with mock.patch.dict(os.environ, self.BASE, clear=True):
            environment = c3denv.require_environment()
        self.assertEqual(environment.environment, 'prod')
        self.assertEqual(environment.api_host, 'https://api.cognitive3d.com')

    def test_the_override_pair_moves_the_api_host(self):
        with mock.patch.dict(os.environ, {**self.BASE, 'C3D_DATA_HOST': 'data.example.test',
                                          'C3D_API_HOST': 'api.example.test'}, clear=True):
            environment = c3denv.require_environment()
        self.assertEqual(environment.environment, 'custom')
        self.assertEqual(environment.api_host, 'https://api.example.test')

    def test_dev_is_refused_and_the_refusal_names_the_variable(self):
        with mock.patch.dict(os.environ, {**self.BASE, 'C3D_ENVIRONMENT': 'dev'}, clear=True):
            with self.assertRaises(SystemExit) as refused:
                c3denv.require_environment()
        self.assertIn('C3D_ENVIRONMENT', str(refused.exception))

    def test_half_a_pair_is_refused_naming_the_other_half(self):
        with mock.patch.dict(os.environ, {**self.BASE, 'C3D_API_HOST': 'api.example.test'},
                             clear=True):
            with self.assertRaises(SystemExit) as refused:
                c3denv.require_environment()
        self.assertIn('C3D_DATA_HOST', str(refused.exception))

    def test_the_project_and_key_are_still_required(self):
        with mock.patch.dict(os.environ, {'C3D_ORG_API_KEY': 'orgkey-placeholder'}, clear=True):
            with self.assertRaises(SystemExit) as refused:
                c3denv.require_environment()
        self.assertIn('C3D_PROJECT_ID', str(refused.exception))



class TheKeyStaysOutOfPrintedForms(unittest.TestCase):
    KEY = 'orgkey-placeholder-that-must-not-print'

    def test_repr_and_str_name_everything_but_the_key(self):
        with mock.patch.dict(os.environ, {'C3D_PROJECT_ID': '7', 'C3D_ORG_API_KEY': self.KEY},
                             clear=True):
            environment = c3denv.require_environment()
        self.assertEqual(environment.org_key, self.KEY)
        for printed in (repr(environment), str(environment), f'{environment}',
                        repr([environment])):
            with self.subTest(printed=printed):
                self.assertNotIn(self.KEY, printed)
                self.assertNotIn('placeholder', printed)
                self.assertIn('project_id=7', printed)
                self.assertIn('api.cognitive3d.com', printed)


if __name__ == '__main__':
    unittest.main(verbosity=2)
