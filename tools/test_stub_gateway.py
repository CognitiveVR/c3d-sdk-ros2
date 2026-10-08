"""Unit tests for tools/stub_gateway.py -- no network beyond loopback, no real credentials.

The property under test is that a capture tool never writes the credential it captured. Every key
used here is a fixed non-secret placeholder.

Run: uv run tools/test_stub_gateway.py
"""
import importlib.util
import json
import pathlib
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

# Loaded by path with argv set, because the module reads sys.argv[2] at import to choose its output
# directory. The __main__ guard is what makes importing it safe at all.
_OUT = tempfile.mkdtemp()
_REAL_ARGV = list(sys.argv)
sys.argv = ['stub_gateway.py', '0', _OUT]
try:
    _spec = importlib.util.spec_from_file_location(
        'stub_gateway', pathlib.Path(__file__).resolve().parent / 'stub_gateway.py')
    stub_gateway = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(stub_gateway)
finally:
    # Restored, or unittest.main() would parse the fake argv as test names to run.
    sys.argv = _REAL_ARGV

# Not a credential: a fixed placeholder of the right shape (32 chars, the DATA key shape).
PLACEHOLDER_KEY = 'AAAABBBBCCCCDDDDEEEEFFFFGGGGHHHH'


class TestAuthScheme(unittest.TestCase):
    """The scheme is resolved against an allowlist, never by splitting the value."""

    def test_space_separated_schemes_report_the_scheme(self):
        for scheme in ('APIKEY:DATA', 'APIKEY:ORGANIZATION', 'APIKEY:DEVELOPER', 'Bearer'):
            self.assertEqual(stub_gateway.auth_scheme(f'{scheme} {PLACEHOLDER_KEY}'), scheme)

    def test_a_longer_scheme_is_not_reported_as_a_shorter_one(self):
        """A bare prefix match has no boundary, so `APIKEY:DATABASE <key>` would report
        as `APIKEY:DATA` and `BearerXYZ` as `Bearer` -- the opposite of proving the header
        well formed. Each allowlist entry carries its
        own separator so that a prefix match IS a boundary match."""
        for value in (f'APIKEY:DATABASE {PLACEHOLDER_KEY}', f'BearerNot-Separated-{PLACEHOLDER_KEY}',
                      'APIKEY:DATA', 'Bearer', f'apikey:data {PLACEHOLDER_KEY}'):
            self.assertEqual(stub_gateway.auth_scheme(value), 'unrecognised', value)

    def test_a_scheme_with_no_credential_after_it_is_distinguishable(self):
        """`auth_present` alone cannot tell a keyed header from `APIKEY:DATA ` with an
        empty key."""
        self.assertTrue(stub_gateway.auth_credential_present(f'APIKEY:DATA {PLACEHOLDER_KEY}'))
        self.assertFalse(stub_gateway.auth_credential_present('APIKEY:DATA '))
        self.assertFalse(stub_gateway.auth_credential_present('APIKEY:DATA'))
        self.assertFalse(stub_gateway.auth_credential_present(None))
        self.assertTrue(stub_gateway.auth_credential_present(f'orgkey-{PLACEHOLDER_KEY}'))

    def test_a_scheme_with_no_space_does_not_leak_the_key(self):
        """`orgkey-<key>` is ONE token. `value.split(" ")[0]` -- the obvious implementation --
        returns the entire credential for this shape, which is why the allowlist exists."""
        result = stub_gateway.auth_scheme(f'orgkey-{PLACEHOLDER_KEY}')
        self.assertEqual(result, 'orgkey-')
        self.assertNotIn(PLACEHOLDER_KEY, result)

    def test_an_unknown_scheme_reports_a_constant_not_the_value(self):
        result = stub_gateway.auth_scheme(f'Something-{PLACEHOLDER_KEY}')
        self.assertEqual(result, 'unrecognised')
        self.assertNotIn(PLACEHOLDER_KEY, result)

    def test_a_missing_header_is_none(self):
        self.assertIsNone(stub_gateway.auth_scheme(None))

    def test_no_input_can_appear_in_the_output(self):
        """The exhaustive statement of the property: whatever goes in, the answer is one of the
        allowlist's own constants, `unrecognised`, or None."""
        allowed = {scheme.rstrip(' ') for scheme in stub_gateway.AUTH_SCHEMES} | {'unrecognised', None}
        for value in (None, '', PLACEHOLDER_KEY, f'APIKEY:DATA {PLACEHOLDER_KEY}',
                      f'orgkey-{PLACEHOLDER_KEY}', f'Bearer {PLACEHOLDER_KEY}',
                      'APIKEY:DATA', 'orgkey-', f'  APIKEY:DATA {PLACEHOLDER_KEY}'):
            self.assertIn(stub_gateway.auth_scheme(value), allowed, value)


class TestCapturedRecord(unittest.TestCase):
    """End to end over loopback: what actually lands on disk."""

    class Server(ThreadingHTTPServer):
        # The default listen backlog is 5. The concurrency test opens 40 connections at once,
        # and on a loaded CI runner the surplus are refused -- the test then fails as
        # "38 != 40", which reads like a lost capture rather than a refused connection.
        request_queue_size = 128

    def setUp(self):
        self.directory = pathlib.Path(tempfile.mkdtemp())
        stub_gateway.OUT = str(self.directory)
        stub_gateway.COUNT['n'] = 0
        self.server = self.Server(('127.0.0.1', 0), stub_gateway.Handler)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.thread.join(timeout=5)
        self.server.server_close()      # or each test leaks a listening socket

    def post(self, part: int, stream: str = 'gaze') -> None:
        request = urllib.request.Request(
            f'http://127.0.0.1:{self.port}/v0/{stream}/scene-1?version=2',
            data=json.dumps({'part': part, 'data': [{'time': float(part)}]}).encode(),
            headers={'Authorization': f'APIKEY:DATA {PLACEHOLDER_KEY}',
                     'Content-Type': 'application/json'})
        urllib.request.urlopen(request).read()

    def test_the_credential_never_reaches_disk(self):
        """The whole point. Recording the Authorization header verbatim would leave hundreds of
        files per run each carrying the project's live DATA key -- a run directory that could not
        safely be attached to a bug report."""
        self.post(0)
        captured = sorted(self.directory.glob('*-gaze.json'))
        self.assertEqual(len(captured), 1)
        text = captured[0].read_text()
        self.assertNotIn(PLACEHOLDER_KEY, text)
        record = json.loads(text)
        # The EXACT key set, not just "auth is absent". A new field carrying
        # `(authorization or '')[-6:]` passes an assertion that only looks for the whole
        # key -- and an accidental new field is precisely the shape the allowlist exists to
        # prevent.
        self.assertEqual(set(record), {
            'path', 'stream', 'version', 'auth_present', 'auth_scheme',
            'auth_credential_present', 'content_type', 'method_override', 'body'})
        self.assertTrue(record['auth_present'])
        self.assertEqual(record['auth_scheme'], 'APIKEY:DATA')
        self.assertTrue(record['auth_credential_present'])

    def test_the_rest_of_the_request_is_still_captured(self):
        """Redaction must not cost the assertions harnesses actually make."""
        self.post(7, stream='dynamics')
        record = json.loads(next(self.directory.glob('*-dynamics.json')).read_text())
        self.assertEqual(record['path'], '/v0/dynamics/scene-1?version=2')
        self.assertEqual(record['stream'], 'dynamics')
        self.assertEqual(record['version'], '2')
        self.assertEqual(record['content_type'], 'application/json')
        self.assertEqual(record['body']['part'], 7)

    def test_every_concurrent_post_gets_its_own_file(self):
        """`ThreadingHTTPServer` serves each request on its own thread, and `COUNT['n'] += 1` is a
        read-modify-write. CPython's GIL makes the window narrow enough that this test does not
        reliably fail without the lock -- it is a guard against a latent race, not a reproduction
        of a measured one. Stated that way on purpose: the lock is correct and free, and claiming
        a failure this test cannot demonstrate would be worse than claiming nothing.
        """
        failures = []

        def attempt(index):
            try:
                self.post(index)
            except Exception as error:      # noqa: BLE001 -- recorded, then asserted below
                failures.append(f'{type(error).__name__}: {error}')

        threads = [threading.Thread(target=attempt, args=(index,)) for index in range(40)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        # Asserted, not swallowed: a client error inside a worker thread would disappear, so a
        # refused connection would present as a missing capture and blame the wrong mechanism.
        self.assertEqual(failures, [])
        self.assertEqual(len(sorted(self.directory.glob('*-gaze.json'))), 40)

    def test_no_partial_file_is_left_behind_and_every_capture_is_valid_json(self):
        """Records are written to a temporary name and renamed into place. A harness SIGTERMs
        this process during teardown, and dumping straight onto an open handle can leave a
        truncated object that every reader would have to defend against by name."""
        for part in range(5):
            self.post(part)
        self.assertEqual(list(self.directory.glob(f'*{stub_gateway.PARTIAL_SUFFIX}')), [])
        for path in self.directory.glob('*-gaze.json'):
            json.loads(path.read_text())      # raises if any capture is truncated

    def test_the_final_name_is_reachable_only_through_the_rename(self):
        """The property the temporary name exists for, constrained against the SOURCE.

        Observing a successful write cannot tell the rename from a direct `open(path, "w")`:
        both leave a complete file and no temporary. Here `json.dump` is made to fail, so the
        only way a `*-gaze.json` could appear is if the code wrote straight to the final name.
        """
        class FailingJson:
            """Only this module's view of `json` is replaced.

            Assigning to `stub_gateway.json.dump` instead reaches the real `json` module for the
            whole process -- shared with every other test and with unittest itself.
            """
            loads = staticmethod(json.loads)

            @staticmethod
            def dump(*_args, **_kwargs):
                raise RuntimeError('disk full')

        original_json, original_handle_error = stub_gateway.json, self.server.handle_error
        stub_gateway.json = FailingJson
        # The handler is now expected to raise; its traceback on stderr is noise, not a result.
        self.server.handle_error = lambda *_a: None
        try:
            with self.assertRaises(Exception):
                self.post(0)
        finally:
            stub_gateway.json = original_json
            self.server.handle_error = original_handle_error
        self.assertEqual(list(self.directory.glob('*-gaze.json')), [],
                         'a failed write must not produce a readable capture')
        self.assertEqual(list(self.directory.glob(f'*{stub_gateway.PARTIAL_SUFFIX}')), [],
                         'a failed write must not leave an orphan to accumulate')

    def test_the_temporary_name_cannot_be_mistaken_for_a_capture(self):
        """Readers glob `*-<stream>.json`, `<stream>_*.json` and `*.json`; the in-flight name
        must match none of them.

        Derived from `temporary_path` rather than hard-coded: asserting a literal string tests
        `PurePath.match` and cannot fail from ANY change to the source, so a temp name matching
        every reader's glob would still pass.
        """
        in_flight = pathlib.PurePath(stub_gateway.temporary_path('000001-gaze.json'))
        for pattern in ('*-gaze.json', 'gaze_*.json', '*.json'):
            self.assertFalse(in_flight.match(pattern), f'{in_flight} matches {pattern}')

    def post_binary(self, stream: str, body: bytes) -> None:
        """The camera and lidar routes: session-keyed, so no scene id and no version."""
        request = urllib.request.Request(
            f'http://127.0.0.1:{self.port}/v0/{stream}',
            data=body,
            headers={'Authorization': f'APIKEY:DATA {PLACEHOLDER_KEY}',
                     'Content-Type': 'application/octet-stream'})
        urllib.request.urlopen(request).read()

    def test_a_binary_body_lands_as_bytes_beside_its_record(self):
        """Every byte value, because that is what a packed Cap'n Proto payload is and
        what a JSON string could not have carried."""
        body = bytes(range(256))
        self.post_binary('camera', body)

        captured = sorted(self.directory.glob('*-camera.bin'))
        self.assertEqual(len(captured), 1)
        self.assertEqual(captured[0].read_bytes(), body)

        record = json.loads(next(self.directory.glob('*-camera.json')).read_text())
        self.assertEqual(record['stream'], 'camera')
        self.assertEqual(record['path'], '/v0/camera')
        self.assertIsNone(record['version'])
        self.assertEqual(record['content_type'], 'application/octet-stream')
        self.assertEqual(record['body_file'], captured[0].name)
        self.assertEqual(record['body_bytes'], 256)
        # The bytes are NOT in the record: a reader counts frames with capnp_dump,
        # and a base64 copy here would be a second encoding of the same payload.
        self.assertNotIn('body', record)

    def test_a_binary_capture_never_records_the_credential_either(self):
        """The redaction is a property of the tool, not of the JSON path."""
        self.post_binary('lidar', b'\x00\x01\x02')
        for path in list(self.directory.glob('*-lidar.json')) + \
                list(self.directory.glob('*-lidar.bin')):
            self.assertNotIn(PLACEHOLDER_KEY.encode(), path.read_bytes())
        record = json.loads(next(self.directory.glob('*-lidar.json')).read_text())
        self.assertEqual(set(record), {
            'path', 'stream', 'version', 'auth_present', 'auth_scheme',
            'auth_credential_present', 'content_type', 'method_override',
            'body_file', 'body_bytes'})
        self.assertEqual(record['auth_scheme'], 'APIKEY:DATA')
        self.assertTrue(record['auth_credential_present'])

    def test_the_binary_body_is_renamed_into_place_like_everything_else(self):
        """A reader globbing `*-camera.bin` must never see a half-written body, and a
        failed write must leave neither half of the capture behind.

        The rename is forced to FAIL rather than merely observed to have happened.
        Asserting only that no `.partial` survives a successful write passes just as
        well for a straight `open(path, "wb").write(body)`, because a direct write
        leaves no temporary either. Breaking
        `os.replace` separates them: through the rename nothing is left behind, and
        written directly the body is already at its final name.
        """
        in_flight = pathlib.PurePath(stub_gateway.temporary_path('000001-camera.bin'))
        for pattern in ('*-camera.bin', '*.bin', '*-camera.json'):
            self.assertFalse(in_flight.match(pattern), f'{in_flight} matches {pattern}')

        self.post_binary('camera', b'abc')
        self.assertEqual(list(self.directory.glob(f'*{stub_gateway.PARTIAL_SUFFIX}')), [])

        class FailingReplace:
            """Only this module's view of `os`; everything but `replace` is the real one."""

            @staticmethod
            def replace(*_args):
                raise RuntimeError('disk full')

            def __getattr__(self, name):
                import os as real_os
                return getattr(real_os, name)

        original_os, original_handle_error = stub_gateway.os, self.server.handle_error
        stub_gateway.os = FailingReplace()
        self.server.handle_error = lambda *_a: None
        try:
            with self.assertRaises(Exception):
                self.post_binary('lidar', b'def')
        finally:
            stub_gateway.os = original_os
            self.server.handle_error = original_handle_error
        self.assertEqual(list(self.directory.glob('*-lidar.bin')), [],
                         'a body that never reached its final name must not be readable')
        self.assertEqual(list(self.directory.glob(f'*{stub_gateway.PARTIAL_SUFFIX}')), [],
                         'a failed write must not leave an orphan to accumulate')

    def test_a_json_stream_is_unaffected_by_the_binary_path(self):
        """The four JSON streams keep the record shape every existing reader parses."""
        self.post(3)
        self.assertEqual(list(self.directory.glob('*.bin')), [])
        record = json.loads(next(self.directory.glob('*-gaze.json')).read_text())
        self.assertEqual(record['body']['part'], 3)
        self.assertNotIn('body_file', record)

    def test_a_path_with_no_stream_segment_answers_instead_of_killing_the_thread(self):
        """`/` and `/v0` must not raise IndexError out of the handler: that means no response,
        no file, and a client seeing a dropped connection."""
        for path in ('/', '/v0'):
            request = urllib.request.Request(f'http://127.0.0.1:{self.port}{path}', data=b'{}',
                                             headers={'Content-Type': 'application/json'})
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(request, timeout=5)
            self.assertEqual(caught.exception.code, 400)
        self.post(0)        # and the server is still serving afterwards
        self.assertEqual(len(list(self.directory.glob('*-gaze.json'))), 1)


if __name__ == '__main__':
    unittest.main()
