"""Unit tests for tools/c3d_api/http.py -- loopback only, no real credentials.

The property under test is that an Authorization header never reaches a host we did not address.
`urllib`'s default redirect handler copies every header except content-length and content-type to
the redirect target, so a 302 from an API host to anywhere else would hand that host our
ORGANIZATION key. The credential here is a fixed non-secret placeholder.

Run: uv run tools/test_c3d_api_http.py
"""
import http.client
import pathlib
import socketserver
import sys
import threading
import unittest
import urllib.error
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from c3d_api import http as c3dhttp

PLACEHOLDER = 'orgkey-NOT-A-REAL-CREDENTIAL'
seen_by_second_host = []


class Second(BaseHTTPRequestHandler):
    def do_GET(self):
        seen_by_second_host.append(dict(self.headers))
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b'{"reached": true}')

    do_POST = do_GET

    def log_message(self, *args):
        pass


def make_first(target):
    class First(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(302)
            self.send_header('Location', target)
            self.end_headers()

        do_POST = do_GET

        def log_message(self, *args):
            pass
    return First


class RedirectsCarryCredentials(unittest.TestCase):
    def setUp(self):
        seen_by_second_host.clear()
        self.second = ThreadingHTTPServer(('127.0.0.1', 0), Second)
        threading.Thread(target=self.second.serve_forever, daemon=True).start()
        target = f'http://127.0.0.1:{self.second.server_address[1]}/landed'
        self.first = ThreadingHTTPServer(('127.0.0.1', 0), make_first(target))
        threading.Thread(target=self.first.serve_forever, daemon=True).start()
        self.first_url = f'http://127.0.0.1:{self.first.server_address[1]}/v0/projects/1/apiKeys/data'

    def tearDown(self):
        self.first.shutdown()
        self.second.shutdown()

    def test_get_refuses_the_redirect_and_reports_the_status(self):
        status, body = c3dhttp.get_json(self.first_url, PLACEHOLDER)
        self.assertEqual(status, 302)
        self.assertIn('refusing to follow a redirect', str(body))

    def test_the_second_host_never_sees_the_credential_on_a_get(self):
        c3dhttp.get_json(self.first_url, PLACEHOLDER)
        self.assertEqual(seen_by_second_host, [],
                         'the redirect target was contacted at all, so it saw the Authorization '
                         'header: urllib copies it across a redirect')

    def test_the_second_host_never_sees_the_credential_on_a_post(self):
        # post_json carries the same organization key in verify_gaze_contract.py, and a 302 on a
        # POST is redirected as a GET by the same handler.
        c3dhttp.post_json(self.first_url, PLACEHOLDER, {'sessionId': 'x'})
        self.assertEqual(seen_by_second_host, [])

    def test_a_direct_request_still_carries_the_header(self):
        # The guard must refuse redirects, not authorization. Without this, a broken opener that
        # sent no credential anywhere would pass every test above.
        direct = f'http://127.0.0.1:{self.second.server_address[1]}/v0/direct'
        status, body = c3dhttp.get_json(direct, PLACEHOLDER)
        self.assertEqual(status, 200)
        self.assertEqual(body, {'reached': True})
        self.assertEqual(len(seen_by_second_host), 1)
        self.assertEqual(seen_by_second_host[0].get('Authorization'), PLACEHOLDER)


class PeerBytesNeverEchoed(unittest.TestCase):
    """A peer that answers with no HTTP status line makes http.client raise BadStatusLine, whose text
    is the whole line the peer sent. A caller prints the status-0 body, so none of it may reach
    there: a broken proxy can answer with the very key list it should have framed."""

    def answering(self, raw):
        class Raw(socketserver.BaseRequestHandler):
            def handle(self):
                self.request.recv(65536)
                self.request.sendall(raw)

        server = socketserver.ThreadingTCPServer(('127.0.0.1', 0), Raw)
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return f'http://127.0.0.1:{server.server_address[1]}/v0/projects/1/apiKeys/data'

    def assert_unechoed(self, status, body):
        self.assertEqual(status, 0, body)
        self.assertNotIn(PLACEHOLDER, str(body))
        self.assertIn('no usable HTTP response', str(body))

    def test_a_get_answered_without_a_status_line(self):
        url = self.answering(f'[{{"apiKey": "{PLACEHOLDER}"}}]\r\n'.encode())
        self.assert_unechoed(*c3dhttp.get_json(url, 'orgkey', timeout_seconds=5))

    def test_a_malformed_status_line_carrying_the_key(self):
        url = self.answering(f'HTTP/1.1 2OO {PLACEHOLDER}\r\n\r\n'.encode())
        self.assert_unechoed(*c3dhttp.get_json(url, 'orgkey', timeout_seconds=5))

    def test_a_post_answered_without_a_status_line(self):
        url = self.answering(f'[{{"apiKey": "{PLACEHOLDER}"}}]\r\n'.encode())
        self.assert_unechoed(*c3dhttp.post_json(url, 'orgkey', {'sessionId': 'x'}, timeout_seconds=5))

    def test_a_peer_that_hangs_up_unanswered(self):
        # urllib raises it unwrapped from the response it waits for.
        url = self.answering(b'')
        status, body = c3dhttp.get_json(url, 'orgkey', timeout_seconds=5)
        self.assert_unechoed(status, body)
        self.assertIn('RemoteDisconnected', str(body))

    def test_a_failure_wrapped_while_sending_is_named_without_its_text(self):
        # urllib wraps an OSError raised while it sends in a URLError, and RemoteDisconnected is
        # one: a proxy that hangs up on CONNECT. Its text is not the peer's, but what the wrapper
        # holds is reported as the bare failure is, by type alone.
        for wrapped in (http.client.RemoteDisconnected('Remote end closed connection'),
                        http.client.BadStatusLine(f'HTTP/1.1 2OO {PLACEHOLDER}')):
            with self.subTest(wrapped=type(wrapped).__name__):
                said = c3dhttp._transport_failure(urllib.error.URLError(wrapped))
                self.assertEqual(said, f'URLError: {type(wrapped).__name__} (the peer sent no '
                                       f'usable HTTP response)')

    def test_a_refused_connection_still_says_why(self):
        # The local system's reason is kept: dropping it would cost every operator the one clue.
        probe = socketserver.TCPServer(('127.0.0.1', 0), socketserver.BaseRequestHandler)
        port = probe.server_address[1]
        probe.server_close()
        status, body = c3dhttp.get_json(f'http://127.0.0.1:{port}/', 'orgkey', timeout_seconds=5)
        self.assertEqual(status, 0)
        self.assertIn('URLError', str(body))
        self.assertNotIn('no usable HTTP response', str(body))


if __name__ == '__main__':
    unittest.main(verbosity=2)
