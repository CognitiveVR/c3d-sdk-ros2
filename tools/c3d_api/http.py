"""Thin stdlib HTTP helper for the Cognitive3D analytics API.

Only the calls this repo's tools make: a JSON POST and a JSON GET.

Returns (status, body): status 0 means no HTTP response at all (a transport-level failure);
body is parsed JSON when the server sent JSON, otherwise stripped text on success or
truncated text on an HTTP error. Nothing here raises for a transport error -- the caller
decides what a failure means.
"""
from __future__ import annotations

import http.client
import json
import urllib.error
import urllib.request


class _RefuseRedirects(urllib.request.HTTPRedirectHandler):
    """Never follow a redirect, because urllib would take the Authorization header with it.

    `HTTPRedirectHandler.redirect_request` copies every request header to the new URL except
    `content-length` and `content-type` -- so a 302 from an API host to any other host hands that
    host our ORGANIZATION key, which reads across every project in the organization. Raising here
    turns the redirect into an ordinary HTTPError, so the caller sees the 3xx status instead of a
    credential quietly changing destination.

    This is the Python counterpart of `CURLOPT_FOLLOWLOCATION 0L` in the node's C++ transport,
    which is set for exactly this reason.
    """

    def redirect_request(self, request, fp, code, msg, headers, newurl):
        raise urllib.error.HTTPError(
            request.full_url, code,
            f'{msg} -- refusing to follow a redirect to {newurl} with an Authorization header',
            headers, fp)


_OPENER = urllib.request.build_opener(_RefuseRedirects())


def shape(value: object) -> str:
    """A value's type and size, and none of its content: a body of an unexpected shape can still
    hold keys, and an error body can quote the request it refused, Authorization header included."""
    if value is None:
        return 'an empty body'
    try:
        return f'{type(value).__name__}, length {len(value)}'
    except TypeError:
        return type(value).__name__


def _transport_failure(error: BaseException) -> str:
    """What a transport-level failure says, with none of the HTTP response's bytes in it.

    An http.client.HTTPException's text is the peer's own bytes: BadStatusLine is the whole first
    line it answered with, up to 64 KiB, and a broken proxy can answer with the very body it should
    have framed. An OS, DNS, TLS or timeout reason is kept. One of those carries peer text: an HTTPS
    proxy's CONNECT refusal quotes its reason phrase. That proxy only ever saw `CONNECT host:443`,
    never the Authorization header or the response inside the tunnel, so it has no key to quote.
    """
    if isinstance(error, http.client.HTTPException):
        return f'{type(error).__name__} (the peer sent no usable HTTP response)'
    # One the error wraps: urllib wraps an OSError raised while sending, and RemoteDisconnected,
    # from a proxy that hangs up on CONNECT, is one.
    reason = getattr(error, 'reason', error)
    if reason is not error and isinstance(reason, http.client.HTTPException):
        return f'{type(error).__name__}: {type(reason).__name__} (the peer sent no usable HTTP response)'
    return f'{type(error).__name__}: {reason}'


def _decode(status: int, body: str) -> tuple[int, object]:
    if not body:
        return status, None
    try:
        return status, json.loads(body)
    except json.JSONDecodeError:
        return status, body.strip()


def post_json(url: str, authorization: str, payload: dict, timeout_seconds: float = 60) -> tuple[int, object]:
    data = json.dumps(payload).encode('utf-8')
    request = urllib.request.Request(url, data=data, method='POST', headers={
        'Authorization': authorization, 'Accept': 'application/json', 'Content-Type': 'application/json'})
    try:
        with _OPENER.open(request, timeout=timeout_seconds) as response:
            return _decode(response.status, response.read().decode('utf-8', 'replace'))
    except urllib.error.HTTPError as error:
        # error.read() runs INSIDE this handler, so an IncompleteRead raised while
        # draining the error body escapes the clause below rather than being
        # caught by it -- an intercepting proxy answering 502 with a Content-Length
        # larger than it sends does exactly that. The status is the useful half of
        # the answer either way, so keep it and give up on the body.
        try:
            body = error.read().decode('utf-8', 'replace')[:400]
        except Exception as read_error:
            return error.code, f'<body unreadable: {type(read_error).__name__}>'
        # An error status with an empty body says nothing; a 3xx has no body at all. Fall back to
        # the reason so a caller printing `body` has something to print.
        # Both halves are truncated the same way. The reason can carry a redirect URL, and a
        # caller printing `body` should not have one path bounded and the other not.
        return error.code, body if body.strip() else str(error.reason)[:400]
    # http.client.HTTPException (IncompleteRead, most often) derives from Exception,
    # NOT from OSError, so it escapes a URLError/OSError tuple unless named in it.
    except (urllib.error.URLError, TimeoutError, OSError, http.client.HTTPException) as error:
        return 0, _transport_failure(error)


def get_json(url: str, authorization: str, timeout_seconds: float = 60) -> tuple[int, object]:
    """GET with the same (status, body) contract as post_json.

    Redirects are refused, not followed -- see `_RefuseRedirects`. Resolving the initial host
    through `c3d_api.env.resolve` constrains where the request is SENT and says nothing about
    where a 3xx could send it next, which is the whole hazard.
    """
    request = urllib.request.Request(url, method='GET', headers={
        'Authorization': authorization, 'Accept': 'application/json'})
    try:
        with _OPENER.open(request, timeout=timeout_seconds) as response:
            return _decode(response.status, response.read().decode('utf-8', 'replace'))
    except urllib.error.HTTPError as error:
        try:
            body = error.read().decode('utf-8', 'replace')[:400]
        except Exception as read_error:
            return error.code, f'<body unreadable: {type(read_error).__name__}>'
        # An error status with an empty body says nothing; a 3xx has no body at all. Fall back to
        # the reason so a caller printing `body` has something to print.
        # Both halves are truncated the same way. The reason can carry a redirect URL, and a
        # caller printing `body` should not have one path bounded and the other not.
        return error.code, body if body.strip() else str(error.reason)[:400]
    except (urllib.error.URLError, TimeoutError, OSError, http.client.HTTPException) as error:
        return 0, _transport_failure(error)
