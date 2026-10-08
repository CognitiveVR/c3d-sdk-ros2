"""Stand-in for data.cognitive3d.com.

Accepts the ingestion POSTs, answers 200 with the cvr-request-time header the
client checks for, and writes every payload to disk so a test can assert on what
would actually have been transmitted.

A JSON stream lands as one `<NNNNNN>-<stream>.json` carrying the parsed body. The
camera and lidar routes send packed Cap'n Proto, so those land as TWO files: the
bytes as `<NNNNNN>-<stream>.bin`, and the same metadata record beside it naming
the body file and its size. `uv run tools/capnp_dump.py <file>.bin` decodes one.

## What it deliberately does NOT write

The `Authorization` header's value is a live API key. Recorded verbatim, it would
land in every capture file -- hundreds per run, each carrying the project's DATA
key in plaintext -- and nothing reads the value.

That is the write-time counterpart of "never print a secret". Printing puts a
credential in a transcript; writing puts it in a file that is later attached to a
bug report, copied to compare two runs, or swept into a commit by `git add -A`.
A tool whose whole purpose is to capture request traffic for inspection is
exactly where that distinction gets lost.

So the record carries `auth_present`, `auth_scheme` and `auth_credential_present`
instead. `auth_scheme` is resolved against a fixed allowlist rather than by
splitting the value, because not every scheme is space-separated: `orgkey-<key>`
is one token, so `value.split(" ")[0]` would return the entire credential. Each
allowlist entry INCLUDES its own separator, so a prefix match is a boundary
match -- without that, `APIKEY:DATABASE <key>` would report as `APIKEY:DATA` and
`BearerXYZ` as `Bearer`, which is the opposite of proving the header well formed.

## Why writes are renamed into place

A harness SIGTERMs this process during teardown. Dumping straight onto an open
handle can leave a truncated JSON object, which every reader would then have to
defend against by name.

Renaming fixes that but trades one failure for another, and the trade is only
worth it because of the signal handler below: a kill landing between the open
and the rename leaves a `.partial` that matches NO reader's glob, so the capture
is silently short by one instead of loudly corrupt -- and silently short is the
worse failure, because a wrong aggregate still looks measured. Hence SIGTERM is
caught and in-flight handlers are joined rather than killed. The `.partial`
orphan remains possible only on SIGKILL or a write error, and a reader should
treat a leftover `.partial` as a named failure for that reason.
"""
import json, os, signal, sys, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/c3d_capture"
os.makedirs(OUT, exist_ok=True)
COUNT = {"n": 0}
# ThreadingHTTPServer serves each request on its own thread, so the counter that
# names the file needs one. Without it two concurrent POSTs can read the same
# value, land on the same filename, and one payload is overwritten with no trace
# -- a capture that is silently short by one, which is the worst way for a
# capture tool to fail.
LOCK = threading.Lock()

# Every Authorization scheme this SDK's endpoints accept, each INCLUDING the
# separator that must follow it. Matching a prefix and emitting the prefix is
# what makes it impossible to leak the key by accident; including the separator
# is what stops a longer scheme being reported as a shorter one.
AUTH_SCHEMES = ("APIKEY:DATA ", "APIKEY:ORGANIZATION ", "APIKEY:DEVELOPER ", "Bearer ", "orgkey-")

# The suffix an in-flight capture carries. Chosen so it cannot match any reader's
# glob (`*-<stream>.json`, `<stream>_*.json`, `*.json`, `*-<stream>.bin`);
# `temporary_path` is the single definition, so a test can assert that property
# against the source.
PARTIAL_SUFFIX = ".partial"

# The camera and lidar routes send packed Cap'n Proto, not JSON. The CONTENT TYPE
# decides, not the stream name, because that is what the real gateway dispatches
# on -- a JSON body announced as octet-stream is a defect at the sender and this
# tool should show it rather than hide it by re-reading the route.
BINARY_CONTENT_TYPE = "application/octet-stream"


def temporary_path(final_path):
    """Where a capture is written before it is renamed to `final_path`."""
    return f"{final_path}{PARTIAL_SUFFIX}"


def write_atomically(path, mode, write):
    """Write through a `.partial` and rename into place, or leave nothing behind.

    One definition for both halves of a binary capture. The body and its metadata
    are two files, and a reader that found the metadata for a body that was never
    renamed would be reading a capture that is silently short -- so the body is
    written FIRST and the record only afterwards.
    """
    temporary = temporary_path(path)
    try:
        with open(temporary, mode) as f:
            write(f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary, path)
    except BaseException:
        # A failed write must not leave an orphan behind to accumulate across
        # runs. The final name is only ever reached through the rename above.
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def auth_scheme(value):
    """The scheme of an Authorization header, or None/"unrecognised". Never the key."""
    if value is None:
        return None
    for scheme in AUTH_SCHEMES:
        if value.startswith(scheme):
            return scheme.rstrip(" ")
    return "unrecognised"


def auth_credential_present(value):
    """Whether anything followed the scheme. Never what followed it.

    `auth_present` alone cannot tell a keyed header from `APIKEY:DATA` with no
    key or an empty one, so on its own it cannot prove the header usable. This
    can, without recording the value.
    """
    if value is None:
        return False
    for scheme in AUTH_SCHEMES:
        if value.startswith(scheme):
            return bool(value[len(scheme):].strip())
    return False


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length)
        parsed = urlparse(self.path)
        segments = parsed.path.strip("/").split("/")
        if len(segments) < 2:
            # Answer, rather than raising out of the handler. An unhandled
            # IndexError kills the thread with no response at all, which a client
            # sees as a dropped connection and a harness sees as a capture that
            # is short by one with nothing on disk to explain it.
            self.send_error(400, "expected /v<n>/<stream>/<scene>")
            return
        stream = segments[1]
        with LOCK:
            COUNT["n"] += 1
            index = COUNT["n"]
        authorization = self.headers.get("Authorization")
        record = {
            "path": self.path,
            "stream": stream,
            "version": parse_qs(parsed.query).get("version", [None])[0],
            # Presence, scheme and whether a credential followed it -- see the
            # module docstring. Never the value.
            "auth_present": authorization is not None,
            "auth_scheme": auth_scheme(authorization),
            "auth_credential_present": auth_credential_present(authorization),
            "content_type": self.headers.get("Content-Type"),
            "method_override": self.headers.get("X-HTTP-Method-Override"),
        }
        if record["content_type"] == BINARY_CONTENT_TYPE:
            # The bytes go to their own file, not into the record. Cap'n Proto is
            # arbitrary bytes and a JSON string cannot hold them; base64 in the
            # record would make every reader decode before it could even count
            # frames, and `uv run tools/capnp_dump.py <file>.bin` is the point.
            body_name = f"{index:06d}-{stream}.bin"
            write_atomically(os.path.join(OUT, body_name), "wb", lambda f: f.write(body))
            record["body_file"] = body_name
            record["body_bytes"] = len(body)
        else:
            try:
                record["body"] = json.loads(body)
            except Exception as e:
                record["parse_error"] = str(e)
                record["raw"] = body.decode("utf-8", "replace")
        write_atomically(os.path.join(OUT, f"{index:06d}-{stream}.json"), "w",
                         lambda f: json.dump(record, f, indent=1))
        self.send_response(200)
        # The client treats a 200 without this header as a captive portal. Sent
        # only after the rename, so a client holding a 200 has a complete file.
        self.send_header("cvr-request-time", "12")
        self.send_header("Content-Length", "2")
        self.end_headers()
        self.wfile.write(b"ok")

    def log_message(self, *a):
        pass


def serve(port, server_class=ThreadingHTTPServer):
    """Run until SIGTERM/SIGINT, finishing any capture already in flight."""
    server = server_class(("127.0.0.1", port), Handler)
    # NOT daemon threads: at teardown an in-flight handler must be allowed to
    # finish its rename. Killed between the open and the rename it leaves a
    # `.partial` that matches no reader's glob -- a capture silently short by
    # one, which is worse than a truncated file.
    server.daemon_threads = False

    def stop(_signum, _frame):
        # From a helper thread: shutdown() waits for serve_forever() to return,
        # so calling it on the thread that is inside serve_forever() deadlocks.
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    serve(int(sys.argv[1]))
