#!/usr/bin/env python3
"""Derive `C3D_APPLICATION_API_KEY` from an organization key and a project id.

Why this exists rather than "copy it from the dashboard": the application key is the value whose
mismatch is SILENT. It is not a secret -- it ships in every client and doubles as routing, naming
the project that receives the data -- but point it at the wrong project and the collector answers
200 to every POST while the session never appears. Reading it from the API removes the
transcription step where that mistake is made, and `doctor`'s key/project check is the backstop
for anyone who still types it by hand.

Reads `C3D_PROJECT_ID` and `C3D_ORG_API_KEY` from the process environment, and the hosts as
`c3d_api/env.py` resolves them: production, unless `C3D_DATA_HOST` and `C3D_API_HOST` together name
another stack. An organization key is required: the route below refuses a developer key with
401.

  ros2 run cognitive3d_ros fetch_application_key.py              # what this project has, masked
  ros2 run cognitive3d_ros fetch_application_key.py --env-line   # C3D_APPLICATION_API_KEY=... to append

From a source checkout, `uv run tools/fetch_application_key.py` takes the same arguments.
"""
from __future__ import annotations

import argparse
import sys

from c3d_api import env as c3denv, http as c3dhttp


# doctor's MaskSecret threshold: below it, even the last four characters are much of the key.
_SHORTEST_MASKABLE = 16


def mask(key: str) -> str:
    """The key exactly as doctor prints it: `ending ...wxyz`, or its length alone when short."""
    if not key:
        return '<empty>'
    if len(key) < _SHORTEST_MASKABLE:
        return f'<{len(key)} characters, too short to mask safely>'
    return f'ending ...{key[-4:]}'


def choose(keys: list[dict]) -> tuple[dict | None, str]:
    """The key to use and a one-line account of why, or (None, reason) when none is usable.

    Any VALID key of a project routes to that project, so among several the choice cannot be
    wrong for routing -- but it is still reported, because "we picked one" silently is how a
    person ends up unable to explain which credential a session used.
    """
    if not keys:
        return None, 'this project has no application keys'
    valid = [k for k in keys if k.get('valid')]
    if not valid:
        return None, f'{len(keys)} application key(s), none of them valid'
    newest = max(valid, key=lambda k: k.get('createdAt') or 0)
    if len(valid) == 1:
        return newest, 'the project has exactly one valid application key'
    return newest, (f'{len(valid)} valid keys; chose the most recently created (id '
                    f'{newest.get("id")}). Any valid key of this project routes to it, so this '
                    f'choice cannot be wrong for routing -- name a different one yourself if it '
                    f'matters for your own bookkeeping.')


def fetch(environment, timeout_seconds: float = 60) -> tuple[int, object]:
    url = (f'{environment.api_host}/v0/projects/{environment.project_id}/apiKeys/data')
    return c3dhttp.get_json(url, c3denv.auth_header(environment.org_key), timeout_seconds)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--env-line', action='store_true',
                        help='print `C3D_APPLICATION_API_KEY=<key>` and nothing else, so it can '
                             'be appended to a .env file. Without this the key is masked.')
    arguments = parser.parse_args(argv[1:])

    environment = c3denv.require_environment()
    status, body = fetch(environment)

    if status == 0:
        print(f'FAIL no response from {environment.api_host}: {body}', file=sys.stderr)
        return 1
    if status in (401, 403):
        print(f'FAIL {environment.api_host} answered {status}. This route needs an ORGANIZATION '
              f'key; a developer key is refused. Check C3D_ORG_API_KEY.', file=sys.stderr)
        return 1
    # Any other status is described by its shape too, never echoed: urllib hands back every 2xx as
    # an ordinary response, so a proxy's 203 carries the same key list a 200 would, and an error
    # body can quote the request it refused.
    if status != 200:
        print(f'FAIL HTTP {status} from apiKeys/data -- got {c3dhttp.shape(body)}', file=sys.stderr)
        return 1
    if not isinstance(body, list):
        print(f'FAIL HTTP 200 but the body is not a list of keys -- got {c3dhttp.shape(body)}',
              file=sys.stderr)
        return 1

    chosen, why = choose(body)
    if chosen is None:
        print(f'FAIL project {environment.project_id}: {why}', file=sys.stderr)
        return 1

    key = chosen.get('apiKey') or ''
    if not key:
        fields = ', '.join(sorted(str(field) for field in chosen)) or 'none'
        print(f'FAIL the chosen key record carries no apiKey field -- its fields: {fields}',
              file=sys.stderr)
        return 1

    if arguments.env_line:
        print(f'C3D_APPLICATION_API_KEY={key}')
        return 0

    print(f'project {environment.project_id} on {environment.environment} '
          f'({environment.api_host})')
    print(f'  {why}')
    for record in body:
        marker = '->' if record.get('id') == chosen.get('id') else '  '
        state = 'valid' if record.get('valid') else 'INVALID'
        print(f'  {marker} id {record.get("id")}  {mask(record.get("apiKey") or "")}  {state}')
    print()
    print('Masked above on purpose -- a terminal transcript gets pasted into tickets. To write it:')
    print('  ros2 run cognitive3d_ros fetch_application_key.py --env-line >> .env')
    print('or, from a source checkout:')
    print('  uv run tools/fetch_application_key.py --env-line >> .env')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
