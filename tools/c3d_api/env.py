"""Cognitive3D analytics-API environment: which host, which project, which key -- read from
the process environment. The key never leaves this process except as an HTTP header.

Host resolution mirrors core/src/environment.cpp exactly, and both are tested against the same
case table, core/test/environment_cases.json:

  * C3D_ENVIRONMENT unset or exactly `prod` is production; any other value, including the empty
    string, is refused. Matched exactly: no trimming, no case folding.
  * C3D_DATA_HOST and C3D_API_HOST are set together or not at all, and never beside
    C3D_ENVIRONMENT: the hosts are named one way. Set, each must be a bare host with an optional
    :port, and together they replace the production pair.

The two halves of the toolchain must never disagree about which environment a run is in, so a
rule changed here is changed there in the same commit, and the table gains the case.

Only the three variables `tools/verify_gaze_contract.py` and its siblings need are read: they talk
only to the analytics REST API (`api.<host>`) with the organization key, never to ingestion (the
application key) or scene/mesh upload (the developer key). There is no `.env` file fallback.
"""
from __future__ import annotations

import dataclasses
import os
import re
from typing import Mapping

DEFAULT_ENVIRONMENT = 'prod'
CUSTOM_ENVIRONMENT = 'custom'
ENVIRONMENT_VARIABLE = 'C3D_ENVIRONMENT'
DATA_HOST_VARIABLE = 'C3D_DATA_HOST'
API_HOST_VARIABLE = 'C3D_API_HOST'
REQUIRED = ('C3D_PROJECT_ID', 'C3D_ORG_API_KEY')


@dataclasses.dataclass(frozen=True)
class Hosts:
    data: str
    api: str


_TABLE = {'prod': Hosts('data.cognitive3d.com', 'api.cognitive3d.com')}

# A host name or IPv4 literal, then an optional :port. A scheme, path, query or `user@` would let
# the value move a request somewhere its host part does not say.
_BARE_HOST = re.compile(r'[A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?(?::[0-9]{1,5})?')


@dataclasses.dataclass(frozen=True)
class Resolved:
    name: str
    hosts: Hosts
    custom_hosts: bool


def _problem_with_host(variable: str, value: str) -> str | None:
    if value == '':
        return f'{variable} is set but empty'
    match = _BARE_HOST.fullmatch(value)
    if match is None or (':' in value and int(value.rsplit(':', 1)[1]) > 65535):
        return (f'{variable} must be a bare host name with an optional :port -- no scheme, path or '
                f"credentials -- got '{value}'")
    return None


def resolve(environ: Mapping[str, str]) -> Resolved:
    """The hosts `environ` selects. Raises ValueError naming the variable at fault."""
    name = environ.get(ENVIRONMENT_VARIABLE, DEFAULT_ENVIRONMENT)
    if name not in _TABLE:
        raise ValueError(
            f"{ENVIRONMENT_VARIABLE} must be {', '.join(_TABLE)} or unset, got '{name}'. Matched "
            f'exactly: no trimming, no case folding. Other hosts are reached with '
            f'{DATA_HOST_VARIABLE} and {API_HOST_VARIABLE}, set together.')
    data_host = environ.get(DATA_HOST_VARIABLE)
    api_host = environ.get(API_HOST_VARIABLE)
    if (data_host is None) != (api_host is None):
        set_name, unset_name = ((DATA_HOST_VARIABLE, API_HOST_VARIABLE) if data_host is not None
                                else (API_HOST_VARIABLE, DATA_HOST_VARIABLE))
        raise ValueError(f'{set_name} is set but {unset_name} is not. The two hosts are one '
                         'decision: set both, or neither.')
    if data_host is None:
        return Resolved(name=name, hosts=_TABLE[name], custom_hosts=False)
    if ENVIRONMENT_VARIABLE in environ:
        raise ValueError(f'{ENVIRONMENT_VARIABLE} and {DATA_HOST_VARIABLE} / {API_HOST_VARIABLE} '
                         'are both set; name the hosts one way.')
    for variable, value in ((DATA_HOST_VARIABLE, data_host), (API_HOST_VARIABLE, api_host)):
        problem = _problem_with_host(variable, value)
        if problem is not None:
            raise ValueError(problem)
    return Resolved(name=CUSTOM_ENVIRONMENT, hosts=Hosts(data_host, api_host), custom_hosts=True)


def resolve_process() -> Resolved:
    """`resolve` over the process environment; exits 1 naming the variable at fault."""
    try:
        return resolve(os.environ)
    except ValueError as error:
        raise SystemExit(f'FAIL {error}') from error


def auth_header(org_key: str) -> str:
    """Authorization header value for an organization key -- the only kind this package builds."""
    return org_key if org_key.startswith('orgkey-') else f'APIKEY:ORGANIZATION {org_key}'


@dataclasses.dataclass(frozen=True)
class C3dEnvironment:
    environment: str
    project_id: int
    # Out of the repr, so printing the environment, or an exception or a debugger showing it,
    # cannot write the key out.
    org_key: str = dataclasses.field(repr=False)
    hosts: Hosts

    @property
    def api_host(self) -> str:
        """The analytics API base URL."""
        return f'https://{self.hosts.api}'


def require_environment() -> C3dEnvironment:
    """The process environment, validated; exits 1 naming the first missing or invalid variable."""
    resolved = resolve_process()
    for variable in REQUIRED:
        if not os.environ.get(variable):
            raise SystemExit(f'FAIL {variable}: not set in the process environment')
    project_id_raw = os.environ['C3D_PROJECT_ID']
    if not project_id_raw.isdigit():
        raise SystemExit('FAIL C3D_PROJECT_ID: must be an integer')
    return C3dEnvironment(environment=resolved.name, project_id=int(project_id_raw),
                          org_key=os.environ['C3D_ORG_API_KEY'], hosts=resolved.hosts)
