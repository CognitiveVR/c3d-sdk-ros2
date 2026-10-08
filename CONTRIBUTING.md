# Contributing

Thank you for helping improve the Cognitive3D ROS 2 SDK. This page covers how the repository is
built and tested, the rules a change has to follow, and how a release is cut.

For help integrating the SDK, use [Discord](https://discord.gg/x38sNUdDRH) or
`support@cognitive3d.com` rather than an issue. To report a vulnerability, follow
[SECURITY.md](SECURITY.md) and do not open a public issue.

By contributing, you agree that your contribution is a modification under the terms of
[LICENSE](LICENSE), including its assignment of rights in modifications to Cognitive3D.

## Layout

```text
core/                  the ingest core: sessions, serialization, the spool, uploads. No ROS. Plain CMake.
core/capnp/            vendored Cap'n Proto schemas; never edited by hand (core/capnp/README.md)
cognitive3d_ros/       the ament package: the lifecycle node, translators, QoS, configuration, doctor
cognitive3d_ros_msgs/  the StartSession and EndSession services and the Event message
cognitive3d_ros_nav2/  optional: the Nav2 behavior tree translator and its node entry point
tools/                 test harnesses, the onboarding tools, the session verifiers, release tooling
docker/                the test image every build and test runs in
docs/                  the integrator documentation
```

## The core/ rule

`core/` never includes a ROS header and never links a ROS target. It builds and tests on its
own with plain CMake and CTest, so it can ship to a consumer that has no ROS. If you add a
dependency to `core/`, it must be one you could ship to such a consumer.

This is easy to break by accident. The ROS test image exports `AMENT_PREFIX_PATH` and puts
`/opt/ros/<distro>/bin` on `PATH`, and CMake derives a package search prefix from every `PATH`
entry ending in `/bin`, so `find_package(rclcpp REQUIRED)` inside `core/` configures
successfully in a shell that never sourced `setup.bash`. `tools/build_and_test.sh` therefore
builds `core/` under `env -i` with a `PATH` that contains no ROS, and CI runs that build as the
`core (no ROS)` job.

## Building and testing

Every build and test runs in a container, so a host with Docker and bash runs the whole suite
without a ROS installation:

```bash
tools/build_and_test.sh              # everything
tools/build_and_test.sh core         # core only: fast, no ROS packages
tools/build_and_test.sh ros          # the ament packages, on Jazzy
tools/build_and_test.sh synthetic    # the capture tier
tools/build_and_test.sh humble       # core and the ament packages, on Humble
```

Every selector but `humble` runs on Jazzy. Humble's APIs differ from Jazzy's in ways a Jazzy
build cannot see, so run `humble` before a change that touches the ament packages is merged.
The `humble` selector does not run the capture tier.

The image is built from `docker/Dockerfile.test` on first use. Its tag is a hash of the
Dockerfile and every `package.xml`, so editing a manifest rebuilds the image on its own;
`REBUILD_IMAGE=1` forces a rebuild. The image resolves its dependencies with `rosdep install`
from the manifests, so an undeclared dependency fails the image build rather than a user's
clean checkout. Builds go to container-local paths and the source is mounted read-only:
nothing lands in your tree.

### The test tiers

[tools/README.md](tools/README.md) describes each tier and its tools in detail.

| tier | what it covers | how to run it |
| -- | -- | -- |
| unit tests | the conversions, wire conformance, spool recovery, translators, doctor checks | `tools/build_and_test.sh core`, `ros`, `humble` |
| Python suites | the tools in `tools/` | `for f in tools/test_*.py; do python3 "$f"; done` |
| capture tier | a ROS graph, the node and a stub gateway; asserts on the decoded payloads | `tools/build_and_test.sh synthetic` |

The capture tier is the only tier that decodes a payload. The generated Cap'n Proto readers are
private to `core/`, so that a consumer needs no Cap'n Proto, which also means no ament test can
read what reaches the wire. Anything true only on the wire is checked there or nowhere. To keep
the captured payloads for inspection:

```bash
CAPTURE=/tmp/c3d_capture bash tools/run_synthetic_integration.sh
```

### CI

Every pull request runs `core (no ROS)`, `ament packages`, `ament packages (Humble)`,
`synthetic integration`, `python tools (3.10)`, `python tools (3.12)` and `markdownlint`. Pushes to
`main` also run the arm64 builds and a dry run of the deb build. A pull request is merged only when
every job passes.

## Writing tests

Mutation-check every test you add or change. Break the code the test is meant to protect, run
the test and confirm it fails, then restore the code. A test that only asserts the
implementation back to itself passes against a broken implementation, and reading the test
does not reveal that; breaking the code does. Say in the pull request which change you made and
which test failed.

Comment-only changes need no mutation check.

## Conventions

- Comments state durable constraints, never history. No ticket ids, dates, review outcomes or
  "we tried X first". That belongs in the commit message or the pull request. Comments in this
  repository are dense where they carry a reason a reader would otherwise undo: match the
  density of the file you are editing.
- Units are spelled out in wire and property names: `meters_per_second`, not `m/s`
  (see `cognitive3d_ros_msgs/msg/Event.msg`). ROS parameter names keep their established suffix
  form, such as `sample_rate_hz`, `spool_max_mb`, `reference_frame_grace_s` and `mesh_yaw_offset_deg`. They
  are public surface: never rename one.
- Python in `tools/` uses the standard library only, so an operator verifying a session never
  needs a virtual environment, and CI runs the suites with no dependency step. `uv run` is the
  documented invocation. `capnp_dump.py` is the one exception: it declares its dependencies
  inline (PEP 723), and its test skips the parts that need them when they are absent.
- Never edit `core/capnp/*.capnp`. They are copies of the platform's schemas, pinned by
  `core/capnp/CHECKSUMS`; a schema change starts with Cognitive3D.
- Markdown has a blank line after every heading, and every fenced block names its language. CI
  lints every tracked `*.md` with the repository's `.markdownlint.json`; run the same check
  locally with `npx --yes markdownlint-cli@0.49.1 <files>`.
- Never commit a credential, and never write one into a test, a log line or a document. Refer
  to a key by its environment variable, or by its last four characters as `doctor` prints them
  (`ending ...wxyz`). The node reads its application key from `C3D_APPLICATION_API_KEY`, never
  from a ROS parameter.

## Versions and releases

`kSdkVersion` in `core/include/c3d/version.hpp` is authoritative: it is what reaches the wire as
`c3d.version`. Every `package.xml` declares the same version, and `core/CMakeLists.txt` derives
its project version from the header. Change them together with:

```bash
tools/bump_version.sh 1.1.0
tools/bump_version.sh --check 1.1.0
```

`bump_version.sh` changes no documentation, and the docs name the release in places that pin a
reader to it. Update these in the same pull request as the bump:

- `docs/install.md`: `SDK_VERSION=`, the `--branch v<x.y.z>` clone, the example deb file name and
  the example startup line.
- `docs/doctor-and-verification.md`: the version in the sample output's first line.
- `README.md` and `.claude/skills/README.md`: the `--branch v<x.y.z>` clone of the skill.
- `.claude/skills/integrate-cognitive3d-ros2/SKILL.md` and `params.template.yaml`: the version
  the skill says it targets, the version it checks for, and every `/blob/v<x.y.z>/` link.
- `CHANGELOG.md`: the release link at the foot of the file.

`git grep -n -F '<old-minor>' -- README.md docs .claude/skills`, with the old minor version such as
`1.0`, finds them. Leave the version where it is history: every `CHANGELOG.md` entry.

The order is always bump, merge, dry run, then tag:

1. In the bump pull request, also turn `## [Unreleased]` in `CHANGELOG.md` into the release's
   section, dated: `## [1.1.0] - 2026-11-02`. Add a new, empty `## [Unreleased]` above it.
2. Once it has merged, dry-run the release on `main`:
   `gh workflow run release.yml --ref main -f version=1.1.0`. The dry run makes every check the
   release makes but the date, so it can run before the heading is dated, and builds and
   smoke-tests every deb, arm64 included. It creates no release.
3. Tag the commit the dry run passed on, `v1.1.0`, and push the tag. The release workflow runs
   again on the tag and leaves a draft release for a maintainer to publish.

Both runs refuse a tree where any declaration names another version, where the version's
`CHANGELOG.md` section is missing or holds an HTML comment, or where a doc pins another version.
The tag also refuses a heading that is not dated `YYYY-MM-DD`. Tags are `v<x.y.z>` only: there
are no pre-release tags, and the dry run takes their place. A tag pushed on a tree that fails
these checks must be deleted. The release job also refuses to run when the tag already has a
release: delete the draft, then re-run.

The SDK follows semantic versioning: a breaking change is a major release, an addition a minor
release and a fix a patch release. A surface the docs mark as provisional may change in a minor
release. Every user-visible change adds an entry under `## [Unreleased]` at the top of
`CHANGELOG.md`, and one that breaks a configuration, provisional or not, says how to migrate.

## Pull requests

- Branch from `main` and open the pull request against `main`.
- Keep commits small and logical. Commit messages say what is true after the change.
- Fill in the pull request template: the mutation checks, the docs you updated and the
  `CHANGELOG.md` entry.
