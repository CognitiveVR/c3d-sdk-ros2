# Agent skills

This directory holds one agent skill, for AI coding agents that integrate the SDK into a robot.

| skill | use it for |
| -- | -- |
| [`integrate-cognitive3d-ros2`](integrate-cognitive3d-ros2/SKILL.md) | Adding the Cognitive3D SDK for ROS 2 to a robot that already works: survey, decisions, params file, launch, `doctor`, the first session and a hand-off report |

The skill is `SKILL.md` plus `params.template.yaml`, the params file it fills in. It links to the
documentation of the SDK version it targets, and it asks the agent to check the installed version
first. It comes only from this repository: the Debian packages do not install it, so fetch it from
GitHub even on a robot that runs the debs.

## Install it with Claude Code

Copy the skill directory, from a checkout of this repository at the tag of the SDK version you run
or from the source archive on that version's GitHub Release page, into your own repository's
`.claude/skills/`. Committed there, it is shared with your team and stays pinned to that version:

```bash
git clone --depth 1 --branch v1.1.0 https://github.com/CognitiveVR/c3d-sdk-ros2.git /tmp/c3d-sdk-ros2
mkdir -p .claude/skills
cp -r /tmp/c3d-sdk-ros2/.claude/skills/integrate-cognitive3d-ros2 .claude/skills/
```

To use it in every project of your own instead, copy it into `~/.claude/skills/`. A copy is a
snapshot: copy it again when you upgrade the SDK.

## Use it with another agent

`SKILL.md` is plain Markdown with shell commands and names no agent-specific tool. Point your agent
at the file, for example with a line in your repository's `AGENTS.md`, or paste it into the agent's
context, and keep `params.template.yaml` beside it.

AI-assisted integration is experimental. Review the params file the agent writes and the `doctor`
output it reports, and check the first session in Session Replay yourself.
