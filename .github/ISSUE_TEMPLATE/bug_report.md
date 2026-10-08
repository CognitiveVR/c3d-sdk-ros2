---
name: Bug report
about: The SDK does something it should not, or does not do something it should
title: ''
labels: bug
assignees: ''
---

<!--
Never paste a key. doctor masks every key it prints; check anything else you paste.
For integration questions, use Discord: https://discord.gg/x38sNUdDRH
For a security vulnerability, do not open an issue: see SECURITY.md.
-->

## What happened

What you did, what you expected, and what happened instead.

## doctor output

The full output of doctor, run with the params file the node uses:

```bash
ros2 run cognitive3d_ros doctor --ros-args --params-file <path/to/params.yaml>
```

```text
paste here
```

## Environment

- SDK version (`ros2 pkg xml -t version cognitive3d_ros`):
- ROS 2 distro (Humble, Jazzy):
- RMW implementation (`echo $RMW_IMPLEMENTATION`, or the default):
- How the SDK was installed (debs from a GitHub release, built from source, other):
- OS and architecture (for example Ubuntu 22.04, arm64):
- Running in a container (yes or no):

## Configuration

Your params file, with any key removed:

```yaml
paste here
```

## Logs

The node's log around the problem, with any key removed:

```text
paste here
```
