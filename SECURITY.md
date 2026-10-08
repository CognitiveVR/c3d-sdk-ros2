# Security

## Reporting a vulnerability

Email `support@cognitive3d.com` with "Security" in the subject line. Do not open a public issue,
pull request or Discord thread for a vulnerability.

Include what you can of:

- the SDK version (`ros2 pkg xml -t version cognitive3d_ros`), the ROS 2 distro and how the SDK
  was installed;
- what an attacker can do, and what they need first (access to the ROS graph, to the host, to
  the network path to Cognitive3D);
- the steps or a configuration that reproduces it.

Never include a working credential. Refer to a key by its environment variable, or by its last
four characters as `doctor` prints them (`ending ...wxyz`).

We acknowledge each report, keep you informed while we work on it, and agree a disclosure date
with you.

## Supported versions

Fixes are released for the latest 1.x minor version only; upgrade to it to receive them.

| version | receives fixes |
| -- | -- |
| latest 1.x minor | yes |
| older versions | no |

## How the SDK handles credentials and data

These are the properties a security review usually asks about. A report that one of them does
not hold is in scope.

- **The application key comes from the environment, never from a parameter.** The node reads it
  from `C3D_APPLICATION_API_KEY` in its own process environment. A ROS parameter is readable by
  anything on the graph and is echoed into launch logs and parameter dumps, so the node never
  declares `application_key`: a params file that sets it, to any value including an empty one, is
  refused at configure, a `ros2 param set` of it is rejected, and neither refusal repeats the
  value.
- **Hosts come from the environment.** The data and api hosts are resolved from the process
  environment. A params file can override only the data host, with `gateway`, which receives
  the application key; the organization key, which `doctor` and the tools use, goes only to the
  api host the environment resolves. A spooled chunk is sent to a URL rebuilt from the uploading
  process's own configuration, and a chunk recorded for another host is kept and never sent with
  this process's key.
- **Transport is HTTPS.** `protocol: http` is accepted only beside an overridden `gateway`, such
  as a local capture stub, never for a Cognitive3D host.
- **Keys are masked.** `doctor` prints only the application key's last four characters, once,
  and never the organization key; `fetch_application_key.py` lists keys by their last four
  characters unless asked for an environment line. The capture tier's stub gateway records
  whether an `Authorization` header was present, never its value.
- **The node is subscribe-only.** It creates no publisher on any topic it does not own and
  cannot command a robot. Deactivating it destroys every subscription, including the tf listener
  and `~/events`, so a deactivated node receives no topic traffic.
- **The spool is per user.** By default it is a per-user state directory, not a shared one, and
  a spool directory held by another process, or a lock file planted as a link, is refused at
  configure. The spool holds recorded telemetry in plain files: protect it as you would the
  telemetry itself.

## Cap'n Proto on Ubuntu 22.04 (Humble)

Ubuntu 22.04 ships Cap'n Proto 0.8.0, which is affected by
[CVE-2022-46149](https://ubuntu.com/security/CVE-2022-46149), an out-of-bounds read when
**parsing** a crafted message. Ubuntu marks it "Ignored" for 22.04, so a scanner will flag the
`libcapnp` package on a Humble install.

The SDK only **writes** Cap'n Proto messages, for the camera and lidar payloads it uploads. It
never parses one, so the affected code is not reachable through the SDK. Ubuntu 24.04 (Jazzy)
ships Cap'n Proto 1.0.1, which is not affected.
