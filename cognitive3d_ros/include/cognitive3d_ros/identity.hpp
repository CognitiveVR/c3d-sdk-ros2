#pragma once

#include <string>

#include "c3d/session.hpp"

namespace cognitive3d_ros {

// `$ROS_DISTRO`, or "unknown". An empty variable counts as unset: an absent answer must not read
// as a wrong one, and `ROS_DISTRO=` says no more than no variable at all.
std::string RosDistro();

// `PRETTY_NAME` from os-release, or "unknown". A file read of the host's own declaration, not
// hardware detection -- it infers nothing about the robot, which an integrator names through
// `session_properties`.
//
// The paths and quoting follow os-release(5): /etc/os-release first, /usr/lib/os-release as the
// fallback for a stateless system, and a value may be bare, double-quoted or single-quoted.
std::string HostOperatingSystem(const std::string& primary = "/etc/os-release",
                                const std::string& fallback = "/usr/lib/os-release");

// "ROS 2 <distro>, <rmw>": what a log line or a support report needs beside the SDK version to
// say which build of which stack produced it.
std::string RuntimeDescription();

// `$ROS_DOMAIN_ID` and `$ROS_AUTOMATIC_DISCOVERY_RANGE`, or "unset". These two variables decide
// WHOSE data a session contains: at their defaults, two robots on one network join one ROS graph,
// and every measurement afterwards is the union of both with nothing in the payload to reveal it.
//
// Recorded as a transcript, never resolved to an effective default. An absent `ROS_DOMAIN_ID`
// really does mean domain 0, but reporting it as "0" would be indistinguishable from someone
// having chosen 0 -- and the difference between a value and an unmade decision is the whole
// reason to record these. `doctor` is where the effective default gets explained.
std::string RosDomainId();
std::string RosDiscoveryRange();

// Whether this build's ROS reads `ROS_AUTOMATIC_DISCOVERY_RANGE` at all. Humble does not, so there
// the variable decides nothing and `c3d.ros.discovery_range` is not recorded.
bool DistroHasDiscoveryRange();

// The host identity every producer of a Cognitive3D session sets, in one place so that a tool
// driving `c3d::Session` directly sends the same set the node does.
//
// These are process constants, but they are session-SCOPED on the wire: `Session::End` restores
// the properties to their pre-Begin snapshot, so a session that is closed and reopened has to be
// given them again.
void SetHostIdentityProperties(c3d::Session& session);

}  // namespace cognitive3d_ros
