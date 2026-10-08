#pragma once

namespace c3d {

// Transmitted as the `c3d.version` session property, so the platform can
// attribute every session to the SDK release that recorded it.
constexpr const char* kSdkVersion = "1.0.0";
// `c3d.app.sdktype` names the INTEGRATION, not this library's implementation,
// and never the robot -- an integrator sets hardware identity through
// `session_properties`. `ros` rather than `ros2` so one value outlives a major
// ROS version.
constexpr const char* kSdkType = "ros";
constexpr const char* kAppEngine = "ROS2";

}  // namespace c3d
