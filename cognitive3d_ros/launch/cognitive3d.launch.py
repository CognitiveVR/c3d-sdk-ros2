"""Launch cognitive3d_ros as a managed lifecycle node.

The node comes up unconfigured. Configure and activate it explicitly, either
with a lifecycle manager or by hand:

    ros2 lifecycle set /cognitive3d configure
    ros2 lifecycle set /cognitive3d activate

or pass `autostart:=true`, which configures the node as soon as it starts and
activates it once configure succeeds -- for a systemd unit or a container with
no lifecycle manager. Only the launch's own configure is followed by an
activate; after that the lifecycle is the operator's, so a later manual
configure leaves the node inactive. A configure that fails leaves the node
unconfigured, with the reason in its log.

`params_file` is required: start from the package's config/params.minimal.yaml.

Deactivating destroys every subscription, the tf listener and `~/events`
included, so the node stops receiving topic traffic entirely rather than merely
ignoring it. That is the kill switch a safety review asks about.
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


_MINIMAL_PARAMS = (
    "$(ros2 pkg prefix cognitive3d_ros)/share/cognitive3d_ros/config/params.minimal.yaml"
)

_TRUE_VALUES = ("true", "1", "yes", "on")
_FALSE_VALUES = ("false", "0", "no", "off")


def _entry_point():
    """The package and executable to run, which depends on what is installed.

    The Nav2 translators need `nav2_msgs`, and a robot that does not run Nav2 must not be made to
    install it, so they live in their own package and the composition happens THERE: the node is
    the same one either way, and `cognitive3d_ros_nav2` ships an entry point that adds them before
    configure. Choosing it here is what keeps that from being something an operator has to know --
    and the node names the message types it ended up with on every startup, so the choice is
    visible in the log rather than inferred.
    """
    try:
        get_package_share_directory("cognitive3d_ros_nav2")
    except PackageNotFoundError:
        return "cognitive3d_ros", "cognitive3d_node"
    return "cognitive3d_ros_nav2", "cognitive3d_node"


def _boolean(name, value):
    lowered = value.lower()
    if lowered in _TRUE_VALUES:
        return True
    if lowered in _FALSE_VALUES:
        return False
    raise RuntimeError(f"{name} must be true or false, got {value!r}")


def _change_state(node, transition_id):
    return EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node), transition_id=transition_id
        )
    )


def _launch_node(context):
    """Build the node action, with use_sim_time overriding only when given.

    The parameters list is ordered, and a later entry wins. Appending
    {"use_sim_time": ...} unconditionally would make the parameter DEAD in every
    params file, because an unspecified launch argument would still supply its
    own default and beat the file. That is the one parameter this SDK documents
    as silently fatal to get wrong under simulation, so it is the worst one to
    quietly ignore.
    """
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    if not params_file:
        raise RuntimeError(
            "params_file:=<path> is required. Start from a copy of " + _MINIMAL_PARAMS
        )
    # Checked here because a missing file otherwise reaches the node as an rcl
    # argument error and an exit code, with nothing naming the path.
    if not os.path.isfile(params_file):
        raise RuntimeError(f"params_file {params_file!r} is not a file")

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip()

    parameters = [params_file]
    if use_sim_time:
        # Refuse rather than coerce. A typo silently meaning "false" is how
        # a simulation run ends up putting near-zero ROS time on the wire,
        # which the platform accepts with a 200 and then never shows.
        parameters.append({"use_sim_time": _boolean("use_sim_time", use_sim_time)})

    autostart = _boolean(
        "autostart", LaunchConfiguration("autostart").perform(context).strip()
    )

    package, executable = _entry_point()
    node = LifecycleNode(
        package=package,
        executable=executable,
        name="cognitive3d",
        namespace="",
        output="screen",
        parameters=parameters,
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )
    if not autostart:
        return [node]
    # The handler is registered before the configure is emitted, and fires once:
    # an activate that followed every configure would also follow an operator's
    # manual cleanup and reconfigure.
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[_change_state(node, Transition.TRANSITION_ACTIVATE)],
            handle_once=True,
        )
    )
    return [
        node,
        activate_after_configure,
        _change_state(node, Transition.TRANSITION_CONFIGURE),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description=(
                    "Path to the cognitive3d_ros parameter file. Required; start from "
                    + _MINIMAL_PARAMS
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="",
                description=(
                    "Set true under Gazebo. ROS time never reaches the wire: "
                    "Cognitive3D timestamps are wall clock, or with the "
                    "sim_time_on_wire parameter, wall clock at the session start "
                    "advanced by ROS time. Leave UNSET to take the value from "
                    "params_file; the parameter file's own default is false."
                ),
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="false",
                description=(
                    "Set true to configure the node as soon as it starts and "
                    "activate it once configure succeeds, for systemd and "
                    "containers. Leave false when a lifecycle manager or an "
                    "operator drives the transitions."
                ),
            ),
            DeclareLaunchArgument(
                "log_level", default_value="info", description="rclcpp log level."
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )
