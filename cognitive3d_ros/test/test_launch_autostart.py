"""`autostart:=true` configures and then activates the node; without it the node stays unconfigured.

Run with the built `cognitive3d_node` as the only argument. Each case runs the shipped launch
description with that executable and reads the node's state from its own `get_state` service, so
what is checked is the state the node reports, not the actions the launch file builds.
"""

import importlib.util
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest

import rclpy
from launch import LaunchContext, LaunchDescriptionSource, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

_EXECUTABLE = sys.argv.pop(1) if len(sys.argv) > 1 else ""

_LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "launch", "cognitive3d.launch.py"
)

# Long enough for a configure and an activate that were going to happen to have happened.
_SETTLE_S = 4.0


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("cognitive3d_launch", _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Probe:
    """A client of the launched node's lifecycle services, used from the probe thread."""

    def __init__(self):
        self.node = rclpy.create_node("autostart_probe")
        self.get_state = self.node.create_client(GetState, "/cognitive3d/get_state")
        self.change_state = self.node.create_client(ChangeState, "/cognitive3d/change_state")

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        return future.result()

    def state(self):
        response = self.call(self.get_state, GetState.Request())
        return response.current_state.label if response else "no answer"

    def transition(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self.call(self.change_state, request)
        return bool(response and response.success)

    def wait_for(self, label, timeout_s):
        deadline = time.monotonic() + timeout_s
        state = self.state()
        while state != label and time.monotonic() < deadline:
            time.sleep(0.2)
            state = self.state()
        return state

    def destroy(self):
        self.node.destroy_node()


class Autostart(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.launch = _load_launch_module()
        self.launch._entry_point = lambda: (None, _EXECUTABLE)
        self.dir = tempfile.mkdtemp(prefix="c3d_autostart_")
        self.params = os.path.join(self.dir, "params.yaml")
        # Everything configure needs, and nothing that reaches a network: no session is started
        # on activate and no remote variables are fetched, and the gateway is a closed port.
        with open(self.params, "w") as out:
            out.write(
                "/**:\n  ros__parameters:\n"
                '    scene_id: "a1b2c3d4-0000-4000-8000-000000000001"\n'
                '    scene_version: "1"\n    device_id: "robot"\n'
                f'    spool_dir: "{os.path.join(self.dir, "spool")}"\n'
                '    protocol: "http"\n    gateway: "127.0.0.1:9"\n'
                '    pose_convention: "unity"\n    remote_variables: false\n'
                "    auto_start_session: false\n"
            )
        os.environ["C3D_APPLICATION_API_KEY"] = "AUTOSTARTTESTKEY"

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def run_launch(self, arguments, probe_body):
        """Runs the launch in this (the main) thread and `probe_body(probe)` beside it.

        The probe body's return value is this method's. The launch is shut down when the body
        returns, or after a minute if the node never offers its services.
        """
        service = LaunchService(argv=[], noninteractive=True)
        service.include_launch_description(
            IncludeLaunchDescription(
                LaunchDescriptionSource(self.launch.generate_launch_description()),
                launch_arguments=[("params_file", self.params)] + arguments,
            )
        )
        outcome = {}

        def probe_thread():
            probe = _Probe()
            try:
                if not probe.get_state.wait_for_service(timeout_sec=60.0):
                    outcome["error"] = "the node never offered its lifecycle services"
                else:
                    outcome["value"] = probe_body(probe)
            except Exception as error:  # noqa: BLE001 -- reported by the test, not swallowed
                outcome["error"] = repr(error)
            finally:
                probe.destroy()
                service.shutdown()

        thread = threading.Thread(target=probe_thread)
        thread.start()
        service.run()
        thread.join()
        if "error" in outcome:
            self.fail(outcome["error"])
        return outcome["value"]

    def test_autostart_true_reaches_active(self):
        state = self.run_launch(
            [("autostart", "true")], lambda probe: probe.wait_for("active", 30.0)
        )
        self.assertEqual(state, "active")

    def test_without_autostart_the_node_stays_unconfigured(self):
        def body(probe):
            time.sleep(_SETTLE_S)
            return probe.state()

        self.assertEqual(self.run_launch([], body), "unconfigured")

    def test_only_the_launchs_own_configure_is_followed_by_an_activate(self):
        def body(probe):
            states = [probe.wait_for("active", 30.0)]
            states.append(probe.transition(Transition.TRANSITION_DEACTIVATE))
            states.append(probe.transition(Transition.TRANSITION_CLEANUP))
            states.append(probe.transition(Transition.TRANSITION_CONFIGURE))
            time.sleep(_SETTLE_S)
            states.append(probe.state())
            return states

        self.assertEqual(
            self.run_launch([("autostart", "true")], body),
            ["active", True, True, True, "inactive"],
        )


class AutostartArgument(unittest.TestCase):
    def setUp(self):
        self.launch = _load_launch_module()

    def test_the_default_is_false(self):
        description = self.launch.generate_launch_description()
        declared = {
            entity.name: entity
            for entity in description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("autostart", declared)
        performed = "".join(
            part.perform(LaunchContext()) for part in declared["autostart"].default_value
        )
        self.assertEqual(performed, "false")

    def test_a_value_that_is_not_a_boolean_is_refused_by_name(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as params:
            params.write("/**:\n  ros__parameters: {}\n")
        try:
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "params_file": params.name,
                    "use_sim_time": "",
                    "log_level": "info",
                    "autostart": "ture",
                }
            )
            with self.assertRaises(RuntimeError) as raised:
                self.launch._launch_node(context)
            self.assertIn("autostart", str(raised.exception))
            self.assertIn("'ture'", str(raised.exception))
        finally:
            os.unlink(params.name)


if __name__ == "__main__":
    if not _EXECUTABLE:
        sys.exit("usage: test_launch_autostart.py <path to cognitive3d_node>")
    unittest.main()
