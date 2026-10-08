"""The launch file requires `params_file` and refuses one that is not a file.

There is no default file: the reference file documents every parameter and is not one to run
on. The refusal names the file to start from.
"""

import importlib.util
import os
import tempfile
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode

_LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "launch", "cognitive3d.launch.py"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("cognitive3d_launch", _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ParamsFileIsRequired(unittest.TestCase):
    def setUp(self):
        self.launch = _load_launch_module()

    def context(self, params_file):
        context = LaunchContext()
        context.launch_configurations["params_file"] = params_file
        context.launch_configurations["use_sim_time"] = ""
        context.launch_configurations["log_level"] = "info"
        context.launch_configurations["autostart"] = "false"
        return context

    def test_the_argument_has_no_default_file(self):
        description = self.launch.generate_launch_description()
        declared = {
            entity.name: entity
            for entity in description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("params_file", declared)
        default = declared["params_file"].default_value
        performed = "".join(part.perform(self.context("")) for part in default or [])
        self.assertEqual(performed, "")

    def test_unset_is_refused_and_names_the_file_to_start_from(self):
        with self.assertRaises(RuntimeError) as raised:
            self.launch._launch_node(self.context(""))
        self.assertIn("params_file", str(raised.exception))
        self.assertIn("params.minimal.yaml", str(raised.exception))

    def test_a_path_that_is_not_a_file_is_refused_by_name(self):
        missing = os.path.join(tempfile.gettempdir(), "c3d-no-such-params-file.yaml")
        with self.assertRaises(RuntimeError) as raised:
            self.launch._launch_node(self.context(missing))
        self.assertIn(missing, str(raised.exception))

    def test_an_existing_file_reaches_the_node(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as params:
            params.write("/**:\n  ros__parameters: {}\n")
        try:
            actions = self.launch._launch_node(self.context(params.name))
            self.assertEqual(len(actions), 1)
            self.assertIsInstance(actions[0], LifecycleNode)
        finally:
            os.unlink(params.name)


if __name__ == "__main__":
    unittest.main()
