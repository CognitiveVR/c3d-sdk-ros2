"""The shipped executable names the parameter and the fix when its parameters cannot construct the node.

Run with the built `cognitive3d_node` as the only argument.
"""

import os
import subprocess
import sys
import tempfile
import unittest

_EXECUTABLE = sys.argv.pop(1) if len(sys.argv) > 1 else ""


class ConstructionFailure(unittest.TestCase):
    def run_node(self, parameters):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as params:
            params.write("/**:\n  ros__parameters:\n" + parameters)
        try:
            return subprocess.run(
                [_EXECUTABLE, "--ros-args", "--params-file", params.name],
                capture_output=True,
                text=True,
                timeout=60,
            )
        finally:
            os.unlink(params.name)

    def test_an_empty_list_exits_1_naming_the_line_to_delete(self):
        result = self.run_node("    cameras: []\n")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("Delete the `cameras: []` line", result.stderr)
        self.assertNotIn("terminate called", result.stderr)

    def test_a_mistyped_value_exits_1_naming_the_parameter_and_the_type(self):
        result = self.run_node("    scene_version: 1\n")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("scene_version must be a string", result.stderr)
        self.assertIn("Quote the value", result.stderr)


if __name__ == "__main__":
    if not _EXECUTABLE:
        sys.exit("usage: test_node_main.py <path to cognitive3d_node>")
    unittest.main()
