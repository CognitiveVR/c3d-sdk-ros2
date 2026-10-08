"""`ros2 run cognitive3d_ros doctor` reports on this package's node once this package is installed.

The launch file runs this package's node whenever it is installed, and only this build translates
nav2_msgs/msg/BehaviorTreeLog, so a report on the plain node would say nothing about it. This runs
cognitive3d_ros's installed doctor with this package's install prefix on the ament index.

Run with the path of cognitive3d_ros's installed `doctor` as the only argument.
"""

import subprocess
import sys
import unittest

_EXECUTABLE = sys.argv.pop(1) if len(sys.argv) > 1 else ""


class Handoff(unittest.TestCase):
    def test_the_report_names_this_build_and_its_nav2_translator(self):
        result = subprocess.run([_EXECUTABLE], capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode, 1, "nothing configures without parameters")
        build = [line for line in result.stdout.splitlines() if "] build " in line]
        self.assertEqual(len(build), 1, result.stdout + result.stderr)
        self.assertIn("[ok  ] build", build[0])
        self.assertIn("cognitive3d_ros_nav2's node, the one the launch file runs", build[0])
        self.assertIn("nav2_msgs/msg/BehaviorTreeLog", build[0])


if __name__ == "__main__":
    if not _EXECUTABLE:
        sys.exit("usage: test_doctor_handoff.py <path to cognitive3d_ros's doctor>")
    unittest.main()
