"""`doctor` checks the build of the node the launch file runs, and its report names which.

The launch file runs cognitive3d_ros_nav2's node whenever that package is on the ament index, so
`doctor` becomes that package's doctor then, with the same arguments. Here a prefix of this test's
own registers the package, and its "doctor" is a script that reports what it was given.

Run with the built `doctor` as the only argument.
"""

import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

_EXECUTABLE = sys.argv.pop(1) if len(sys.argv) > 1 else ""
_NAV2 = "cognitive3d_ros_nav2"


def without_nav2(prefix_path):
    """AMENT_PREFIX_PATH with every prefix that registers cognitive3d_ros_nav2 removed."""
    marker = pathlib.Path("share", "ament_index", "resource_index", "packages", _NAV2)
    return os.pathsep.join(p for p in prefix_path.split(os.pathsep)
                           if p and not (pathlib.Path(p) / marker).exists())


class DoctorBuild(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.prefix = pathlib.Path(self.directory.name)
        self.base_path = without_nav2(os.environ.get("AMENT_PREFIX_PATH", ""))

    def tearDown(self):
        self.directory.cleanup()

    def register_nav2(self, doctor_script=None):
        marker = self.prefix / "share" / "ament_index" / "resource_index" / "packages" / _NAV2
        marker.parent.mkdir(parents=True)
        marker.write_text("")
        if doctor_script is not None:
            doctor = self.prefix / "lib" / _NAV2 / "doctor"
            doctor.parent.mkdir(parents=True)
            doctor.write_text(doctor_script)
            doctor.chmod(0o755)

    def run_doctor(self, *arguments, registered=False):
        environment = dict(os.environ)
        environment["AMENT_PREFIX_PATH"] = (
            os.pathsep.join([str(self.prefix), self.base_path]) if registered else self.base_path)
        return subprocess.run([_EXECUTABLE, *arguments], capture_output=True, text=True,
                              timeout=120, env=environment)

    def test_with_the_nav2_package_installed_its_doctor_runs_with_the_same_arguments(self):
        self.register_nav2('#!/bin/sh\necho "nav2 doctor: $0 $*"\nexit 7\n')
        result = self.run_doctor("--ros-args", "-p", "scene_version:='2'", registered=True)
        self.assertEqual(result.returncode, 7, result.stdout + result.stderr)
        self.assertEqual(
            result.stdout.strip(),
            f"nav2 doctor: {self.prefix / 'lib' / _NAV2 / 'doctor'} "
            "--ros-args -p scene_version:='2'")

    def test_without_it_the_report_names_the_plain_build(self):
        result = self.run_doctor()
        self.assertEqual(result.returncode, 1, "nothing configures without parameters")
        build = [line for line in result.stdout.splitlines() if "] build " in line]
        self.assertEqual(len(build), 1, result.stdout)
        self.assertIn("[ok  ] build", build[0])
        self.assertIn("cognitive3d_ros's node, the one the launch file runs while "
                      "cognitive3d_ros_nav2 is not installed", build[0])
        self.assertIn("sensor_msgs/msg/BatteryState", build[0])
        self.assertNotIn("nav2_msgs", build[0])

    def test_a_nav2_package_without_a_doctor_is_named_beside_the_plain_report(self):
        self.register_nav2()
        result = self.run_doctor(registered=True)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        build = [line for line in result.stdout.splitlines() if "] build " in line]
        self.assertEqual(len(build), 1, result.stdout)
        self.assertIn("[note] build", build[0])
        self.assertIn("that package's doctor did not start", build[0])
        self.assertIn(str(self.prefix / "lib" / _NAV2 / "doctor"), build[0])
        self.assertIn("Install cognitive3d_ros_nav2 at the same version", result.stdout)

    def test_an_unreadable_index_is_named_in_the_report_rather_than_aborting_it(self):
        environment = {k: v for k, v in os.environ.items() if k != "AMENT_PREFIX_PATH"}
        result = subprocess.run([_EXECUTABLE], capture_output=True, text=True, timeout=120,
                                env=environment)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        build = [line for line in result.stdout.splitlines() if "] build " in line]
        self.assertEqual(len(build), 1, result.stdout + result.stderr)
        self.assertIn("[note] build", build[0])
        self.assertIn("which build the launch file runs is unknown", build[0])
        self.assertIn("AMENT_PREFIX_PATH", build[0])


if __name__ == "__main__":
    if not _EXECUTABLE:
        sys.exit("usage: test_doctor_build.py <path to doctor>")
    unittest.main()
