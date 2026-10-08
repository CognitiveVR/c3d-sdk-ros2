"""`doctor` on a configuration that gets past configure, against a graph and an API of this test's own.

What only a whole run can show: that the scene lookup goes to the api host the process environment
names, and only when an organisation key may be sent; that topics/qos reads every publisher of an
allowlisted topic; that a described sensor's topic is checked against the type its kind reads;
that nothing doctor or the node it builds prints carries more of the application
key than its last four characters, once; that a spool directory doctor creates is the one the node
would have created; and that doctor's write probe never writes through a link planted under its
name. The api host is a loopback listener that speaks no TLS, so the lookup fails once it has
connected, and the connection is the evidence.

Run with the built `doctor` as the only argument.
"""

import os
import pathlib
import re
import socket
import stat
import subprocess
import sys
import tempfile
import threading
import unittest

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_EXECUTABLE = sys.argv.pop(1) if len(sys.argv) > 1 else ""
_NAV2 = "cognitive3d_ros_nav2"
_TOPIC = "/doctor_report/joint_states"
# Allowlisted, and published by nothing.
_SILENT = "/doctor_report/line"
_SCENE = "a1b2c3d4-0000-4000-8000-000000000001"
# Made of runs that no English word, check name or log line contains.
_KEY = "Qx7Zk2Wv9Bj4Mn8Pr3Ty6Lg5"
_PROBE = ".c3d-doctor-write-probe"
_VICTIM_TEXT = "not doctor's to write\n"


def without_nav2(prefix_path):
    """AMENT_PREFIX_PATH with every prefix that registers cognitive3d_ros_nav2 removed."""
    marker = pathlib.Path("share", "ament_index", "resource_index", "packages", _NAV2)
    return os.pathsep.join(p for p in prefix_path.split(os.pathsep)
                           if p and not (pathlib.Path(p) / marker).exists())


def closed_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class Listener:
    """Accepts every connection on a loopback port, keeps the first bytes each one sends, and hangs
    up with the rest unread, which resets the connection in the middle of the TLS handshake. The
    doctor must report that, not die of it: on libcurl 7 the client's next write to the socket
    raises SIGPIPE."""

    def __init__(self):
        self.received = []
        self._socket = socket.socket()
        self._socket.bind(("127.0.0.1", 0))
        self._socket.listen()
        self.port = self._socket.getsockname()[1]
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        while True:
            try:
                connection, _ = self._socket.accept()
            except OSError:
                return
            with connection:
                connection.settimeout(5)
                try:
                    self.received.append(connection.recv(5))
                except OSError:
                    self.received.append(b"")

    def close(self):
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._socket.close()


def checks_in(report):
    """Each report line's check name, mapped to its label and detail."""
    checks = {}
    for line in report.splitlines():
        match = re.match(r"\s*\[(.{4})\] (.+?)\s{2,}(.*)$", line)
        if match:
            checks[match.group(2)] = (match.group(1), match.group(3))
    return checks


def run_doctor(directory, data_host, api_port, protocol_lines):
    params = pathlib.Path(directory, "params.yaml")
    params.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        f'    scene_id: "{_SCENE}"\n'
        '    scene_version: "1"\n'
        '    device_id: "doctor-report"\n'
        '    pose_convention: "gltf_authored"\n'
        f'    spool_dir: "{directory}/state/cognitive3d"\n'
        f'    topics: ["{_TOPIC}", "{_SILENT}"]\n'
        '    sensors: ["misdescribed", "silent"]\n'
        '    sensor:\n'
        '      misdescribed:\n'
        '        kind: "cone_rangefinder"\n'
        f'        topic: "{_TOPIC}"\n'
        '        units: "m"\n'
        '        cone_half_angle_deg: 7.5\n'
        '      silent:\n'
        '        kind: "binary_array"\n'
        f'        topic: "{_SILENT}"\n'
        '        element_count: 4\n' + protocol_lines)
    environment = {k: v for k, v in os.environ.items() if not k.startswith("C3D_")}
    environment.update({
        "AMENT_PREFIX_PATH": without_nav2(os.environ.get("AMENT_PREFIX_PATH", "")),
        "C3D_APPLICATION_API_KEY": _KEY,
        "C3D_ORG_API_KEY": "orgkey-doctor-report-not-a-key",
        "C3D_DATA_HOST": data_host,
        "C3D_API_HOST": f"127.0.0.1:{api_port}",
    })
    result = subprocess.run([_EXECUTABLE, "--ros-args", "--params-file", str(params)],
                            capture_output=True, text=True, timeout=120, env=environment)
    return result.stdout + result.stderr, result.returncode


def plant_probe_link(spool):
    """An existing spool directory holding a link, under the name of doctor's write probe, to a file
    outside it. Returns the file."""
    spool.mkdir(mode=0o700, parents=True)
    victim = spool.parent.parent / "victim.txt"
    victim.write_text(_VICTIM_TEXT)
    (spool / _PROBE).symlink_to(victim)
    return victim


class DoctorReport(unittest.TestCase):
    """Two runs, one sending the organisation key and one that must not, beside two publishers of
    one topic that disagree on durability. The first finds its spool directory already there, with
    a link planted under the write probe's name; the second creates it."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.nodes = []
        for name, durability in (("joint_state_broadcaster", DurabilityPolicy.TRANSIENT_LOCAL),
                                 ("joint_state_publisher", DurabilityPolicy.VOLATILE)):
            node = rclpy.create_node(name)
            node.create_publisher(String, _TOPIC, QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=durability))
            cls.nodes.append(node)

        cls.runs = {}
        cls.spool_modes = {}
        cls.planted = {}
        # The default umask, under which a directory made without a mode is 0755.
        umask = os.umask(0o022)
        try:
            # Under http the gateway must name a capture stub, so not the data host's host.
            for protocol, data_host, lines in (
                    ("https", f"127.0.0.1:{closed_port()}", '    protocol: "https"\n'),
                    ("http", f"localhost:{closed_port()}",
                     f'    protocol: "http"\n    gateway: "127.0.0.1:{closed_port()}"\n')):
                api = Listener()
                try:
                    with tempfile.TemporaryDirectory() as directory:
                        spool = pathlib.Path(directory, "state", "cognitive3d")
                        victim = plant_probe_link(spool) if protocol == "https" else None
                        report, returncode = run_doctor(directory, data_host, api.port, lines)
                        cls.spool_modes[protocol] = [
                            stat.S_IMODE(made.stat().st_mode) if made.is_dir() else None
                            for made in (spool.parent, spool)]
                        if victim is not None:
                            cls.planted = {
                                "victim": victim.read_text(),
                                "link": os.readlink(spool / _PROBE)
                                        if (spool / _PROBE).is_symlink() else None,
                                "probes": sorted(name for name in os.listdir(spool)
                                                 if name.startswith(_PROBE)),
                                "target": str(victim),
                            }
                finally:
                    api.close()
                cls.runs[protocol] = (report, checks_in(report), list(api.received), returncode)
        finally:
            os.umask(umask)

    @classmethod
    def tearDownClass(cls):
        for node in cls.nodes:
            node.destroy_node()
        rclpy.shutdown()

    def test_the_scene_is_read_from_the_api_host_the_environment_names(self):
        report, checks, received, _ = self.runs["https"]
        self.assertEqual(len(received), 1, report)
        self.assertEqual(received[0][:1], b"\x16",
                         "a TLS handshake, so the lookup went out over https")
        label, detail = checks.get("scene/project", ("", report))
        self.assertEqual(label, "note", report)
        self.assertTrue(detail.startswith("could not reach the API"), detail)

    def test_an_api_that_resets_the_handshake_does_not_end_the_process(self):
        report, _, received, returncode = self.runs["https"]
        self.assertEqual(len(received), 1, report)
        self.assertGreaterEqual(returncode, 0,
                                f"doctor was killed by signal {-returncode}\n{report}")

    def test_topics_qos_names_every_publisher_of_an_allowlisted_topic(self):
        report, checks, _, _ = self.runs["https"]
        label, detail = checks.get("topics/qos", ("", report))
        self.assertEqual(label, "note", report)
        self.assertIn(f"{_TOPIC}: its publishers disagree on durability", detail)
        self.assertIn("/joint_state_broadcaster reliable/transient_local", detail)
        self.assertIn("/joint_state_publisher reliable/volatile", detail)

    def test_a_sensor_is_checked_against_the_type_its_kind_reads(self):
        report, checks, _, _ = self.runs["https"]
        label, detail = checks.get("sensor/misdescribed", ("", report))
        self.assertEqual(label, "FAIL", report)
        self.assertIn(f"sensor.misdescribed.topic {_TOPIC} is std_msgs/msg/String", detail)
        self.assertIn("std_msgs/msg/Float32", detail)
        label, detail = checks.get("sensor/silent", ("", report))
        self.assertEqual(label, "note", report)
        self.assertIn(f"{_SILENT} has no publisher", detail)

    def test_the_output_carries_the_last_four_of_the_application_key_once_and_nothing_else(self):
        runs = {_KEY[at:at + 4] for at in range(len(_KEY) - 3)}
        for protocol, (report, _, _, _) in self.runs.items():
            carrying = [line for line in report.splitlines() if any(run in line for run in runs)]
            self.assertEqual(len(carrying), 1, f"{protocol}:\n{report}")
            self.assertIn("application key", carrying[0])
            self.assertEqual([run for run in runs if run in carrying[0]], [_KEY[-4:]], carrying[0])

    def test_the_spool_directory_doctor_creates_is_owner_only_with_every_parent_it_creates(self):
        """The node keeps a spool_dir it finds as it is, so one doctor created first is the one the
        node runs on."""
        report, checks, _, _ = self.runs["http"]
        self.assertEqual(checks.get("spool directory", ("", report))[0], "ok  ", report)
        self.assertEqual(self.spool_modes["http"], [0o700, 0o700], "state/ and state/cognitive3d/")

    def test_the_write_probe_writes_through_no_link_and_leaves_what_it_found(self):
        """A link under the probe's name, planted by another account that can write to spool_dir,
        must not have doctor write the file it names, nor remove the link that shows it was
        planted, nor call a writable directory unwritable."""
        report, checks, _, _ = self.runs["https"]
        self.assertEqual(self.planted["victim"], _VICTIM_TEXT, "the link's target was written")
        self.assertEqual(self.planted["link"], self.planted["target"], "the link was removed")
        self.assertEqual(self.planted["probes"], [_PROBE], "a probe of doctor's own was left")
        label, detail = checks.get("spool directory", ("", report))
        self.assertEqual(label, "ok  ", report)
        self.assertTrue(detail.endswith(" is writable"), detail)

    def test_a_loopback_capture_sends_the_organisation_key_nowhere(self):
        report, checks, received, _ = self.runs["http"]
        self.assertIn("scene/project (NOT CHECKED)", checks, report)
        self.assertEqual(received, [], report)


if __name__ == "__main__":
    if not _EXECUTABLE:
        sys.exit("usage: test_doctor_report.py <path to doctor>")
    unittest.main()
