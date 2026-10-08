"""A fake NavigateToPose action server that produces the outcomes a simulator won't.

A simulator hands you SUCCEEDED almost every time. The bugs in an objective
translator live in ABORTED, CANCELED, preemption and goals that vanish without
terminating. Those are only reachable by scripting a server.
"""
import sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import BatteryState, JointState

# Scripted outcomes, in order. This is the point of the whole file.
SCRIPT = ["succeed", "abort", "succeed", "succeed"]

class FakeNav(Node):
    def __init__(self):
        super().__init__("fake_nav_server")
        self.cbg = ReentrantCallbackGroup()
        self.index = 0
        self.server = ActionServer(
            self, NavigateToPose, "navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=self.cbg)

        self.battery = self.create_publisher(BatteryState, "/battery_state", 10)
        self.diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.pct = 0.95
        self.tick = 0
        self.create_timer(0.5, self.publish_telemetry, callback_group=self.cbg)

    def publish_telemetry(self):
        self.tick += 1
        self.pct = max(0.0, self.pct - 0.004)
        b = BatteryState()
        b.percentage = float(self.pct)
        b.voltage = 24.0 + self.pct
        b.current = -2.5
        b.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING
                                 if self.tick > 40
                                 else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.battery.publish(b)

        js = JointState()
        js.name = ["left_wheel", "right_wheel"]
        js.position = [self.tick * 0.1, self.tick * 0.1]
        js.velocity = [0.4, 0.42]
        js.effort = [1.0 + (self.tick % 5) * 0.3, 1.1]
        self.joints.publish(js)

        d = DiagnosticArray()
        s = DiagnosticStatus()
        s.name = "motor_driver"
        # Degrade partway through, then recover. Two events, not hundreds.
        if 20 < self.tick < 35:
            s.level = DiagnosticStatus.ERROR
            s.message = "overcurrent on left wheel"
        else:
            s.level = DiagnosticStatus.OK
            s.message = "nominal"
        s.hardware_id = "md-01"
        d.status = [s]
        self.diag.publish(d)

    def execute(self, goal_handle):
        outcome = SCRIPT[self.index % len(SCRIPT)]
        self.index += 1
        self.get_logger().info(f"goal accepted, scripted outcome: {outcome}")
        for i in range(12):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            fb = NavigateToPose.Feedback()
            fb.number_of_recoveries = i // 4
            fb.distance_remaining = float(12 - i)
            goal_handle.publish_feedback(fb)
            time.sleep(0.25)
        if outcome == "abort":
            goal_handle.abort()
            self.get_logger().info("goal aborted")
        else:
            goal_handle.succeed()
            self.get_logger().info("goal succeeded")
        return NavigateToPose.Result()

rclpy.init()
node = FakeNav()
ex = MultiThreadedExecutor()
ex.add_node(node)
try:
    ex.spin()
except KeyboardInterrupt:
    pass
