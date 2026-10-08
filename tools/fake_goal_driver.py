"""Sends a sequence of NavigateToPose goals, cancelling one of them."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

rclpy.init()
node = Node("goal_driver")
client = ActionClient(node, NavigateToPose, "navigate_to_pose")
client.wait_for_server(timeout_sec=20.0)

for i in range(4):
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = float(i)
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    handle = fut.result()
    node.get_logger().info(f"goal {i} accepted={handle.accepted if handle else None}")
    result_fut = handle.get_result_async()
    if i == 2:
        # Cancel this one mid-flight. CANCELED is an outcome a simulator will
        # essentially never hand you, and it is exactly where the objective
        # translator can go wrong.
        time.sleep(1.5)
        cancel_fut = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_fut, timeout_sec=10.0)
        node.get_logger().info("goal 2 cancel requested")
    rclpy.spin_until_future_complete(node, result_fut, timeout_sec=25.0)
    node.get_logger().info(f"goal {i} finished")
    time.sleep(0.5)

node.get_logger().info("driver done")
