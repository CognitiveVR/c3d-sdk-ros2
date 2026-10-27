"""Publishes a moving map->odom->base_link chain, with a deliberate relocalisation jump.

`--gimbal` adds a camera on a panning gimbal under `base_link`: `camera_link`, turning about its
own Z at 20 Hz, and a static `camera_optical_frame` under it. The pose stream's capture pass reads
it as a moving joint of the rig.

`--mount-frame` adds a STATIC sensor mount under `map` as well. Its parent is the reference
frame itself rather than `base_link` on purpose: a sensor hanging off the robot moves, so its
world pose is only knowable at a record's own stamp, while this one is the transform that was
published and nothing else. That is what lets the scan-plane pass reconstruct a ray's world
position from numbers it typed rather than from the SDK's own conversion.
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


# The gimbal's mount on the body and how far it pans either way.
GIMBAL_X_METERS = 0.08
GIMBAL_Z_METERS = 0.1
GIMBAL_PAN_RADIANS = 0.6


def quaternion_from_rpy(roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll) as (x, y, z, w) -- the ROS composition order."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def triple(text):
    """`a,b,c` as three floats, for the mount's translation and its roll/pitch/yaw."""
    parts = text.split(',')
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f'want three comma-separated numbers, got {text!r}')
    return tuple(float(part) for part in parts)


class TfPub(Node):
    def __init__(self, args):
        super().__init__("tf_pub")
        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.t = 0.0
        now = self.get_clock().now().to_msg()
        statics = []
        s = TransformStamped()
        s.header.frame_id = "base_link"; s.child_frame_id = "base_scan"
        s.transform.translation.z = 0.2; s.transform.rotation.w = 1.0
        s.header.stamp = now
        statics.append(s)
        self.gimbal = args.gimbal
        if self.gimbal:
            o = TransformStamped()
            o.header.stamp = now
            o.header.frame_id = "camera_link"; o.child_frame_id = "camera_optical_frame"
            o.transform.rotation.x = -0.5; o.transform.rotation.y = 0.5
            o.transform.rotation.z = -0.5; o.transform.rotation.w = 0.5
            statics.append(o)
        if args.mount_frame:
            x, y, z = args.mount_xyz
            qx, qy, qz, qw = quaternion_from_rpy(*(math.radians(a) for a in args.mount_rpy_deg))
            m = TransformStamped()
            m.header.stamp = now
            m.header.frame_id = args.mount_parent; m.child_frame_id = args.mount_frame
            m.transform.translation.x = x
            m.transform.translation.y = y
            m.transform.translation.z = z
            m.transform.rotation.x = qx
            m.transform.rotation.y = qy
            m.transform.rotation.z = qz
            m.transform.rotation.w = qw
            statics.append(m)
        self.static_br.sendTransform(statics)
        self.create_timer(0.05, self.tick)

    def tick(self):
        self.t += 0.05
        now = self.get_clock().now().to_msg()
        # A deliberate localisation jump at t=8s, so the jump detector has
        # something to detect.
        jump = 1.5 if self.t > 8.0 else 0.0
        mo = TransformStamped()
        mo.header.stamp = now; mo.header.frame_id = "map"; mo.child_frame_id = "odom"
        mo.transform.translation.x = jump; mo.transform.rotation.w = 1.0
        self.br.sendTransform(mo)

        yaw = self.t * 0.3
        ob = TransformStamped()
        ob.header.stamp = now; ob.header.frame_id = "odom"; ob.child_frame_id = "base_link"
        ob.transform.translation.x = self.t * 0.5
        ob.transform.translation.y = math.sin(self.t * 0.4)
        ob.transform.rotation.z = math.sin(yaw / 2.0)
        ob.transform.rotation.w = math.cos(yaw / 2.0)
        self.br.sendTransform(ob)

        if self.gimbal:
            pan = GIMBAL_PAN_RADIANS * math.sin(self.t)
            g = TransformStamped()
            g.header.stamp = now; g.header.frame_id = "base_link"; g.child_frame_id = "camera_link"
            g.transform.translation.x = GIMBAL_X_METERS
            g.transform.translation.z = GIMBAL_Z_METERS
            g.transform.rotation.z = math.sin(pan / 2.0)
            g.transform.rotation.w = math.cos(pan / 2.0)
            self.br.sendTransform(g)

        o = Odometry()
        o.header.stamp = now; o.header.frame_id = "odom"; o.child_frame_id = "base_link"
        o.pose.pose.position.x = self.t * 0.5
        o.pose.pose.position.y = math.sin(self.t * 0.4)
        o.twist.twist.linear.x = 0.5
        o.twist.twist.angular.z = 0.3
        self.odom_pub.publish(o)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--gimbal', action='store_true',
                        help='also publish camera_link panning under base_link, and its optical frame')
    parser.add_argument('--mount-frame', default='',
                        help='also publish a static sensor mount by this name (default: none)')
    parser.add_argument('--mount-parent', default='map',
                        help='the mount\'s parent frame (default: %(default)s)')
    parser.add_argument('--mount-xyz', type=triple, default=(0.0, 0.0, 0.0),
                        help='the mount\'s translation, metres, as x,y,z')
    parser.add_argument('--mount-rpy-deg', type=triple, default=(0.0, 0.0, 0.0),
                        help='the mount\'s rotation, degrees, as roll,pitch,yaw')
    return parser.parse_args(argv)


def main():
    args = parse_args()
    if args.mount_frame:
        # A tier that reconstructed rays against the wrong transform has to be able to say
        # which one it published.
        print(f'fake_tf: static {args.mount_parent} -> {args.mount_frame} at '
              f'{args.mount_xyz} m, rpy {args.mount_rpy_deg} deg', flush=True)
    rclpy.init()
    node = TfPub(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
