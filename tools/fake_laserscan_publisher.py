"""Publishes a LaserScan with non-finite rays and intensities, for the lidar stream's capture tier.

Runs inside the ROS test container against the fake tf tree; nothing imports it.
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

# inf is what a driver publishes for a ray that returned nothing and NaN for one whose
# measurement is invalid. Both have to be on the wire, because the wire copies a scan as
# published and a reader that assumes every range is a number is wrong on a real robot.
NO_RETURN_EVERY = 128
INVALID_EVERY = 128
INVALID_PHASE = 64


def ranges_and_intensities(rays, range_min, range_max, seq):
    """One sweep. The shape turns with `seq`, so two captured scans are never identical."""
    ranges = []
    intensities = []
    for ray in range(rays):
        angle = -math.pi + 2.0 * math.pi * ray / rays
        if ray % NO_RETURN_EVERY == 0:
            ranges.append(float('inf'))
        elif ray % INVALID_EVERY == INVALID_PHASE:
            ranges.append(float('nan'))
        else:
            distance = 3.0 + 1.5 * math.sin(2.0 * angle + seq * 0.1)
            ranges.append(min(max(distance, range_min), range_max))
        # Populated for every ray, including the non-finite ones, so the rule that intensities
        # are dropped unless they were asked for has something to drop.
        intensities.append(120.0 + 100.0 * math.cos(angle))
    return ranges, intensities


class FakeLaserScan(Node):
    def __init__(self, args):
        super().__init__('fake_laserscan')
        self.args = args
        self.seq = 0
        # Reliable, which is the default: a best-effort subscriber still matches a reliable
        # publisher, but a reliable subscriber matches nothing on a best-effort one -- and a QoS
        # mismatch is a subscription that never fires and never says why.
        self.publisher = self.create_publisher(LaserScan, args.topic, QoSProfile(depth=10))
        self.create_timer(1.0 / args.rate, self.tick)

    def tick(self):
        args = self.args
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = args.frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        # Derived from the ray count rather than typed, so a consumer that walks the rays by
        # angle_increment and one that walks len(ranges) describe the same sweep.
        scan.angle_increment = (scan.angle_max - scan.angle_min) / args.rays
        scan.scan_time = 1.0 / args.rate
        scan.time_increment = scan.scan_time / args.rays
        scan.range_min = args.range_min
        scan.range_max = args.range_max
        scan.ranges, scan.intensities = ranges_and_intensities(
            args.rays, args.range_min, args.range_max, self.seq)
        self.publisher.publish(scan)
        self.seq += 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--topic', default='/fake/scan')
    parser.add_argument('--rate', type=float, default=5.0, help='Hz (default: %(default)s)')
    parser.add_argument('--rays', type=int, default=640)
    parser.add_argument('--range-min', type=float, default=0.1, help='metres')
    parser.add_argument('--range-max', type=float, default=12.0, help='metres')
    parser.add_argument('--frame', default='base_link',
                        help='header.frame_id (default: %(default)s)')
    return parser.parse_args(argv)


def main():
    args = parse_args()
    # A tier that captured nothing has to be able to say which publisher was silent.
    print(f'fake_laserscan: {args.topic}, {args.rays} rays over [-pi, pi] with intensities, '
          f'range {args.range_min}-{args.range_max} m, at {args.rate} Hz, '
          f'frame {args.frame}', flush=True)
    rclpy.init()
    node = FakeLaserScan(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
