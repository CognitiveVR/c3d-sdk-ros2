"""Publishes a coloured PointCloud2 with non-finite points, for the lidar stream's capture tier.

Runs inside the ROS test container against the fake tf tree; nothing imports it.
"""
import argparse
import random
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2, PointField

POINT_STEP = 16              # x, y, z, rgb, four bytes each
POINT = struct.Struct('<fffI')

# A few cubic metres around the robot: (min, max) per axis, metres.
BOX = ((-2.0, 2.0), (-2.0, 2.0), (0.0, 1.0))

# Each drawn position is emitted this many times with jitter well under half a 5 cm voxel leaf.
# Drawing independent positions out of a box this size would leave almost no two of them in one
# leaf, and a downsample pass that merges nothing cannot be shown to have run at all.
REPEATS = 4
JITTER_M = 0.01

SEED = 1234567           # fixed, so two runs of the tier carry the same cloud


def fields():
    return [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        # rgb is declared FLOAT32 and carries a uint32 0x00RRGGBB bit pattern. That is the PCL
        # convention, not a mistake in this file: a reader that CONVERTS rather than
        # reinterprets these four bytes gets noise.
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]


def generate(count, nan_fraction, seed=SEED):
    """`count` (x, y, z, rgb) tuples, some of them non-finite.

    The non-finite ones are what the encoder's finite filter has to have something to filter;
    `is_dense` is False precisely because they are here.
    """
    rng = random.Random(seed)
    points = []
    while len(points) < count:
        base = tuple(rng.uniform(low, high) for low, high in BOX)
        rgb = (rng.randrange(256) << 16) | (rng.randrange(256) << 8) | rng.randrange(256)
        for _ in range(REPEATS):
            if len(points) >= count:
                break
            jittered = tuple(axis + rng.uniform(-JITTER_M, JITTER_M) for axis in base)
            points.append((jittered[0], jittered[1], jittered[2], rgb))

    nan_count = int(count * nan_fraction)
    for index in rng.sample(range(count), nan_count):
        points[index] = (float('nan'), float('nan'), float('nan'), points[index][3])
    return points


def pack(points):
    return b''.join(POINT.pack(*point) for point in points)


class FakePointCloud(Node):
    def __init__(self, args):
        super().__init__('fake_pointcloud')
        self.args = args
        self.seq = 0
        self.buffer = pack(generate(args.points, args.nan_fraction))
        # Reliable, which is the default: a best-effort subscriber still matches a reliable
        # publisher, but a reliable subscriber matches nothing on a best-effort one -- and a QoS
        # mismatch is a subscription that never fires and never says why.
        self.publisher = self.create_publisher(PointCloud2, args.topic, QoSProfile(depth=5))
        self.create_timer(1.0 / args.rate, self.tick)

    def tick(self):
        args = self.args
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = args.frame
        cloud.height = 1
        cloud.width = args.points
        cloud.fields = fields()
        cloud.is_bigendian = False
        cloud.point_step = POINT_STEP
        cloud.row_step = POINT_STEP * args.points
        # False because the cloud carries NaNs. A consumer that trusts is_dense skips its own
        # finite check, so claiming True here would hide exactly what this publisher is for.
        cloud.is_dense = False
        # The same points, rotated by one per message. A cloud has no required point order, so
        # this changes nothing a reader may rely on -- it only makes two captured messages
        # impossible to confuse for one message captured twice.
        offset = (self.seq % args.points) * POINT_STEP
        cloud.data = self.buffer[offset:] + self.buffer[:offset]
        self.publisher.publish(cloud)
        self.seq += 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--topic', default='/fake/points')
    parser.add_argument('--rate', type=float, default=2.0, help='Hz (default: %(default)s)')
    parser.add_argument('--points', type=int, default=4000)
    parser.add_argument('--nan-fraction', type=float, default=0.05,
                        help='share of points set to NaN (default: %(default)s)')
    parser.add_argument('--frame', default='base_link',
                        help='header.frame_id (default: %(default)s)')
    return parser.parse_args(argv)


def main():
    args = parse_args()
    # A tier that captured nothing has to be able to say which publisher was silent.
    print(f'fake_pointcloud: {args.topic}, {args.points} points of xyz+rgb '
          f'({POINT_STEP} bytes each, {args.nan_fraction:.0%} NaN) at {args.rate} Hz, '
          f'frame {args.frame}', flush=True)
    rclpy.init()
    node = FakePointCloud(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
