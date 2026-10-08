"""Publishes a changing camera image, raw and JPEG, and its CameraInfo, for the capture tier.

Runs inside the ROS test container against the fake tf tree; nothing imports it.
"""
import argparse
import base64
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

# cv2 is what turns a frame into a real JPEG. The ROS base image does not necessarily carry
# python3-opencv, and this publisher must not be the reason a capture tier cannot run, so a
# real baseline JPEG is embedded below and published instead when the import fails.
try:
    import cv2
    import numpy
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

# A decodable baseline JPEG whose SOF0 declares 320 wide by 240 high -- this node's default
# size. At a different --width/--height the raw stream carries yours and this one does not,
# which is the cost of the fallback and the reason cv2 is used whenever it is there.
FALLBACK_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDACgcHiMeGSgjISMtKygwPGRBPDc3PHtYXUlkkYCZlo+AjIqgtObDoKrarYqM"
    "yP/L2u71////m8H////6/+b9//j/2wBDASstLTw1PHZBQXb4pYyl+Pj4+Pj4+Pj4+Pj4+Pj4+Pj4+Pj4+Pj4+Pj4+Pj4"
    "+Pj4+Pj4+Pj4+Pj4+Pj4+Pj4+Pj/wAARCADwAUADASIAAhEBAxEB/8QAFgABAQEAAAAAAAAAAAAAAAAAAAQD/8QAFBAB"
    "AAAAAAAAAAAAAAAAAAAAAP/EABcBAQEBAQAAAAAAAAAAAAAAAAADAgT/xAAXEQEAAwAAAAAAAAAAAAAAAAAAAgNB/9oA"
    "DAMBAAIRAxEAPwDIBN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA"
    "0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAA"
    "IlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0"
    "gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAI"
    "lqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0g"
    "AAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIl"
    "qJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gA"
    "AtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlq"
    "JqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAA"
    "tATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJ"
    "qKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAt"
    "ATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJq"
    "KN2ADSAAC0BN2gACJaiaijdgA0gAAtATdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAC0BN2gACJaiaijdgA0gAAtA"
    "TdoAAiWomoo3YANIAALQE3aAAIlqJqKN2ADSAAD/2Q==")

BYTES_PER_PIXEL = 3          # rgb8

# The CameraInfo zooms: its focal length alternates between these two, as a fraction of the
# image width, from one frame to the next. At 0.5 the horizontal field of view is exactly 90
# degrees; at 1.0 it is 2 atan(0.5). The capture tier expects both on the wire.
ZOOM_FOCAL_FRACTIONS = (0.5, 1.0)
COUNTER_BLOCK = 8            # pixels square
BAND_PIXELS = 16
BAND_STEP = 8                # pixels the band moves per frame


def background_bytes(width, height):
    """A fixed gradient, built once. Every frame is a copy of it with the moving parts painted."""
    rows = []
    for y in range(height):
        green = y * 255 // max(height - 1, 1)
        row = bytearray()
        for x in range(width):
            row += bytes((x * 255 // max(width - 1, 1), green, 64))
        rows.append(bytes(row))
    return b''.join(rows)


def paint(background, width, height, seq):
    """One rgb8 frame.

    The band is so a human watching the topic sees motion, and it repeats once it has crossed
    the image. The counter block in the top-left corner is what makes a capture of N frames
    provably N distinct frames: it carries `seq` as a colour, so it repeats after 16M.
    """
    frame = bytearray(background)
    band_start = (seq * BAND_STEP) % width
    band_end = min(band_start + BAND_PIXELS, width)
    stripe = b'\xff' * ((band_end - band_start) * BYTES_PER_PIXEL)
    for y in range(height):
        row = y * width * BYTES_PER_PIXEL
        frame[row + band_start * BYTES_PER_PIXEL:row + band_end * BYTES_PER_PIXEL] = stripe

    counter = bytes((seq & 0xFF, (seq >> 8) & 0xFF, (seq >> 16) & 0xFF))
    block = counter * min(COUNTER_BLOCK, width)
    for y in range(min(COUNTER_BLOCK, height)):
        row = y * width * BYTES_PER_PIXEL
        frame[row:row + len(block)] = block
    return frame


def with_sequence_comment(jpeg, seq):
    """The fallback JPEG with a COM segment carrying `seq` spliced in after its SOI.

    A decoder skips COM, so the picture and its SOF0 are unchanged and the bytes still decode,
    while no two frames are byte-identical. Without it a capture of N fallback frames could not
    be told from one frame captured N times.
    """
    payload = f'seq={seq}'.encode()
    segment = b'\xff\xfe' + struct.pack('>H', len(payload) + 2) + payload
    return jpeg[:2] + segment + jpeg[2:]


def encode_jpeg(frame, width, height, seq):
    if HAVE_CV2:
        rgb = numpy.frombuffer(bytes(frame), dtype=numpy.uint8).reshape(height, width, 3)
        # cvtColor rather than a reversing slice: cv2 needs a contiguous array and a negative
        # stride is not one.
        encoded, buffer = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if encoded:
            return buffer.tobytes()
    return with_sequence_comment(FALLBACK_JPEG, seq)


class FakeCamera(Node):
    def __init__(self, args):
        super().__init__('fake_camera')
        self.args = args
        self.seq = 0
        self.background = background_bytes(args.width, args.height)
        # Reliable, which is the default: a best-effort subscriber still matches a reliable
        # publisher, but a reliable subscriber matches nothing on a best-effort one -- and a QoS
        # mismatch is a subscription that never fires and never says why.
        self.image_pub = self.create_publisher(Image, args.image_topic, QoSProfile(depth=10))
        self.compressed_pub = self.create_publisher(
            CompressedImage, args.compressed_topic, QoSProfile(depth=10))
        self.info_pub = self.create_publisher(
            CameraInfo, args.camera_info_topic, QoSProfile(depth=10))
        self.create_timer(1.0 / args.rate, self.tick)

    def tick(self):
        args = self.args
        # One stamp for both messages: they are the same frame, and a consumer pairing them has
        # nothing but the stamp to pair them on.
        stamp = self.get_clock().now().to_msg()
        frame = paint(self.background, args.width, args.height, self.seq)

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = args.frame
        image.height = args.height
        image.width = args.width
        image.encoding = 'rgb8'
        image.is_bigendian = 0
        image.step = args.width * BYTES_PER_PIXEL
        image.data = bytes(frame)
        self.image_pub.publish(image)

        compressed = CompressedImage()
        compressed.header.stamp = stamp
        compressed.header.frame_id = args.frame
        compressed.format = 'jpeg'
        compressed.data = encode_jpeg(frame, args.width, args.height, self.seq)
        self.compressed_pub.publish(compressed)

        # Pinhole intrinsics with a centred principal point, square pixels and no distortion.
        focal = ZOOM_FOCAL_FRACTIONS[self.seq % len(ZOOM_FOCAL_FRACTIONS)] * args.width
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = args.frame
        info.width = args.width
        info.height = args.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [focal, 0.0, args.width / 2.0, 0.0, focal, args.height / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, args.width / 2.0, 0.0, 0.0, focal, args.height / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)
        self.seq += 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--image-topic', default='/fake/camera/image_raw')
    parser.add_argument('--compressed-topic', default='/fake/camera/image_raw/compressed')
    parser.add_argument('--camera-info-topic', default='/fake/camera/camera_info')
    parser.add_argument('--rate', type=float, default=5.0, help='Hz (default: %(default)s)')
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=240)
    parser.add_argument('--frame', default='base_link',
                        help='header.frame_id (default: %(default)s)')
    return parser.parse_args(argv)


def main():
    args = parse_args()
    encoder = 'cv2' if HAVE_CV2 else 'the embedded 320x240 fallback'
    # A tier that captured nothing has to be able to say which publisher was silent.
    print(f'fake_camera: {args.image_topic} (rgb8 {args.width}x{args.height}), '
          f'{args.compressed_topic} (jpeg via {encoder}) and {args.camera_info_topic} '
          f'(zooming) at {args.rate} Hz, frame {args.frame}', flush=True)
    rclpy.init()
    node = FakeCamera(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
