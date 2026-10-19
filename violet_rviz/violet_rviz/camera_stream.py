"""Decode the Pi's network video stream and publish ROS images."""
import subprocess
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def read_frame(stream, size):
    """Pipe reads may return only part of a frame."""
    data = bytearray()
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


def frame_to_image(data, width, height, stamp, frame_id):
    if len(data) != width * height * 3:
        raise ValueError('Incomplete BGR frame')
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width, msg.height = width, height
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = width * 3
    msg.data = data
    return msg


def decoder_command(url, width, height):
    return [
        'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
        '-rw_timeout', '5000000', '-fflags', 'nobuffer', '-flags', 'low_delay',
        '-threads', '1', '-probesize', '32768', '-analyzeduration', '1',
        '-i', url, '-an', '-vf', f'scale={width}:{height}',
        '-vsync', '0', '-pix_fmt', 'bgr24', '-flush_packets', '1',
        '-f', 'rawvideo', 'pipe:1',
    ]


class CameraStream(Node):
    def __init__(self):
        super().__init__('camera_stream')
        self.url = self.declare_parameter('camera_url', 'tcp://192.168.1.38:8888').value
        self.width = self.declare_parameter('image_width', 640).value
        self.height = self.declare_parameter('image_height', 480).value
        self.frame_id = self.declare_parameter('camera_frame', 'camera_optical_frame').value
        if self.width <= 0 or self.height <= 0:
            raise ValueError('Image dimensions must be positive')
        # Reliable delivery works with the default RViz Image QoS.
        self.publisher = self.create_publisher(Image, 'telemetry/camera/image_raw', 1)
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.publish_timer = self.create_timer(1.0 / 30.0, self.publish_latest)
        self.stop_event = threading.Event()
        self.process_lock = threading.Lock()
        self.process = None
        self.worker = threading.Thread(target=self.receive, daemon=True)
        self.worker.start()

    def receive(self):
        while not self.stop_event.is_set():
            self.get_logger().info(f'Connecting to camera: {self.url}')
            command = decoder_command(self.url, self.width, self.height)
            try:
                with self.process_lock:
                    if self.stop_event.is_set():
                        return
                    self.process = subprocess.Popen(command, stdout=subprocess.PIPE)
                    process = self.process
                first_frame = True
                while not self.stop_event.is_set():
                    data = read_frame(process.stdout, self.width * self.height * 3)
                    if data is None:
                        break
                    # Keep draining the decoder even if ROS/RViz is slow.
                    with self.frame_lock:
                        self.latest_frame = (data, self.get_clock().now().to_msg())
                    if first_frame:
                        self.get_logger().info('Camera frames arriving')
                        first_frame = False
            except FileNotFoundError:
                self.get_logger().error('FFmpeg is missing. Install it with: sudo apt install ffmpeg')
                return
            except Exception as error:
                if not self.stop_event.is_set():
                    self.get_logger().warning(f'Camera receive failed: {error}')
            finally:
                with self.process_lock:
                    if self.process is not None:
                        if self.process.poll() is None:
                            self.process.terminate()
                        try:
                            self.process.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            self.process.kill()
                            self.process.wait()
                        self.process.stdout.close()
                        self.process = None
            with self.frame_lock:
                self.latest_frame = None
            if not self.stop_event.is_set():
                self.get_logger().warning('Camera stream unavailable; retrying in 2 seconds')
                self.stop_event.wait(2)

    def publish_latest(self):
        with self.frame_lock:
            frame = self.latest_frame
            self.latest_frame = None
        if frame is not None:
            data, stamp = frame
            self.publisher.publish(frame_to_image(
                data, self.width, self.height, stamp, self.frame_id))

    def destroy_node(self):
        self.stop_event.set()
        with self.process_lock:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
        self.worker.join(timeout=8)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
