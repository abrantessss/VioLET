"""Write mission messages directly so the initial trajectory cannot be missed."""
from datetime import datetime
from pathlib import Path
import re

import rosbag2_py
from rclpy.serialization import serialize_message


def default_bag_directory():
    # Resolve source paths for normal and symlink-install workspaces.
    for parent in Path(__file__).resolve().parents:
        if (parent / 'violet_plots').is_dir():
            return str(parent / 'violet_plots' / 'bags')
    for parent in Path.cwd().resolve().parents:
        if (parent / 'violet_plots').is_dir():
            return str(parent / 'violet_plots' / 'bags')
        candidate = parent / 'src' / 'VioLET' / 'violet_plots'
        if candidate.is_dir():
            return str(candidate / 'bags')
    raise RuntimeError('Cannot locate violet_plots; set the bag_directory ROS parameter')


class MissionBag:
    def __init__(self, node, directory, controller, trajectory, kphi):
        self.node = node
        self.topics = set()
        root = Path(directory).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        clean = lambda value: re.sub(r'[^a-zA-Z0-9_.-]', '_', str(value))
        stamp = datetime.now().astimezone().strftime('%Y-%m-%d_%H-%M-%S_%f')
        self.path = root / f'{clean(controller)}_{clean(trajectory)}_kphi{clean(kphi)}_{stamp}'
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.path), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''),
        )
        node.get_logger().info(f'Recording mission rosbag: {self.path}')

    def write(self, topic, msg):
        if topic not in self.topics:
            module = type(msg).__module__.split('.')[0]
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=f'{module}/msg/{type(msg).__name__}',
                serialization_format='cdr',
            ))
            self.topics.add(topic)
        self.writer.write(topic, serialize_message(msg), self.node.get_clock().now().nanoseconds)

    def close(self):
        # Humble SequentialWriter finalizes metadata when released.
        self.writer = None
