"""Optional rosbag process for the real fixed-wing visualization launch."""
import re
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def default_bag_directory(share_directory):
    # Works for source/symlink installs and the standard VioLET colcon layout.
    for parent in Path(share_directory).resolve().parents:
        for folder in (parent / 'violet_plots', parent / 'src' / 'VioLET' / 'violet_plots'):
            if folder.is_dir():
                return str(folder / 'bags')
    return str(Path.home() / 'VioLET' / 'src' / 'VioLET' / 'violet_plots' / 'bags')


def recording_command(directory, namespace, stamp=None):
    folder = Path(directory).expanduser()
    folder.mkdir(parents=True, exist_ok=True)
    stamp = stamp or datetime.now(ZoneInfo('Europe/Lisbon'))
    output = folder / ('real_fixedwing_test_data_' + stamp.strftime('%Y%m%d_%H%M%S_%f'))
    if output.exists():
        raise FileExistsError(f'Bag already exists: {output}')
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
    pattern = '^(' + re.escape(prefix) + r'/(telemetry|rviz_plots)/.*|/tf|/tf_static)$'
    return ['ros2', 'bag', 'record', '--storage', 'sqlite3', '--include-hidden-topics',
            '--regex', pattern, '--output', str(output)]


def launch_recording(context):
    enabled = LaunchConfiguration('record_bag').perform(context).lower()
    if enabled not in ('true', 'false', '1', '0'):
        raise ValueError('record_bag must be true or false')
    if enabled in ('false', '0'):
        return []
    command = recording_command(LaunchConfiguration('bag_directory').perform(context),
                                LaunchConfiguration('vehicle_ns').perform(context))
    return [ExecuteProcess(cmd=command, name='fixedwing_rosbag', output='screen',
                           sigterm_timeout='15', sigkill_timeout='15')]
