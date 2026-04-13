#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.shuttle_mission_base import ShuttleMissionNode, spin_mission


class ShuttleLineNode(ShuttleMissionNode):
    def __init__(self):
        super().__init__('shuttle_line_node', 'line')

    def build_final_trajectory(self):
        traj = Trajectory()
        traj.path_type = 1  # 1 indicates Line
        traj.line = [1.0, 1.0, -10.0, 20.0, 20.0, -20.0, 0.15]
        return traj


def main(args=None):
    spin_mission(ShuttleLineNode, args=args)


if __name__ == '__main__':
    main()
