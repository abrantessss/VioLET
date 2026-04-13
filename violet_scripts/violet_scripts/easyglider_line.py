#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.easyglider_mission_base import EasyGliderMissionNode, spin_mission


class EasyGliderLineNode(EasyGliderMissionNode):
    def __init__(self):
        super().__init__('easyglider_line_node', 'line')

    def build_trajectory(self):
        traj = Trajectory()
        traj.path_type = 1  # 1 indicates Line
        traj.line = [0.0, 0.0, -25.0, 160.0, 60.0, -35.0, 12.0]
        return traj


def main(args=None):
    spin_mission(EasyGliderLineNode, args=args)


if __name__ == '__main__':
    main()
