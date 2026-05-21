#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.easyglider_mission_base import EasyGliderMissionNode, spin_mission


class EasyGliderCircleNode(EasyGliderMissionNode):
    def __init__(self):
        super().__init__('easyglider_circle_node', 'circle')

    def build_trajectory(self):
        traj = Trajectory()
        traj.path_type = 2  # 2 indicates Circle
        traj.circle = [-250.0, 50.0, -30.0, 100.0, 18.0]
        return traj


def main(args=None):
    spin_mission(EasyGliderCircleNode, args=args)


if __name__ == '__main__':
    main()
