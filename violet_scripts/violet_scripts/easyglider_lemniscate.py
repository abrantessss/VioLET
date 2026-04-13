#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.easyglider_mission_base import EasyGliderMissionNode, spin_mission


class EasyGliderLemniscateNode(EasyGliderMissionNode):
    def __init__(self):
        super().__init__('easyglider_lemniscate_node', 'lemniscate')

    def build_trajectory(self):
        traj = Trajectory()
        traj.path_type = 3  # 3 indicates Lemniscate
        traj.lemniscate = [0.0, 0.0, -30.0, 200.0, 14.0]
        return traj


def main(args=None):
    spin_mission(EasyGliderLemniscateNode, args=args)


if __name__ == '__main__':
    main()
