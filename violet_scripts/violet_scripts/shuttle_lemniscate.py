#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.shuttle_mission_base import ShuttleMissionNode, spin_mission


class ShuttleLemniscateNode(ShuttleMissionNode):
    def __init__(self):
        super().__init__('shuttle_lemniscate_node', 'lemniscate')

    def build_final_trajectory(self):
        traj = Trajectory()
        traj.path_type = 3  # 3 indicates Lemniscate
        traj.lemniscate = [10.0, 10.0, -15.0, 25.0, 0.15]
        return traj


def main(args=None):
    spin_mission(ShuttleLemniscateNode, args=args)


if __name__ == '__main__':
    main()
