#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.shuttle_mission_base import ShuttleMissionNode, spin_mission


class ShuttleLemniscateNode(ShuttleMissionNode):
    def __init__(self):
        super().__init__('shuttle_lemniscate_node', 'lemniscate')

    def build_final_trajectory(self):
        traj = Trajectory()
        traj.path_type = 3  # Gerono figure eight; final parameter is phase rate [rad/s]
        traj.lemniscate = [-5.0, 10.0, -15.0, 30.0, 0.21]
        return traj


def main(args=None):
    spin_mission(ShuttleLemniscateNode, args=args)


if __name__ == '__main__':
    main()
