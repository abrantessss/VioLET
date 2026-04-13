#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.shuttle_mission_base import ShuttleMissionNode, spin_mission


class ShuttleWaypointNode(ShuttleMissionNode):
    def __init__(self):
        super().__init__('shuttle_waypoint_node', 'waypoint')

    def build_final_trajectory(self):
        traj = Trajectory()
        traj.path_type = 0  # 0 indicates Waypoint
        traj.waypoint = [20.0, 20.0, -20.0]
        return traj


def main(args=None):
    spin_mission(ShuttleWaypointNode, args=args)


if __name__ == '__main__':
    main()
