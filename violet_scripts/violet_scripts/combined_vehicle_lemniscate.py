#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.combined_vehicle_mission_base import CombinedVehicleMissionNode, spin_mission


class CombinedVehicleLemniscateNode(CombinedVehicleMissionNode):
    def __init__(self):
        super().__init__('combined_vehicle_lemniscate_node', 'lemniscate')
        self.declare_parameter('lemniscate', [0.0, 0.0, -8.0, 15.0, 2.0])

    def build_trajectory(self):
        trajectory = Trajectory()
        trajectory.path_type = 3
        trajectory.lemniscate = list(self.get_parameter('lemniscate').value)
        return trajectory


def main(args=None):
    spin_mission(CombinedVehicleLemniscateNode, args=args)


if __name__ == '__main__':
    main()
