#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.combined_vehicle_mission_base import CombinedVehicleMissionNode, spin_mission


class CombinedVehicleCircleNode(CombinedVehicleMissionNode):
    def __init__(self):
        super().__init__('combined_vehicle_circle_node', 'circle')
        self.declare_parameter('circle', [0.0, 0.0, -4.0, 30.0, 2.0])

    def build_trajectory(self):
        trajectory = Trajectory()
        trajectory.path_type = 2
        trajectory.circle = list(self.get_parameter('circle').value)
        return trajectory


def main(args=None):
    spin_mission(CombinedVehicleCircleNode, args=args)


if __name__ == '__main__':
    main()
