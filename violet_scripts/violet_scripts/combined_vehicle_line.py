#!/usr/bin/env python3
from violet_msgs.msg import Trajectory

from violet_scripts.combined_vehicle_mission_base import CombinedVehicleMissionNode, spin_mission


class CombinedVehicleLineNode(CombinedVehicleMissionNode):
    def __init__(self):
        super().__init__('combined_vehicle_line_node', 'line')
        self.declare_parameter('line', [20.0, -10.0, -5.0, 30.0, 20.0, -5.0, 2.0])

    def build_trajectory(self):
        trajectory = Trajectory()
        trajectory.path_type = 1
        trajectory.line = list(self.get_parameter('line').value)
        return trajectory


def main(args=None):
    spin_mission(CombinedVehicleLineNode, args=args)


if __name__ == '__main__':
    main()
