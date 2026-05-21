# violet_rviz

RViz plotting support for the same telemetry used by `violet_plots`.

The Plot2D RViz plugin plots scalar fields, while `violet_msgs/PlotData` stores vectors as fixed arrays. `violet_rviz` bridges `/drone1/plots/data` into scalar and point topics under `/drone1/rviz_plots`, then loads an RViz layout with:

- vehicle and desired 3D paths
- XY vehicle path versus desired path
- position error norm and axis errors
- UAV velocity, target path velocity, and desired velocity

## Plugin Dependency

Import the external RViz plugin source with:

```bash
vcs import . < violet_rviz/rviz_2d_plot_plugin.repos
```

Then install/build your workspace however you normally do. The package depends on `rviz_2d_plot_plugin`, but does not vendor its source.

## Launch

```bash
ros2 launch violet_rviz violet_rviz.launch.py
```

Useful arguments:

- `vehicle_ns:=drone1`
- `plot_data_topic:=/drone1/plots/data`
- `output_prefix:=rviz_plots`
- `history_length:=2000`
- `desired_velocity_path_scaled:=false`

The default RViz config expects `/drone1/rviz_plots/...` topics.

Use `desired_velocity_path_scaled:=true` when you want the Shuttle/Mellinger MATLAB convention for the desired velocity line (`vd * |dpd_dgamma|`). Leave it false for the EasyGlider/LOS convention (`vd`).
