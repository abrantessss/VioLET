# LOS4 live RViz diagnostics

The Pi publishes standard MAVLink TUNNEL diagnostic messages (local payload type
32768) alongside telemetry. The desktop receiver converts them to ROS topics.
Rebuild both the Pi executable and the desktop visualization/plugin to use this.
No flight-control behavior is changed by these diagnostic messages.

## Update Pi sources from the desktop

```bash
cd /home/abrantess/Downloads/capture-pi-saved/fil2k
scp MAVSDK/CAPTURE/pf_los4/src/{pf_los4.cpp,los4_config.hpp,los4_safety.hpp,los4_runtime.hpp,los4_runtime.cpp,los4_diagnostics.hpp} \
  capture@192.168.1.38:~/MAVSDK/CAPTURE/pf_los4/src/
```

## Pi console: build and run gamma-only test with telemetry forwarding

```bash
cd ~/MAVSDK/CAPTURE/pf_los4
cmake -S . -B build
cmake --build build -j2
./build/pf_los4 serial:///dev/ttyAMA0:115200 10 udp://192.168.1.94:14540 \
  --controller-enabled false --test-gamma true --offline false \
  --path circle --center 0,0,-30 --radius 50 --speed 22
```

Stop any other pf_los4 instance first. `--offline false` is essential when testing
RViz: offline mode opens no MAVLink connection. Test mode sends diagnostics and
advances gamma_dot=1, but never sends setpoints or requests flight mode changes.

## Desktop console: build and launch

```bash
cd ~/VioLET
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select rviz_2d_plot_plugin violet_rviz --cmake-force-configure \
  --cmake-args -DMAVSDK_DIR=/home/abrantess/pegasus_external/install/mavsdk_vendor/lib/cmake/MAVSDK
source install/setup.bash
ros2 launch violet_rviz pf_los4_rviz.launch.py \
  connection_url:=udp://:14540 camera_url:=tcp://192.168.1.38:8888
```

Keep the Pi rpicam-vid process running separately, or use camera_enabled:=false.

## Displays and conventions

- Full desired path in 3D and XY: configuration is repeated once per second;
  the 2D plugin accepts complete nav_msgs/Path snapshots, so it appears before
  the gamma target has travelled around it. Line geometry displays p0 to p1;
  the controller's mathematical line continues beyond that segment.
- Cyan ball in 3D and cyan point in XY: pd(gamma), the current virtual target.
  The XY point is replaced each update instead of leaving a target history.
- Velocity: measured norm and N/E/D components, desired speed vd, and virtual
  target speed |dpd/dgamma|*|gamma_dot|. Gamma_dot itself is a separate ROS scalar:
  parameter units/s for a line, radians/s for a circle or lemniscate.
- Path tracking error: position - pd(gamma), signed N/E/D and norm in metres.
  This is virtual-target tracking error, not nearest-point distance to the curve.
- Attitude: measured roll/pitch/yaw and desired roll/pitch/guidance-heading.
- Attitude error: desired-minus-measured, wrapped to [-pi,pi]. Heading-minus-yaw
  is explicitly labelled: LOS4 commands yaw rate, not an independent yaw angle.

Telemetry and reference geometry use map_ned: X North, Y East, Z Down. The
rviz_world transform gives the camera an upright view. Angular curves are radians.
Diagnostics use a maximum 20 Hz; path configuration uses 1 Hz. The bridge publishes
only received packets. Invalid/stale position/attitude references do not produce
new error samples. Desired roll/pitch is available in active control, or as a
shadow computation during gamma test when measured speed/attitude is suitable.
A stationary bench test still shows path/gamma and target speeds, but may have
no valid desired attitude. Existing autopilot telemetry/camera displays remain.

## Topics under /drone1

- telemetry/los4/config: std_msgs/Float32MultiArray, 15 float values:
  [version=1, kind=1, path_type, p0_N,E,D, p1_N,E,D, centre_N,E,D, R, a, vd].
- telemetry/los4/state: std_msgs/Float32MultiArray, 25 float values:
  [version=1, kind=2, phase, gamma, gamma_dot, vd, pd_N,E,D, dpd_dgamma_N,E,D,
   position_N,E,D, velocity_N,E,D, roll,pitch,yaw, desired_roll,pitch,heading, flags].
  Flags: 1 position valid, 2 velocity valid, 4 attitude valid, 8 reference valid.
  Phase: 0 disabled, 1 test, 2 waiting, 3 priming, 4 starting, 5 active,
  6 returning, 7 fault. Unflagged sensor/reference fields must not be used.
- rviz_plots/desired_path: nav_msgs/Path (transient local).
- rviz_plots/gamma_position: visualization_msgs/Marker.
- rviz_plots/gamma_xy: nav_msgs/Path with one pose, for the XY point.
- rviz_plots/velocity/desired and velocity/gamma_speed: std_msgs/Float32, m/s.
- rviz_plots/gamma and gamma_dot: std_msgs/Float32.
- rviz_plots/path_error/{north,east,down,norm}: std_msgs/Float32, m.
- rviz_plots/attitude_desired/{roll,pitch,heading}: std_msgs/Float32, rad.
- rviz_plots/attitude_error/{roll,pitch,heading}: std_msgs/Float32, rad.

## RC mode and kill behavior

PX4's configured RC channel 7 selects Manual (low/mid) or Offboard (high).
PX4 heartbeat is authoritative for actual mode and armed state (arming remains
RC/PX4-controlled, channel 5 in this setup). Channel 6 HIGH inhibits LOS4.
With controller-enabled true, test-gamma false, and offline false, the Pi waits
for RC arming/takeoff, valid telemetry and adequate ground speed. It prepares the
Offboard setpoint stream while PX4 reports Manual; gamma/integrators freeze.
After >=1.2 s preparation, channel 7 HIGH plus confirmed Offboard permits LOS4.
The script never requests Offboard entry. Selecting low/mid stops active LOS4
and lets PX4 follow the RC Manual selection. If heartbeat still reports Offboard
when the switch drops, or if a fault occurs, the script requests Manual.
Kill, disarming or stale data stops the setpoints. Fault recovery requires an
RC cycle through Manual; held-HIGH never silently retries activation.
The startup hard inhibit and gamma-test override remain in place.

Continuous setpoints before and during Offboard are a PX4 requirement; MAVLink
heartbeats alone do not satisfy it. See the
[PX4 Offboard guide](https://docs.px4.io/v1.14/en/flight_modes/offboard).
The script does not clear PX4's emergency kill or change flight-mode parameters.
Link/process loss recovery still depends on configured PX4 failsafes.

## Mode display and named actuators

`telemetry/flight_mode` (String), `telemetry/armed` and `telemetry/kill_active`
(Bool), and `telemetry/rc_channels` (raw PWM Float32MultiArray) are available.
RViz shows mode/armed/kill text above the vehicle; stale status shows UNKNOWN.
Actual mode comes from PX4 heartbeat, not an inferred channel position.
Kill indication uses channel 6, high >=1700 PWM by default, matching Pi config.

The Actuator Outputs graph has a fixed [-1,1] axis and these five signals:

| Name | MAIN output | Array index |
|---|---|---|
| Motor | 8 | 7 |
| Elevator | 4 | 3 |
| Rudder | 3 | 2 |
| Right aileron | 2 | 1 |
| Left aileron | 1 | 0 |

It displays normalized electrical output commands, not measured servo angles.
Default conversion is 1000 PWM -> -1, 1500 -> 0, 2000 -> +1 (including motor:
-1 means minimum/off). Per-output min/trim/max and encoding are ROS node startup
parameters: actuator_channels, actuator_pwm_min/trim/max and actuator_encoding.
Use actual calibrated values for the airframe. Original raw output topics remain
available and are recorded alongside the five named scalar topics.

## Optional rosbag recording

Recording defaults to false. Enable it on the desktop with:

```bash
ros2 launch violet_rviz pf_los4_rviz.launch.py record_bag:=true \
  connection_url:=udp://:14540 camera_url:=tcp://192.168.1.38:8888
```

Bags go to `~/VioLET/src/VioLET/violet_plots/bags/` as
`real_fixedwing_test_data_YYYYMMDD_HHMMSS_microseconds/`, containing `.db3` files
and metadata.yaml. The timestamp uses Europe/Lisbon time. Override bag_directory
if the workspace is elsewhere. It records this vehicle's telemetry and rviz_plots
(including LOS4 configuration/state, path/gamma/error plots, current mode, RC,
actuators and camera when running), plus /tf and /tf_static.
Stop the launch with Ctrl-C to close the bag cleanly.


Stationary Offboard checks are described in [BENCH_OFFBOARD.md](BENCH_OFFBOARD.md).
Diagnostic phase 8 indicates bench Offboard active; LOS4 is not evaluated.

## RC channel 7 and armed-state plot

The bottom-right step plot compares received channel 7 and PX4 armed state.
`rviz_plots/rc/channel7` stores raw PWM-like values in microseconds, extracted
from `telemetry/rc_channels` index 6. The display applies `(PWM - 1000)/1000`:
Low is approximately 0, Mid 0.5, and High 1. Receiver endpoints can lie slightly
outside this nominal interval. Armed is 0 for false and 1 for true. No initial
zero is invented and missing input does not become a false disarmed indication.

The live `pf_los4_rviz.launch.py` starts `rc_status_plot` automatically. These
scalar topics are included by the existing optional bag recording topic regex.
There are no Pi-side changes.

For older bags containing `telemetry/rc_channels` and `telemetry/armed` but not
the new plotting topics, run this additional computer terminal during playback:

```bash
source /opt/ros/humble/setup.bash
source ~/VioLET/install/setup.bash
ros2 run violet_rviz rc_status_plot --ros-args -r __ns:=/drone1 -p use_sim_time:=true
```

Then open RViz and play the bag with `--clock` as usual. New bags containing the
plot scalars can be replayed directly without this extra node.

The same live/playback plotting node also publishes
`rviz_plots/velocity/horizontal_speed` from `telemetry/velocity` as
`sqrt(vNorth^2 + vEast^2)`. The velocity graph shows this alongside the existing
full `sqrt(vNorth^2 + vEast^2 + vDown^2)` curve, now labelled Ground Speed (3D).
It works with old bags through the playback launch, without regenerating the
other recorded plot topics. No missing sample is replaced with zero.
