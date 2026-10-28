# VioLET

**VioLET** is a GNC framework for autonomous multirotor and fixed-wing UAV flights based on ROS 2, PX4 stack and Gazebo. It is inspired by the work developed in the [**Pegasus Project**](https://github.com/PegasusResearch), with the objective of supporting self-learning and Master's thesis validation.
<br>
<br>
<p align="center">
  <img src="docs/VioLET.jpeg" width="700"/>
</p>

---

## Table of Contents

1. [Requirements](#requirements)
2. [Repository Structure](#repository-structure)
3. [Installation](#installation)
   1. [ROS 2 Humble](#1-ros-2-humble)
   2. [PX4-Autopilot](#2-px4-autopilot)
   3. [Gazebo Classic 11](#3-gazebo-classic-11)
   4. [Micro XRCE-DDS Agent](#4-micro-xrce-dds-agent)
   5. [VioLET Airframes in PX4](#5-violet-airframes-in-px4)
   6. [VioLET Workspace](#6-violet-workspace)
   7. [MAVSDK backend (optional)](#7-mavsdk-backend-optional)
4. [Running a Simulation](#running-a-simulation)
5. [Acknowledgements](#acknowledgements)

---

## Requirements

| Component            | Version                                      |
| -------------------- | -------------------------------------------- |
| OS                   | Ubuntu 22.04 LTS                             |
| ROS 2                | Humble Hawksbill                             |
| PX4-Autopilot        | `main` @ `8a8496d` (matches `px4_msgs` submodule) |
| Simulator            | Gazebo Classic 11 + `gazebo_ros_pkgs`        |
| Middleware           | Micro XRCE-DDS Agent `v2.4.3`                |
| Optional             | MAVSDK 2 (for the MAVLink backend)           |

> [!IMPORTANT]
> The `px4_msgs` definitions **must match** the PX4 firmware version you run. The `violet_msgs/px4_msgs` submodule was generated from PX4 commit `8a8496d57e0d5140d9210c255d18435cac1de57a`, so build PX4 from that commit (see [step 2](#2-px4-autopilot)). If you update one, update the other.

---

## Repository Structure

| Package               | Description                                                                 |
| --------------------- | --------------------------------------------------------------------------- |
| `violet`              | Top-level launch files and vehicle configuration (`config/*.yaml`)          |
| `violet_autopilot`    | Guidance and control (rate, attitude, Mellinger, LOS path-following, ...)   |
| `violet_interface`    | Bridge between PX4 uORB/DDS topics and VioLET messages                      |
| `violet_msgs`         | VioLET custom messages + `px4_msgs` (submodule)                             |
| `violet_console`      | Terminal console to command and monitor the vehicles                        |
| `violet_gazebo`       | Gazebo Classic models (`shuttle`, `easyglider`, ...), worlds and plugins    |
| `violet_gz`           | Experimental support for the new Gazebo (Harmonic)                          |
| `violet_rviz`         | RViz 2 visualization, plotting and bag-recording tools                      |
| `violet_scripts`      | Mission scripts (lines, circles, lemniscates, controller tests)             |
| `violet_plots`        | MATLAB post-processing scripts                                              |
| `rviz_2d_plot_plugin` | RViz 2D plotting plugin (submodule)                                         |

---

## Installation

The instructions assume the following layout — adjust paths if yours differ:

```
~/PX4-Autopilot        # PX4 firmware
~/VioLET/src/VioLET    # this repository (inside the ~/VioLET colcon workspace)
```

### 1. ROS 2 Humble

Follow the official guide: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html> (`ros-humble-desktop`). Then install the development tools:

```bash
sudo apt update
sudo apt install -y ros-dev-tools python3-colcon-common-extensions python3-rosdep libeigen3-dev
sudo rosdep init   # skip if already done
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. PX4-Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout 8a8496d57e0d5140d9210c255d18435cac1de57a
git submodule update --init --recursive

# Install the PX4 toolchain and Python dependencies
bash ./Tools/setup/ubuntu.sh
```

Reboot after the script finishes. VioLET locates PX4 through the `PX4_DIR` environment variable, which **must** be set:

```bash
echo "export PX4_DIR=$HOME/PX4-Autopilot" >> ~/.bashrc
source ~/.bashrc
```

### 3. Gazebo Classic 11

On Ubuntu 22.04 the PX4 setup script installs the new Gazebo (Harmonic). VioLET's main simulation pipeline (`violet_gazebo`) uses **Gazebo Classic**, so replace it:

```bash
sudo apt remove gz-harmonic
sudo apt install -y aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
```

Install the ROS 2 ↔ Gazebo Classic bridge (used to spawn the vehicles):

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

Build PX4 SITL together with the Gazebo Classic plugins (this creates `build/px4_sitl_default/build_gazebo-classic`, which VioLET adds to `GAZEBO_PLUGIN_PATH`):

```bash
cd $PX4_DIR
DONT_RUN=1 make px4_sitl gazebo-classic
```

Optionally verify the install with `make px4_sitl gazebo-classic` — an Iris quadrotor should appear in Gazebo.

### 4. Micro XRCE-DDS Agent

PX4 talks to ROS 2 through uXRCE-DDS. The agent is started automatically by the VioLET launch files (`MicroXRCEAgent udp4 -p 8888`), so it only has to be installed:

```bash
cd ~
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Check it with `MicroXRCEAgent udp4 -p 8888` (stop it with `Ctrl+C`).

### 5. VioLET Airframes in PX4

VioLET spawns each vehicle with `PX4_SIM_MODEL=gazebo-classic_<model>`, so PX4 needs a matching airframe file for each vehicle:

| Airframe | `SYS_AUTOSTART` | Vehicle | Type |
| --- | --- | --- | --- |
| `10020_gazebo-classic_shuttle` | 10020 | Capture Shuttle quadrotor | Multirotor |
| `1046_gazebo-classic_easyglider` | 1046 | EasyGlider | Fixed-wing |

**5.1 Create the airframe files** in `$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/`.

**Capture Shuttle** — `10020_gazebo-classic_shuttle`:

```bash
cat > $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_shuttle << 'EOF'
#!/bin/sh
#
# @name Capture Shuttle Quadrotor SITL
#
# @type Quadrotor Shuttle
#
# @maintainer Luís Abrantes <luis.g.abrantes@tecnico.ulisboa.pt>
#

. ${R}etc/init.d/rc.mc_defaults


param set-default CA_AIRFRAME 0

param set NAV_DLL_ACT 0 #no need

param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 0.248
param set-default CA_ROTOR0_PY 0.248
param set-default CA_ROTOR0_KM 0.016
param set-default CA_ROTOR1_PX -0.248
param set-default CA_ROTOR1_PY -0.248
param set-default CA_ROTOR1_KM 0.016
param set-default CA_ROTOR2_PX 0.248
param set-default CA_ROTOR2_PY -0.248
param set-default CA_ROTOR2_KM -0.016
param set-default CA_ROTOR3_PX -0.248
param set-default CA_ROTOR3_PY 0.248
param set-default CA_ROTOR3_KM -0.016

param set-default MC_ROLLRATE_K 0.3000
param set-default MC_ROLLRATE_D 0.0010
param set-default MC_ROLLRATE_I 0.100
param set-default MC_PITCHRATE_K 0.3000
param set-default MC_PITCHRATE_D 0.0010
param set-default MC_PITCHRATE_I 0.100
param set-default MC_YAWRATE_K 1.0000
param set-default MC_YAWRATE_I 0.21

param set-default MC_ROLL_P 6.50
param set-default MC_PITCH_P 6.50
param set-default MC_YAW_P 4.48

param set-default MPC_XY_VEL_P_ACC 1.30
param set-default MPC_XY_VEL_I_ACC 0.20
param set-default MPC_XY_VEL_D_ACC 0.20
param set-default MPC_Z_VEL_P_ACC 2.00
param set-default MPC_Z_VEL_I_ACC 0.20
param set-default MPC_Z_VEL_D_ACC 0.90

param set-default MPC_XY_P 1.25
param set-default MPC_Z_P 1.50

param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104

param set-default COM_DISARM_PRFLT 0.0
param set-default COM_DISARM_LAND -1
EOF
```

**EasyGlider** — `1046_gazebo-classic_easyglider`:

```bash
cat > $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/1046_gazebo-classic_easyglider << 'EOF'
#!/bin/sh
#
# @name EasyGlider Fixed-Wing SITL
#
# @type Standard Plane
#
# @maintainer Luís Abrantes <luis.g.abrantes@tecnico.ulisboa.pt>
#

. ${R}etc/init.d/rc.fw_defaults

param set-default FW_LND_ANG 8
param set-default NPFG_PERIOD 12

param set-default FW_MAN_P_MAX 30
param set-default FW_PR_P 0.9
param set-default FW_PR_FF 0.5
param set-default FW_PR_I 0.5

param set-default FW_PSP_OFF 2
param set-default FW_P_LIM_MAX 32
param set-default FW_P_LIM_MIN -15

param set-default FW_RR_FF 0.5
param set-default FW_RR_P 0.3
param set-default FW_RR_I 0.5

param set-default FW_YR_FF 0.5
param set-default FW_YR_P 0.6
param set-default FW_YR_I 0.5

param set-default FW_SPOILERS_LND 0.4

param set-default FW_THR_MIN 0.05
param set-default FW_THR_TRIM 0.25

param set-default FW_T_CLMB_MAX 8
param set-default FW_T_SINK_MAX 2.7
param set-default FW_T_SINK_MIN 2.2

param set-default FW_W_EN 1

param set-default MIS_TAKEOFF_ALT 30

param set-default NAV_ACC_RAD 15
param set-default NAV_DLL_ACT 0 #2

param set-default RWTO_TKOFF 1

param set-default CA_AIRFRAME 1

param set-default CA_ROTOR_COUNT 1
param set-default CA_ROTOR0_PX 0.115

param set-default CA_SV_CS_COUNT 4
param set-default CA_SV_CS3_TYPE 4
param set-default CA_SV_CS2_TYPE 3
param set-default CA_SV_CS1_TYPE 2
param set-default CA_SV_CS0_TRQ_R -0.5
param set-default CA_SV_CS0_TYPE 1
param set-default CA_SV_CS1_TRQ_R 0.5
param set-default CA_SV_CS1_TYPE 2
param set-default CA_SV_CS2_TRQ_P 1
param set-default CA_SV_CS2_TYPE 3
param set-default CA_SV_CS3_TRQ_Y 1
param set-default CA_SV_CS3_TYPE 4
param set-default CA_SV_CS4_TYPE 9
param set-default CA_SV_CS5_TYPE 10
param set-default PWM_MAIN_FUNC3 204
param set-default PWM_MAIN_FUNC4 205
param set-default PWM_MAIN_FUNC5 101
param set-default PWM_MAIN_FUNC6 201
param set-default PWM_MAIN_FUNC7 202
param set-default PWM_MAIN_FUNC8 203
param set-default PWM_MAIN_FUNC9 206
param set-default PWM_MAIN_REV 256

param set-default COM_DISARM_PRFLT 0.0
param set-default COM_DISARM_LAND 0.0
EOF
```

**5.2 Register them** in `$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`, inside the `px4_add_romfs_files(...)` block (keep numerical order):

```cmake
px4_add_romfs_files(
	...
	1041_gazebo-classic_tailsitter
	...
	1046_gazebo-classic_easyglider    # <-- add
	...
	10020_gazebo-classic_shuttle      # <-- add
	...
)
```

**5.3 Rebuild PX4:**

```bash
cd $PX4_DIR
DONT_RUN=1 make px4_sitl gazebo-classic
```

### 6. VioLET Workspace

```bash
mkdir -p ~/VioLET/src
cd ~/VioLET/src
git clone --recursive https://github.com/abrantessss/VioLET.git
# already cloned without --recursive?  ->  git submodule update --init --recursive

cd ~/VioLET
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "source ~/VioLET/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> Building `px4_msgs` takes a few minutes the first time.

### 7. MAVSDK backend (optional)

The default pipeline uses uXRCE-DDS and does **not** need MAVSDK. To also build the MAVLink backend (`*_mavlink` launch files), install [MAVSDK 2](https://github.com/mavlink/MAVSDK/releases) (`.deb` for Ubuntu 22.04) and rebuild:

```bash
sudo dpkg -i libmavsdk-dev_<version>_ubuntu22.04_amd64.deb
cd ~/VioLET
colcon build --symlink-install --cmake-args -DBUILD_MAVLINK=ON
```

---

## Running a Simulation

Each launch file starts Gazebo Classic, the PX4 SITL instance(s), the Micro XRCE-DDS agent, `violet_interface` and `violet_autopilot`.

```bash
# Quadrotor (Capture Shuttle)
ros2 launch violet shuttle.launch.py

# Fixed-wing (EasyGlider)
ros2 launch violet easyglider.launch.py

# Both vehicles in the same world
ros2 launch violet combined.launch.py

# Headless (no Gazebo GUI)
ros2 launch violet shuttle.launch.py gui:=false
```

Vehicle parameters are set in `violet/config/*.yaml`; pass a different file with `config_yaml:=/path/to/file.yaml`.

In a second terminal, open the console and/or run a mission script:

```bash
ros2 run violet_console violet_console

ros2 run violet_scripts shuttle_circle
ros2 run violet_scripts easyglider_lemniscate
```

Visualization in RViz 2:

```bash
ros2 launch violet_rviz violet_rviz.launch.py
```

Each vehicle runs in its own namespace (`/drone1`, `/drone2`, ...) and its PX4 working files are written to `~/tmp/px4_dir/<namespace>`.

---

## Acknowledgements

- [Pegasus Project](https://github.com/PegasusResearch) — inspiration for the framework architecture.
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) and [px4_msgs](https://github.com/PX4/px4_msgs).
- [rviz_2d_plot_plugin](https://github.com/amgaber95/rviz_2d_plot_plugin).
### Compare LOS4 controllers in wind

Both runs below use `helipad_wind.world` (5 m/s from north) and matching
controller gains/limits. Restart the simulation between runs and use the same
mission script and parameters.

```bash
# Updated controller: true airspeed energy feedback and estimated-wind curvature compensation
ros2 launch violet easyglider_wind.launch.py controller:=los4_wind

# Baseline controller: ground-speed energy feedback and original curvature feedforward
ros2 launch violet easyglider_wind.launch.py controller:=los4
```

In a second terminal, run the same mission for each experiment, for example:

```bash
ros2 run violet_scripts easyglider_lemniscate
```

Mission recording is enabled by default. Bags under `violet_plots/bags` start
with `los4_wind_lemniscate_...` or `los4_lemniscate_...`, respectively. Both record
`/<vehicle_ns>/fmu/out/wind` and
`/<vehicle_ns>/fmu/out/airspeed_validated_v1` whenever those messages arrive,
including their validity/source fields. The baseline records these sensors but
does not use them in its control law. A `config_yaml:=...` override takes
precedence over the controller argument; keep the two configurations' gains
matched when tuning a comparison.
