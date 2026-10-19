# pf_los4 — Pi PX4 fixed-wing LOS4 controller and telemetry relay

Standalone C++ MAVSDK 2 package in FIL2K/MAVSDK/CAPTURE/pf_los4.
Camera streaming remains independent (rpicam-vid on Raspberry Pi OS).

## Compile on Raspberry Pi

MAVSDK 2 development files for the Pi architecture must already be installed.

```bash
sudo apt install cmake g++ libeigen3-dev
cd ~/MAVSDK/CAPTURE/pf_los4
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

If MAVSDK is installed under a custom prefix, add
`-DMAVSDK_DIR=/actual/pi/path/lib/cmake/MAVSDK` to the configure command.
Unit tests need Eigen3. Loopback integration tests are registered when Python
and pymavlink are installed. Never run loopback tests against an aircraft.

## Run (safe defaults)

The original three positional arguments remain compatible:

```bash
./build/pf_los4 serial:///dev/ttyAMA0:115200 10 udp://192.168.1.94:14540
```

This only observes/forwards telemetry. Controller enable defaults to false.
The forwarding address is the desktop IP. RC mode channel defaults to 7, emergency kill channel to 6.
The return mode defaults to manual, matching the RC flight-mode selection.
All config defaults are visible in `src/los4_config.hpp`; all CLI options are in
`./build/pf_los4 --help`. Options can appear before or after positional arguments.
Values use `--option value`, including explicit true/false booleans.

## Path settings (local NED metres; speed m/s)

Common options: `--path line|circle|lemniscate --speed 14 --gamma0 0`.
Defaults: circle, radius 50 m, centre [0,0,-30], speed 14 m/s, gamma0=0.

```bash
# Line, pd = p0 + gamma*(p1-p0); infinite line (not an automatic endpoint stop).
./build/pf_los4 --path line --p0 0,0,-30 --p1 200,0,-30 --speed 14

# Circle, pd = centre + [R*cos(gamma), R*sin(gamma), 0].
./build/pf_los4 --path circle --center 0,0,-30 --radius 50 --speed 14

# Gerono lemniscate, matching Mellinger (not Bernoulli).
# pd = centre + [a*cos(gamma), a*sin(gamma)*cos(gamma), 0].
./build/pf_los4 --path lemniscate --center 0,0,-30 --scale 50 --speed 14
```

NED origin is the flight controller's local estimator origin, not the point where
the RC switch was flipped. Negative Down is above that origin. Gain options include
k1/k2, kpE/kiE, kpB/kiB, ka, kphi/ktheta, mass and Tmax; limits and rate options are
also configurable. Malformed, nonfinite or degenerate path settings are rejected.

## Hard inhibit and gamma-only test

`--controller-enabled false` prevents this controller from transmitting any
setpoint or mode-change command. It is a process configuration, not an RC motor
kill: it does not disarm the vehicle or block other MAVLink applications.
To change process configuration, stop/restart it. Stop an active run with Ctrl-C
first so it can attempt handback.

`--test-gamma true` overrides controller enable and always inhibits commands.
Gamma integrates gamma_dot=1 in radians/second from gamma0, independently of
arming/airborne state and RC. It prints gamma, gamma_dot and pd once per second.
This is not yet connected to the RViz path/gamma displays; that is a later step.

```bash
# Gamma test with telemetry/forwarding still available:
./build/pf_los4 serial:///dev/ttyAMA0:115200 10 udp://192.168.1.94:14540 \
  --test-gamma true --path circle --radius 50

# Gamma test without Pixhawk (no MAVLink connection or forwarding):
./build/pf_los4 --test-gamma true --offline true --path lemniscate --scale 50
```

## RC activation and Offboard sequence

Configuration now matches QGroundControl: arm switch channel 5 (PX4 owns arming),
emergency kill channel 6 HIGH, flight mode channel 7 low/mid Manual and high
Offboard. Return mode now defaults to manual. Controller enable remains false;
your current test/offline defaults are preserved. Explicit CLI flags below select
whether this is an observer, test or live controller run.

```bash
# Observe/forward only:
./build/pf_los4 serial:///dev/ttyAMA0:115200 10 udp://192.168.1.94:14540 \
  --controller-enabled false --test-gamma false --offline false

# Gamma-only test with RViz forwarding, no control commands:
./build/pf_los4 serial:///dev/ttyAMA0:115200 10 udp://192.168.1.94:14540 \
  --controller-enabled false --test-gamma true --offline false

# Enable the controller (RC pilot still arms, takes off and selects mode):
./build/pf_los4 serial:///dev/ttyAMA0:115200 10 udp://192.168.1.94:14540 \
  --controller-enabled true --test-gamma false --offline false \
  --rc-channel 7 --kill-channel 6 --kill-active-high true --return-mode manual
```

The Pi requires armed/in-air PX4 status, fresh finite position/attitude/RC and
horizontal speed >=min-speed. With channel 7 low or mid and actual mode Manual,
it prepares a continuous Offboard setpoint stream with gamma/integrators frozen.
Hold Manual for at least 1.2 s before selecting Offboard on RC. PX4 must report
Offboard before LOS4 integrates gamma. The program NEVER requests Offboard entry.
Starting with the mode switch already high is blocked until a Manual cycle.

When channel 7 drops, active LOS4 stops and PX4 follows the RC Manual selection.
If PX4 still reports Offboard, the script requests Manual. Kill HIGH, disarming,
stale/invalid telemetry or numerical/send failures stops setpoints and requests
Manual if needed. Faults require an RC Manual cycle before reactivation. The
script does not clear hardware kill or change PX4 parameters. Ctrl-C releases
active Offboard to Manual with a bounded confirmation attempt.

The preparation stream while Manual is necessary: PX4 requires setpoints before
accepting RC-selected Offboard and continuously thereafter. It ignores the
Offboard control setpoints while in Manual. Like takeoff_squareForever, this
program sends setpoints ahead of mode transition, but it does not arm/take off.
Raw MAVLink rate/throttle messages are sent at the configured loop frequency;
there is no Offboard plugin timer resending stale commands after a fault.

See RVIZ_DIAGNOSTICS.md for current mode display, normalized actuator mapping,
and optional rosbag recording with record_bag:=true.

## Physical vehicle and recovery limits

Controller gains/mass/Tmax default to the supplied Easyglider SIMULATION settings.
The square-root throttle mapping assumes T = Tmax*throttle^2. A real propeller/ESC
must be calibrated; `--throttle-model linear` is also available but likewise needs
calibration. This law uses ground-relative NED velocity, as in the supplied file;
it does not compensate wind or substitute measured airspeed. Hardware gains,
limits and available body-rate Offboard support must be verified for the actual
PX4 version before use. No automatic safety-parameter changes or check bypasses.

On link loss, OS/process failure or unavailable RC, the Pi cannot guarantee an RC
handoff. Setpoints stop and the Pixhawk must use its configured Offboard-loss and
RC-loss failsafes. A mode request is never reported as successful unless confirmed.
See [PX4 Offboard](https://docs.px4.io/v1.14/en/flight_modes/offboard).

Automated checks cover path derivatives, gamma dynamics, finite/low-speed guards,
hard inhibit, test isolation, startup-high switch, activation after OFF->ON,
Offboard prestream/confirmation, switch release and stale-data return requests,
fault latching, pilot takeover and bidirectional forwarding. This is not flight
validation; tests use a fake Pixhawk on local UDP. RViz LOS4 diagnostics and rosbag
recording are not added in this step.
