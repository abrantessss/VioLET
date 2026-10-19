# Stationary RC Offboard test

Use --bench-offboard true to prepare/maintain Offboard without requiring airborne
status, local position or the LOS4 minimum ground speed. It still requires an
armed fixed-wing PX4, fresh heartbeat/attitude/RC, kill OFF, and an RC Manual cycle
before high/confirmed Offboard. Controller hard-disable and gamma-test overrides
still apply. It never requests arming or Offboard entry.

This option does NOT run the LOS4 controller at zero speed. It enforces zero
commanded thrust and zero commanded body rates at the MAVLink send boundary,
and freezes gamma at gamma0. Servos can still respond to PX4 rate stabilization;
zero thrust is not a hardware motor kill. Remove the propeller for armed bench
checks. The physical kill switch remains available and stops the stream.

Copy changed desktop sources to Pi:

```bash
cd /home/abrantess/Downloads/capture-pi-saved/fil2k
scp MAVSDK/CAPTURE/pf_los4/src/{pf_los4.cpp,los4_config.hpp,los4_runtime.cpp} \
  capture@192.168.1.38:~/MAVSDK/CAPTURE/pf_los4/src/
```

On Pi, stop the previous executable and rebuild:

```bash
cd ~/MAVSDK/CAPTURE/pf_los4
cmake --build build -j1
./build/pf_los4 serial:///dev/ttyAMA0:115200 10 udp://192.168.1.94:14540 \
  --controller-enabled true --test-gamma false --offline false \
  --bench-offboard true
```

Arm through RC in Manual, keep channel 6 kill OFF, and wait >=1.2 s for the
preparation stream. Select channel 7 HIGH. Once PX4 reports Offboard, console
shows BENCH_OFFBOARD_ACTIVE. Low/mid returns Manual; kill, disarming or stale
attitude/RC/heartbeat stops setpoints. If PX4 still refuses, check its rejection
message: its own sensor/preflight requirements are not bypassed.

Normal --bench-offboard false (default) still requires airborne status and the
configured minimum horizontal speed for real LOS4. Do not fly in bench mode.
VLOS diagnostics phase 8 means bench Offboard active; desired-attitude validity
is cleared, because no LOS4 attitude reference is being calculated.
