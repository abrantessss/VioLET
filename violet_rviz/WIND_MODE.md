# Real fixed-wing wind mode

Pi option `--wind false` is the default. It preserves the original ground-speed
energy control and curvature feedforward. Airspeed/wind telemetry is collected
and forwarded in either mode, independently of arming and flight mode.

`--wind true` selects the LOS4WindController law from violet_autopilot:
- kinetic-energy error uses measured calibrated airspeed;
- turn curvature feedforward uses reference airspeed, horizontal estimated wind,
  path tangent and measured ground velocity;
- missing/stale wind (2 s) falls back to baseline curvature feedforward;
- infeasible forward path bearing disables curvature feedforward, leaving bounded
  direction feedback. There is no excess-wind recovery;
- gamma, position guidance, attitude feedback, derivative filters, output limits
  and the RC-controlled Offboard sequence stay unchanged.

`--speed` is the reference airspeed when wind is enabled. Ground speed is allowed
to vary. The existing 1 m/s horizontal-ground-speed activation guard still applies.

## Telemetry source

The Pi uses `Telemetry::subscribe_fixedwing_metrics()` and requests 10 Hz with
`Telemetry::set_rate_fixedwing_metrics()`. Its `airspeed_m_s` field carries PX4
calibrated airspeed (CAS) from VFR_HUD. Ground velocity remains the NED vector
from LOCAL_POSITION_NED; horizontal speed is sqrt(vN^2 + vE^2). No pressure/temperature
conversion or HIGHRES_IMU subscription is used. CAS must be finite, nonnegative
and younger than 1 s. SYS_STATUS must report the differential pressure sensor
present, enabled and healthy with status younger than 3 s. These checks do not
expose every internal PX4 airspeed-selector validity field. Wind-mode control
requires valid CAS; otherwise the existing inhibit/fault/handback logic applies.
`--speed` is now a CAS reference in wind mode. The wind-triangle calculation uses
that reference as the low-altitude approximation to air-relative speed; this
is not density-corrected TAS compensation at arbitrary altitude.

WIND_COV supplies N/E estimates; its horizontal variance must be finite and
positive (PX4 encodes the sum of horizontal estimator variances). Zero estimated
wind with positive variance is valid. Vertical wind is unknown, not zero.

## ROS topics and recording

Under the vehicle namespace (default `/drone1`):

| Topic | Type | Data |
|---|---|---|
| telemetry/airspeed | Float32 | Valid CAS [m/s], NaN when invalid |
| telemetry/calibrated_airspeed | Float32 | Latest PX4 CAS [m/s]; consult validity |
| telemetry/wind | Vector3Stamped | N/E wind [m/s], Z NaN; invalid estimate N/E NaN |
| telemetry/wind_enabled | Bool | Selected controller mode |
| telemetry/airspeed_valid | Bool | Fresh, healthy air data |
| telemetry/wind_valid | Bool | Fresh usable wind estimate |
| telemetry/los4/environment | Float32MultiArray | Complete environment packet |
| rviz_plots/velocity/airspeed | Float32 | Valid CAS samples |
| rviz_plots/wind/speed | Float32 | Valid horizontal wind magnitude |
| rviz_plots/wind/north, east | Float32 | Valid horizontal wind components |

The speed graph contains ground-speed norm, calibrated airspeed, desired speed and gamma
speed. N/E/D speed components remain available as topics but are removed from that
graph. A separate wind graph shows magnitude and N/E components. Missing data
produces no new plot samples; it is not plotted as zero.

`record_bag:=true` records all these topics with the existing namespace regex into
`violet_plots/bags/real_fixedwing_test_data_<timestamp>/`. Default remains false.

Wire packet: standard TUNNEL payload type 32768, magic VLOS. Existing config/state
packets are unchanged. Environment kind 3 version 2 has 12 float32 values:
`[2, 3, wind_enabled, NaN, CAS, wind_N, wind_E, air_age_s, wind_age_s,
air_valid, wind_valid, curvature_status]`.
Curvature status: 0 baseline/off, 1 wind triangle available, 2 infeasible bearing.
It describes feedforward availability, not whether Offboard is active.

## Copy and build

On the computer:
```bash
scp /home/abrantess/easyglider/pf_los4/src/{pf_los4.cpp,los4_config.hpp,los4_core.hpp,los4_runtime.cpp,los4_runtime.hpp,los4_air_data.hpp,los4_diagnostics.hpp} \
  capture@192.168.1.38:~/MAVSDK/CAPTURE/pf_los4/src/
```
On the Pi, stop pf_los4 before rebuilding:
```bash
cd ~/MAVSDK/CAPTURE/pf_los4
cmake --build build -j1
```
Append `--wind true` or `--wind false` to the usual Pi command. Live control still
needs `--controller-enabled true --test-gamma false --offline false` and the
existing RC/flight readiness conditions.

Rebuild the computer's violet_rviz package and source the resulting install before
launching the usual RViz command with `record_bag:=true` if recording is wanted.

Version 1 environmental packets remain readable. Their CAS is at index 4 too.
Previously recorded rviz_plots/velocity/airspeed samples retain their old derived
TAS meaning; replay does not recalculate them. New CAS recordings use version 2.
