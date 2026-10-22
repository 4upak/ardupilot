# CFMOTO CFORCE 520L reference profile

This directory records hardware facts recovered from the earlier ArduRover
4.6.2 installation.  Treat the values as bench-test starting points, not as
proof that a mechanism is safe to move.

## Relay contract

| Relay | Function | Pixhawk pin |
|---:|---|---:|
| 1 | ignition | 50 |
| 2 | starter | 51 |
| 3 | headlights | 52 |
| 4 | electronic-warfare payload | 53 |
| 5 | differential lock | 105 |
| 6 | four-wheel drive | 106 |

All relays default OFF.  Assign operator RC channels only after confirming the
final transmitter layout.  Do not copy the legacy RC1/RC3 assignments: those
channels are steering and throttle in the current Rover input map.

## Gearbox reference positions

The earlier installation stored five actuator positions:

```text
1000, 1225, 1500, 1725, 2000 microseconds
```

The profile maps these in the presumed selector order P/R/N/H/L.  Verify every
position with the actuator disconnected from the selector first, then confirm
the real gear through the Nano telemetry before enabling `ATV_ENABLE`.

## Other recovered hardware settings

- RPM inputs used pins 54 and 55.  Identify engine versus driveline input and
  calibrate scaling before using either value in shift interlocks.
- The ELRS receiver used `RC_OPTIONS=8224`: 420 kbaud plus idle-throttle arming.
- The old Pixhawk mounting rotation was roll -90, pitch 0, yaw 180.  Recalibrate
  the actual board instead of copying IMU or compass offsets.
- Never copy the old `ARMING_REQUIRE=0`, `ARMING_CHECK=0`,
  `BRD_SAFETY_DEFLT=0`, or direct RC passthrough output configuration.

Load `cforce520-reference.param` only while actuator power is disconnected.
The file keeps `ATV_ENABLE=0` and gearbox telemetry disabled for calibration.
