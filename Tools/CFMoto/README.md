# CFMOTO CFORCE 520L reference profile

This directory records hardware facts recovered from the earlier ArduRover
4.6.2 installation.  Treat the values as bench-test starting points, not as
proof that a mechanism is safe to move.

## Relay contract

| Relay | Function | Pixhawk pin |
|---:|---|---:|
| 1 | ignition (MAIN5) | 105 |
| 2 | starter (MAIN6) | 106 |
| 3 | four-wheel drive (AUX3) | 52 |
| 4 | differential lock (AUX4) | 53 |
| 5 | disabled | -1 |
| 6 | disabled | -1 |

All relays default OFF.  Assign operator RC channels only after confirming the
final transmitter layout.  Do not copy the legacy RC1/RC3 assignments: those
channels are steering and throttle in the current Rover input map.
Because the relay GPIOs share pins with MAIN5/6 and AUX3/4, keep
`SERVO5/6_FUNCTION` and `SERVO11/12_FUNCTION` set to `-1`.

## Gearbox reference positions

The earlier installation stored five actuator positions:

```text
1000, 1225, 1500, 1725, 2000 microseconds
```

The profile maps these in the presumed selector order P/R/N/H/L.  Verify every
position with the actuator disconnected from the selector first, then confirm
the real gear through the Nano telemetry before enabling `MAUL_ENABLE`.

The recovered selector inputs are active-low. D3 through D7 normally read
HIGH and exactly one selected gear input is pulled LOW. The firmware defaults
therefore decode P/R/N/H/L as `0xF0/0xE8/0xD8/0xB8/0x78` under mask `0xF8`.

The CFMOTO shift controller uses an explicit sequence: apply the service
brake, confirm stopped conditions continuously, pass through Neutral when
moving between two non-Neutral positions, dwell, command the target, and
verify telemetry. A sequence timeout latches a fault; requesting Neutral while
stopped clears it through the same feedback check.

## Engine and driveline state machines

`MAUL_ENG_EN` enables a guarded ignition/starter sequence on relays 1 and 2.
Starting requires an armed vehicle, neutral throttle, stopped speed, P or N,
valid gearbox feedback, and a configured engine RPM input. The starter has an
ignition delay, maximum crank time, retry delay and retry limit; reaching the
running RPM threshold releases it immediately. Relay 1 is MAIN5 and relay 2 is
MAIN6. Keep the feature disabled until relay polarity and engine RPM identity
are measured.

`MAUL_4WD_CH` and `MAUL_DIFF_CH` control relays 3 and 4 only while throttle,
speed and optional RPM interlocks are safe. Differential lock is allowed only
with 4WD enabled. Failsafe retains the existing driveline state instead of
switching a loaded mechanism. The channels default to zero because the legacy
RC1/RC3 assignments conflict with current Rover steering/throttle inputs.

`MAUL_LIGHT_CH` and `MAUL_EW_CH` are retained only for parameter
compatibility and no longer drive outputs because AUX3/AUX4 belong to the
driveline.

## Frequency inputs

The recovered installation used the last two FMU/AUX outputs as rising-edge
frequency inputs:

| Connector | GPIO | Parameter set | Recovered scaling |
|---|---:|---|---:|
| AUX5 | 54 | RPM1 | 0.00058 |
| AUX6 | 55 | RPM2 | 1.0 |

`SERVO13_FUNCTION=-1` and `SERVO14_FUNCTION=-1` reserve AUX5 and AUX6 as
GPIO instead of PWM outputs. Both RPM instances use `TYPE=2` (GPIO), the
standard ArduPilot interrupt-based frequency backend.

The firmware reports:

```text
reported RPM = scaling * input frequency in Hz * 60
```

The old parameter file does not identify which wire is engine RPM and which is
vehicle/driveline speed. Confirm both signals and their units on the bench
before using either input in a gearbox interlock. The recovered
`RPMx_MIN_QUAL=0` is retained for compatibility but is not a sufficient
health threshold for a safety decision.

## Other recovered hardware settings

- The ELRS receiver is connected to TELEM1, which is `SERIAL1` on Pixhawk 6C.
  Configure `SERIAL1_PROTOCOL=23` (RC input), `SERIAL1_OPTIONS=0`,
  `RSSI_TYPE=3`, and `RC_OPTIONS=8224` (ELRS 420 kbaud plus idle-throttle
  arming).  ArduPilot controls the UART baud rate automatically for serial RC
  protocols; `SERIAL1_BAUD` is not used to select 420 kbaud in this mode.
- Wire the full-duplex CRSF link:

  ```text
  ELRS TX -> TELEM1 RX
  ELRS RX -> TELEM1 TX
  ELRS 5V -> TELEM1 5V
  ELRS GND -> TELEM1 GND
  ```

  Verify the receiver's supply requirements before using the TELEM1 5 V rail.
- The old Pixhawk mounting rotation was roll -90, pitch 0, yaw 180.  Recalibrate
  the actual board instead of copying IMU or compass offsets.
- Never copy the old `ARMING_REQUIRE=0`, `ARMING_CHECK=0`,
  `BRD_SAFETY_DEFLT=0`, or direct RC passthrough output configuration.

Load `cforce520-reference.param` only while actuator power is disconnected.
The file keeps `MAUL_ENABLE=0` and gearbox telemetry disabled for calibration.
