# Flight Controller

This folder contains the Arduino UNO Q flight-controller side of the autonomous quadrotor system.

## Contents

| Path | Description |
|---|---|
| `unoq-firmware/` | STM32U585 MCU firmware for sensing, control, telemetry, PWM, RC input, mission logic, and autonomous modes |
| `companion-vision-bridge/` | UNO Q Linux/container-side app that forwards helipad vision target data to the MCU |
| `docs/` | Board migration, UNO Q porting, and USB host boot notes |

## Firmware Focus

The MCU firmware is centered on a fixed-rate flight-control loop. The major logic paths are split into mode management, autopilot control, sensor/navigation state, telemetry, and actuator output.

The most important source files are:

- `unoq-firmware/Mode.cpp`: mode transitions and autonomous mission behavior
- `unoq-firmware/Autopilot.cpp`: attitude, altitude, velocity, position, and thrust mixing control
- `unoq-firmware/filter.cpp`: barometer altitude and complementary/fading-memory filtering
- `unoq-firmware/Telem.cpp`: telemetry/debug-frame protocol
- `unoq-firmware/hw_config.h`: board wiring, control constants, and safety thresholds

## Companion Bridge

The companion app runs a helipad tracker, reads the latest tracking packet, and sends compact target information to the MCU through Arduino Bridge. This lets the Linux side handle vision while the MCU keeps the realtime control loop small.
