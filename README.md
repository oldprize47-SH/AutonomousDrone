# Autonomous Drone Development

**A team-built quadrotor platform. My portfolio focus is its camera and vision pipeline.**

[▶ Flight demonstration](https://www.youtube.com/watch?v=GLonDTGTSmQ) · [My vision implementation](https://github.com/oldprize47-SH/realsense-drone-vision) · [Full engineering archive](#overview)

![Team quadrotor field prototype](assets/drone-photos/drone_field_01.jpg)

| Platform | My responsibility | Observed outcome |
|---|---|---|
| Arduino Uno Q, onboard sensing and ground station | Helipad data, lightweight vision models, embedded inference and target validity | Approached the target; accurate marker-centre landing was not achieved |

## Sangheon Park — contribution to this team project

My responsibility was the camera and vision work: collecting and labelling helipad
images, comparing and selecting lightweight models, preparing embedded inference,
comparing detection/tracking approaches, and checking the relative-position and
validity information supplied to the flight-control team. I used AI coding tools
for implementation and experiments. The complete flight controller, ground-control
station and every source file in this archive are not my individual work.

This fork preserves the original team archive prepared by Kim Yejoon. The original
author credit below is retained; this section identifies my contribution for
portfolio readers.

**Start with my area:** [vision landing results](development-report/experiment-results/vision-landing)
and the [companion bridge](flight-controller/companion-vision-bridge/main.py).
The bridge starts a separately deployed `track_helipad.py`; that detector/tracker
implementation is maintained in my separate
[vision portfolio repository](https://github.com/oldprize47-SH/realsense-drone-vision/blob/main/README.en.md).
That repository includes the detector/tracker source, while model weights, private
calibration and target-board dependencies are separate requirements. The bridge
alone is not a self-contained reproduction of the vision pipeline.

**Result boundary:** the team approached the landing target but did not achieve
accurate marker-centre landing. Detection examples and offline accuracy do not
establish successful autonomous landing.

## Overview

This repository reorganizes the main development artifacts from an autonomous quadrotor project into a report-style engineering archive. The project integrates an Arduino UNO Q based flight controller, a Windows C++ ground-control station, onboard sensing, telemetry, mission guidance, and experimental validation data.

The original course folder contained assignments, robot-arm work, raw logs, CAD archives, and generated files. This repository keeps only the materials needed to explain and reproduce the autonomous drone development flow:

- flight-controller firmware and companion vision bridge
- GCS source code and waypoint tooling
- final report diagrams and representative experiment results
- complementary-filter study used as supporting sensor-fusion background

## Demo

- [Autonomous drone flight demo](https://www.youtube.com/watch?v=GLonDTGTSmQ)

## System Snapshot

![Hardware architecture](development-report/system-architecture/HW_Diagram_1.png)

The system is organized around three communicating layers:

| Layer | Main Artifacts | Role |
|---|---|---|
| Quadrotor MCU | `flight-controller/unoq-firmware` | 200 Hz flight-control loop, RC/PWM handling, navigation state, control modes, telemetry |
| UNO Q Linux side | `flight-controller/companion-vision-bridge` | Runs helipad vision tracker and forwards target state to the MCU through Arduino Bridge |
| Ground Control Station | `ground-control-station` | Serial/UDP telemetry, waypoint mission upload, NTRIP correction, OptiTrack receive, ImGui dashboard, CSV logging |

![GCS structure](development-report/system-architecture/GCS_Struc.png)

Field prototype:

| Front payload view | Sensor and onboard stack |
|---|---|
| ![Drone front payload view](assets/drone-photos/drone_field_02.jpg) | ![Drone sensor stack](assets/drone-photos/drone_field_03.jpg) |

## Repository Structure

```text
.
├─ flight-controller/
│  ├─ unoq-firmware/              # Arduino UNO Q MCU firmware
│  ├─ companion-vision-bridge/    # QRB2210/Linux-side helipad vision bridge
│  └─ docs/                       # Board migration and operation notes
├─ ground-control-station/        # C++/CMake/ImGui GCS
├─ development-report/
│  ├─ final-report/               # Final PDF report
│  ├─ system-architecture/        # System diagrams
│  └─ experiment-results/         # Curated result figures and summary CSVs
├─ complementary-filter/          # Supporting altitude/complementary-filter study
└─ assets/drone-photos/           # Field test photos
```

## Flight Controller

The flight controller is implemented under `flight-controller/unoq-firmware`. It targets the STM32U585 MCU on the Arduino UNO Q and coordinates sensing, state estimation, command handling, and motor output.

Key modules:

| Module | Responsibility |
|---|---|
| `sketch.ino` | Arduino entrypoint and top-level initialization |
| `hw_config.h` | Pin map, control frequency, telemetry frequency, gains, safety thresholds, mission constants |
| `Mode.cpp/h` | Main mode/state logic for RC control, velocity, PNG guidance, position control, auto takeoff/landing, mission execution |
| `Autopilot.cpp/h` | Attitude, altitude, climb-rate, velocity, position control, and thrust mixing |
| `INSS.cpp/h` | Navigation and sensor-state handling |
| `Telem.cpp/h` | Telemetry and debug-frame packing |
| `QuadPWM.cpp/h` | Motor PWM output |
| `RCInput.cpp/h` | RC channel input handling |
| `CollisionCone.cpp/h` | Obstacle-avoidance guidance logic |
| `filter.cpp/h` | Barometer altitude conversion, complementary filter, fading memory filter |

The firmware uses a 200 Hz control loop and 20 Hz telemetry/debug downlink. The mode logic combines manual RC control with autonomous behaviors such as waypoint guidance, auto takeoff, auto landing, and vision-assisted landing.

![Flight state diagram](development-report/system-architecture/State.png)

## Companion Vision Bridge

`flight-controller/companion-vision-bridge/main.py` runs on the UNO Q Linux/container side. It starts a helipad tracking process, reads the latest JSON telemetry packet, and forwards compact landing-target data to the MCU:

```text
helipad tracker -> latest JSON -> Bridge.notify("set_helipad_target", valid, x, y, xy, gnd)
```

This separates camera/vision workload from the MCU control loop while still allowing the landing controller to consume fresh target offsets.

## Ground Control Station

The GCS is a Windows C++20/CMake project in `ground-control-station`.

Key modules:

| Module | Responsibility |
|---|---|
| `gcs_main.cpp` | Main serial/UDP loop, keyboard commands, mission receive/save, CSV logging |
| `src/p8_comm.cpp`, `include/p8_comm.h` | P8 telemetry parser, debug frame parser, command packet sender |
| `src/dashboard.cpp`, `include/dashboard.h` | ImGui/OpenGL realtime dashboard |
| `src/ntrip_client.cpp` | NTRIP correction client |
| `src/OptiTrack.cpp` | OptiTrack UDP receiver |
| `WayPoint/*.txt` | Mission waypoint files |

The CMake project vendors only the minimum Dear ImGui files needed by the dashboard build. If the vendored folder is missing, the existing `CMakeLists.txt` can fetch ImGui v1.91.5.

Basic build on Windows:

```powershell
cd ground-control-station
cmake -S . -B build
cmake --build build --config Release
```

## Experiment Results

Representative result figures were reorganized from the final report and source logs into `development-report/experiment-results`. The raw CSV dump is intentionally excluded; this repo keeps the plots, summary tables, and report-extracted figures needed to explain the development result.

### Altitude Control

The altitude controller was evaluated by comparing command tracking, controller output signals, and internal P/I terms between the flight experiment and simulation. The report notes that the controller followed the command trend, while the real response included more low-frequency drift and sensor/environment-driven fluctuation than simulation.

![Altitude command tracking](development-report/experiment-results/altitude-control/altitude_command_tracking.png)

![Altitude controller outputs](development-report/experiment-results/altitude-control/altitude_controller_outputs.png)

### Attitude Control

The attitude-control validation focuses on roll/pitch/yaw-rate response and internal controller signals. The experimental response follows the commanded direction, but residual error, vibration, and internal command variation are larger than in simulation, showing where model refinement is still needed.

![Yaw-rate response](development-report/experiment-results/attitude-control/yaw_rate_response.png)

![Yaw-rate internal signals](development-report/experiment-results/attitude-control/yaw_rate_internal_signals.png)

### Position And Guidance Control

The mission controller combines position feedback, PNG waypoint guidance, circular guidance, and collision-avoidance logic. The final report evaluates waypoint approach, yaw-rate tracking, forward velocity tracking, circular-path tracking, and obstacle-avoidance behavior.

![PNG guidance summary](development-report/experiment-results/position-guidance-control/png_summary.png)

![Circular guidance path](development-report/experiment-results/position-guidance-control/circular_guidance_path.png)

![Circular mission analysis](development-report/experiment-results/position-guidance-control/circular_mission_analysis.png)

The final mission summary records mission duration, guidance time, landing time, waypoint count, range metrics, loop timing, RTK/GNSS availability, vision usage, and state transitions:

- `development-report/experiment-results/position-guidance-control/Final_mission_validation_summary.csv`

### Vision Landing

The vision-landing pipeline detects and tracks the helipad with the UNO Q Linux-side vision process, then forwards target offsets to the MCU landing controller. The curated assets include the report pipeline diagram, dataset/labeling examples, detection examples, and final vision landing summary.

![Vision landing pipeline](development-report/experiment-results/vision-landing/vision_landing_pipeline.png)

![Helipad detection examples](development-report/experiment-results/vision-landing/helipad_detection_examples.jpeg)

- `development-report/experiment-results/vision-landing/Final_vision_landing_summary.csv`

## Complementary Filter Study

The `complementary-filter` folder preserves the supporting MATLAB/Simulink work used to study altitude estimation from barometer and IMU-related data.

![Floor altitude average](complementary-filter/docs/all_floor_average.png)

Included materials:

- MATLAB scripts for complementary-filter analysis
- Simulink models for filter experiments
- sampled pressure/altitude logs
- floor/barometer result plots
- original presentation slide deck

This section is kept as a technical appendix rather than a separate project because it supports the altitude-estimation and sensor-fusion logic used by the drone controller.

## Report

The final project report is archived here:

- `development-report/final-report/AVC_26S_Final_Report.pdf`

The report covers project goals, mission definition, hardware design, dynamics modeling, parameter modeling, controller design, mission implementation, and experimental validation.

## Notes On Excluded Files

The original working directory contained raw flight logs, large CAD/STEP/STL files, generated build folders, executable artifacts, backup folders, and unrelated course/project materials. They were intentionally excluded from this repository to keep it focused and GitHub-friendly.

Excluded categories include:

- full raw CSV log dump
- large CAD archives and duplicated zip files
- generated build folders and IDE metadata
- executable/runtime binaries
- unrelated assignments and robot-arm project files

## Author

Kim Yejoon - Autonomous Vehicle Control project archive.
