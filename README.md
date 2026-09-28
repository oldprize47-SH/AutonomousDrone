# Autonomous Drone Development

This is the team archive for our 2026 autonomous-vehicle control project. The platform combines an Arduino Uno Q flight controller, onboard sensing, a ground-control station and a camera for helipad detection.

![The team's quadrotor during field testing](assets/drone-photos/drone_field_01.jpg)

[Watch the flight demonstration](https://www.youtube.com/watch?v=GLonDTGTSmQ).

## My work

I was responsible for the camera and vision work: collecting and labelling images, comparing lightweight models, preparing embedded inference, testing detection and tracking approaches, and checking relative-position and validity information supplied to the flight-control team. I used AI coding tools during implementation and experiments.

The detector and tracker implementation is in my separate [RealSense vision repository](https://github.com/oldprize47-SH/realsense-drone-vision). This archive contains the [companion bridge](flight-controller/companion-vision-bridge/main.py), which starts the separately deployed tracker and forwards its output.

The complete flight controller, ground-control station and aircraft are not my individual work. This fork preserves the original team archive prepared by Kim Yejoon and its author history.

## Understanding the archive

There are several separate parts to the team system. The onboard firmware reads sensors and runs the aircraft's control logic. The ground-control station is the desktop interface used to communicate with the system and inspect its state. The camera pipeline supplies target observations through a companion bridge. A successful test of one part is not automatically a successful integrated flight.

For my contribution, begin with the linked RealSense repository, then read the companion bridge here to see where the vision output enters the team system. For the wider project, start with the final report and compare its discussion with the saved experiment plots. The firmware and ground-station folders provide implementation context, with the original team authorship preserved.

During the vision work, the practical issue was not only detecting a helipad in an image. I also needed to prepare data, choose a model suited to the onboard computer, compare detection and tracking approaches, and check that the relative-position information and validity state were usable by the flight-control team. The team's final landing result remains separate from the offline detector evaluation.

## Results and files

The team approached the landing target but did not achieve accurate marker-centre landing. Offline detection accuracy should not be interpreted as autonomous-landing success.

The repository includes [firmware](flight-controller), [ground-control station source](ground-control-station), [experiment results](development-report/experiment-results), the [final report](development-report/final-report/AVC_26S_Final_Report.pdf) and a supporting [complementary-filter study](complementary-filter).

![Recorded altitude command tracking](development-report/experiment-results/altitude-control/altitude_command_tracking.png)

This is a recorded team result included in the original report. It illustrates the difference between command tracking in the experiment and simulation; it is not a measurement made for this portfolio update.

## Build notes

The Windows ground-control station uses C++20, CMake and ImGui. Its original build commands are:

```sh
cd ground-control-station
cmake -S . -B build
cmake --build build --config Release
```

These commands were not rerun for this documentation change. Firmware and camera deployment require the original board environment and compatible dependencies. Model weights and private calibration are not supplied by the companion bridge.

Original archive author: Kim Yejoon, Autonomous Vehicle Control project.
