# RealSense Drone Vision

A depth-assisted helipad tracking and advisory-target pipeline for an autonomous-drone course project. Intel RealSense color/depth frames are converted into a fail-closed target packet and forwarded from an Arduino Uno Q container to the flight-controller bridge.

## Scope and ownership

- **Period:** 2026-1
- **Project:** team autonomous-drone build
- **My primary role:** camera/vision pipeline, RGB/depth target measurement, Uno Q integration, telemetry/Bridge forwarding, and flight-log diagnosis
- **Not claimed here:** airframe construction, full flight-controller ownership, or successful precision landing

Only Sangheon's vision/bridge application files are included. Team FC/STM code, vendor libraries, model weights, raw images/videos, calibration outputs, and flight logs are excluded.

## Architecture

```text
RealSense D435 color + depth
  -> SSDLite/NCNN detection
  -> LK optical-flow tracking
  -> fail-closed LandingGate
  -> depth deprojection + level-body FRD offset
  -> atomic compact telemetry JSON
  -> Uno Q Arduino App bridge
  -> Bridge.notify(set_helipad_target, valid, x, y, xy, ground)
```

`valid=0` is sent when telemetry becomes stale after a previously valid target. Tracker-only predictions cannot authorize the landing advisory gate.

## Repository structure

- `vision/track_helipad.py` — RealSense capture, detection/tracking, LandingGate, depth/FRD measurement, telemetry, CSV/video diagnostics
- `vision/inference.py` — SSD anchor generation, box decoding, NMS, and NCNN diagnostic path
- `bridge/main.py` — Uno Q container entrypoint; supervises the tracker and forwards fresh packets to `Bridge.notify`
- `tests/` — hardware-independent tests with stubbed camera/NCNN/Arduino modules
- `docs/flight-evaluation-summary.md` — bounded summary of the recorded landing-trial analysis

## Verification

The portfolio copy is verified with Python compilation and hardware-independent tests. The tests cover debug metrics, non-blocking recording, fail-closed tracker behavior, packet validity, bridge forwarding, and stale-telemetry invalidation.

The target runtime additionally requires RealSense, NCNN, the trained model artifacts, and the Arduino Uno Q App runtime. Those dependencies are intentionally not redistributed and were not live-rerun during this migration.

## Flight-result boundary

Using a conservative post-hoc operational criterion, only one of six trials with a vision lock was classified as **partial success / near**. The other five lost fresh vision during descent or retained horizontal error. This is not a precision-landing success claim; it is evidence that late-stage vision freshness and closed-loop convergence remain unresolved.

## Limitations

- Level-body offsets require the flight controller to fuse roll/pitch/yaw before final use.
- Model weights and personal calibration files are not included.
- The camera, container, MCU bridge, and closed loop are not reproduced by desktop tests.
- Detection must remain fresh at low altitude; stale latch behavior was a material failure mode in flight logs.
- The project has not demonstrated repeatable precision landing.

## Attribution

The overall drone was a team project. This repository isolates Sangheon's camera/vision and Uno Q bridge scope and does not grant a new license for joint academic work or third-party components.
