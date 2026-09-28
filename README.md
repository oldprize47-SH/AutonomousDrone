# RealSense Drone Vision

**Find a helipad, track it between detections, and send fresh target offsets to the team’s flight controller.**

[한국어 상세 문서](README.ko.md) · [Team platform](https://github.com/oldprize47-SH/Autonomous_Drone_Development) · [Portfolio](https://github.com/oldprize47-SH)

![RealSense camera, vision and Uno Q bridge architecture](docs/images/realsense-system-overview.svg)

| What I built | How it contributes |
|---|---|
| Data and training tools | Capture, label and evaluate helipad imagery |
| Detector + tracker | Combine NCNN inference with LK optical flow |
| Target validity and bridge | Reject stale observations and forward offsets to the team controller |

**My scope:** camera, vision and companion bridge. The complete aircraft and flight controller are team work.

| Recorded detector result | Flight evidence boundary |
|---|---|
| 65-image local validation: 93.8% recall, 96.8% F1 | Repeated precision landing was not demonstrated |

![Recorded local detector evaluation](docs/images/detector-metric-summary.png)

## Implementation and reproduction


**Camera and vision contribution to a university autonomous-vehicle team project.**

[한국어 상세 문서](README.md) · [Team project and flight demo](https://github.com/oldprize47-SH/Autonomous_Drone_Development)

![RGB-D vision pipeline](docs/images/realsense-system-overview.svg)

## What I worked on

I was responsible for the course team's camera and vision work: collecting and
labelling helipad images, comparing lightweight models, preparing embedded
inference, comparing detection and tracking approaches, and checking relative
position and validity information passed to flight control. AI coding tools
supported implementation and experiments. This repository documents that scope;
it does not claim individual ownership of the complete aircraft or flight controller.

## Engineering problem

A distant landing marker occupies few pixels, while a larger detector input costs
more computation on an embedded board. The pipeline uses SSDLite/MobileNetV3-Small
with NCNN and LK optical flow between detection frames. RealSense depth provides
geometry, and a validity/freshness gate controls advisory output to the Uno Q bridge.

```text
RGB + depth -> detection and tracking -> validity gate -> relative geometry
            -> latest JSON -> Uno Q bridge -> team's flight-control interface
```

The original team archive and this implementation serve different purposes:
the [team fork](https://github.com/oldprize47-SH/Autonomous_Drone_Development)
presents the system, diagrams and flight experiments; this repository focuses on
vision source, data tools and evaluation boundaries.

## Code map

| Entry | Purpose |
|---|---|
| [vision/track_helipad.py](vision/track_helipad.py) | Camera, NCNN detector, tracking and output pipeline |
| [bridge/main.py](bridge/main.py) | Companion process and Uno Q bridge integration |
| [training/train.py](training/train.py) | Detector training |
| [evaluation/evaluate_detector.py](evaluation/evaluate_detector.py) | Local detector evaluation |
| [tests](tests) | Host-side checks; see the detailed README for the recorded test scope |

## Recorded results and limits

The recorded detector evaluation used **65 local validation images with one class**.
At confidence 0.5 and matching IoU 0.3, it reported 61 true positives, no false
positives and four false negatives: precision 100%, recall 93.8% and F1 96.8%.
Recorded mAP@0.5:0.95 was 72.2%. These values do not establish performance on
independent subjects, backgrounds or flight conditions.

The retrospective flight analysis classified only one of six trials with vision
lock as partial/near success. Repeatable precision landing was not established.
These historical results were not re-measured during this documentation update.

## Reproduction boundary

The repository contains source, but training data, trained weights, NCNN model
artifacts and personal calibration are not distributed here. Camera inference
requires compatible RealSense/NCNN dependencies and the target setup; bridge
execution requires the Arduino Uno Q runtime. This is not a one-command hardware
reproduction package.

Use the [detailed README](README.md) for the existing data layout, training,
evaluation and runtime commands. No camera, motor or flight command was executed
for this portfolio presentation update.
