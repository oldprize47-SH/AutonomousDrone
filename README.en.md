# RealSense Drone Vision

I worked on the camera and vision software for a university autonomous-drone team project. My responsibilities included collecting and labelling helipad images, comparing lightweight detection models, preparing embedded inference and checking the target information sent to the flight controller. AI coding tools supported parts of the implementation and experiments.

The [team repository](https://github.com/oldprize47-SH/Autonomous_Drone_Development) contains the aircraft and flight experiments. This repository contains my vision work; the complete aircraft and flight controller were team work.

[한국어 상세 문서](README.ko.md)

## The problem I worked on

The landing marker occupies fewer pixels when the aircraft is farther away. A detector therefore needs to recognise small targets while leaving enough computing time for the rest of the onboard software. The camera output also needs to be useful to another program: a box drawn on a video frame does not tell the flight controller whether the measurement is recent or reliable.

I collected and labelled helipad images in different backgrounds and lighting, compared lightweight model candidates and prepared the selected model for the embedded runtime. I used AI coding tools to support implementation and performance measurements, then used those comparisons to choose the model and tracking approach. My responsibility was the vision input to the team system; the aircraft's attitude, altitude and position controllers were separate team work.

## Implementation

The camera is an Intel RealSense D435. SSDLite/MobileNetV3-Small runs through NCNN, with LK optical flow between detection frames. Depth measurements provide relative target geometry. A validity check rejects stale observations before the Uno Q bridge forwards target offsets to the flight controller.

The main runtime is [vision/track_helipad.py](vision/track_helipad.py), and the bridge is [bridge/main.py](bridge/main.py). Data preparation, training and evaluation are kept in [tools/data-preparation](tools/data-preparation), [training](training) and [evaluation](evaluation).

### From an image to a target observation

The detector locates the helipad in the RGB image. Between detector updates, Lucas–Kanade (LK) optical flow follows image features to maintain the target location without running the neural network on every frame. The depth image supplies a distance measurement near the detected target, and the runtime converts the image/depth observation into relative target information.

The validity checks matter as much as the coordinates. The runtime checks the age and consistency of detector observations; tracking alone does not authorise a valid landing observation. The bridge reads the latest JSON record and forwards the observation to the Uno Q application. A stale or rejected observation must not be treated as a fresh target just because the last coordinates remain available.

For a code review, start with [vision/track_helipad.py](vision/track_helipad.py), then follow the output into [bridge/main.py](bridge/main.py). The [technical notes](docs/기술-요약.md) explain the coordinate conventions and validity rules. These rules are part of the vision interface, not proof that the complete aircraft can land safely.

## Results

![Recorded detector evaluation on 65 local validation images](docs/images/detector-metric-summary.png)

The recorded evaluation used 65 local validation images of one class. At confidence 0.5 and matching IoU 0.3, the detector found 61 true positives, with no false positives and four false negatives. Precision was 100%, recall 93.8% and F1 96.8%; mAP@0.5:0.95 was 72.2%.

Precision describes how many reported detections were correct; recall describes how many labelled targets were found. The mAP value summarises detection performance across several overlap thresholds, so it is a different measure from the single-threshold precision and recall above.

These results apply to that validation set. They do not establish performance under all flight conditions. A retrospective review of six trials with vision lock classified one as partial/near success. Repeatable precision landing was not demonstrated. See the [flight evaluation](docs/flight-evaluation-summary.md) for the recorded conditions.

## Running and testing

The repository does not distribute training data, model weights, NCNN model files or personal calibration. Camera inference needs a compatible RealSense/NCNN setup, and the bridge requires the Arduino Uno Q runtime.

Hardware-independent tests can be run with:

```sh
python -m pip install -r requirements-test.txt
python -m pytest -q
```

[deployment/center_depth.py](deployment/center_depth.py) was added from my local Uno Q tools on 28 September 2026. It reports centre depth for camera diagnostics. Its hardware-free check is `python deployment/center_depth.py --self-test`. That check and all 17 host tests passed. The existing published runtime and JSON bridge checks were retained because they contain changes beyond the local copies.

The [detailed Korean README](README.ko.md) contains the data layout and training, evaluation and runtime commands. Host tests do not reproduce the camera, MCU or flight system. No new camera or flight test was performed for this documentation update.

### Choosing a starting point

To understand the project without hardware, read the saved evaluation and run the host tests. To work with a camera, first check the RealSense dependencies and the standalone depth diagnostic before attempting the complete inference pipeline. To train or evaluate a detector, prepare your own compatible labelled data and model files; the repository's scripts do not supply those assets.

The test requirements cover the host checks, not the full training or board environment. A passing host test means the exercised software contract behaves as expected. It does not reproduce camera timing, depth quality, inference speed or the team's flight results.

Source credits and distribution restrictions are described in [ATTRIBUTION.md](ATTRIBUTION.md) and [NOTICE.md](NOTICE.md).
