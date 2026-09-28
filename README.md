# RealSense Drone Vision

I worked on the camera and vision software for a university autonomous-drone team project. My responsibilities included collecting and labelling helipad images, comparing lightweight detection models, preparing embedded inference and checking the target information sent to the flight controller. AI coding tools supported parts of the implementation and experiments.

The [team repository](https://github.com/oldprize47-SH/Autonomous_Drone_Development) contains the aircraft and flight experiments. This repository contains my vision work; the complete aircraft and flight controller were team work.

[한국어 상세 문서](README.ko.md)

## Implementation

The camera is an Intel RealSense D435. SSDLite/MobileNetV3-Small runs through NCNN, with LK optical flow between detection frames. Depth measurements provide relative target geometry. A validity check rejects stale observations before the Uno Q bridge forwards target offsets to the flight controller.

The main runtime is [vision/track_helipad.py](vision/track_helipad.py), and the bridge is [bridge/main.py](bridge/main.py). Data preparation, training and evaluation are kept in [tools/data-preparation](tools/data-preparation), [training](training) and [evaluation](evaluation).

## Results

![Recorded detector evaluation on 65 local validation images](docs/images/detector-metric-summary.png)

The recorded evaluation used 65 local validation images of one class. At confidence 0.5 and matching IoU 0.3, the detector found 61 true positives, with no false positives and four false negatives. Precision was 100%, recall 93.8% and F1 96.8%; mAP@0.5:0.95 was 72.2%.

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

Source credits and distribution restrictions are described in [ATTRIBUTION.md](ATTRIBUTION.md) and [NOTICE.md](NOTICE.md).
