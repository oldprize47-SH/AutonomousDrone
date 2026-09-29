# 착륙 표식 인식 · 실행 안내

[한국어](#korean) · [English](#english) · [전체 드론 프로젝트](../README.md)

<a id="korean"></a>
## 한국어

이 디렉터리에는 자율비행 드론의 비전 코드와 테스트를 모았습니다. 프로젝트의 목적, 팀원별 역할과 결과는 [대표 README](../README.md)에서 함께 읽으실 수 있습니다. 여기서는 코드를 살펴보고 실행하는 순서를 안내합니다.

### 코드 구성

| 경로 | 살펴볼 내용 |
|---|---|
| [vision/track_helipad.py](vision/track_helipad.py) | 검출·추적·깊이 처리와 관측 유효성 |
| [비행제어 연결 프로그램](../flight-controller/companion-vision-bridge/main.py) | 최신 관측을 읽고 Uno Q로 전달하는 단일 기준 구현 |
| [tools/data-preparation](tools/data-preparation) | 영상 수집, 라벨링·검수, 데이터 분리 |
| [training](training) / [evaluation](evaluation) | 모델 학습과 검출 성능 평가 |
| [deployment/center_depth.py](deployment/center_depth.py) | 카메라 중앙 깊이 진단 |

### 하드웨어 없이 살펴보기

저장소 루트에서 아래 순서로 실행하시면 비전 프로그램의 호스트 테스트를 확인할 수 있습니다.

```sh
cd vision-system
python -m pip install -r requirements-test.txt
python -m pytest -q
python deployment/center_depth.py --self-test
```

이 테스트는 소프트웨어의 입력·출력 계약을 확인합니다. 실제 카메라 지연, 깊이 품질, 추론 속도와 비행 동작은 해당 장비에서 따로 확인해야 합니다.

### 카메라·학습 환경으로 이어가기

먼저 [기술 가이드](README.ko.md)의 준비 단계와 [좌표·유효성 규약](docs/기술-요약.md)을 살펴보시면 전체 실행 흐름을 이해하기 쉽습니다. 카메라 추론에는 RealSense와 NCNN 환경이 필요하고, 데이터·모델 가중치·개인 보정값은 별도로 준비해야 합니다. 학습·평가 의존성은 [requirements-pipeline.txt](requirements-pipeline.txt)에 정리했습니다.

Uno Q 애플리케이션은 상위 [companion-vision-bridge](../flight-controller/companion-vision-bridge)에서 관리합니다. 같은 브리지 구현을 두 곳에서 수정하지 않도록 이 디렉터리의 테스트도 해당 파일을 직접 확인합니다.

[기록된 비행 평가](docs/flight-evaluation-summary.md), [출처](ATTRIBUTION.md), [사용 범위](NOTICE.md)를 함께 참고해 주세요.

---

<a id="english"></a>
## English

This directory contains the drone's vision code and tests. The [main project README](../README.md) explains the project goal, team responsibilities and results together. This guide focuses on exploring and running the code.

### Code layout

| Path | What to explore |
|---|---|
| [vision/track_helipad.py](vision/track_helipad.py) | Detection, tracking, depth and observation validity |
| [Flight-controller bridge](../flight-controller/companion-vision-bridge/main.py) | The single maintained implementation that forwards observations to Uno Q |
| [tools/data-preparation](tools/data-preparation) | Capture, labelling, review and dataset splitting |
| [training](training) / [evaluation](evaluation) | Model training and detector evaluation |
| [deployment/center_depth.py](deployment/center_depth.py) | Centre-depth camera diagnostics |

### Start without hardware

From the repository root, you can run the host checks with:

```sh
cd vision-system
python -m pip install -r requirements-test.txt
python -m pytest -q
python deployment/center_depth.py --self-test
```

These checks exercise software input/output contracts. Camera timing, depth quality, inference speed and flight behaviour still need to be evaluated on the actual equipment.

### Continue with a camera or training environment

The [technical guide](README.ko.md) and [coordinate and validity notes](docs/기술-요약.md) explain the preparation and runtime flow in Korean. Camera inference requires RealSense and NCNN; you will also need compatible data, model files and calibration. Training and evaluation dependencies are listed in [requirements-pipeline.txt](requirements-pipeline.txt).

The Uno Q application lives in [companion-vision-bridge](../flight-controller/companion-vision-bridge). Tests here load that implementation directly, so there is only one bridge to maintain.

The [recorded flight evaluation](docs/flight-evaluation-summary.md), [attribution](ATTRIBUTION.md) and [distribution notes](NOTICE.md) provide context for using the material.
