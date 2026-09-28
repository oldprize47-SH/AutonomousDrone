# RealSense 드론 비전

[English overview](README.en.md) · [팀 프로젝트 fork](https://github.com/oldprize47-SH/Autonomous_Drone_Development)

Intel RealSense D435, SSDLite/NCNN, LK optical flow와 Arduino Uno Q bridge를 연결해
헬리패드의 **freshness-aware advisory target**을 생성한 자율착륙 비전 프로젝트입니다.

![RealSense RGB-D 입력에서 착륙 표식 좌표 전달까지의 전체 비전 파이프라인](docs/images/realsense-system-overview.svg)

> 팀 프로젝트 환경에서 저장한 헬리패드 검출 화면입니다. 이 저장소는 카메라·비전·Uno Q
> bridge라는 개인 기여 범위만 다루며, 기체 전체나 flight-controller를 단독 구현했다고
> 주장하지 않습니다.

## 한눈에 보기

| 항목 | 내용 |
|---|---|
| 기간 | 2026년 1학기 |
| 목표 | D435 RGB/depth로 헬리패드를 찾고 body-FRD 수평 오차를 안전하게 전달 |
| 개인 기여 | 데이터 도구, SSDLite 학습·평가, NCNN 검출/LK 추적, depth 측정, Uno Q bridge |
| 핵심 안전 동작 | tracker-only 예측은 착륙 advisory를 허가하지 않으며 stale packet은 `valid=0`으로 무효화 |
| 로컬 검증 | 단일 클래스 validation 65장: Precision 100.0%, Recall 93.8%, F1 96.8% |
| 비행 경계 | vision lock이 있었던 6회 중 1회만 partial/near; 반복 정밀착륙 성공은 입증하지 못함 |

더 자세한 설계 근거와 평가 경계는 [기술 요약](docs/기술-요약.md)에 정리했습니다.

## 문제와 목표

헬리패드는 고도가 높아질수록 작은 물체가 되고, 임베디드 보드에서 매 프레임 detector를
실행하면 지연이 커집니다. 검출이 끊긴 뒤 마지막 좌표가 계속 유효한 것처럼 남는 현상도
착륙 단계에서는 위험합니다. 이 프로젝트는 다음 네 문제를 하나의 파이프라인으로 다뤘습니다.

1. 다양한 거리·각도·조명에서 학습 데이터를 수집하고 빠르게 라벨링한다.
2. 작은 표식을 위한 512 px SSDLite detector를 학습하고 정량 평가한다.
3. NCNN detector 사이 프레임을 LK optical flow로 추적하되 freshness를 잃으면 fail closed 한다.
4. depth와 카메라 장착 offset을 body FRD 좌표로 바꾸고 Uno Q bridge로 전달한다.

## 개인 기여와 팀 경계

### 이 저장소에 포함한 개인 기여

- HEIC 변환, D435 RGB/depth 캡처, 4점 라벨링, 자동 라벨·검수, 데이터 분리 도구
- SSDLite + MobileNetV3-Small 학습과 작은 물체용 custom anchor 구성
- one-class validation의 precision/recall/F1/AP/mAP 계산 및 그래프 생성
- NCNN detector, LK optical-flow tracker와 fail-closed `LandingGate`
- RealSense depth deprojection과 level-body FRD offset 계산
- atomic latest-JSON telemetry, Uno Q 프로세스 감시와 `Bridge.notify` 전달
- stale telemetry 무효화, debug 기록과 camera-stage 지연 벤치마크

### 포함하지 않은 팀 범위

- 기체 구조, 추진·전원·배선, airframe CAD
- STM/flight-controller 상태기계와 최종 제어기
- 팀 보고서·발표자료·비행 영상과 다른 팀원의 코드

## 아키텍처

```mermaid
flowchart LR
    subgraph Offline[오프라인 데이터·학습 — 개인 범위]
        HEIC[HEIC/JPG 및 D435 캡처] --> LABEL[4점 라벨링\n자동 라벨·수동 검수]
        LABEL --> SPLIT[Train/Validation 분리]
        SPLIT --> TRAIN[SSDLite 512 학습\nMobileNetV3-Small]
        TRAIN --> PTH[PyTorch weight\n비공개]
        PTH --> EVAL[65장 로컬 평가\nP/R/F1/AP/mAP]
        PTH -. 별도 변환·parity 검증 필요 .-> NCNN[NCNN param/bin\n비공개]
    end

    subgraph Runtime[Uno Q 실시간 비전 — 개인 범위]
        D435[RealSense D435\nRGB + depth] --> DET[SSDLite/NCNN detector]
        NCNN --> DET
        DET --> LK[LK optical-flow tracking]
        LK --> GATE[Fail-closed LandingGate]
        GATE --> FRD[Depth deprojection\nlevel-body FRD]
        FRD --> JSON[Atomic compact JSON]
        JSON --> BRIDGE[Arduino App bridge\nBridge.notify]
    end

    BRIDGE --> FC[팀 STM / flight controller]
```

`LandingGate`는 최근 detector 관측, 연속 유효 프레임, jitter, box 면적 변화와 detection
age를 함께 확인합니다. tracker-only 예측은 화면상 추적을 이어갈 수는 있지만 `valid=1`을
허가하지 않습니다.

## 데이터 준비 → 학습 → 평가

![헬리패드 라벨 예시](docs/images/helipad-label-example.png)

> 네 점을 클릭해 원형 헬리패드의 bounding box를 만든 예시입니다. 원본 학습 데이터는
> 개인정보와 용량·사용권 때문에 공개하지 않습니다.

### 1. 데이터 준비

```bash
# 선택: 휴대전화 HEIC를 JPG로 변환
python tools/data-preparation/convert_heic.py \
  --src ./heic-input --dst ./data/images

# D435 RGB/depth 캡처(pyRealSense2가 설치된 데스크톱)
python tools/data-preparation/capture_frames.py --output-dir ./data

# 수동 라벨 또는 기존 모델을 이용한 자동 라벨 후 검수
python tools/data-preparation/label_four_points.py \
  --image-dir ./data/images --label-dir ./data/labels
python tools/data-preparation/auto_label.py \
  --img-dir ./data/images --label-dir ./data/labels --weights ./weights/best.pth
python tools/data-preparation/review_labels.py

# 재현 가능한 train/validation 분리
python tools/data-preparation/split_dataset.py --data-dir ./data --seed 42
```

기본 데이터 레이아웃은 다음과 같으며 `data/`는 Git에서 제외됩니다.
라벨은 정규화된 5열 bounding-box 텍스트 형식을 사용하지만, detector는 SSDLite이며
YOLO/IMX500 학습·배포 코드는 이 저장소에 섞지 않았습니다.

```text
data/
├── images/                 # 분리 전 RGB pool
├── labels/                 # YOLO 형식 label pool
├── depth/                  # 선택적 16-bit depth
├── train/{images,labels}/
└── val/{images,labels}/
```

### 2. SSDLite 512 학습

```bash
python training/train.py \
  --data-dir ./data \
  --output-dir ./weights \
  --epochs 80 \
  --batch-size 16 \
  --seed 42
```

학습기는 512×512 입력, 작은 정사각형 물체를 여러 feature-map level에서 받는 custom
anchor, 회전·밝기/대비·좌우반전 augmentation, 3 epoch warm-up과 cosine schedule을
사용합니다.

![학습 validation loss 비교](docs/images/training-validation-loss.png)

> 기록된 두 번의 320 px run과 최종 512 px run의 best validation loss입니다. 서로 다른
> run의 요약이므로 동일 조건 통제 실험으로 과대 해석하지 않습니다.

### 3. 로컬 detector 평가

```bash
python evaluation/evaluate_detector.py \
  --data-dir ./data \
  --weights ./weights/best.pth \
  --output-dir ./output/evaluation
```

2026-06-27에 기록한 결과는 **단일 클래스, 로컬 validation 65장/GT 65개**에 한정됩니다.
confidence 0.5와 matching IoU 0.3에서 detection/TP 61, FP 0, FN 4였습니다. AP 계산 시에는
순위 보존을 위해 모델 post-processing score threshold를 0.001로 낮췄으며, 이는 비행
runtime의 허용 임계값이 아닙니다.

| 지표 | 기록값 |
|---|---:|
| Precision | 100.0% |
| Recall | 93.8% |
| F1 | 96.8% |
| Mean IoU (matched) | 86.6% |
| AP@0.3 | 100.0% |
| AP@0.5 | 96.9% |
| mAP@0.5:0.95 | 72.2% |

![65장 로컬 평가 지표](docs/images/detector-metric-summary.png)

> 이 수치는 독립 외부 데이터, 다중 클래스, 고도·조명·배경 전체의 일반화나 폐루프 착륙
> 성공률을 의미하지 않습니다.

## NCNN 런타임 → Uno Q bridge

학습 weight, NCNN `.param`/`.bin`, 개인 calibration은 공개하지 않습니다. 호환 artifact와
RealSense/NCNN 런타임이 준비된 환경에서는 다음 형태로 실행합니다.

```bash
python vision/track_helipad.py \
  --param ./models/ssdlite_v3small_helipad.param \
  --bin ./models/ssdlite_v3small_helipad.bin \
  --headless --no-log \
  --telemetry latest --telemetry-format stm \
  --telemetry-path ./output/helipad-latest.json
```

`vision/track_helipad.py`는 compact packet을 임시 파일에 쓴 뒤 latest JSON을 원자적으로
교체합니다. Uno Q의 `bridge/main.py`는 subprocess와 파일 freshness를 감시하고 다음 계약으로
비행 제어 측에 전달합니다.

```text
Bridge.notify(
  "set_helipad_target",
  valid,
  helipad_x_m,
  helipad_y_m,
  helipad_xy_distance_m,
  drone_ground_distance_m,
)
```

이전에 유효했던 packet이 stale이 되면 `valid=0`을 한 번 전송해 마지막 관측값이 현재 표적으로
남지 않게 합니다. 실제 bridge 실행에는 Uno Q Arduino App runtime이 필요합니다.

PC에서 학습 weight를 시각 점검하거나 camera stage 지연을 계측할 수도 있습니다.

```bash
python demo/desktop_demo.py --weights ./weights/best.pth
python deployment/bench_camera.py --frames 150
```

## 비행 평가 경계

기존 비행 로그를 보수적인 사후 기준으로 분석했을 때, vision lock이 있었던 6회 중 하나만
**partial success / near**로 분류했습니다. 해당 trial은 약 0.643 m 고도까지 fresh vision을
유지했고 helipad residual은 약 0.113 m였지만, touchdown 전 제어기의 수평 오차가 약
0.256 m 남았습니다. 다른 다섯 번은 하강 중 fresh vision이 사라지거나 수평 오차가 남았습니다.

따라서 이 저장소는 반복 정밀착륙 성공을 주장하지 않습니다. 자세한 판정 규칙은
[비행 평가 요약](docs/flight-evaluation-summary.md)에 있습니다.

## 저장소 구조

```text
.
├── bridge/main.py                       # Uno Q tracker 감시·Bridge.notify 전달
├── vision/
│   ├── inference.py                     # SSD anchor/box decode/NMS·NCNN 진단 경로
│   └── track_helipad.py                 # D435, detector/LK, gate, depth/FRD, telemetry
├── tools/data-preparation/
│   ├── convert_heic.py
│   ├── capture_frames.py
│   ├── label_four_points.py
│   ├── auto_label.py
│   ├── review_labels.py
│   └── split_dataset.py
├── training/train.py                    # SSDLite 512 학습
├── evaluation/evaluate_detector.py      # one-class local 평가/그래프
├── demo/desktop_demo.py                 # PC RealSense 실시간 시각 점검
├── deployment/bench_camera.py           # Uno Q 배포 병목 계측
├── docs/
│   ├── 기술-요약.md
│   ├── flight-evaluation-summary.md
│   └── images/                          # 자체 제작 개요도와 검증 결과 이미지 4개
├── tests/                               # hardware-independent 계약/회귀 테스트
├── requirements-test.txt
├── requirements-pipeline.txt
├── ATTRIBUTION.md
└── NOTICE.md
```

## 설치와 검증

### 하드웨어 독립 테스트

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux/macOS: source .venv/bin/activate
python -m pip install -r requirements-test.txt
python -m pytest -q
```

테스트는 debug metrics, 비차단 video recorder, tracker-only fail-closed gate, compact packet
validity, bridge forwarding, stale invalidation, 새 포트폴리오 파일의 portable-path/secret-output
계약과 Python compilation을 검사합니다.

### 데이터·학습·평가 도구

```bash
python -m pip install -r requirements-pipeline.txt
```

`requirements-pipeline.txt`는 데이터 준비·PyTorch 학습·평가·시각화 의존성입니다.
RealSense 카메라 도구에는 호환되는 `pyrealsense2`/librealsense 설치가 추가로 필요합니다.
Uno Q 배포에는 보드 이미지가 제공하는 NCNN Python binding, RealSense runtime과 Arduino App
runtime이 필요하므로 일반 pip requirements로 고정하지 않았습니다.

## 제한 사항

- 모델 weight, raw image/depth, calibration, 비행 로그를 배포하지 않습니다.
- PyTorch→NCNN 변환 artifact는 본 저장소에 포함하지 않으며, 실제 target runtime에서 별도
  검증이 필요합니다.
- level-body FRD 측정은 기체 자세를 완전히 보정하지 않습니다. flight controller가
  roll/pitch/yaw를 융합해야 합니다.
- 데스크톱 테스트는 RealSense, Uno Q container, MCU bridge와 폐루프 비행을 재현하지 않습니다.
- detector validation 결과와 비행 성공률은 분리해 해석해야 합니다.
- 저고도 fresh vision과 반복적인 수평 수렴은 아직 해결해야 할 과제입니다.

## 출처와 사용 범위

팀 산출물, 다른 팀원 코드, 얼굴이 보이는 montage, 비행 영상, 수업 보고서와 개인 식별정보는
포함하지 않았습니다. 코드·이미지별 출처와 범위는 [ATTRIBUTION.md](ATTRIBUTION.md), 공개
제한은 [NOTICE.md](NOTICE.md)를 참고하세요. 제3자 라이브러리는 각 소유자의 라이선스를 따릅니다.
