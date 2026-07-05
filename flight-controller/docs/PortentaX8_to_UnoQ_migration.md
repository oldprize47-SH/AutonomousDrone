# Portenta X8 → Arduino UNO Q 전환 검토

> 작성일: 2026-05-08



---

## 1. 전환 배경

- Portenta X8 보드가 Serial1 UART 테스트 중 브릭 발생 (2026-05-07)
- SSH 연결 불가, BOOT 복구 모드 진입 실패, 사실상 사용 불가
- 대체 보드로 Arduino UNO Q (Qualcomm QRB2210 + STM32U585) 검토



---

## 2. 전환해야 하는 이유

### 2-1. UART 제한: X8과 동일 (전환 불이익 없음)

X8에서 M4 코어가 사용할 수 있는 외부 UART는 Serial1 (PA9/PA10) **1개뿐**이었다.
SERIAL3 (PJ8/PJ9)는 X8 보드 패키지 `pins_arduino.h`에 미정의되어 사용 불가 확인됨 (Arduino 공식 지원 @manchoz, GitHub issue #584).
`arduino::UART P8Serial(PJ_8, PJ_9);`로 직접 생성해도 데이터 출력 안 됨.

|                   | Portenta X8 (M4)            | UNO Q (STM32U585)               |
| ----------------- | --------------------------- | ------------------------------- |
| 사용 가능 UART    | Serial1 (PA9/PA10) **1개**  | USART1 (D0/D1, PB7/PB6) **1개** |
| SERIAL3 (PJ8/PJ9) | 사용 불가 (미정의)          | 해당 핀 없음                    |
| USB CDC 디버그    | 직접 불가 (Linux 경유 필요) | 사용 가능                       |

**결론**: UART 상황은 양쪽 동일. X8을 유지할 UART 측면 이점 없음.



### 2-2. 물리적 크기/무게 절감

|           | Portenta X8                           | UNO Q                  |
| --------- | ------------------------------------- | ---------------------- |
| 구성      | X8 모듈 + Breakout 캐리어 보드 (필수) | 단일 보드 (UNO 폼팩터) |
| 핀 접근   | 고밀도 커넥터 → Breakout 보드 경유    | 핀 헤더에 직접 접근    |
| 드론 탑재 | 공간 차지 큼, 무거움                  | 소형, 경량             |

Breakout 보드 없이 핀 헤더에 직접 배선할 수 있어 드론 탑재 시 공간/무게 모두 유리.



### 2-3. Linux 코어 성능 비교: UNO Q 우위

| 항목       | X8 (i.MX 8M Mini)                      | UNO Q (QRB2210)                          |
| ---------- | -------------------------------------- | ---------------------------------------- |
| CPU        | Cortex-A53 × 4 @ 1.8GHz (14nm)         | Cortex-A53 × 4 @ 2.0GHz (11nm)           |
| RAM        | 2GB LPDDR4                             | 2GB / 4GB LPDDR4X                        |
| GPU        | Vivante GCNano (~6.4 GFLOPS, 1 shader) | **Adreno 702 (~1.3 TFLOPS, 128 shader)** |
| AI 가속기  | 없음                                   | **Hexagon DSP (듀얼코어)**               |
| OpenCL     | 제한적                                 | **2.0 지원**                             |
| WiFi       | 802.11n (65Mbps)                       | **802.11ac (WiFi 5)**                    |
| 카메라 ISP | 단일                                   | **듀얼 (13MP + 13MP)**                   |

- GPU 성능 약 200배 차이 (6.4 GFLOPS vs 1.3 TFLOPS)
- Hexagon DSP로 TensorFlow Lite / SNPE 하드웨어 가속 가능
- QRB2210에서 YOLOv11n (320×240) 15-20 FPS 실적 있음



### 2-4. 영상처리 / 딥러닝 가능 여부

| 항목               | X8 (i.MX 8M Mini) | UNO Q (QRB2210)             |
| ------------------ | ----------------- | --------------------------- |
| Python + OpenCV    | 가능 (CPU only)   | **가능 (GPU 가속)**         |
| TensorFlow Lite    | CPU only          | **CPU / GPU / Hexagon DSP** |
| SNPE (Qualcomm)    | 미지원            | **지원 (INT8/INT4 양자화)** |
| YOLOv11n 추론      | ~5-10 FPS         | **15-20 FPS**               |
| MobileNet 객체감지 | 느림              | **실시간**                  |

카메라 프로세싱 + 가벼운 딥러닝 목적이라면 UNO Q가 우위.





---

## 3. 하드웨어 기능 호환성 검토



### 3-1. PWM (모터 ESC 4ch)

**요구사항**: 50Hz PWM 출력 × 4채널

| X8 (H7 M7)             |        | UNO Q (STM32U585)       |      |
| ---------------------- | ------ | ----------------------- | ---- |
| PJ11 (PWM4, TIM1_CH2)  | CH0 FL | **D5** (PA11, TIM1_CH4) | 가능 |
| PH15 (PWM6, TIM8_CH3N) | CH1 RL | **D6** (PB1, TIM3_CH4)  | 가능 |
| PC6 (PWM1, TIM3_CH1)   | CH2 RR | **D3** (PB0, TIM3_CH3)  | 가능 |
| PK1 (PWM5, TIM1_CH1)   | CH3 FR | **D9** (PB8, TIM4_CH3)  | 가능 |

UNO Q는 10개 이상 PWM 핀 사용 가능 (D0,D1,D2,D3,D5,D6,D7,D9,D10,D11,D12,D13).
TIM1, TIM2, TIM3, TIM4, TIM8 타이머 사용 가능. **4채널 ESC 출력 충분.**

> 주의: D0/D1은 USART1(MTi용)로 사용하므로 PWM에서 제외. D4는 PWM 미지원.



### 3-2. GPIO 인터럽트 (RC 수신기 5ch)

**요구사항**: attachInterrupt(CHANGE) × 5채널, EXTI 충돌 없음

| X8 (Breakout GPIO)   |              | UNO Q (JDIGITAL)       |      |
| -------------------- | ------------ | ---------------------- | ---- |
| PC13 (GPIO0, EXTI13) | CH1 Throttle | **D2** (PB3, EXTI3)    | 가능 |
| PC15 (GPIO1, EXTI15) | CH2 Pitch    | **D7** (PB2, EXTI2)    | 가능 |
| PD4 (GPIO2, EXTI4)   | CH3 Roll     | **D8** (PB4, EXTI4)    | 가능 |
| PD5 (GPIO3, EXTI5)   | CH4 Rudder   | **D10** (PB9, EXTI9)   | 가능 |
| PE3 (GPIO4, EXTI3)   | CH5 Aux      | **D12** (PB14, EXTI14) | 가능 |

STM32U585는 D0~D13 전체 EXTI 지원. D0/D1(UART)과 PWM 4핀을 제외하고도 5채널 이상 확보 가능.
위 배정 예시에서 EXTI 라인 (3,2,4,9,14) 모두 충돌 없음.

> 참고: 최종 핀 배정은 UNO Q 보드 입수 후 실제 테스트를 거쳐 확정.



### 3-3. UART (MTi-680G IMU)

|           | X8                          | UNO Q                         |
| --------- | --------------------------- | ----------------------------- |
| IMU 연결  | Serial1 (PA9/PA10)          | USART1 (D0=PB7 RX, D1=PB6 TX) |
| Baud      | 115200                      | 115200                        |
| MAX3232   | 필요 (RS-232 레벨 변환)     | 필요 (동일)                   |
| RTCM 전달 | ForwardGnssData via Serial1 | ForwardGnssData via USART1    |

기존 IMU 드라이버 (XBUS 파싱 + ForwardGnssData) 로직은 동일. 핀 번호만 변경.



### 3-4. 텔레메트리 (P8 모듈)

|      | X8 계획                              | UNO Q 계획                     |
| ---- | ------------------------------------ | ------------------------------ |
| 경로 | M4 SERIAL3 (PJ8/PJ9) → **사용 불가** | Linux USB-C → P8 (CP2102 내장) |
| 대안 | Linux RPC 경유                       | **Linux에서 직접 USB 시리얼**  |

UNO Q에서는 P8 모듈을 Linux 측 USB-C에 연결 (CP2102로 `/dev/ttyUSB`로 인식).
MCU → Bridge RPC → Linux → P8 → GCS 경로로 텔레메트리 전송.



### 3-5. 기타 인터페이스

| 기능   | X8                   | UNO Q                           | 비고             |
| ------ | -------------------- | ------------------------------- | ---------------- |
| CAN    | FDCAN1 (Breakout)    | FDCAN1 (D4=PA12 TX, D5=PA11 RX) | 현재 미사용      |
| I2C    | I2C1, I2C2 사용 가능 | I2C2 (D20/D21), I2C3 (A4/A5)    | 센서 확장용      |
| SPI    | SPI1 사용 가능       | SPI2 (JSPI 헤더)                | 센서 확장용      |
| Analog | A0~A6 (독립 5채널)   | A0~A5 (6채널)                   | ADC 사용 가능    |
| LED    | LEDR/LEDG/LEDB (RGB) | 8×13 LED 매트릭스               | 매크로 변경 필요 |

---

## 4. UNO Q 시스템 아키텍처

```
MCU (STM32U585, Cortex-M33 160MHz)     MPU (QRB2210, Linux Debian)
┌────────────────────────┐              ┌──────────────────────────┐
│                        │              │                          │
│  USART1 (D0/D1)        │              │  USB-C (허브 사용 가능)     │
│   └─ MTi-680G IMU      │              │   ├─ P8 텔레메트리 모듈     │
│      (MAX3232 경유)     │              │   │  (CP2102, /dev/ttyUSB)│
│      IMU 데이터 수신     │              │   └─ USB 카메라 (UVC)      │
│      RTCM 보정 전달     │               │      또는 MIPI-CSI 카메라  │
│                        │   Bridge     │                          │
│  비행 제어 로직          │◄──(RPC)────► │  GCS 텔레메트리 중계        │
│   └─ PID 100Hz         │              │   (WiFi 또는 P8 경유)      │
│   └─ filter.c          │              │                          │
│                        │   Bridge     │  NTRIP RTK 클라이언트      │
│  IMU::fwdRtcm()        │◄──(RPC)──────│   └─ WiFi → RTCM 수신     │
│                        │              │   └─ Bridge로 MCU에 전달   │
│  GPIO ← RC 수신기 5ch   │              │                          │
│  PWM → ESC 4ch 모터     │              │  카메라 프로세싱           │
│                        │              │   └─ Python + OpenCV     │
│                        │              │   └─ YOLOv11n / SNPE     │
└────────────────────────┘              └──────────────────────────┘
```



### Bridge RPC 통신

- 프로토콜: MessagePack RPC
- 물리 레이어: 내부 UART (115200 baud)
- 라이브러리: Arduino_RouterBridge (MCU), arduino-router (Linux, Go), pyrpc (Python)
- 동기/비동기/노티피케이션 모드 지원



### P8 + 카메라 USB 동시 사용

- USB 3.1 대역폭: ~1.2 GB/s
- P8 데이터량: ~7 KB/s (57600 baud) — USB 대역폭의 0.001% 미만
- USB 카메라 (720p MJPEG): ~30 MB/s
- USB 허브로 동시 연결 시 대역폭 충돌 없음
- MIPI-CSI 카메라 사용 시 USB는 P8 전용 (더 안정적)



## 5. 결론

| 판단 기준       | X8 유지              | UNO Q 전환               |
| --------------- | -------------------- | ------------------------ |
| UART 수         | 1개                  | 1개 (동일)               |
| 공간/무게       | Breakout 필요 (대형) | **단일 보드 (소형)**     |
| Linux 성능      | 기본                 | **GPU 200배, AI 가속기** |
| 영상처리/딥러닝 | CPU only, 느림       | **GPU+DSP 가속, 실시간** |
| 보드 상태       | **고장 (브릭)**      | 구매 고려                |

**X8 → UNO Q 전환은 UART 제한의 불이익 없이, 크기/성능/개발편의 모든 면에서 이점이 있다.**