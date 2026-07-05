# AVC UNO Q - Flight Controller

Portenta H7에서 Arduino UNO Q로 포팅된 쿼드로터 비행 제어 펌웨어.

## 아키텍처

| 구성 | 역할 |
|------|------|
| **MCU** (STM32U585, Cortex-M33 160MHz) | 비행 제어: INS, RC, PWM, PID |
| **MPU** (QRB2210, Linux Debian) | 통신: NTRIP RTK, 텔레메트리 중계 |
| **P8 USART3** | MCU <-> MPU 텔레메트리 (PA7 TX / PA5 RX, 115200 baud) |

## 핀 매핑

| 기능 | UNO Q 핀 | MCU 핀 | 비고 |
|------|----------|--------|------|
| IMU UART RX | D0 | PB7 | USART1, MAX3232 필요 |
| IMU UART TX | D1 | PB6 | USART1 |
| Motor CH0 (FL) | D3 | PB0 | TIM3_CH3, HW PWM |
| Motor CH1 (RL) | D5 | PA11 | TIM1_CH4, HW PWM |
| Motor CH2 (RR) | D6 | PB1 | TIM3_CH4, HW PWM |
| Motor CH3 (FR) | D9 | PB8 | TIM4_CH3, HW PWM |
| RC CH1 Throttle | D2 | PB3 | EXTI |
| RC CH2 Pitch | D4 | PA12 | EXTI |
| RC CH3 Roll | D7 | PB2 | EXTI |
| RC CH4 Rudder | D8 | PB4 | EXTI |
| RC CH5 Aux | D10 | PB9 | EXTI |
| P8 Telem TX | - | PA7 | USART3, AF7 |
| P8 Telem RX | - | PA5 | USART3, AF7 |

## 동작 모드

- **IDLE** - 대기
- **RC_CONTROL** - RC 수신기 + PID + 모터 출력
- **RTK_MAIN** - GNSS/INS 데이터 모니터 + RTCM 전달
- **PWM_TEST** - 개별 모터 제어
- **MTI_TEST** - IMU 데이터 출력

## 소스 파일 구조

```
sketch/
├── sketch.ino          -- 메인 (setup/loop, 전역변수 정의)
├── sketch.yaml         -- 빌드 설정 (arduino:zephyr:unoq)
├── hw_config.h         -- 하드웨어 핀/UART/PWM 설정
├── INSS.h/cpp          -- INS 센서 드라이버 (MTi-680G IMU/GNSS)
├── Telem.h/cpp         -- 텔레메트리 (76B 고정 다운링크 + 업링크 명령)
├── Mode.h/cpp          -- 모드 관리 + 모드별 루프
├── QuadPWM.h/cpp       -- 4CH ESC PWM (STM32 HW 타이머 직접 제어)
├── RCInput.h/cpp       -- RC 수신기 5CH PWM (EXTI ISR)
├── PIDControl.h/cpp    -- PID 컨트롤러
├── Usart3Serial.h/cpp  -- P8 USART3 레지스터 레벨 드라이버
├── RealTime.h/cpp      -- 리얼타임 타이머 (100Hz)
├── filter.h/c          -- 상보 필터 (고도 추정)
└── DebugConsole.h/cpp  -- USB 시리얼 디버그 콘솔
```

## 텔레메트리 프로토콜

### 다운링크 (MCU -> GCS, 10Hz)

고정 76바이트 `TelemFrame` 구조체를 sync + CRC로 감싸서 전송.

```
프레임: [0xAA][0x55][TelemFrame 76B][CRC8_XOR] = 79 bytes
```

TelemFrame 필드:
- mode, status_flags (비트필드: imu_ok, motor_armed, rc_ok, gnss_fix, auto_mode, emergency, rtk_status)
- roll_deg, pitch_deg, yaw_deg (자세)
- lat_deg, lon_deg, alt_m (위치 WGS84)
- vel_e_ms, vel_n_ms, vel_u_ms (ENU 속도)
- gnss_num_sv, gnss_fix_type (GNSS 상태)
- batt1_mv, batt2_mv (배터리 전압, 미구현=0)
- rc_ch[5] (RC 입력 us)
- motor_us[4] (모터 출력 us)
- uptime_ms (MCU 가동시간)
- reserved[2] (확장용)

### 업링크 (GCS -> MCU)

```
프레임: [0xAA][0x55][CMD_ID][LEN_H][LEN_L][PAYLOAD...][CRC8_XOR]
```

CMD: SET_MODE(0x01), ABORT(0x02), PWM_SET(0x03), PWM_DISARM(0x04), RTCM_DATA(0x10), HEARTBEAT(0xF0)

## 빌드 및 업로드

### 빌드 환경

- arduino-cli 1.4.1
- arduino:zephyr 코어 0.55.0

### 빌드

```bash
arduino-cli compile -b arduino:zephyr:unoq sketch/
```

### 업로드 (USB 직접)

```bash
arduino-cli upload -b arduino:zephyr:unoq -p COM13 sketch/
```

### SCP + SSH 배포

```bash
scp sketch/build/arduino.zephyr.unoq/sketch.ino.bin arduino@<UNO_Q_IP>:~/
ssh arduino@<UNO_Q_IP>
arduino-app-cli sketch upload ~/sketch.ino.bin
arduino-app-cli app restart
```

## 주의사항

- MTi-680G는 RS-232 출력이므로 MAX3232 레벨 시프터 필요
- MAX3232는 921600 baud 미지원, 115200 고정
- 모든 GPIO는 3.3V 로직
- **업로드 중 전원 차단/USB 분리 절대 금지** (bricking 위험)
- ESC PWM은 STM32 하드웨어 타이머 직접 제어 (TIM1/TIM3/TIM4, 50Hz)
