포팅 작업이 완료되었습니다. 최종 결과를 정리합니다.
---
완료된 파일 구조
UnoQ/
├── README.md                    # 프로젝트 문서 (핀맵, 빌드 방법 등)
├── app.yaml                     # App Lab 매니페스트
├── python/
│   ├── main.py                  # MPU: Bridge RPC + NTRIP RTK 클라이언트
│   └── requirements.txt
└── sketch/
    ├── sketch.ino               # 메인 (setup/loop, 모드 상태머신, Bridge RPC)
    ├── sketch.yaml              # arduino-cli 빌드 설정
    ├── hw_config.h              # UNO Q 핀 매핑
    ├── QuadPWM.h / .cpp         # Servo 기반 4채널 PWM
    ├── Protocol.h / .cpp        # Bridge RPC 텔레메트리
    ├── IMU.h / .cpp             # MTi-680G 드라이버 (Serial1)
    ├── RealTime.h / .cpp        # 타이머 (변경 없음)
    ├── MTI_XBUS.h               # XBUS 프로토콜 (변경 없음)
    ├── MTI_DATA.h               # 데이터 구조체 (변경 없음)
    ├── GNSS.h                   # GNSS 구조체 (변경 없음)
    └── filter.h / .c            # 필터 (변경 없음)
주요 변경 포인트 요약
항목	H7 (기존)	UNO Q (신규)
PWM	mbed::PwmOut	Servo 라이브러리
텔레메트리	UART (P8Serial)	Bridge RPC
RTK/NTRIP	MCU에서 WiFi 직접	Python (MPU) 쓰레드
핀 참조	PJ_11, PC_13 등	3, 5, 6 등 Arduino 번호
UART 타입	arduino::HardwareSerial	HardwareSerial
다음 단계 (보드 수령 후)
1. arduino-cli로 빌드 테스트
2. LED Blink으로 보드 기본 동작 확인
3. Serial1 + MAX3232로 MTi-680G 통신 테스트
4. Servo PWM ESC 출력 확인
5. Bridge RPC 양방향 통신 검증
6. NTRIP RTCM 전달 지연 측정