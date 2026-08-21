# 귀속 및 출처 경계

## 프로젝트 범위

이 저장소는 2026년 자율이동체 팀 프로젝트에서 카메라·비전 영역을 포트폴리오용으로 분리한 것입니다. 기체, 비행제어기/STM 구현, 기구·전장 통합과 다른 팀원의 제출 자료는 재배포하지 않습니다.

포함된 Python 파일은 박상헌이 담당한 다음 범위를 다룹니다.

- RealSense 데이터 수집과 라벨 준비·검수
- SSDLite 학습·평가와 NCNN 런타임 추적
- depth 기반 표적 좌표 계산
- Arduino Uno Q 애플리케이션 통합과 bridge 전달

## 소스 정리 방식

`tools/data-preparation/`, `training/`, `evaluation/`, `demo/`, `deployment/`의 스크립트는 프로젝트 작업 파일을 검토 가능한 구조로 재배치한 것입니다. 개인 장비 절대경로, 비공개 데이터셋, 모델 가중치, 내부 작업 메모와 환경 변수 전체를 출력할 수 있는 명령은 옮기지 않았습니다. 파일명과 CLI 옵션은 이식 가능하게 정리했으며 핵심 알고리즘은 유지했습니다.

## 이미지 출처

- `docs/images/detector-metric-summary.png` — 2026-06-27에 수행한 단일 클래스·65장 로컬 validation 결과로 생성
- `docs/images/training-validation-loss.png` — 기록된 SSDLite 학습 run 요약으로 생성; 320 px 두 run과 최종 512 px run 비교
- `docs/images/helipad-label-example.png` — 프로젝트에서 직접 만든 헬리패드 라벨 예시
- `docs/images/realsense-system-overview.svg` — 이 저장소에 포함된 본인 구현 코드를 근거로 포트폴리오 정리 과정에서 직접 제작한 자체 완결형 파이프라인 개요도; 외부 이미지나 원격 리소스를 사용하지 않음

원본 데이터셋, 식별 가능한 인물이 있는 montage, 비행 영상, 수업 보고서, 학번, 비공개 로그, calibration 파일과 학습 가중치는 포함하지 않았습니다.

## 제3자 구성 요소

Intel RealSense, OpenCV, PyTorch, TorchVision, Albumentations, NCNN, NumPy, Matplotlib과 Arduino의 명칭·소프트웨어는 각 권리자의 조건을 따릅니다. 이 저장소는 공동 학술 결과물이나 제3자 코드에 새로운 라이선스를 부여하지 않습니다.
