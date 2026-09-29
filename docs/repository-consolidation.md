# 저장소 통합 기록 / Repository consolidation

자율비행 드론과 RealSense 표식 인식은 같은 팀 프로젝트입니다. 비전 구현과 테스트를 이 저장소의 `vision-system/`으로 옮겨 기체·제어·지상국과 함께 관리합니다. 전체 목표와 팀 역할은 루트 README 한 곳에서 설명하고, 비전 디렉터리의 README는 실행 안내에 집중합니다.

The drone and RealSense marker-vision work belong to the same team project. Vision sources and tests now live in `vision-system/`; the root README explains the whole project, while the module README focuses on running the code.

| 이전 위치 / Previous location | 기준 위치 / Maintained location |
|---|---|
| `realsense-drone-vision/vision`, training, evaluation, deployment, tools, tests | `vision-system/`의 같은 하위 경로 / corresponding subdirectories |
| `realsense-drone-vision/bridge/main.py` | `flight-controller/companion-vision-bridge/main.py` |
| 반복된 비전 처리 그림 / Duplicate vision-processing figure | `development-report/experiment-results/vision-landing/vision_landing_pipeline.png` |
| 독립 프로젝트 소개 / Separate project introductions | 루트 README / root README |

두 브리지의 실행 구문 트리는 같았으며 차이는 경로를 설명하는 주석뿐이었습니다. 배포 설정과 함께 있는 구현을 남겼고, 비전 테스트가 그 파일을 직접 읽도록 경로를 바꿨습니다. 그림은 파일 바이트가 같은 것을 확인한 뒤 중복 사본을 정리했습니다. 보존된 원본 이력에서 이전 경로를 찾을 수 있습니다.

The bridge implementations had identical Python syntax trees and differed only in a path comment. The implementation next to its deployment configuration is retained, and vision tests load it directly. Duplicate figure bytes were checked before removing redundant copies. Earlier paths remain recoverable through the preserved Git history.

통합 전 기준 커밋 / Pre-integration commits:

- Team: `d45a8cb96a9ac683f6e66392b1d8bf7ad35235ae`
- Vision: `ab43c0a597625e389fa8aba4f92b29b046423691`

통합 커밋은 두 이력을 부모로 연결합니다. 작성자 이력을 새로 쓰거나 기여를 다시 귀속하지 않습니다. 비전의 출처·사용 범위는 [ATTRIBUTION](../vision-system/ATTRIBUTION.md)과 [NOTICE](../vision-system/NOTICE.md)에 보존했습니다.

The integration commit joins both histories as parents without rewriting authorship. Vision attribution and distribution boundaries remain in the linked notices. Host tests check software contracts; this repository consolidation does not constitute a new camera or flight trial.
