# 개발 로드맵

하나의 monorepo에서 단계별 독립 실행·테스트를 유지합니다. 이후 단계는 기존 코드를 삭제하거나
덮어쓰지 않고, `01-camera-stream/src/drone_camera`의 `CameraSource`, `Frame`, 설정 모듈을
재사용할 수 있습니다. 소스 패키징이 필요한 시점에는 호환성을 유지하며 공통 패키지로 추출합니다.

1. **01 — Camera stream**: MJPEG·RTSP·USB·로컬 영상 두 채널, 독립 수신, 최신 프레임,
   수신 FPS·상태·설정, Windows/macOS GUI, 테스트와 문서. 현재 작업 범위입니다.
   `v0.2.0`은 이 단계의 입력 확장으로 Android/iPhone 자체 송출 앱, Wi-Fi·USB 전송을 추가합니다.
   이후 AI·3D 단계를 앞당겨 구현하지 않습니다.
2. **02 — YOLO detection** (`v0.5.0`, 2D): 학습된 YOLO11n(`best.pt`)로 카메라별 탐지, Ultralytics ByteTrack
   카메라별 추적 ID, 바운딩 서클(중심·반지름)과 후속 단계용 데이터 구조. 1단계의 최신 프레임을 별도 AI 스레드가
   소비하므로 GUI/수신 주기를 막지 않습니다. [설명서](ai-detection.md). 실제 드론 비행 검증은 아직 남아 있습니다.
   카메라 간 대응·3D 좌표·거리·충돌 경고는 포함하지 않습니다.
3. **03 — 3D tracking** (`v0.6.0`): 체커보드 내부·스테레오 외부 보정(`calibration.py`), 렌즈 왜곡 보정,
   기준선 길이 설정, 프레임 시간 동기화 측정·보정, 에피폴라 기하 기반 카메라 간 드론 대응 및 모호성 기각,
   스테레오 삼각측량(`stereo_3d.py`)과 재투영 오차·신뢰도 평가. 합성 데이터 검증 완료, 실제 비행 보정·검증 전. [설명서](stereo-3d-and-collision.md).
4. **04 — Collision warning** (`v0.6.0`): 두 드론 3D 거리·유효 거리(불확실성 차감)·속도·상대 접근 속도·CPA(`t_cpa`, `d_cpa`) 기반
   위험 단계(`SAFE`/`CAUTION`/`DANGER`/`UNKNOWN`) 판정 및 시각·청각 경고(`collision.py`). 미보정·비동기 시 `판단 불가(UNKNOWN)` 처리.
   자동 비행 제어는 수행하지 않으며 실제 비행 검증 전입니다.
5. **05 — Dashboard** (`v0.6.0`): 3구역 관제 대시보드(`gui.py`), 3D 공간 시각화(`dashboard_widgets.py`),
   스테레오 보정 마법사, 실험 데이터 JSONL/AVI 기록 및 CSV 내보내기(`recorder.py`), 기록 재생·분석 창(`PlaybackDialog`).

## 실제 장비 검증 과제

- (Phase 03~05) 실제 카메라 2대 체커보드 스테레오 보정, 정적/비행 중 3D 위치 및 드론 간 거리 정확도, 프레임 촬영 시각 동기화 오차,
  충돌 위험 경고 정확도·지연 측정([현장 실험 절차서](stereo-3d-and-collision.md#5-실제-드론-2대--카메라-2대-현장-실험-절차-사전-준비-가이드)).
- (Phase 02) 실제 비행 영상에서 탐지율·오탐·작은/먼 드론·프로펠러·역광, 두 드론 교차 시 ID 유지, Windows CPU PC의 AI FPS,
  Intel Mac(torch 2.2.2) 및 CUDA PC 동작.
- iOS device와 두 번째 iOS 또는 Android 장치의 앱·코덱·MJPEG/RTSP 주소 확인.
- 같은 LAN에서 AP 클라이언트 격리·방화벽·장치 절전·화면 잠금·스트림 인증 확인.
- Windows 10/11, Apple Silicon, 가능한 Intel Mac에서 권한·USB 드라이버·백엔드 점검.
- 720p30 두 스트림의 수신 FPS, 지연, CPU·메모리, 장시간 동작, 네트워크 단절·복구 측정.
- **수신 시각은 촬영 시각이 아닙니다.** 3D 계산 전 별도의 카메라 동기화·지연 보정 실험이 필요합니다.
