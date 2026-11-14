## 목적 및 구현

1단계에서 카메라 두 대의 영상을 CPU로 동시 수신·표시하는 Windows/macOS 데스크톱 앱을 추가합니다.
iPhone·iPad·Android의 스트리밍 앱 직접 MJPEG/RTSP 주소, USB 웹캠, 로컬 동영상을 사용할 수 있습니다.
드론 인식·3D·충돌 경고 기능은 이번 변경에 포함하지 않습니다.

- 카메라별 spawn 수신 프로세스, 한 장의 공유 최신 프레임, 수신 시각·번호·해상도 메타데이터.
- 독립 연결·해제·유한 횟수 자동 재연결, 수동 재연결, 타임아웃과 멈춘 backend 종료.
- PySide6 다크 GUI, 비율 유지, 실제 수신 FPS·해상도·상태·수신 경과 시간.
- 검증된 JSON 설정 저장, 비밀정보 제외, 파일 재생 및 반복, 종료 시 자원 정리.

## 구조 및 플랫폼

`stages/01-camera-stream`에서 독립 실행·테스트합니다. 공통 문서는 `docs`, CI와 릴리즈 자동화는
`.github/workflows`, 합성 영상·소스 ZIP 생성기는 `scripts`에 있습니다.
설정·수신 모듈은 Qt와 분리해 다음 단계에서 재사용할 수 있습니다.

동일 Python 3.11/3.12 코드베이스로 Windows 10/11 x86-64, macOS Apple Silicon/Intel을 대상으로 합니다.
CI matrix: windows-latest와 macos-latest 각각 Python 3.11/3.12, macos-15-intel Python 3.11.

## 테스트

- 로컬 Linux Python 3.12: pytest **40개 통과**.
- 실제 임시 AVI 두 채널, HTTP MJPEG loopback, FPS·최신 프레임·파일 종료·실패 격리.
- 자동 재시도·복구, RTSP 타임아웃 전달, USB backend 전달, 멈춘 backend 종료·watchdog.
- offscreen Qt 초기화·두 채널 표시·비율 유지·독립 해제·종료, 설정 왕복·오류 검증.
- Windows의 낮은 monotonic clock 해상도에서 같은 시각의 연속 프레임을 처리할 때
  FPS 계산이 0으로 나뉘지 않도록 고해상도 performance counter와 회귀 테스트를 추가했습니다.
- 별도 720p30 합성 파일 두 채널 실행: 각각 약 30 FPS, GUI 응답과 모든 worker 종료 확인.
- 소스 compile/import, 예제 설정 CLI 검증, ZIP 무결성과 압축 해제 후 테스트 확인.

위 결과는 로컬 검증입니다. Windows/macOS 실제 CI 결과는 이 PR의 checks를 기준으로 확인합니다.
실패하거나 실행되지 않은 CI를 통과로 간주하지 않으며 보호 규칙을 우회하지 않습니다.

## 추가 실장비 검증 및 다음 단계

iOS 또는 Android 장치 두 대의 앱·직접 URL·코덱·권한, USB 드라이버, 실제 운영체제,
Wi-Fi 단절·복구, 장시간 지연·720p30 두 채널 CPU 부하를 확인해야 합니다.
수신 시각은 촬영 시각이 아니며 카메라 동기화는 아직 없습니다.
다음 작업은 YOLO 드론 인식·추적이고 이후 카메라 보정·3D 추적, 충돌 경고, 대시보드 순입니다.

병합은 모든 필수 CI·정책을 만족한 후 squash merge합니다. `v0.1.0`은 병합된 기본 브랜치
커밋에서만 게시하고 소스 ZIP을 첨부합니다. `.exe` / `.app`은 제공하지 않습니다.
