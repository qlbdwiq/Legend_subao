# Drone Collision Detection System

외부 카메라 두 대로 DJI Mavic Mini 두 대를 추적하고, 스테레오 보정·삼각측량으로
3D 좌표를 계산하여 충돌 위험을 경고하는 시스템을 단계별로 개발하는 Python monorepo입니다.
**현재 구현 범위는 1단계(두 카메라 수신·표시)와 2단계(YOLO11 드론 탐지·ByteTrack 추적·바운딩 서클, 2D 픽셀 좌표)입니다.**
이 버전으로 드론의 3차원 위치, 드론 간 거리, 충돌 위험을 판단할 수 없습니다.

## 개발 단계

| 단계 | 내용 | 구현 상태 |
| --- | --- | --- |
| 01 | 두 카메라 실시간 수신·표시 | 코드 구현, 검증 범위는 아래 참고 |
| 02 | YOLO 드론 인식·추적 (2D) | 코드 구현, 합성 영상까지 검증 — [설명서](docs/ai-detection.md). 실제 비행 검증 전 |
| 03 | 카메라 보정·삼각측량·3D 추적 | 계획 |
| 04 | 거리 계산·충돌 위험 경고 | 계획 |
| 05 | 관제 대시보드·실험 데이터 분석 | 계획 |

## 구조

```text
.github/workflows/      Windows/macOS CI 및 수동 릴리즈 워크플로
apps/android-camera/    Galaxy/Android Camera2 송출 앱, APK 빌드
apps/ios-camera/        iPhone/iPad AVFoundation 송출 앱, Xcode 프로젝트
stages/01-camera-stream/
  src/drone_camera/     Qt와 독립된 설정·영상 소스 모듈, Qt GUI
  src/main.py           실행 진입점
  tests/               설정·수신·오류·재연결·headless GUI 검증
  config.example.json
  requirements.txt
  README.md
docs/ai-detection.md   YOLO11 + ByteTrack 사용법·구조·데이터 구조·라이선스
docs/roadmap.md
docs/releases/           버전별 릴리즈 노트와 단계별 분류
docs/mobile/README.md   모바일 설치 및 Wi-Fi·USB 연결
scripts/                합성 테스트 영상 및 소스 ZIP 생성
```

카메라 장치는 iPhone, iPad, Android 또는 USB 웹캠을 사용할 수 있습니다.
**자체 Drone Camera 모바일 앱과 Wi-Fi·USB 연결**을 `v0.2.0`에 추가했습니다.
Android는 개발용 서명 APK, iPhone은 무료 Apple 계정으로 직접 설치할 Xcode 프로젝트와
서명 전 IPA를 제공합니다. **서명 전 IPA는 그대로 설치할 수 없습니다.**
[설치 및 무선·유선 연결 안내](docs/mobile/README.md)를 참고하세요.
기존 외부 MJPEG/RTSP 앱 입력도 계속 지원하며, 기본 카메라 앱이나 브라우저 시청 페이지를
직접 스트림으로 가정하지 않습니다.

## 빠른 시작

**Windows 일반 사용자는 Python 설치 없이 EXE를 실행할 수 있습니다.**
[최신 릴리즈](https://github.com/qlbdwiq/Legend_subao/releases/latest)의 `…-windows-x64.exe`를
내려받아 더블클릭하고 설정에서 카메라를 연결하세요. [EXE 사용 설명](docs/windows-exe.md).
Python·OpenCV·Qt와 Android USB용 adb를 포함합니다. 기기 USB 드라이버·권한 승인은 필요합니다.

**macOS 사용자는 Python 설치 없이 DMG를 사용할 수 있습니다.** 최신 [릴리즈](https://github.com/qlbdwiq/Legend_subao/releases/latest)에서
Apple Silicon은 `…-macos-arm64.dmg`, Intel Mac은 `…-macos-x64.dmg`를 받으세요.
Developer ID 서명·공증이 없어 첫 실행에 보안 경고가 나올 수 있습니다. [설치·해결 방법](docs/macos-app.md).

아래 Python 설치는 **소스 개발**용입니다.

Python **3.11 또는 3.12**를 권장합니다(Intel Mac은 3.11). Windows 10/11 x86-64, macOS Apple Silicon 및
Intel을 대상으로 같은 소스를 사용합니다. 2단계 AI(PyTorch)는 GPU 없이 CPU로 동작하며, CUDA·Apple GPU는 선택 사항입니다.
Python 3.13 이상은 이 릴리즈의 고정 의존성·CI 지원 범위가 아닙니다. AI 모델 파일(`best.pt`)은 저장소·릴리스에 포함되지 않으며
프로그램의 **모델 불러오기**로 직접 선택합니다. **Ultralytics는 AGPL-3.0**이므로 [라이선스 안내](docs/ai-detection.md#모델과-라이선스)를 확인하세요.

저장소 루트에서:

```powershell
# Windows PowerShell: 가상환경 활성화나 실행 정책 변경 불필요
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r stages/01-camera-stream/requirements.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -r stages/01-camera-stream/requirements-ultralytics.txt
.\.venv\Scripts\python.exe stages/01-camera-stream/src/main.py
```

```bash
# macOS
python3.11 -m venv .venv
.venv/bin/python -m pip install -r stages/01-camera-stream/requirements.txt
.venv/bin/python -m pip install --no-deps -r stages/01-camera-stream/requirements-ultralytics.txt
.venv/bin/python stages/01-camera-stream/src/main.py
```

상단 **설정**에서 두 카메라의 입력 종류와 주소/장치 번호/파일 경로를 지정하고 저장한 후,
각 카메라 또는 전체 연결을 누릅니다. 설정은 OS의 사용자 설정 폴더에 저장됩니다.
자세한 장비 준비·테스트 영상·문제 해결은 [1단계 사용 설명](stages/01-camera-stream/README.md)을 참고하세요.

## 검증 및 릴리즈

```bash
# 위에서 만든 가상환경의 python 사용 (Windows는 해당 python.exe 경로로 교체)
.venv/bin/python -m pytest stages/01-camera-stream/tests -v
.venv/bin/python stages/01-camera-stream/src/main.py --validate-config stages/01-camera-stream/config.example.json
.venv/bin/python scripts/package_release.py
```

CI는 PR과 `main` push에 Windows/macOS Python 3.11·3.12 및 macOS Intel 3.11 검사를
실행하도록 구성되어 있습니다. CI 설정 파일의 존재는 실행 성공을 의미하지 않습니다.
실제 통과 상태는 저장소 Actions 및 PR checks에서 확인해야 합니다.
모바일 CI는 Android APK·단위 테스트·lint·서명 확인, iOS 시뮬레이터 테스트·실제 iPhone arm64
컴파일 및 서명 전 IPA·Xcode 프로젝트 생성을 추가로 수행합니다.

[GitHub Releases](https://github.com/qlbdwiq/Legend_subao/releases)에서 게시된 버전을 확인할 수 있으며,
[릴리즈 단계 분류표](docs/releases/README.md)에서 버전별 구현 범위를 확인할 수 있습니다.
버전 태그(`vX.Y.Z`) 또는 단계 태그(`phase-NN-vX.Y.Z`, 예: `phase-01-v0.5.0`)를 푸시하면
테스트를 다시 실행한 뒤 Windows EXE·macOS DMG(arm64/x64)·소스를 한 릴리즈로 자동 게시합니다.
단계 태그 릴리즈에는 `Phase NN`이 제목에 표시되어 1차 구현, 2차 구현처럼 구분됩니다.
패키지 버전과 `docs/releases/vX.Y.Z.md`가 태그 버전과 일치해야 하며, 기존 릴리즈는 덮어쓰지 않습니다. `v0.2.1`은 보존됩니다.
모바일 APK·서명 전 IPA·Xcode 프로젝트는 최신 릴리스에서 함께 제공합니다.
기존 `v0.1.0` 태그와 게시 파일은 보존합니다.
태그·릴리즈의 실제 게시 상태는 Releases 페이지를 기준으로 하세요. macOS `.app`은 제공하지 않습니다.

로컬 합성 영상·MJPEG 테스트나 CI 통과는 실제 iPhone/Android 스트림, USB 드라이버 또는
모든 Windows/macOS 장비에서의 성능을 보장하지 않습니다. 수신기의 720p30 두 채널은 회귀 기준입니다.
자체 모바일 앱은 실시간 스트리밍 형식을 열거해 기본 720p30에서 기기 지원 상한(최대 3840×2160·240 FPS)까지
선택할 수 있고, 데스크톱은 설정값이 아닌 실측 수신 해상도·FPS를 함께 표시합니다. 고해상도·고FPS 실장비 검증은 남아 있습니다.

라이선스: [MIT](LICENSE). 향후 계획: [로드맵](docs/roadmap.md).
