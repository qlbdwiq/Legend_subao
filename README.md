# Drone Collision Detection System

외부 카메라 두 대로 DJI Mavic Mini 두 대를 추적하고, 스테레오 보정·삼각측량으로
3D 좌표를 계산하여 충돌 위험을 경고하는 시스템을 단계별로 개발하는 Python monorepo입니다.
LiDAR는 사용하지 않습니다. **현재 구현 범위는 1단계: 두 카메라 영상 수신 및 표시뿐입니다.**
이 버전으로 드론 간 거리나 충돌 위험을 판단할 수 없습니다.

## 개발 단계

| 단계 | 내용 | 구현 상태 |
| --- | --- | --- |
| 01 | 두 카메라 실시간 수신·표시 | 코드 구현, 검증 범위는 아래 참고 |
| 02 | YOLO 드론 인식·추적 | 계획 |
| 03 | 카메라 보정·삼각측량·3D 추적 | 계획 |
| 04 | 거리 계산·충돌 위험 경고 | 계획 |
| 05 | 관제 대시보드·실험 데이터 분석 | 계획 |

## 구조

```text
.github/workflows/      Windows/macOS CI 및 수동 릴리즈 워크플로
stages/01-camera-stream/
  src/drone_camera/     Qt와 독립된 설정·영상 소스 모듈, Qt GUI
  src/main.py           실행 진입점
  tests/               설정·수신·오류·재연결·headless GUI 검증
  config.example.json
  requirements.txt
  README.md
docs/roadmap.md
docs/releases/v0.1.0.md  릴리즈 노트 원고
scripts/                합성 테스트 영상 및 소스 ZIP 생성
```

카메라 장치는 iPhone, iPad, Android 또는 USB 웹캠을 사용할 수 있습니다. 휴대폰·태블릿은
직접 MJPEG/RTSP 주소를 제공하는 별도 스트리밍 앱이 필요합니다. iOS/Android 기본 카메라
앱이나 브라우저 시청 페이지가 스트림을 제공한다고 가정하지 않습니다.

## 빠른 시작

Python **3.11 또는 3.12**를 권장합니다. Windows 10/11 x86-64, macOS Apple Silicon 및
Intel을 대상으로 같은 소스를 사용합니다. CPU만 필요하며 CUDA는 사용하지 않습니다.
Python 3.13 이상은 이 릴리즈의 고정 의존성·CI 지원 범위가 아닙니다.

저장소 루트에서:

```powershell
# Windows PowerShell: 가상환경 활성화나 실행 정책 변경 불필요
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r stages/01-camera-stream/requirements.txt
.\.venv\Scripts\python.exe stages/01-camera-stream/src/main.py
```

```bash
# macOS
python3.11 -m venv .venv
.venv/bin/python -m pip install -r stages/01-camera-stream/requirements.txt
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

[GitHub Releases](https://github.com/qlbdwiq/Legend_subao/releases)에서 게시된 버전을 확인할 수 있습니다.
`v0.1.0` 릴리즈용 소스 ZIP 생성기와 기본 브랜치 전용 수동 게시 워크플로를 제공합니다.
태그·릴리즈의 실제 게시 상태는 Releases 페이지를 기준으로 하세요. `.exe` / `.app`은 제공하지 않습니다.

로컬 합성 영상·MJPEG 테스트나 CI 통과는 실제 iPhone/Android 스트림, USB 드라이버 또는
모든 Windows/macOS 장비에서의 성능을 보장하지 않습니다. 720p30 두 채널은 실장비 검증 목표입니다.

라이선스: [MIT](LICENSE). 향후 계획: [로드맵](docs/roadmap.md).
