# Phase 01 — Cross-platform Dual Camera Streaming

두 영상만 수신·표시하는 데스크톱 프로그램입니다. iPhone/iPad/Android의 스트리밍 앱,
USB 웹캠, 로컬 동영상을 혼합해 사용할 수 있습니다. LiDAR·YOLO·3D·충돌 경고는 포함하지 않습니다.

## Windows 설치 및 실행

Python.org의 **Python 3.11 또는 3.12 x86-64**를 설치합니다. PowerShell에서 저장소 루트로 이동하여:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r stages/01-camera-stream/requirements.txt
.\.venv\Scripts\python.exe stages/01-camera-stream/src/main.py
```

가상환경을 활성화하지 않고 그 안의 Python을 직접 실행하므로 PowerShell 실행 정책을 바꿀 필요가 없습니다.
Windows 10/11의 **설정 → 개인정보(및 보안) → 카메라 → 데스크톱 앱 카메라 접근 허용**을 확인하세요.
USB 카메라를 다른 프로그램이 점유하면 해제한 후 다시 연결하세요.

## macOS 설치 및 실행

Python.org 또는 이미 사용 중인 패키지 관리 도구로 Python 3.11/3.12를 준비합니다.
Apple Silicon에서는 native arm64 Python, Intel에서는 x86-64 Python을 권장합니다.
서로 다른 아키텍처의 가상환경을 복사해 재사용하지 마세요.

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r stages/01-camera-stream/requirements.txt
.venv/bin/python stages/01-camera-stream/src/main.py
```

터미널/Python의 카메라 사용 요청을 허용하세요. **시스템 설정 → 개인정보 보호 및 보안 → 카메라**와
해당되는 macOS 버전의 **로컬 네트워크** 권한도 확인하세요. 의존성 휠을 받을 수 있는 macOS 버전이
필요합니다(PySide6 6.8 계열은 macOS 12 이상을 권장). Intel 지원은 CI 및 실제 장비 확인 범위에서 판단합니다.
GPU·CUDA·별도 iOS/Android 앱 빌드는 필요하지 않습니다.

## iPhone / iPad / Android 스트림 준비

1. 노트북과 각 장치를 같은 신뢰할 수 있는 Wi-Fi/LAN에 연결합니다. 서로 다른 OS를 섞어도 됩니다.
2. 장치에 **직접 HTTP MJPEG 또는 RTSP 영상 주소를 제공하는 카메라 스트리밍 앱**을 준비합니다.
   앱 설명에서 프로토콜, 후면 카메라 선택, 해상도/FPS, 잠금 중 송출 여부를 확인하세요.
   기본 카메라 앱, HLS/WebRTC 전용 앱, 웹 시청 페이지 주소는 이 입력으로 사용할 수 없습니다.
3. 앱에서 일반 후면 카메라, 우선 1280×720·30fps를 선택하고 송출을 시작합니다.
4. 앱이 표시하는 **정확한 스트림 URL**을 확인합니다. 다음은 형식 예시일 뿐 실제 주소가 아닙니다.
   `http://192.168.1.20:8080/video`, `rtsp://192.168.1.21:8554/live`.
   경로·포트·인증은 앱마다 다릅니다. 광고 페이지 URL이나 앱의 설정 페이지를 넣지 마세요.
5. PC의 VLC 등으로 해당 주소에서 영상이 나오는지 먼저 확인하면 네트워크 문제를 분리할 수 있습니다.
6. 앱 카메라·로컬 네트워크 권한, 배터리 절약/자동 잠금, AP 클라이언트 격리, 방화벽을 확인합니다.
   필요한 로컬 접근만 허용하고 OS 방화벽 전체를 끄지 마세요.

스트림을 인터넷에 공개할 필요는 없습니다. 앱에 인증을 설정하고 신뢰할 수 있는 LAN을 사용하세요.
URL 비밀번호는 사용자 설정 파일에 저장됩니다. 설정을 Git에 추가하거나 화면 캡처로 공유하지 마세요.
오류 안내에는 주소·인증 정보를 출력하지 않습니다.

## 연결 및 설정 저장

- 상단 또는 하단 **설정**에서 이름·입력 종류·주소/장치 번호·타임아웃·자동 재시도 횟수를 지정합니다.
- `mjpeg`: HTTP(S) 직접 MJPEG 주소, `rtsp`: RTSP 주소, `usb`: 정수 장치 번호(보통 0/1),
  `file`: 동영상 경로. 파일은 절대 경로를 권장합니다.
- **Camera 01/02 연결**로 독립 연결하거나 **전체 연결**을 누릅니다. 오류가 있는 한쪽만 해제·재연결할 수 있습니다.
- 설정 변경 전 두 연결을 해제하세요. 저장은 임시 파일 후 원자적으로 교체합니다.
- 기본 설정 파일은 Qt의 사용자별 `AppConfigLocation/DroneCameraPhase01` 계열 경로에 생성됩니다.
  보통 Windows `%LOCALAPPDATA%` 아래, macOS `~/Library/Preferences` 아래이며 Qt/OS 설정에 따라 달라집니다.
  정확한 위치가 필요하면 아래처럼 직접 파일을 지정하세요. 기본 설정은 저장 버튼을 누르기 전에는 생성하지 않습니다.

```powershell
# 저장소 루트, Windows
Copy-Item stages/01-camera-stream/config.example.json stages/01-camera-stream/config.json
.\.venv\Scripts\python.exe stages/01-camera-stream/src/main.py --config stages/01-camera-stream/config.json
```

```bash
# 저장소 루트, macOS
cp stages/01-camera-stream/config.example.json stages/01-camera-stream/config.json
.venv/bin/python stages/01-camera-stream/src/main.py --config stages/01-camera-stream/config.json
```

`config.json`은 `.gitignore`에 포함됩니다. 예제에는 실제 주소나 인증 정보가 없습니다.
설정의 두 카메라 ID는 `camera-01`, `camera-02`로 고정하며 이름은 바꿀 수 있습니다.
상대 파일 경로는 **설정 JSON이 있는 폴더**를 기준으로 해석합니다. `--config`에 없는 파일을
지정하면 오류로 종료합니다. 파일을 복사하거나 GUI의 기본 설정 저장을 먼저 사용하세요.

## 실제 카메라 없이 테스트

어떤 일반 동영상 파일도 사용할 수 있고 같은 파일을 두 입력에 지정할 수 있습니다.
합성 720p30 영상 생성은 저장소 전체를 내려받았을 때 루트에서:

```bash
.venv/bin/python scripts/make_test_video.py /tmp/drone-test.avi --seconds 10
```

Windows에서는 `.venv/bin/python`을 `.\.venv\Scripts\python.exe`로, 출력 경로를
`"$env:TEMP\drone-test.avi"`로 바꾸세요. 기존 파일은 덮어쓰지 않습니다.
릴리즈 ZIP에도 생성기가 포함됩니다. 가지고 있는 일반 동영상을 사용해도 됩니다.

설정에서 두 카메라를 모두 `file`로 지정하고 생성한 파일의 절대 경로를 입력합니다.
**파일 반복**을 켜면 반복 재생합니다. 꺼두면 끝에서 `End of file` 상태가 됩니다.
재생은 파일의 FPS에 맞춰 진행하고 잘못된 FPS 메타데이터는 30fps로 대체합니다.
파일 영상의 FPS 표시는 디코딩 최대 속도가 아닌 재생 주기에 따른 수신 속도입니다.

## 구조와 동작

- 카메라마다 `spawn` 프로세스 하나를 사용합니다. GUI 메인 스레드는 네트워크/장치를 읽지 않습니다.
- 카메라마다 고정 공유 버퍼 **한 장**만 유지합니다. GUI가 느리면 중간 프레임을 버리므로
  이벤트 큐에 오래된 영상이 계속 쌓이지 않습니다. 외부 카메라 앱/FFmpeg 자체 버퍼 지연까지
  제거한다고 보장하지는 않습니다.
- `CameraSource.latest_frame(after_sequence)`은 소유권이 분리된 최신 BGR NumPy 배열과
  카메라 ID·프레임 번호·수신 wall clock·수신 monotonic time을 반환합니다.
  GUI와 무관한 이 API를 나중에 AI 소비자가 재사용할 수 있습니다.
- **수신 시각은 촬영 시각이 아닙니다.** 두 카메라의 촬영 시각 동기화나 3D 대응은 아직 없습니다.
- FPS는 최근 최대 120개 실제 읽기 완료 시각으로 계산합니다. 연결 직후에는 0부터 시작하며
  수신이 정지하거나 연결이 끊기면 0으로 표시합니다. 해상도는 실제 프레임에서 읽습니다.
- 네트워크 OpenCV FFmpeg backend에 연결·읽기 타임아웃을 전달합니다. 일반 open/read 실패는
  설정된 횟수만큼 자동 재시도합니다. 최대 재시도는 세션 전체 예산이며 수동 연결로 초기화됩니다.
- 네이티브 USB/file backend가 타임아웃을 무시하면 watchdog이 프로세스를 종료하고 오류를
  표시합니다. 이 경우 **수동으로 다시 연결**하세요. 기본 타임아웃에 프로세스 시작 여유 3초를 둡니다.
- 연결 해제는 비동기 요청입니다. 1초 내 종료되지 않으면 프로세스를 종료하고, 필요하면 강제 종료합니다.
  이는 Python 스레드 강제 종료를 사용하지 않으며 장치 핸들은 자식 프로세스와 함께 반환됩니다.
  프로그램 종료 시에는 모든 자식 프로세스를 join하고 공유 메모리를 해제합니다.
- 최대 입력은 3840×2160 BGR, 버퍼는 채널당 약 25MB입니다. 설정·실제 입력에 따라 CPU/메모리 사용량이 다릅니다.

## 검증

저장소 루트에서(Windows에서는 가상환경 Python 경로를 교체):

```bash
.venv/bin/python -m compileall -q stages/01-camera-stream/src
.venv/bin/python stages/01-camera-stream/src/main.py --validate-config stages/01-camera-stream/config.example.json
.venv/bin/python -m pytest stages/01-camera-stream/tests -v
```

단계 폴더 안에서는 `python -m pytest -v`, `python src/main.py`로 독립 실행할 수 있습니다.
테스트는 실제 임시 AVI, loopback MJPEG 서버, 실패/정지 backend, offscreen Qt 초기화·두 채널 표시·종료를
검증합니다. RTSP backend 전달·오류 경로는 검증하지만 실제 RTSP 장치·USB 장치는 별도 검증이 필요합니다.
CI에서 `QT_QPA_PLATFORM=offscreen`을 사용하며 실제 디스플레이·카메라는 요구하지 않습니다.

## 문제 해결과 제한

| 증상 | 확인할 사항 |
| --- | --- |
| 연결 실패 | 앱 송출, 정확한 직접 URL, 주소 변경, 장치 번호, 다른 앱의 카메라 점유, 권한 |
| RTSP 재생 불가 | 앱의 코덱과 OpenCV FFmpeg 호환성, VLC 동작, Wi-Fi/방화벽 |
| 프레임 중단 | 앱 절전/화면 잠금, Wi-Fi 품질, 재시도 예산 소진, 수동 재연결 |
| HTTPS 실패 | 스트리밍 앱의 인증서·서버 설정을 점검; 검증을 끄지 않고 올바른 인증서를 사용 |
| GUI가 작거나 느림 | 해상도/FPS 낮추기, 두 개의 CPU 디코딩 부하와 OS 배율 확인 |
| 파일이 열리지 않음 | 설정 파일 기준 상대 경로, 절대 경로, 지원 코덱·손상 여부 확인 |
| PySide6 설치 실패 | Python 3.11/3.12, OS/CPU 아키텍처, 지원 macOS 버전, 새 가상환경 확인 |

720p30 두 채널은 초기 실험 목표이며 이 버전이 모든 노트북에서 보장하지 않습니다.
CI 성공도 실제 iPhone/Android 앱·USB 드라이버·장시간 안정성 검증을 대체하지 않습니다.
