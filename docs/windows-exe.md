# Windows — Python 설치 없이 실행

Windows 10/11 **x86-64**용 단일 EXE입니다. Python·pip·OpenCV·PySide6를 따로 설치할 필요가 없습니다.
Windows ARM 및 macOS에서 실행할 EXE는 아닙니다. macOS는 기존 Python 소스를 사용합니다.

1. [v0.2.1 릴리즈](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.2.1)에서
   `DroneCollisionDetection-v0.2.1-windows-x64.exe`를 내려받습니다.
2. EXE를 더블클릭합니다. 관리자 권한·Python 설치·가상환경 활성화는 필요하지 않습니다.
3. 설정에서 휴대폰 앱의 URL을 입력합니다. Wi-Fi는 `mjpeg`, 케이블은 **유선 USB 연결 도우미**를 사용합니다.
4. 연결 또는 전체 연결을 누릅니다.

`DroneCollisionDetection-v0.2.1-windows-x64.zip`에는 같은 EXE, 설명서, 설정 예제와 라이선스가 있습니다.
ZIP을 받았다면 먼저 압축을 풀고 `DroneCollisionDetection.exe`를 실행하세요.
EXE 내부 라이브러리를 임시 폴더로 푸는 최초 실행에는 시간이 조금 걸릴 수 있습니다.
프로그램과 별도로 설치하는 Python 런타임은 없으며, 설정은 기존 사용자별 설정 폴더에 저장합니다.

## 무선·유선 연결

- **Wi-Fi:** 기존 iPhone/Galaxy 앱과 같은 네트워크에서 연결합니다.
- **Android USB:** 공식 `adb` 도구를 내장했습니다. 별도의 Platform-Tools 설치나 PATH 설정은 필요 없습니다.
  데이터 케이블, USB 디버깅 승인, 기기에 필요한 Windows USB 드라이버는 있어야 합니다.
- **iPhone USB:** Apple Devices/iTunes의 Mobile Device 드라이버·서비스와 기기 신뢰 승인이 필요합니다.
  Apple 드라이버나 권한 승인을 EXE가 대체하지 않습니다.

Android 도구는 실행 중 잠길 수 있는 공유 서버를 사용하므로 사용자 `LOCALAPPDATA/DroneCollisionDetection/Tools`
폴더에 무결성을 확인한 사본을 자동으로 준비합니다. 설정이나 개인 영상은 EXE 옆에 쓰지 않습니다.
기존 모바일 앱은 [v0.2.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.2.0)의 APK/Xcode 프로젝트를 그대로 사용합니다.
자세한 휴대폰 설치·연결은 [모바일 설명서](https://github.com/qlbdwiq/Legend_subao/blob/main/docs/mobile/README.md)를 참고하세요.

## 검증과 제한

Windows CI는 빌드한 EXE 하나만 한글·공백 경로에 복사한 후 Python·SDK PATH를 제거한 환경에서 실행합니다.
실제 Qt Windows GUI, 두 720p 파일, 두 HTTP MJPEG 입력, 프레임 표시·FPS·자식 프로세스 종료,
내장 adb 실행을 확인하고 JSON 결과와 화면 캡처를 남깁니다.
CI 머신에 설치된 Python을 제거한 별도 VM에서 검증한 것은 아니며, 런타임은 EXE에 포함됩니다.
실제 휴대폰·USB 드라이버·모든 Windows 10/11 장비를 검증한 것으로 간주하지 않습니다.

이 개발 배포본에는 Windows Authenticode 게시자 서명이 없습니다. Windows가 게시자 확인을 표시할 수 있으며,
조직 관리 정책이 차단하는 경우 해당 정책을 우회하지 않습니다. 릴리즈의 `SHA256SUMS.txt`로 다운로드 파일을 확인할 수 있습니다.

## 개발자가 다시 빌드하기

Windows에서 Python 3.11 x64 개발 환경을 준비한 뒤 저장소 루트에서:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r stages/01-camera-stream/requirements.txt -r packaging/windows/requirements-build.txt
.\.venv\Scripts\python.exe scripts/build_windows.py
.\.venv\Scripts\python.exe scripts/smoke_windows_exe.py dist/windows/DroneCollisionDetection-v0.2.1-windows-x64.exe build/windows/evidence
```

개발 빌드만 Python을 사용하며, 배포된 EXE 사용자에게는 필요 없습니다.
