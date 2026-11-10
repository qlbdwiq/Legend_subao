# macOS 앱 (DMG)

Python을 따로 설치하지 않아도 실행되는 **macOS 11 이상** 앱을 `.dmg`로 제공합니다.
Windows EXE와 같은 소스·기능(두 카메라 수신, Wi-Fi, Android USB)을 사용합니다.

## 어떤 파일을 받나요?

| Mac | 파일 |
| --- | --- |
| Apple Silicon (M1/M2/M3/M4…) | `DroneCollisionDetection-vX.Y.Z-macos-arm64.dmg` |
| Intel Mac | `DroneCollisionDetection-vX.Y.Z-macos-x64.dmg` |

 메뉴 > 이 Mac에 관하여에서 칩(Apple M…/Intel)을 확인하세요. 릴리즈의 `SHA256SUMS.txt`로 파일을 확인할 수 있습니다.

## 설치

1. DMG를 더블클릭해 엽니다.
2. **Drone Collision Detection** 앱을 **Applications** 폴더로 드래그합니다.
3. Applications에서 앱을 실행합니다.

## 처음 실행할 때 "열 수 없음" 경고가 나오는 경우

이 앱은 **Apple Developer ID로 서명·공증되지 않은** 개발 배포본입니다(아래 "서명과 공증" 참고).
인터넷에서 받은 앱은 macOS Gatekeeper가 처음 한 번 차단할 수 있습니다. 정상적인 허용 방법:

1. 앱을 더블클릭해 경고를 한 번 띄웁니다("확인").
2. **시스템 설정 > 개인정보 보호 및 보안**을 열고 아래쪽의 "Drone Collision Detection이(가) 차단되었습니다" 옆
   **그래도 열기**를 누른 후 관리자 암호/Touch ID로 확인합니다. (macOS 14 이하는 앱을 우클릭 > 열기 > 열기)
3. 이후에는 일반 앱처럼 실행됩니다.

터미널을 선호하면 다음으로 이 앱 하나에 대한 격리 속성만 제거할 수 있습니다(출처를 신뢰할 때만):

```bash
xattr -dr com.apple.quarantine "/Applications/Drone Collision Detection.app"
```

시스템 전체의 Gatekeeper를 끄는 설정은 권하지 않습니다.

## 권한

- **카메라:** USB 웹캠(장치 번호) 입력을 쓸 때 macOS가 카메라 접근을 묻습니다. 허용하세요.
  거부했다면 시스템 설정 > 개인정보 보호 및 보안 > 카메라에서 다시 허용합니다.
- **로컬 네트워크:** 같은 Wi-Fi의 휴대폰 앱(MJPEG/RTSP)에 연결할 때 묻습니다. 허용하세요.
- **Android USB:** 공식 `adb`(arm64·x86_64 겸용)가 앱에 내장되어 있어 Platform-Tools 설치가 필요 없습니다.
  데이터 케이블과 휴대폰의 USB 디버깅 승인은 필요합니다.
- **iPhone USB:** macOS 내장 USB 서비스(usbmuxd)를 사용하므로 별도 드라이버는 필요 없습니다.
  iPhone을 연결해 잠금 해제 후 "이 컴퓨터를 신뢰"를 승인하세요.
- 설정은 `~/Library/Preferences`의 사용자 설정 폴더(Qt AppConfigLocation)에 저장되며 앱 안에는 쓰지 않습니다.

## 서명과 공증

이 배포본에는 **ad-hoc 서명**만 되어 있습니다(Apple Silicon에서 실행되려면 최소한의 서명이 필요). 이는 개발자 신원을
증명하지 않습니다. Developer ID Application 인증서와 Apple 공증(notarization) 인증 정보가 저장소에 없으므로
**Developer ID 서명과 공증은 수행하지 않았고** 임의로 생성하거나 우회하지 않았습니다. 인증서를 준비하면
`scripts/build_macos.py`의 `codesign --sign -` 단계를 인증서로 바꾸고 `xcrun notarytool`/`stapler`를 추가하면 됩니다
(GitHub Secrets로 주입, 저장소에 포함 금지).

## 제한

- CI는 GitHub macOS 러너에서 빌드·마운트·실행(두 파일·두 MJPEG 입력)을 검증합니다. **실제 사용자 Mac, 실제 카메라,
  실제 휴대폰 USB는 검증하지 않았습니다.**
- 3D·AI·충돌 경고 기능은 없습니다(1단계 범위).

## 개발자가 다시 빌드하기

macOS에서 Python 3.11 개발 환경을 준비한 뒤 저장소 루트에서:

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r stages/01-camera-stream/requirements.txt -r packaging/macos/requirements-build.txt
.venv/bin/python scripts/build_macos.py
.venv/bin/python scripts/smoke_macos_app.py dist/macos/DroneCollisionDetection-v*-macos-*.dmg build/macos/evidence
```

실행한 Mac의 CPU(arm64 또는 x64)용 DMG가 만들어집니다.
