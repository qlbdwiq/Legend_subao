# Drone Camera — 휴대폰 앱 / Wi-Fi·USB 연결

`v0.2.0`은 기존 1단계 수신 프로그램에 자체 Android·iPhone 카메라 송출 앱과
USB 연결 도우미를 추가합니다. 두 드론 인식·3D·충돌 경고는 포함하지 않습니다.
같은 PC에 휴대폰 두 대를 연결할 수 있고 iPhone+Galaxy, 무선+USB를 섞어도 됩니다.

## 다운로드 파일의 차이

GitHub [Releases](https://github.com/qlbdwiq/Legend_subao/releases)에서 실제 게시된 버전을 확인하세요.

| 파일 | 용도 |
| --- | --- |
| `drone-camera-android-v0.2.0-debug.apk` | Android/Galaxy에 설치 가능한 개발용 서명 APK |
| `drone-camera-ios-v0.2.0-xcode-project.zip` | Mac의 Xcode에서 무료 Apple 계정으로 직접 서명·설치할 프로젝트 |
| `drone-camera-ios-v0.2.0-unsigned.ipa` | 실제 iPhone arm64 빌드의 **서명 전** 패키지. 그대로 설치할 수 없음 |
| `DroneCollisionDetection-v0.2.1-windows-x64.exe` | v0.2.1 릴리즈의 Python 설치 없는 Windows 데스크톱 |
| `phase-01-camera-stream-v0.2.1.zip` | v0.2.1 릴리즈의 Windows/macOS 데스크톱 Python 소스 |

APK는 Play Store용 배포 키가 아닌 CI 개발 키로 서명됩니다. 다른 CI 빌드로 업데이트할 때
서명이 다르면 기존 앱을 삭제한 뒤 재설치해야 합니다. IPA에 Apple 서명을 위조하거나
임의 프로비저닝 프로파일을 넣지 않습니다. Windows EXE는 `v0.2.1`에서 제공하며 macOS `.app`/DMG는 `v0.3.0`부터 제공합니다.

## Android / Galaxy 설치

1. APK를 내려받아 휴대폰에서 엽니다. 설치를 요청한 앱에 대해 기기가 요구하는 설치 권한을 허용합니다.
2. **Drone Camera**를 실행하고 카메라 권한을 허용합니다. Android 8.0(API 26) 이상이 대상입니다.
3. 휴대폰을 가로로 놓고 **송출 시작**을 누릅니다. 일반 후면 카메라만 사용합니다.
4. 화면의 연결 URL을 복사합니다. 앱은 최대 1280×720, JPEG 품질 70, 최대 30fps (15fps 선택 가능)를 목표로 합니다.
   지원 해상도는 카메라가 제공하는 크기 중 선택하며 기기·발열·연결 상황에 따라 실제 성능이 달라집니다.

마이크·위치·LiDAR·저장소 권한은 요구하지 않습니다. 녹화하거나 외부 서버로 업로드하지 않습니다.

## iPhone — 무료 Apple 계정 + Mac에서 직접 설치

1. Mac에 Xcode와 iPhone 플랫폼 지원을 설치합니다. 앱의 iOS 대상 버전은 **iOS 16 이상**입니다.
2. `drone-camera-ios-v0.2.0-xcode-project.zip`을 풀고 `DroneCamera.xcodeproj`를 엽니다.
3. Xcode **Settings → Accounts**에서 본인 Apple 계정을 추가합니다. 계정/비밀번호는 채팅에 보내지 않습니다.
4. 프로젝트의 **DroneCamera target → Signing & Capabilities**에서 **Automatically manage signing**을 켜고
   본인 **Personal Team**을 선택합니다. Bundle Identifier가 등록되지 않으면 `com.본인고유이름.dronecamera`로 바꿉니다.
5. 데이터 전송 가능한 USB 케이블로 iPhone을 연결하고 잠금을 해제한 뒤 **이 컴퓨터를 신뢰**합니다.
6. Xcode 실행 장치에서 실제 iPhone을 선택해 **Run(▶)** 합니다. 기기에서 Developer Mode 활성화를
   요청하면 **설정 → 개인정보 보호 및 보안 → 개발자 모드**에서 설정하고 안내에 따라 재시동합니다.
7. 개발자 신뢰 안내가 나오면 **설정 → 일반 → VPN 및 기기 관리**에서 본인의 개발자 서명을 확인합니다.
8. 앱을 열고 카메라·로컬 네트워크 권한을 허용한 뒤 송출을 시작합니다.

무료 Personal Team 프로비저닝은 보통 7일 등 Apple이 정한 유효기간과 기기 제한이 있습니다.
만료되면 Xcode에서 다시 실행·설치해야 합니다. 서명 전 IPA를 이름만 바꾸거나 Finder에 끌어
넣는 것으로는 설치되지 않습니다. 이 릴리즈는 TestFlight/App Store 배포를 제공하지 않습니다.
Xcode 프로젝트 ZIP에는 테스트도 포함되어 있습니다. 자동 생성 프로젝트가 필요하면 저장소의
`apps/ios-camera`에서 `brew install xcodegen` 후 `xcodegen generate`를 실행해도 됩니다.

## 무선 Wi-Fi 연결

1. 휴대폰과 PC를 **같은 Wi-Fi/LAN**에 연결합니다. 인터넷 연결 자체는 필요하지 않습니다.
2. 휴대폰 앱의 송출을 시작하고 전체 URL을 복사합니다.
3. 데스크톱 앱의 **설정**에서 연결 방식 `Wi-Fi / 일반 입력`, 입력 종류 `mjpeg`를 선택하고 URL을 붙여 넣습니다.
4. 저장 후 해당 카메라 또는 전체 연결을 누릅니다. 다른 휴대폰도 다른 카메라 입력에 같은 방식으로 등록합니다.

`127.0.0.1` 주소가 휴대폰에 표시되면 Wi-Fi IPv4를 찾지 못한 상태입니다. 이 주소는 **USB 도우미**에서
사용하거나 Wi-Fi에 연결한 뒤 송출을 다시 시작하세요. 일반 Wi-Fi 입력으로 PC의 localhost에 직접 연결하면 안 됩니다.
기기 간 통신을 막는 게스트 Wi-Fi/AP 격리에서는 다른 네트워크를 사용해야 합니다. 5GHz Wi-Fi를 권장합니다.

## Android USB — Wi-Fi 없이 연결

이 방식은 휴대폰을 USB 웹캠(UVC)으로 위장하지 않고, 앱의 영상을 USB로 전달합니다.
USB 테더링이나 모바일 데이터 요금제가 필요하지 않습니다.

1. PC에 Google 공식 [Android SDK Platform-Tools](https://developer.android.com/tools/releases/platform-tools)를 설치하고
   `adb` 실행 파일이 있는 폴더를 `PATH`에 추가합니다. 명령창의 `adb version`으로 확인합니다.
   **Windows v0.2.1 EXE 사용자는 adb가 내장되어 이 설치 단계를 생략합니다.**
2. Galaxy의 **설정 → 휴대전화 정보 → 소프트웨어 정보 → 빌드번호**를 여러 번 눌러 개발자 옵션을 열고
   **USB 디버깅**을 켭니다. 처음 연결 시 휴대폰에서 본인 PC의 디버깅 키를 승인합니다.
3. 데이터 USB 케이블로 연결하고 `adb devices`에서 `device` 상태를 확인합니다.
4. 휴대폰의 Drone Camera에서 송출을 시작합니다. Wi-Fi가 꺼져 있어도 됩니다.
5. 데스크톱 **설정 → 유선 USB 연결 도우미 → Android/Galaxy**를 선택하고 휴대폰의 전체 URL을 입력합니다.
6. 한 대면 장치 번호를 비워 둡니다. Android 두 대라면 `adb devices`의 USB 일련번호를 각각 지정합니다.
7. 확인·저장 후 연결합니다. 앱은 본인이 만든 `adb forward tcp:0` 항목만 사용·삭제하며 다른 전달 설정은 지우지 않습니다.

디버깅이 `unauthorized`이면 휴대폰의 승인 대화상자를 확인하세요. 충전 전용 케이블은 사용할 수 없습니다.
Windows에서 기기가 보이지 않으면 제조사의 USB 드라이버도 확인하세요. 무선 adb가 켜져 있어도 기본 선택은 `adb -d`로 USB 장치를 선택합니다.

## iPhone USB — Wi-Fi 없이 연결

1. **Mac**: iPhone을 케이블로 연결하고 잠금 해제·컴퓨터 신뢰를 완료합니다. macOS의 USB 서비스를 사용합니다.
2. **Windows**: Apple 공식 **Apple Devices 또는 iTunes의 Mobile Device 드라이버/서비스**를 준비하고 동일하게 신뢰를 승인합니다.
3. iPhone 앱에서 송출을 시작합니다. 데스크톱 **설정 → 유선 USB 연결 도우미 → iPhone/iPad**에 전체 URL을 입력합니다.
4. iPhone 한 대면 장치 번호를 비워 둡니다. 두 대라면 Finder/Xcode에 표시되는 각 기기의 UDID를 지정합니다.
5. 확인·저장 후 연결합니다. OS의 usbmux 서비스를 통해 TCP 데이터를 전달하므로 Wi-Fi/개인용 핫스팟/모바일 데이터는 필요하지 않습니다.

이 기능은 Windows/macOS를 대상으로 하며 Apple의 USB 서비스가 실제로 실행 중이어야 합니다.
신뢰·개발자 서명·USB 권한은 우회하지 않습니다. Linux iPhone USB 사용 환경은 이 릴리즈의 지원 대상이 아닙니다.

## 동작과 제한

- 앱이 다른 화면으로 이동하거나 잠기면 카메라·네트워크 송출을 중지합니다. 앱으로 돌아와 **송출 시작**을 누르세요.
- 같은 앱 프로세스 안에서는 중지/재시작 시 포트·접근 토큰을 재사용하므로 URL을 복사해 다른 앱에서 전달한 뒤 돌아올 수 있습니다.
  앱이 완전히 종료·재생성되거나 Wi-Fi IP가 바뀌면 새 URL을 데스크톱에 다시 입력하세요.
- 접근 토큰은 128비트 난수입니다. 연결 URL에 포함되므로 URL/개인 설정을 공유하지 마세요.
  이 HTTP 프로토콜은 신뢰하는 LAN 또는 직접 USB 연결용이며 인터넷 공개·전송 암호화 용도가 아닙니다.
- 송출은 최대 두 클라이언트, 프레임은 항상 최신 한 장만 유지합니다. 느린 클라이언트는 프레임을 건너뜁니다.
- 데스크톱은 원래 휴대폰 URL과 USB 방식/장치 번호를 저장하고, 실행마다 임시 localhost 포트를 다시 만듭니다.
  URL의 토큰이 달라졌다면 설정을 갱신해야 합니다. USB 실패 시 Wi-Fi로 몰래 전환하지 않습니다.
- 케이블 재연결·앱 재시작 후 자동 재연결 예산이 소진됐다면 해당 카메라를 해제한 뒤 다시 연결합니다.
- iPhone과 Android의 촬영 시각을 동기화하지 않습니다. 데스크톱 프레임의 시각은 **수신 시각**입니다.
- 장시간 사용·온도·배터리·실제 카메라·케이블·드라이버·Wi-Fi 지연은 실장비 검증이 필요합니다.
  빌드/시뮬레이터/Mock 테스트 성공을 실제 장치 검증으로 표현하지 않습니다.

## 개발·테스트

Android는 JDK 17 이상과 Android SDK 35가 필요합니다.

```bash
cd apps/android-camera
./gradlew :app:testDebugUnitTest :app:assembleDebug :app:lintDebug
# Windows: gradlew.bat 사용
```

iOS는 Mac에서:

```bash
cd apps/ios-camera
brew install xcodegen
xcodegen generate
open DroneCamera.xcodeproj
```

데스크톱 및 프로토콜 통합 검증은 저장소 루트에서 가상환경의 Python을 사용합니다.

```bash
.venv/bin/python -m pytest stages/01-camera-stream/tests -v
.venv/bin/python scripts/smoke_mobile_protocol.py  # JDK의 java/javac가 PATH에 필요
```

통합 테스트는 **APK와 동일한 Java MJPEG 서버**를 실제 JVM에서 실행해 OpenCV 수신 및 로컬 relay를
검증합니다. 카메라 영상은 합성 JPEG, 물리 USB 전송은 로컬 소켓으로 대체합니다. 별도 테스트는
adb 선택/전달/해제와 Apple usbmux plist의 분할 수신·장치 선택·포트 바이트 순서를 검증합니다.
CI에는 기존 Windows/macOS 데스크톱 검사, Android APK·서명·lint·Java 테스트, iOS 시뮬레이터
Swift 테스트·arm64 빌드·서명 전 IPA 및 Xcode 프로젝트 생성이 포함됩니다.
