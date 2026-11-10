# Drone Camera — Android / Galaxy

Camera2 후면 카메라 → YUV stride 보정 → JPEG → 인증된 HTTP MJPEG 송출 앱입니다.
Android 8.0(API 26)+, 최대 1280×720 / 30fps (15fps 선택 가능), 최대 2클라이언트, 최신 프레임 한 장을 유지합니다.
UI 미리보기는 별도 최신 프레임을 약 10Hz로 표시합니다. 성능은 실제 장치에서 확인해야 합니다.

Wi-Fi 또는 데스크톱의 Android USB 도우미로 연결합니다. 설치·연결·권한 안내는
[모바일 설명서](../../docs/mobile/README.md)에 있습니다.

JDK 17+, Android SDK 35에서:

```bash
./gradlew :app:testDebugUnitTest :app:assembleDebug :app:lintDebug
```

Windows는 `gradlew.bat`를 사용합니다. SDK 경로는 `ANDROID_HOME` 또는 Git에 올리지 않는
`local.properties`의 `sdk.dir`로 지정합니다. APK는 `app/build/outputs/apk/debug/app-debug.apk`입니다.
디버그 서명이며 스토어 배포 서명이 아닙니다. 서명 키/실제 연결 주소는 저장소에 넣지 않습니다.

테스트는 최근 프레임 관리, 실제 HTTP 인증·multipart 응답, 패딩·pixel stride가 있는 YUV 변환을
검증합니다. `MjpegFixture.java`는 데스크톱 통합 검사용이며 APK에 포함하지 않습니다.
