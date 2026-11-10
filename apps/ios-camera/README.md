# Drone Camera — iPhone / iPad

SwiftUI, AVFoundation 일반 후면 카메라, CoreImage JPEG, Network.framework TCP 서버를 사용합니다.
iOS 16+, 최대 1280×720 / 30fps (15fps 선택 가능) 목표, 최신 프레임 하나와 클라이언트별 하나의 미완료 전송만 유지합니다.
Wi-Fi 또는 데스크톱의 Apple USB 도우미를 사용합니다. LiDAR/마이크/AI 기능은 없습니다.

**무료 Apple 계정 사용자는 Mac의 Xcode Personal Team으로 직접 서명·설치합니다.**
CI의 `unsigned.ipa`는 실제 arm64 빌드이지만 그대로 설치할 수 없습니다.
자세한 설치·USB·Wi-Fi 절차는 [모바일 설명서](https://github.com/qlbdwiq/Legend_subao/blob/main/docs/mobile/README.md)를 참고하세요.

```bash
brew install xcodegen
xcodegen generate
open DroneCamera.xcodeproj
```

생성한 프로젝트의 DroneCamera target에서 Personal Team, 고유 Bundle Identifier를 선택하고
신뢰한 실제 iPhone을 대상으로 Run 합니다. 생성 프로젝트·개인 Xcode 설정·서명 자료는 Git에 저장하지 않습니다.
릴리즈의 `xcode-project.zip`은 CI에서 생성한 프로젝트를 포함하므로 바로 열 수 있습니다.

CI는 사용 가능한 iPhone 시뮬레이터에서 Swift 프로토콜 테스트를 실행하고, 별도로 generic iOS
device arm64 Release 빌드를 합니다. 시뮬레이터의 가상 카메라를 실제 후면 카메라 검증으로 간주하지 않습니다.
