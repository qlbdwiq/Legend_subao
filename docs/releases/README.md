# 릴리즈 분류

`v0.1.0`–`v0.4.0`은 **Phase 01 — Camera Stream**, `v0.5.0`부터는 **Phase 02 — YOLO Detection & Tracking(2D)** 범위입니다.
스테레오 보정·3D 추적은 Phase 03, 충돌 위험 경고는 Phase 04에서 다룹니다.

| 릴리즈 | 분류 | 주요 추가 내용 |
| --- | --- | --- |
| [v0.1.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.1.0) | Phase 01 | 두 카메라 수신·표시의 최초 구현 |
| [v0.2.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.2.0) | Phase 01 | Android/iPhone 송출 앱, Wi-Fi·USB 연결 |
| [v0.2.1](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.2.1) | Phase 01 | Windows 독립 실행 EXE |
| [v0.3.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.3.0) | Phase 01 | Apple Silicon·Intel macOS DMG |
| [v0.4.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.4.0) | Phase 01 | 카메라 해상도·FPS 선택 확장 |
| [phase-02-v0.5.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/phase-02-v0.5.0) | Phase 02 | YOLO11 탐지·ByteTrack 추적·바운딩 서클 (2D). macOS arm64 DMG·Windows EXE 제공, Intel Mac 미제공 |

기존 `vX.Y.Z` 태그와 릴리즈는 보존됩니다. 새 릴리즈는
`phase-NN-vX.Y.Z` 태그를 사용하면 제목에 `Phase NN`이 자동으로 붙습니다.
예를 들어 1차 구현의 후속 버전은 `phase-01-v0.5.0`으로 게시합니다.
