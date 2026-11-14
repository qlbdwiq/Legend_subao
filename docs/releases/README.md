# 릴리즈 분류

`v0.1.0`–`v0.4.0`은 **Phase 01 — Camera Stream**, `v0.5.0`은 **Phase 02 — YOLO Detection & Tracking(2D)**,
`v0.6.0`–`v0.6.1`(`phase-05-v0.6.0`, `phase-05-v0.6.1`)은 **Phase 03~05 — 스테레오 3D 좌표 계산·충돌 위험 분석·통합 관제 대시보드** 범위입니다.

| 릴리즈 | 분류 | 주요 추가 내용 |
| --- | --- | --- |
| [v0.1.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.1.0) | Phase 01 | 두 카메라 수신·표시의 최초 구현 |
| [v0.2.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.2.0) | Phase 01 | Android/iPhone 송출 앱, Wi-Fi·USB 연결 |
| [v0.2.1](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.2.1) | Phase 01 | Windows 독립 실행 EXE |
| [v0.3.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.3.0) | Phase 01 | Apple Silicon·Intel macOS DMG |
| [v0.4.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/v0.4.0) | Phase 01 | 카메라 해상도·FPS 선택 확장 |
| [phase-02-v0.5.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/phase-02-v0.5.0) | Phase 02 | YOLO11 탐지·ByteTrack 추적·바운딩 서클 (2D). macOS arm64/Intel DMG·Windows EXE 제공 |
| [phase-05-v0.6.0](https://github.com/qlbdwiq/Legend_subao/releases/tag/phase-05-v0.6.0) | Phase 03~05 | 스테레오 보정·3D 삼각측량(Phase 03), 3D 충돌 위험 분석·경고(Phase 04), 통합 관제 대시보드·기록·재생(Phase 05). 실험용 / 실제 비행 미검증 |
| [phase-05-v0.6.1](https://github.com/qlbdwiq/Legend_subao/releases/tag/phase-05-v0.6.1) | Phase 03~05 | 카메라 지원 범위 비동기 조회·설정 자동 복구 개선, 체커보드 없는 자동 보정·상하 카메라 배치, 모바일 송출 앱(APK/IPA/Xcode) 통합 첨부. 실험용 / 실제 비행 미검증 |

기존 `vX.Y.Z` 태그와 릴리즈는 보존됩니다. 새 릴리즈는
`phase-NN-vX.Y.Z` 태그를 사용하면 제목에 `Phase NN`이 자동으로 붙습니다.
예를 들어 1차 구현의 후속 버전은 `phase-01-v0.5.0`으로 게시합니다.
