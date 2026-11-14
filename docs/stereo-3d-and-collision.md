# 3단계~5단계: 실시간 3차원 좌표 계산, 충돌 위험 분석 및 관제 대시보드

이 문서는 **3단계(스테레오 카메라 보정·3D 삼각측량)**, **4단계(3D 충돌 위험 분석·경고)**, **5단계(통합 관제 대시보드·실험 기록·재생)**의
구조, 알고리즘, 사용 방법, 그리고 **실제 드론 2대 현장 실험 절차 및 안전·성능 측정 계획**을 설명합니다.

> **중요 안내 (실험용 보조 시스템 / 실제 비행 미검증)**
> - 본 시스템은 연구·실험용 관제 보조 도구이며 **실제 비행 안전을 보장하지 않습니다.**
> - 드론 자동 제어·강제 회피 명령 전송 기능은 포함하지 않습니다.
> - 소프트웨어 알고리즘은 단위 테스트·합성 스테레오 데이터·자동화 CI로 검증되었으나, **실제 드론 2대 비행 및 실외 현장 실험은 아직 수행되지 않았습니다.**

---

## 1. 좌표계 및 모듈 구조

| 모듈 | 단계 | 주요 역할 |
| --- | --- | --- |
| `src/drone_camera/calibration.py` | Phase 03 | 체커보드 코너 검출(`cv2.findChessboardCornersSB`), 단일·스테레오 카메라 보정(`cv2.calibrateCamera`, `cv2.stereoCalibrate`), 렌즈 왜곡 보정(`cv2.undistort`, `cv2.undistortPoints`), 기준선(`baseline_m`) 스케일링, `stereo_calibration.json` 저장·불러오기, 합성 보정 프리셋 생성 |
| `src/drone_camera/stereo_3d.py` | Phase 03 | 프레임 시간 차이 측정·오프셋 보정(`evaluate_frame_sync`, `estimate_time_offset_s`), 에피폴라 기하(`F` 행렬 Sampson 거리) 기반 카메라 간 드론 대응(`match_observations_across_cameras`), 모호성 기각(`ambiguous_match`), 스테레오 삼각측량(`triangulate_point`), 재투영 오차·신뢰도·위치 불확실성(`position_uncertainty_m`) 계산 |
| `src/drone_camera/collision.py` | Phase 04 | 3D 위치 기반 속도·이동 방향 추정(`DroneMotionFilter`), 3D 거리·상대 접근 속도·최근접 거리(`d_cpa_m`)·최근접 시점(`t_cpa_s`) 계산, 불확실성 반영 유효 거리(`effective_distance_m`), 위험 단계(`SAFE` / `CAUTION` / `DANGER` / `UNKNOWN`) 판정, 시각·청각 경고 쿨다운, 위험 이벤트(`RiskEvent`) 기록 |
| `src/drone_camera/recorder.py` | Phase 05 | 실험 세션 JSONL 텔레메트리 기록(`ExperimentRecorder`), 나란히 합성한 스테레오 영상(`.avi`) 녹화, 프레임별 CSV 및 위험 이벤트 CSV 내보내기, 세션 통계 요약(`summarize_session`) |
| `src/drone_camera/dashboard_widgets.py` | Phase 05 | 3D 공간 시각화 위젯(`DroneSpace3DWidget`: Top X-Z / Front X-Y / 3D Iso), 스테레오 보정 마법사(`CalibrationWizardDialog`), 기록 데이터 재생·분석 창(`PlaybackDialog`) |
| `src/drone_camera/gui.py` | Phase 05 | PySide6 메인 관제 대시보드(`MainWindow`) 3구역 레이아웃 통합 |

### 3차원 기준 좌표계 (Camera 1 광학 중심 기준, 단위: 미터 `m`)
- **원점 `(0, 0, 0)`**: 카메라 1의 광학 중심(Optical Center)
- **`+X` 축**: 카메라 1 화면의 오른쪽 방향 (m)
- **`+Y` 축**: 카메라 1 화면의 아래쪽 방향 (m)
- **`+Z` 축**: 카메라 1 정면 깊이 방향 (m, `Z > 0`만 유효)

---

## 2. 3단계 — 실시간 3차원 좌표 계산 핵심 규칙

1. **미보정 시 임의 좌표 출력 금지**
   - `StereoCalibration` 데이터가 없으면(`calibration is None`) `StereoDroneTracker`는 `status="uncalibrated"`를 반환하며, 어떤 경우에도 가짜 3D 좌표를 생성하지 않습니다.
2. **카메라 간 독립 ByteTrack ID 대응**
   - 카메라 1의 ByteTrack ID와 카메라 2의 ByteTrack ID가 같다고 가정하지 않습니다.
   - 왜곡 보정된 정규화 좌표에서 기본 행렬(Fundamental Matrix $F$)의 에피폴라 선 거리(Sampson distance)와 삼각측량 재투영 오차, 체럴리티($Z_1 > 0, Z_2 > 0$)를 모두 만족하는 쌍을 전역 최소 비용으로 매칭합니다.
3. **다중 드론 대응 모호성 기각 (`ambiguous_match`)**
   - 두 드론이 동일한 에피폴라 평면 근처에 위치해 교차 매칭 비용 차이가 `ambiguity_margin_px` 이내이거나 한 관측치의 차선 후보가 너무 가까우면 좌표를 확정하지 않고 `status="ambiguous_match"`로 기각합니다.
4. **프레임 시간 동기화 및 속도 보간**
   - 두 카메라 프레임의 수신 시각 차이 $\Delta t = (t_1 - t_2) - \text{time\_offset\_s}$를 매 틱 계산합니다.
   - $|\Delta t| > \text{max\_sync\_delta\_s}$(기본 `0.10 s`)이면 `status="unsynchronized"`로 처리합니다.
   - 허용 범위 내의 미세 시간 차이는 카메라 2의 2D 픽셀 속도를 이용해 카메라 1 시각으로 선형 보간한 뒤 삼각측량합니다.

---

## 3. 4단계 — 충돌 위험 분석 및 판단 불가(`UNKNOWN`) 처리

- **판단 불가(`RISK_UNKNOWN`) 우선 원칙**
  - 카메라 미보정, 프레임 동기화 불확실(`unsynchronized`), 대응 모호(`ambiguous_match`), 삼각측량 실패, 혹은 추적된 드론 수가 2대 미만일 때는 **절대 `'안전(SAFE)'`으로 표시하지 않고 `'판단 불가 / UNKNOWN'`** 상태로 표시합니다.
- **2D 바운딩 서클과 3D 안전거리의 분리**
  - 2D 바운딩 서클(픽셀 반지름)은 화면 내 드론 위치·크기 시각화 전용이며, 실제 드론 간 충돌 위험 판단은 오직 미터(`m`) 단위의 3D 좌표 거리와 접근 속도로만 수행합니다.
- **위치 불확실성 반영**
  - 각 드론의 재투영 오차와 거리($Z^2 / (f \cdot B)$)에 비례하는 3D 위치 불확실성 $\sigma_1, \sigma_2$를 합산하여 보수적인 유효 거리 $d_{\text{eff}} = \max(0, d_{3D} - (\sigma_1 + \sigma_2))$를 함께 평가합니다.
- **거리 및 CPA(최근접 시점·거리) 기반 단계 판정**
  - **위험 (`DANGER`)**: $d_{\text{eff}} \le \text{danger\_distance\_m}$ (기본 `2.5 m`), 또는 접근 속도 $v_{\text{close}} \ge \text{min\_closing\_speed\_mps}$이면서 예측 시간(`prediction_horizon_s`, 기본 `3.0 s`) 내 예상 최근접 거리 $d_{\text{CPA,eff}} \le \text{danger\_distance\_m}$인 경우.
  - **주의 (`CAUTION`)**: $d_{\text{eff}} \le \text{warning\_distance\_m}$ (기본 `5.0 m`), 또는 예측 시간 내 $d_{\text{CPA,eff}} \le \text{warning\_distance\_m}$인 경우.
  - **안전 (`SAFE`)**: 두 드론의 3D 좌표가 모두 신뢰 가능하고 위 조건에 해당하지 않는 경우.

---

## 4. 5단계 — 관제 대시보드, 기록 및 재생

메인 창(`MainWindow`)은 화면 복잡도를 낮추기 위해 3개 구역으로 정리되어 있습니다.

1. **좌측 구역 (카메라 1·2 실시간 영상)**
   - 실시간 영상, YOLO11 바운딩 박스, ByteTrack ID, 2D 바운딩 서클 오버레이
2. **중앙 구역 (3D 공간 시각화 및 드론 텔레메트리)**
   - 상단 충돌 위험 배너 (`안전 / 주의 / 위험 / 판단 불가`)
   - 보정 상태 · 동기화 $\Delta t$ · 두 드론 3D 거리 · 상대 접근 속도 · CPA 요약 카드
   - **3D 공간 뷰 (`DroneSpace3DWidget`)**: `위에서 보기 (Top X-Z)`, `정면 보기 (Front X-Y)`, `3D 입체 (Iso)` 전환, 카메라 1·2 위치, 드론 위치·불확실성 반경·속도 벡터·드론 간 거리선 실시간 렌더링
   - 드론별 3D 위치 $(X, Y, Z)\text{ [m]}$, 속도 $(V_x, V_y, V_z)\text{ [m/s]}$, 이동 방향, 재투영 오차, 신뢰도 표시
3. **우측 구역 (2D AI 설정 · 3D 보정 · 위험 임계값 · 실험 기록 및 재생)**
   - **보정 마법사 (`CalibrationWizardDialog`)**: 실시간 체커보드 캡처·스테레오 보정, 기준선 길이(`baseline_m`) 및 시간 오프셋 설정, 합성 보정 프리셋 적용, `.json` 저장·불러오기
   - **충돌 임계값 설정**: 주의 거리, 위험 거리, CPA 예측 시간, 청각 경고(비프음) ON/OFF (`collision_settings.json` 자동 저장)
   - **실험 데이터 기록·내보내기·재생**:
     - `실험 기록 시작` 클릭 시 `recordings/session-YYYYMMDD-HHMMSS/` 아래에 `telemetry.jsonl`, `stereo_video.avi`, `risk_events.csv`, `telemetry.csv` 생성
     - `CSV 내보내기`, `위험 이벤트 CSV`, `기록 재생 / 분석`(`PlaybackDialog` 타임라인 슬라이더 및 세션 통계) 제공

---

## 5. 실제 드론 2대 & 카메라 2대 현장 실험 절차 (사전 준비 가이드)

소프트웨어 개발 완료 후 실제 장비를 이용한 현장 실험은 아래 순서로 진행합니다.

### 5.1 사전 안전 체크리스트
- [ ] 비행 구역 내 보행자·장애물 통제 및비상 착륙 구역 확보 (이중 조종사 또는 즉시 호버링 전환 가능 상태 유지)
- [ ] 본 시스템은 **경고 보조용**이며 드론을 자동 제어하지 않음을 조종사 전원 숙지
- [ ] 두 드론(drone 등) 배터리 잔량 80% 이상, 프로펠러 및 GPS/비전 센서 상태 점검
- [ ] 카메라 2대 삼각대 고정 상태 확인 (**보정 후 카메라 위치·각도가 흔들리면 3D 좌표가 무효화됨**)
- [ ] 카메라 1·2의 해상도·FPS 설정 일치 여부 및 스트림 수신 지연 확인

### 5.2 현장 스테레오 카메라 보정 절차
1. **카메라 배치**: 비행 구역을 바라보도록 두 카메라를 `1.5 m ~ 4.0 m` 간격(기준선 $B$)으로 삼각대에 단단히 고정합니다.
2. **기준선 실측**: 줄자로 카메라 1 렌즈 중심과 카메라 2 렌즈 중심 사이의 실제 거리(m)를 측정합니다.
3. **체커보드 캡처**:
   - 대시보드 우측 **보정 마법사**를 열고 체커보드 내부 코너 수(예: `7 × 5`)와 정사각형 한 칸 크기(예: `50 mm`)를 입력합니다.
   - 두 카메라 화면에 체커보드가 동시에 보이도록 들고, 중앙·좌·우·상·하 및 기울기를 바꾸며 **최소 10~15장**의 코너 쌍을 캡처합니다.
4. **보정 계산 및 검증**:
   - 실측한 기준선 길이(m)를 입력하고 **스테레오 보정 실행 (실제 보정)**을 클릭합니다.
   - 계산된 스테레오 RMS 재투영 오차가 **`1.0 px` 이하**(권장 `0.5 px` 이하)인지 확인합니다.
   - 보정 상태가 **`실제 보정 완료`**로 표시되는지 확인합니다 (`합성/테스트 보정` 상태로는 실제 비행 실험을 진행하지 마세요).

### 5.3 현장 실험에서 측정해야 할 성능 지표
1. **정적 3D 위치 정확도 (Static Position Accuracy)**
   - 줄자·마커로 실제 좌표 $(X_{\text{true}}, Y_{\text{true}}, Z_{\text{true}})$를 아는 지점(예: 거리 `3 m`, `5 m`, `8 m`, `10 m`)에 드론 또는 타깃을 두고 추정 좌표와의 오차 $\| \hat{P} - P_{\text{true}} \|_2$ 및 축별 오차($\Delta X, \Delta Y, \Delta Z$) 측정
2. **3D 드론 간 거리 정확도 (Inter-Drone Distance Error)**
   - 두 드론을 알려진 간격(예: `2.0 m`, `4.0 m`, `6.0 m`)으로 호버링시킨 뒤 추정 거리와 실제 거리의 차이 측정
3. **동기화 및 종단간 지연 (End-to-End Latency)**
   - 카메라 수신 $\rightarrow$ YOLO11 추론 $\rightarrow$ 3D 삼각측량 $\rightarrow$ 위험 경고 표시까지의 총 지연 시간(ms) 및 프레임 시간 차이($\Delta t$) 분포 측정
4. **충돌 위험 경고 성능 (Alert Precision / Recall / Lead Time)**
   - 안전 거리 유지 비행(오경고율 False Alarm Rate 확인)과 저속 접근 시나리오(주의·위험 경고가 목표 거리/CPA 시점에 정확히 발생하는지 확인)를 `실험 기록 시작`으로 녹화 후 `telemetry.csv` 및 `risk_events.csv`로 정량 분석
