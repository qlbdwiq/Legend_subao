# Phase 02 — YOLO11 드론 탐지와 ByteTrack 추적 (2D)

두 카메라 영상에서 학습된 YOLO11n 모델로 드론을 찾고, Ultralytics **ByteTrack**으로 카메라별 추적 ID를 부여하며,
바운딩 박스를 감싸는 **바운딩 서클**의 중심·반지름(픽셀)을 계산해 화면에 표시하고 데이터로 제공합니다.

> **이 단계는 2D 픽셀 좌표까지만 다룹니다.** 3차원 좌표, 스테레오 삼각측량, 드론 간 거리, 충돌 위험 경고는
> 포함하지 않으며 후속 단계(Phase 03·04)에서 개발합니다. 실제 드론 비행·실외·두 카메라 동기화·거리 정확도는
> **아직 검증되지 않았습니다.**

## 사용 방법

1. 프로그램을 실행하고 **AI 탐지** 패널의 **모델 불러오기**로 학습된 `.pt` 파일을 선택합니다.
   모델 경로는 코드에 고정되어 있지 않으며 사용자 설정(`ai_settings.json`)에 저장됩니다.
   파일이 이동·삭제되었거나 손상되었으면 패널에 오류가 표시되며 **카메라 수신은 계속 동작**합니다.
2. **AI 탐지 ON**을 켭니다. 끄면 추론을 하지 않으며 AI 스레드도 프레임을 받지 않습니다(추적 상태는 초기화됩니다).
3. 필요하면 신뢰도·NMS IoU·추론 해상도·장치·서클 여유 계수를 조절하고, 박스/서클/ID/신뢰도 표시를 선택합니다.

모델을 바꾸거나 장치를 바꾸면 모델을 다시 불러옵니다(백그라운드). **다시 불러오기**로 같은 경로를 재시도할 수 있습니다.

> **신뢰할 수 있는 `.pt` 파일만 사용하세요.** PyTorch 모델 파일은 pickle 형식이라 불러올 때 파일에 담긴 코드가 실행될 수 있습니다.

| 설정 | 기본값 | 설명 |
| --- | --- | --- |
| 신뢰도 | 0.25 | 이 값 미만의 탐지는 버립니다. ByteTrack은 0.25 이상으로 새 추적을 시작합니다(`bytetrack.yaml` 기본값). |
| NMS IoU | 0.5 | 겹친 박스를 하나로 합치는 기준. 프로젝트 모델은 0.7에서 한 대에 중복 박스를 낸 적이 있어 0.5를 기본으로 합니다. 두 드론이 많이 겹치면 값을 올리세요. |
| 추론 해상도 | 960 px | 학습 해상도입니다. CPU가 느리면 640/480으로 낮추되, 멀리 있는 작은 드론은 놓칠 수 있습니다. |
| 장치 | 자동 | CUDA가 있으면 CUDA, 아니면 CPU. Apple GPU(MPS)는 직접 선택합니다. 초기화에 실패하면 CPU로 전환하고 알립니다. |
| 서클 여유 계수 | 1.2 | `안전 반지름 = 박스 대각선의 절반 × 계수`(1.0–3.0). **실험용 값이며 비행 안전을 보장하지 않습니다.** |

## 구조

```text
stages/01-camera-stream/src/drone_camera/
  geometry.py      바운딩 서클 계산 (순수 Python, GUI·torch 독립)
  detections.py    DroneObservation / FrameDetections 데이터, 추적 상태 분류
  ai_settings.py   AI 설정 검증·저장 (ai_settings.json, 카메라 config.json과 분리)
  engine.py        YOLO11 + ByteTrack 엔진 (torch/ultralytics는 사용할 때 지연 import)
  ai_pipeline.py   백그라운드 추론 스레드, 카메라별 최신 프레임 우편함
  gui.py           AI 패널과 오버레이 (기존 구조 유지)
```

* **스레드 분리:** torch import, 모델 로드, 추론은 모두 하나의 AI 스레드에서 실행됩니다. GUI 스레드와 카메라 수신
  프로세스는 추론을 기다리지 않습니다. GUI는 최신 프레임 참조를 넘기기만 합니다.
* **오래된 프레임이 쌓이지 않음:** 카메라마다 깊이 1의 우편함이 있어 모델이 느리면 중간 프레임을 건너뛰고 항상
  최신 프레임을 처리합니다(`skipped`로 집계). 카메라 두 대는 한 AI 스레드가 번갈아 처리해 CPU/GPU 경쟁을 피합니다.
* **카메라별 독립 상태:** 카메라마다 별도의 YOLO 인스턴스(=별도의 ByteTrack 상태)와 결과를 가집니다.
  Ultralytics 문서의 "여러 스트림은 모델 인스턴스를 분리" 지침을 따랐습니다.
* **재연결·ON/OFF 시 초기화:** 연결이 끊기거나 AI를 껐다 켜면 추적을 초기화하고, 진행 중이던 이전 프레임의 결과는 폐기합니다.
* **AI OFF의 비용:** AI를 한 번도 켜지 않으면 스레드를 만들지 않고 torch도 불러오지 않습니다.
* **네트워크·텔레메트리 차단:** Ultralytics의 온라인 확인, 자동 설치, 익명 사용 통계(`sync`)를 끄고, 설정 파일은
  사용자의 전역 폴더가 아닌 앱 설정 폴더(`ultralytics/`)에 둡니다.

## 추적 ID에 대하여

* **카메라 1의 ID 1과 카메라 2의 ID 1은 같은 드론이라는 뜻이 아닙니다.** 카메라 간 대응은 3단계에서 구현합니다.
* ID는 바뀔 수 있습니다. ByteTrack은 외형 특징 없이 위치·박스 겹침으로만 연결하므로, **두 드론이 겹쳐 지나가면
  ID가 서로 바뀌거나 새 ID가 생길 수 있습니다**(합성 영상에서 실제로 관찰됨). 짧은 미검출(수 프레임)은 같은 ID로 이어집니다.
* 새 객체는 **연속 두 번째 처리 프레임부터** 출력됩니다(ByteTrack의 미확정 트랙 단계).
* 일시적으로 놓친 드론은 **표시하지 않습니다.** 마지막 위치를 관측값처럼 그리지 않으며, 다시 나타나면 `reacquired`로 구분합니다.
* 출력 박스는 ByteTrack의 칼만 필터가 보정한 추정값이고, 신뢰도는 매칭된 탐지의 값입니다. 빠르게 움직이면 박스가
  약간 늦거나 부드러워질 수 있습니다.
* AI는 카메라 FPS보다 느릴 수 있으므로 추적은 "처리된 프레임" 기준으로 동작합니다(프레임 번호 `frame_sequence` 기록).
* 화면의 오버레이는 최신 영상 위에 **가장 최근 AI 결과**를 그립니다. 결과가 1초 이상 오래되면 숨깁니다.
  현재 위치로 오인하지 않도록 화면 아래에 AI FPS·추론 지연·수신→결과 지연을 표시합니다.

## 3단계로 넘길 데이터 구조

`detections.py`의 `FrameDetections`(프레임당 하나)와 `DroneObservation`(드론당 하나)입니다. 모두 단순 값이라
`to_dict()`로 JSON 직렬화할 수 있습니다. `AiPipeline.latest(camera_index)`로 읽습니다.

```text
FrameDetections
  camera_id, frame_sequence            분석한 프레임 (카메라별 번호)
  received_at, received_monotonic      PC가 프레임을 받은 시각 — 촬영(노출) 시각이 아님
  completed_monotonic, inference_ms    추론 완료 시각, 모델 호출 시간
  latency_ms                           수신→결과 지연 (큐 대기 포함)
  image_size                           (width, height) — 좌표의 기준 해상도
  observations: DroneObservation[]
DroneObservation
  camera_id, frame_sequence, received_at, received_monotonic
  track_id        카메라 내부 ID (카메라 간 비교 불가)
  bbox            (x1, y1, x2, y2) 픽셀
  center          (cx, cy) 픽셀
  base_radius     박스 대각선의 절반
  safe_radius     base_radius × safety_factor
  safety_factor   적용된 계수
  confidence      탐지 신뢰도
  state           new / tracked / reacquired
```

**수신 시각은 실제 촬영 시각이 아닙니다.** 3D 삼각측량 전에 두 카메라의 동기화·지연 보정 실험이 별도로 필요합니다.
좌표는 왜곡 보정 전의 원본 영상 픽셀입니다.

## 모델과 라이선스

* 모델: YOLO11n 단일 클래스(`{0: 'drone'}`), 학습 해상도 960. 프로그램은 클래스가 `{0: 'drone'}`이 아니면 경고를 표시합니다.
  모델 평가 결과는 학습 환경에 따라 달라지며, 이 프로그램이나 다른 환경에서 특정 성능을 보장하지 않습니다.
* **`best.pt`와 데이터셋은 저장소에 포함하지 않으며 릴리스에도 첨부하지 않습니다.** 각자 `.pt` 파일을 선택해 사용합니다.
* **Ultralytics(YOLO11, ByteTrack 구현)는 AGPL-3.0**입니다. Ultralytics는 이 라이브러리로 학습한 모델도 기본적으로
  AGPL-3.0 대상이라고 안내합니다. 이 저장소의 자체 코드는 MIT이지만, **Ultralytics를 포함한 배포 파일(EXE·DMG)과 소스를
  배포할 때는 AGPL-3.0의 조건(소스 공개 등)이 적용될 수 있습니다.** 릴리스에는 소스 ZIP을 함께 제공합니다.
  교내·연구·비영리 사용을 넘어 상용·비공개 제품으로 배포한다면 Ultralytics Enterprise License 등을 검토하세요.
  (이 문서는 법률 자문이 아닙니다. 상세: <https://www.ultralytics.com/license>)

## 플랫폼과 성능

| 환경 | 지원 |
| --- | --- |
| Windows x64 (소스) | CPU. NVIDIA CUDA는 [pytorch.org](https://pytorch.org/get-started/locally/)의 CUDA 빌드 torch를 설치하면 사용 가능(미검증). |
| Windows x64 (EXE) | **CPU 전용.** PyPI의 Windows torch는 CPU 빌드이며 CUDA 빌드는 용량(수 GB)이 커서 EXE에 넣지 않았습니다. |
| Apple Silicon Mac | CPU, 또는 MPS 직접 선택. |
| Intel Mac | CPU. PyTorch 2.3부터 Intel Mac 휠이 없어 **torch 2.2.2 + NumPy 1.26**을 사용합니다. |

추론 속도와 지연은 CPU/GPU 종류, 해상도, 입력 스트림 수에 따라 달라집니다. 작은 모델에서도
장치 간 전송·초기화 오버헤드가 발생할 수 있으므로, 느린 환경에서는 추론 해상도를 낮추세요.

## 테스트

```bash
# 일반 테스트 (CI에서 실행): Mock 엔진 + 실제 torch/ultralytics 런타임(무작위 가중치) 검증
python -m pytest stages/01-camera-stream/tests -v

# 실제 학습 모델 테스트 (로컬에서만): best.pt와 라벨된 샘플 이미지 폴더(*.jpg + 같은 이름의 YOLO *.txt) 필요
DRONE_YOLO_MODEL=/path/to/best.pt DRONE_YOLO_SAMPLES=/path/to/samples python -m pytest stages/01-camera-stream/tests -v -rs
```

테스트는 세 종류로 구분됩니다.

* **Mock 기반**(`fakes.py`의 가짜 엔진): 스레드·최신 프레임·카메라 분리·리셋·오류·GUI 동작 검증. YOLO/ByteTrack 품질과 무관합니다.
* **`ultralytics_runtime`**: 실제 torch/ultralytics/ByteTrack 코드를 무작위 가중치 모델로 실행. 라이브러리 동작·번들 파일 검증용이며 탐지 정확도와 무관합니다.
* **`real_model`**: 실제 `best.pt`로 실제 이미지·합성 영상 추론. `DRONE_YOLO_MODEL`이 없으면 skip됩니다(CI에서는 skip).
  합성 영상은 학습 이미지의 드론 크롭을 배경에 붙여 만들었으므로 **동작 확인용이며 실제 비행 성능 평가가 아닙니다.**

## 알려진 한계

* 합성 영상 기준으로 두 드론이 겹쳐 지나가면 ID가 바뀔 수 있습니다. 작은 드론·프로펠러·역광은 놓치거나 박스가 흔들릴 수 있습니다.
* 서클 여유 계수 1.2는 임의의 실험값입니다.
* 모델은 학습 환경과 다른 실외 영상·실제 카메라 화질에서 검증되지 않았습니다.
