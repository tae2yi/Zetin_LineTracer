# Line Tracer V25 - Second Drive 개발 인수인계

## 1. 문서 목적과 현재 기준점

이 문서는 현재 프로젝트의 First Drive V25 상태를 다른 개발자 또는 AI가 빠르게 이해하고, First Drive에서 RAM에 수집한 트랙 정보를 이용하는 Second Drive를 이어서 개발하기 위한 인수인계 문서다.

- 프로젝트 경로: `/Users/ehoi/STM32CubeIDE/line_tracer`
- 현재 펌웨어 버전: `V25`
- 버전 정의: `Main/Inc/app_version.h`
- 현재 Debug 빌드: 성공
- 현재 메모리 사용량:
  - RAM: 38,344 B / 272 KB (13.77%)
  - Flash: 103,792 B / 512 KB (19.80%)
- 빌드 명령:

```sh
cmake --build --preset Debug -j 8
```

현재 V25의 크로스/엔드마커 분리 수정은 빌드 검증까지 끝났으며, 실제 트랙 재시험은 다음 작업자가 확인해야 한다.

## 2. 반드시 먼저 읽을 파일

우선순위 순서:

1. `Main/Src/drive.c`, `Main/Inc/drive.h`
2. `Main/Src/track.c`, `Main/Inc/track.h`
3. `Main/Src/sensor.c`, `Main/Inc/sensor.h`
4. `Main/Src/motor.c`, `Main/Inc/motor.h`
5. `Main/Src/marker.c`, `Main/Inc/marker.h`
6. `Main/Src/menu.c`
7. `Core/Src/tim.c`, `Core/Src/stm32h5xx_it.c`

추가 자료:

- 대회 규정: `/Users/ehoi/Downloads/20회 단국대학교 전국 지능형 로봇대회 규정 - 라인트레이서.pdf`
- 학습/발표 자료: `/Users/ehoi/Downloads/2026년 9주차 발표자료.pdf`
- 회로도 상면: `/Users/ehoi/Downloads/Schematic-STEP-TOP.pdf`
- 회로도 하면: `/Users/ehoi/Downloads/Schematic-STEP-BOTTOM.pdf`

규정이나 마커 의미가 불확실할 때 추측으로 바꾸지 말고 위 자료를 먼저 확인한다.

## 3. 하드웨어와 센서 배치에서 가장 중요한 사실

### 3.1 센서 역할

센서는 물리적으로 왼쪽부터 `S0 ... S7`이다.

- `S0`: 물리적 최좌측, 왼쪽 마커 전용
- `S1 ... S6`: 주행 라인 위치와 자세 추종 전용
- `S7`: 물리적 최우측, 오른쪽 마커 전용

위치 부호:

- S1 쪽: `-2500`, 물리적 왼쪽
- 중앙: `0`
- S6 쪽: `+2500`, 물리적 오른쪽
- 센서 간 위치 간격: 1000

`S0/S7`은 절대로 정상 라인 위치 가중평균에 넣지 않는다. 과거 S0/S7을 위치 계산에 포함했을 때 마커를 주행선으로 선택하거나 곡선에서 잘못된 위치를 만드는 문제가 반복되었다.

### 3.2 실제 센서가 없는 구간

중앙 S1-S6 센서 보드와 외곽 S0/S7 사이의 다리 부분에는 센서가 없다. 따라서 라인이 다음과 같이 이동하면 짧은 블라인드 구간이 생긴다.

```text
S2 -> S1 -> 센서 없는 다리 -> S0
S5 -> S6 -> 센서 없는 다리 -> S7
```

이 블라인드 구간에서 해야 하는 일:

- 마지막 유효 위치와 조향 방향을 유지한다.
- 바깥쪽 S0/S7이 같은 방향에서 나타나면 라인이 계속 외곽으로 진행한다는 증거로만 쓴다.
- S0/S7로 새로운 연속 위치를 계산하지 않는다.
- S1-S6가 라인을 다시 잡으면 즉시 정상 위치 추종으로 복귀한다.

현재 구현은 `FirstDrive_UpdateBridgeRecovery()`에서 이 동작을 수행한다.

### 3.3 적외선 구동과 ADC

- U9(ULN2803)는 이미터를 대칭 쌍으로 구동한다.
  - IR0: S0/S7
  - IR1: S1/S6
  - IR2: S2/S5
  - IR3: S3/S4
- ADC는 4배 오버샘플링되어 최대값은 16380이다.
- 최근 3개 프레임의 미디언 필터가 적용된다.
- 센서 프레임 링버퍼는 64개다.

센서 보드의 가변저항을 너무 조였을 때 신호가 매우 약했고, 이를 풀어 아날로그 이득을 확보한 뒤 캘리브레이션과 모터 주행이 안정화되었다. 디지털에서 무조건 큰 배율을 곱하는 방식으로 되돌리지 않는다.

## 4. 인터럽트와 제어 주기

- TIM7 Global Interrupt는 현재 활성화되어 있다.
- `.ioc`: `NVIC.TIM7_IRQn=true...`
- NVIC 우선순위: 2, 0
- TIM7 주기: 2 kHz
- `FIRST_DRIVE_CONTROL_DIVIDER = 2`
- 실제 주행 제어: 1 kHz
- 흐름:

```text
TIM7_IRQHandler
  -> HAL_TIM_IRQHandler
  -> HAL_TIM_PeriodElapsedCallback
  -> HAL_TIM7_IRQ_Handler
  -> FirstDrive_ControlTick (2회 중 1회)
```

USB 전원만 연결한 검사에서 IRQ:Control 카운터가 약 2:1로 확인되었다. TIM7 IRQ가 다시 꺼지면 화면과 메뉴는 동작해도 실제 라인 제어가 갱신되지 않을 수 있다.

## 5. 캘리브레이션과 정규화

현재 메뉴의 캘리브레이션 과정:

1. 센서 워밍업 0.5초
2. 검은 바닥 수집 3초
3. 흰선 준비 1초
4. 흰선 스윕 최소 8초
5. 8개 센서가 모두 품질 기준을 통과하면 8초 이후 자동 종료
6. 미충족 센서가 있으면 최대 12초까지 계속 수집

검은 바닥 처리:

- 최소 512 프레임 필요
- 히스토그램 기반 P95를 검은 기준점으로 사용
- P50/P99 차이로 검은 바닥 노이즈를 추정
- 순간 그림자나 ADC 스파이크를 절대 최대값으로 저장하지 않는다.

흰선 처리:

- 검은 기준보다 충분히 높은 값만 후보로 사용
- 센서별 상위 32개 샘플 유지
- 최상위 이상치 4개를 제외한 평균을 흰 기준으로 사용

유효 조건:

- 센서별 raw 범위가 최소 800
- 신호 범위가 추정 노이즈의 6배 이상
- 8개 센서 모두 통과해야 calibration complete

정규화:

```text
raw <= black_baseline -> 0
raw >= white_reference -> 1000
그 사이 -> (raw - black) / (white - black) * 1000
```

- 주행 흰색 ON: 500
- 주행 흰색 OFF: 300
- 마커 강한 흰색 기준: 600
- 히스테리시스를 삭제하지 않는다.

## 6. First Drive V25 주행 알고리즘

### 6.1 시작과 기본 설정

- 버튼 입력 후 2초 카운트다운
- 시작 시 유효한 S1-S6 라인이 없으면 출발하지 않음
- 기본 P 게인: 450 (`Q10` 스케일)
- 기본 D 게인: 0
- 기본 직선 속도: 3820 SPS, 약 1.5 m/s
- 최대 설정 속도: 5200 SPS
- 초기 모터 속도: 400 SPS
- 가속: 제어 1 ms당 8 SPS, 약 3.1 m/s²
- 감속: 제어 1 ms당 10 SPS, 약 3.9 m/s²
- 모터 전류 DAC: 1536, 약 0.91 A/phase

DAC 512는 약 0.30 A/phase라 실제 차체에서 토크 부족과 탈조 위험이 있었다. 주행 전류를 임의로 다시 낮추지 않는다.

### 6.2 위치와 군집 선택

`Sensor_GetLineMeasurement()`은 S1-S6의 연속된 활성 센서를 최대 3개 군집으로 분리한다. First Drive는 다음 기준으로 군집을 다시 고른다.

- 직전 위치와 가장 가까운 군집
- 정상 추종 중 위치 점프가 1400을 넘는 군집은 거부
- 같은 거리면 강도가 큰 군집 선택
- 유효 군집이 없으면 마지막 위치를 보존하고 line invalid 처리

S0/S7은 이 군집에 들어가지 않는다.

마커가 S1/S2 또는 S5/S6까지 덮는 경우가 있어서 `FirstDrive_FilterMarkerSpill()`이 중앙 라인이 따로 보이고 직전 위치가 중앙일 때 마커 쪽 인접 센서를 임시 제외한다. 이 로직을 제거하면 방향 마커를 주행선으로 따라갈 수 있다.

### 6.3 PD와 조향 연속성

- 위치 데드밴드: ±100
- 미분값 제한: ±2000
- D 저역통과 필터: shift 2
- 정상 최대 조향비: 0.45
- 외곽 복구 최대 조향비: 0.60
- 조향비 slew 제한으로 갑작스러운 좌우 반전을 방지
- 외곽 위치에서는 P 항을 1.25배 강화

라인이 사라진 동안에는 새로운 위치나 미분값을 만들지 않는다. 마지막 조향비를 유지하고, 외곽 센서 증거가 있으면 해당 방향으로 0.60 복구 조향을 준다. 과거 라인 로스 시 D를 강제로 0으로 만들면서 전체 회전력이 사라지는 문제가 있었으므로 같은 형태로 회귀하지 않도록 한다.

### 6.4 속도 프로파일

- 중앙에서는 base speed 유지
- |position|이 커질수록 1800 SPS 방향으로 선형 감속
- 마커 접근 시 최대 2400 SPS
- TURN 상태 최대 2200 SPS
- 라인 복구 중 직선 최대 1400 SPS
- 라인 복구 중 회전 최대 1800 SPS

Second Drive는 First Drive보다 직선 및 전반 속도를 높이는 것이 목표지만, 마커 진입 전에 충분한 감속 여유를 확보해야 한다.

### 6.5 코스 상태 머신

코스 상태:

```text
STRAIGHT
APPROACH_LEFT / APPROACH_RIGHT
TURN_LEFT / TURN_RIGHT
EXIT_LEFT / EXIT_RIGHT
CROSS
```

방향 부호:

- 왼쪽: -1
- 오른쪽: +1

방향 마커 처리:

1. 중앙 라인이 유효한 상태에서 한쪽 S0 또는 S7을 3프레임 확인
2. provisional marker로 조기 감속 및 APPROACH 진입
3. 완성된 Track 이벤트로 확인
4. 같은 방향 마커가 TURN 중 다시 나오면 EXIT
5. EXIT 이후 같은 방향 마커 또는 중앙 안정 조건으로 STRAIGHT

긴 225도/270도 일정 곡률 구간에서는 라인이 중앙으로 돌아왔다는 이유만으로 TURN을 끝내면 안 된다. TURN 종료는 대응 마커를 중심으로 판단한다.

### 6.6 크로스 라인

V24에서는 크로스가 S0/S7과 중앙 센서를 동시에 켜 `BOTH` 엔드마커로 분류되어 정지하는 문제가 있었다.

V25 수정:

- 중앙 센서 4개 이상이 3프레임 이상 지속된 광폭 패턴을 먼저 `CROSS`로 분류
- 양측 외곽 감지보다 중앙 광폭 증거가 우선
- CROSS 상태에서는 약 510 step 동안 직진 통과
- CROSS 중 중앙이 넓게 보이면 직전 위치를 유지하고 D=0으로 두어 가로 가지를 따라가지 않음

V25의 이 변경은 빌드 완료 상태이며 실제 크로스 재시험이 필요하다. 로그에서 크로스는 `CR`, 엔드는 `BT`로 나와야 한다.

### 6.7 엔드마커

규정상 출발점과 도착점은 주행선 양쪽에 표시선이 있다.

정지 조건:

- 시작 마커 무시 구간 300 step 이후
- S0/S7 동시 overlap이 2프레임 이상
- 중앙 최대 활성 센서 수가 4 미만
- 중앙 광폭 지속이 3프레임 미만

즉, 중앙이 넓게 보이는 크로스는 엔드가 될 수 없다. 엔드 이벤트가 완성되면:

1. 모터 정지
2. 센서 정지
3. TIM7 정지
4. `Track_FinalizeSegments()` 실행
5. 상태를 `FIRST_DRIVE_STOPPED`로 변경

### 6.8 안전 정지

주요 fault:

- 캘리브레이션 없음
- 시작 라인 없음
- 센서 프레임 정지
- 라인 연속 손실
- 모터 명령 오류
- TIM7 제어 watchdog
- Track RAM overflow
- 직선/전이 상태의 외곽 고착

라인 손실 허용 시간은 상황별로 다르다.

- 직선 중앙: 60 ms
- 전이: 80 ms
- 확인된 TURN/외곽: 120 ms
- 센서 없는 다리 진입: 140 ms
- 같은 쪽 S0/S7 증거가 있는 다리 복구: 최대 250 ms
- 독립적인 불안정 제한: 기본 150 ms

확인된 TURN에서는 일정 곡률 때문에 S1/S6에 오래 머무는 것이 정상일 수 있어 EDGE STUCK 타이머로 정지시키지 않는다. 이 수정으로 270도 원형 구간을 통과했다.

## 7. RAM에 저장되는 First Drive 트랙 정보

### 7.1 저장 위치와 수명

`Main/Src/track.c`의 정적 배열에만 저장된다.

- Flash/EEPROM 저장 아님
- 전원 차단 또는 MCU reset 시 사라짐
- `Track_Reset()` 호출 시 즉시 사라짐
- `FirstDrive_Init()`과 `FirstDrive_Start()`는 현재 `Track_Reset()`을 호출함

따라서 First Drive 정상 종료 후 Second Drive가 맵을 사용하기 전에 `FirstDrive_Init()`, `FirstDrive_Start()`, `Track_Reset()`을 다시 호출하면 안 된다.

### 7.2 용량

- `TRACK_MAX_EVENTS = 512`
- `TRACK_MAX_SEGMENTS = 512`
- event/segment count와 API index는 `uint16_t`
- 약 200개 마커가 있는 실제 대회를 고려해 여유를 둠
- 513번째 이벤트 저장 시 overflow

ELF 심볼 기준:

- events: 18,432 B, 이벤트당 36 B
- segments: 4,096 B, 세그먼트당 8 B
- 합계 약 22.5 KB

과거 64개와 `uint8_t` count를 사용했을 때 `FAULT TRACK FULL`, `N64`가 발생했다. 배열만 늘리고 count를 8비트로 남기는 실수를 하지 않는다.

### 7.3 `TrackMarkerEvent_t`

각 마커 이벤트는 다음 정보를 보존한다.

```c
MarkerEventType_t type;
uint8_t edge_union;
uint8_t full_union;
uint8_t max_center_count;
uint16_t edge0_run;
uint16_t edge7_run;
uint16_t both_overlap_run;
uint16_t wide_center_run;
uint32_t entry_frame;
uint32_t exit_frame;
uint32_t entry_step;
uint32_t exit_step;
uint32_t center_step;
uint8_t confidence;
```

이벤트 종류:

- `EDGE_0`: 왼쪽 방향 마커
- `EDGE_7`: 오른쪽 방향 마커
- `CROSS`: 중앙 광폭 크로스
- `BOTH`: 출발/엔드 후보
- `UNKNOWN`

분류 핵심:

- 중앙 광폭 3프레임 이상이면 CROSS 우선
- 그 외 S0/S7 동시 overlap 2프레임 이상이면 BOTH
- 한쪽 edge union이면 EDGE_0 또는 EDGE_7
- 드라이브는 confidence 20 미만 이벤트를 제어에 사용하지 않음

### 7.4 `TrackSegment_t`

First Drive가 엔드마커에서 정상 종료될 때 이벤트 배열을 세그먼트 배열로 변환한다.

```c
typedef struct {
    TrackSegmentType_t type;
    uint32_t distance_steps;
} TrackSegment_t;
```

종류:

- STRAIGHT
- LEFT
- RIGHT
- CROSS
- END

`distance_steps`는 현재 이벤트 중심과 다음 이벤트 중심 사이의 평균 생성 스텝 차이다. 시작 직후 300 step 안의 BOTH 이벤트는 출발 마커로 보고 세그먼트 생성에서 제외한다.

공개 API:

```c
uint16_t Track_GetEventCount(void);
uint16_t Track_GetSegmentCount(void);
const TrackMarkerEvent_t *Track_GetEvent(uint16_t index);
const TrackSegment_t *Track_GetSegment(uint16_t index);
bool Track_HasOverflow(void);
void Track_FinalizeSegments(void);
```

### 7.5 매우 중요한 거리 주의사항

차량에는 엔코더가 없다. `Motor_DriveGetAverageSteps()`는 모터에 생성한 step pulse 수를 센 것이지 실제 바퀴 이동을 측정한 것이 아니다.

- 탈조
- 미끄러짐
- 좌우 속도차
- 바닥 마찰
- 고속 코너의 슬립

이 있으면 실제 거리와 달라진다.

현재 주석 기준 바퀴 지름 5 cm, 400 half-step/rev이므로 무슬립 이론값은:

```text
1 step ≈ 0.393 mm
300 step ≈ 11.8 cm
510 step ≈ 20.0 cm
600 step ≈ 23.6 cm
```

Second Drive에서 step 거리는 감속 시작을 예측하는 보조 수단으로만 쓰고, 세그먼트 전환의 최종 기준은 실제 마커 재검출이어야 한다.

## 8. Second Drive 권장 설계

### 8.1 시작 조건

Second Drive는 다음 조건을 모두 확인한 뒤 활성화하는 것이 안전하다.

- First Drive 상태가 정상 `STOPPED`
- telemetry의 `end_candidate == 1`
- First Drive fault 없음
- `Track_HasOverflow() == false`
- `Track_GetEventCount() > 0`
- `Track_GetSegmentCount() > 0`
- 캘리브레이션 유지

First Drive가 수동 중단 또는 fault로 끝났다면 불완전한 맵으로 Second Drive를 시작하지 않는 것이 기본 정책이다.

### 8.2 모듈 분리

권장 파일:

```text
Main/Inc/second_drive.h
Main/Src/second_drive.c
```

First Drive의 검증된 센서 처리와 안전 로직을 복사해서 두 군데로 갈라놓기보다, 가능한 부분을 공통 helper로 분리한다.

공통 유지 대상:

- S1-S6 위치 계산
- 마커 spill 제거
- 직전 위치 기반 군집 선택
- 센서 없는 다리 복구
- line-loss watchdog
- TIM7/control watchdog
- 조향 slew
- 엔드/크로스 분리

Second Drive 전용 대상:

- 세그먼트 index
- 예상 다음 마커와 기대 방향
- 기록 거리 기반 가속/감속 계획
- 세그먼트별 목표 속도
- 예상 마커와 실제 마커 불일치 처리

### 8.3 세그먼트 동기화

권장 방식:

1. 현재 segment index를 가진다.
2. 해당 segment type으로 속도 envelope를 선택한다.
3. 기록된 `distance_steps`를 이용해 다음 마커 전 감속 시작점을 예측한다.
4. 실제 마커가 검출되면 그 순간 segment index를 확정 전진시킨다.
5. 예상 거리와 실제 마커가 다르면 실제 마커를 우선하고 오차를 다음 세그먼트에 누적하지 않는다.
6. 예상 마커가 일정 허용 범위 안에 나오지 않으면 고속을 유지하지 말고 First Drive 수준으로 fallback한다.

### 8.4 권장 속도 정책

초기 구현은 한 번에 최고속을 노리지 않는다.

- STRAIGHT:
  - First Drive보다 높은 목표 속도 사용
  - 다음 마커까지 기록 거리가 충분한 경우만 가속
  - 다음 마커 전 보수적인 braking margin 확보
- LEFT/RIGHT:
  - 처음에는 First Drive의 2200 SPS 수준을 기준으로 시작
  - First Drive에서 안정적으로 통과한 곡선임을 전제로 단계적으로 상승
- CROSS:
  - 직전 자세를 유지해 직진 통과
  - 가로 라인의 위치 평균을 따라가지 않음
- END:
  - V25의 엔드 판별 조건을 그대로 사용해 정지

고속에서는 센서 지연, 마커 5 cm 통과 시간, 감속 거리 모두 짧아지므로 step 기반 lookahead와 마커 provisional 검출을 함께 사용한다.

### 8.5 현재 맵에 없는 정보

현재 `TrackSegment_t`에는 type과 distance만 있다. 다음 정보는 저장하지 않는다.

- 실제 통과 시간
- 구간 최대 |position|
- 평균/최대 조향비
- 코너에서의 line-loss 횟수
- 추천 통과 속도
- 곡률 또는 반경

Second Drive 성능을 더 높이려면 First Drive 중 세그먼트별 통계 구조를 추가하는 것이 좋다. 예:

```c
typedef struct {
    uint32_t entry_step;
    uint32_t exit_step;
    uint32_t elapsed_ms;
    uint16_t max_abs_position;
    uint16_t max_steer_permille;
    uint16_t line_loss_count;
    uint16_t recommended_sps;
} TrackSegmentProfile_t;
```

다만 새 배열을 512개 추가하기 전에 RAM 사용량을 계산하고 링크 결과를 확인한다. 현재 전체 RAM 여유는 충분하지만 무계획하게 이벤트 원본을 중복 저장하지 않는다.

## 9. 알려진 회귀 위험과 금지사항

1. S0/S7을 위치 가중평균에 다시 넣지 않는다.
2. S0/S7을 마커와 외곽 복구 증거 외의 연속 위치로 사용하지 않는다.
3. 센서 없는 다리에서 즉시 line lost 처리하지 않는다.
4. 마커 spill이 S1/S2 또는 S5/S6를 덮을 수 있다는 점을 무시하지 않는다.
5. 크로스보다 BOTH 판정을 우선하지 않는다.
6. 엔드마커는 단순 `edge_union == 0x81`만으로 정지하지 않는다.
7. 일정 곡률 TURN의 S1/S6 장기 감지를 EDGE STUCK으로 정지시키지 않는다.
8. 라인 로스 중 가짜 새 위치나 큰 D kick을 만들지 않는다.
9. First Drive 완료 후 Second Drive가 읽기 전에 `Track_Reset()`을 호출하지 않는다.
10. generated step count를 엔코더 실제 거리라고 가정하지 않는다.
11. TIM7 IRQ enable과 2:1 IRQ/control 비율을 확인하지 않고 주행하지 않는다.
12. 모터 DAC를 과거의 512 수준으로 낮추지 않는다.
13. 매 펌웨어 업데이트마다 `APP_VERSION_NUMBER`를 1 증가시킨다.
14. 화면 전체를 매 프레임 다시 그리지 않는다. 현재 메뉴는 변경된 줄 위주로 갱신해 깜빡임을 줄였다.

## 10. 지금까지의 주요 개발 이력

세부 버전별 전체 diff를 보관한 문서는 아니며, 핵심 진화 과정은 다음과 같다.

- 초기: TIM7 Global Interrupt가 꺼져 라인 추종 제어가 실행되지 않는 문제 확인 및 활성화
- 초기 메뉴: 진단 메뉴를 센서 raw, calibration, sensor state, motor phase, motor speed, PD tuning, First Drive 중심으로 정리
- 화면: 전체 화면 반복 갱신을 줄여 깜빡임 완화, 우측 상단 버전 표시 추가
- 센서: S0가 물리적 왼쪽, S7이 오른쪽임을 바로잡음
- 센서: 0-1000 정규화, 검은 기준 차이 기반 정규화, 히스테리시스, median-3 적용
- 캘리브레이션: 검은 quantile 및 흰 상위 샘플 기반 품질 판정으로 재작성
- 모터: DAC 1536 공통화, 초기 저전류 탈조 위험 완화
- First Drive: PD 연속성, 라인 로스 시 마지막 조향 유지, 2초 출발 지연, fault 로그 추가
- 중기: 방향 마커 provisional 처리와 course phase 로그 추가
- 중기: 군집 연속성 및 마커 spill 격리 추가
- 중기: 일정 곡률 225/270도에서 EDGE STUCK 오검출을 수정
- V19 이후: S0/S7을 자세 추종에서 완전히 제외하고 S1-S6만 위치에 사용
- V20/V21: 센서 없는 다리와 외곽 복구, 마커 이벤트 로그 강화
- V23: Track 이벤트/세그먼트 용량 64 -> 512, count/index `uint16_t`
- V24: 양측 엔드마커 정상 정지, 흰선 캘리브레이션 최소 8초
- V25: 크로스 라인이 엔드마커로 오인되는 문제를 막기 위해 광폭 중앙 패턴을 CROSS로 우선 분류하고 엔드 정지에 추가 방어조건 적용

## 11. 다음 작업자가 가장 먼저 할 일

1. V25로 크로스 라인을 실제 주행 테스트한다.
   - 기대 로그: `CR`
   - 정지하면 안 됨
2. 실제 엔드마커를 테스트한다.
   - 기대 로그: `BT`
   - 크로스와 달리 중앙 광폭 조건이 없어야 함
   - 정상 STOP 및 segment finalize 확인
3. First Drive 직후 `Track_GetEventCount()`와 `Track_GetSegmentCount()`를 화면 또는 디버거로 확인한다.
4. 이벤트/세그먼트 몇 개를 출력하여 type, center_step, distance_steps가 실제 코스 순서와 맞는지 확인한다.
5. 맵을 지우지 않는 Second Drive 메뉴 진입 경로를 먼저 만든다.
6. Second Drive 첫 버전은 First Drive와 같은 코너 속도 + 직선만 제한적으로 고속화한다.
7. 마커 기반 동기화와 안전 fallback이 검증된 뒤 전체 속도를 단계적으로 높인다.

## 12. 최종 원칙

First Drive의 목적은 단순 완주뿐 아니라 Second Drive가 사용할 신뢰할 수 있는 마커 순서와 구간 길이 추정치를 RAM에 남기는 것이다. Second Drive는 이 정보를 이용해 미리 가속하고 감속하되, 엔코더가 없으므로 기록 step을 절대적인 실제 거리로 믿어서는 안 된다. 실제 센서 라인과 마커가 항상 최종 판단 기준이며, 기록 지도는 예측과 속도 계획을 위한 보조 정보다.
