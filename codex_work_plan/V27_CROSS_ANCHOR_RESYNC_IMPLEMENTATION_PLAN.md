# Line Tracer V27 - 교차로 앵커 재동기화 및 고속 교차로 주행 구현 계획

> 대상 구현자: Codex 5.6 Luna Max  
> 기준 코드: V26  
> 목표 버전: V27  
> 실제 프로젝트 경로: `/Users/ehoi/STM32CubeIDE/line_tracer_2026`  
> 작성 목적: 이전 대화 컨텍스트 없이 이 문서와 프로젝트만 보고 안전하게 V27을 구현할 수 있게 하는 자립형 명세

---

## 0. Luna에게 먼저 내리는 작업 지시

이 문서를 끝까지 읽고 아래 우선순위대로 구현한다.

1. V26의 검증된 First Drive 라인 추종과 안전 로직을 보존한다.
2. Second Drive에서 마커 1회 불일치가 영구 fallback을 만드는 구조를 제거한다.
3. 회전 마커를 놓친 경우 다음 실제 교차로를 절대 위치 앵커로 사용해 맵을 재동기화한다.
4. 맵이 동기화된 상태에서는 교차로를 코너가 아니라 직선 경로로 취급한다.
5. 단, 다음 코너까지 제동거리가 부족하면 교차로 전부터 정상적으로 감속해야 한다.
6. 시립대 규정의 고정 곡률반경 25cm와 45도 단위 원호 정보를 맵 메타데이터와 진단에 반영한다.
7. UI에서 `MAP SYNC`, `SEEK CROSS`, 재동기화 성공 횟수와 불일치 원인을 확인할 수 있게 한다.
8. 앱 버전을 반드시 V27로 올리고 Debug clean build를 통과시킨다.

임의로 전체 제어기를 다시 작성하지 않는다. 이번 작업은 `track`, `second_drive`, Second Drive UI 중심의 제한된 변경이다.

---

## 1. 현재 프로젝트 기준점

### 1.1 경로와 버전

- 실제 프로젝트 경로: `/Users/ehoi/STM32CubeIDE/line_tracer_2026`
- 이전 경로 `/Users/ehoi/STM32CubeIDE/line_tracer`는 더 이상 실제 프로젝트가 아니다.
- 현재 버전: `Main/Inc/app_version.h`의 `APP_VERSION_NUMBER 26U`
- 목표 버전: `27U`
- Git 브랜치: `main`
- 현재 코드 변경사항은 없고 `.DS_Store`만 수정 상태였다. 사용자 파일이므로 건드리지 않는다.

### 1.2 현재 빌드 기준

기존 V26 Debug ELF 기준:

- text: 109,872 B
- data: 416 B
- bss: 38,056 B
- RAM 합계: 38,472 B
- 이전 clean build 결과: RAM 13.81%, Flash 21.04%

빌드 명령:

```sh
cmake --build --preset Debug --clean-first -j 8
```

기존 경고로 `Drivers/BSP/ST7735/st7735_lcd.c`의 미사용 지역 변수 `text`가 있다. 이번 변경과 무관하므로 별도 요청 없이 수정하지 않는다.

### 1.3 실주행 검증 상태

V26으로 다음이 실제 성공했다.

- First Drive 완주 성공
- First Drive 맵을 보존한 Second Drive 완주 성공
- 교차로와 엔드마커 구분 성공
- Second Drive 메뉴에서 직선 속도와 전체 속도 조정 성공

실주행에서 발견된 문제:

1. `CROSS` 구간이 직선 최고속 가속 대상에서 제외되어 교차로 이후 최고속 효과가 작았다.
2. 예상 마커와 실제 이벤트가 한 번만 어긋나도 `fallback_active`가 주행 종료까지 유지되었다.
3. 불일치 이벤트에서도 segment/event index가 전진해 이후 맵 위치가 연쇄적으로 어긋날 수 있었다.
4. fallback 이후 모든 맵 기반 가속이 사라져 Second Drive의 시간 단축 효과가 크게 줄었다.

---

## 2. 반드시 먼저 읽을 소스

우선순위:

1. `Main/Src/second_drive.c`, `Main/Inc/second_drive.h`
2. `Main/Src/track.c`, `Main/Inc/track.h`
3. `Main/Src/drive.c`, `Main/Inc/drive.h`
4. `Main/Src/menu.c`
5. `Main/Src/sensor.c`, `Main/Inc/sensor.h`
6. `Main/Src/motor.c`, `Main/Inc/motor.h`
7. `Main/Src/marker.c`, `Main/Inc/marker.h`
8. `V25_SECOND_DRIVE_HANDOFF.md`

빌드 구조:

- `Main/CMakeLists.txt`가 `Main/Src/*.c`를 glob으로 포함한다.
- 새 `.c` 파일이 꼭 필요한 상황이 아니면 만들지 않는다.
- 이번 기능은 기존 `track.c`와 `second_drive.c` 안에서 구현하는 것을 우선한다.

---

## 3. 공식 시립대 규정에서 확정된 코스 조건

공식 규정 URL:

`https://zetin.uos.ac.kr/?mid=contest_limit`

2026-08-10 확인 내용:

- 검은 바탕, 흰색 주행선
- 주행선 폭 2cm
- 주행선은 직선과 원호로 구성
- 원호 곡률반경은 항상 25cm
- 원호 중심각은 45도 단위이며 45도에서 270도 사이
- 턴마크 폭 2cm, 길이 5cm
- 턴마크는 주행선 가장자리에서 4cm 떨어짐
- 곡선 시작점에 회전 방향 마크가 있음
- 곡선 종료 후 직선 시작점에 마지막 회전 방향과 같은 마크가 있음
- 회전 방향은 연속해서 바뀔 수 있음
- 교차로 주행선은 서로 수직
- 교차로 중심으로부터 적어도 20cm까지 직선
- 교차로에서는 직진만 허용

중요한 해석:

- 교차로는 별도의 좌우 마커가 아니라 서로 수직으로 만나는 주행선 자체의 광폭 센서 패턴이다.
- 교차로는 방향 선택 구간이 아니다. 정상 경로는 무조건 직진이다.
- 맵 동기화가 확실하면 교차로를 코너처럼 2400 SPS로 제한할 이유가 없다.
- 다만 교차로 뒤 20cm 지점에 바로 코너가 올 수 있으므로, 제동거리 계산은 교차로 다음 구간까지 봐야 한다.
- 연속 곡선이 허용되므로 모든 곡선 사이에 가속 가능한 직선이 있다고 가정하면 안 된다.

---

## 4. 하드웨어와 제어기에서 절대 바꾸면 안 되는 사실

### 4.1 센서 역할

- S0: 물리적 왼쪽 외곽, 왼쪽 마커 및 외곽 복구 증거
- S1-S6: 정상 주행선 위치 추종 전용
- S7: 물리적 오른쪽 외곽, 오른쪽 마커 및 외곽 복구 증거
- S1 쪽 위치는 음수, S6 쪽 위치는 양수
- 위치 범위는 약 -2500에서 +2500

금지:

- S0/S7을 정상 위치 가중평균에 넣지 않는다.
- S0/S7을 연속 위치 센서처럼 사용하지 않는다.
- 마커 spill 제거와 센서 없는 다리 복구를 삭제하지 않는다.

### 4.2 제어 주기

- TIM7 IRQ: 2kHz
- `FIRST_DRIVE_CONTROL_DIVIDER = 2`
- 실제 제어: 1kHz
- Second Drive planner는 센서 프레임 처리 경로에서 호출된다.
- UI는 main context에서 telemetry를 읽는다.

planner 상태는 IRQ context에서 변경될 수 있으므로:

- 동적 할당 금지
- 긴 블로킹 처리 금지
- 이벤트가 발생하지 않은 매 1ms 경로에서 전체 맵 스캔 금지
- 앵커 검색은 CROSS 이벤트가 완성된 순간에만 수행
- telemetry 구조 복사는 기존처럼 짧은 critical section을 사용

### 4.3 모터 거리의 의미

`Motor_DriveGetAverageSteps()`는 엔코더 거리가 아니다.

```c
average = (left_generated_steps + right_generated_steps) / 2
```

영향 요소:

- 탈조
- 슬립
- 좌우 조향량
- 실제 타이어 지름
- 바닥 마찰

따라서 step 거리는 예측과 후보 비교용이고, 실제 마커/교차로 센서 이벤트가 최종 권위다.

### 4.4 현재 구동 수치

- 바퀴 지름 주석 기준: 5cm
- 1회전: 400 half-step
- 모터 절대 최대: 6500 SPS
- Second Drive 직선 설정 범위: 4000-6200 SPS
- 기본 Second Drive 직선 속도: 5200 SPS
- First Drive 기본 속도: 3820 SPS
- First Drive 회전 상한: 2200 SPS
- First Drive 마커/CROSS 제한: 2400 SPS
- 가속: 8 SPS/ms
- 감속: 10 SPS/ms
- 모터 전류 DAC: 1536, 약 0.91 A/phase

모터 DAC를 512 수준으로 낮추지 않는다. 실제 차체에서 토크 부족과 탈조 위험이 있었다.

---

## 5. 규정 수치와 현재 step 수치의 관계

이론상:

```text
wheel circumference = 5*pi cm
distance per step = 5*pi/400 cm = 약 0.03927cm = 0.3927mm
20cm = 약 509.3 step
```

현재:

```c
#define FIRST_DRIVE_CROSS_PASS_STEPS 510U
```

이는 규정의 교차로 중심 이후 최소 20cm 직선과 거의 정확히 일치한다. 이 상수는 유지한다.

곡률반경 25cm에서 원호 길이와 이론 step:

| 각도 | 원호 길이 | 이론 평균 step | curve unit |
|---:|---:|---:|---:|
| 45도 | 19.635cm | 500 | 1 |
| 90도 | 39.270cm | 1000 | 2 |
| 135도 | 58.905cm | 1500 | 3 |
| 180도 | 78.540cm | 2000 | 4 |
| 225도 | 98.175cm | 2500 | 5 |
| 270도 | 117.810cm | 3000 | 6 |

따라서 V27에서는 LEFT/RIGHT segment에 `curve_units`를 저장한다.

- 0: 직선/CROSS/END 또는 신뢰할 수 없는 곡선 길이
- 1-6: 45도 단위 곡선 길이
- 위치 확정의 최종 기준으로 사용하지 않는다.
- 앵커 진단, 맵 구조 검사, 향후 속도 최적화를 위한 보조 메타데이터다.

---

## 6. 현재 V26 데이터 흐름

```text
Sensor frame
  -> Sensor_GetLineMeasurement()
  -> marker spill filter
  -> tracked cluster selection
  -> bridge recovery
  -> FirstDrive_ProcessMarker()
       First Drive: Track_ProcessSensor()
       Second Drive: Track_ProcessReplaySensor()
  -> FirstDrive_UpdateMotorCommand()
       FirstDrive_GetTargetBaseSps()
       Second Drive이면 SecondDrivePlanner_GetTargetSps()
       speed ramp
       PD/steer 적용
       Motor_DriveSetSpeeds()
```

First Drive 정상 종료:

```text
BOTH end event
  -> FirstDrive_StopAtEndMarker()
  -> Track_FinalizeSegments()
  -> drive_map_ready = true if structurally valid
  -> STOPPED
```

Second Drive 진입:

```text
SecondDrive_Init()
  -> Track_ReplayReset()
  -> SecondDrivePlanner_Reset()
  -> 기존 events[]/segments[] 보존
```

First Drive를 다시 init/start하면 `Track_Reset()`이 호출되어 기존 맵이 지워진다. 정상 First Drive 종료 후 Second Drive 전에 First Drive init/start를 다시 호출하면 안 된다.

---

## 7. 현재 V26의 정확한 결함

### 7.1 불일치 한 번으로 영구 fallback

`Main/Src/second_drive.c`의 `SecondDrivePlanner_OnEvent()`는 다음 중 하나면 즉시:

```c
planner_status.fallback_active = 1U;
```

- 예상 event type과 실제 type 불일치
- 기록 segment 거리와 실제 이동거리 차이가 허용범위 초과
- current/next segment 또는 expected event가 없음

`SecondDrivePlanner_GetTargetSps()`는 이후 항상 `first_drive_target_sps`를 반환하며 주행 중 복구 경로가 없다.

### 7.2 불일치 이벤트에서도 인덱스 전진

현재 코드는 `mismatch == true`여도 아래 동작을 수행한다.

```text
expected_event_index++
segment_index++
segment_start_step = false event step
```

한 번의 false event가 이후 전체 event sequence를 밀어버릴 수 있다.

V27의 핵심 불변조건:

> 검증에 실패한 이벤트에서는 event index와 segment index를 절대로 전진시키지 않는다.

### 7.3 CROSS를 제한 구간으로 취급

현재:

- 다음 segment가 CROSS면 2400 SPS를 braking target으로 사용
- current segment가 CROSS면 fast straight 조건에서 제외
- course phase가 CROSS면 First Drive layer가 2400 SPS로 제한
- planner가 이를 다시 고속으로 올리지 않음

결과:

- 교차로 전 선제 감속
- 교차로 통과 중 감속
- 교차로 이후 다음 마커까지 최고 직선 속도 사용 불가

### 7.4 한 segment만 보는 제동 lookahead

CROSS를 단순히 직선 target으로 바꾸기만 하면 또 다른 위험이 생긴다.

- 현재 straight의 다음 segment가 CROSS이면 planner는 다음 코너를 보지 못함
- 규정은 교차로 중심 뒤 최소 20cm 직선만 보장
- 6200 -> 2200 SPS의 현재 제동거리와 margin은 20cm보다 훨씬 길 수 있음

따라서 V27은 STRAIGHT/CROSS를 통과해 다음 LEFT/RIGHT/END까지 누적 거리를 계산하는 다중 segment lookahead가 필요하다.

---

## 8. V27 목표 상태 머신

기존 `fallback_active` 중심 구조를 다음 sync state로 교체한다.

```c
typedef enum {
	SECOND_DRIVE_SYNC_MAP = 0,
	SECOND_DRIVE_SYNC_SEEK_CROSS,
	SECOND_DRIVE_SYNC_INVALID
} SecondDriveSyncState_t;
```

의미:

### `SECOND_DRIVE_SYNC_MAP`

- event/segment index가 실제 위치와 동기화됨
- 맵 기반 직선 가속 허용
- 맵 기반 고속 CROSS 직진 허용
- 다음 제한 segment까지 braking lookahead 사용

### `SECOND_DRIVE_SYNC_SEEK_CROSS`

- 회전 마커 누락, false event, 거리 초과 등으로 현재 맵 위치를 신뢰하지 않음
- 이벤트/segment index를 임의로 전진시키지 않음
- First Drive 속도 envelope 사용
- CROSS 이벤트는 계속 검출
- 다음 신뢰 가능한 CROSS anchor에서 맵 재동기화 시도
- 재동기화 성공 시 `SYNC_MAP`으로 복귀

### `SECOND_DRIVE_SYNC_INVALID`

- First Drive map이 구조적으로 유효하지 않음
- Second Drive 시작 자체를 `NO TRACK`으로 차단
- 런타임 단일 mismatch 때문에 이 상태로 가지 않는다.

상태 흐름:

```text
SYNC_MAP
  | expected marker mismatch
  | segment overdue
  | bounds error
  v
SEEK_CROSS
  | valid forward CROSS anchor matched
  v
SYNC_MAP

INVALID
  -> Second Drive start prohibited
```

맵에 CROSS가 없으면:

- 정상 동기화 상태에서는 기존 event matching 사용 가능
- mismatch 후에는 남은 주행을 First Drive envelope로 완료
- END 검출과 안전정지는 그대로 동작

---

## 9. 제안 데이터 구조

### 9.1 `track.h`

다음 상수를 추가한다.

```c
#define TRACK_MAX_CROSS_ANCHORS              64U
#define TRACK_CURVE_UNIT_NOMINAL_STEPS      500U
#define TRACK_CURVE_UNIT_MIN                  1U
#define TRACK_CURVE_UNIT_MAX                  6U
#define TRACK_CURVE_UNIT_ERROR_MIN_STEPS    150U
```

`TrackSegment_t`를 확장한다.

```c
typedef struct {
	TrackSegmentType_t type;
	uint32_t distance_steps;
	uint8_t curve_units; /* 0 unknown/not curve, 1..6 = 45..270 degrees */
} TrackSegment_t;
```

정렬 padding으로 크기가 늘어날 수 있다. build 후 map/size로 확인한다.

교차로 앵커:

```c
typedef struct {
	uint16_t order;
	uint16_t event_index;
	uint16_t segment_index_after_cross;
	uint32_t center_step;
	uint32_t distance_from_previous_cross;
} TrackCrossAnchor_t;
```

공개 API:

```c
uint16_t Track_GetCrossAnchorCount(void);
const TrackCrossAnchor_t *Track_GetCrossAnchor(uint16_t index);
const TrackCrossAnchor_t *Track_FindCrossAnchorByEventIndex(
		uint16_t event_index);
```

필요하면 `bool Track_HasAnchorOverflow(void)`를 추가하고 구조적 맵 검사에 포함한다.

### 9.2 `track.c`

정적 저장:

```c
static TrackCrossAnchor_t cross_anchors[TRACK_MAX_CROSS_ANCHORS];
static uint16_t cross_anchor_count;
static bool cross_anchor_overflow;
```

`Track_Reset()`은 반드시 위 배열/count/overflow를 초기화한다.

`Track_ReplayReset()`은 First Drive map 및 anchor를 지우면 안 된다.

### 9.3 `second_drive.h`

mismatch reason:

```c
typedef enum {
	SECOND_DRIVE_MISMATCH_NONE = 0,
	SECOND_DRIVE_MISMATCH_EVENT_TYPE,
	SECOND_DRIVE_MISMATCH_EVENT_DISTANCE,
	SECOND_DRIVE_MISMATCH_SEGMENT_OVERDUE,
	SECOND_DRIVE_MISMATCH_MAP_BOUNDS,
	SECOND_DRIVE_MISMATCH_ANCHOR_NOT_FOUND,
	SECOND_DRIVE_MISMATCH_ANCHOR_AMBIGUOUS
} SecondDriveMismatchReason_t;
```

`SecondDrivePlannerStatus_t` 권장 형태:

```c
typedef struct {
	uint8_t map_valid;
	SecondDriveSyncState_t sync_state;
	SecondDriveMismatchReason_t last_mismatch_reason;

	uint8_t mismatch_count;
	uint8_t resync_count;
	uint16_t ignored_event_count;
	uint16_t replay_event_count;

	uint16_t expected_event_index;
	uint16_t segment_index;
	uint16_t segment_count;

	uint16_t anchor_count;
	uint16_t current_anchor_order; /* UINT16_MAX if none confirmed */

	TrackSegmentType_t segment_type;
	TrackSegmentType_t next_segment_type;
	uint8_t curve_units;

	uint32_t segment_distance_steps;
	uint32_t segment_travelled_steps;
	uint32_t segment_remaining_steps;
	uint32_t next_restriction_distance_steps;
} SecondDrivePlannerStatus_t;
```

기존 `fallback_active`를 남겨 이중 상태를 만들지 않는다. `menu.c`를 `sync_state` 기준으로 함께 수정한다.

---

## 10. First Drive map finalize 구현

### 10.1 segment 생성은 기존 의미 유지

현재 의미:

- 첫 실제 event까지 초기 STRAIGHT segment가 있을 수 있음
- event `i`의 type이 event `i`부터 `i+1`까지 segment type을 결정
- 같은 방향 EDGE pair:
  - 첫 EDGE: 해당 방향 curve 시작
  - 같은 방향 다음 EDGE: curve 종료 후 다음 segment를 STRAIGHT로 시작
- CROSS event 이후 다음 marker까지 현재는 CROSS segment
- BOTH는 END segment

V27에서는 기존 `TRACK_SEGMENT_CROSS` type을 없애지 않는다.

- map topology와 UI에는 CROSS 의미가 필요하다.
- speed planner에서만 CROSS를 straight geometry로 취급한다.

### 10.2 `curve_units` 계산

LEFT/RIGHT segment에만 적용한다.

권장 정수 계산:

```c
units = (distance_steps + TRACK_CURVE_UNIT_NOMINAL_STEPS / 2U)
		/ TRACK_CURVE_UNIT_NOMINAL_STEPS;
units = clamp(units, 1, 6);
expected = units * TRACK_CURVE_UNIT_NOMINAL_STEPS;
error = abs(distance_steps - expected);
tolerance = max(TRACK_CURVE_UNIT_ERROR_MIN_STEPS, expected / 4U);
curve_units = (error <= tolerance) ? units : 0U;
```

주의:

- floating point를 넣지 않는다.
- `curve_units`가 0이어도 map을 무효화하지 않는다.
- generated step 오차와 path cutting이 있으므로 진단 메타데이터로만 사용한다.

### 10.3 anchor 생성

`Track_FinalizeSegments()`가 모든 segment를 만든 후 CROSS event를 순회해 anchor를 만든다.

각 CROSS event에 대해 저장:

- 원본 event index
- CROSS 직후 사용할 segment index
- First Drive의 절대 center step
- 이전 CROSS center와의 step 차이
- anchor 순번

segment index 계산을 추측으로 여러 곳에 복제하지 않는다. finalize 과정에서 event와 생성 segment의 관계를 한 곳에서 확정한다.

권장 방법:

- finalize 중 `first_index`와 초기 STRAIGHT 존재 여부를 이미 알고 있다.
- non-END event `i`가 시작하는 segment index를 계산해 CROSS anchor에 기록한다.
- anchor의 `segment_index_after_cross`는 Second Drive에서 CROSS를 검출한 직후 활성화할 segment다.
- index가 `segment_count` 범위를 벗어나면 anchor를 만들지 말고 overflow/invalid 진단을 남긴다.

구조적 유효성:

- events > 0
- segments > 0
- 마지막 segment type END
- track overflow 없음
- anchor overflow 없음

CROSS가 0개인 것은 유효한 맵일 수 있으므로 map invalid 조건이 아니다.

---

## 11. Second Drive 이벤트 처리 상세

### 11.1 공통 원칙

- confidence < 20 이벤트는 기존처럼 planner 제어에 사용하지 않는다.
- 시작 300 step 안의 BOTH는 기존처럼 무시한다.
- 실제 END 판정과 모터 정지는 계속 `drive.c`가 담당한다.
- planner가 SEEK 상태여도 marker detector와 END detector는 계속 실행한다.
- 검증 실패 이벤트에서는 segment/event index를 전진시키지 않는다.

### 11.2 정상 `SYNC_MAP` 이벤트

기존 expected event 비교를 유지하되 로직을 helper로 분리한다.

추천 helper:

```c
static bool SecondDrive_EventMatchesExpected(...);
static void SecondDrive_AdvanceExpectedEvent(...);
static void SecondDrive_EnterSeekCross(...);
static bool SecondDrive_TryResyncAtCross(...);
static void SecondDrive_LoadSegmentStatus(...);
```

정상 match:

1. event type 일치 확인
2. 현재 segment recorded distance와 observed distance 비교
3. match면 index를 정확히 한 번 전진
4. `planner_segment_start_step = event->center_step`
5. mismatch reason을 NONE으로 변경
6. event가 CROSS면 현재 confirmed anchor 정보 갱신

### 11.3 mismatch 처리

type 또는 distance mismatch:

```text
mismatch_count++
last_mismatch_reason 기록
sync_state = SEEK_CROSS
현재 expected_event_index 유지
현재 segment_index 유지
planner_segment_start_step 유지
현재 이벤트는 ignored_event_count++
```

한 이벤트에서 mismatch count를 한 번만 올린다.

### 11.4 segment overdue 처리

`SecondDrivePlanner_GetTargetSps()`에서:

```text
travelled > segment.distance + tolerance
```

이면 `SEEK_CROSS`로 전환한다.

주의:

- 1kHz마다 mismatch count를 반복 증가시키면 안 된다.
- `SYNC_MAP -> SEEK_CROSS` 전이 순간에만 count/reason을 갱신한다.
- 이후 target은 First Drive envelope를 반환한다.

### 11.5 SEEK 상태의 일반 이벤트

- EDGE_0, EDGE_7, UNKNOWN: 맵 index를 바꾸지 않고 무시/진단 count만 갱신
- BOTH: planner는 index를 바꾸지 않지만 drive의 END 판정은 그대로 실행
- CROSS: anchor 재동기화 시도

### 11.6 CROSS anchor 후보 검색

권장 상수:

```c
#define SECOND_DRIVE_ANCHOR_LOOKAHEAD_COUNT          3U
#define SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_MIN 500U
#define SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_DIV   3U
```

후보 제약:

1. 이미 확인한 anchor보다 뒤쪽이어야 함
2. `expected_event_index`보다 앞의 event를 가리키면 안 됨
3. forward anchor 최대 3개까지만 비교
4. 절대 뒤로 rewind하지 않음

거리 기준:

- confirmed anchor가 있으면:
  - observed = 현재 CROSS run step - last confirmed CROSS run step
  - recorded = candidate map step - last confirmed CROSS map step
- confirmed anchor가 없으면:
  - observed = 현재 CROSS run step
  - recorded = candidate map center step

허용범위:

```c
tolerance = max(recorded / 3U, 500U);
```

후보 선택:

- tolerance 안에 들어온 후보 중 distance error가 가장 작은 후보
- 두 후보가 사실상 동률이거나 안전하게 구분되지 않으면 resync하지 않고 SEEK 유지
- 첫 forward candidate가 명확히 맞으면 뒤 후보를 선택하지 않음
- curve unit/signature는 동률 해소용 보조 정보로만 사용 가능하며 필수 거부 조건으로 만들지 않음

### 11.7 resync 성공 처리

```text
expected_event_index = anchor.event_index + 1
segment_index = anchor.segment_index_after_cross
planner_segment_start_step = observed_cross.center_step
last_anchor_order = anchor.order
last_anchor_map_step = anchor.center_step
last_anchor_run_step = observed_cross.center_step
sync_state = SYNC_MAP
last_mismatch_reason = NONE
resync_count++
segment telemetry refresh
```

resync 시 First Drive 원본 `events[]`나 `segments[]`를 수정하지 않는다.

### 11.8 CROSS가 SYNC 상태에서 예상과 다르게 나타난 경우

예상하지 않은 CROSS는 일반 mismatch로 버리기 전에 즉시 forward anchor matching을 시도한다.

- 명확한 anchor가 있으면 그 자리에서 바로 resync
- 없거나 모호하면 SEEK 유지
- false CROSS 하나가 임의 index 전진을 만들면 안 됨

---

## 12. V27 속도 planner 상세

### 12.1 sync 상태별 기본 정책

```text
SYNC_MAP:
  map based acceleration/braking enabled

SEEK_CROSS:
  return first_drive_target_sps
  no map based high-speed prediction
  actual line following and safety still active

INVALID:
  Second Drive start prohibited
```

### 12.2 fast geometry 정의

다음은 고속 직선 geometry다.

```c
segment->type == TRACK_SEGMENT_STRAIGHT
|| segment->type == TRACK_SEGMENT_CROSS
```

CROSS 의미:

- 실제 교차점에서 직진
- CROSS event 이후 다음 marker까지 물리적으로 직선일 수 있음
- map topology type은 CROSS로 유지
- speed class만 STRAIGHT와 동일하게 취급

### 12.3 course phase 허용

STRAIGHT segment:

- `course_phase == FIRST_DRIVE_COURSE_STRAIGHT`일 때만 fast target 허용

CROSS segment:

- `FIRST_DRIVE_COURSE_STRAIGHT` 허용
- `FIRST_DRIVE_COURSE_CROSS`도 sync 상태이면 허용

왜 안전한가:

- CROSS 중 `drive.c`는 중앙 광폭 패턴에서 `position`, `control_position`을 직전 값으로 유지
- derivative를 0으로 두어 가로 가지를 따라가지 않음
- 규정상 교차로는 직진만 허용
- 규정상 중심 전후 최소 20cm 직선

First Drive 또는 SEEK 상태의 CROSS 2400 SPS 제한은 유지한다. `drive.c`의 공통 cap을 전역 삭제하지 않는다. Second Drive planner가 SYNC일 때만 최종 target을 안전하게 override한다.

### 12.4 다음 CROSS를 제한 target으로 사용하지 않기

현재 `SecondDrive_TargetForSegment(TRACK_SEGMENT_CROSS)`는 2400 SPS를 반환한다.

V27 SYNC 상태에서는 CROSS가 speed restriction이 아니므로 straight target을 반환하도록 바꾼다.

단, SEEK 상태는 `GetTargetSps()` 시작에서 First Drive target을 반환하므로 기존 안전 제한을 유지한다.

### 12.5 다중 segment braking lookahead

새 helper 권장:

```c
static uint32_t SecondDrive_DistanceToNextRestriction(
		uint32_t current_remaining,
		uint16_t current_segment_index,
		TrackSegmentType_t *restriction_type);
```

동작:

1. 현재 segment remaining부터 시작
2. 이후 STRAIGHT/CROSS segment의 `distance_steps`를 누적
3. 첫 LEFT/RIGHT/END에서 중단
4. 해당 type의 target speed를 계산
5. 누적 거리를 braking distance와 비교

반드시 bounded scan을 사용한다.

```c
#define SECOND_DRIVE_LOOKAHEAD_MAX_SEGMENTS 16U
```

16개 안에 제한 segment가 없으면:

- map bounds를 확인
- END 또는 현재 straight target을 보수적으로 선택
- 배열 범위를 넘지 않음

braking 판단:

```text
distance_to_restriction
  > braking_steps + brake_margin + accel_enable_margin
    -> straight_sps
  otherwise
    -> restriction target
```

이렇게 해야:

- 교차로 뒤에 충분한 직선이 있으면 고속 통과
- 교차로 뒤 20cm 후 바로 코너면 교차로 전부터 감속
- CROSS 자체 때문에 무조건 감속하지 않음

### 12.6 곡선 target

시립대 원호는 모두 R=25cm이므로 steady curve speed 기준은 동일하게 유지할 수 있다.

- V27 초기값은 기존 2200 SPS reference 유지
- `overall_percent` scaling 유지
- curve angle unit은 곡선 지속거리와 진단에 사용
- 이번 V27에서 45도와 270도 코너의 최고속을 별도로 공격적으로 올리지 않는다.

### 12.7 END

- END는 제한 segment
- 기존 1800 SPS approach target 유지
- 실제 BOTH end 조건과 정지는 `drive.c`의 검증된 로직 유지
- CROSS wide center가 BOTH로 정지하는 회귀를 만들지 않는다.

---

## 13. 파일별 수정 계획

### `Main/Inc/app_version.h`

- `APP_VERSION_NUMBER 26U` -> `27U`

### `Main/Inc/track.h`

- `TrackSegment_t.curve_units` 추가
- `TrackCrossAnchor_t` 추가
- anchor max/curve unit 상수 추가
- anchor getter API 추가

### `Main/Src/track.c`

- cross anchor 정적 배열/count/overflow 추가
- `Track_Reset()`에서 anchor 초기화
- `Track_FinalizeSegments()`에서 curve unit 계산
- finalize 후 CROSS anchor table 생성
- bounds/overflow 방어
- First Drive event collector와 replay detector의 분리 유지

### `Main/Inc/second_drive.h`

- sync state enum 추가
- mismatch reason enum 추가
- planner status telemetry 확장
- 기존 `fallback_active` 중심 API 제거/교체

### `Main/Src/second_drive.c`

- 파일 상단의 “any mismatch permanently falls back” 주석 수정
- helper 기반 event matching/advance/seek/resync 구현
- mismatch 시 index를 전진하지 않도록 수정
- CROSS anchor forward search 구현
- segment overdue를 1회 상태 전이로 처리
- CROSS를 straight speed class로 변경
- 다중 segment lookahead 구현
- SEEK 상태에서는 First Drive target 반환
- 64-bit braking 계산과 기존 config setter 유지

### `Main/Src/drive.c`

가능하면 최소 변경한다.

유지 대상:

- marker detection 호출 순서
- planner `OnEvent()`가 drive의 marker switch 전에 호출되는 구조
- CROSS course phase와 510-step 유지
- CROSS 중앙 광폭에서 직전 position/heading 유지
- First Drive speed cap
- line-loss/edge-stuck/watchdog
- map lifecycle

수정이 필요한 경우:

- Second Drive planner에 추가 runtime 정보가 꼭 필요할 때만 API argument 확장
- First Drive 전역 CROSS cap을 삭제하지 않음
- SYNC 상태의 최종 target override는 되도록 `second_drive.c`에서 처리

### `Main/Src/menu.c`

Second Drive active 화면:

```text
MAP SYNC
SEEK CROSS
MAP INVALID
```

표시 권장:

- segment index/count/type
- current anchor order/anchor count
- expected event index
- remaining 또는 next restriction distance
- mismatch total
- resync count
- last mismatch reason 짧은 문자열
- target/current/L/R speed

문자열 예:

```text
SYNC A2/5 RS1
SEEK TYPE M2
SEEK DIST M3
```

기존 100ms UI update와 부분 redraw 정책을 유지한다. 1kHz마다 LCD를 그리지 않는다.

### 수정하지 않는 파일

특별한 컴파일 필요가 없으면 다음은 변경하지 않는다.

- `sensor.c/h`
- `motor.c/h`
- `marker.c/h`
- CubeMX 생성 파일
- linker script
- `.ioc`

---

## 14. 구현 순서

### 1단계: 데이터 구조

1. `track.h`에 curve unit과 cross anchor 구조 추가
2. `track.c` reset/finalize/anchor getter 구현
3. bounds와 overflow 확인
4. 이 단계에서 build

### 2단계: sync state refactor

1. `second_drive.h` enum/status 변경
2. `SecondDrivePlanner_Reset()`을 새 상태로 초기화
3. 정상 event match helper 작성
4. mismatch 시 SEEK로 전환하고 index 유지
5. overdue 1회 전이 구현
6. build

### 3단계: CROSS resync

1. last confirmed anchor map/run step 상태 추가
2. forward candidate 검색 helper 구현
3. SEEK에서 CROSS 이벤트 처리
4. 성공 시 event/segment index snap
5. 정상 SYNC CROSS에서도 confirmed anchor 갱신
6. build

### 4단계: 속도 planner

1. CROSS를 fast geometry로 취급
2. CROSS restriction target 제거
3. 다중 segment lookahead 구현
4. SYNC CROSS course phase에서 straight target 허용
5. SEEK에서는 First Drive target 유지
6. build

### 5단계: UI와 버전

1. menu의 fallback 표시를 sync state 표시로 교체
2. mismatch reason 문자열 추가
3. anchor/resync telemetry 표시
4. 앱 버전 V27
5. clean build

### 6단계: 정적 검토

1. 모든 array index bounds 검사
2. mismatch event에서 index가 바뀌지 않는지 확인
3. Track_Reset과 ReplayReset 역할 확인
4. First Drive 코드 경로가 그대로인지 확인
5. END/CROSS 분류 우선순위 확인
6. RAM/Flash 증가 확인

---

## 15. 의사코드

### 15.1 event 처리

```c
void SecondDrivePlanner_OnEvent(const TrackMarkerEvent_t *event)
{
	if (!valid_confidence_or_not_start_marker(event)) {
		return;
	}

	status.replay_event_count++;

	if (event->type == MARKER_EVENT_CROSS) {
		if (status.sync_state == SECOND_DRIVE_SYNC_SEEK_CROSS) {
			if (SecondDrive_TryResyncAtCross(event)) {
				return;
			}
			status.ignored_event_count++;
			return;
		}
	}

	if (status.sync_state != SECOND_DRIVE_SYNC_MAP) {
		status.ignored_event_count++;
		return;
	}

	if (SecondDrive_EventMatchesExpected(event)) {
		SecondDrive_AdvanceExpectedEvent(event);
		return;
	}

	if ((event->type == MARKER_EVENT_CROSS)
			&& SecondDrive_TryResyncAtCross(event)) {
		return;
	}

	SecondDrive_EnterSeekCross(detected_reason);
	status.ignored_event_count++;
}
```

### 15.2 target 처리

```c
uint16_t SecondDrivePlanner_GetTargetSps(...)
{
	if (!map_valid || sync_state != SECOND_DRIVE_SYNC_MAP) {
		return first_drive_target_sps;
	}

	segment = Track_GetSegment(segment_index);
	if (segment == NULL) {
		SecondDrive_EnterSeekCross(MAP_BOUNDS);
		return first_drive_target_sps;
	}

	update_remaining();
	if (segment_overdue()) {
		SecondDrive_EnterSeekCross(SEGMENT_OVERDUE);
		return first_drive_target_sps;
	}

	if (!is_fast_geometry(segment)
			|| !course_phase_allows_fast(segment, course_phase)
			|| abs_position_too_large()) {
		return scale(first_drive_target_sps);
	}

	distance = distance_to_next_left_right_or_end();
	next_target = target_for_restriction();
	braking = braking_steps(max(straight_sps, current_sps), next_target);

	if (distance > braking + margins) {
		return straight_sps;
	}
	return next_target;
}
```

---

## 16. 메모리와 성능 주의

현재 큰 정적 배열:

- events: 약 18,432 B
- segments: 약 4,096 B, 구조 확장 후 증가 예상

anchor 예산:

- 64개 x 약 16 B = 약 1KB

허용되는 증가량이지만 build 후 반드시 확인한다.

금지:

- events 복제 배열 추가
- 512개짜리 대형 profile을 근거 없이 추가
- malloc/free
- float/double 기반 곡률 계산
- 매 1ms 전체 512 event/segment 검색
- telemetry에 대형 배열 복제

가능하면 anchor scan은 최대 64개, 실제 CROSS 이벤트 순간에만 수행한다.

---

## 17. 실차 시험 매트릭스

### Test A - 정상 map sync

1. First Drive 정상 완주
2. map event/segment/anchor count 확인
3. Second Drive 기본값 5200 SPS / 100%
4. 끝까지 `MAP SYNC` 유지 확인

기대:

- 기존 완주 성능 유지
- END 정상 정지
- mismatch 0

### Test B - 동기화된 교차로 고속 직진

기대:

- next CROSS 때문에 2400 SPS로 미리 떨어지지 않음
- CROSS 중앙에서 가로 가지를 따라 조향하지 않음
- braking 여유가 충분하면 straight target 유지
- CROSS 이후에도 빠르게 straight target 유지

### Test C - 교차로 직후 가까운 코너

기대:

- CROSS가 fast geometry여도 다음 코너까지 누적거리로 판단
- 제동거리가 부족하면 교차로 전부터 감속
- 무조건 고속 CROSS를 강요하지 않음

### Test D - 회전마커 하나 누락

방법:

- 안전한 테스트 코스에서 한쪽 턴마크를 가리거나 debug injection 사용

기대:

```text
MAP SYNC -> SEEK CROSS
```

- false/missed event에서 index가 전진하지 않음
- SEEK 동안 First Drive envelope
- 다음 실제 CROSS에서 anchor match
- `resync_count++`
- `MAP SYNC` 복귀
- 이후 직선/CROSS 가속 재개

### Test E - false directional event

기대:

- SEEK 전환
- 잘못된 index 전진 없음
- 다음 CROSS에서 복구

### Test F - false CROSS 또는 모호한 anchor

기대:

- 거리 조건을 만족하지 않으면 anchor snap 금지
- 뒤로 rewind 금지
- SEEK 유지
- 안전 속도 유지

### Test G - CROSS 하나 누락

기대:

- 첫 후보를 강제로 적용하지 않음
- 다음 forward anchor가 거리 기준으로 유일하게 맞을 때만 복구
- 복구되지 않으면 First Drive envelope로 완주

### Test H - CROSS 없는 맵

기대:

- 정상 event sequence에서는 Second Drive 사용 가능
- mismatch 이후 복구 anchor가 없으므로 안전 속도로 끝까지 주행
- map 자체를 INVALID로 만들지 않음

### Test I - END/CROSS 회귀

기대:

- CROSS는 `MARKER_EVENT_CROSS`
- END는 `MARKER_EVENT_BOTH`
- CROSS에서 정지하지 않음
- END에서 정상 정지

### Test J - 안전정지 회귀

확인:

- line lost
- sensor stale
- control watchdog
- motor command fault
- center emergency stop
- edge stuck

모두 V26과 동일하게 동작해야 한다.

---

## 18. 빌드 및 정적 검증 체크리스트

```sh
cmake --build --preset Debug --clean-first -j 8
```

확인:

- 새 warning 없음
- `app_version.h`가 27
- RAM/Flash 출력 기록
- anchor array와 확장 segment array 크기 확인
- stack usage가 과도하게 증가하지 않음
- `SecondDrivePlanner_OnEvent()`에서 mismatch branch 후 index 변경 없음
- `SecondDrivePlanner_GetTargetSps()`가 SEEK에서 즉시 First Drive target 반환
- Track_ReplayReset이 anchors/events/segments를 지우지 않음
- FirstDrive_Init/Start만 새 map을 위해 Track_Reset 호출
- normal First Drive STOP 후 Second Drive 진입 시 map 유지

가능하면 ELF 확인:

```sh
arm-none-eabi-size build/Debug/2026_LINE_TRACER_STEP.elf
arm-none-eabi-nm -S --size-sort build/Debug/2026_LINE_TRACER_STEP.elf | tail
```

---

## 19. 회귀 금지사항

1. S0/S7을 위치 평균에 넣지 않는다.
2. CROSS보다 BOTH를 우선 분류하지 않는다.
3. 단순 `edge_union == 0x81`만으로 END 정지하지 않는다.
4. line lost 중 가짜 위치나 derivative kick을 만들지 않는다.
5. TURN에서 중앙으로 돌아왔다는 이유만으로 exit marker 없이 TURN을 끝내지 않는다.
6. 센서 없는 다리 복구 시간을 줄이지 않는다.
7. 마커 spill 제거를 삭제하지 않는다.
8. mismatch 이벤트에서 index를 전진시키지 않는다.
9. SEEK 중 map 기반 고속 예측을 사용하지 않는다.
10. CROSS를 전역적으로 항상 고속화하지 않는다. First Drive와 SEEK는 기존 안전 envelope를 사용한다.
11. 다음 코너까지 braking distance가 부족한데 CROSS라는 이유만으로 최고속을 유지하지 않는다.
12. First Drive 완료 map을 Second Drive 진입 전에 reset하지 않는다.
13. generated step을 실제 엔코더 거리로 표현하지 않는다.
14. UI를 제어 IRQ에서 직접 그리지 않는다.
15. `.ioc`, CubeMX 생성 설정, 모터 전류를 불필요하게 바꾸지 않는다.
16. `.DS_Store`와 사용자 백업 폴더를 수정하지 않는다.

---

## 20. 완료 조건

아래를 모두 만족해야 V27 구현 완료다.

- [ ] 버전 V27
- [ ] clean Debug build 성공
- [ ] First Drive 코드 경로와 안전 로직 보존
- [ ] First Drive 정상 완료 시 CROSS anchor table 생성
- [ ] LEFT/RIGHT segment curve unit 0-6 저장
- [ ] mismatch에서 index 전진하지 않음
- [ ] 영구 one-shot fallback 제거
- [ ] SEEK CROSS 상태 구현
- [ ] 실제 CROSS에서 forward anchor resync 구현
- [ ] resync 성공 후 MAP SYNC와 가속 복귀
- [ ] SYNC 상태에서 CROSS를 straight speed class로 처리
- [ ] CROSS를 넘어 다음 제한 segment까지 braking lookahead
- [ ] SEEK 상태에서 First Drive speed envelope
- [ ] UI에 sync state, mismatch reason, resync count 표시
- [ ] CROSS/END 분류 및 정지 회귀 없음
- [ ] RAM/Flash 증가량 보고
- [ ] 실차 시험 전 기본값 5200 SPS / 100% 유지

---

## 21. 최종 설계 원칙

V27에서 맵은 고속 주행을 위한 예측 도구이고 센서가 최종 판단 기준이다.

```text
일반 턴마커가 맞으면 맵 진행
일반 턴마커가 어긋나면 인덱스를 멈추고 SEEK CROSS
교차로가 나타나면 거리와 순서로 절대 위치 복구
동기화되면 고속 주행 재개
동기화되지 않으면 First Drive 수준으로 안전 완주
```

교차로는 감속용 코너가 아니다. 규정상 직진 경로이며, 맵 동기화가 확실하고 다음 코너까지 제동거리가 충분한 경우 직선 속도로 통과한다. 반대로 위치가 불확실하거나 다음 제한 구간이 가까우면 기존 안전속도를 우선한다.

