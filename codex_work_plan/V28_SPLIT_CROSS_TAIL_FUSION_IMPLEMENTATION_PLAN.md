# Line Tracer V28 - 분리형 교차로 꼬리 결합 및 종료선 오인 방지 구현 계획

> 대상 구현자: Codex 5.6 Luna Max  
> 기준 코드: 현재 작업 트리의 V27  
> 목표 버전: V28  
> 실제 프로젝트 경로: `/Users/ehoi/STM32CubeIDE/line_tracer_2026`  
> 작성 목적: 이전 대화 내용을 볼 수 없는 구현자가 이 문서와 현재 프로젝트만으로 V28을 안전하게 구현하도록 하는 자립형 명세

---

## 0. Luna에게 내리는 최우선 작업 지시

이 문서를 처음부터 끝까지 읽은 뒤 구현한다.

이번 V28의 단일 핵심 목표는 다음과 같다.

> 하나의 물리 교차로가 `CROSS`와 지연된 `BOTH` 두 이벤트로 분리되는 경우, 뒤쪽 S0/S7 이벤트를 같은 교차로의 꼬리로 결합하여 First Drive의 조기 종료와 잘못된 맵 생성을 막는다.

구현 우선순위:

1. V27의 CROSS 앵커 재동기화, 다중 braking lookahead, Second Drive 속도 설정 UI를 보존한다.
2. First Drive와 Second Drive replay가 동일한 CROSS-tail 판정을 사용하게 한다.
3. 실제 종료선인 독립 `BOTH`는 기존과 동일하게 확실히 정지시킨다.
4. tail 이벤트를 두 번째 CROSS로 발행하지 않는다. 공개 이벤트 수는 물리 마커 수와 일치해야 한다.
5. CROSS의 원래 `center_step`을 보존한다. tail 때문에 앵커 위치를 뒤로 이동시키지 않는다.
6. CROSS 꼬리에서 S0 또는 S7이 먼저 감지되어 가짜 회전 접근 상태로 전이되는 것도 막는다.
7. 임계값을 frame/time이 아니라 generated step으로 적용한다.
8. 실차 로그를 재현하는 합성 시퀀스와 독립 종료선 회귀시험을 모두 통과시킨다.
9. 앱 버전을 V28로 올리고 clean Debug build를 통과시킨다.

이번 작업은 마커 이벤트 상관관계 보강이다. 라인 추종 PD, 모터 구동, 센서 정규화, V27 planner를 임의로 재작성하지 않는다.

---

## 1. 현재 프로젝트와 작업 트리 상태

### 1.1 기준 경로와 버전

- 실제 프로젝트: `/Users/ehoi/STM32CubeIDE/line_tracer_2026`
- 현재 앱 버전: `Main/Inc/app_version.h`의 `APP_VERSION_NUMBER 27U`
- 목표 앱 버전: `28U`
- 기준 브랜치: `main`
- V27 변경은 아직 현재 작업 트리에 존재한다. `git reset`, `git checkout --`, `git restore`로 되돌리지 않는다.
- `.DS_Store`, build 산출물, 사용자 백업 파일을 정리하거나 삭제하지 않는다.

현재 수정 상태에서 V28을 이어서 구현해야 한다. Git HEAD의 V26 코드로 돌아가 구현하면 안 된다.

### 1.2 V27에서 이미 구현된 기능

다음 파일과 기능은 V28의 기준선이다.

- `track.h/c`
  - `TrackSegment_t.curve_units`
  - `TrackCrossAnchor_t`
  - 최대 64개 CROSS anchor
  - anchor overflow 검사
- `second_drive.h/c`
  - `SECOND_DRIVE_SYNC_MAP`
  - `SECOND_DRIVE_SYNC_SEEK_CROSS`
  - `SECOND_DRIVE_SYNC_INVALID`
  - CROSS anchor forward resync
  - STRAIGHT/CROSS fast geometry
  - 최대 16개 segment braking lookahead
- `menu.c`
  - Second Drive sync/mismatch/resync telemetry
- `app_version.h`
  - V27

V27 작업 결과 문서:

`codex_worked_review/V27_CROSS_ANCHOR_RESYNC_WORK_RESULT.md`

V28 구현 전 반드시 읽는다.

### 1.3 V27 빌드 기준

V27 clean build 결과:

```text
text: 113,520 B
data:     416 B
bss:   41,160 B
RAM 합계: 41,576 B / 272 KB = 14.93%
Flash 출력: 113,936 B / 512 KB = 21.73%
```

현재 환경에서 사용한 빌드 명령:

```sh
cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --clean-first -j 8
```

기존 경고 하나:

```text
Drivers/BSP/ST7735/st7735_lcd.c:50: unused variable 'text'
```

이번 작업과 무관하므로 별도 요청 없이 수정하지 않는다.

---

## 2. 실차에서 발생한 V27 실패

### 2.1 상황

- 실패 모드: First Drive
- 위치: 약 7~8번째 교차로 부근
- 증상: 교차로 통과 중 종료선으로 오인하여 정지
- 정지 상태: `STOP FAULT NONE`
- 정지 step: 약 `62420`

### 2.2 실차 로그

```text
STOP FAULT NONE
LAST P -84 L18 NOW M18 L18 Q00
PHAL V0/0 U+59/+58
LOSS 0/120 LR 0/0
S62420 B- C450 L/R LOG
```

```text
MKBT C60 N53 P- E81
0 CR>AL MPR S62405
1 ST>CR CMK S62340
2 CR>ST CTO S62080
```

```text
0 BT C60 E81 S62390
1 CR C40 E00 S62301
2 CR C100 E81 S61486
3 E0 C20 E01 S59779
4 E0 C20 E01 S59478
```

### 2.3 로그의 정확한 의미

정상 교차로:

```text
CR C100 E81 S61486
```

- 중앙 S1~S6 광폭 증거: +40
- S0 지속 증거: +20
- S7 지속 증거: +20
- S0/S7 동시 overlap: +20
- 하나의 CROSS 이벤트에 중앙과 뒤쪽 센서 증거가 모두 결합됨

실패 교차로:

```text
CR C40 E00 S62301
BT C60 E81 S62390
```

- 첫 이벤트는 중앙 광폭만 포함한 CROSS
- 두 이벤트 중심 사이 간격: `62390 - 62301 = 89 generated steps`
- 뒤쪽 S0/S7이 별도 BOTH로 저장됨
- BOTH는 종료 조건을 만족하여 First Drive가 정상 종료 경로로 정지함

추가 오탐:

```text
CR>AL MPR S62405
```

- 뒤쪽 S0/S7 중 한쪽이 먼저 보이는 순간 provisional directional marker가 작동함
- 실제 코스는 교차로 직진이지만 CROSS에서 APPROACH_LEFT로 바뀜

---

## 3. 현재 코드에서 문제가 발생하는 정확한 경로

### 3.1 실제 주행 이벤트 경로

실제 주행은 `marker.c`의 `MarkerDetector_Update()`가 아니라 다음 경로를 사용한다.

```text
FirstDrive_ProcessNewFrame()
  -> FirstDrive_ProcessMarker()
    -> Track_ProcessSensor()          First Drive
    -> Track_ProcessReplaySensor()    Second Drive
      -> Track_ClassifyPending()
```

따라서 `marker.c`만 수정하면 실제 주행 문제는 해결되지 않는다.

### 3.2 현재 이벤트 종료 조건

`Main/Inc/track.h`:

```c
#define TRACK_MARK_CLEAR_FRAMES      5U
#define TRACK_MARK_COOLDOWN_STEPS   50U
```

`Track_ProcessSensor()`와 replay collector는 광폭 중앙 또는 S0/S7이 없는 상태가 5프레임 지속되면 현재 pending event를 종료한다.

실패 시퀀스:

```text
S1~S6 광폭 3프레임 이상
  -> pending에 CROSS 증거

active가 아닌 상태 5프레임
  -> CROSS C40 E00 발행 및 저장

50 step cooldown 이후 S0/S7 도착
  -> 새로운 pending event
  -> BOTH C60 E81 발행 및 저장
  -> END 정지
```

### 3.3 현재 종료 조건

`drive.c`의 `MARKER_EVENT_BOTH` 처리:

```c
if ((event->center_step >= FIRST_DRIVE_START_MARKER_IGNORE_STEPS)
        && (event->both_overlap_run >= TRACK_MARKER_MIN_OVERLAP)
        && (event->max_center_count < MARKER_WIDE_CENTER_COUNT)
        && (event->wide_center_run < MARKER_WIDE_MIN_FRAMES)) {
    FirstDrive_StopAtEndMarker();
}
```

`BT C60 E81`은 위 조건을 모두 만족한다.

### 3.4 잘못된 맵도 구조적으로 유효해질 수 있음

`FirstDrive_StopAtEndMarker()`는 즉시 `Track_FinalizeSegments()`를 호출한다.

`Track_FinalizeSegments()`는 첫 `MARKER_EVENT_BOTH`를 `TRACK_SEGMENT_END`로 만들고 중단한다.

V27 구조 검사는 마지막 segment가 END이면 map valid로 볼 수 있다. 따라서 이번 잘못된 중간 정지가 다음과 같이 이어질 수 있다.

```text
교차로 tail BOTH
  -> First Drive 조기 STOP
  -> 중간 지점에서 END segment 생성
  -> 의미상 잘린 맵이 구조 검사 통과
  -> Second Drive가 잘못된 맵을 사용할 위험
```

V28은 단순히 모터 정지만 막는 것이 아니라 tail BOTH가 event 배열에 들어가는 것부터 막아야 한다.

---

## 4. V28 설계 원칙

### 4.1 물리 마커 하나는 공개 이벤트 하나

```text
하나의 교차로 = CROSS 이벤트 한 개
하나의 종료선 = BOTH 이벤트 한 개
```

금지:

```text
한 교차로 = CROSS + BOTH
한 교차로 = CROSS + CROSS
```

### 4.2 CROSS와 END의 구분 기준

확률값으로 구분하지 않는다. 센서 증거의 순서와 공간 거리를 사용한다.

교차로 tail:

```text
직전 공개 이벤트가 CROSS
+ 그 CROSS에 S0/S7 양쪽 증거 E81이 아직 없음
+ 새로운 이벤트가 짧은 step 거리 안에서 시작
+ 새로운 이벤트는 edge 증거를 가지지만 새로운 wide-center CROSS 증거는 없음
```

독립 종료선:

```text
최근 미완성 CROSS가 없음
또는
직전 CROSS가 이미 E81까지 포함해 완성됨
또는
새 이벤트 시작점이 CROSS-tail 거리 제한 밖임
```

### 4.3 step 기반 판정

초기 상수:

```c
#define TRACK_CROSS_TAIL_MAX_GAP_STEPS 160U
```

이 값은 `CROSS center -> tail center`가 아니라 다음 공간을 제한하는 값이다.

```text
candidate.entry_step - source_cross.exit_step
```

이유:

- 실차에서 center 차이는 89 step
- 5cm wheel, 400 half-step/rev 기준 160 step은 약 6.3cm
- 시립대 교차로 중심에서 다음 코스 요소까지의 직선 여유 20cm는 약 509 step
- 160 step은 센서 기판 종방향 간격을 수용하면서 별도 코스 요소보다 충분히 짧음

`FIRST_DRIVE_CROSS_PASS_STEPS 510U`를 tail 상수로 재사용하지 않는다. 510 step은 조향 phase 유지 거리이며 END 억제 범위로는 너무 크다.

### 4.4 원래 CROSS 위치 보존

tail을 결합해도 다음 필드는 최초 CROSS 값을 유지한다.

- `type = MARKER_EVENT_CROSS`
- `entry_step`
- `center_step`
- `entry_frame`

다음 필드는 tail 증거로 보강할 수 있다.

- `edge_union`
- `full_union`
- `edge0_run`
- `edge7_run`
- `both_overlap_run`
- `exit_step`
- `exit_frame`
- `confidence`

앵커 `center_step`을 tail까지 포함한 전체 midpoint로 다시 계산하지 않는다.

---

## 5. 권장 V28 수집기 구조

현재 First/Replay collector가 거의 같은 코드를 복제한다. V28에서 tail 로직을 각각 따로 구현하면 한쪽만 수정되는 회귀가 생길 가능성이 높다.

공통 runtime 구조와 하나의 processing helper를 권장한다.

### 5.1 처리 결과 enum

`track.h`에 추가:

```c
typedef enum {
    TRACK_PROCESS_NONE = 0,
    TRACK_PROCESS_EVENT_READY,
    TRACK_PROCESS_CROSS_TAIL_MERGED
} TrackProcessResult_t;
```

기존 bool 반환을 이 enum으로 변경한다.

```c
TrackProcessResult_t Track_ProcessSensor(...);
TrackProcessResult_t Track_ProcessReplaySensor(...);
```

의미:

- `NONE`: 아직 event 없음
- `EVENT_READY`: 공개할 새 event가 `GetLastEvent()`에 있음
- `CROSS_TAIL_MERGED`: tail을 기존 CROSS에 결합했으며 새 공개 event는 없음

`CROSS_TAIL_MERGED`에서 drive나 SecondDrivePlanner에 event를 전달하면 안 된다.

### 5.2 collector runtime

권장 내부 구조:

```c
typedef struct {
    TrackPendingEvent_t pending;
    TrackMarkerEvent_t last_event;
    uint32_t cooldown_until_step;

    bool cross_tail_pending;
    uint32_t cross_tail_source_exit_step;
    bool pending_started_in_cross_tail;
    uint16_t cross_tail_suppressed_count;
    uint32_t last_cross_tail_gap_steps;
    uint8_t last_cross_tail_edge_union;
} TrackCollectorRuntime_t;
```

`pending_started_in_cross_tail`은 새 pending이 시작되는 순간 고정한다. tail edge가 160 step 안에서 시작했지만 마커 폭과 clear frame 때문에 candidate 완성 시점이 160 step을 넘을 수 있기 때문이다.

정적 객체:

```c
static TrackCollectorRuntime_t first_collector;
static TrackCollectorRuntime_t replay_collector;
```

필요하면 기존 `pending`, `last_event`, `cooldown_until_step` 이름을 유지해도 되지만, First/Replay가 반드시 동일한 helper를 통과해야 한다.

### 5.3 공통 processing helper

권장 형태:

```c
static TrackProcessResult_t Track_ProcessInternal(
        TrackCollectorRuntime_t *runtime,
        uint8_t sensor_mask,
        uint32_t frame_number,
        uint32_t average_step,
        bool store_event);
```

First Drive:

```c
return Track_ProcessInternal(&first_collector, ..., true);
```

Second Drive replay:

```c
return Track_ProcessInternal(&replay_collector, ..., false);
```

기존 센서 누적, run count, wide count, clear frame 의미는 유지한다.

---

## 6. candidate event 완성 절차

현재 `Track_FinishPending()`은 candidate 생성, `last_event` 덮어쓰기, event 배열 저장을 한 번에 수행한다. tail 판정 전에 배열에 저장되므로 V28에서는 역할을 분리한다.

### 6.1 Build 단계

```c
static bool Track_BuildCompletedEvent(
        TrackPendingEvent_t *pending,
        TrackMarkerEvent_t *candidate);
```

역할:

- confirm frame 검사
- pending evidence를 candidate로 변환
- type/confidence 계산
- pending 초기화
- 아직 `events[]`에 저장하지 않음
- 아직 runtime `last_event`를 덮어쓰지 않음

### 6.2 Correlate 단계

candidate를 만든 다음 아래 순서로 처리한다.

```text
1. candidate가 기존 CROSS의 tail인지 검사
2. tail이면 기존 CROSS에 병합하고 새 이벤트를 발행하지 않음
3. tail이 아니면 정상 공개 이벤트로 저장/발행
4. candidate가 미완성 CROSS이면 새 tail 대기를 시작
5. 다른 독립 이벤트면 이전 tail 대기를 종료
```

새 pending을 시작할 때:

```c
runtime->pending_started_in_cross_tail =
        runtime->cross_tail_pending
        && (average_step >= runtime->cross_tail_source_exit_step)
        && ((average_step - runtime->cross_tail_source_exit_step)
                <= TRACK_CROSS_TAIL_MAX_GAP_STEPS);
```

pending을 clear/reset할 때 이 flag도 반드시 false로 만든다.

### 6.3 Publish 단계

정상 새 이벤트만:

```c
runtime->last_event = candidate;
if (store_event) {
    events[event_count++] = candidate;
}
return TRACK_PROCESS_EVENT_READY;
```

tail candidate는 `event_count`를 증가시키면 안 된다.

---

## 7. CROSS-tail 판정 상세

### 7.1 tail 대기 시작 조건

공개할 candidate가 CROSS일 때:

```c
runtime->cross_tail_pending =
        ((candidate.edge_union & MARKER_EDGE_MASK) != MARKER_EDGE_MASK);
runtime->cross_tail_source_exit_step = candidate.exit_step;
```

즉 `CR C100 E81`처럼 E81을 이미 포함한 CROSS는 tail 대기를 만들지 않는다.

```text
CR C100 E81 -> 완성 CROSS, tail pending false
CR C40 E00  -> 미완성 CROSS, tail pending true
CR C60 E01  -> 한쪽만 포함, tail pending true
```

### 7.2 tail candidate 조건

아래를 모두 만족해야 한다.

```c
runtime->cross_tail_pending
&& runtime->last_event.type == MARKER_EVENT_CROSS
&& runtime->pending_started_in_cross_tail
&& candidate.entry_step >= runtime->cross_tail_source_exit_step
&& (candidate.entry_step - runtime->cross_tail_source_exit_step)
        <= TRACK_CROSS_TAIL_MAX_GAP_STEPS
&& candidate.max_center_count < MARKER_WIDE_CENTER_COUNT
&& candidate.wide_center_run < MARKER_WIDE_MIN_FRAMES
&& (candidate.edge_union & MARKER_EDGE_MASK) != 0U
```

candidate type만 `EDGE_0/EDGE_7/BOTH`로 제한하기보다 실제 evidence를 검사한다. edge union은 있으나 overlap 부족으로 UNKNOWN이 된 짧은 꼬리도 중복 map event를 만들지 않게 하기 위함이다.

단, confirm frame 미만 노이즈는 기존대로 event가 되지 않는다.

### 7.3 거리 계산 안전성

unsigned underflow를 금지한다.

```c
if (candidate.entry_step < source_exit_step) {
    return false;
}
gap = candidate.entry_step - source_exit_step;
```

주행 중 generated step reset은 run 시작 시 한 번뿐이라는 기존 불변조건을 유지한다.

### 7.4 tail 결합

```c
static void Track_MergeCrossTail(
        TrackMarkerEvent_t *cross,
        const TrackMarkerEvent_t *tail);
```

병합 규칙:

```c
cross->edge_union |= tail->edge_union;
cross->full_union |= tail->full_union;
cross->max_center_count = max(cross->max_center_count,
        tail->max_center_count);
cross->edge0_run = max(cross->edge0_run, tail->edge0_run);
cross->edge7_run = max(cross->edge7_run, tail->edge7_run);
cross->both_overlap_run = max(cross->both_overlap_run,
        tail->both_overlap_run);
cross->wide_center_run = max(cross->wide_center_run,
        tail->wide_center_run);
cross->exit_frame = max(cross->exit_frame, tail->exit_frame);
cross->exit_step = max(cross->exit_step, tail->exit_step);
cross->confidence = Track_CalculateEventConfidence(cross);
```

절대 변경 금지:

```c
cross->type
cross->entry_frame
cross->entry_step
cross->center_step
```

First Drive에서 마지막 저장 event가 해당 CROSS인지 bounds와 type을 확인한 뒤 함께 갱신한다.

```c
if (store_event && event_count > 0U
        && events[event_count - 1U].type == MARKER_EVENT_CROSS
        && events[event_count - 1U].center_step
                == runtime->last_event.center_step) {
    events[event_count - 1U] = runtime->last_event;
}
```

조건이 맞지 않으면 배열을 추측으로 수정하지 않는다. 진단 오류/overflow 상태로 처리하거나 tail 결합을 포기하고 독립 이벤트로 처리한다.

### 7.5 tail 결합 결과

```text
runtime last_event: 보강된 CROSS
First Drive events[] 마지막 원소: 보강된 CROSS
event_count: 증가하지 않음
cooldown: tail.exit_step + 기존 cooldown으로 갱신
suppressed count: 포화 증가
last gap/edge 진단: 갱신
return TRACK_PROCESS_CROSS_TAIL_MERGED
```

merged CROSS가 E81을 완성했으면 tail pending을 종료한다.

```c
if ((runtime->last_event.edge_union & MARKER_EDGE_MASK)
        == MARKER_EDGE_MASK) {
    runtime->cross_tail_pending = false;
}
```

한쪽 edge만 결합됐다면 원래 `cross_tail_source_exit_step`을 유지한 채 window 안에서 반대쪽 tail을 한 번 더 받을 수 있다. tail을 결합할 때마다 window 시작을 뒤로 미는 sliding window를 만들지 않는다.

### 7.6 tail 대기 만료

매 processing call에서 step이 원래 source exit보다 제한을 넘으면 pending을 해제한다. 단, 이미 window 안에서 시작한 pending event가 수집 중이면 candidate가 완성될 때까지 source context를 유지한다.

```c
if (runtime->cross_tail_pending
        && !(runtime->pending.collecting
                && runtime->pending_started_in_cross_tail)
        && average_step >= runtime->cross_tail_source_exit_step
        && (average_step - runtime->cross_tail_source_exit_step)
                > TRACK_CROSS_TAIL_MAX_GAP_STEPS) {
    runtime->cross_tail_pending = false;
}
```

새로운 독립 wide-center CROSS가 나타나면 이전 pending을 종료하고 새 CROSS 기준으로 갱신한다.

금지되는 구현:

```text
tail entry는 150 step에서 시작
tail event는 175 step에서 완성
현재 step이 160을 넘었다는 이유로 tail context를 먼저 삭제
```

판정 기준은 candidate 완료시점이 아니라 candidate `entry_step`이다.

---

## 8. 종료선 판정 안전성

### 8.1 독립 BOTH는 유지

tail correlation을 통과하지 않은 `MARKER_EVENT_BOTH`는 기존 조건으로 drive에 전달한다.

```text
최근 미완성 CROSS 없음 + E81 overlap -> 정상 END
```

시작선 ignore 300 step도 유지한다.

### 8.2 방어적 이중 확인

Track layer가 tail을 억제하는 것이 주 방어선이다. `drive.c`에는 잘못된 tail BOTH가 통과했을 때의 짧은 방어 조건을 추가할 수 있다.

권장 runtime:

```c
static bool drive_cross_tail_guard_active;
static uint32_t drive_cross_tail_source_exit_step;
static uint32_t drive_cross_tail_until_step;
```

CROSS event 수신 시 E81이 없을 때만 활성화한다.

```c
if ((event->edge_union & MARKER_EDGE_MASK) != MARKER_EDGE_MASK) {
    drive_cross_tail_guard_active = true;
    drive_cross_tail_source_exit_step = event->exit_step;
    drive_cross_tail_until_step = event->exit_step
            + TRACK_CROSS_TAIL_MAX_GAP_STEPS;
} else {
    drive_cross_tail_guard_active = false;
}
```

BOTH stop 전에는 candidate의 `entry_step`이 guard window 안인지 확인한다. 단순히 현재 `course_phase == CROSS`만 검사하지 않는다.

이번 실차에서는 tail BOTH가 완성되기 전에 `CR>AL MPR`이 발생하여 phase가 이미 CROSS가 아니었기 때문이다.

Track layer가 정상 동작하면 이 방어 조건은 실제로 발동하지 않아야 한다. 발동 횟수는 진단 count로 남긴다.

### 8.3 실제 종료선 누락 방지

다음 조건에서는 guard를 적용하지 않는다.

- 직전 CROSS가 이미 E81을 포함함
- event entry가 160 step window 밖임
- event에 새 wide-center 증거가 있어 별도 CROSS임
- step 순서가 역전됨

`CROSS 이후 모든 BOTH를 510 step 동안 무시`하는 구현은 금지한다.

---

## 9. provisional 회전마커 억제

현재 provisional marker는 Track event가 완성되기 전에 S0 또는 S7 3프레임으로 동작한다.

실패 로그:

```text
CR>AL MPR S62405
```

따라서 Track layer에서 나중에 tail을 억제하는 것만으로는 phase 오탐을 막을 수 없다.

`FirstDrive_ProcessMarker()`에서 `left_marker_now/right_marker_now`를 평가하기 전에 dedicated cross-tail guard를 검사한다.

```c
bool suppress_provisional = FirstDrive_CrossTailGuardContainsStep(
        average_step);

if (suppress_provisional) {
    left_marker_now = false;
    right_marker_now = false;
    drive_marker_left_frames = 0U;
    drive_marker_right_frames = 0U;
} else {
    /* existing provisional marker logic */
}
```

주의:

- guard는 E81이 빠진 CROSS에서만 활성화한다.
- `FIRST_DRIVE_CROSS_PASS_STEPS`가 아니라 160 step tail 범위만 사용한다.
- First Drive와 Second Drive에서 공통 drive marker 경로를 사용하므로 둘 다 보호된다.
- CROSS case가 실행될 때 기존 provisional marker를 0으로 지우는 동작은 유지한다.
- run Init/Start/Reset에서 guard 상태를 반드시 초기화한다.

---

## 10. Track API 및 진단 telemetry

### 10.1 권장 진단 구조

`track.h`:

```c
typedef struct {
    uint16_t cross_tail_suppressed_count;
    uint32_t last_cross_tail_gap_steps;
    uint8_t last_cross_tail_edge_union;
    uint8_t cross_tail_pending;
} TrackCollectorDiagnostics_t;

void Track_GetCollectorDiagnostics(bool replay,
        TrackCollectorDiagnostics_t *diagnostics);
```

count는 최대값에서 포화시킨다.

### 10.2 drive telemetry 확장

`FirstDriveTelemetry_t` 권장 추가 필드:

```c
uint16_t cross_tail_suppressed_count;
uint32_t last_cross_tail_gap_steps;
uint8_t last_cross_tail_edge_union;
uint8_t cross_tail_guard_active;
uint16_t end_guard_reject_count;
```

Second Drive에서도 replay collector 진단을 표시할 수 있게 mode에 따라 올바른 runtime을 조회한다.

### 10.3 marker log 보강

현재 marker log는 type, confidence, edge union, center step만 저장한다. threshold 튜닝을 위해 다음 중 최소한 entry/exit step을 추가한다.

```c
uint32_t entry_step;
uint32_t exit_step;
uint8_t max_center_count;
uint16_t wide_center_run;
uint16_t both_overlap_run;
```

RAM 증가는 5개 log entry에 한정한다. 대형 진단 배열을 만들지 않는다.

TRACK_PROCESS_CROSS_TAIL_MERGED에서는 새 marker log row를 push하지 않는다. 기존 newest row가 같은 CROSS center인지 확인한 뒤 confidence, edge union, exit 증거만 갱신한다. UI에서도 물리 교차로 하나가 log 한 줄로 보여야 한다.

### 10.4 UI 표시

First Drive STOP/FAULT 진단 페이지에 다음 한 줄을 추가하거나 기존 페이지를 확장한다.

```text
TAIL N1 G39 E81 A0
```

의미:

- `N1`: 결합/억제된 tail 수
- `G39`: 마지막 `tail.entry - cross.exit` gap
- `E81`: tail edge union
- `A0/1`: 현재 guard active

marker event 상세 페이지에는 가능하면 다음을 표시한다.

```text
CR C100 E81 I62280 X62410
```

화면 폭 때문에 full step 대신 하위 자리 또는 별도 페이지를 사용해도 된다. 기존 100ms UI 갱신과 partial redraw 정책을 유지한다. 제어 IRQ에서 LCD를 그리지 않는다.

---

## 11. reset과 lifecycle

### `Track_Reset()`

다음 모두 초기화:

- First collector pending/last/cooldown
- First cross-tail pending/source/count/diagnostics
- event/segment/anchor 배열 및 count
- replay collector 전체 상태

### `Track_ReplayReset()`

다음만 초기화:

- Replay collector pending/last/cooldown
- Replay cross-tail pending/source/count/diagnostics

절대 초기화하면 안 되는 것:

- First Drive events
- segments
- cross anchors
- completed map

### Drive Init/Start

다음 상태를 Init과 실제 Start 양쪽에서 0으로 만든다.

- drive cross-tail guard active
- source/until step
- end guard reject count
- marker left/right provisional frame count

이전 run의 guard가 다음 run으로 누출되면 안 된다.

---

## 12. 파일별 변경 계획

### `Main/Inc/app_version.h`

- `APP_VERSION_NUMBER 27U` -> `28U`

### `Main/Inc/track.h`

- `TRACK_CROSS_TAIL_MAX_GAP_STEPS 160U`
- `TrackProcessResult_t`
- `TrackCollectorDiagnostics_t`
- Process API 반환형 변경
- diagnostics getter 추가

기존 V27 segment/anchor 구조는 유지한다.

### `Main/Src/track.c`

- First/Replay 중복 수집 코드를 공통 helper로 통합
- pending event build와 publish 분리
- CROSS-tail pending state 구현
- candidate entry/previous cross exit step 상관관계 판정
- tail evidence 병합
- First events[] 마지막 CROSS 동기 갱신
- tail candidate를 새 event로 발행하지 않음
- cooldown과 진단 count 갱신
- First/Replay reset 대칭성 보장
- 기존 finalize/anchor center semantics 유지

### `Main/Inc/drive.h`

- tail 및 end-guard telemetry 추가
- marker log entry/exit 증거 보강

### `Main/Src/drive.c`

- `TrackProcessResult_t` 처리
- `TRACK_PROCESS_CROSS_TAIL_MERGED`에서 planner/switch 호출 금지
- incomplete CROSS 수신 시 dedicated tail guard 시작
- tail guard 안의 provisional EDGE 억제
- BOTH stop 방어 조건
- Init/Start guard reset
- telemetry 갱신

보존 대상:

- CROSS pass 510 step
- First Drive CROSS 2400 SPS cap
- 중앙 광폭에서 last position/derivative 유지
- line-loss, bridge recovery, edge-stuck, watchdog
- marker spill filtering
- END의 독립 E81 overlap 조건

### `Main/Src/menu.c`

- First Drive 진단에 tail count/gap/edge/guard 표시
- 기존 phase/marker log 페이지 유지
- Second Drive UI의 V27 sync 화면을 훼손하지 않음

### `Main/Src/second_drive.c`, `Main/Inc/second_drive.h`

원칙적으로 변경하지 않는다.

replay collector가 tail event를 발행하지 않으면 planner는 자동으로 올바른 물리 event stream을 받는다. 컴파일 API 적응 외에 V27 sync/속도 로직을 재작성하지 않는다.

### `Main/Src/marker.c`, `Main/Inc/marker.h`

현재 MARK DIAGNOSTIC이 `#if 0`이고 실제 drive 경로가 아니다. 실제 수정은 `track.c`에 한다.

진단기를 재활성화하지 않는다면 불필요하게 수정하지 않는다. 나중에 진단기를 사용할 경우에만 동일 semantics로 맞춘다.

### 수정 금지

- `sensor.c/h`
- `motor.c/h`
- CubeMX 생성 파일
- `.ioc`
- linker script
- 모터 전류/DAC
- 사용자 `.DS_Store` 및 백업

---

## 13. 상세 의사코드

### 13.1 candidate 처리

```c
static TrackProcessResult_t Track_CompleteCandidate(
        TrackCollectorRuntime_t *runtime,
        TrackMarkerEvent_t *candidate,
        bool store_event)
{
    if (Track_IsCrossTailCandidate(runtime, candidate)) {
        uint32_t gap = candidate->entry_step
                - runtime->cross_tail_source_exit_step;

        Track_MergeCrossTail(&runtime->last_event, candidate);
        if (store_event) {
            if (!Track_UpdateLastStoredCross(&runtime->last_event)) {
                /* do not corrupt an unknown event index */
                Track_SetOverflowOrDiagnosticFailure();
                return TRACK_PROCESS_NONE;
            }
        }

        runtime->last_cross_tail_gap_steps = gap;
        runtime->last_cross_tail_edge_union = candidate->edge_union;
        SaturatingIncrement(&runtime->cross_tail_suppressed_count);
        runtime->cooldown_until_step = candidate->exit_step
                + TRACK_MARK_COOLDOWN_STEPS;

        if ((runtime->last_event.edge_union & MARKER_EDGE_MASK)
                == MARKER_EDGE_MASK) {
            runtime->cross_tail_pending = false;
        }
        return TRACK_PROCESS_CROSS_TAIL_MERGED;
    }

    runtime->cross_tail_pending = false;
    runtime->last_event = *candidate;
    if (store_event) {
        Track_StoreEvent(candidate);
    }
    runtime->cooldown_until_step = candidate->exit_step
            + TRACK_MARK_COOLDOWN_STEPS;

    if ((candidate->type == MARKER_EVENT_CROSS)
            && ((candidate->edge_union & MARKER_EDGE_MASK)
                    != MARKER_EDGE_MASK)) {
        runtime->cross_tail_pending = true;
        runtime->cross_tail_source_exit_step = candidate->exit_step;
    }
    return TRACK_PROCESS_EVENT_READY;
}
```

주의: 독립 candidate를 publish할 때 이전 tail pending을 먼저 종료한 후, candidate 자체가 incomplete CROSS이면 새 pending을 시작한다.

### 13.2 drive marker 처리

```c
TrackProcessResult_t result;

update_cross_tail_guard_expiry(average_step);
suppress_provisional_if_cross_tail_guard(average_step);

result = (drive_run_mode == DRIVE_RUN_FIRST)
        ? Track_ProcessSensor(...)
        : Track_ProcessReplaySensor(...);

update_tail_telemetry();

if (result == TRACK_PROCESS_NONE) {
    return;
}
if (result == TRACK_PROCESS_CROSS_TAIL_MERGED) {
    event = get_last_event_for_mode();
    refresh_newest_marker_log_without_push(event);
    update_last_marker_telemetry(event);
    /* no event count increment, no planner call, no marker switch */
    return;
}

event = get_last_event_for_mode();
record_marker_event(event);

if (drive_run_mode == DRIVE_RUN_SECOND) {
    SecondDrivePlanner_OnEvent(event);
}

switch (event->type) {
case MARKER_EVENT_CROSS:
    start_cross_phase();
    configure_short_tail_guard_if_incomplete(event);
    break;

case MARKER_EVENT_BOTH:
    if (is_defensive_cross_tail(event)) {
        saturating_increment(end_guard_reject_count);
        break;
    }
    if (existing_end_conditions(event)) {
        FirstDrive_StopAtEndMarker();
    }
    break;
}
```

### 13.3 defensive guard 판정

```c
static bool FirstDrive_IsDefensiveCrossTail(
        const TrackMarkerEvent_t *event)
{
    uint32_t gap;

    if (!drive_cross_tail_guard_active || event == NULL
            || event->entry_step < drive_cross_tail_source_exit_step) {
        return false;
    }
    gap = event->entry_step - drive_cross_tail_source_exit_step;
    return (gap <= TRACK_CROSS_TAIL_MAX_GAP_STEPS)
            && (event->wide_center_run < MARKER_WIDE_MIN_FRAMES)
            && ((event->edge_union & MARKER_EDGE_MASK) != 0U);
}
```

Track layer가 이 event를 이미 억제해야 정상이다. 이 함수가 실행되면 진단에 남겨 collector defect를 찾을 수 있게 한다.

---

## 14. 피해야 할 잘못된 해결책

### 14.1 `TRACK_MARK_CLEAR_FRAMES` 전역 증가

금지 이유:

- 방향마커 event 확정이 늦어짐
- 속도에 따라 같은 frame 수의 공간 거리가 달라짐
- 가까운 별도 마커가 하나로 합쳐질 수 있음
- provisional과 completed event timing 차이가 커짐

기존 5프레임은 유지하고 완성된 event 사이를 step으로 correlate한다.

### 14.2 BOTH confidence 임계값 상향

`BT C60 E81`은 실제 종료선과 동일한 강한 bilateral evidence를 가질 수 있다. C80 또는 C100만 END로 허용하면 실제 종료선을 놓칠 수 있다.

확률/점수가 아니라 선행 CROSS와 거리로 구분한다.

### 14.3 CROSS phase 동안 모든 BOTH 무시

이번 로그에서 BOTH 완성 전에 `CR>AL MPR`로 phase가 변경됐다. phase만으로는 충분하지 않다.

또한 510-step CROSS phase 전체에서 BOTH를 무시하면 실제 코스 요소를 과도하게 숨길 수 있다.

### 14.4 tail BOTH를 두 번째 CROSS로 바꿈

공개 CROSS가 두 개가 되어 다음 문제가 생긴다.

- First Drive anchor 2개
- event/segment index 증가
- Second Drive expected event mismatch
- anchor 거리 왜곡

tail은 기존 CROSS에 병합하고 새 공개 이벤트는 만들지 않는다.

### 14.5 모터 정지만 막음

drive.c에서 stop만 막아도 `events[]`에는 false BOTH가 남는다. finalize가 중간 END에서 끊기므로 맵이 잘린다.

반드시 Track event storage 단계에서 억제한다.

### 14.6 `marker.c`만 수정

현재 실제 주행은 Track collector를 사용한다. `marker.c`만 수정하면 실차 동작은 바뀌지 않는다.

---

## 15. 잠재 위험과 대응

### 실제 종료선이 CROSS tail로 억제될 위험

대응:

- 직전 CROSS가 E81 미완성일 때만 pending
- candidate entry와 CROSS exit 사이 160 step 제한
- 새 wide-center evidence가 없는 edge-only candidate만 tail
- 시립대 교차로 주변 20cm 직선 조건보다 훨씬 짧은 범위 사용
- 독립 END 합성시험 필수

### 실제 회전마커가 억제될 위험

대응:

- 160 step은 약 6.3cm로 코스 요소 최소 여유보다 짧음
- tail pending이 없는 일반 CROSS 이후에는 억제하지 않음
- window가 sliding하지 않도록 최초 cross exit 기준 고정

### CROSS anchor 위치 왜곡

대응:

- center/entry 고정
- exit와 evidence만 보강
- anchor는 기존 center 사용

### First와 Replay 동작 차이

대응:

- 하나의 `Track_ProcessInternal()` 사용
- runtime만 분리
- 동일 테스트 벡터를 First/Replay 양쪽에 실행

### tail event가 cooldown 중 일부만 보이는 문제

대응:

- candidate type 문자열보다 실제 edge evidence 사용
- UNKNOWN이라도 edge union과 거리 조건이 맞으면 tail로 결합
- confirm frame 미만 노이즈는 기존대로 무시

### threshold 과적합

대응:

- UI에 `entry - exit` gap 표시
- 첫 실차에서 여러 교차로의 최대 gap 확인
- 160은 초기값이며 근거 없이 510까지 확대하지 않음
- 조정 시 기록된 최대값 + 합리적 margin으로 변경

### step overflow/역행

대응:

- subtraction 전 순서 비교
- run 중 step reset 금지
- wrap-safe helper 또는 명시적 bounds 사용

---

## 16. 구현 순서

### 1단계: 현재 기준 보호

1. `git diff --check`
2. V27 변경 파일 목록 기록
3. 기존 V27 build 가능 여부 확인
4. 사용자 변경을 reset하지 않음

### 2단계: Track result/runtime refactor

1. `TrackProcessResult_t` 추가
2. First/Replay runtime 구조 추가
3. 중복 processing을 공통 helper로 이동
4. 아직 tail 로직 없이 기존 결과와 동일한지 build

### 3단계: candidate build/publish 분리

1. pending -> candidate helper
2. candidate가 storage 전에 검사되게 변경
3. 정상 event count/order가 기존과 같은지 synthetic test
4. build

### 4단계: CROSS-tail correlation

1. 160-step 상수
2. incomplete CROSS pending
3. entry-exit gap 검사
4. evidence merge
5. First events[] 마지막 CROSS 갱신
6. replay suppression
7. diagnostics
8. synthetic tests

### 5단계: drive guard

1. incomplete CROSS에서 guard 시작
2. provisional marker 억제
3. Track result enum 처리
4. BOTH defensive guard
5. Init/Start reset
6. synthetic/static review

### 6단계: UI와 버전

1. tail count/gap/edge/guard 표시
2. marker log 증거 보강
3. `APP_VERSION_NUMBER 28U`
4. clean build

### 7단계: 최종 회귀 검토

1. First map lifecycle
2. Replay lifecycle
3. V27 sync state
4. CROSS anchor center
5. END stop
6. line-loss/watchdog/emergency stop
7. RAM/Flash

---

## 17. 필수 합성 테스트 벡터

실제 보드가 없어도 collector 입력 sequence로 아래를 검증한다. 테스트 harness가 없다면 `/private/tmp`에 임시 host harness를 만들고 production repository에 대형 테스트 프레임워크를 추가하지 않는다.

### Test A - 기존 정상 결합 CROSS

입력:

```text
wide center >= 3 frames
S0/S7 overlap before event clear
quiet >= 5 frames
```

예상:

```text
EVENT_READY 1회
type CROSS
edge_union E81
confidence 100
tail suppressed 0
```

### Test B - 실차 실패 재현

입력:

```text
wide center >= 3 frames
quiet >= 5 frames
CROSS 발행: C40 E00
50-step cooldown 이후
source cross exit에서 160 step 이내 S0/S7 overlap
quiet >= 5 frames
```

예상:

```text
첫 결과 EVENT_READY CROSS
두 번째 결과 CROSS_TAIL_MERGED
event_count 증가 없음
last stored event type CROSS
stored edge_union E81
stored center_step 최초 CROSS와 동일
tail suppressed count 1
BOTH 공개 없음
```

### Test C - 한쪽 tail이 먼저 도착

입력:

```text
CR C40 E00
window 안 EDGE_0
window 안 EDGE_7
```

예상:

```text
두 tail 모두 새 공개 event 없음
최종 CROSS edge_union E81
원래 source exit 기준 window 유지
provisional AL/AR 전이 없음
```

### Test D - 독립 종료선

입력:

```text
최근 incomplete CROSS 없음
S0/S7 overlap >= 2
wide center 없음
```

예상:

```text
EVENT_READY BOTH
기존 END 조건 통과
FirstDrive_StopAtEndMarker 호출 가능
```

### Test E - CROSS 뒤 실제 END가 window 밖

입력:

```text
CR C40 E00
source exit + 161 step 이후 E81 BOTH
```

예상:

```text
tail pending 만료
BOTH 공개
END 정지 가능
```

### Test F - 이미 완성된 CROSS 뒤 BOTH

입력:

```text
CR C100 E81
그 뒤 별도 BOTH
```

예상:

```text
첫 CROSS는 tail pending false
후속 BOTH를 거리만으로 억제하지 않음
```

### Test G - 새 wide-center CROSS

입력:

```text
incomplete CROSS pending
window 안 또는 경계에 새로운 wide-center sequence
```

예상:

```text
edge-only tail로 병합하지 않음
새 물리 CROSS로 정상 분류
```

### Test H - UNKNOWN edge tail

입력:

```text
incomplete CROSS
window 안 edge union은 있으나 BOTH overlap 부족
confirm frame은 충족
```

예상:

```text
새 UNKNOWN map event로 발행하지 않음
CROSS tail evidence로 결합 또는 억제
```

### Test I - First/Replay 대칭

Test A~H를 First와 replay API 양쪽에 적용한다.

예상:

- 공개 event type/order 동일
- replay는 First event 배열을 수정하지 않음
- SecondDrivePlanner에는 tail event가 전달되지 않음

### Test J - map finalize

입력:

```text
정상 markers
split CROSS sequence
추가 course markers
실제 final BOTH
```

예상:

- split CROSS 위치에 CROSS segment/anchor 1개
- false END 없음
- 실제 final BOTH에서 END 1개
- map structurally valid
- anchor center는 최초 CROSS center

---

## 18. 필수 실차 시험

### Run 1 - 문제가 발생한 속도의 First Drive

확인:

- 7~8번째 교차로 통과
- 조기 STOP 없음
- `TAIL N` 증가 여부
- `G` 값 기록
- `CR>AL/AR MPR` 없음
- 최종 종료선에서 정상 STOP

### Run 2 - 낮은 속도 First Drive

속도가 낮으면 frame 간 공간 거리가 줄어 중앙과 rear gap의 frame 수가 늘 수 있다. step 판정이 동일하게 동작하는지 확인한다.

### Run 3 - V28 map 기반 Second Drive

확인:

- map anchor count가 실제 교차로 수와 일치
- replay에서 동일 tail suppression
- tail로 인한 SEEK CROSS 없음
- 실제 END에서 정상 정지
- V27 직선/CROSS 가속 유지

### Run 4 - 종료선 단독 재현

최근 incomplete CROSS가 없는 상태의 bilateral finish marker에서 반드시 정지한다.

### Run 5 - CROSS 직후 코스 요소

규정상 가장 가까운 허용 위치의 방향/종료 마커가 tail window 밖에서 정상 검출되는지 확인한다.

---

## 19. 빌드 및 정적 검증 체크리스트

```sh
git diff --check
cmake --build build/Debug --clean-first -j 8
arm-none-eabi-size build/Debug/2026_LINE_TRACER_STEP.elf
arm-none-eabi-nm -S --size-sort build/Debug/2026_LINE_TRACER_STEP.elf | tail
```

확인 항목:

- 새 warning 없음
- RAM/Flash 증가량 보고
- malloc/free 없음
- float/double 없음
- control tick에서 대형 scan 없음
- array index bounds
- uint32 subtraction 전 순서 검사
- count saturation
- First/Replay reset 대칭
- tail merge에서 event_count 증가 없음
- First events[] 마지막 원소 외 임의 수정 없음
- Second Drive planner에 tail event 전달 없음
- V27 source 변경 보존

---

## 20. 회귀 금지사항

1. `TRACK_MARK_CLEAR_FRAMES`를 근거 없이 늘리지 않는다.
2. `TRACK_MARK_COOLDOWN_STEPS`를 문제 해결의 주 수단으로 늘리지 않는다.
3. CROSS 이후 모든 BOTH를 무조건 무시하지 않는다.
4. 510-step CROSS PASS 전체를 END suppression window로 사용하지 않는다.
5. BOTH confidence threshold만 올리지 않는다.
6. tail을 두 번째 CROSS로 발행하지 않는다.
7. tail을 events[]에 저장한 뒤 drive stop만 막지 않는다.
8. tail 병합 후 CROSS center/entry를 다시 계산하지 않는다.
9. tail 결합 시 source window를 뒤로 계속 연장하지 않는다.
10. First만 고치고 replay를 남기지 않는다.
11. replay가 First map 배열을 수정하지 않게 한다.
12. provisional marker 억제를 course phase 하나에만 의존하지 않는다.
13. 독립 BOTH의 기존 종료 조건을 삭제하지 않는다.
14. 시작선 300-step ignore를 삭제하지 않는다.
15. S0/S7을 line position 평균에 넣지 않는다.
16. CROSS wide center의 직진 조향 억제를 삭제하지 않는다.
17. V27 sync/anchor/lookahead를 되돌리지 않는다.
18. First Drive map을 Second Drive 전에 reset하지 않는다.
19. line-loss, bridge recovery, edge-stuck, watchdog 시간을 변경하지 않는다.
20. UI를 control IRQ에서 그리지 않는다.
21. `.ioc`, CubeMX 생성 파일, 모터 DAC/current를 수정하지 않는다.
22. 사용자 파일과 기존 build 산출물을 파괴적으로 정리하지 않는다.

---

## 21. 완료 조건

아래를 모두 만족해야 V28 구현 완료다.

- [ ] 앱 버전 V28
- [ ] V27 변경사항 보존
- [ ] clean Debug build 성공
- [ ] 새 compiler warning 없음
- [ ] First/Replay 공통 collector semantics
- [ ] split CROSS 실차 로그 합성 재현 성공
- [ ] `CR C40 E00 + tail E81`이 CROSS 하나로 유지
- [ ] tail에서 event_count 증가 없음
- [ ] tail BOTH가 END로 공개되지 않음
- [ ] tail EDGE가 provisional turn을 만들지 않음
- [ ] CROSS center/anchor 위치 보존
- [ ] 독립 BOTH가 정상 END로 공개됨
- [ ] 실제 종료선 정지 회귀 없음
- [ ] false mid-track END map 없음
- [ ] map final END 1개 및 구조 유효
- [ ] SecondDrivePlanner에 tail event 미전달
- [ ] V27 MAP/SEEK/RESYNC 기능 유지
- [ ] V27 CROSS 고속 및 braking lookahead 유지
- [ ] UI에 suppression count와 실제 gap 표시
- [ ] RAM/Flash 증가량 보고
- [ ] First Drive 실차 완주
- [ ] V28 map 기반 Second Drive 실차 완주

---

## 22. 구현 결과 문서 지시

구현을 마친 Luna는 다음 경로에 결과를 작성한다.

```text
codex_worked_review/V28_SPLIT_CROSS_TAIL_FUSION_WORK_RESULT.md
```

반드시 포함할 내용:

1. 실제 변경 파일
2. 최종 상수값과 선택 근거
3. First/Replay 공통화 방식
4. tail 판정 조건
5. center/entry 보존 여부
6. synthetic test 입력과 결과
7. build 명령과 text/data/bss
8. 새 warning 여부
9. 실차에서 확인해야 할 UI 필드
10. 계획과 다르게 구현한 부분 및 이유

---

## 23. 최종 설계 요약

V28의 핵심은 BOTH 판정을 약하게 만드는 것이 아니다.

```text
중앙 광폭을 본 적이 없는 독립 E81
  -> 종료 BOTH

E81이 빠진 CROSS 직후 160 step 안의 edge evidence
  -> 같은 교차로의 rear tail
  -> 기존 CROSS에 결합
  -> 새 event 없음

E81까지 이미 포함한 CROSS 뒤의 독립 BOTH
  -> 종료 후보 유지
```

이 설계는 실제 종료선의 강한 bilateral evidence를 그대로 보존하면서, 센서 기판의 앞뒤 위치 차이 때문에 하나의 교차로가 두 이벤트로 갈라지는 경우만 공간적으로 결합한다.

최종적으로 First Drive map에는 물리 교차로당 CROSS anchor가 정확히 하나 저장되고, Second Drive replay에서도 같은 tail이 가짜 END나 sync mismatch를 만들지 않아야 한다.
