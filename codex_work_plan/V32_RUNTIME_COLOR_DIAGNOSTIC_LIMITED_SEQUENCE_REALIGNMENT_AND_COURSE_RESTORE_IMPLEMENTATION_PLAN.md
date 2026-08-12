# V32 Runtime Color Diagnostic·Limited Sequence Realignment·Course Restore 구현 계획

## 0. 문서 목적

이 문서는 `/Users/ehoi/STM32CubeIDE/line_tracer_2026` 프로젝트의 V31 실차 결과를 기반으로 V32를 구현할 Luna Max용 단독 인수인계 문서다. Luna는 사용자와 Codex가 나눈 이전 대화를 볼 수 없다고 가정한다.

V31 실차 주행은 First Drive와 Second Drive 모두 완료되었다. Second Drive 영상에서는 다음 현상이 확인되었다.

```text
출발 후 정상 가속
-> 주행 중 갑자기 정속에 가까운 속도로 전환
-> 이후 물리적 직선에 들어가도 같은 정속 유지
-> 다음 CROSS 부근에서 다시 가속
```

현재 소스와 이 현상을 함께 보면 가장 가능성이 높은 흐름은 다음과 같다.

```text
방향 marker 하나를 놓치거나 replay event가 map과 불일치
-> SECOND_DRIVE_SYNC_SEEK_CROSS
-> Second Drive performance target 대신 First Drive fallback target 사용
-> 다음 CROSS anchor에서 map 재동기화
-> fast gate 재진입 후 5600 SPS profile 허용
```

V32의 목표는 최고속도를 추가로 올리는 것이 아니다. V32는 아래 세 가지를 구현한다.

1. Second Drive 주행 중 map 신뢰 상태를 영상으로 판독할 수 있는 전체 화면 색상 진단.
2. marker 하나 누락 또는 marker 하나 오탐만 처리하는 제한된 sequence realignment.
3. realignment 성공 시 planner map 위치뿐 아니라 replay marker pair와 공통 drive course phase까지 함께 복원.

우선순위는 반드시 다음 순서를 따른다.

1. P0: 기존 정상 MAP replay, CROSS anchor resync, END active brake를 회귀시키지 않는다.
2. P0: 한 번의 mismatch로 임의의 먼 map 위치에 붙지 못하도록 realignment 범위를 한 edit로 제한한다.
3. P0: planner index와 course phase가 서로 다른 위치를 가리키는 partial recovery를 금지한다.
4. P0: protected marker인 CROSS와 final BOTH/END를 놓쳤다고 추론하지 않는다.
5. P1: `SUSPECT -> PROBATION -> MAP` 단계와 CROSS 즉시 복구를 구현한다.
6. P1: 주행 중 LCD는 상태 전환 시에만 전체 화면을 한 번 갱신한다.
7. P1: 실차와 영상에서 realignment 경로를 구분할 최소 진단을 남긴다.

> V32에서 straight/curve/overall 기본 속도, First Drive curve 기본값, recovery 1400/1800 SPS, END 1800 SPS fallback, active brake timing을 변경하지 않는다. V32의 목적은 속도 상향이 아니라 잘못된 장시간 fallback을 줄이는 것이다.

> 현재 worktree에는 V27~V31 소스, 문서, 테스트와 build 산출물이 함께 있으며 깨끗하지 않다. `git reset`, `git restore`, `git checkout --`, 광범위 삭제를 사용하지 않는다. 사용자 변경을 보존하고 현재 V31 소스 위에 V32 변경만 추가한다.

---

## 1. 반드시 먼저 읽을 자료

구현 전에 아래 문서를 순서대로 읽는다.

1. `codex_worked_review/V31_END_SAFETY_SPEED_TUNING_AND_DIAGNOSTIC_TRUST_WORK_RESULT.md`
2. `codex_work_plan/V31_END_SAFETY_SPEED_TUNING_AND_DIAGNOSTIC_TRUST_IMPLEMENTATION_PLAN.md`
3. `codex_worked_review/V30_SECOND_DRIVE_SPEED_CORRIDOR_HARD_BRAKE_AND_LCD_FREEZE_WORK_RESULT.md`
4. `codex_worked_review/V29_FIRST_DRIVE_RUN_RECORD_AND_LOG_UI_WORK_RESULT.md`
5. `codex_worked_review/V28_SPLIT_CROSS_TAIL_FUSION_WORK_RESULT.md`
6. `codex_worked_review/V27_CROSS_ANCHOR_RESYNC_WORK_RESULT.md`

핵심 소스:

- `Main/Src/second_drive.c`, `Main/Inc/second_drive.h`
- `Main/Src/drive.c`, `Main/Inc/drive.h`
- `Main/Src/track.c`, `Main/Inc/track.h`
- `Main/Src/menu.c`, `Main/Inc/menu.h`
- `Main/Src/motor.c`, `Main/Inc/motor.h`
- `Main/Inc/app_version.h`
- `tests/v31/`

V31 현재 host regression은 다음 명령으로 통과한다.

```sh
sh tests/v31/run_host_tests.sh
```

확인된 출력:

```text
V31 planner/diagnostic harness PASS
V31 track pair harness PASS
V31 motor brake mock PASS
V31 curve contract harness PASS
V31 candidate harness PASS
V31 source marker/version checks PASS
```

V32 완료 시 현재 소스에 맞는 `tests/v32/run_host_tests.sh`를 별도로 제공해야 한다. V31 script의 `APP_VERSION_NUMBER 31U` source check를 그대로 호출하면 V32에서 실패하므로, V32 script는 기존 harness의 의미 있는 회귀를 재사용하되 현재 version check는 32로 분리한다.

---

## 2. 현재 프로젝트 구조와 실행 흐름

### 2.1 First Drive map에 이미 저장되는 정보

`TrackMarkerEvent_t`에는 다음 정보가 저장된다.

- marker type: `EDGE_0`, `EDGE_7`, `CROSS`, `BOTH`
- `entry_step`, `exit_step`, `center_step`
- `confidence`
- edge/center/overlap evidence

`TrackSegment_t`에는 다음 정보가 저장된다.

- segment type: `STRAIGHT`, `LEFT`, `RIGHT`, `CROSS`, `END`
- 다음 marker까지의 `distance_steps`
- 일정 곡률 curve 단위 정보

`center_step`은 시간 tick이 아니라 좌·우 모터에 명령된 half-step count의 평균이다.

```c
(motor_l_step_count + motor_r_step_count) / 2U
```

marker의 중심은 다음과 같이 계산된다.

```c
entry_step + (exit_step - entry_step) / 2U
```

따라서 V32 sequence realignment에 필요한 marker 순서와 step 거리 정보는 이미 map에 있다. V32는 간격 전용 map을 새로 만들지 않는다.

### 2.2 Second Drive 이벤트 처리 순서

현재 `drive.c`의 marker 흐름은 대략 다음과 같다.

```text
raw sensor frame
-> provisional S0/S7 direction detection
-> Track_ProcessReplaySensor()
-> finalized TrackMarkerEvent_t
-> SecondDrivePlanner_OnEvent(event)
-> common drive course phase event processing
-> target calculation
-> motor command
```

중요한 점은 provisional marker가 finalized event보다 먼저 `drive_course_phase`를 변경할 수 있다는 것이다. 따라서 V32 realignment가 planner index만 수정하면 안 된다. false event 또는 skipped event를 판정한 시점에는 공통 course phase가 이미 잘못 바뀌었을 수 있다.

### 2.3 현재 planner sync 정책

현재 sync state는 다음 세 가지다.

```text
MAP
SEEK_CROSS
INVALID
```

`MAP`에서 event type 또는 distance가 기대값과 맞지 않으면 대부분 `SEEK_CROSS`로 전환한다. `SEEK_CROSS`에서는 non-CROSS event를 무시하고, forward CROSS anchor만 `SecondDrive_TryResyncAtCross()`에서 검사한다.

`SecondDrivePlanner_GetTargetSps()`는 map이 없거나 `sync_state != MAP`이면 `first_drive_target_sps`를 반환한다. 중앙 직선에서는 대략 First Drive 3820 SPS profile이므로 영상에서 정속 직선처럼 보일 수 있다.

### 2.4 현재 CROSS anchor 복구

CROSS 복구는 다음 조건을 사용한다.

- 현재 anchor보다 전진한 anchor만 탐색
- expected event 이전 anchor 금지
- 최대 세 개 forward anchor 후보
- 마지막 CROSS 또는 시작점부터의 step 거리 비교
- best와 second-best가 너무 가까우면 ambiguous reject

복구 성공 시 다음을 수행한다.

- expected event index 갱신
- segment index/start step 갱신
- anchor order 갱신
- sync를 MAP으로 복귀
- replay pair를 closed로 reset

이 경로는 V32에서도 가장 강한 복구 경로로 유지한다.

### 2.5 현재 course phase와 marker pair

공통 drive course phase:

```text
STRAIGHT
APPROACH_LEFT / APPROACH_RIGHT
TURN_LEFT / TURN_RIGHT
EXIT_LEFT / EXIT_RIGHT
CROSS
```

directional marker pair 규칙:

```text
pair closed + RIGHT -> RIGHT curve entry, pair open
pair open RIGHT + RIGHT -> RIGHT curve close, pair closed
pair open RIGHT + LEFT -> LEFT curve entry로 교체, pair open LEFT
CROSS -> pair closed reset
```

`TURN_LEFT/RIGHT`는 라인이 중앙에 돌아왔다는 이유만으로 종료되지 않는다. 같은 방향의 close marker가 필요하다. 이 정책은 일정 곡률의 긴 코너에서 중앙 추종을 코너 종료로 오판하지 않기 위해 유지해야 한다.

---

## 3. V32 범위와 비범위

### 3.1 구현 범위

- Second Drive sync state에 `SUSPECT`, `PROBATION` 추가.
- 한 개 map marker 누락 가설.
- 한 개 live false-positive marker 삽입 가설.
- marker type sequence와 step distance를 결합한 local scoring.
- last trusted checkpoint 유지.
- realignment commit 시 map index, segment, pair state, course phase restore.
- probation 중 제한 속도와 다음 marker confirmation.
- 모든 intermediate state에서 CROSS anchor 즉시 복구.
- active run LCD full-screen state color.
- realignment count/reason/step/error 최소 진단.
- host unit/integration regression.

### 3.2 명시적 비범위

아래 항목은 V32에서 구현하지 않는다.

- step 간격만으로 전체 map을 검색하는 복구.
- 임의의 marker index로 점프하는 global matching.
- 두 개 이상 marker를 건너뛰는 복구.
- 뒤쪽 map index로 되돌아가는 복구.
- CROSS를 놓친 것으로 추론하고 건너뛰는 복구.
- BOTH/END를 놓친 것으로 추론하고 건너뛰는 복구.
- confidence 20 미만 event를 realignment 근거로 사용.
- encoder가 없는 motor step을 실제 이동거리라고 과신하는 보정.
- 기본 속도 또는 recovery 속도 상향.
- 기존 END guard, END fallback, active brake 변경.
- First Drive map 생성 규칙 변경.
- LCD backlight GPIO 전원 제어.

> 사용자가 말한 “마커 간격이 동일하면 해당 구간으로 복구”는 V32 목표가 아니다. step distance는 local sequence hypothesis를 검증하는 보조 근거이며, 거리만으로 위치를 결정하면 안 된다.

---

## 4. 용어와 상태 정의

### 4.1 edit 용어

- `MISSED_ONE`: 1차 map에는 있는 expected marker `k`를 2차에서 놓치고, 현재 live event가 `k+1`과 일치.
- `EXTRA_ONE`: 2차에서 map에 없는 false-positive event 하나를 봤고, 다음 live event가 원래 expected `k`와 일치.
- `NORMAL`: 현재 live event가 expected `k`와 일치.
- `CROSS_ANCHOR`: 현재 live CROSS가 기존 anchor policy로 유일하게 일치.

### 4.2 권장 sync state

`SecondDriveSyncState_t`를 다음 의미로 확장한다.

```c
typedef enum {
    SECOND_DRIVE_SYNC_MAP = 0,
    SECOND_DRIVE_SYNC_SUSPECT,
    SECOND_DRIVE_SYNC_PROBATION,
    SECOND_DRIVE_SYNC_SEEK_CROSS,
    SECOND_DRIVE_SYNC_INVALID
} SecondDriveSyncState_t;
```

의미:

| 상태 | 의미 | 속도 정책 |
|---|---|---|
| MAP | map 위치와 sequence가 확정됨 | 기존 V31 planner performance 허용 |
| SUSPECT | 현재 mismatch를 extra event 가능성으로 한 번 보류 | First Drive fallback |
| PROBATION | 한 marker 누락 가설로 map 위치를 임시 복구, 다음 event 확인 대기 | map restriction 유지 + exit급 cap |
| SEEK_CROSS | local one-edit 복구 실패 | 기존 First Drive fallback, CROSS만 복구 |
| INVALID | 구조적으로 사용할 map 없음 | 기존 invalid 처리 |

### 4.3 권장 realignment source

진단을 위해 별도 enum을 추가한다.

```c
typedef enum {
    SECOND_DRIVE_REALIGN_NONE = 0,
    SECOND_DRIVE_REALIGN_CROSS_ANCHOR,
    SECOND_DRIVE_REALIGN_MISSED_ONE,
    SECOND_DRIVE_REALIGN_EXTRA_ONE
} SecondDriveRealignSource_t;
```

### 4.4 protected marker

다음 event는 skipped marker로 추론하지 않는다.

```text
MARKER_EVENT_CROSS
MARKER_EVENT_BOTH
```

이유:

- CROSS는 강한 물리 anchor이며 기존 anchor matcher로 처리해야 한다.
- BOTH는 start/END guard와 즉시 정지 정책에 연결되어 있다.
- protected marker를 추론으로 건너뛰면 END 안전과 anchor order를 훼손할 수 있다.

---

## 5. Last trusted checkpoint

local realignment의 기준점을 명확히 하기 위해 planner 내부에 last trusted checkpoint를 둔다. 기존 `planner_segment_start_step`을 무작정 재사용하지 말고, 의미가 명확한 구조체로 묶는 것을 권장한다.

예시:

```c
typedef struct {
    bool valid;
    uint16_t accepted_event_index;
    uint16_t expected_event_index;
    uint16_t segment_index;
    uint32_t map_event_step;
    uint32_t run_event_step;
    bool replay_turn_open;
    int8_t replay_turn_direction;
    uint16_t current_anchor_order;
} SecondDriveTrustedCheckpoint_t;
```

초기 상태는 map step 0, run step 0을 기준으로 하고, start ignore 이후 첫 expected event를 가리킨다.

checkpoint는 다음 경우에만 갱신한다.

- 정상 expected event 확정.
- CROSS anchor resync 성공.
- EXTRA_ONE 확인 성공.
- PROBATION에서 다음 event 확인 성공.

checkpoint를 갱신하지 않는 경우:

- 첫 mismatch.
- SUSPECT 진입.
- MISSED_ONE 가설로 PROBATION 진입 직후.
- ambiguous/reject.

PROBATION은 tentative position과 last trusted position을 둘 다 보존해야 한다. probation 실패 시 tentative state를 연속해서 다시 realign하지 않고 SEEK_CROSS로 내려가기 때문이다.

---

## 6. 제한된 sequence realignment 알고리즘

### 6.1 전체 상태 전이

```text
                       matching next event
       +----------------------------------------------+
       |                                              v
MAP --normal----------------------------------------> MAP
 |
 | mismatch, current == map[k+1], protected skip 아님,
 | type+distance unique
 v
PROBATION --next expected match---------------------> MAP
   |                      
   +--mismatch---------------------------------------> SEEK_CROSS

MAP --other non-CROSS mismatch----------------------> SUSPECT
                                                       |
                                                       | next event == map[k]
                                                       v
                                                      MAP
                                                       |
                                                       +--otherwise--> SEEK_CROSS

SUSPECT / PROBATION / SEEK_CROSS --valid CROSS------> MAP
```

### 6.2 MAP에서 정상 event

현재 expected index를 `k`라고 한다.

다음을 모두 만족하면 V31과 동일하게 정상 진행한다.

- type이 `map[k].type`과 동일.
- last trusted run step부터 현재 live center step까지의 거리가 허용범위 이내.
- segment/event bounds 정상.

정상 event는 기존 `SecondDrive_AdvanceAfterEvent()` 의미를 유지한다.

### 6.3 MISSED_ONE 후보

정상 event matching이 실패했을 때만 `k+1` 후보를 평가한다.

필수 조건:

1. `k+1 < Track_GetEventCount()`.
2. `map[k]`가 CROSS/BOTH가 아님.
3. current live event도 CROSS/BOTH가 아님.
4. live type이 `map[k+1].type`과 동일.
5. live confidence가 기존 accepted floor인 20 이상.
6. observed cumulative distance와 recorded cumulative distance가 tight tolerance 이내.
7. normal `k` 가설과 skip `k+1` 가설이 동시에 유효하지 않음.
8. index/segment 계산이 forward one-step 범위이고 overflow 없음.

거리 정의:

```text
observed = live.center_step - trusted.run_event_step
recorded = map[k+1].center_step - trusted.map_event_step
error    = abs(observed - recorded)
```

map absolute step이 checkpoint보다 작거나 live step이 checkpoint보다 작으면 bounds failure다.

MISSED_ONE 성공 시 즉시 완전 MAP으로 복귀하지 않는다.

```text
tentative expected index = k + 2
tentative segment index  = current segment + 2
segment start run step   = current live center_step
sync                     = PROBATION
realign source           = MISSED_ONE
```

그리고 pair/course state를 `map[k+1]` 처리 직후 상태로 복원한다.

### 6.4 EXTRA_ONE 후보

현재 mismatch가 `k+1` skip 후보로 충분하지 않으면 한 번만 extra event 가능성을 보류한다.

SUSPECT 진입 시 저장할 정보:

- mismatch event type/center step/confidence.
- original expected `k`.
- original segment index/start step.
- original trusted checkpoint.
- mismatch reason.

SUSPECT에서는 map index, segment index, replay pair를 advance하지 않는다.

다음 finalized event에서:

- valid CROSS이면 기존 anchor resync를 우선한다.
- event가 original expected `k`와 type+distance 모두 일치하면 앞 event를 EXTRA_ONE으로 확정한다.
- EXTRA_ONE 성공 시 expected event를 정상적으로 advance하고 MAP으로 복귀한다.
- 그 외에는 local edit budget을 모두 사용한 것으로 보고 SEEK_CROSS로 전환한다.

SUSPECT에서 다시 MISSED_ONE 후보를 연결하거나 두 번째 extra를 보류하면 안 된다.

### 6.5 PROBATION confirmation

PROBATION은 MISSED_ONE으로 tentative commit한 직후 상태다.

다음 event가 tentative expected와 정상 type+distance로 일치하면:

- tentative state를 trusted checkpoint로 승격.
- sync를 MAP으로 변경.
- probation success count 증가.
- full performance는 fast gate 안정 조건을 다시 통과한 뒤 허용.

다음 event가 일치하지 않으면:

- 추가 local edit를 시도하지 않는다.
- `SECOND_DRIVE_MISMATCH_PROBATION_FAILED` 기록.
- SEEK_CROSS로 전환.
- CROSS anchor를 기다린다.

PROBATION 중 live CROSS가 forward anchor로 유일하게 일치하면 probation을 폐기하고 CROSS anchor state로 즉시 완전 복구한다.

### 6.6 distance tolerance

V31 일반 matching은 `max(200 step, recorded/3)`, CROSS anchor는 `max(500 step, recorded/3)`를 사용한다. 이 33% tolerance를 directional one-event realignment에 그대로 쓰지 않는다.

V32 초기 권장 정책:

```text
realign tolerance = clamp(recorded / 5, 200, 600) step
confirmation tolerance = clamp(recorded / 5, 150, 400) step
candidate score = absolute step error
minimum score margin = 100 step
```

정확한 상수명은 Luna가 일관되게 정하되 모두 `second_drive.c` 상단에 모은다. 예:

```c
SECOND_DRIVE_REALIGN_TOLERANCE_DIV
SECOND_DRIVE_REALIGN_TOLERANCE_MIN
SECOND_DRIVE_REALIGN_TOLERANCE_MAX
SECOND_DRIVE_REALIGN_CONFIRM_TOLERANCE_MIN
SECOND_DRIVE_REALIGN_CONFIRM_TOLERANCE_MAX
SECOND_DRIVE_REALIGN_SCORE_MARGIN_STEPS
```

주의:

- 위 수치는 실차 marker center 오차가 아직 축적되지 않은 초기 안전값이다.
- V32에서 실차 데이터를 보지 않고 tolerance를 더 넓히지 않는다.
- type sequence가 먼저 맞아야 하며 distance는 보조 검증이다.
- 같은 type이 연속되는 map에서는 normal/skip score가 모두 유효하면 ambiguous 처리한다.

### 6.7 우선순위

한 event를 처리할 때 우선순위는 다음을 지킨다.

1. null/low confidence/invalid bounds reject.
2. confirmed final END의 기존 안전 처리 보존.
3. current live CROSS의 기존 anchor matching.
4. MAP normal expected match.
5. 기존 V30/V31 local close repair가 명백히 성립하는 경우.
6. MAP MISSED_ONE 평가.
7. MAP EXTRA_ONE suspect 시작.
8. SUSPECT/PROBATION 전용 처리.
9. 실패 시 SEEK_CROSS.

local close repair는 map event index를 움직이지 않는 보조 geometry repair다. 이를 sequence skip success로 잘못 집계하지 않는다.

---

## 7. Map state commit 계약

### 7.1 partial mutation 금지

realignment 후보를 평가하는 동안 live planner 변수를 직접 조금씩 수정하지 않는다.

권장 구조:

```text
read current immutable snapshot
-> evaluate candidate into local result struct
-> validate all bounds/type/distance/role
-> build complete tentative state
-> one commit function에서 planner state publish
```

후보 검증 실패 시 다음 값은 바뀌면 안 된다.

- expected event index
- segment index
- segment start step
- anchor order
- replay pair state
- last trusted checkpoint

### 7.2 segment index 계산

현재 state에서 expected `k`가 맞으면 next segment는 `current + 1`, `k+1`을 한 marker skip으로 맞추면 next segment는 `current + 2`다.

단순 덧셈 전 다음을 검사한다.

- `Track_GetSegment()`가 non-null.
- event count와 segment count 경계.
- `uint16_t` overflow 없음.
- protected event를 건너뛰지 않음.

전역 event index와 segment index가 언제나 같은 offset이라고 새로 가정하지 말고, 현재 planner의 known expected/segment 관계에서 상대적으로 계산한다. start marker ignore 때문에 처음부터 절대 index를 동일시하면 안 된다.

### 7.3 anchor 상태

MISSED_ONE/EXTRA_ONE은 CROSS를 건너뛰지 않으므로 last confirmed CROSS anchor는 그대로 유지한다.

- `current_anchor_order`를 임의로 증가시키지 않는다.
- `planner_last_anchor_map_step/run_step`을 directional realignment로 덮지 않는다.
- 이후 CROSS가 들어오면 기존 forward anchor rule을 그대로 적용한다.

### 7.4 fast gate

realignment commit 또는 course restore 직후에는 fast gate를 reset한다.

```text
planner_fast_gate_ready = false
planner_fast_stable_frames = 0
centered_stable_frames = 0
final exit override = false
cross exit active = false, CROSS restore인 경우만 기존 policy 적용
```

잘못된 상태에서 5600 SPS가 한 control tick이라도 유지되지 않도록 sync/limiter publish 순서를 점검한다.

---

## 8. Replay marker pair 재구성

### 8.1 왜 pair restore가 필요한가

예를 들어 map이 다음과 같다고 한다.

```text
RIGHT entry -> RIGHT close -> LEFT entry -> LEFT close
```

Second Drive가 `RIGHT close`를 놓치고 `LEFT entry`를 현재 event로 보아 `k+1`에 realign했다면, 단순히 expected index를 `LEFT close`로 옮기는 것만으로 부족하다.

잘못된 기존 runtime pair:

```text
open RIGHT
```

map sequence상 `LEFT entry` 직후의 올바른 pair:

```text
open LEFT
```

이 pair를 복원하지 않으면 `MARKER_PAIR_UNCLOSED` limiter, curve direction, final corridor 판단이 계속 잘못된다.

### 8.2 pure map replay helper

rare recovery 경로이므로 O(number of nearby events) 또는 O(total events) pure replay는 허용된다. 1 kHz target 계산 경로에 넣지만 않으면 된다.

권장 helper 역할:

```c
typedef enum {
    SECOND_DRIVE_MAP_ROLE_NONE = 0,
    SECOND_DRIVE_MAP_ROLE_ENTRY_LEFT,
    SECOND_DRIVE_MAP_ROLE_ENTRY_RIGHT,
    SECOND_DRIVE_MAP_ROLE_CLOSE_LEFT,
    SECOND_DRIVE_MAP_ROLE_CLOSE_RIGHT,
    SECOND_DRIVE_MAP_ROLE_CROSS,
    SECOND_DRIVE_MAP_ROLE_END
} SecondDriveMapEventRole_t;

typedef struct {
    bool turn_open;
    int8_t turn_direction;
    uint16_t open_event_index;
    SecondDriveMapEventRole_t matched_role;
} SecondDriveMapReplayState_t;
```

`Track_SegmentTypeFromEvent()`의 규칙과 동일한 pair semantics를 사용한다. 가능하면 규칙을 공용 pure helper로 추출해 track map 생성과 second-drive replay가 같은 구현을 사용하게 한다. 동일 규칙을 두 파일에 복제하면 향후 다시 어긋날 수 있다.

단, `Track_FinalizeSegments()`의 기존 결과를 바꾸지 않는 회귀 test를 반드시 둔다.

### 8.3 replay open step의 live 좌표 변환

pair가 open 상태라면 `planner_replay_turn_open_step`은 live run step 좌표여야 한다.

matched map event에서 open map event까지의 차이를 사용한다.

```text
map_delta = matched_map.center_step - open_map.center_step
run_open_step = live_matched.center_step - map_delta
```

underflow, reversed map step, out-of-range index가 있으면 realignment를 reject하고 SEEK_CROSS로 간다.

현재 matched event 자체가 entry라면 `run_open_step = live_matched.center_step`이다.

### 8.4 geometry source

다음 source를 구분할 수 있게 확장한다.

```text
SEQUENCE_MISSED_ONE
SEQUENCE_EXTRA_ONE
```

기존 `PAIR_OPEN`, `PAIR_CLOSE`, `CROSS_RESET`, `LOCAL_CLOSE_REPAIR` 의미를 유지한다. PROBATION이라는 sync trust와 pair geometry source를 하나의 enum으로 섞지 않는다.

---

## 9. 공통 drive course phase 복원

### 9.1 planner만 복구하면 안 되는 이유

directional marker는 finalized event 이전 provisional 단계에서 이미 `FirstDrive_HandleDirectionalMarker()`를 호출할 수 있다.

따라서 false extra marker가 들어온 경우:

```text
planner map index는 아직 정상
하지만 drive_course_phase는 잘못된 APPROACH/TURN으로 변경될 수 있음
```

또는 close marker를 놓친 경우:

```text
planner는 SEEK/PROBATION에서 새 위치를 찾음
하지만 drive_course_phase는 이전 TURN_RIGHT에 남아 있음
```

이 상태에서는 map 복구 후에도 curve target 또는 pair-open 제한이 유지될 수 있다.

### 9.2 planner event decision을 drive에 반환

`SecondDrivePlanner_OnEvent()`가 내부 state만 바꾸고 `void`로 끝나는 구조를 확장한다. 다음과 같은 decision/result를 반환하는 방식을 권장한다.

```c
typedef enum {
    SECOND_DRIVE_EVENT_NORMAL = 0,
    SECOND_DRIVE_EVENT_SUSPECT_HELD,
    SECOND_DRIVE_EVENT_PROBATION_STARTED,
    SECOND_DRIVE_EVENT_REALIGNED,
    SECOND_DRIVE_EVENT_CROSS_RESYNCED,
    SECOND_DRIVE_EVENT_SEEKING
} SecondDriveEventAction_t;

typedef struct {
    SecondDriveEventAction_t action;
    bool suppress_normal_directional_transition;
    bool restore_course;
    FirstDriveCoursePhase_t restored_phase;
    int8_t restored_direction;
    uint32_t reference_step;
    SecondDriveRealignSource_t source;
} SecondDriveEventDecision_t;
```

정확한 API 모양은 달라도 되지만 아래 계약은 지켜야 한다.

- planner가 어떤 event를 normal로 소비했는지 drive가 알 수 있어야 한다.
- suspect/realign event를 common directional handler로 다시 중복 적용하지 않아야 한다.
- planner가 계산한 map role에 따라 drive course를 한 번 복원해야 한다.
- CROSS/END의 기존 공통 안전 처리는 보존해야 한다.

### 9.3 course phase mapping

matched map event의 role을 기준으로 다음처럼 복원한다.

| map event role | 복원 phase |
|---|---|
| ENTRY_LEFT | `TURN_LEFT` |
| ENTRY_RIGHT | `TURN_RIGHT` |
| CLOSE_LEFT | `EXIT_LEFT` |
| CLOSE_RIGHT | `EXIT_RIGHT` |
| CROSS | 기존 `CROSS` 처리 |
| END | realignment 대상 아님, 기존 stop 처리 |

close로 복원된 경우 바로 STRAIGHT로 강제하지 않는다.

```text
EXIT_LEFT/RIGHT
-> 기존 line centered frame 조건
-> STRAIGHT
-> fast gate 30 stable frames
-> full straight speed
```

이렇게 해야 물리적으로 아직 코너 탈출 중인 구간에서 즉시 5600 SPS를 허용하지 않는다.

### 9.4 drive restore helper

`drive.c` 내부에 전용 helper를 추가한다. 기존 marker handler를 fake event로 재호출하지 않는다.

권장 책임:

```text
drive_course_phase 설정
drive_expected_turn 설정
drive_state FOLLOW/TURN/CROSS 일관성 갱신
drive_turn_center_frames reset
drive_marker_left/right_frames reset
provisional marker clear
marker slow window를 reference step 기준으로 안전하게 설정
필요 시 cross timer 설정
telemetry course/provisional 동기화
phase transition log에 REALIGN reason 기록
```

`FirstDrivePhaseReason_t`에 다음과 같은 reason을 추가한다.

```text
SECOND_DRIVE_REALIGN_MISSED
SECOND_DRIVE_REALIGN_EXTRA
SECOND_DRIVE_CROSS_RESYNC
```

이 helper는 Second Drive에서만 호출하며 First Drive 동작을 바꾸지 않는다.

### 9.5 event 처리 순서

finalized event 한 개에 대해 권장 순서는 다음과 같다.

```text
planner decision 계산 및 planner state commit
-> CROSS/BOTH 안전 처리가 필요한지 확인
-> directional normal transition을 적용할지 decision 확인
-> normal이면 기존 handler
-> restore directive면 provisional effect를 지우고 전용 course restore
-> telemetry publish
```

`suppress_normal_directional_transition`이 true인데 기존 `FirstDrive_HandleDirectionalMarker()`까지 호출하면 event를 두 번 적용하게 되므로 금지한다.

END는 planner decision 때문에 stop guard를 건너뛰면 안 된다. `MARKER_EVENT_BOTH`의 V28 defensive cross-tail guard와 confirmed END active brake 흐름은 항상 기존대로 실행한다.

---

## 10. PROBATION 속도 정책

### 10.1 목표

PROBATION은 SEEK_CROSS보다 map 위치 신뢰가 높지만 완전 MAP보다 낮다. 다음 marker 하나가 맞을 때까지 바로 5600 SPS를 허용하지 않는다.

### 10.2 권장 cap

기존 map lookahead, TURN brake, END fallback을 모두 계산한 뒤 최종 performance target에 probation cap을 적용한다.

```text
probation_cap = SecondDrive_GetEffectiveExitSps()
final target = min(existing planner target, probation_cap)
```

기본값에서는 약 3800 SPS다. overall percent가 적용된 exit profile을 사용하되 center max clamp는 기존 정책을 따른다.

다음 제한은 probation cap보다 항상 우선한다.

- line recovery 1400/1800.
- END safe approach 1800.
- turn braking restriction.
- motor/wheel absolute clamp.
- invalid line/position safety.

### 10.3 SUSPECT와 SEEK

- `SUSPECT`: V31 SEEK와 같은 First Drive fallback target.
- `SEEK_CROSS`: V31 그대로 First Drive fallback target.
- `MAP`: 기존 V31 performance planner.

새 limiter reason을 분리한다.

```text
SECOND_DRIVE_LIMIT_REALIGN_SUSPECT
SECOND_DRIVE_LIMIT_REALIGN_PROBATION
```

기존 `SEEK_CROSS`와 합쳐 기록하면 영상에서 green/yellow 시간을 수치와 대응시킬 수 없다.

---

## 11. 주행 중 전체 화면 색상 진단

### 11.1 목적과 의미

사용자는 주행 영상을 촬영하므로 주행 후 숫자만 보는 것보다, map trust가 떨어진 순간과 회복 순간을 영상에서 바로 보고 싶어 한다.

필수 색상 계약:

| 화면 | 의미 |
|---|---|
| BLACK | `SYNC_MAP`, map 기반 performance 주행이 허용되는 정상 상태 |
| GREEN | `SUSPECT` 또는 `SEEK_CROSS`, First Drive fallback 정속/안전 주행 |
| YELLOW | `PROBATION`, one-marker realignment 후 다음 marker 확인 대기 |
| RED | active run에서 map invalid 또는 치명 상태. fault 정지 후에는 기존 fault/result UI |

중요:

> BLACK은 모터 SPS가 매 순간 상승 중이라는 뜻이 아니다. 정상 curve, braking lookahead, END approach에서도 map trust가 유지되면 BLACK이다. 이 진단의 목적은 “왜 직선에서 가속하지 않았는가”를 영상에서 구분하는 것이므로 motor speed derivative가 아니라 planner trust state를 표시한다.

사용자가 요청한 “가속일 때 화면을 끈다”는 LCD backlight 전원 제어가 아니라 full-screen BLACK fill로 구현한다. backlight GPIO를 끄면 이후 GREEN 전환 때 재활성화 타이밍, LCD 상태, 보드별 차이가 생긴다.

### 11.2 화면 갱신 위치

LCD/SPI API는 TIM7 control callback, motor IRQ, sensor IRQ, `SecondDrivePlanner_GetTargetSps()`에서 호출하면 안 된다.

권장 구조:

```text
planner/control path
-> sync state enum만 갱신

Main_Menu() main loop
-> Menu_UpdateSecondDrive()
-> active run이면 현재 visual state snapshot
-> 이전 visual state와 다를 때만 Menu_FillScreen(color) 1회
```

### 11.3 V30/V31 display lock과의 결합

V31의 `second_drive_run_screen_locked`는 live run 최초 static screen 이후 periodic redraw를 모두 막는다. V32에서는 이를 다음처럼 바꾼다.

- live run 중 text telemetry redraw는 계속 금지.
- state color transition fill만 허용.
- 같은 색 상태가 유지되면 LCD API 호출 없이 return.
- STOPPED/FAULT로 바뀌면 lock을 해제하고 기존 result UI를 그린다.

권장 runtime 변수:

```c
static SecondDriveRuntimeVisualState_t second_drive_last_visual_state;
static uint32_t second_drive_visual_transition_count;
```

`second_drive_active_render_count`를 유지한다면 V32부터 의미를 “periodic redraw count 0”가 아니라 “실제 active color transition fill 횟수”로 명확히 바꾼다. 동일 색 반복 호출로 count가 늘면 실패다.

### 11.4 버튼과 정지

전체 화면 색상을 채워도 CENTER emergency stop 입력 처리는 계속 동작해야 한다. 화면에 문구가 없다는 이유로 버튼 처리 분기를 제거하면 안 된다.

다음 상태에서 result/fault UI 복귀를 확인한다.

- 정상 END active brake 완료.
- manual stop.
- emergency stop.
- sensor/motor/control fault.

### 11.5 깜빡임 방지

- target SPS, fast gate, course phase로 직접 색을 바꾸지 않는다.
- sync state가 바뀔 때만 색을 바꾼다.
- BLACK↔GREEN↔YELLOW state transition 자체는 즉시 보여야 하므로 긴 debounce는 두지 않는다.
- 같은 control frame에서 tentative state를 여러 번 publish하지 말고 최종 state만 publish한다.

---

## 12. 진단 구조

영상 진단이 주목적이지만 sequence algorithm 검증을 위해 최소한의 수치 기록은 유지한다. V31 limiter log를 삭제하지 않는다.

### 12.1 planner status 권장 필드

```text
realign_source
realign_pending
realign_candidate_event_index
realign_skipped_event_index
realign_observed_steps
realign_recorded_steps
realign_error_steps
realign_best_error_steps
realign_second_error_steps
probation_expected_event_index
suspect_event_type
suspect_event_step
```

### 12.2 run stats 권장 필드

```text
suspect_entry_count
missed_one_probation_count
missed_one_confirmed_count
extra_one_confirmed_count
probation_failed_count
realign_ambiguous_count
realign_protected_reject_count
cross_resync_count
suspect_max_ms / steps
probation_max_ms / steps
last_realign_source / step / error
```

모든 count는 saturating increment를 사용한다.

### 12.3 result UI

기존 V31 네 페이지를 삭제하지 않는다. 결과 UI에는 최소 한 페이지 또는 기존 마지막 페이지 일부를 사용하여 다음을 확인할 수 있게 한다.

```text
REALIGN Mx Ex Cx Fx
SUS x  PROB x  CROSS x
LAST MISS/EXTRA/CROSS Sxxxxx
ERR xxx  SKIP Exx  FINAL MAP/SEEK
```

화면 공간이 부족하면 상세 best/second score는 telemetry/run stats에만 두고 결과 페이지에는 success/failure count와 last source만 표시한다.

영상 색상과 결과 수치의 대응:

- GREEN duration: SUSPECT + SEEK limiter episode.
- YELLOW duration: PROBATION limiter episode.
- GREEN -> BLACK at CROSS: cross resync count 증가.
- YELLOW -> BLACK: missed-one confirmed count 증가.

---

## 13. API 및 구조 변경 권고

### 13.1 `second_drive.h`

추가/변경 후보:

- `SecondDriveSyncState_t`: SUSPECT/PROBATION.
- `SecondDriveRealignSource_t`.
- `SecondDriveMapEventRole_t`.
- `SecondDriveEventAction_t`.
- `SecondDriveEventDecision_t`.
- mismatch/reject reason 확장.
- limiter reason 확장.
- planner status/run stats realignment fields.
- `SecondDrivePlanner_OnEvent()` decision 반환 계약.

API 예시:

```c
SecondDriveEventDecision_t SecondDrivePlanner_OnEvent(
        const TrackMarkerEvent_t *event,
        FirstDriveCoursePhase_t live_course_phase);
```

`live_course_phase`를 실제 scoring 근거로 사용하지 않더라도, decision/log에서 provisional contamination을 확인하는 데 쓸 수 있다. map sequence가 authoritative source다.

### 13.2 `drive.h`

- realignment phase reason enum 추가.
- 필요하면 telemetry에 last restore source/phase 추가.
- internal helper는 public API로 노출하지 않아도 된다.

### 13.3 `track.h/c`

pair replay 규칙을 공용 pure helper로 추출할 경우에만 변경한다.

예:

```c
bool Track_ApplyMarkerToPairState(MarkerEventType_t type,
        bool *turn_open, int8_t *turn_direction,
        TrackSegmentType_t *segment_type);
```

이 helper를 추가했다면 `Track_FinalizeSegments()`도 같은 helper를 사용하여 기존 map 결과와 새 replay 결과가 동일하도록 한다.

### 13.4 `menu.h/c`

- active visual state enum 또는 mapping helper.
- transition-only full-screen fill.
- visual transition count getter는 host test에 필요할 때만 public.
- 주행 중 text formatting/snprintf를 반복하지 않는다.

### 13.5 `app_version.h`

```c
#define APP_VERSION_NUMBER 32U
```

version bump는 구현과 test가 모두 통과한 마지막 단계에서 한다.

---

## 14. 파일별 작업 지시

### 14.1 `Main/Src/second_drive.c`

1. trusted/tentative/suspect runtime state 추가.
2. tight distance helper와 saturating/clamped 계산 추가.
3. normal/skip/extra hypothesis 평가를 side-effect-free helper로 분리.
4. protected marker 검사.
5. map pair replay helper 또는 track 공용 helper 사용.
6. event role과 course restore decision 생성.
7. SUSPECT/PROBATION state handler.
8. CROSS priority 유지.
9. atomic/complete commit helper.
10. realignment 시 fast gate reset.
11. probation cap과 새 limiter reason.
12. stats/trace 기록.
13. reset/begin/finalize에서 새 필드 초기화·flush.

### 14.2 `Main/Src/drive.c`

1. planner event decision을 받도록 marker processing integration 변경.
2. directional event double-apply 방지.
3. Second Drive 전용 course restore helper.
4. provisional marker와 frame counters 정리.
5. phase/state/telemetry 일관성 갱신.
6. CROSS/BOTH/END 기존 안전 순서 보존.
7. First Drive path는 기존과 byte-for-byte 동일할 필요는 없지만 동작은 완전히 동일해야 함.

### 14.3 `Main/Src/track.c`

공용 pair transition helper가 필요할 경우 최소 변경한다. 다음 결과가 V31과 동일해야 한다.

- marker event count/type.
- segment count/type/distance.
- CROSS anchor event/segment index.
- unmatched pair diagnostics.

### 14.4 `Main/Src/menu.c`

1. live run 최초 BLACK fill.
2. sync visual state 변경 시 한 번만 full-screen fill.
3. active state에서 text redraw 금지.
4. STOP/FAULT에서 결과 화면 정상 복귀.
5. result page에 최소 realignment summary.
6. transition count 의미와 test 갱신.

### 14.5 `tests/v32/`

다음을 저장한다.

- `run_host_tests.sh`
- planner realignment harness.
- planner/drive course restore integration harness 또는 host-only adapter test.
- track pair regression.
- LCD visual-state pure mapping/transition test.
- 기존 END/brake/candidate/curve regression.

임시 `/private/tmp` source만 남기지 않는다. test 실행 파일은 `mktemp -d` 아래 만들고 종료 시 정리한다.

---

## 15. 상세 host test matrix

### 15.1 정상 replay 회귀

- 모든 event가 순서와 거리대로 들어옴.
- sync는 계속 MAP.
- suspect/probation count 0.
- expected/segment index V31과 동일.
- pair open/close V31과 동일.
- fast straight target V31과 동일.
- LCD semantic state BLACK.

### 15.2 MISSED_ONE 성공

예시 map:

```text
RIGHT(entry) -> RIGHT(close) -> LEFT(entry) -> LEFT(close) -> CROSS -> END
```

입력:

```text
RIGHT(entry) 정상
RIGHT(close) 생략
LEFT(entry) live event, map cumulative step와 tight match
```

기대:

- PROBATION 진입.
- expected는 LEFT(close).
- pair open LEFT.
- course restore directive `TURN_LEFT`.
- segment는 LEFT 이후 segment.
- anchor order 불변.
- target은 probation cap 이하.
- LCD YELLOW.

이후 LEFT(close) 정상 입력:

- MAP 복귀.
- pair closed.
- course는 normal close transition 또는 restore 결과로 `EXIT_LEFT`.
- fast gate reset 후 stable frame을 채워야 full speed.
- missed-one confirmed count 1.
- LCD BLACK.

### 15.3 EXTRA_ONE 성공

입력:

```text
RIGHT(entry) 정상
map에 없는 LEFT false event
원래 expected RIGHT(close)가 정상 step에서 입력
```

기대:

- false LEFT에서 SUSPECT.
- map expected/segment/checkpoint advance 없음.
- false provisional effect를 course restore로 제거.
- course는 RIGHT curve state와 일치.
- target은 First Drive fallback.
- LCD GREEN.
- 다음 RIGHT에서 EXTRA_ONE 확정, MAP 복귀.
- pair closed, course `EXIT_RIGHT`.
- extra count 1, skipped count 0.
- LCD BLACK.

### 15.4 distance-only 금지

- recorded cumulative distance와 거의 같지만 type이 다른 event.
- MISSED_ONE으로 복구하면 실패.
- SUSPECT 또는 SEEK로 이동.
- map index jump 없음.

### 15.5 repeated type ambiguity

- `RIGHT, RIGHT, RIGHT`처럼 local type이 반복되는 fixture.
- normal/skip score가 margin 이내이면 ambiguous.
- PROBATION 금지.
- SEEK_CROSS 또는 SUSPECT 후 SEEK.

### 15.6 protected marker

- expected `k`가 CROSS인데 live event가 `k+1` directional type과 맞는 fixture.
- CROSS skip 추론 금지.
- expected `k`가 BOTH/END인 fixture도 동일.
- protected reject count 증가.

### 15.7 probation failure

- MISSED_ONE으로 PROBATION 진입.
- 다음 event type 또는 distance mismatch.
- 두 번째 local edit 시도 금지.
- SEEK_CROSS.
- tentative state를 MAP으로 승격하지 않음.
- LCD GREEN.

### 15.8 CROSS recovery priority

각 상태에서 valid CROSS를 입력한다.

- SUSPECT -> MAP.
- PROBATION -> MAP.
- SEEK_CROSS -> MAP.
- pair closed reset.
- anchor order forward.
- expected event/segment index anchor 기준.
- source CROSS_ANCHOR.
- LCD BLACK.

### 15.9 invalid/ambiguous CROSS

- forward 후보 없음.
- 두 anchor score 차이가 tie tolerance 이하.
- MAP 복귀 금지.
- 기존 anchor ambiguous reason 유지.

### 15.10 course restore double-apply 방지

- restore directive가 있는 directional event에서 기존 handler까지 적용했을 때 생길 수 있는 TURN→EXIT 이중 전이를 test로 차단.
- 한 finalized event당 map role 한 번만 적용.
- provisional marker clear.
- phase log reason이 REALIGN으로 한 번 기록.

### 15.11 speed policy

- MAP straight: 기존 effective straight target.
- SUSPECT: first-drive fallback.
- PROBATION straight: exit cap 이하.
- PROBATION + recovery: 1400/1800 우선.
- PROBATION + END window: 1800 END cap 우선.
- SEEK CROSS: V31 fallback.

### 15.12 LCD transition-only

pure visual mapping 또는 host adapter에서 다음을 검증한다.

```text
MAP 반복 1000회       -> BLACK fill 1회 이하
MAP -> SUSPECT        -> GREEN fill 1회
SUSPECT 반복          -> 추가 fill 0회
SUSPECT -> PROBATION  -> YELLOW fill 1회
PROBATION -> MAP      -> BLACK fill 1회
STOPPED               -> result renderer 복귀
```

LCD API가 interrupt/control 파일에 추가되지 않았는지 source check도 둔다.

### 15.13 V31 안전 회귀

- pair-open final END fallback.
- corridor false confirmed END active brake.
- emergency/fault full-off.
- marker candidate episode counting.
- cross-tail fusion.
- local close repair negative tests.
- curve speed contract.
- map/anchor overflow invalidation.

---

## 16. 구현 순서

다음 순서를 권장한다.

1. 현재 V31 host test와 firmware build 결과를 기록한다.
2. V32 enum/status/stats 구조만 추가하고 build한다.
3. trusted checkpoint와 side-effect-free hypothesis evaluator를 구현한다.
4. MISSED_ONE planner-only host test를 통과시킨다.
5. SUSPECT/EXTRA_ONE planner-only test를 통과시킨다.
6. PROBATION confirmation/failure와 CROSS priority를 구현한다.
7. pair pure replay와 live open-step 변환을 구현한다.
8. planner event decision 구조를 추가한다.
9. drive course restore helper와 double-apply suppression을 구현한다.
10. probation speed cap과 limiter reason을 적용한다.
11. run stats/result summary를 추가한다.
12. menu transition-only full-screen color를 구현한다.
13. END/brake/First Drive/track regression을 모두 실행한다.
14. ARM firmware build와 stack/RAM/map 사용량을 확인한다.
15. 마지막에 version을 32로 올린다.
16. 구현 결과를 `codex_worked_review`에 작성한다.

각 단계에서 컴파일 가능한 상태를 유지한다. planner와 drive API를 동시에 크게 바꾸고 마지막에 한꺼번에 오류를 고치지 않는다.

---

## 17. 실차 검증 순서

### 17.1 정상 baseline

V31과 동일한 안전한 설정으로 시작한다.

```text
Second Drive straight 5600
Second Drive curve 3000
overall 100%
First Drive curve는 V31 실차에서 사용한 값 유지
```

확인:

- First Drive map 성공.
- Second Drive 정상 marker sequence에서는 주행 중 화면이 대부분 BLACK.
- 정상 curve/braking 때문에 GREEN으로 변하지 않음.
- CROSS 통과 시 불필요한 GREEN flash 없음.
- END에서 result UI 복귀와 active brake 성공.

### 17.2 기존 문제 재현 시 영상 판독

영상에서 다음을 구분한다.

```text
BLACK -> GREEN -> BLACK at CROSS
  = local realignment 실패, 기존 CROSS 복구

BLACK -> YELLOW -> BLACK at next directional marker
  = MISSED_ONE probation 성공

BLACK -> GREEN -> BLACK at expected directional marker
  = EXTRA_ONE 확인 성공

BLACK -> YELLOW -> GREEN -> BLACK at CROSS
  = MISSED_ONE 가설 실패 후 CROSS 복구
```

### 17.3 안전 확인

- YELLOW 직후 즉시 5600으로 튀지 않음.
- restore된 close marker는 EXIT를 거쳐 STRAIGHT로 감.
- restore된 entry marker는 올바른 LEFT/RIGHT TURN 방향.
- 라인 손실 시 기존 recovery cap 우선.
- CROSS/END를 건너뛴 것으로 복구하지 않음.
- final END active brake 유지.

### 17.4 tuning

V32 첫 실차에서는 tolerance와 속도를 동시에 바꾸지 않는다.

1. 기본 tolerance와 기본 속도 run.
2. realign error/best/second 결과 확인.
3. false recovery가 없다는 것이 확인된 뒤에만 tolerance 조정.
4. V32 안정 확인 후 별도 버전에서 속도 상향 검토.

---

## 18. 완료 기준

다음을 모두 만족해야 V32 완료다.

### 기능

- normal MAP replay가 V31과 동일하게 동작.
- 한 map marker 누락을 `k+1`까지만 탐색.
- 한 live extra marker를 한 번만 보류.
- type sequence와 step distance를 함께 사용.
- 거리만으로 map 위치를 선택하지 않음.
- CROSS/BOTH를 skipped marker로 추론하지 않음.
- PROBATION 다음 event 확인 전 full straight speed 금지.
- realignment 성공 시 map/segment/pair/course가 함께 복원.
- CROSS anchor 복구는 모든 uncertain state에서 유지.

### LCD

- MAP BLACK, SUSPECT/SEEK GREEN, PROBATION YELLOW.
- 같은 state에서 반복 LCD redraw 없음.
- LCD API가 ISR/control path에서 호출되지 않음.
- CENTER emergency stop 정상.
- STOP/FAULT result 화면 정상 복귀.

### 안전 회귀

- END safe approach와 active brake 통과.
- line recovery cap 유지.
- local close repair 유지.
- cross-tail/END guard 유지.
- First Drive map 결과 회귀 없음.
- map/anchor overflow 정책 유지.

### 품질

- `sh tests/v32/run_host_tests.sh` PASS.
- firmware Debug build PASS.
- compiler warning 없음.
- stack usage와 RAM 증가량 보고.
- `APP_VERSION_NUMBER 32U`.

---

## 19. 특히 주의할 잠재 문제

### 19.1 false recovery가 긴 fallback보다 더 위험함

잘못된 map 위치로 MAP 복귀하면 다음 코너 braking 시점과 END 위치가 틀어진다. 후보가 애매하면 시간을 잃더라도 SEEK_CROSS를 유지한다.

### 19.2 provisional event contamination

planner가 event를 extra라고 판정해도 provisional marker가 이미 course phase를 바꿨을 수 있다. restore helper 없이 map index만 유지하면 V31과 같은 느린 TURN 상태가 남는다.

### 19.3 double application

realignment commit 후 같은 physical event를 common directional handler에 다시 넣으면 pair/course가 두 번 진행된다. event decision과 suppress flag를 명확히 둔다.

### 19.4 END 처리 순서

unexpected BOTH를 sequence matcher가 소비하거나 무시하여 기존 END guard/active brake를 우회하면 안 된다.

### 19.5 anchor order 손상

directional realignment는 CROSS anchor를 생성하거나 current anchor order를 올리지 않는다.

### 19.6 motor step은 encoder가 아님

탈조와 slip이 있으면 map/run step 차이가 누적된다. tolerance를 넓혀 이를 모두 흡수하려 하지 않는다. 넓은 tolerance는 반복 marker sequence에서 false recovery를 만든다.

### 19.7 LCD resource

full-screen fill은 text rendering보다 단순하지만 SPI 전송 자체는 비용이 있다. control state가 바뀔 때 한 번만 실행하고 target/phase 변화로 깜빡이지 않게 한다.

### 19.8 enum 추가에 따른 array 크기

`SECOND_DRIVE_LIMIT_COUNT`가 늘어나면 run stats histogram과 stack/RAM이 증가한다. menu의 loop와 longest-reason helper가 새 enum을 빠뜨리지 않았는지 확인한다.

### 19.9 run stats snapshot 일관성

menu가 planner status를 읽을 때 기존처럼 짧은 IRQ disable snapshot을 사용한다. LCD fill 동안 IRQ를 막으면 안 된다.

---

## 20. Luna 구현 결과 보고서 요구사항

구현 완료 후 다음 파일을 작성한다.

```text
codex_worked_review/V32_RUNTIME_COLOR_DIAGNOSTIC_LIMITED_SEQUENCE_REALIGNMENT_AND_COURSE_RESTORE_WORK_RESULT.md
```

보고서에 반드시 포함할 내용:

1. 수정 파일 목록.
2. 최종 sync state diagram.
3. normal/MISSED_ONE/EXTRA_ONE/CROSS event flow.
4. 사용한 tolerance 상수와 선정 이유.
5. protected marker 정책.
6. planner state commit 구조.
7. pair replay/open-step 변환 방식.
8. course restore mapping과 double-apply 방지 방식.
9. probation speed cap과 limiter 우선순위.
10. LCD 색상 의미와 실제 LCD API 호출 위치.
11. 같은 state에서 redraw가 없다는 test 증거.
12. host test 전체 명령과 실제 출력.
13. firmware build 결과.
14. ELF/map size와 주요 struct RAM 증가량.
15. stack usage 변화.
16. 미검증 항목과 실차에서 확인해야 할 항목.
17. V31 대비 동작 변경표.

실차를 하지 않았다면 “실차 검증 완료”라고 쓰지 않는다. host fixture와 board/track 실차 결과를 구분해서 보고한다.

---

## 21. 최종 설계 요약

V32의 핵심 원칙은 다음과 같다.

```text
첫 mismatch에서 전체 map을 검색하지 않는다.
현재 expected k와 바로 다음 k+1만 본다.
한 marker 누락이면 PROBATION에서 다음 marker로 확인한다.
한 false marker이면 SUSPECT에서 원래 expected marker를 한 번 기다린다.
두 번째 불일치가 생기면 CROSS anchor로 후퇴한다.
복구할 때 planner index, segment, pair, course phase를 함께 맞춘다.
확신 전에는 5600 SPS를 허용하지 않는다.
영상에서는 BLACK/GREEN/YELLOW로 trust state를 즉시 확인한다.
```

이 원칙을 지키면 V31의 “marker 하나를 놓친 뒤 다음 CROSS까지 긴 First Drive fallback” 시간을 줄이면서도, 반복되는 LEFT/RIGHT 패턴이나 일정한 curve 간격 때문에 잘못된 map 위치로 붙는 위험을 제한할 수 있다.
