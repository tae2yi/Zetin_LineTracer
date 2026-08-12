# V31 END Safety·Speed Tuning·Diagnostic Trust 구현 계획

## 0. 문서 목적

이 문서는 `/Users/ehoi/STM32CubeIDE/line_tracer_2026` 프로젝트의 V30 구현을 기반으로 V31을 구현할 Luna Max용 단독 인수인계 문서다. Luna는 사용자와 Codex가 나눈 이전 대화를 볼 수 없다고 가정한다.

V30은 아직 실차 주행을 하지 않았다. 따라서 V31의 목적은 최고속도를 즉시 더 올리는 것이 아니라, V30에서 발견된 END 안전 경로와 진단 신뢰성 문제를 먼저 해결하고 이후 실차에서 곡선 속도를 안전하게 올릴 수 있는 조정 수단과 검증 자료를 마련하는 것이다.

우선순위는 반드시 다음 순서를 따른다.

1. P0: final corridor가 성립하지 않아도 END 전 안전 감속이 실제로 적용되게 한다.
2. P0: V28 guard가 확정한 Second Drive END에서는 corridor 상태와 무관하게 active brake를 수행한다.
3. P0: marker reject 통계를 실제 S0/S7 marker 후보 episode 기준으로 수정한다.
4. P0: V30에서 부족했던 negative/regression host test를 현재 소스로 재현 가능하게 저장한다.
5. P1: V30 limiter log를 유지하고 감속 episode의 횟수·최대 지속시간·발생 위치를 추가한다.
6. P1: First Drive 곡선 속도를 보드에서 조정할 수 있게 하되 기본값은 V30과 동일하게 유지한다.
7. P2: 실차 검증 후에만 기본 곡선 속도 또는 safety fallback 속도 상향을 결정한다.

> V31 구현 중 V30 기본 straight/curve/overall 값과 recovery 속도를 임의로 올리지 않는다. V30을 실차에서 검증하지 않은 상태에서 안전 수정과 성능 상향을 동시에 기본값으로 적용하면 실패 원인을 분리할 수 없다.

> 작업 트리는 깨끗하지 않으며 V27~V30 구현과 문서, build 산출물이 함께 있다. `git reset`, `git restore`, `git checkout --`, 광범위 삭제를 사용하지 않는다. 현재 소스 위에 V31 변경만 추가한다.

---

## 1. 반드시 먼저 읽을 자료

- V30 계획: `codex_work_plan/V30_SECOND_DRIVE_SPEED_CORRIDOR_HARD_BRAKE_AND_LCD_FREEZE_IMPLEMENTATION_PLAN.md`
- V30 결과: `codex_worked_review/V30_SECOND_DRIVE_SPEED_CORRIDOR_HARD_BRAKE_AND_LCD_FREEZE_WORK_RESULT.md`
- V29 계획: `codex_work_plan/V29_FIRST_DRIVE_RUN_RECORD_AND_LOG_UI_IMPLEMENTATION_PLAN.md`
- V29 결과: `codex_worked_review/V29_FIRST_DRIVE_RUN_RECORD_AND_LOG_UI_WORK_RESULT.md`
- V28 결과: `codex_worked_review/V28_SPLIT_CROSS_TAIL_FUSION_WORK_RESULT.md`
- V27 결과: `codex_worked_review/V27_CROSS_ANCHOR_RESYNC_WORK_RESULT.md`

핵심 소스:

- `Main/Src/drive.c`, `Main/Inc/drive.h`
- `Main/Src/second_drive.c`, `Main/Inc/second_drive.h`
- `Main/Src/track.c`, `Main/Inc/track.h`
- `Main/Src/motor.c`, `Main/Inc/motor.h`
- `Main/Src/menu.c`
- `Main/Inc/app_version.h`

V30 검토 시 다음 명령은 통과했다.

```text
/private/tmp/v30_track_pair_harness
  -> V30 track pair harness PASS

/private/tmp/v30_second_drive_planner_harness
  -> V30 planner local repair PASS

/private/tmp/v30_motor_brake_mock
  -> V30 motor brake mock PASS

cmake --build build/Debug -j 8
  -> Built target 2026_LINE_TRACER_STEP
```

그러나 `/private/tmp`의 test는 임시 파일이므로 장기 회귀 기반으로 인정하지 않는다. V31은 필요한 harness를 프로젝트 내부의 재현 가능한 위치에 두거나, 최소한 작업 결과 문서에서 소스와 build 명령을 완전하게 재현할 수 있게 해야 한다.

---

## 2. 현재 속도와 안전 정책

### 2.1 First Drive V30 기준

중심속도 기준:

| 상태 | 현재 동작 |
|---|---:|
| launch | 400 SPS에서 시작 |
| centered straight | 3820 SPS |
| position 기반 straight envelope | 3820 → 1800 SPS |
| APPROACH/TURN/EXIT | 최대 2200 SPS |
| 방향 marker 이후 600 step | 최대 2400 SPS, turn cap이 있으면 2200 |
| CROSS | 최대 2400 SPS |
| recovery straight | 1400 SPS |
| recovery turn/outer | 1800 SPS |
| First Drive wheel max | 5200 SPS |

현재 곡선 target은 대부분 2200 SPS이고, line position이 최외곽으로 가면 1800 SPS까지 낮아진다.

### 2.2 Second Drive V30 기준

| profile | 100% | 120% |
|---|---:|---:|
| STRAIGHT | 5600 | 6200 clamp |
| APPROACH | 3300 | 3960 |
| CURVE | 3000 | 3600 |
| EXIT | 3800 | 4560 |
| TURN braking target | 3300 | 3960 |
| recovery straight | 1400 | 1400 |
| recovery turn/outer | 1800 | 1800 |

설정 범위:

```text
STRAIGHT 4800..6000, step 100
CURVE    2600..3600, step 100
ALL      90..120%, step 5
center performance max 6200
wheel absolute max 6500
```

### 2.3 V30에서 SAFE로 묶이는 실제 경로

SAFE는 하나의 속도가 아니다.

| limiter/situation | 실제 target |
|---|---|
| MAP_INVALID / SEEK_CROSS | First Drive 위치·phase 기반 target |
| SEGMENT_UNCERTAIN / POSITION / PHASE | First Drive 기반 target |
| MARKER_PAIR_UNCLOSED | Second Drive effective curve phase target |
| line recovery | 1400 또는 1800 SPS |
| intended END fallback | 1800 SPS |

`MARKER_PAIR_UNCLOSED`는 SAFE 통계에 포함되지만 overall percent가 적용된 curve profile을 사용한다. 기본 TURN은 3000 SPS이고 120%에서는 3600 SPS다. 반면 MAP_INVALID/SEEK와 recovery에는 overall이 적용되지 않는다.

V31은 이 서로 다른 안전 경로를 하나의 `safe_speed`로 합치지 않는다.

---

## 3. V30 검토에서 확인된 문제

### 3.1 P0: END fallback이 early return에 의해 우회됨

현재 `SecondDrivePlanner_GetTargetSps()`의 흐름은 대략 다음과 같다.

```text
final performance corridor 판단
-> curve phase면 curve target 즉시 return
-> pair open이면 curve phase target 즉시 return
-> position/phase 불일치면 First Drive target 즉시 return
-> 그 이후에야 lookahead restriction과 END 1800 target 계산
```

따라서 final corridor가 불완전한 상황이 오히려 END restriction 계산에 도달하지 못한다.

대표 재현:

```text
1차 맵에서 마지막 close L 누락
-> 2차에서도 L 누락
-> expected final END, replay pair open
-> MARKER_PAIR_UNCLOSED에서 3000 SPS 반환
-> END 1800 braking 계산 미실행
```

V30 보고서의 “corridor가 불완전하면 기존 safe END fallback 유지”는 현재 제어 흐름과 일치하지 않는다.

### 3.2 P0: active brake가 final performance corridor에 종속됨

현재 `FirstDrive_StopAtEndMarker()`의 Second Drive 분기는 다음 조건일 때만 brake hold를 시작한다.

```c
SecondDrivePlanner_IsFinalEndCorridor()
&& Motor_DriveBrakeHoldStart(...)
```

final corridor가 false이면 `Motor_DriveStop()`으로 timer와 coil을 즉시 끄지만, holding torque를 사용하는 HARD 30ms + REDUCED 120ms active brake는 수행하지 않는다.

성능 corridor는 “END 전까지 고속을 허용할지”에 대한 판단이다. V28 guard를 통과한 confirmed END에서 active brake를 수행할지는 별개의 정지 정책이어야 한다.

### 3.3 P0: marker reject count가 marker 후보가 없는 일반 frame도 집계함

현재 `FirstDrive_GetMarkerEdges()`는 raw S0/S7 edge 유무를 확인하기 전에 다음 상태에서 reject count를 증가시킨다.

- line invalid
- bridge recovery
- abs(position) > 1000
- selected center mask 없음

따라서 일반 curve frame도 `OFF_CENTER` 또는 `NO_CENTER_MASK`로 누적될 수 있다. 이 숫자로 마지막 close marker를 놓친 원인을 판단할 수 없다.

### 3.4 P1: 감속 비율은 알 수 있지만 한 번의 긴 episode를 구분하기 어려움

V30 limiter histogram은 의미가 있다. 하지만 아래 두 run은 같은 SAFE 10%로 보일 수 있다.

```text
run A: 20ms safe episode 50회
run B: 1000ms safe episode 1회
```

실제 랩타임 손실과 위험은 다르므로 reason별 episode count와 최대 연속시간이 필요하다.

### 3.5 P1: V30 test가 계획의 모든 거절·회귀 경로를 다루지 않음

추가로 검증해야 한다.

- local repair wrong-side/no-open/ordinary expected/low confidence/duplicate 거절
- 현재 V30 소스로 mismatch → SEEK → forward CROSS → MAP 복귀
- pair-open final END fallback
- position/phase gate 상태의 final END fallback
- confirmed END active brake가 corridor false에서도 동작
- brake hold 중 emergency/fault full-off
- marker 후보가 없을 때 reject episode가 증가하지 않음
- LCD active render count

---

## 4. V31 범위와 비범위

### 4.1 반드시 구현할 범위

1. final performance corridor와 final stop policy 분리.
2. final corridor 불성립 시 END distance 기반 fallback deceleration.
3. V28-confirmed Second Drive END의 unconditional active brake attempt.
4. active brake 시작 실패 시 즉시 full-off fallback.
5. marker raw candidate episode 기반 reject diagnostics.
6. limiter reason별 episode count와 최대 연속시간.
7. SEEK CROSS episode 횟수·최대 지속시간·최대 step 거리.
8. pair-open episode 횟수·최대 지속시간.
9. END fallback 진입 여부·진입 step·진입 속도·stop mode 기록.
10. First Drive 곡선 속도 board UI 조정 기능. 기본값은 V30과 동일.
11. V30 positive test와 누락된 negative/regression test를 프로젝트에서 재현 가능하게 정리.
12. Second Drive 종료 결과 화면에서 가장 큰 감속 원인을 바로 읽을 수 있게 개선.
13. 앱 버전 31.

### 4.2 V31에서 기본값으로 변경하지 않을 것

- Second Drive straight default 5600 SPS.
- Second Drive curve default 3000 SPS.
- Second Drive overall default 100%.
- recovery 1400/1800 SPS.
- First Drive centered base 3820 SPS.
- First Drive curve default 2200 SPS.
- First Drive curve outer floor default 1800 SPS.
- marker/CROSS threshold와 V28 END guard.
- V27 anchor matching tolerance.
- PD/FF/steer ratio와 motor current.
- local close repair 허용 범위.

### 4.3 V31에서 하지 않을 것

- recovery 속도에 overall percent 적용.
- map invalid/SEEK 상태에서 Second Drive straight 5600 사용.
- centered line만으로 open turn close.
- marker detector threshold를 원인 계측 없이 완화.
- hard brake 조건을 raw S0/S7 또는 provisional BOTH로 약화.
- V30 실차 데이터 없이 Second Drive curve 기본값을 3200 이상으로 변경.

---

## 5. P0 — END 성능 정책과 정지 정책 분리

### 5.1 용어를 명확히 분리

다음 세 상태를 서로 다른 bool/enum으로 관리한다.

```text
final_end_expected
  map이 정상 sync 상태이며 expected event가 final BOTH인지

final_performance_corridor
  marker geometry와 live phase가 검증되어 END 전까지 straight 고속 허용인지

confirmed_end_stop
  V28 FirstDrive_IsEndMarkerEvent()를 통과한 실제 END stop event인지
```

`final_performance_corridor == false`는 active brake 금지 조건이 아니다.

### 5.2 권장 enum과 status

```c
typedef enum {
    SECOND_DRIVE_END_POLICY_NONE = 0,
    SECOND_DRIVE_END_POLICY_FAST_CORRIDOR,
    SECOND_DRIVE_END_POLICY_SAFE_APPROACH,
    SECOND_DRIVE_END_POLICY_MAP_UNCERTAIN
} SecondDriveEndPolicy_t;

typedef enum {
    SECOND_DRIVE_STOP_MODE_NONE = 0,
    SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE,
    SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE_FAILED_FULL_OFF,
    SECOND_DRIVE_STOP_MODE_EMERGENCY_FULL_OFF,
    SECOND_DRIVE_STOP_MODE_FAULT_FULL_OFF
} SecondDriveStopMode_t;
```

`SecondDrivePlannerStatus_t` 또는 `SecondDriveRunStats_t`에 최소 추가:

```text
final_end_expected
end_policy
end_fallback_active
end_fallback_entry_step
end_fallback_entry_sps
end_distance_steps
stop_mode
brake_start_attempted
brake_start_succeeded
```

### 5.3 target 결정 순서 수정

현재처럼 curve/pair/position branch가 END fallback보다 먼저 return하지 않게 한다.

권장 순서:

1. map/sync 및 current segment 확인.
2. travelled/remaining/overdue 확인.
3. expected event와 final END 여부 계산.
4. replay pair/geometry/fast gate로 final performance corridor 계산.
5. nominal phase target 계산. 아직 return하지 않는다.
6. final performance corridor면 straight target과 hard-end policy 사용.
7. final END expected이지만 performance corridor가 아니면 END safe restriction 거리 계산.
8. safe END braking window 안이면 `min(nominal_target, 1800)` 적용.
9. TURN restriction braking과 더 낮은 target이 있으면 낮은 값을 우선.
10. recovery cap 적용.
11. final limiter reason 확정.

핵심은 `nominal target 선택`과 `restriction cap 적용`을 분리하는 것이다.

### 5.4 safe END braking 계산

기존 공식을 재사용한다.

```text
braking_steps = ceil((current_or_nominal_high^2 - 1800^2)
                     / (2 × 10000))
trigger distance = braking_steps + 300 margin
```

V30의 `150 step acceleration re-enable margin`은 END safe fallback에는 재가속 방지 hysteresis로 사용할 수 있다.

조건:

- map valid.
- sync MAP.
- final END expected.
- final performance corridor false.
- expected END까지 map-relative distance 계산 가능.

동작:

- braking window 밖: current safe/phase target 유지.
- braking window 안: target을 1800 이하로 cap.
- 한번 `end_fallback_active`가 되면 END를 통과하거나 run이 끝날 때까지 해제하지 않는다.
- pair open, EXIT 고착, position gate, fast gate not-ready 모두 동일하게 적용한다.

map invalid/SEEK라 expected END 거리 자체를 신뢰할 수 없으면 기존 First Drive 기반 fallback을 유지한다. 단, 실제 END가 확정되면 active brake는 수행한다.

### 5.5 confirmed END active brake

`FirstDrive_IsEndMarkerEvent()`가 true이고 run mode가 Second Drive이면:

```text
record final target/step/end policy
-> disable control
-> stop sensor and TIM7
-> Motor_DriveBrakeHoldStart(HARD Vref) 시도
-> 성공: RUNOUT/HARD 30ms/REDUCED 120ms/OFF
-> 실패: Motor_DriveStop() full off + stop mode 기록
```

다음과 무관하게 active brake를 시도한다.

- final performance corridor true/false.
- MAP/SEEK state.
- pair open/closed.
- 직전 limiter reason.
- END 접근 속도.

단, 다음은 active brake를 사용하지 않고 즉시 full off한다.

- manual emergency.
- fault.
- motor가 이미 fault/비정상 상태.

V28 END guard를 통과하지 않은 raw BOTH/CROSS-tail은 정지 자체를 시작하지 않는다.

### 5.6 stale status에 의존하지 말 것

END marker event 처리는 planner event index를 마지막 event 다음으로 advance할 수 있다. hard brake 여부를 직전 control tick의 `final_end_corridor` bool 하나로 결정하지 않는다.

END stop record에는 marker 처리 직전의 end policy snapshot과 실제 confirmed event step을 별도로 저장한다.

---

## 6. P0 — marker candidate/reject 진단 수정

### 6.1 문제 정의

reject 통계의 목적은 “물리적 S0/S7 marker 신호가 있었지만 line/geometry gate 때문에 detector가 사용하지 않은 사건”을 찾는 것이다. marker 신호가 없던 일반 주행 frame은 reject가 아니다.

### 6.2 raw candidate를 먼저 계산

`FirstDrive_GetMarkerEdges()` 또는 호출부에서 가장 먼저 계산한다.

```c
raw_marker_edges = line->marker_mask & SENSOR_MARKER_MASK;
```

원칙:

- `raw_marker_edges == 0`: reject counter를 증가시키지 않는다.
- `raw_marker_edges != 0`: candidate frame이며 이후 gate 결과를 기록할 수 있다.
- `line == NULL`: marker candidate 자체를 알 수 없으므로 candidate reject로 세지 않는다.

### 6.3 frame count와 physical episode count 분리

하나의 5cm marker가 수십 frame 보일 수 있으므로 frame count만으로 marker 수를 추정하지 않는다.

권장 runtime:

```c
typedef struct {
    uint8_t active;
    uint8_t edge_union;
    uint8_t reject_reason_mask;
    uint8_t quiet_frames;
    uint32_t entry_step;
    uint32_t last_step;
} SecondDriveMarkerCandidateRuntime_t;
```

episode 시작:

- raw S0/S7 중 하나 이상 보이고 active가 false.

episode 유지:

- edge union 누적.
- 발생한 reject reason bit 누적.
- accepted Track event가 나오면 accepted flag 기록.

episode 종료:

- raw edge가 `TRACK_MARK_CLEAR_FRAMES` 이상 사라짐.
- candidate episode total 1 증가.
- accepted event면 accepted 1 증가.
- accepted되지 않았으면 rejected episode 1 증가.
- reason mask의 각 원인을 episode counter에 1회 반영.

### 6.4 reason 정의

최소:

```text
NO_LINE
BRIDGE
OFF_CENTER
NO_CENTER_MASK
CROSS_TAIL_SUPPRESSED
LOW_CONFIDENCE
COOLDOWN_OR_DUPLICATE
```

하나의 episode에 여러 reason이 있을 수 있으므로 overall rejected episode와 reason별 count 합이 같다는 invariant를 강제하지 않는다.

UI에는 frame count보다 episode count를 우선 표시한다. frame count가 필요하면 이름에 `_frames`를 명시한다.

### 6.5 First Drive와 Second Drive

마지막 close marker 누락은 First Drive map 품질 문제이므로 candidate episode 진단은 가능하면 First Drive run record에도 동일하게 남긴다.

최소 요구:

- First Drive: candidate/accepted/rejected episode와 마지막 reject reason/step.
- Second Drive: 동일 값 + local repair 성공 여부.

기존 V29 3×3 UI를 깨지 않는 범위에서 debug page 또는 run record 필드를 확장한다.

---

## 7. P1 — V30 감속 원인 로그 강화

### 7.1 V30 limiter log는 유지해야 함

V30의 `SecondDriveLimitReason_t`, `limiter_samples[]`, transition trace는 의미 있는 구현이다. 이 정보가 있어야 다음을 구분할 수 있다.

- curve profile 자체가 낮아서 느림.
- map mismatch 후 SEEK 상태가 길어서 느림.
- close marker 누락으로 pair-open 상태가 길어서 느림.
- position gate가 자주 fast를 해제함.
- 다음 TURN braking이 너무 일찍 시작함.
- recovery가 자주 발생함.
- straight target이 clamp됨.

따라서 V31은 이 구조를 제거하거나 단일 SAFE count로 축약하지 않는다.

### 7.2 reason별 episode 통계

`SecondDriveRunStats_t`에 다음을 추가한다.

```c
uint16_t limiter_episode_count[SECOND_DRIVE_LIMIT_COUNT];
uint32_t limiter_max_consecutive_samples[SECOND_DRIVE_LIMIT_COUNT];
uint32_t limiter_first_step[SECOND_DRIVE_LIMIT_COUNT];
uint32_t limiter_last_step[SECOND_DRIVE_LIMIT_COUNT];
```

RAM이 부담되면 first/last는 SAFE 계열에만 둬도 된다. `samples`는 1kHz control 기준이므로 결과 화면에서는 ms로 표시할 수 있다.

상태 갱신:

- reason이 바뀔 때 이전 streak max 갱신.
- 새 reason episode count 증가.
- 동일 reason이면 streak만 증가.
- run finalize 시 마지막 streak도 반드시 반영.

### 7.3 별도 핵심 episode

다음은 전용 요약 필드를 둔다.

```text
seek_episode_count
seek_max_ms
seek_max_steps
pair_open_episode_count
pair_open_max_ms
position_limit_episode_count
position_limit_max_ms
recovery_episode_count
recovery_max_ms
end_fallback_entry_step/sps
```

### 7.4 final reason 기록 우선순위

한 control sample에는 최종 limiter reason 하나만 누적한다.

권장 우선순위:

```text
FAULT/STOP 제외
RECOVERY
END_SAFE_BRAKE
TURN_BRAKE
MAP_INVALID/SEEK
PAIR_OPEN
POSITION/PHASE
CURVE profile
FAST profile
MAX_CLAMP은 실제 target을 낮췄을 때만
```

`END_SAFE_BRAKE`를 새 reason으로 추가한다. 기존 `END_BRAKE`가 motor stop을 의미한다면 이름 충돌을 피하도록 `END_APPROACH_SAFE`와 `END_HOLD`를 구분한다.

### 7.5 transition trace

기존 16-entry ring을 유지해도 되지만 다음을 포함해야 한다.

```text
step
reason
duration of previous reason
planner target
final target
phase
segment
sync
pair open/direction
geometry source
distance to next restriction
expected event type/index
```

구조가 너무 커지면 full snapshot 대신 compact integer field를 사용한다. ISR/control path에서 `snprintf`, LCD, 동적 할당을 사용하지 않는다.

---

## 8. P1 — First Drive 곡선 속도 조정 UI

### 8.1 목적

First Drive는 현재 board에서 곡선 속도를 바꿀 수 없고 APPROACH/TURN/EXIT가 모두 2200 SPS cap을 공유한다. V31에서는 실차에서 작은 단계로 튜닝할 수 있게 하되 기본값은 현재와 동일하게 유지한다.

### 8.2 최소 변경 설계

`FirstDriveConfig_t`에 명시적 curve setting을 추가하거나 기존 config와 별도 runtime config를 둔다.

권장:

```c
#define FIRST_DRIVE_CURVE_CRUISE_DEFAULT_SPS 2200U
#define FIRST_DRIVE_CURVE_CRUISE_MIN_SPS     2200U
#define FIRST_DRIVE_CURVE_CRUISE_MAX_SPS     2600U
#define FIRST_DRIVE_CURVE_CRUISE_STEP_SPS     100U
#define FIRST_DRIVE_CURVE_FLOOR_DELTA_SPS     400U
#define FIRST_DRIVE_CURVE_FLOOR_MIN_SPS      1800U
```

현재 V30의 `FIRST_DRIVE_CURVE_MIN_SPS`는 실제로 position envelope의 1800 SPS floor를 뜻한다. V31에서 이 이름을 새 UI의 cruise minimum으로 재사용하지 않는다. 기존 상수는 `FIRST_DRIVE_CURVE_FLOOR_MIN_SPS`처럼 의미가 명확한 이름으로 옮기고, 조정 가능한 중심 곡선 속도에는 `CURVE_CRUISE` 이름을 사용한다.

effective curve floor:

```text
max(1800, configured_curve_sps - 400)
```

예:

| configured curve | outer floor |
|---:|---:|
| 2200 | 1800 |
| 2300 | 1900 |
| 2400 | 2000 |
| 2500 | 2100 |
| 2600 | 2200 |

기존 position envelope 계산은 effective floor를 사용해 다음처럼 유지한다.

```text
envelope = base_sps
         - ((base_sps - effective_curve_floor) × abs(position)
            / SENSOR_LINE_POSITION_MAX)
```

그 뒤 phase별 cap을 적용한다. 즉 APPROACH는 `min(envelope, 2200)`, confirmed TURN/EXIT는 `min(envelope, configured_curve_sps)`다. 이렇게 해야 2200 설정에서 V30과 완전히 같고, 2400 설정에서는 중심 곡선 2400과 최외곽 2000을 얻는다.

### 8.3 phase별 정책

V31 첫 구현에서는 marker 검출과 map 품질을 보호한다.

| First Drive phase | target/cap |
|---|---|
| STRAIGHT | 기존 3820→floor position envelope |
| provisional marker / APPROACH | 기존 2200 유지 |
| confirmed TURN | configured curve SPS, position에 따라 curve floor까지 감소 |
| EXIT | configured curve SPS 이하 |
| CROSS | 기존 2400 유지 |
| recovery | 기존 1400/1800 유지 |

중요:

- `configured curve = 2200`이면 V30과 target trace가 동일해야 한다.
- APPROACH와 marker collection window는 곡선 설정을 올려도 2200을 유지한다.
- TURN confirmed 이후에만 2300~2600을 사용할 수 있다.
- recovery에는 curve setting을 적용하지 않는다.

### 8.4 UI

First Drive READY 화면에 `CURVE` 설정을 추가한다.

예:

```text
FIRST DRIVE READY
BASE 3820
CURVE 2200 FLOOR 1800
L/R ADJUST C NEXT
```

기존 PD tuning/ARM/back semantics와 충돌하지 않게 한다. 주행 중에는 변경할 수 없다.

설정은 RAM runtime 값으로 충분하다. V31에서 Flash persistence를 새로 추가하지 않는다.

### 8.5 V31 기본값 정책

firmware default는 반드시 2200/1800으로 둔다. 실차 시험에서 사용자가 직접 다음 순서로 올린다.

```text
2200 -> 2300 -> 2400
```

2400이 안정적인 것을 확인하기 전에는 2500/2600을 사용하지 않는다.

### 8.6 Second Drive curve

V30 UI가 이미 2600..3600을 지원하므로 V31에서 범위를 늘릴 필요가 없다. 기본값 3000을 유지한다.

권장 실차 순서:

```text
CURVE 3000 / ALL 100
-> CURVE 3200 / ALL 100
-> CURVE 3400 / ALL 100
-> 필요할 때 ALL 105
```

overall을 먼저 올리지 않는다. overall은 pair-open performance profile에도 적용되므로 원인 분리가 어렵다.

---

## 9. Second Drive 결과 UI

V30의 3페이지를 유지하고 정보 우선순위를 개선한다. 공간이 부족하면 stopped/fault 상태에서 4번째 debug page를 추가할 수 있다.

### 9.1 Result 1 — 주행 결과

```text
SECOND DRIVE END
TIME 48.2s STEP 171290
CTR AVG/MAX 3560/5600
SYNC M1 R1 FINAL MAP
L/R PAGE C HOLD:BACK
```

### 9.2 Result 2 — 감속 원인

```text
LIMIT SHARE 2/4
FAST62 CURVE24 BRK8
SAFE3 POS2 REC1
LONG SEEK 840ms
EP S1 P0 R2
```

표시 우선순위:

- SAFE 계열 중 최대 연속시간이 가장 긴 reason.
- 해당 duration.
- SEEK/pair/recovery episode count.

### 9.3 Result 3 — END/pair/brake

```text
END POLICY SAFE/FAST
FALL S168200 V3000
BRAKE V1800 DONE
PAIR CLOSED SRC LOCAL
EXP S171263 ERR+27
```

stop mode가 full-off fallback이면 `BRAKE FAIL OFF`를 명시한다.

### 9.4 Result 4 — marker candidate debug

공간이 부족하면 4페이지를 추가한다.

```text
MARK CANDIDATE 4/4
CAND53 ACC52 REJ1
OFF1 LINE0 CTR0 BR0
LAST OFF S162470 E01
L/R PAGE C HOLD:BACK
```

이 페이지의 count는 frame이 아니라 physical candidate episode여야 한다.

---

## 10. 권장 코드 구조

### 10.1 `Main/Inc/second_drive.h`

- `SecondDriveEndPolicy_t`.
- `SecondDriveStopMode_t`.
- `SECOND_DRIVE_LIMIT_END_APPROACH_SAFE` limiter reason.
- end fallback status/stats.
- limiter episode count/max streak.
- SEEK/pair-open duration summary.
- marker candidate episode summary getter가 필요하면 선언.

### 10.2 `Main/Src/second_drive.c`

- target 선택과 restriction cap을 분리.
- curve/pair/position early return 제거 또는 공통 finalize path로 통합.
- final END expected distance helper.
- sticky END safe fallback.
- final performance corridor는 V30 조건 유지.
- limiter episode accounting.
- SEEK/pair-open episode accounting.
- local repair semantics와 map index 불변성 유지.

권장 내부 구조:

```text
SecondDrivePlanner_ComputeNominalTarget()
SecondDrivePlanner_ComputeTurnRestriction()
SecondDrivePlanner_ComputeEndRestriction()
SecondDrivePlanner_ApplySafetyCaps()
SecondDrivePlanner_FinalizeDecision()
```

함수명은 달라도 되지만 여러 branch에서 중복 return하여 restriction을 우회하지 않게 한다.

### 10.3 `Main/Src/drive.c`, `Main/Inc/drive.h`

- confirmed END active brake를 corridor bool에서 분리.
- First Drive curve config와 phase별 target.
- marker candidate episode runtime.
- candidate accepted/rejected hook.
- First Drive run record에 candidate episode 진단.
- recovery cap과 V28 END guard 유지.

### 10.4 `Main/Src/motor.c`, `Main/Inc/motor.h`

- V30 brake state machine 유지.
- brake start 실패/full-off stop mode를 상위에 반환할 수 있게 한다.
- active hold 중 emergency/fault가 즉시 `Motor_Stop()`으로 state를 IDLE로 만드는지 test.
- hold timing과 Vref 기본값은 V31에서 변경하지 않는다.

### 10.5 `Main/Src/menu.c`

- First Drive CURVE 설정 UI.
- V31 Second Drive result pages.
- active LCD freeze 유지.
- stopped 상태에서만 page navigation/설정 변경.

### 10.6 `Main/Src/track.c`, `Main/Inc/track.h`

- 기존 pair segment semantics 변경 금지.
- candidate episode를 track collector에서 더 정확히 알 수 있을 때 최소 getter 추가 가능.
- original map event/segment/anchor 배열 구조와 capacity 변경은 가급적 피한다.

### 10.7 `Main/Inc/app_version.h`

```c
#define APP_VERSION_NUMBER 31U
```

버전 변경은 구현과 test가 모두 끝난 뒤 한다.

---

## 11. 구현 순서

순서를 바꾸지 않는 것을 권장한다.

1. 현재 V30 source clean/incremental ARM build baseline 확보.
2. V30 `/private/tmp` harness를 프로젝트 내부 재현 가능한 test로 옮기거나 build script화.
3. 기존 V30 positive harness를 현재 source로 재빌드해 baseline PASS.
4. 실패해야 하는 V31 END fallback test를 먼저 작성.
5. `SecondDrivePlanner_GetTargetSps()`를 nominal/restriction/finalize 단계로 정리.
6. pair-open/position/phase final END fallback test 통과.
7. confirmed END active brake를 corridor에서 분리.
8. active brake failure/emergency/fault test 통과.
9. raw marker candidate episode 진단 구현.
10. marker 없는 curve frame에서 reject 0 test 통과.
11. limiter episode/max duration 진단 구현.
12. V27 resync를 현재 source로 재빌드하는 test 추가.
13. local repair negative test 추가.
14. First Drive curve config와 기본값 회귀 test 구현.
15. First Drive CURVE UI 구현.
16. Second Drive result UI 개선과 active LCD render test.
17. 전체 host test와 ARM clean build.
18. Flash/RAM/stack 증가량 기록.
19. 앱 버전 31 확정.
20. 작업 결과 문서 작성.

속도 기본값 변경은 위 구현 순서에 포함하지 않는다.

---

## 12. 필수 host/mock test

### 12.1 END safe fallback

1. 정상 final corridor
   - final straight target 유지.
   - END long pre-deceleration 없음.
   - confirmed END에서 active brake.
2. pair-open final END
   - braking window 밖에서 pair-open safe target.
   - braking window 진입 시 1800 cap.
   - fallback sticky 유지.
   - confirmed END에서 active brake.
3. local repair 이후 EXIT 49 frame
   - performance corridor 아직 false.
   - END fallback restriction은 우회되지 않음.
4. local repair 이후 EXIT 50 frame
   - final performance corridor true.
   - straight target 허용.
5. position gate/fast gate not-ready
   - final END expected이면 END fallback 계산 수행.
6. map SEEK
   - map distance를 신뢰하지 않고 First Drive fallback.
   - confirmed V28 END에서는 active brake.
7. fallback active 후 line centered
   - END 전 다시 fast로 올라가지 않음.

### 12.2 active brake

- corridor true confirmed END: HARD→REDUCED→OFF.
- corridor false confirmed END: 동일 active brake.
- pair open confirmed END: 동일 active brake.
- brake start 실패: 즉시 full off, stop mode 기록.
- HARD 중 emergency: 즉시 full off.
- REDUCED 중 fault: 즉시 full off.
- raw BOTH/V28 reject: brake 시작 안 함.
- repeated stop/brake call idempotent.

### 12.3 marker candidate episode

- raw edge 0 + off-center 1000 frame: candidate/reject 0.
- raw edge 있음 + off-center 30 frame: candidate 1, reject episode 1, OFF_CENTER 1.
- raw edge 있음 + reason이 중간에 NO_LINE으로 변경: candidate 1, rejected 1, reason mask 두 종류.
- accepted direction event: candidate 1, accepted 1, rejected 0.
- CROSS-tail merged: accepted/suppressed semantics가 중복 집계되지 않음.
- candidate clear debounce가 한 physical marker를 여러 episode로 쪼개지 않음.

### 12.4 local repair negative test

- no open pair.
- wrong-side direction.
- ordinary LEFT/RIGHT expected.
- low confidence.
- duplicate/cooldown.
- map invalid.
- SEEK state.
- segment direction mismatch.

모든 거절에서 expected index, segment start, map arrays 불변을 확인한다.

### 12.5 V27 resync current-source regression

현재 `track.c`와 `second_drive.c`로 fixture를 다시 build한다.

```text
normal mismatch
-> SEEK CROSS
-> forward CROSS candidate
-> unambiguous anchor
-> MAP sync
-> expected index/segment/anchor order 정상
-> replay pair reset
-> stable gate 이후 fast 재진입
```

예전 binary 실행만으로 PASS 처리하지 않는다.

### 12.6 limiter episode invariant

- control sample당 histogram sample 정확히 1개.
- 동일 reason 100 sample: episode 1, max 100.
- A10→B20→A30: A episode 2, A max 30, B episode 1, B max 20.
- run finalize 시 마지막 active streak 반영.
- recovery가 planner reason을 최종적으로 덮을 때 RECOVERY만 1개 누적.

### 12.7 First Drive curve default regression

configured curve 2200에서 V30과 동일 sensor trace를 입력한다.

- STRAIGHT target 동일.
- APPROACH 2200 cap 동일.
- TURN 2200 cap 동일.
- EXIT 2200 cap 동일.
- outer floor 1800 동일.
- marker/CROSS/recovery 동일.

설정 2400 fixture:

- APPROACH 2200 유지.
- confirmed TURN center 2400.
- outer floor 2000.
- recovery 1400/1800 유지.

### 12.8 UI/LCD

- First Drive READY에서 CURVE 2200..2600, step100.
- active run에서 설정 변경 불가.
- Second Drive active periodic render count 0.
- STOP/FAULT에서 결과 page 이동.
- candidate episode와 longest limiter가 올바른 page에 표시.
- center emergency는 LCD freeze와 무관하게 동작.

---

## 13. ARM build와 정적 확인

```bash
cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --clean-first -j 8
```

확인:

- V31 변경 파일 warning/error 없음.
- 기존 ST7735 unused `text` warning은 별도 기존 warning으로 기록.
- control ISR에 새 64-bit division 없음.
- LCD API와 `snprintf`가 ISR/control path에 없음.
- brake path에 `HAL_Delay()` 없음.
- limiter episode update O(1).
- marker candidate runtime에 동적 할당 없음.
- Flash/RAM/stack V30 대비 증감 기록.
- `git diff --check` 통과.

---

## 14. 실차 시험 순서

V31은 V30을 건너뛰어 번호를 올리지만, 첫 실차 시험은 속도 상향 시험이 아니라 V30+V31 통합 검증이다.

### 14.1 스탠드

1. First Drive CURVE 2200 기본값 확인.
2. Second Drive 5600/3000/100% 확인.
3. confirmed END 입력에서 corridor false/true 모두 active brake 확인.
4. brake hold 중 center emergency full off 확인.
5. 주행 화면 redraw lock 확인.

### 14.2 저속 marker 기록

실제 final sequence:

```text
R -> R -> R -> L -> R -> L -> L -> END
```

확인:

- 마지막 close L candidate episode.
- accepted/rejected 여부.
- reject reason과 step.
- First Drive finalized pair closed 여부.

### 14.3 END fallback 전용

1. 마지막 close가 없는 fixture/map.
2. Second Drive에서도 close를 놓치는 조건.
3. Result에서 `PAIR OPEN` 확인.
4. END 접근 braking window에서 `END_SAFE`와 1800 target 확인.
5. END confirmed 후 active brake 확인.

### 14.4 V30 기본 성능 검증

```text
First curve 2200
Second straight 5600
Second curve 3000
Overall 100
```

확인:

- CROSS 전 fast corridor.
- CROSS resync.
- final fast corridor/local repair.
- limiter share와 longest episode.
- END stop error.

### 14.5 Second Drive curve tuning

V30 기본 run이 성공한 뒤:

```text
3000 -> 3200 -> 3400
```

각 단계에서 비교:

- lap time.
- CURVE limiter share.
- line loss/recovery episode.
- max edge dwell.
- outer boost 빈도.
- wheel 6500 clamp.
- marker candidate reject.

### 14.6 First Drive curve tuning

Second Drive와 별도 run으로:

```text
2200 -> 2300 -> 2400
```

각 단계에서 marker map 품질과 last close acceptance를 먼저 확인한다. First Drive 속도 향상으로 marker 누락이 증가하면 즉시 이전 값으로 돌아간다.

### 14.7 safety 속도는 마지막

MAP/SEEK 또는 recovery가 실제 랩타임의 큰 비율을 차지한다는 로그가 확인되기 전에는 safety 속도를 올리지 않는다.

SAFE 시간이 길다면 먼저 원인을 수정한다.

- mismatch가 원인: marker matching/resync 확인.
- pair open이 원인: close marker 후보/reject 확인.
- position이 원인: gate threshold가 아니라 line following/phase 확인.
- recovery가 원인: line loss 원인 확인.

---

## 15. 완료 조건

- [ ] 앱 버전이 31이다.
- [ ] Second Drive END 전 성능 corridor와 END 정지 정책이 분리됐다.
- [ ] pair-open final END에서 braking window 진입 시 1800 SPS cap이 실제 적용된다.
- [ ] position/phase/fast-gate 불성립 상태에서도 END fallback이 early return으로 우회되지 않는다.
- [ ] END fallback은 한번 활성화되면 END까지 sticky다.
- [ ] V28-confirmed Second Drive END는 corridor 상태와 무관하게 active brake를 시도한다.
- [ ] brake 시작 실패는 즉시 full off되고 stop mode에 기록된다.
- [ ] emergency/fault는 hold 상태와 무관하게 즉시 full off한다.
- [ ] raw marker edge가 없는 일반 frame은 marker reject episode를 증가시키지 않는다.
- [ ] 하나의 physical marker는 candidate episode 1개로 집계된다.
- [ ] accepted/rejected candidate와 reason/step을 결과에서 확인할 수 있다.
- [ ] limiter reason histogram은 V30과 같이 control sample당 하나를 유지한다.
- [ ] reason별 episode count와 최대 연속시간이 정확하다.
- [ ] SEEK/pair-open/recovery 최대 지속시간을 확인할 수 있다.
- [ ] V27 resync test가 현재 V31 source로 재빌드되어 PASS한다.
- [ ] local repair 모든 negative case가 PASS한다.
- [ ] V30 pair/local repair positive test가 유지된다.
- [ ] First Drive CURVE 2200 설정은 V30 target trace와 동일하다.
- [ ] First Drive CURVE를 2200..2600 범위에서 board UI로 조정할 수 있다.
- [ ] APPROACH/marker/recovery 기본 안전속도는 curve 설정을 올려도 유지된다.
- [ ] Second Drive V30 기본 profile 5600/3000/100%가 유지된다.
- [ ] active LCD redraw lock이 유지된다.
- [ ] host/mock test와 ARM clean build가 통과한다.
- [ ] V30 대비 Flash/RAM/stack 증감이 기록된다.
- [ ] 실차 전 기본 속도를 임의로 상향하지 않았다.

---

## 16. Luna가 작성할 작업 결과 문서

구현 완료 후 다음 파일을 작성한다.

`codex_worked_review/V31_END_SAFETY_SPEED_TUNING_AND_DIAGNOSTIC_TRUST_WORK_RESULT.md`

반드시 포함:

1. 변경 파일 목록과 파일별 변경 내용.
2. `SecondDrivePlanner_GetTargetSps()`의 V30/V31 결정 순서 비교.
3. curve/pair/position early return을 제거하거나 안전하게 통합한 실제 symbol.
4. pair-open final END fallback test의 거리·target 변화 원문.
5. END fallback sticky test 결과.
6. corridor false confirmed END active brake test 결과.
7. brake failure/emergency/fault full-off test 결과.
8. V28 END guard를 유지했다는 코드 근거.
9. marker candidate episode state machine과 reason priority/mask.
10. marker edge 없는 off-center 1000 frame에서 reject 0인 test.
11. candidate accepted/rejected episode test.
12. limiter episode count/max duration 구현 symbol과 invariant test.
13. 현재 source로 재빌드한 V27 SEEK→CROSS resync 결과.
14. local repair negative test 전체 결과.
15. First Drive curve UI 범위와 기본값.
16. curve 2200 V30 회귀와 curve 2400 target fixture.
17. Second Drive profile 기본값을 변경하지 않았다는 근거.
18. 종료 결과 화면 실제 문자열 또는 사진.
19. active LCD render count test.
20. 전체 host/mock test 명령과 PASS 원문.
21. ARM clean build 명령과 결과.
22. V30 대비 Flash/RAM/stack 증감.
23. 남은 실차 위험과 정확한 실차 시험 순서.

단순히 “구현 완료”라고 쓰지 않는다. Codex가 다시 검토할 수 있도록 실제 함수명, enum, 상수, test input/output을 기록한다.

---

## 17. 최종 주의사항

- V31의 최우선 목표는 속도 숫자 증가가 아니라 END safety correctness다.
- final performance corridor false는 active brake 금지 조건이 아니다.
- confirmed END와 raw BOTH를 혼동하지 않는다.
- END fallback은 nominal phase target을 선택한 뒤 restriction cap으로 적용한다.
- pair-open 상태에서 overall 120%가 적용될 수 있으므로 SAFE라는 이름만 보고 저속이라고 가정하지 않는다.
- recovery 속도는 실차 데이터 없이 올리지 않는다.
- marker reject count는 raw candidate가 있을 때만 의미가 있다.
- frame count와 physical marker episode count를 섞지 않는다.
- V30 limiter log는 유지한다. 속도 튜닝의 근거가 되는 핵심 계측이다.
- First Drive curve 설정 기본값은 2200으로 유지한다.
- Second Drive curve 설정 기본값은 3000으로 유지한다.
- 속도 설정과 PD/FF/steer limit을 같은 단계에서 동시에 크게 변경하지 않는다.
- 기존 `track.c` pair FSM과 V27/V28/V29 의미를 바꾸지 않는다.
- local repair는 expected index, segment start, 원본 map arrays를 수정하지 않는다.
- test는 `/private/tmp` binary 실행만으로 회귀 PASS 처리하지 않는다.
- 주행 중 LCD freeze와 center emergency 처리를 회귀시키지 않는다.
