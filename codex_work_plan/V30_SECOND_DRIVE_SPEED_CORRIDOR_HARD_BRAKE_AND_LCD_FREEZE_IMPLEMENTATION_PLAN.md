# V30 Second Drive 고속 Corridor·코너 속도·END Hard Brake·LCD Freeze 구현 계획

## 0. 문서 목적

이 문서는 `line_tracer_2026` 프로젝트의 **V29를 기반으로 V30을 구현하는 Luna Max용 단독 인수인계 문서**다. Luna는 사용자와 Codex가 나눈 이전 대화를 볼 수 없다고 가정한다.

V30의 핵심 목표는 다음과 같다.

1. 현재 2차 주행이 느린 정확한 이유를 매 control tick에서 기록할 수 있게 한다.
2. 교차로가 있는 긴 직선에서 첫 CROSS를 통과하기 전부터 안전하게 고속 주행한다.
3. 마지막 END 전 긴 직선을 LEFT/RIGHT 구간으로 잘못 취급해 저속 주행하는 문제를 해결한다.
4. 현재 120% TURN 속도인 2640 SPS보다 높은 속도를 V30의 기본 코너 속도로 사용하되, 코너 추종 안전장치는 유지한다.
5. END 앞에서 긴 거리 동안 1800~2160 SPS로 미리 기어가는 동작을 제거하고, 검증된 END가 확정되면 즉시 hard brake를 건다.
6. 2차 주행 중 LCD의 100ms 주기 redraw를 중단해 깜빡임과 SPI/CPU 부하를 없앤다.
7. 1차 주행, V27 anchor resync, V28 CROSS-tail/END 방어, V29 결과 로그를 회귀시키지 않는다.

> 구현 기준은 현재 작업 디렉터리의 V29 소스다. 작업 트리가 깨끗하지 않으며 V27~V29 구현 변경과 문서가 포함되어 있을 수 있다. `git reset`, `git restore`, `git checkout --` 등으로 기존 작업을 되돌리지 말고 현재 상태 위에 V30 변경만 추가한다.

---

## 1. 현재 상태와 실차 결과

### 1.1 프로젝트

- 프로젝트 루트: `/Users/ehoi/STM32CubeIDE/line_tracer_2026`
- 현재 펌웨어 버전: V29
- 버전 정의: `Main/Inc/app_version.h`
- V29 계획: `codex_work_plan/V29_FIRST_DRIVE_RUN_RECORD_AND_LOG_UI_IMPLEMENTATION_PLAN.md`
- V29 결과: `codex_worked_review/V29_FIRST_DRIVE_RUN_RECORD_AND_LOG_UI_WORK_RESULT.md`
- V28 결과: `codex_worked_review/V28_SPLIT_CROSS_TAIL_FUSION_WORK_RESULT.md`
- V27 결과: `codex_worked_review/V27_CROSS_ANCHOR_RESYNC_WORK_RESULT.md`

### 1.2 V29 실차 확인 결과

사용자가 제공한 First Drive 종료 기록:

```text
TIME 70859ms STEP 171307
TOTAL 135 CROSS 26 END 1
SEG 135 ANC 26 MAP OK
START1 LEFT50 RIGHT57 CROSS26 END1 UNK0
LOSS E11 MAX70ms REC11 LOSTF114
TAIL E1 C1 GAP50/160 REJ0 OVR0/0
CTR AVG2414 MAX3820 TARGET MAX3820
WHEEL MAX L4566 R3985
```

최근 마커:

```text
NEW>OLD
END   S171263
LEFT  S162476
RIGHT S162078
LEFT  S161150
```

종료 snapshot:

```text
STATE STOP PH XL
POS -95 LAST -95
V2200>2200 L/R2200/2200
FAULT NONE MAP1
```

판단:

- marker total invariant가 성립한다.
  - `1 + 50 + 57 + 26 + 1 + 0 = 135`
- CROSS 26개와 anchor 26개가 일치한다.
- unknown, END reject, track/anchor overflow가 모두 0이다.
- loss 11회를 모두 복구했다.
- V29 map은 구조적으로 정상이다.
- 2차 주행에서는 V28 이전과 달리 CROSS 자체를 통과할 때 2400 SPS로 감속하지 않고 정상적으로 달렸다.

그러나 `MAP OK`는 배열/anchor/END 구조가 일관된다는 의미다. 모든 물리적 직선과 코너가 정확히 분류되었다는 뜻은 아니다.

---

## 2. 현재 2차 주행이 느린 이유

### 2.1 현재 속도 상수와 120%의 실제 효과

현재 주요 값:

```text
Second Drive straight default       5200 SPS
Second Drive straight adjustable    4000..6200 SPS
Overall percent                     80..120%, default 100%
First Drive centered base           3820 SPS
First Drive turn cap                2200 SPS
First Drive marker/cross cap        2400 SPS
Second Drive END approach           1800 SPS
Recovery straight/turn              1400/1800 SPS
Motor absolute max                  6500 SPS
```

현재 120%를 선택했을 때:

| 상황 | 최종 대표값 |
|---|---:|
| fast STRAIGHT/CROSS | 5200 SPS, overall이 적용되지 않음 |
| 일반 중앙 1차 기반 target | 3820 × 1.2 = 4584 SPS |
| APPROACH/TURN/EXIT | 2200 × 1.2 = 2640 SPS |
| marker 제한 | 2400 × 1.2 = 2880 SPS |
| END 접근 | 1800 × 1.2 = 2160 SPS |
| recovery straight/turn | 1400/1800 SPS, overall 적용 후 다시 제한됨 |
| SEEK CROSS/map invalid | 1차 target 그대로, overall도 적용되지 않음 |

즉, 현재 `ALL SPEED 120%`는 이름과 달리 모든 속도를 120%로 만드는 옵션이 아니다. straight 설정값에는 적용되지 않고, recovery와 sync fallback에도 적용되지 않는다.

### 2.2 fast speed 허용 조건이 너무 좁다

현재 5200 SPS fast target은 아래를 모두 만족해야 한다.

1. map valid.
2. sync state가 `SECOND_DRIVE_SYNC_MAP`.
3. 현재 map segment가 STRAIGHT 또는 CROSS.
4. 현재 course phase가 segment에 허용되는 phase.
5. `abs(line_position) <= 700`.
6. 다음 TURN/END restriction까지 제동 거리가 충분함.
7. line recovery 제한이 걸리지 않음.

조건 하나라도 실패하면 1차 주행 기반 target으로 내려간다.

### 2.3 CROSS 전 직선이 느린 구조적 이유

`Track_FinalizeSegments()`는 `event[i]`의 type으로 `event[i]`부터 `event[i+1]`까지 segment type을 만든다.

```text
previous directional marker ---- CROSS ---- next marker
       LEFT/RIGHT segment           CROSS segment
```

따라서 물리적으로 CROSS 전후가 모두 긴 직선이어도 CROSS를 통과하기 전에는 이전 방향 마커가 만든 LEFT/RIGHT segment로 남을 수 있다. CROSS를 실제로 통과해 planner index가 다음 segment로 이동한 뒤에야 CROSS fast geometry가 된다.

이 현상은 사용자가 관찰한 “첫 CROSS를 지난 뒤부터 가속”과 정확히 일치한다.

### 2.4 마지막 직선이 느린 이유

시립대 규정의 턴 마커 의미는 다음과 같다.

- 턴 마커는 주행선에서 4cm 떨어져 회전 방향에 놓인다.
- 곡선 시작점에는 앞으로 회전할 방향의 마커가 있다.
- 곡선 종료점에는 마지막 회전 방향과 같은 방향의 마커가 있다.
- 곡률반경은 25cm로 고정되어 있다.
- 회전 방향은 연속해서 바뀔 수 있으며, 각 경계 마커는 해당 회전 방향을 나타낸다.
- 교차로 중심 전후 최소 20cm는 직선이다.

따라서 같은 방향 마커 한 쌍은 아래 의미다.

```text
R start ---- RIGHT curve ---- R end ---- STRAIGHT
L start ---- LEFT  curve ---- L end ---- STRAIGHT
```

현재 `Track_SegmentTypeFromEvent()`의 기본 FSM은 이 규정과 맞는다.

- 첫 방향 마커: 해당 방향 turn open, 그 마커부터 다음 마커까지 LEFT/RIGHT.
- 열린 turn과 같은 방향 마커: turn close, 그 마커부터 다음 마커까지 STRAIGHT.
- 열린 turn과 반대 방향 마커: 새 방향 curve로 전환.
- CROSS: open turn state reset.

사용자가 확인한 실제 마지막 마커열은:

```text
R1 -> R2 -> R3 -> L1 -> R4 -> L2 -> L3 -> END
```

정상적으로 모두 기록되면 segment는:

```text
R1-R2 RIGHT
R2-R3 STRAIGHT
R3-L1 RIGHT
L1-R4 LEFT
R4-L2 RIGHT
L2-L3 LEFT
L3-END STRAIGHT
```

즉, 정상 marker sequence에서 현재 FSM은 마지막 직선을 STRAIGHT로 만든다.

그러나 V29 최근 로그는:

```text
LEFT S161150 -> RIGHT S162078 -> LEFT S162476 -> END S171263
```

였다. 실제 끝부분 `... L1 -> R4 -> L2 -> L3 -> END`에서 마지막 곡선 종료 마커 `L3`를 놓치면 기록이 `... L1 -> R4 -> L2 -> END`가 되어 V29 로그와 정확히 일치한다. 이 경우 L2가 연 LEFT turn을 닫아 줄 L3가 없으므로 마지막 약 8787 step이 LEFT로 남는다. 종료 snapshot의 `PH XL(EXIT_LEFT)`, target 2200 SPS도 unmatched close marker 가설과 일치한다.

따라서 V30의 원인 정의는 다음으로 고정한다.

```text
정상 pair 분류 규칙 자체의 오류가 아니라,
1차 주행에서 마지막 동일 방향 curve-close 마커가 누락되어
turn_open과 LEFT segment가 END까지 유지된 것이 가장 유력하다.
```

현재 방식은 모든 방향 마커가 정확히 저장된다는 전제에서는 맞지만, 한 마커가 빠지면 pair parity와 open-turn state가 CROSS 또는 명시적 close까지 틀어진다는 구조적 약점이 있다. V30은 pair FSM을 뒤집지 말고, 누락 진단과 2차 주행 local close repair를 추가해야 한다.

### 2.5 현재 END 동작

현재 planner는 END를 다음 restriction으로 발견하면 `1800 × overall`까지 미리 감속한다. END marker event가 V28 조건으로 확정된 뒤에는 `Motor_DriveStop()`을 즉시 호출한다.

그러나 현재 `Motor_DriveStop()`은:

- motor timer 중단
- Vref 0
- coil output off

를 수행한다. 명령상 즉시 stop이지만 holding torque를 이용한 active brake는 아니다.

### 2.6 LCD 깜빡임

`Menu_UpdateSecondDrive()`가 `SECOND_DRIVE_UPDATE_MS = 100`마다 `Menu_RenderSecondDrive()`를 호출한다. render 함수는 약 232×94 영역을 검게 지운 뒤 텍스트를 전부 다시 그린다. 주행 중 10Hz full-area redraw가 깜빡임과 불필요한 LCD 전송 부하를 만든다.

---

## 3. V30 범위와 비범위

### 3.1 반드시 구현할 범위

1. Second Drive speed decision reason 및 누적 통계.
2. true overall scaling semantics.
3. 별도 straight/curve 속도 설정과 V30 상향 기본 profile.
4. CROSS 접근 fast corridor.
5. 방향 marker pair/open-turn 진단과 curve-close marker 누락 검출.
6. 2차 주행에서 맵에 없던 동일 방향 close marker를 발견했을 때 local geometry repair.
7. local repair 또는 정상 close pair로 확인된 마지막 END 접근 fast corridor.
8. fast gate의 position hysteresis와 centered stability.
9. TURN 접근 제동 유지 및 limiter reason 기록.
10. END를 장거리 저속 restriction으로 사용하지 않는 hard-end mode.
11. V28-confirmed END 직후 비차단 active brake hold.
12. 주행 중 LCD redraw 중단, 상태 전이 시 1회 redraw.
13. Second Drive 종료 후 속도 제한·pair repair 원인을 확인할 수 있는 compact 결과 화면 또는 telemetry snapshot.
14. 앱 버전 30.

### 3.2 변경하지 않을 범위

- 1차 주행 속도, PD, phase FSM, line recovery/fault threshold.
- V27 CROSS anchor 구조와 forward resync 조건.
- V28 CROSS-tail merge 및 END guard 조건.
- V29 First Drive run record와 3×3 결과 UI.
- track event/segment/anchor RAM capacity.
- motor run current `MOTOR_VREF_DAC_RUN`.
- 센서 sampling/control 주기.

현재 marker pair 규칙을 반대로 뒤집거나 `current-next` 단순 비교식으로 교체하지 않는다. 규정상 첫 동일 방향 마커부터 두 번째 동일 방향 마커까지가 곡선이고, 두 번째 마커 이후가 직선이므로 현재 open/close 방향이 맞다.

V30의 새 speed planner와 hard brake는 원칙적으로 `DRIVE_RUN_SECOND`에서만 동작해야 한다.

---

## 4. V30 속도 profile

### 4.1 새 기본값

V30의 100% 기본 profile을 아래 값으로 시작한다.

```c
#define SECOND_DRIVE_DEFAULT_STRAIGHT_SPS        5600U
#define SECOND_DRIVE_DEFAULT_CURVE_SPS           3000U
#define SECOND_DRIVE_APPROACH_BONUS_SPS           300U
#define SECOND_DRIVE_EXIT_BONUS_SPS               800U
#define SECOND_DRIVE_PERFORMANCE_MAX_CENTER_SPS  6200U
#define SECOND_DRIVE_DEFAULT_OVERALL_PERCENT      100U
```

100%에서 계산:

```text
STRAIGHT/CROSS     5600 SPS
APPROACH           3300 SPS
CURVE cruise       3000 SPS
EXIT               3800 SPS
```

현재 V29의 120% TURN 2640 SPS보다 V30 100% curve 3000 SPS가 높다. 사용자의 “기본속도부터 현재 120% 이상” 요구를 충족한다.

이 값은 첫 실차 시험용 시작값이다. Luna가 임의로 더 높이지 않는다. 실차 시험에서는 curve를 100~200 SPS 단위로 올리거나 내린다.

### 4.2 UI 조정 범위

```c
#define SECOND_DRIVE_STRAIGHT_MIN_SPS      4800U
#define SECOND_DRIVE_STRAIGHT_MAX_SPS      6000U
#define SECOND_DRIVE_STRAIGHT_STEP_SPS      100U

#define SECOND_DRIVE_CURVE_MIN_SPS         2600U
#define SECOND_DRIVE_CURVE_MAX_SPS         3600U
#define SECOND_DRIVE_CURVE_STEP_SPS         100U

#define SECOND_DRIVE_OVERALL_MIN_PERCENT     90U
#define SECOND_DRIVE_OVERALL_MAX_PERCENT    120U
#define SECOND_DRIVE_OVERALL_STEP_PERCENT     5U
```

`SecondDriveConfig_t`:

```c
typedef struct {
    uint16_t straight_sps;
    uint16_t curve_sps;
    uint8_t overall_percent;
} SecondDriveConfig_t;
```

board 설정 순서:

```text
STRAIGHT -> CURVE -> ALL SPEED -> ARM
```

- L/R: 현재 항목 조절.
- C short: 다음 항목.
- 마지막 ALL SPEED 항목에서 C short: ARM.
- C hold: 기존 back/emergency semantics 유지.

### 4.3 overall 의미 수정

V30에서는 overall이 performance profile 전체에 실제로 적용되어야 한다.

```text
effective straight = straight_sps × overall / 100
effective curve     = curve_sps × overall / 100
effective approach  = (curve_sps + 300) × overall / 100
effective exit      = (curve_sps + 800) × overall / 100
```

effective center target은 `SECOND_DRIVE_PERFORMANCE_MAX_CENTER_SPS = 6200`으로 clamp한다. wheel별 최종 clamp는 기존 motor absolute max 6500을 유지한다.

READY 화면에는 nominal과 effective가 혼동되지 않게 표시한다.

예:

```text
STRAIGHT B5600 E5600
CURVE    B3000 E3000
ALL SPEED 100%
```

120%에서 straight가 6200으로 clamp되면 `E6200 MAX`처럼 표시한다.

### 4.4 안전 속도는 overall과 분리

다음은 performance tuning과 분리한다.

- line recovery straight/turn 1400/1800 SPS.
- map invalid/SEEK CROSS의 기존 First Drive fallback.
- fault stop.
- emergency stop.

V30에서 recovery 속도를 올리지 않는다. recovery는 속도 향상 대상이 아니라 안전 복구 상태다.

---

## 5. Speed decision reason과 계측

V30 속도 튜닝은 “현재 target 숫자”만으로는 부족하다. 왜 제한됐는지를 반드시 기록한다.

### 5.1 enum

속도 제한 원인과 geometry 근거를 분리한다.

```c
typedef enum {
    SECOND_DRIVE_GEOMETRY_MAP_SEGMENT = 0,
    SECOND_DRIVE_GEOMETRY_PAIR_OPEN,
    SECOND_DRIVE_GEOMETRY_PAIR_CLOSE,
    SECOND_DRIVE_GEOMETRY_CROSS_RESET,
    SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR,
    SECOND_DRIVE_GEOMETRY_UNCERTAIN
} SecondDriveGeometrySource_t;

typedef enum {
    SECOND_DRIVE_LIMIT_NONE = 0,
    SECOND_DRIVE_LIMIT_FAST_STRAIGHT,
    SECOND_DRIVE_LIMIT_FAST_CROSS_APPROACH,
    SECOND_DRIVE_LIMIT_FAST_CROSS_EXIT,
    SECOND_DRIVE_LIMIT_FAST_END_CORRIDOR,
    SECOND_DRIVE_LIMIT_FAST_LOCAL_CLOSE_REPAIR,
    SECOND_DRIVE_LIMIT_CURVE_APPROACH,
    SECOND_DRIVE_LIMIT_CURVE_CRUISE,
    SECOND_DRIVE_LIMIT_CURVE_EXIT,
    SECOND_DRIVE_LIMIT_MAP_INVALID,
    SECOND_DRIVE_LIMIT_SEEK_CROSS,
    SECOND_DRIVE_LIMIT_SEGMENT_UNCERTAIN,
    SECOND_DRIVE_LIMIT_MARKER_PAIR_UNCLOSED,
    SECOND_DRIVE_LIMIT_PHASE_NOT_STRAIGHT,
    SECOND_DRIVE_LIMIT_POSITION,
    SECOND_DRIVE_LIMIT_TURN_BRAKE,
    SECOND_DRIVE_LIMIT_RECOVERY,
    SECOND_DRIVE_LIMIT_MAX_CLAMP,
    SECOND_DRIVE_LIMIT_END_BRAKE
} SecondDriveLimitReason_t;
```

명칭은 기존 style에 맞게 조정할 수 있지만 의미를 합치지 않는다. map segment가 무엇인지와 실제 replay marker pair가 무엇을 확인했는지를 별도 필드로 남긴다.

### 5.2 status 확장

`SecondDrivePlannerStatus_t`에 최소 다음을 추가한다.

```c
uint16_t nominal_target_sps;
uint16_t planner_target_sps;
uint16_t final_target_sps;
SecondDriveLimitReason_t limit_reason;
SecondDriveGeometrySource_t geometry_source;
uint8_t fast_gate_ready;
uint8_t cross_approach_corridor;
uint8_t final_end_corridor;
uint8_t replay_turn_open;
int8_t replay_turn_direction;
uint8_t local_close_repair_active;
uint16_t local_close_repair_count;
uint16_t marker_pair_mismatch_count;
uint16_t centered_stable_frames;
uint32_t replay_turn_open_step;
uint32_t local_close_repair_step;
uint32_t expected_marker_distance_steps;
```

### 5.3 누적 run stats

```c
typedef struct {
    uint8_t valid;
    uint32_t elapsed_ms;
    uint32_t control_samples;
    uint64_t center_sps_sum;
    uint16_t center_sps_max;
    uint16_t target_sps_max;
    uint32_t limiter_samples[SECOND_DRIVE_LIMIT_COUNT];
    uint16_t fast_entry_count;
    uint16_t fast_exit_count;
    uint16_t mismatch_count;
    uint16_t resync_count;
    uint16_t local_close_repair_count;
    uint16_t unmatched_turn_at_end_count;
    uint16_t marker_reject_no_line_count;
    uint16_t marker_reject_off_center_count;
    uint16_t marker_reject_no_center_mask_count;
    uint16_t marker_reject_bridge_count;
    uint32_t end_brake_step;
    uint16_t end_brake_entry_sps;
    uint16_t brake_hold_ms;
    uint8_t end_brake_completed;
} SecondDriveRunStats_t;
```

- 1kHz control path에서는 O(1) increment/add/max만 한다.
- division과 percent 계산은 stop/menu에서 한다.
- limiter sample은 최종 적용 target을 결정한 가장 강한 원인으로 1 tick에 정확히 1개만 증가시킨다.
- line recovery가 planner 이후 target을 낮췄다면 최종 reason은 RECOVERY다.
- max clamp가 실제 target을 낮췄다면 MAX_CLAMP를 기록하되, 원래 planner reason도 status에 보존할 수 있다.
- local close repair는 mismatch/resync와 별도 집계한다.
- END 도착 시 replay turn이 아직 열려 있으면 unmatched turn count와 방향을 보존한다.
- marker reject count는 threshold를 완화하기 위한 값이 아니라, 실제 close marker가 어느 gate에서 사라졌는지 확인하기 위한 진단값이다.

### 5.4 limiter transition trace

최근 제한 전환 12~16개를 ring buffer로 남긴다.

```c
typedef struct {
    uint32_t step;
    uint16_t segment_index;
    TrackSegmentType_t segment_type;
    FirstDriveCoursePhase_t phase;
    SecondDriveLimitReason_t reason;
    SecondDriveGeometrySource_t geometry_source;
    uint8_t replay_turn_open;
    int8_t replay_turn_direction;
    uint16_t requested_sps;
    uint16_t final_sps;
    int16_t line_position;
} SecondDriveLimitTraceEntry_t;
```

reason이 바뀔 때만 기록한다. 매 tick 기록하지 않는다.

---

## 6. Fast gate hysteresis

현재는 `abs(position) > 700`인 한 frame만으로 fast target을 취소한다. V30에서는 진입과 해제를 분리한다.

```c
#define SECOND_DRIVE_FAST_ENTER_POSITION       500
#define SECOND_DRIVE_FAST_EXIT_POSITION        900
#define SECOND_DRIVE_FAST_STABLE_FRAMES         30U
```

동작:

- fast gate OFF 상태:
  - line valid.
  - course phase STRAIGHT.
  - `abs(position) <= 500`이 30개 연속 control frame.
  - 만족하면 fast gate ON.
- fast gate ON 상태:
  - line invalid 즉시 OFF.
  - phase가 STRAIGHT/CROSS 허용 상태가 아니면 해당 profile로 전환.
  - `abs(position) >= 900`이면 OFF.
- `500 < abs(position) < 900`은 이전 상태 유지.

30 frame은 1kHz control 기준 약 30ms다. 단일 position noise로 속도가 흔들리는 것을 방지한다.

CROSS pass에서는 기존 straight-through 제어가 활성화되고 line이 유효하다면 gate를 유지할 수 있다. TURN/APPROACH/EXIT에서는 fast straight gate를 강제로 사용하지 않고 각 phase profile을 적용한다.

---

## 7. CROSS 접근 fast corridor

### 7.1 원칙

CROSS marker는 물리적으로 straight-through 지점이다. map sync가 정상이고 다음 expected marker가 CROSS라면, 현재 map segment가 LEFT/RIGHT로 남아 있어도 live controller가 이미 안정된 STRAIGHT를 확인한 이후 구간은 CROSS 접근 corridor로 취급한다.

### 7.2 조건

아래를 모두 만족할 때 `cross_approach_corridor = 1`:

1. map valid.
2. sync state MAP.
3. `Track_GetEvent(expected_event_index)`가 CROSS.
4. replay turn이 닫혀 있거나, confirmed same-side close marker의 local repair가 완료됨.
5. geometry source가 정상 pair close, local close repair 또는 정상 STRAIGHT map 근거 중 하나.
6. live course phase STRAIGHT.
7. fast gate stable.
8. line recovery 아님.
9. segment overdue가 아님.

현재 segment type이 LEFT/RIGHT여도 local close repair가 실제 close marker를 확인했다면 허용할 수 있다. 반대로 replay turn이 open인 상태에서는 CROSS가 다음이라는 이유만으로 거절 조건을 무시하지 않는다.

### 7.3 다음 restriction 거리

현재 segment가 non-fast여도 cross approach override가 활성화되면:

```text
distance to restriction = current remaining-to-CROSS
                        + segment after CROSS
                        + following STRAIGHT/CROSS segments
                        ... until next real LEFT/RIGHT restriction
```

기존 `SecondDrive_DistanceToNextRestriction()`는 non-fast current segment를 즉시 restriction으로 반환하므로 그대로 사용할 수 없다. 함수에 “current segment를 fast corridor로 override”하는 명시적 인자를 추가하거나 별도 helper를 만든다.

CROSS 뒤 segment는 기존처럼 fast geometry다. CROSS가 실제 검출되어 planner index가 advance되면 reason을 `FAST_CROSS_EXIT`로 바꾼다.

### 7.4 안전

- next expected marker가 CROSS라는 이유만으로 curve 중간부터 가속하면 안 된다.
- 반드시 confirmed pair-close 또는 local close repair로 replay turn이 닫혀 있어야 한다.
- live phase STRAIGHT + centered stable gate도 함께 만족해야 한다.
- 중앙 정렬만으로 열린 turn을 닫지 않는다.
- sync가 SEEK CROSS로 바뀌면 corridor override를 즉시 해제하고 기존 safe fallback으로 전환한다.
- CROSS anchor로 resync되면 replay turn을 reset하고 stable gate를 다시 만족한 뒤 fast를 허용한다.

---

## 8. 방향 Marker Pair 추적 및 Local Close Repair

### 8.1 규정 기반 pair 상태

Second Drive replay는 finalized map segment만 보지 말고, 이번 주행에서 실제로 다시 검출된 방향 marker sequence로 독립적인 pair 상태를 유지한다.

```c
static bool planner_replay_turn_open;
static int8_t planner_replay_turn_direction; /* -1 left, +1 right */
static uint32_t planner_replay_turn_open_step;
static bool planner_local_close_repair_active;
static uint32_t planner_local_close_repair_step;
```

규칙은 First Drive finalize와 동일하다.

- turn closed + L/R 검출: 해당 방향 turn open.
- turn open + 같은 방향 검출: turn close, 그 마커 이후 straight.
- turn open + 반대 방향 검출: 새 방향 curve로 전환.
- confirmed CROSS: turn state reset.
- END: 남아 있는 open turn을 진단값으로 보존.

이 상태는 map event index와 별도다. map은 1차 주행 기록이고 replay pair state는 2차 주행에서 실제로 본 물리 마커 증거다.

### 8.2 정상 expected event 처리

먼저 기존 `SecondDrive_EventMatchesExpected()`를 수행한다.

- event가 map expected event와 정상 일치하면 기존 index advance/anchor 처리 후 replay pair state도 갱신한다.
- 정상 같은 방향 close pair가 확인되면 geometry source를 `PAIR_CLOSE`로 기록한다.
- expected CROSS가 확인되면 기존 anchor를 갱신하고 replay pair state를 닫힌 상태로 reset한다.
- 정상 match 경로를 local repair보다 먼저 평가해 정상 event를 supplemental marker로 잘못 소비하지 않는다.

### 8.3 local close repair 허용 조건

map에는 없던 방향 marker가 replay에서 나왔다고 모두 무시해서는 안 된다. 아래를 모두 만족할 때만 “1차 주행에서 빠진 curve-close marker”로 인정한다.

1. map valid, sync state MAP.
2. replay Track collector가 확정한 LEFT/RIGHT event이며 confidence가 기존 planner 기준 이상.
3. event가 현재 map expected event와 정상 match하지 않음.
4. replay turn이 open 상태.
5. event 방향이 열린 turn 방향과 같음.
6. 현재 map segment가 열린 방향의 LEFT/RIGHT이거나 pair-unclosed 진단 상태.
7. map expected event가 CROSS 또는 final END.
8. 직전 방향 marker와의 step gap이 cooldown보다 크고 동일 event 중복이 아님.
9. CROSS-tail merged fragment, BOTH, UNKNOWN은 repair 후보가 아님.

expected CROSS와 final END로 범위를 제한하는 이유:

- 규정상 교차로 중심 전후 최소 20cm는 straight다.
- 규정상 곡선 종료 close marker 뒤에는 straight가 시작된다.
- END 앞에는 straight가 존재한다.
- 임의의 연속곡선 중간에서 unexpected marker를 보정하면 실제 방향 전환을 삭제할 위험이 있다.

### 8.4 local close repair 동작

조건을 통과하면:

- replay turn을 close한다.
- `local_close_repair_active = true`.
- geometry source를 `LOCAL_CLOSE_REPAIR`로 기록한다.
- repair count/step/direction을 기록한다.
- planner를 SEEK CROSS로 보내지 않는다.
- `planner_expected_event_index`를 증가시키지 않는다.
- `planner_segment_start_step`을 바꾸지 않는다.
- 원본 Track event/segment/anchor 배열을 수정하지 않는다.

expected index와 segment start를 유지해야 이후 CROSS/END가 1차 주행에서 기록된 원래 거리 기준으로 정상 match할 수 있다. local repair marker는 “맵 진행 이벤트”가 아니라 “누락된 geometry boundary 보충 증거”다.

repair event는 shared drive marker FSM에도 그대로 전달한다. 같은 방향 close marker라면 실제 course phase는 TURN에서 EXIT로 이동하고, 기존 centered exit 확인 뒤 STRAIGHT가 된다.

### 8.5 local repair 거절

다음은 기존 mismatch/SEEK CROSS 정책을 유지한다.

- replay turn이 닫혀 있는데 unexpected 방향 marker가 나옴.
- 열린 방향과 반대 방향 marker가 unexpected로 나옴.
- expected event가 일반 LEFT/RIGHT인데 별도 marker가 삽입됨.
- confidence 부족.
- duplicate/cooldown 범위.
- map invalid 또는 이미 SEEK CROSS.
- segment direction과 replay open direction이 모순됨.

거절 이유는 `LOCAL_NO_OPEN`, `LOCAL_WRONG_SIDE`, `LOCAL_EXPECTED_NOT_BOUNDARY`, `LOCAL_LOW_CONF`, `LOCAL_DUPLICATE`처럼 세분해 trace에 남긴다.

### 8.6 중앙 정렬만으로 straight를 선언하지 말 것

시립대 곡선은 반경 25cm로 일정하고, 로봇이 곡선을 잘 추종하면 곡선 중에도 line position은 중앙에 오래 머물 수 있다. 따라서 다음 조건만으로 turn을 닫으면 안 된다.

```text
abs(position) <= 500 for N frames
P/D가 작음
line valid가 오래 유지됨
```

final straight 승격에는 반드시 다음 중 하나의 marker geometry 증거가 필요하다.

- map에 정상 저장된 같은 방향 close pair.
- 2차 주행에서 정상 재검출된 같은 방향 close pair.
- 위 조건을 통과한 local close repair.
- confirmed CROSS reset.

동일 close marker를 1차와 2차 주행 모두 놓쳤다면 performance fast corridor를 억지로 열지 않고 안전속도로 END까지 주행한다. 속도 손실보다 열린 곡선에서의 오가속 방지가 우선이다.

---

## 9. 마지막 END 전 fast corridor

### 9.1 final END 판정

다음 expected event가 final END인지 helper로 명시적으로 확인한다.

```text
expected event type == MARKER_EVENT_BOTH
expected_event_index == Track_GetEventCount() - 1
last finalized segment type == TRACK_SEGMENT_END
map valid && sync MAP
```

start BOTH는 reset 시 이미 expected index에서 skip된다. 1차 맵에서 마지막 close marker가 빠졌더라도 local repair는 expected index를 소비하지 않으므로 final END는 계속 expected 상태로 남는다.

### 9.2 final corridor 조건

아래를 모두 만족할 때 마지막 straight corridor를 허용한다.

1. final END가 expected.
2. map valid, sync MAP.
3. replay turn이 닫혀 있음.
4. geometry source가 정상 pair close, local close repair 또는 정상 STRAIGHT map 근거 중 하나.
5. live course phase STRAIGHT.
6. fast gate stable.
7. line valid, recovery 아님.
8. segment overdue가 아님.

현재 map segment가 LEFT/RIGHT여도 `LOCAL_CLOSE_REPAIR`가 실제 동일 방향 close marker를 확인했다면 planner 전용으로 straight override할 수 있다. 반대로 replay turn이 open이면 map상 END가 다음이라는 이유만으로 fast를 허용하지 않는다.

이 경우 END는 “미리 1800 SPS로 내려갈 restriction”이 아니라 “hard stop boundary”다. final corridor target은 effective straight speed다.

### 9.3 confirmed close 이후 EXIT 처리

정상 또는 local-repair close marker가 확인되면 shared marker FSM은 TURN에서 EXIT로 이동할 수 있다. V30 planner는 실제 close 증거가 있을 때에만 EXIT 안정화를 기다린다.

```text
final END expected
replay_turn_open == false
geometry source == PAIR_CLOSE or LOCAL_CLOSE_REPAIR
course phase == EXIT_LEFT/EXIT_RIGHT
line valid
abs(position) <= 500 for at least 50 consecutive frames
no directional marker provisional active
not recovering
```

```c
#define SECOND_DRIVE_FINAL_EXIT_STABLE_FRAMES 50U
```

조건을 만족하면 planner 관점에서 final straight로 승격한다. shared `drive_course_phase`를 강제로 바꾸지 않는다.

중요: replay turn이 여전히 open이면 50 frame 중앙 정렬이 있어도 승격하지 않는다. 곡선에서도 정상 추종 시 position은 중앙일 수 있기 때문이다.

### 9.4 close marker가 2차에서도 누락된 경우

1차 맵에서 빠진 close marker를 2차에서도 검출하지 못하면:

- local repair 없음.
- replay turn open 유지.
- final fast corridor 금지.
- END approach fallback 또는 First Drive 기반 안전속도 유지.
- 결과 화면에 `PAIR OPEN L/R`, `LOCAL REPAIR 0` 표시.

이 경우 속도를 억지로 높이지 않는다. 다음 튜닝은 marker detector reject log를 보고 별도 수행한다.

### 9.5 END 거리 진단

END를 speed restriction에서 제외하더라도 예상 END까지 remaining step은 계속 계산·telemetry에 보존한다. 종료 후 실제 trigger step과 recorded expected step 차이, pair open/close 상태, local repair step을 함께 표시한다.

---

## 10. Curve/approach/exit 속도 결정

### 10.1 phase profile

V30에서는 1차 target에 단순 percent를 곱하는 방식 대신 명시적인 profile을 사용한다.

| live phase | V30 target |
|---|---:|
| STRAIGHT + fast corridor | effective straight |
| CROSS + mapped corridor | effective straight |
| APPROACH_LEFT/RIGHT | effective approach |
| TURN_LEFT/RIGHT | effective curve |
| EXIT_LEFT/RIGHT | effective exit |
| map invalid/SEEK | 기존 First Drive target |
| recovery | 기존 1400/1800 cap |

### 10.2 일정 곡률 코스

시립대 코스는 원형 구간의 곡률이 일정하다고 알려져 있다. curve angle이 45도인지 270도인지에 따라 centripetal speed를 다르게 할 필요는 없다. 공통 curve cruise 속도를 사용한다.

기존 `curve_units`는 다음 용도로만 사용한다.

- 예상 curve 길이/segment 진단.
- curve_units 0인 LEFT/RIGHT segment를 geometry uncertain으로 표시.
- 회전 구간이 예상보다 일찍/늦게 끝나는지 로그 확인.

curve_units가 크다는 이유만으로 속도를 더 낮추지 않는다.

### 10.3 steering 안전성

V30에서 First Drive PD, feedforward, steer limit 0.45/0.60, steer slew를 바로 변경하지 않는다. curve 3000 SPS 실차 시험에서 다음을 확인한다.

- outer boost 진입 빈도.
- line loss episode.
- edge dwell.
- slow wheel이 MOTOR_DRIVE_MIN_SPS에 과도하게 붙는지.
- fast wheel이 6500 clamp에 자주 붙는지.

코너 추종이 부족하면 속도를 먼저 100~200 SPS 낮춰 재검증한다. 같은 버전에서 근거 없이 steer gain과 속도를 동시에 크게 변경하지 않는다.

---

## 11. TURN 제동과 안전 제한

### 11.1 기존 braking formula 유지

기존 공식은 유지한다.

```text
braking_steps = (high_sps^2 - low_sps^2) / (2 × decel_sps_per_second)
```

- current decel: 10000 SPS/s.
- brake margin: 300 step.
- acceleration re-enable margin: 150 step.

V30 첫 구현에서 margin까지 동시에 공격적으로 줄이지 않는다. 속도 profile과 corridor 변경 효과를 먼저 검증한다.

### 11.2 restriction target

- 다음 restriction이 LEFT/RIGHT면 effective approach를 low target으로 사용한다.
- 실제 TURN phase 진입 후 effective curve를 사용한다.
- EXIT에서 effective exit로 올리고 centered stable 후 straight로 올린다.
- hard-end mode가 유효한 final END는 braking restriction에서 제외한다.
- hard-end 조건이 유효하지 않거나 sync가 불안정하면 기존 END approach fallback을 사용할 수 있다.

### 11.3 fallback

- MAP INVALID: 기존 First Drive target, reason MAP_INVALID.
- SEEK CROSS: 기존 First Drive target, reason SEEK_CROSS.
- CROSS resync 성공: stable gate를 다시 확인하고 performance profile 복귀.
- 한 번 mismatch됐다는 이유만으로 정상 resync 후에도 영구적으로 저속에 남지 않는다.

---

## 12. END hard brake

### 12.1 의미

V30의 “즉시 brake”는 END 확정 후 `FirstDrive_RampCommonSpeed()`로 0까지 감속하는 것이 아니다.

```text
confirmed END
-> control command 중단
-> motor step timers 즉시 정지
-> 현재 electrical phase를 짧게 유지해 holding torque 생성
-> hold 종료 후 Vref/output off
```

### 12.2 V28 END 방어 유지

V30 첫 구현은 raw S0/S7 한 frame이나 provisional BOTH에서 stop하지 않는다. 반드시 현재 V28의 `FirstDrive_IsEndMarkerEvent()`를 통과한 confirmed event에서 hard brake를 시작한다.

조건:

- BOTH event.
- start ignore step 이후.
- bilateral overlap 최소 충족.
- wide CROSS evidence 없음.
- CROSS tail 방어에 걸리지 않음.

이 원칙은 V28에서 해결한 CROSS 오정지를 되살리지 않기 위해 필수다.

### 12.3 motor API

`motor.h/c`에 Second Drive END 전용으로 사용할 수 있는 non-blocking brake API를 추가한다.

권장 API:

```c
bool Motor_DriveBrakeHoldStart(uint16_t initial_vref_dac);
void Motor_DriveBrakeHoldSetVref(uint16_t vref_dac);
void Motor_DriveBrakeHoldFinish(void);
bool Motor_DriveBrakeHoldIsActive(void);
```

`Motor_DriveBrakeHoldStart()`:

- L/R step timer interrupt를 즉시 중단.
- 현재 `motor_l_phase`, `motor_r_phase` pattern을 그대로 유지.
- `Motor_AllOutputsOff()`를 호출하지 않음.
- DAC/Vref를 지정값으로 유지.
- `motor_drive_running`은 false로 전환하여 추가 speed command가 실패하도록 함.

일반 `Motor_DriveStop()` semantics는 변경하지 않는다. fault/emergency에는 기존 완전 de-energize가 필요하다.

### 12.4 non-blocking hold 단계

초기 제안:

```c
#define SECOND_DRIVE_BRAKE_HARD_HOLD_MS      30U
#define SECOND_DRIVE_BRAKE_REDUCED_HOLD_MS  120U
#define MOTOR_VREF_DAC_BRAKE_HARD   MOTOR_VREF_DAC_RUN
#define MOTOR_VREF_DAC_BRAKE_HOLD            1024U
```

상태:

```text
END confirmed
-> HARD 30ms @ run current
-> HOLD 120ms @ reduced current
-> OFF
```

- `HAL_Delay()` 금지.
- tick 기반 non-blocking state machine.
- 기존 `FIRST_DRIVE_RUNOUT` 상태를 Second Drive brake hold 상태로 재사용할 수 있다.
- HARD/HOLD 완료 후 `FIRST_DRIVE_STOPPED`로 전환한다.
- hold 중 center emergency input이 들어오면 즉시 `Motor_DriveStop()`으로 완전 off한다.
- fault가 발생하면 hold를 건너뛰고 완전 off한다.

초기/감소 Vref와 hold 시간은 실차 결과에 따라 조정 가능하도록 named constants로 둔다.

### 12.5 Second Drive 전용 분기

기존 `FirstDrive_StopAtEndMarker()`에서 run mode를 구분한다.

- First Drive: V29 정상 종료와 snapshot/finalize 순서를 그대로 유지.
- Second Drive: final stats snapshot -> control disable -> hard brake hold state.

Second Drive hard brake 때문에 First Drive map finalize/stop record가 달라지면 안 된다.

### 12.6 END pre-deceleration 제거 조건

END를 restriction에서 제외하는 것은 아래 모두를 만족할 때만 허용한다.

- map structurally valid.
- sync MAP.
- expected event가 최종 BOTH/END.
- replay turn이 confirmed pair-close 또는 local close repair로 닫혀 있음.
- final corridor가 marker geometry evidence와 live STRAIGHT/EXIT 안정 조건을 모두 통과함.

조건이 하나라도 없으면 안전 fallback으로 기존 END approach speed를 허용한다.

---

## 13. 주행 중 LCD Freeze

### 13.1 요구 동작

READY/ARMED:

- 설정과 map 상태 표시.
- L/R/C 입력 시 redraw.

COUNTDOWN:

- countdown 표시 갱신 허용.

LAUNCH/FOLLOW/TURN/CROSS active 진입:

- 화면을 한 번만 정리하고 정적 문구 표시.

```text
SECOND DRIVE RUNNING
DISPLAY UPDATE LOCKED
CENTER = EMERGENCY STOP
```

- 이후 periodic LCD redraw 금지.
- telemetry getter와 `snprintf`도 LCD 목적으로 100ms마다 호출하지 않는다.

RUNOUT/BRAKE:

- END 직후 `BRAKING` 화면을 굳이 매 단계 redraw하지 않는다.
- 필요하면 상태 전이 때 1회만 표시.

STOPPED/FAULT:

- 상태가 active에서 stopped/fault로 바뀐 순간 1회 결과 화면 표시.

### 13.2 구현 방법

`Menu_UpdateSecondDrive()`는 이전 상태를 기억한다.

```c
static FirstDriveState_t second_drive_last_rendered_state;
static uint8_t second_drive_run_screen_locked;
```

- active 진입 transition에서 static running screen 1회.
- active 유지 중 return, render 없음.
- stop/fault transition에서 결과 render 1회.
- 버튼 입력/긴급정지 검사는 기존 main menu process에서 계속 수행.
- `SecondDrive_Process()`는 화면 freeze와 무관하게 매 loop 호출.

주행 중 화면이 멈췄다고 control/watchdog가 멈추면 안 된다.

---

## 14. Second Drive 종료 결과 화면

주행 중 화면을 멈추는 대신 종료 후 limiter 원인을 확인할 수 있어야 한다. 최소 3페이지를 권장한다.

### 14.1 Result 1/3

```text
SECOND DRIVE END
TIME 48.2s STEP 171290
CTR AVG/MAX 3560/5600
SYNC M0 R0 FINAL MAP
L/R PAGE C:BACK
```

### 14.2 Result 2/3

```text
SPEED LIMIT SHARE
FAST 62% CURVE 24%
BRAKE 8% SAFE 3%
POS 2% REC 1%
```

작은 비율은 합쳐도 되지만 raw sample 수는 debug page에 남긴다.

### 14.3 Result 3/3

```text
END BRAKE V5480 DONE
PAIR CLOSED SRC LOCAL
LCR1 S162520 DIR L
EXP S171263 ERR+27
LAST FAST_END
```

정상 map pair였다면 `SRC PAIR`, local close repair였다면 `SRC LOCAL`을 표시한다. open turn 상태로 끝났다면 빨간색으로 `PAIR OPEN L/R`과 marker reject count를 표시한다. fault/manual stop이면 END brake 대신 stop/fault와 마지막 limiter transition을 표시한다.

L/R navigation은 stopped/fault 상태에서만 동작한다. C hold 또는 기존 back semantics를 유지한다.

---

## 15. 권장 코드 구조

### 15.1 `Main/Inc/second_drive.h`

- V30 straight/curve/overall 범위.
- `curve_sps` config 필드.
- limiter reason enum.
- speed decision/status/run stats/trace 타입.
- run stats getter.
- 필요한 planner helper/API.

### 15.2 `Main/Src/second_drive.c`

- true overall scaling.
- phase profile target.
- fast gate hysteresis.
- replay directional-marker pair/open-turn state.
- normal pair close와 supplemental local close repair.
- local repair 시 expected index/segment start를 유지하는 처리.
- CROSS approach corridor.
- pair evidence 기반 final END corridor.
- corridor-aware lookahead restriction.
- limiter/geometry source 결정.
- O(1) stats/transition trace.
- V27 anchor resync 코드 자체는 변경하지 않음.

### 15.3 `Main/Src/drive.c`, `Main/Inc/drive.h`

- planner decision을 받아 recovery/최종 clamp까지 reason 확정.
- confirmed replay marker를 planner pair state와 shared course FSM 양쪽에 전달.
- `FirstDrive_GetMarkerEdges()`의 reject reason을 O(1) 진단값으로 기록하되 threshold는 변경하지 않음.
- Second Drive run stats 시작/종료 hook.
- END에서 run mode별 stop 분기.
- Second Drive RUNOUT/brake tick 처리.
- First Drive stop/fault/run record 경로 보존.

### 15.4 `Main/Src/motor.c`, `Main/Inc/motor.h`

- non-blocking brake hold start/set/finish API.
- 일반 stop semantics 보존.
- brake active 상태에서 speed command 거절.

### 15.5 `Main/Src/menu.c`

- STRAIGHT/CURVE/ALL 3단 설정.
- effective speed 표시.
- active run static screen.
- periodic active redraw 제거.
- stopped result 3페이지.
- result/debug에 replay pair open/direction, geometry source, local repair count/step, marker reject count 표시.

### 15.6 `Main/Inc/app_version.h`

- `APP_VERSION_NUMBER 30U`.

### 15.7 `Main/Src/track.c`, `Main/Inc/track.h`

segment 생성 semantics는 변경하지 않는다. 다음 진단 추가만 허용한다.

- finalize 종료 시 `turn_open` 여부와 방향.
- END 직전 unmatched turn 여부.
- 마지막 confirmed directional event step/direction.
- O(1) getter용 `TrackMapPairDiagnostics_t`.

원본 event/segment/anchor 배열을 local repair로 수정하지 않는다. sensor/marker threshold도 V30에서 임의로 완화하지 않는다. marker 누락 원인은 먼저 reject count로 확인한다.

---

## 16. 속도 결정 권장 순서

한 control tick에서 이벤트와 속도 결정을 아래 순서로 고정한다. 특히 marker mismatch를 곧바로 SEEK로 보내기 전에 정상 expected match와 제한된 local close repair를 순서대로 평가해야 한다.

1. map/sync 상태 확인.
2. confirmed replay marker가 있으면 기존 expected event 정상 match를 먼저 평가한다.
3. 정상 match 결과로 expected index/anchor와 replay pair state를 갱신한다.
4. 정상 match가 아닌 방향 marker에 한해서 local close repair 조건을 평가한다.
5. repair 성공 시 replay turn만 close하고 expected index와 segment start는 유지한다. 거절 시 기존 SEEK CROSS 정책을 수행한다.
6. live phase와 line/recovery 상태를 확인한다.
7. marker geometry source와 replay pair open/closed 상태를 확인한다.
8. fast gate hysteresis를 갱신한다. 이 gate는 line 안정성 검사이며 열린 turn을 닫는 증거가 아니다.
9. pair가 닫힌 경우에만 expected CROSS/final END corridor를 판단한다.
10. nominal performance profile을 선택한다.
11. 다음 실제 TURN restriction braking을 적용한다.
12. hard-end 조건이 아니면 END fallback restriction을 적용한다.
13. recovery cap을 적용한다.
14. center performance max clamp를 적용한다.
15. steer 적용 후 L/R motor max clamp를 적용한다.
16. 최종 limiter reason 1개를 확정하고 stats를 1회 누적한다.
17. motor command를 보낸다.

planner가 FAST를 반환했더라도 drive.c의 recovery가 1400으로 낮췄다면 최종 reason은 RECOVERY다. UI/통계가 planner 중간값만 보여주면 안 된다. local close repair는 geometry 상태만 보완하며 map 진행 거리와 event 순서를 변경하지 않는다.

---

## 17. 테스트 계획

### 17.1 host planner test

기존 V27/V28 harness를 유지하고 V30 planner test를 추가한다.

필수 시나리오:

1. 규정 기반 pair FSM
   - 입력 `R, R, R, L, R, L, L, END`에서 segment가 순서대로 `RIGHT, STRAIGHT, RIGHT, LEFT, RIGHT, LEFT, STRAIGHT`가 되는지 검증한다.
   - 첫 같은 방향 marker가 curve open, 두 번째 같은 방향 marker가 curve close임을 확인한다.
   - opposite marker는 현재 curve close와 다음 curve open을 동시에 의미하는 기존 전환 semantics를 확인한다.
   - confirmed CROSS에서 replay pair가 reset되는지 확인한다.
2. 1차 맵의 마지막 close 누락 재현
   - 입력 `R, R, R, L, R, L, END`에서는 마지막 LEFT가 닫히지 않고 `unmatched LEFT/open turn` 진단이 남는지 확인한다.
   - 실제 로그 tail `LEFT, RIGHT, LEFT, END`가 마지막 `L` 누락과 일치하는 fixture를 둔다.
3. 정상 expected 처리 우선순위
   - expected와 같은 marker는 기존 index/anchor advance를 수행하고 local repair로 소비하지 않는다.
   - 정상 same-side close는 `PAIR_CLOSE`, 정상 CROSS는 `CROSS_RESET` source를 남긴다.
4. local close repair
   - map expected가 final END이고 replay LEFT가 open인 상태에서 unexpected confirmed LEFT를 받으면 repair 성공.
   - map expected가 CROSS이고 replay RIGHT가 open인 상태에서 unexpected confirmed RIGHT를 받으면 repair 성공.
   - repair 후 replay turn은 closed, source는 `LOCAL_CLOSE_REPAIR`, sync는 MAP 유지.
   - expected event index와 planner segment start step은 repair 전후 동일.
   - 원본 Track event/segment/anchor 배열은 byte-for-byte 동일.
   - repair marker가 shared phase FSM의 TURN→EXIT 전환에는 전달되는지 확인한다.
5. local repair 거절
   - replay turn closed, wrong-side marker, 일반 LEFT/RIGHT expected, low confidence, duplicate/cooldown, tail/BOTH/UNKNOWN은 repair 금지.
   - 각 경우 기존 mismatch/SEEK 정책과 세분된 reject reason을 확인한다.
6. Fast gate와 geometry gate 분리
   - 29 centered frame에서는 fast gate OFF, 30 frame에서 ON.
   - 500~899 hysteresis 유지, 900 이상/line invalid에서 OFF.
   - replay turn open이면 50 frame 이상 중앙 정렬이어도 CROSS/END fast corridor 금지.
   - 1차와 2차 모두 close marker를 놓친 경우 safe fallback을 유지한다.
7. CROSS approach
   - pair closed + expected CROSS + live stable STRAIGHT이면 current map segment LEFT/RIGHT도 fast corridor 허용.
   - pair open, phase TURN 또는 SEEK CROSS이면 fast 금지.
   - 정상 CROSS 통과 후 기존 anchor/index advance와 pair reset이 정상.
8. Final END corridor
   - 정상 pair close 또는 local close repair + final expected BOTH + stable STRAIGHT에서 fast override.
   - confirmed close 후 EXIT phase는 50 centered frame 뒤 planner-only final straight 승격.
   - pair open, final이 아닌 BOTH 또는 map invalid면 override 금지.
9. Overall semantics
   - straight/curve/approach/exit에 동일 percent 적용.
   - 6200 center clamp.
   - recovery/fallback에는 performance scale 미적용.
10. Lookahead
   - CROSS approach current remaining과 CROSS 뒤 fast segment를 합산해 다음 TURN까지 거리 계산.
   - next TURN braking target은 effective approach.
11. Hard END policy
   - valid final corridor에서는 END를 1800 restriction으로 사용하지 않음.
   - pair open/sync/final 조건 실패 시 END fallback restriction 유지.
12. Limiter/geometry stats
   - 1 tick에 limiter reason sample 하나.
   - reason transition 때만 trace 추가.
   - pair source, local repair count/step/direction, reject reason이 기대값과 일치.
13. V27 resync
   - mismatch -> SEEK -> forward CROSS anchor -> MAP 복귀.
   - CROSS resync 시 replay pair reset.
   - 복귀 후 pair/geometry와 stable gate를 다시 확인한 뒤 fast 재진입.
### 17.2 motor brake host/mock test

- brake start가 timer를 즉시 stop.
- current phase output을 바로 off하지 않음.
- HARD 30ms -> HOLD 120ms -> OFF 상태 순서.
- blocking delay 없음.
- emergency/fault stop은 즉시 output off.
- brake active 중 `Motor_DriveSetSpeeds()` 실패.
- repeated brake start/finish가 idempotent.

### 17.3 UI test

- READY에서 STRAIGHT/CURVE/ALL 선택/조절.
- effective clamp 표시.
- COUNTDOWN update.
- active 진입 시 running 화면 1회.
- active 상태에서 1초 이상 menu loop를 돌려도 LCD render count가 증가하지 않음.
- center emergency stop은 화면 freeze 상태에서도 즉시 동작.
- STOP/FAULT transition에서 결과 화면 1회.
- 결과 3페이지 L/R navigation.

가능하면 test build에서 LCD render counter를 임시/조건부로 두어 active 중 0회 redraw를 검증하고 production에서는 제거하거나 debug macro로 감싼다.

### 17.4 회귀

- First Drive 동일 sensor trace에서 V29와 marker/phase/speed/fault 결과 동일.
- 방향 marker pair로 segment를 만드는 기존 `track.c` semantics를 변경하지 않음.
- 정상 `R,R,R,L,R,L,L,END` trace에서 마지막 구간이 STRAIGHT로 finalize됨.
- V29 3×3 result UI 동일.
- V28 split CROSS/tail merge/END guard harness PASS.
- V27 anchor resync harness PASS.
- 정상 expected marker의 index/anchor advance 동작 동일.
- local repair를 사용하지 않은 map에서는 V29 planner 진행 결과가 동일.
- Second Drive CROSS 통과 중 2400 cap 재발 없음.
- map invalid/SEEK 또는 replay pair open에서 무조건 fast가 되지 않음.
### 17.5 단계별 실차 시험

한 번에 120% 최대값으로 시작하지 않는다. 전체맵 고속 시험 전에 마지막 curve-close marker 검출 여부와 local repair를 저속으로 먼저 확인한다.

1. 바퀴 공중/스탠드
   - straight 5600, curve 3000, overall100.
   - hard brake timer/Vref/coil 상태 확인.
   - synthetic marker 입력에서 pair open/close, expected index 불변, local repair 표시 확인.
2. 마지막 marker sequence 저속 기록 시험
   - 실제 `R -> R -> R -> L -> R -> L -> L -> END`를 통과한다.
   - 두 번째 마지막 `L`이 confirmed event로 기록되는지 확인한다.
   - detector가 거절했다면 low confidence/cooldown/tail 등 reject reason과 step을 확보한다.
   - 정상 검출 시 Result/trace가 `PAIR CLOSED SRC PAIR`인지 확인한다.
3. local close repair 재생 시험
   - 1차 맵에서 마지막 close `L`이 빠진 fixture/map을 사용한다.
   - 2차에서 해당 `L` 검출 시 `SRC LOCAL`, repair count 1을 확인한다.
   - expected END index와 segment start가 repair 전후 바뀌지 않고 SEEK로 가지 않는지 확인한다.
   - 동일 marker를 2차에서도 놓치면 `PAIR OPEN`, safe fallback이며 fast가 켜지지 않는지 확인한다.
4. 짧은 직선 END 시험
   - overall90 또는 straight4800부터.
   - END confirmed 후 hard brake와 정지 거리 확인.
   - CROSS를 END로 오정지하지 않는지 별도 시험.
5. 단일 일정 곡률 코너
   - curve 2800 -> 3000 순서.
   - 곡선에서 line position이 중앙이어도 open pair 상태에서는 straight 승격되지 않는지 확인한다.
   - line loss/outer boost/wheel clamp 확인.
6. CROSS가 포함된 긴 직선
   - 정상 pair close 또는 CROSS reset 증거 뒤 CROSS 전 fast reason 진입 확인.
   - CROSS 후 anchor/index와 replay pair reset 확인.
7. 마지막 긴 직선
   - `PAIR CLOSED`와 geometry source를 먼저 확인한다.
   - `FAST_END_CORRIDOR` 진입 확인.
   - END 전 1800/2160 장기 제한이 사라졌는지 확인.
8. 전체 예선맵
   - overall100부터.
   - 정상 후 105, 110 순서.
   - 120은 motor/라인 안정성이 확인된 뒤 사용.

각 시험 후 Result 2/3 limiter share와 Result 3/3의 pair/source/local repair, marker reject count, END brake 값을 함께 기록한다.

---

## 18. 빌드와 정적 확인

프로젝트 루트에서 clean build:

```bash
cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --clean-first -j 8
```

확인:

- warning/error 없음.
- ISR/control stack 증가 확인.
- V29 대비 Flash/RAM 증가량 기록.
- `SecondDriveRunStats_t`/trace 배열 크기 기록.
- 64-bit division이 ISR에 들어가지 않았는지 확인.
- LCD API가 control ISR에서 호출되지 않는지 확인.
- `HAL_Delay()`가 brake 경로에 없는지 확인.

---

## 19. 구현 순서

1. V27~V29 plan/result를 읽고 current diff를 확인한다.
2. V29 source로 clean build와 기존 harness baseline을 확보한다.
3. 시립대 규정 기반 marker pair 표와 실제 final sequence fixture를 host test에 먼저 추가한다.
4. limiter reason/status/run stats, geometry source, pair diagnostics, marker reject counter를 O(1) 구조로 구현한다.
5. 기존 `track.c` pair FSM을 변경하지 않은 채 정상 sequence와 마지막 close 누락 sequence를 재현한다.
6. Second Drive replay pair open/direction/step 상태를 구현하고 정상 expected match 경로에 연결한다.
7. 정상 match가 아닌 confirmed same-side close에 한정한 local close repair를 구현한다.
8. repair 전후 expected index, segment start, 원본 map 배열 불변 테스트를 통과시킨다.
9. wrong-side/unopened/ordinary expected/low-confidence/duplicate repair 거절과 기존 SEEK 경로를 검증한다.
10. overall semantics와 curve config/UI를 구현한다.
11. fast gate hysteresis를 구현하되 marker geometry gate와 분리한다.
12. pair-close/CROSS-reset/local-repair 증거 기반 CROSS approach corridor와 corridor-aware lookahead를 구현한다.
13. pair evidence 기반 final END corridor와 close 이후 EXIT-stable planner override를 구현한다.
14. phase speed profile과 TURN braking을 연결한다.
15. motor non-blocking hard brake API를 구현한다.
16. Second Drive confirmed END에 RUNOUT/brake state를 연결한다.
17. active LCD freeze와 stopped result 화면을 구현한다.
18. host tests와 V27/V28/V29 회귀를 수행한다.
19. ARM clean build와 size를 확인한다.
20. 앱 버전을 30으로 확정한다.
21. 작업 결과 문서를 작성한다.

계측과 재현 test를 먼저 구현해야 이후 speed behavior가 기대와 다를 때 원인을 즉시 확인할 수 있다. local repair를 corridor보다 먼저 완성해 corridor 코드가 불완전한 pair 상태를 임의로 해석하지 않게 한다.

---

## 20. 완료 조건

- [ ] 앱 버전이 30이다.
- [ ] 기존 marker pair FSM을 반대로 해석하거나 현재/다음 marker만 보는 단순 분류로 교체하지 않았다.
- [ ] 규정 sequence `R,R,R,L,R,L,L,END`가 `RIGHT,STRAIGHT,RIGHT,LEFT,RIGHT,LEFT,STRAIGHT`로 재현된다.
- [ ] 마지막 close `L` 누락 fixture가 unmatched LEFT/open turn으로 재현된다.
- [ ] 정상 expected marker 처리가 local repair보다 먼저 수행된다.
- [ ] final END/CROSS expected에서 confirmed same-side close만 local repair된다.
- [ ] local repair 후 expected index와 planner segment start가 변하지 않고 원본 map/anchor 배열도 변하지 않는다.
- [ ] wrong-side, unopened, 일반 방향 marker expected, low-confidence, duplicate/tail event는 repair되지 않는다.
- [ ] replay pair가 open이면 50 frame 이상 중앙 정렬이어도 CROSS/END fast corridor를 허용하지 않는다.
- [ ] close marker를 1차와 2차에서 모두 놓치면 안전 fallback을 유지한다.
- [ ] 결과/trace에서 pair open/direction, geometry source, local repair와 marker reject reason을 확인할 수 있다.
- [ ] V30 100% curve가 3000 SPS로 현재 V29 120% TURN 2640보다 높다.
- [ ] straight/curve/approach/exit 모두 overall percent가 실제 적용된다.
- [ ] effective center target은 6200으로 clamp되고 UI에 표시된다.
- [ ] recovery 및 map fallback 안전속도는 performance scale과 분리된다.
- [ ] current map segment가 LEFT/RIGHT여도 pair close 증거 + expected CROSS + live stable straight에서 CROSS 전 가속한다.
- [ ] 마지막 END 전 physical straight가 정상 pair close 또는 local close repair 후 `FAST_END_CORRIDOR`로 가속한다.
- [ ] confirmed close 후 final EXIT phase 고착이 planner-only stable confirmation으로 해소된다.
- [ ] 다음 실제 TURN에는 기존 braking distance를 이용해 미리 접근 속도로 감속한다.
- [ ] valid final corridor에서 END 1800/2160 장거리 제한이 사라진다.
- [ ] V28-confirmed END에서 즉시 step timer가 멈춘다.
- [ ] 30ms hard + 120ms reduced hold 후 output off가 non-blocking으로 동작한다.
- [ ] fault/emergency는 active hold 없이 즉시 완전 off한다.
- [ ] 주행 중 LCD periodic redraw가 0회다.
- [ ] 화면 freeze 중 center emergency stop이 동작한다.
- [ ] 종료 후 limiter share, pair/repair 상태와 END brake 정보가 확인된다.
- [ ] limiter reason은 control tick당 정확히 하나 누적된다.
- [ ] V27 anchor resync가 유지되고 resync 후 replay pair가 안전하게 reset된다.
- [ ] V28 CROSS-tail/END guard가 유지된다.
- [ ] V29 First Drive 기록/UI와 `track.c` segment semantics가 유지된다.
- [ ] host tests와 clean build가 통과한다.
- [ ] V29 대비 Flash/RAM 증가량이 기록된다.

---

## 21. Luna가 작성할 작업 결과 문서

구현 완료 후 다음 파일을 작성한다.

`codex_worked_review/V30_SECOND_DRIVE_SPEED_CORRIDOR_HARD_BRAKE_AND_LCD_FREEZE_WORK_RESULT.md`

반드시 포함:

1. 변경 파일 목록과 파일별 변경 내용.
2. 시립대 규정에 따른 direction marker pair 해석 표와 이를 구현한 실제 symbol/function.
3. 실제 final sequence `R,R,R,L,R,L,L,END` fixture의 segment 출력.
4. 마지막 close `L` 누락 fixture와 unmatched LEFT 진단 출력.
5. replay pair state의 symbol, 갱신 지점, CROSS reset 동작.
6. local close repair의 모든 허용/거절 조건과 실제 symbol.
7. local repair 전후 expected index, segment start와 map 배열이 불변이라는 test evidence.
8. marker detector reject counter/reason과 결과 화면 실제 문구.
9. 중앙 정렬만으로 열린 turn을 straight로 승격하지 않는 test evidence.
10. 실제 적용한 speed profile 상수와 UI 범위.
11. overall scaling 전/후 계산 예시.
12. limiter reason/geometry source enum과 최종 우선순위.
13. pair evidence를 포함한 CROSS approach corridor 조건과 테스트 결과.
14. pair evidence를 포함한 final END corridor 조건과 마지막 LEFT map segment override 결과.
15. confirmed close 이후 EXIT-stable planner override 구현 방식.
16. TURN braking 거리 계산 보존 여부.
17. motor hard brake 상태 머신과 Vref/timing.
18. V28 END guard를 유지했다는 코드 근거.
19. active LCD render가 0회라는 검증 결과.
20. Second Drive stopped 결과 화면 실제 문구 또는 사진.
21. host test/harness PASS 원문.
22. ARM clean build 명령과 결과.
23. V29 대비 Flash/RAM 증감.
24. First Drive marker/segment behavior를 변경하지 않았다는 회귀 근거.
25. 실차에서 반드시 확인해야 할 위험과 권장 시험 순서.

“구현 완료”만 기록하지 말고 Codex가 source와 test evidence를 재검토할 수 있도록 실제 symbol, 상수, 테스트 입력/출력을 적는다. 특히 local repair가 expected event를 소비하지 않았다는 전후 값을 반드시 남긴다.

---

## 22. 최종 주의사항

- 최고속도만 올리면 해결되지 않는다. 현재 문제의 핵심은 marker close 누락에 취약한 pair 상태와 fast eligibility다.
- 기존 pair FSM 자체는 시립대 규정과 실제 marker sequence에 부합한다. 이를 반대로 뒤집거나 `현재 방향 == 다음 방향이면 직선` 같은 단순 규칙으로 교체하지 않는다.
- straight 속도를 높일수록 braking distance가 제곱으로 증가한다. TURN lookahead를 제거하지 않는다.
- CROSS 접근 override는 expected CROSS만 보고 켜지 말고 confirmed pair close/CROSS reset/local close repair와 live stable straight를 모두 요구한다.
- 마지막 straight override는 final expected END, 닫힌 replay pair, marker geometry source와 live straight confirmation을 모두 요구한다.
- 일정 반경 곡선을 정상 추종할 때도 line position은 중앙일 수 있다. centered frame, 작은 P/D, line-valid 시간만으로 열린 turn을 닫지 않는다.
- local close repair는 replay geometry만 보완한다. expected event index, planner segment start, 원본 map event/segment/anchor를 수정하지 않는다.
- repair 범위를 expected CROSS/final END의 confirmed same-side close로 제한한다. 반대 방향이나 일반 연속곡선 event를 임의로 소비하지 않는다.
- close marker를 1차와 2차에서 모두 놓쳤다면 해당 corridor는 안전속도로 주행한다. 불확실성을 속도 향상으로 덮지 않는다.
- marker threshold/cooldown을 같은 버전에서 근거 없이 완화하지 않는다. reject reason과 실제 간격을 먼저 계측한다.
- END pre-deceleration을 제거하더라도 V28 END 확정 조건을 약화하지 않는다.
- raw BOTH나 한 frame S0/S7로 hard brake를 시작하지 않는다.
- active brake는 fault/emergency stop과 다른 동작이다. 일반 `Motor_DriveStop()` 의미를 바꾸지 않는다.
- brake hold에 blocking delay를 사용하지 않는다.
- 주행 중 LCD를 멈추더라도 control, watchdog, sensor, center emergency 입력은 계속 처리한다.
- V30 첫 실차는 100% profile에서 시작하며, 120%는 마지막 검증 단계다.
