# V29 First Drive Run Record 및 종료 로그 UI 구현 계획

## 0. 문서 목적과 구현자에게 전달할 핵심

이 문서는 `line_tracer_2026` 프로젝트의 **V28을 기반으로 V29를 구현하는 Luna Max용 단독 인수인계 문서**다. Luna는 이전 대화 내용을 볼 수 없다고 가정한다.

V29의 목표는 두 가지다.

1. 1차 주행 중 이미 존재하는 제어 동작은 유지하면서, 주행 결과를 사후 판단할 수 있는 통계를 정확히 기록한다.
2. 1차 주행 종료 후 LCD 로그를 `사용자 요약 3페이지 → 마커 요약 3페이지 → 고밀도 디버그 3페이지` 구조로 재편한다.

V29는 주행 성능을 공격적으로 바꾸는 버전이 아니다. 특히 V28에서 해결한 교차로 분리·tail 융합 로직과 V27의 2차 주행 anchor 재동기화 로직을 훼손하면 안 된다.

> 구현 기준: 현재 작업 디렉터리의 V28 소스가 기준이다. 작업 트리가 깨끗하지 않을 수 있으므로 기존 변경을 reset, restore, checkout으로 되돌리지 말고 현재 상태 위에 필요한 변경만 추가한다.

---

## 1. 현재 프로젝트 상태

### 1.1 기준 버전

- 프로젝트 루트: `/Users/ehoi/STM32CubeIDE/line_tracer_2026`
- 현재 버전: V28
- 버전 정의: `Main/Inc/app_version.h`
- V28 구현 결과 문서: `codex_worked_review/V28_SPLIT_CROSS_TAIL_FUSION_WORK_RESULT.md`
- V27 구현 결과 문서: `codex_worked_review/V27_CROSS_ANCHOR_RESYNC_WORK_RESULT.md`
- V28 실차 결과: 기존의 한 교차로가 CROSS와 종료 BOTH로 중복 해석되어 멈추던 문제는 재발하지 않았다.

V28 빌드 결과는 다음과 같았다.

- Flash: 116,360 B / 512 KB, 약 22.19%
- RAM: 41,720 B / 272 KB, 약 14.98%
- V28 host harness: PASS

### 1.2 반드시 보존할 V27/V28 동작

V27:

- 1차 주행에서 확정된 CROSS를 기반으로 anchor를 생성한다.
- 2차 주행에서 회전 마커를 놓친 경우에도 신뢰도가 높은 교차로를 이용해 맵 위치를 재동기화한다.
- anchor는 물리 마커 종류가 아니라, finalize 후 만들어지는 맵상의 재동기화 기준점이다.

V28:

- 중앙 센서군에서 CROSS가 먼저 확정된 뒤 뒤쪽 외곽 센서가 같은 교차로의 tail을 검출해도 별도 BOTH/END 이벤트로 저장하지 않는다.
- tail은 앞선 CROSS에 융합되고 public marker event 수를 증가시키지 않는다.
- 실제 종료선은 V28의 END 방어 조건을 통과해야 한다.
- 관련 현행 상수의 의미를 바꾸지 않는다.
  - tail gap: 160 step
  - marker confirm: 3 frame
  - clear: 5 frame
  - cooldown: 50 step
  - cross pass: 510 step

### 1.3 주요 파일과 역할

| 파일 | 현재 역할 | V29 변경 |
|---|---|---|
| `Main/Inc/app_version.h` | 펌웨어 버전 | 29로 증가 |
| `Main/Inc/drive.h` | 1차 주행 타입, telemetry, 외부 API | 종료 사유, run record 타입/API 추가 |
| `Main/Src/drive.c` | 1차 주행 제어, 마커 검출, fault/stop | O(1) 통계 누적, 정지 snapshot/final record 생성 |
| `Main/Inc/track.h` | track event/segment/anchor 타입 및 API | tail 최대 gap 등 최종 통계 API/필드 추가 가능 |
| `Main/Src/track.c` | event 저장, segment/anchor finalize, tail 융합 | V28 로직 보존, 진단 통계만 최소 추가 |
| `Main/Src/menu.c` | LCD 화면과 버튼 처리 | 3그룹 × 3페이지 종료 UI 구현 |
| `Main/Src/second_drive.c` | 2차 주행 planner와 anchor resync | 원칙적으로 수정 금지 |

---

## 2. 현재 1차 주행 로직에서 알아야 할 사실

### 2.1 1차 주행 속도는 이미 가변적이다

현재 실제 제어 경로의 대표 값은 다음과 같다.

| 상황 | 실제 목표/제한 값 |
|---|---:|
| 출발 실제 속도 | 400 SPS |
| 직선 기준 속도 | 3820 SPS, 약 1.5 m/s |
| 위치 오차에 따른 최저 목표 | 1800 SPS |
| 확정 TURN 제한 | 2200 SPS |
| 마커 통과 후 600 step 제한 | 2400 SPS |
| CROSS 통과 제한 | 2400 SPS |
| 복구 직선 | 1400 SPS |
| 복구 회전 | 1800 SPS |
| 가속 slew | +8 SPS/ms |
| 감속 slew | -10 SPS/ms |
| 1차 주행 wheel 최대 | 5200 SPS |

직선에서도 line position 오차에 따라 3820에서 1800 SPS까지 선형으로 낮아질 수 있고, steering에 의해 좌·우 wheel SPS는 서로 달라진다.

주의:

- `FirstDriveConfig_t.launch_sps = 800`, `ramp_sps_per_10ms = 20` 같은 설정값이 남아 있더라도 현재 실제 주행 경로는 위의 400, +8/-10 상수를 사용한다.
- V29 화면에는 설정 구조체의 미사용 값을 실제값처럼 표시하지 말고, **제어 루프에서 실제 적용·관찰된 값을 기록해 표시**한다.
- V29에서 이 속도 정책 자체를 정리하거나 변경하지 않는다. 미사용 설정 필드 정리는 별도 버전 범위다.

### 2.2 현재 telemetry와 로그의 한계

현재 `FirstDriveTelemetry_t`에는 상태/fault/phase, 센서 mask, line 복구 상태, 마지막 마커, tail/END 방어 진단, 짧은 phase/marker ring log, 속도, line loss, edge dwell, P/D/steer, step/time/IRQ 수 등이 있다.

그러나 다음 문제가 있다.

- 정상 종료 처리에서 motor target/current/L/R 값을 0으로 만든 뒤 화면이 telemetry를 읽기 때문에 마지막 실제 속도가 사라진다.
- `lost_count`는 line loss **episode 수**가 아니라 invalid frame 누적 수다.
- `edge_dwell_ms`는 현재 dwell이며 edge를 벗어나면 0으로 초기화되어 최대값이 남지 않는다.
- V28 tail 진단의 gap은 마지막 gap이지 최대 gap이 아니다.
- stopped 화면은 raw 수치 위주 4페이지이며 사용자에게 정상 여부가 직관적이지 않다.
- `FirstDrive_GetTelemetry()`가 IRQ를 막고 큰 구조체 전체를 복사하므로 run record를 계속 비대하게 합치면 제어 지연 위험이 커진다.

### 2.3 CROSS, anchor, marker total의 의미

- `CROSS`: 센서가 검출한 실제 교차로 마커 이벤트.
- `anchor`: `Track_FinalizeSegments()` 뒤 CROSS로부터 파생된 맵 bookmark. 원본 event index, CROSS 이후 segment index, center step, 이전 CROSS와의 거리 등을 가진 2차 주행 재동기화 기준점.
- 따라서 정상적인 맵에서는 CROSS 수와 anchor 수가 같아야 한다. 다르면 overflow, finalize 제외, 또는 맵 생성 이상을 의심한다.
- 출발 직후 300 step 안의 BOTH는 start line으로 저장될 수 있지만 segment finalize에서 제외된다. V29 요약에서는 이를 END가 아니라 START로 분류한다.
- marker TOTAL의 정의는 아래로 고정한다.

```text
TOTAL = START + LEFT + RIGHT + CROSS + END + UNKNOWN
```

- CROSS tail 융합은 public event가 아니므로 TOTAL에 포함하지 않는다.

---

## 3. V29 범위와 비범위

### 3.1 반드시 구현할 것

1. 정상 종료, 수동 정지, fault 정지를 명시적으로 구분하는 stop reason.
2. 정지 직전 제어 상태를 0으로 지우기 전에 보존하는 immutable stop snapshot.
3. 1차 주행 전체의 marker, line loss/recovery, edge dwell, tail, END guard, overflow, speed 통계.
4. 종료 시 완성되어 이후 LCD가 안전하게 읽는 `FirstDriveRunRecord_t` 계열 구조.
5. 종료 UI 9페이지와 새 버튼 navigation.
6. fault 정지 시 정확한 원인과 정지 순간 문맥을 첫 화면에 즉시 표시.
7. V28/V27 회귀 테스트 및 clean build.
8. `APP_VERSION_NUMBER`를 29로 변경.

### 3.2 변경하지 말 것

- 1차 주행 PID/PD, steering, phase 판정, 속도 목표/제한, slew rate.
- V28 CROSS/tail/END 판정 임계값과 상태 전이.
- track event의 의미와 기존 segment 생성 규칙.
- V27 2차 주행 planner, anchor 재동기화, safe mode 정책.
- 센서 샘플링 주기와 motor ISR 구조.
- EEPROM/flash 영구 저장 형식. V29 run record는 RAM 기반 종료 결과다.

진단 추가 때문에 실제 주행 분기 순서나 타이밍이 달라지지 않도록 한다.

---

## 4. 권장 데이터 구조

정확한 명칭은 기존 코딩 스타일에 맞춰 조정할 수 있으나, 정보와 수명은 아래 구조를 따른다.

### 4.1 명시적 stop reason

`drive.h`에 1차 주행 종료 이유 enum을 추가한다.

```c
typedef enum {
    FIRST_DRIVE_STOP_REASON_NONE = 0,
    FIRST_DRIVE_STOP_REASON_END_MARKER,
    FIRST_DRIVE_STOP_REASON_MANUAL,
    FIRST_DRIVE_STOP_REASON_FAULT
} FirstDriveStopReason_t;
```

FAULT 세부 원인은 기존 `FirstDriveFault_t`를 함께 저장한다. 이미 fault enum이 충분히 세분되어 있으므로 같은 내용을 stop reason enum에 중복 나열하지 않는다.

표시 규칙:

- `END_MARKER + FAULT_NONE` → 성공
- `MANUAL + FAULT_NONE` → 사용자 중단/미완료
- `FAULT + 구체적 FirstDriveFault_t` → 실패 및 구체 원인

### 4.2 marker summary

최소 다음 누적값을 둔다.

```c
typedef struct {
    uint16_t total_count;
    uint16_t start_count;
    uint16_t left_count;
    uint16_t right_count;
    uint16_t cross_count;
    uint16_t end_count;
    uint16_t unknown_count;
    uint16_t segment_count;
    uint16_t anchor_count;

    uint16_t tail_event_count;
    uint16_t tail_cross_count;
    uint16_t tail_max_gap_steps;
    uint16_t end_guard_reject_count;

    uint8_t track_overflow;
    uint8_t anchor_overflow;
    uint8_t map_valid;
} FirstDriveMarkerSummary_t;
```

타입 폭은 프로젝트의 최대값과 기존 타입에 맞춰 선택한다. 현재 event/segment 최대 512, anchor 최대 64이므로 16-bit count면 충분하다.

의미:

- `tail_event_count`: tail로 억제·융합된 edge 이벤트 개수. 한 CROSS에 E0, E7 두 tail이 붙으면 2가 될 수 있다.
- `tail_cross_count`: tail이 하나 이상 붙은 CROSS 개수. 같은 CROSS에 여러 tail이 붙어도 1회만 증가한다.
- `tail_max_gap_steps`: 주행 전체에서 CROSS 중심과 tail 사이 gap의 최대값. V28의 last gap과 구분한다.
- `map_valid`: 기존 track map validity 판단을 그대로 반영한다.

`tail_cross_count`를 추가하려면 각 pending/recent CROSS에 “이미 affected cross로 집계했는가” 플래그를 둬 중복 증가를 막는다. 기존 융합 조건은 바꾸지 않는다.

### 4.3 line quality summary

```c
typedef struct {
    uint16_t loss_episode_count;
    uint16_t recovery_success_count;
    uint16_t max_loss_ms;
    uint32_t lost_frame_count;
    uint16_t max_edge_dwell_normal_ms;
    uint16_t max_edge_dwell_turn_ms;
} FirstDriveQualitySummary_t;
```

각 값의 정의를 엄격히 지킨다.

- `loss_episode_count`: line이 valid에서 invalid로 처음 바뀐 시점부터 안정 복구될 때까지를 1 episode로 센다.
- `recovery_success_count`: 첫 valid frame이 아니라, 기존 `FIRST_DRIVE_REACQUIRE_FRAMES`인 5개 연속 valid frame을 만족해 recovery가 실제 해제될 때 1 증가한다.
- `max_loss_ms`: 한 episode 안에서 센서가 연속으로 line을 보지 못한 최대 시간. reacquire 중 첫 valid frame이 들어온 순간까지의 invalid duration을 보존한다.
- `lost_frame_count`: 기존 저수준 invalid frame 누적. 디버그 페이지에서만 사용한다.
- `max_edge_dwell_normal_ms`: 확정 TURN이 아닌 상황에서 edge에 머문 최대 시간.
- `max_edge_dwell_turn_ms`: 확정 TURN 중 edge에 머문 최대 시간.

TURN에서는 큰 edge dwell이 정상일 수 있다. 두 값을 합쳐 사용자 품질을 나쁘다고 판단하면 270도 원형 구간을 오진할 수 있으므로 반드시 분리한다.

### 4.4 speed summary

```c
typedef struct {
    uint64_t center_sps_sum;
    uint32_t sample_count;
    uint16_t center_sps_avg;
    uint16_t center_sps_max;
    uint16_t target_sps_max;
    uint16_t left_sps_max;
    uint16_t right_sps_max;
} FirstDriveSpeedSummary_t;
```

규칙:

- motor가 활성화된 1차 주행 control sample만 합산한다.
- `center_sps_sum`과 `sample_count`만 1 kHz 경로에서 O(1)로 누적한다.
- 평균 division은 정지 finalize 또는 menu 표시 시 1회 수행한다. ISR에서 64-bit division을 하지 않는다.
- 합계는 64-bit로 두어 긴 주행에서도 overflow를 피한다.
- average는 실제 applied center SPS 기준으로 정의하고 UI에 `RUN AVG` 또는 `CTR AVG`로 표시한다.
- 출발이 400 SPS이므로 minimum speed는 품질 판단에 유용하지 않아 기록하지 않아도 된다.
- L/R max는 steering이 반영된 실제 wheel command 기준으로 기록한다.

### 4.5 stop snapshot

정상 END, 수동 stop, fault가 결정된 순간 아래 정보를 O(1) 복사한다.

```c
typedef struct {
    uint32_t center_step;
    uint32_t elapsed_ms;
    FirstDriveState_t state;
    FirstDrivePhase_t phase;

    int16_t line_position;
    int16_t last_valid_position;
    uint8_t sensor_mask;
    uint8_t raw_mask;
    uint8_t line_mask;
    uint8_t spill_mask;
    uint8_t line_valid;

    int16_t p_term;
    int16_t d_term;
    int16_t steer;
    int16_t target_steer;
    uint16_t steer_limit;

    uint16_t target_center_sps;
    uint16_t current_center_sps;
    uint16_t left_sps;
    uint16_t right_sps;

    uint16_t loss_ms;
    uint16_t loss_limit_ms;
    uint16_t edge_dwell_ms;
    uint8_t bridge_active;
    uint8_t boost_active;

    /* 마지막 marker evidence 전체 */
    TrackMarkerType_t marker_type;
    uint8_t marker_confidence;
    uint8_t marker_edge_mask;
    uint8_t marker_entry_mask;
    uint8_t marker_exit_mask;
    uint8_t marker_max_center_count;
    uint8_t marker_wide_center_run;
    uint8_t marker_both_overlap_run;
    uint32_t marker_center_step;

    uint32_t control_tick_count;
    uint32_t irq_count;
} FirstDriveStopSnapshot_t;
```

실제 기존 필드 타입과 이름을 우선하되 정보 손실이 없어야 한다.

가장 중요한 순서:

1. stop reason/fault가 확정된다.
2. 현재 제어값이 아직 살아 있을 때 snapshot을 복사한다.
3. runtime summary/final track summary를 확정한다.
4. 그 다음 `Motor_DriveStop()` 또는 target/current/L/R 0 초기화를 수행한다.

모든 stop 경로가 같은 공통 finalize 함수로 들어오게 하여 경로별 누락을 방지한다. 예:

```c
static void FirstDrive_FinalizeRunRecord(FirstDriveStopReason_t reason,
                                         FirstDriveFault_t fault);
```

중복 호출 방지 플래그를 두어 final record는 한 번만 확정한다.

### 4.6 최종 run record와 API

```c
typedef struct {
    uint8_t valid;
    FirstDriveStopReason_t stop_reason;
    FirstDriveFault_t fault;
    FirstDriveStopSnapshot_t stop;
    FirstDriveMarkerSummary_t markers;
    FirstDriveQualitySummary_t quality;
    FirstDriveSpeedSummary_t speed;
    /* 필요하면 기존 최근 marker/phase log의 고정 snapshot */
} FirstDriveRunRecord_t;
```

권장 API:

```c
void FirstDrive_GetRunRecord(FirstDriveRunRecord_t *out);
```

구현 원칙:

- run 시작 시 runtime accumulator와 final record를 명시적으로 0 초기화한다.
- 주행 중에는 작은 runtime accumulator를 O(1) 갱신한다.
- stop 시 final record를 한 번 만든 뒤 immutable로 유지한다.
- LCD stopped 페이지는 live telemetry가 아니라 final record를 주 데이터로 사용한다.
- 기존 phase log 3개와 marker log 5개가 telemetry 내부에 있다면, stopped 화면이 안정적으로 볼 수 있도록 final record에 복사하거나 stop 이후 해당 로그가 변하지 않는다는 수명 규칙을 명확히 보장한다.
- 큰 live telemetry와 더 큰 run record를 하나로 합치지 않는다. IRQ disable 상태에서 거대한 구조체를 매번 복사하는 설계를 피한다.
- menu 호출에서 짧게 IRQ를 막고 final record를 복사하는 방식은 가능하지만, 구조체 크기가 커지면 `valid/generation` 기반 일관성 복사 또는 stop 이후 immutable 특성을 활용한다. 주행 중 화면에서 final record를 읽을 필요는 없다.

---

## 5. 기록 알고리즘 상세

### 5.1 marker count는 event publish 시 O(1) 증가

LCD를 그릴 때 최대 512개의 event를 반복 scan하지 않는다. First Drive가 public track event를 확정·publish하는 한 지점에서 count를 증가시킨다.

분류 순서:

1. BOTH 계열이며 center step < 300이면 START.
2. 정상 종료 조건을 통과한 최종 BOTH이면 END.
3. LEFT, RIGHT, CROSS, UNKNOWN은 저장되는 최종 public type대로 증가.
4. tail로 앞선 CROSS에 병합되어 public event가 되지 않은 검출은 marker total/type count를 증가시키지 않는다.

`total_count`는 개별 public type count의 합과 항상 일치해야 한다. 구현 시 total을 별도로 증가시키더라도 finalize에서 아래 invariant를 검사하거나 host test로 검증한다.

```text
total == start + left + right + cross + end + unknown
```

segment/anchor count, overflow, map validity는 `Track_FinalizeSegments()` 이후 track 모듈의 최종 결과에서 채운다.

### 5.2 line loss episode 상태 머신

기존 line recovery 제어 상태와 별개로 summary용 작은 상태를 둔다.

필요 runtime 필드 예:

```c
uint8_t loss_episode_active;
uint16_t current_invisible_ms;
uint16_t episode_max_invisible_ms;
```

동작:

- valid → 첫 invalid:
  - active가 아니면 `loss_episode_count++`.
  - `loss_episode_active = 1`.
  - invisible duration 시작.
- invalid 지속:
  - current invisible duration 증가.
  - 기존 `lost_frame_count` 증가 동작 유지.
- 첫 valid frame:
  - invisible duration을 `max_loss_ms` 후보로 반영하고 current duration을 멈춘다.
  - 하지만 episode active는 아직 해제하지 않는다.
- reacquire 도중 invalid가 다시 나오면:
  - 같은 episode로 유지한다.
  - 새로운 episode로 세지 않는다.
  - 새 invisible streak를 시작한다.
- 5개 연속 valid frame으로 기존 recovery가 실제 종료될 때:
  - `recovery_success_count++`.
  - `loss_episode_active = 0`.
- loss 상태에서 fault/stop:
  - 진행 중 streak를 max에 반영한다.
  - recovery success는 증가하지 않는다.

따라서 정상적으로 모두 복구되었다면 대체로 `loss_episode_count == recovery_success_count`이고, line lost fault라면 episode가 recovery보다 1 클 수 있다.

카운터 포화(saturating increment)를 사용하는 기존 스타일이 있으면 따른다. wrapping으로 0이 되지 않게 한다.

### 5.3 edge dwell 최대값

현재 threshold 의미를 유지한다.

- outer edge 진입 기준: 약 `|position| >= 2600`
- release 기준: 약 `|position| <= 2200`
- 중심 방향 200 이상 progress 시 watchdog progress reset

현재 `edge_dwell_ms`를 갱신한 뒤 phase가 confirmed TURN인지에 따라 별도 max에 반영한다.

```text
TURN 확정 상태 → max_edge_dwell_turn_ms
그 외          → max_edge_dwell_normal_ms
```

TURN에서 긴 dwell은 fault 판단 자료가 아니라 코스 특성 자료다. 기존 TURN 허용 정책을 바꾸지 않는다.

### 5.4 tail 통계

V28의 suppression/merge 동작은 그대로 둔다.

tail 융합 성공 시에만:

- `tail_event_count++`
- gap이 최대값보다 크면 `tail_max_gap_steps` 갱신
- 해당 CROSS의 첫 tail이면 `tail_cross_count++`

tail 후보였으나 조건을 통과하지 못한 이벤트를 성공 merge 수에 넣지 않는다. 현재 last gap 진단을 유지해도 되지만 V29 화면에는 max gap을 사용한다.

### 5.5 END guard reject와 overflow

- 종료선 후보가 V28 END 방어에 의해 거절될 때 기존 `end_guard_reject_count`를 유지·최종 record로 복사한다.
- track event/segment capacity는 512, anchor capacity는 64다.
- track overflow와 anchor overflow를 각각 보존한다.
- 어느 하나라도 발생하면 map은 2차 주행용으로 안전하지 않으므로 summary 상태는 FAIL로 표시한다.

### 5.6 speed sample

motor output command가 최종 계산된 후, 실제 적용값을 O(1) 누적한다.

- center applied SPS를 sum/sample/max에 반영.
- target center SPS max 기록.
- 좌·우 applied SPS max 기록.
- 정지 후 0으로 만드는 frame은 run average sample에 넣지 않는다.
- idle/menu 시간은 넣지 않는다.
- 1 ms마다 정확히 한 번만 기록되는지 확인한다. 여러 함수 경로에서 중복 기록하면 평균이 왜곡된다.

---

## 6. 종료 품질 판정

LCD의 첫 3페이지는 raw 값 나열보다 판단을 돕는 상태를 보여준다. 판정은 단순하고 설명 가능해야 한다.

### 6.1 최상위 run status

- `SUCCESS` (초록): END marker로 정상 종료했고 map valid이며 overflow가 없다.
- `STOPPED` 또는 `INCOMPLETE` (노랑): 사용자가 수동 정지했다.
- `FAILED` (빨강): fault 또는 map invalid/overflow.

### 6.2 track quality

- `FAIL`:
  - fault가 발생했거나,
  - map invalid이거나,
  - track/anchor overflow가 있다.
- `CHECK`:
  - 정상 END이지만 UNKNOWN > 0, 또는
  - END guard reject > 0, 또는
  - CROSS count != anchor count.
- `GOOD`:
  - 정상 END,
  - map valid,
  - UNKNOWN 0,
  - END reject 0,
  - overflow 없음,
  - CROSS count == anchor count.

line loss episode가 존재한다는 이유만으로 BAD/FAIL로 만들지 않는다. 복구 성공 여부와 최대 시간을 수치로 보여준다. TURN edge dwell 역시 품질 FAIL 조건으로 쓰지 않는다.

---

## 7. 종료 LCD UI: 3그룹 × 3페이지

### 7.1 navigation 규칙

기존 `FIRST_DRIVE_FAULT_PAGE_COUNT 4`와 `first_drive_fault_page` 중심 구조를 아래 개념으로 교체한다.

```c
typedef enum {
    FIRST_DRIVE_RESULT_GROUP_SUMMARY = 0,
    FIRST_DRIVE_RESULT_GROUP_MARKERS,
    FIRST_DRIVE_RESULT_GROUP_DEBUG
} FirstDriveResultGroup_t;

#define FIRST_DRIVE_RESULT_GROUP_COUNT     3U
#define FIRST_DRIVE_RESULT_PAGES_PER_GROUP 3U
```

상태는 `first_drive_result_group`, `first_drive_result_page(0..2)`로 관리한다.

- L/R 짧게: 현재 그룹 안에서 1/3 → 2/3 → 3/3 순환.
- C 짧게: SUMMARY → MARKERS → DEBUG → SUMMARY. 그룹 이동 시 page는 0으로 시작한다.
- C 길게: 기존처럼 상위/메인 메뉴로 복귀.
- 정상 END: SUMMARY 1/3으로 진입.
- 수동 정지: SUMMARY 1/3으로 진입하되 노란 상태.
- fault 정지: DEBUG 1/3으로 즉시 진입해 이유를 바로 보여준다.

중요:

- stopped/fault 상태에서 C 짧게 처리를 generic C start/arm 처리보다 먼저 수행해 재출발로 오인되지 않게 한다.
- footer는 모든 페이지에서 `L/R PAGE  C:NEXT` 또는 현재 다음 그룹명을 일관되게 보여준다.
- stopped record는 변하지 않으므로 주기적으로 전체를 다시 그릴 필요가 없다. 진입/버튼 입력 등 기존 menu redraw 방식에 맞춘다.

### 7.2 SUMMARY 1/3 — 한눈에 보는 주행 결과

예시:

```text
FIRST DRIVE SUCCESS
END MARKER / MAP READY
TIME 41.2s STEP 62420
MARK 45 CROSS 8
L/R PAGE C:MARKERS
```

다른 정지 유형:

- 수동: `FIRST DRIVE STOPPED`, `MANUAL / MAP INCOMPLETE`
- fault: `FIRST DRIVE FAILED`, 아래 줄에 짧은 fault 명칭
- map invalid: `END REACHED / MAP INVALID`

표시 값:

- stop reason 기반 status
- elapsed time, center step
- public marker total, cross count
- map ready/invalid

### 7.3 SUMMARY 2/3 — 주행 품질

예시:

```text
TRACK QUALITY GOOD
LOSS EVT 2 MAX 18ms
RECOVER 2 EDGE-N 34ms
TAIL 1 END REJ 0
UNK 0 OVERFLOW NO
```

`EDGE-N`은 non-turn 최대 edge dwell이다. TURN 최대값은 debug에서 보여준다.

좁은 LCD에 맞춰 값이 커지면 고정 폭을 넘기지 않도록 포화 표기(`999+`) 또는 단위 축약을 사용한다. 숫자를 잘라 다른 값처럼 보이게 하면 안 된다.

### 7.4 SUMMARY 3/3 — 실제 속도 요약

예시:

```text
SPEED SUMMARY
CTR AVG/MAX 3010/3820
WHEEL MAX L4210 R4090
BASE 3820 T2200 C2400
REC S/T 1400/1800
```

표시 의미:

- `CTR AVG/MAX`: 실제 applied center SPS의 run average/max.
- `WHEEL MAX`: 실제 좌/우 최대 command.
- `BASE`: 현행 직선 기준 3820.
- `T`: TURN cap 2200.
- `C`: CROSS/marker cap 2400.
- `REC S/T`: 복구 직선/회전 1400/1800.

하단 상수는 현재 실제 제어 상수와 같은 단일 정의를 참조해야 한다. 숫자를 menu.c에 별도로 복제해 서로 달라지게 만들지 않는다. 기존 상수가 drive.c 내부 private이면, UI용 run record에 실제 사용값을 stop 시 저장하거나 공용 read-only 정의/API를 만든다. 제어 상수의 소유권을 과도하게 재구성하지 않는다.

### 7.5 MARKERS 1/3 — marker 구성

예시:

```text
MARKER SUMMARY
TOTAL 45 S1 L18 R17
CROSS 8 END1 UNK0
SEG 45 ANCHOR8
L/R PAGE C:DEBUG
```

- `S`: START
- `L/R`: LEFT/RIGHT
- CROSS/END/UNKNOWN
- segment와 anchor 수

TOTAL 계산 invariant를 지켜야 하며 START를 END로 세지 않는다.

### 7.6 MARKERS 2/3 — CROSS/END 무결성

예시:

```text
CROSS / END CHECK
CROSS 8 ANCHOR 8 OK
TAIL E1 C1 GAP 39/160
END 1 GUARD REJECT 0
MAP READY OVERFLOW NO
```

- `TAIL E`: 융합된 tail event 개수.
- `TAIL C`: tail 영향을 받은 cross 개수.
- `GAP 39/160`: 실제 최대 gap / 허용 상한.
- CROSS와 anchor가 같으면 `OK`, 다르면 `CHECK`.

track overflow와 anchor overflow를 한 줄에 합치기 어려우면 `OVF T0 A0`처럼 구분한다.

### 7.7 MARKERS 3/3 — 최근 marker 사건

최근 marker log를 사람이 읽을 수 있는 이름으로 보여준다. 가능한 한 최근 4~5개를 최신순으로 표시한다.

예시:

```text
RECENT MARKERS NEW>OLD
0 END   CF60 E81 S62390
1 CROSS CF40 E00 S62301
2 CROSS CF100 E81 S61486
3 LEFT  CF80 E01 S59779
L/R PAGE C:DEBUG
```

약어:

- `CF`: confidence. 이것은 확률 백분율이 아니라 내부 confidence/score 값이다.
- `E`: edge mask.
- `S`: center step.

기존 `BT`, `CR`, `E0` 같은 내부 2글자 표기는 debug에서는 허용되지만 이 marker 요약 페이지에서는 `END`, `CROSS`, `LEFT`, `RIGHT`, `UNK`, `START`로 풀어 쓴다.

### 7.8 DEBUG 1/3 — 정확한 정지 이유

fault 시 이 페이지로 자동 진입한다.

fault 예시:

```text
STOP: LINE LOST
AT S38120 T24.8 PH TL
LOSS 121/120 LASTP-2680
MASK M01 R03 L00 Q00
MAP INVALID
```

정상 종료 예시:

```text
STOP: END MARKER
AT S62420 T41.2 PH AL
LASTP -84 POS 18 VALID1
BT CF60 E81 MC0 WR0 BO4
MAP READY
```

반드시 구현할 fault 문자열 mapping:

- 기존 `FirstDriveFault_t`의 모든 enum 값에 대해 사람이 이해할 수 있는 고정 문자열을 제공한다.
- `FAULT NONE`만 반복하지 말고 `LINE LOST`, `EDGE STUCK`, `TRACK OVERFLOW`, `ANCHOR OVERFLOW` 등 실제 enum 의미를 표시한다.
- enum이 추가되어 mapping이 빠진 경우 `UNKNOWN FAULT n`으로 숫자도 보여준다.

### 7.9 DEBUG 2/3 — 정지 순간 제어 snapshot

예시:

```text
CTRL SNAPSHOT
POS 18 LP-84 P-84 D102
STR 320 T320 LIM2200
SPD T2400 C2380 L2700 R2060
PH AL LOSS0 EDGE0 B0 U0
```

LCD 폭에 따라 5줄로 재배치하되 다음 정보는 유지한다.

- current/last position
- P, D, applied/target steer, steer limit
- target/current center SPS, L/R SPS
- phase, current loss, edge dwell, bridge/boost

이 값들은 stop 이후 0이 된 live motor 값이 아니라 **stop 직전에 보존한 snapshot**이어야 한다.

### 7.10 DEBUG 3/3 — raw marker/system

예시:

```text
RAW EVENT / SYSTEM
MK BT CF60 E81 MC0 WR0 BO4
IN 7E OUT81 CTR62420
TAIL E1 C1 G39 ER0
IRQ62420 CTL62405 LOSTF120
```

안정적으로 정의할 약어:

- `CF`: confidence/score
- `MC`: max center sensor count
- `WR`: wide center run
- `BO`: both overlap run
- `IN/OUT`: marker entry/exit mask
- `CTR`: marker center step
- `ER`: END guard reject count
- `IRQ/CTL`: interrupt/control tick count
- `LOSTF`: invalid frame 누적

가능하다면 DEBUG 3에 `EDGE N/T` 최대값도 포함한다. 공간이 부족하면 RAW marker 필드를 2줄로 재배치한다. 디버그 그룹은 가독성보다 정보량 우선이지만, 필드명과 값 경계는 명확해야 한다.

---

## 8. menu.c 구현 지침

현재 stopped/fault rendering은 대략 `menu.c`의 기존 FIRST DRIVE STOP 관련 4페이지 함수 영역에 있고, 버튼 처리는 파일 후반부에 있다. 정확한 line number는 편집 후 달라질 수 있으므로 symbol과 상태 enum으로 찾는다.

구현 순서:

1. 기존 stopped/fault 4페이지의 유용한 raw 필드를 inventory한다.
2. 새 9페이지에 정보가 빠짐없이 배치되었는지 확인한다.
3. result group/page 상태와 rendering helper를 추가한다.
4. 공통 header/footer, fault string, marker type string helper를 둔다.
5. stopped/fault 진입 시 run record를 읽고 최초 그룹/page를 선택한다.
6. L/R/C short/C hold 우선순위를 구현한다.
7. 기존 main menu 전환, debounce, long-press 동작을 보존한다.

권장 rendering 구조:

```c
static void Menu_DrawFirstDriveResult(void);
static void Menu_DrawFirstDriveSummaryPage(uint8_t page, const FirstDriveRunRecord_t *r);
static void Menu_DrawFirstDriveMarkerPage(uint8_t page, const FirstDriveRunRecord_t *r);
static void Menu_DrawFirstDriveDebugPage(uint8_t page, const FirstDriveRunRecord_t *r);
```

화면용 계산은 menu task에서 해도 되지만, 512 event scan 같은 반복 작업은 금지한다. 평균 division과 짧은 문자열 formatting 정도만 허용한다.

---

## 9. stop/fault 경로 감사 항목

Luna는 `drive.c`에서 1차 주행이 멈추는 모든 경로를 검색해 아래를 표로 정리한 뒤 구현해야 한다.

| 경로 | reason | fault | snapshot 시점 | finalize 호출 |
|---|---|---|---|---|
| 정상 END | END_MARKER | NONE | speed 0 처리 전 | 1회 |
| C/중앙 버튼 수동 정지 | MANUAL | NONE | speed 0 처리 전 | 1회 |
| line lost fault | FAULT | LINE_LOST | fault flag 후, motor stop 전 | 1회 |
| edge stuck 등 모든 fault | FAULT | 해당 enum | fault flag 후, motor stop 전 | 1회 |
| track/anchor overflow | FAULT 또는 기존 정책 | 해당 enum/flag | map 상태 보존 후 | 1회 |

실제 코드에서 overflow가 즉시 fault가 아니라 map invalid만 만드는 정책이라면, 주행 제어 정책은 바꾸지 말고 stop reason은 실제 정지 이유를 유지하면서 summary를 FAIL로 표시한다. 진단과 제어 정책을 혼동하지 않는다.

공통 finalize 함수는 idempotent해야 한다.

```c
if (run_record_finalized) {
    return;
}
```

run 시작 함수에서는 이전 결과가 새 주행 화면에 남지 않도록 final flag, accumulator, ring log cursor를 적절히 초기화한다.

---

## 10. 실시간성과 메모리 주의사항

### 10.1 1 kHz 경로에서 허용

- 정수 덧셈/비교/max 갱신.
- 작은 상태 플래그 전이.
- marker publish 시 type counter 1회 증가.
- stop 시 작은 고정 구조 snapshot 1회 복사.

### 10.2 금지

- ISR/control tick에서 LCD 호출.
- ISR에서 `sprintf`, float formatting, heap allocation.
- ISR에서 64-bit division.
- 매 tick track event 512개 scan.
- 매 UI redraw에서 전체 event/segment scan.
- telemetry getter에 거대한 record를 합쳐 긴 IRQ-off memcpy를 반복.

### 10.3 메모리

RAM 여유는 충분하지만 STM32에서는 정렬 때문에 예상보다 구조체가 커질 수 있다.

- bool/enum/32-bit 필드 순서를 조정해 padding을 줄인다.
- `__attribute__((packed))`는 unaligned access 위험 때문에 무조건 쓰지 않는다.
- 기존 ring log를 불필요하게 중복 복제하지 않는다. 다만 stop 이후 안정성을 위해 필요한 고정 snapshot은 허용한다.
- build 결과에서 V28 대비 Flash/RAM 증가량을 보고한다.

---

## 11. 검증 계획

### 11.1 host/unit test 또는 test harness

가능한 기존 V28 host harness를 확장하거나 별도 작은 test를 만든다.

필수 항목:

1. marker total invariant
   - START/LEFT/RIGHT/CROSS/END/UNKNOWN 합과 TOTAL 일치.
2. start line 분류
   - 300 step 미만 BOTH가 START이며 END count를 올리지 않음.
3. V28 split-cross 회귀
   - CROSS 뒤 tail이 별도 BOTH/END public event로 저장되지 않음.
4. tail 통계
   - tail event count, affected cross count, max gap이 정확함.
   - 한 CROSS에 tail 2개면 E=2, C=1.
5. END 방어
   - 거절 후보는 reject count 증가, END count 증가 없음.
6. CROSS/anchor
   - finalize 후 정상 map에서 수가 같음.
7. overflow
   - track/anchor overflow flag와 quality FAIL 반영.
8. loss episode
   - invalid 여러 frame은 episode 1회.
   - reacquire 중 재손실은 같은 episode.
   - 5 valid frame 완료 때만 recovery success 증가.
   - fault 종료 episode는 success 증가 없음.
9. edge max
   - normal/confirmed-turn max가 분리됨.
10. speed sum
   - sample 중복 없음, 평균/max 정확함, 64-bit overflow 없음.
11. stop snapshot
   - motor 0 처리 후에도 record의 stop L/R/current speed가 직전값으로 남음.
12. finalize once
   - 동일 stop 경로가 반복 호출되어도 count/snapshot이 변하지 않음.

### 11.2 UI 상태 테스트

각 경우를 강제로 재현하거나 fixture record로 rendering path를 확인한다.

- 정상 END → SUMMARY 1/3.
- 수동 stop → SUMMARY 1/3, 노란 STOPPED/INCOMPLETE.
- 각 fault enum → DEBUG 1/3, 정확한 문자열.
- L/R은 현재 그룹 3페이지 안에서만 순환.
- C short는 그룹 순환 후 page 1로 이동.
- C hold는 main으로 복귀.
- stopped 상태 C short가 새 주행 start/arm을 트리거하지 않음.
- 큰 숫자, 0개, 최대 count에서 문자열 overflow/잔상 없음.

LCD 실기 또는 screenshot으로 9페이지 모두 검토한다. 새 문자열이 이전 줄의 긴 문자를 남기지 않도록 line clear/padding 방식을 기존 UI 관례에 맞춘다.

### 11.3 주행 로직 회귀

- V29 instrumentation을 끈 것과 같은 입력 trace에서 phase, speed target, marker publish 결과가 V28과 동일해야 한다.
- First Drive 제어 상수 변경 없음.
- V27 Second Drive source와 동작 변경 없음.
- V28 교차로 통과 성공 조건 유지.

### 11.4 clean build

프로젝트 루트에서 실행한다.

```bash
cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --clean-first -j 8
```

warning/error를 확인하고 binary size를 기록한다.

---

## 12. 권장 구현 순서

1. V27/V28 plan과 work result를 읽고 현재 diff를 확인한다.
2. 1차 주행의 모든 stop/fault 경로와 telemetry update 위치를 추적한다.
3. `drive.h`에 stop reason, summary, snapshot, run record/API를 정의한다.
4. `drive.c`에 runtime accumulator 초기화와 O(1) 갱신을 추가한다.
5. marker publish 지점에 START 포함 type count를 추가한다.
6. line loss episode/recovery 성공/max loss를 구현한다.
7. normal/turn edge max와 speed 통계를 구현한다.
8. `track.c/h`에 V28 동작을 건드리지 않는 tail max gap/affected cross 진단을 추가한다.
9. 모든 stop 경로를 공통 snapshot/finalize 함수에 연결한다.
10. `menu.c`에 9페이지 rendering과 navigation을 구현한다.
11. host test/harness로 count·episode·snapshot·V28 회귀를 검증한다.
12. clean build와 size 확인.
13. `APP_VERSION_NUMBER`를 29로 확정한다.
14. 작업 결과 문서를 작성한다.

버전 번호는 구현 중간에 먼저 올려도 되지만 최종 build에서 반드시 V29로 확인한다.

---

## 13. 완료 조건

아래가 모두 충족되어야 V29 완료다.

- [ ] 정상 END, 수동 stop, fault가 명확히 구분된다.
- [ ] stop snapshot이 motor 값 0 처리 전에 저장된다.
- [ ] 1차 주행 종료 후 실제 직전 speed/position/control 값이 보인다.
- [ ] line loss episode와 lost frame이 혼동되지 않는다.
- [ ] recovery success는 5-frame 안정 복구 완료 시에만 증가한다.
- [ ] normal edge와 TURN edge 최대 dwell이 분리된다.
- [ ] START가 END와 분리되고 marker TOTAL invariant가 성립한다.
- [ ] tail event/cross/max gap이 정확하다.
- [ ] CROSS/anchor, segment, overflow, map validity가 표시된다.
- [ ] 실제 applied speed 평균/최대가 기록된다.
- [ ] 3 SUMMARY + 3 MARKERS + 3 DEBUG 화면이 동작한다.
- [ ] fault 시 DEBUG 1/3에 정확한 정지 이유가 즉시 표시된다.
- [ ] C short/L/R/C hold navigation이 요구사항대로 동작한다.
- [ ] ISR/control path에 scan, formatting, LCD, division이 추가되지 않았다.
- [ ] V28 CROSS-tail-END 로직 회귀가 없다.
- [ ] V27 Second Drive 로직이 변경되지 않았다.
- [ ] host test 및 clean build가 통과한다.
- [ ] V28 대비 Flash/RAM 증가량이 기록된다.
- [ ] 앱 버전이 29다.

---

## 14. Luna가 작성할 작업 결과 문서

구현 완료 후 다음 파일을 생성한다.

`codex_worked_review/V29_FIRST_DRIVE_RUN_RECORD_AND_LOG_UI_WORK_RESULT.md`

반드시 포함할 내용:

1. 변경 파일 목록과 파일별 변경 요약.
2. 실제 추가한 구조체/enum/API 이름.
3. 모든 stop 경로와 reason/fault mapping.
4. 각 통계의 실제 갱신 위치와 계산 방식.
5. marker TOTAL 분류 결과와 START 처리.
6. tail event/cross/max gap 구현 방식.
7. 9개 LCD 페이지의 최종 실제 문구 또는 사진.
8. 버튼 navigation 결과.
9. host test 항목과 PASS/FAIL 원문.
10. clean build 명령 및 결과.
11. V28 대비 Flash/RAM 증감.
12. 수정하지 않은 V27/V28 핵심 로직 확인.
13. 남은 위험, 실차 확인이 필요한 항목.

작업 결과 문서에 “구현 완료”만 적지 말고, Codex가 소스와 시험 결과를 재검토할 수 있을 정도로 구체적인 근거를 남긴다.

---

## 15. 최종 주의사항

- 이 버전의 최우선 가치는 **정확한 기록과 이해 가능한 사후 로그**다.
- 기록을 추가하면서 제어 결과가 변하면 안 된다.
- 하나의 CROSS가 tail 때문에 END로 중복 저장되던 V28 이전 문제를 되살리지 않는다.
- CROSS는 센서 event이고 anchor는 finalize된 맵의 재동기화 bookmark다. UI와 변수 이름에서 둘을 혼동하지 않는다.
- confidence(`CF`)는 확률 백분율로 설명하지 않는다.
- 정상 TURN의 긴 edge dwell을 오류처럼 평가하지 않는다.
- 마지막 속도 0은 정지 명령의 결과일 뿐 정지 직전 주행 속도가 아니다. 반드시 snapshot 순서를 지킨다.
- 사용자용 3페이지는 판단 중심, marker 3페이지는 맵 검증 중심, debug 3페이지는 원인 재현 중심으로 역할을 분리한다.

