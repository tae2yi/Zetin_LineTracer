# V30 Second Drive Speed Corridor·Hard Brake·LCD Freeze 작업 결과

작성일: 2026-08-12  
대상 프로젝트: /Users/ehoi/STM32CubeIDE/line_tracer_2026  
적용 버전: APP_VERSION_NUMBER 30U

## 1. 구현 결과 요약

V30 계획의 Second Drive 기능을 현재 작업 트리 위에 구현했다.

- straight/curve/overall 속도 profile과 effective speed 계산을 분리했다.
- 1 kHz planner에서 limiter reason, geometry source, run statistics, transition trace를 O(1)로 누적한다.
- replay 방향 marker pair를 map event index와 독립적으로 추적한다.
- expected CROSS/final END 직전의 제한된 same-side local close repair를 추가했다.
- centered stability와 position hysteresis를 갖는 fast gate를 추가했다.
- CROSS 접근과 final END 직선 corridor에서 lookahead와 TURN braking을 연결했다.
- final corridor가 검증된 END에 대해서만 Second Drive non-blocking hard brake를 사용한다.
- Second Drive active 화면은 최초 1회 표시 후 LCD redraw를 잠근다.
- 정지/FAULT 후 3페이지 결과 화면에서 속도 제한 원인과 pair/repair/brake 상태를 확인할 수 있다.
- First Drive 분기, V27 anchor resync, V28 CROSS-tail/END guard, V29 기록/UI의 기존 경로는 유지했다.

실차 센서·모터·LCD 시험은 이 환경에서 수행하지 못했으므로, 아래 문서의 host/mock 및 ARM build 결과와 별도로 실차 확인 항목을 마지막에 정리했다.

## 2. 변경 파일

### Header

- Main/Inc/app_version.h
  - 앱 버전을 30으로 변경했다.
- Main/Inc/second_drive.h
  - V30 profile 범위, effective target API, geometry/limiter/local-repair enum, planner status, run stats, 16-entry trace를 추가했다.
- Main/Inc/drive.h
  - V29 First Drive run record와 V30 planner telemetry 연결 구조를 유지/확장했다.
- Main/Inc/motor.h
  - Motor_DriveBrakeHoldStart/SetVref/Process/Finish/IsActive() API와 hard/hold Vref 상수를 추가했다.
- Main/Inc/track.h
  - marker entry/exit mask와 CROSS-tail 진단 확장, TrackMapPairDiagnostics_t getter를 추가했다.

### Source

- Main/Src/second_drive.c
  - V30 profile/effective cache, replay pair FSM, local repair, fast gate, CROSS/final END corridor, braking, stats/trace를 구현했다.
  - SecondDrivePlanner_RecordEndBrake()는 정상 END 처리 후 expected index가 마지막 event 다음으로 이동한 경우에도 map의 마지막 BOTH event를 reference로 사용한다.
  - 1 kHz 제동거리 계산은 32-bit 범위로 처리해 새 64-bit division이 control path에 들어가지 않도록 했다.
- Main/Src/drive.c
  - final target/recovery reason 기록, marker reject counter, Second Drive stats 시작/종료 hook, confirmed END hard-brake RUNOUT 분기를 추가했다.
  - FirstDrive_IsEndMarkerEvent()의 V28 END guard를 통과한 경우에만 END 처리를 진행한다.
- Main/Src/motor.c
  - 현재 phase output을 유지한 채 timer를 멈추는 HARD 30 ms → reduced 120 ms → OFF 상태 머신을 추가했다.
  - brake active 동안 Motor_DriveSetSpeeds()는 실패한다.
- Main/Src/track.c
  - 기존 segment FSM은 변경하지 않고 pair final diagnostic과 CROSS-tail 통계를 추가했다.
- Main/Src/menu.c
  - STRAIGHT → CURVE → ALL SPEED → ARM 입력 순서, effective speed 표시, active LCD freeze, Second Drive 결과 3페이지를 추가했다.

## 3. Direction marker pair 해석

시립대 규정의 marker 의미를 기존 Track_SegmentTypeFromEvent() semantics 그대로 유지했다.

| replay event | pair state | 다음 segment 의미 |
|---|---|---|
| closed + R/L | open, 해당 방향 | RIGHT/LEFT curve |
| open + 같은 방향 | close | 해당 close marker 이후 STRAIGHT |
| open + 반대 방향 | 기존 pair 전환 후 새 방향 open | 새 방향 RIGHT/LEFT curve |
| confirmed CROSS | pair reset | CROSS straight-through |
| END/BOTH | pair를 새로 닫지 않음 | open 상태이면 unmatched 진단 보존 |

관련 symbol:

- First Drive map finalize: Track_SegmentTypeFromEvent()
- map 진단: Track_GetMapPairDiagnostics()
- Second Drive replay 갱신: SecondDrive_ApplyReplayPairEvent()
- CROSS reset: SecondDrive_ResetReplayPair()
- local close: SecondDrive_TryLocalCloseRepair()

### 정상 final sequence fixture

입력:

    R, R, R, L, R, L, L, END

/private/tmp/v30_track_pair_harness.c에서 Track_FinalizeSegments() 결과를 확인했다. 초기 출발 직선은 segment[0]이고, marker sequence에 해당하는 결과는 다음과 같다.

| segment index | type |
|---:|---|
| 1 | RIGHT |
| 2 | STRAIGHT |
| 3 | RIGHT |
| 4 | LEFT |
| 5 | RIGHT |
| 6 | LEFT |
| 7 | STRAIGHT |
| 8 | END |

pair diagnostic는 turn_open=0, unmatched_turn_at_end=0이다.

### 마지막 close 누락 fixture

입력:

    R, R, R, L, R, L, END

마지막 L close가 없으므로 TrackMapPairDiagnostics_t는 다음을 남긴다.

    turn_open=1
    turn_direction=-1 (LEFT)
    unmatched_turn_at_end=1
    tail segment before END = LEFT

host 결과:

    V30 track pair harness PASS

## 4. Replay pair와 local close repair

replay pair 상태는 다음 static state로 유지한다.

- planner_replay_turn_open
- planner_replay_turn_direction
- planner_replay_turn_open_step
- planner_last_direction
- planner_last_direction_step
- planner_last_repair_step
- planner_last_repair_direction

정상 expected event는 SecondDrive_EventMatchesExpected()와 SecondDrive_AdvanceAfterEvent()를 먼저 통과한 뒤 SecondDrive_ApplyReplayPairEvent()를 호출한다. 따라서 정상 same-side close는 local repair로 소비되지 않고 PAIR_CLOSE가 된다. 정상 CROSS는 anchor 처리 후 CROSS_RESET source가 된다.

### local repair 허용 조건

SecondDrive_TryLocalCloseRepair()는 아래를 모두 확인한다.

- map valid이고 sync가 SECOND_DRIVE_SYNC_MAP
- 확정 LEFT/RIGHT event이며 confidence가 20 이상
- replay pair가 open
- event 방향과 open 방향이 동일
- expected event가 CROSS 또는 final END
- 현재 map segment 방향이 open 방향과 동일
- open marker와의 gap이 TRACK_MARK_COOLDOWN_STEPS보다 큼
- 동일 step/direction duplicate가 아님

BOTH, UNKNOWN, CROSS-tail merged fragment는 SecondDrive_EventDirection()에서 repair 후보가 되지 않는다. 거절 원인은 SECOND_DRIVE_LOCAL_REPAIR_* enum과 planner_status.local_repair_reject_reason에 남긴다.

### local repair 동작과 불변성

repair 성공 시:

- replay pair만 close
- geometry_source = SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR
- local_close_repair_active, direction, step, count 갱신
- planner_expected_event_index 불변
- planner_segment_start_step 불변
- sync를 SEEK CROSS로 변경하지 않음
- Track event/segment/anchor 원본 배열을 변경하지 않음

missing-close fixture에서 실제 전후 기준은 다음과 같다.

    expected_event_index: 6 -> 6
    segment_start_step:   3010 -> 3010
    sync:                 MAP -> MAP
    geometry source:      LOCAL_CLOSE_REPAIR
    local repair count:   1

/private/tmp/v30_second_drive_planner_harness.c는 event/segment/anchor를 repair 전 byte copy하고 repair 후 memcmp()로 모두 비교한다.

동일 host harness에서 다음도 확인했다.

- expected CROSS + same-side open pair local repair
- open pair가 중앙 정렬 30 frame을 유지해도 straight performance target 금지
- final END local repair 후 EXIT 50 frame 승격
- normal expected sequence에서는 PAIR_CLOSE 유지

host 결과:

    V30 planner local repair PASS idx=8 start=4010 src=2

src=2는 정상 full fixture의 최종 SECOND_DRIVE_GEOMETRY_PAIR_CLOSE이며, local repair 검증은 내부 assertion으로 별도 확인된다.

## 5. V30 speed profile와 UI

적용 상수:

    STRAIGHT default       5600 SPS
    CURVE default          3000 SPS
    APPROACH bonus         300 SPS
    EXIT bonus             800 SPS
    center performance max 6200 SPS
    overall default        100%

UI 조정 범위:

    STRAIGHT 4800..6000, step 100
    CURVE    2600..3600, step 100
    ALL SPEED 90..120%, step 5%

100% 계산:

    straight = 5600
    curve    = 3000
    approach = 3300
    exit     = 3800

120% 계산:

    straight = 6720 -> 6200 clamp
    curve    = 3600
    approach = 3960
    exit     = 4560

effective target는 second_drive_effective_* cache에 저장하며 setter에서만 percent multiplication/division을 수행한다. recovery 1400/1800, map invalid/SEEK fallback, fault/emergency stop은 performance profile과 분리했다. wheel 최종 제한은 기존 MOTOR_DRIVE_MAX_SPS = 6500을 유지한다.

READY 화면은 다음 세 줄로 nominal/effective를 분리한다.

    STRAIGHT B5600 E5600
    CURVE    B3000 E3000
    ALL SPEED 100%

## 6. Limiter reason, geometry source, stats

SecondDriveLimitReason_t에는 FAST_STRAIGHT, FAST_CROSS_APPROACH, FAST_CROSS_EXIT, FAST_END_CORRIDOR, FAST_LOCAL_CLOSE_REPAIR, CURVE_APPROACH/CRUISE/EXIT, MAP_INVALID, SEEK_CROSS, SEGMENT_UNCERTAIN, MARKER_PAIR_UNCLOSED, PHASE_NOT_STRAIGHT, POSITION, TURN_BRAKE, RECOVERY, MAX_CLAMP, END_BRAKE를 별도로 유지했다.

최종 reason 순서는 다음과 같다.

1. planner map/sync/fallback 판단
2. geometry/pair/fast gate
3. phase profile
4. 다음 LEFT/RIGHT restriction braking
5. final target에 recovery cap이 적용되면 RECOVERY로 덮음
6. center performance max clamp가 실제로 target을 낮추면 MAX_CLAMP
7. SecondDrivePlanner_RecordFinalTarget()에서 control sample당 정확히 한 reason을 누적

SecondDriveRunStats_t는 control sample sum/max, limiter histogram, fast entry/exit, mismatch/resync, local repair, unmatched turn, marker reject, END step error, brake completion을 보관한다. transition trace는 reason 또는 geometry source가 바뀔 때만 ring에 추가한다.

END event 처리 후 planner index가 마지막 event 다음으로 이동해도 RecordEndBrake()가 마지막 BOTH event를 다시 찾아 다음을 계산한다.

    expected_end_step = recorded final BOTH center_step
    end_step_error    = actual END trigger step - expected_end_step

## 7. Fast gate와 corridor

fast gate:

- enter: line valid + STRAIGHT/CROSS phase + abs(position) <= 500, 30 consecutive frames
- hold: 500 < abs(position) < 900
- exit: line invalid, phase not STRAIGHT/CROSS, 또는 abs(position) >= 900

host assertion:

- 29 centered frame: OFF
- 30th frame: ON
- position 700: ON 유지
- position 900: OFF
- fast entry/exit count: 1/1
- 32 control sample에 limiter sample 32개
- limiter transition trace count: 3

### CROSS approach

cross_approach_corridor는 map valid, sync MAP, expected CROSS, pair closed, geometry evidence, live STRAIGHT, stable gate, line valid, no recovery, no overdue를 모두 요구한다. current map segment가 RIGHT인 fixture에서 same-side close를 local repair한 뒤 expected CROSS가 유지되는 경우에도 current segment를 corridor로 override하고 CROSS 뒤 fast geometry부터 다음 실제 TURN까지 lookahead한다.

host assertion:

    CROSS local repair source
    expected index unchanged
    cross_approach_corridor=1
    straight profile target

pair open, TURN phase, SEEK CROSS에서는 corridor를 열지 않는다.

### final END corridor

final helper는 expected event가 마지막 BOTH이고 마지막 finalized segment가 END인지 확인한다. final corridor는 pair closed, source evidence, live stable STRAIGHT 또는 close 이후 50-frame EXIT stability, line valid, no recovery를 추가로 요구한다.

정상 pair close final fixture는 FAST_END_CORRIDOR reason을 사용한다. local close repair final fixture는 source가 local임을 보존하기 위해 FAST_LOCAL_CLOSE_REPAIR reason을 사용하며, 두 경우 모두 final straight target을 사용한다.

open pair final fixture는 30 frame 이상 중앙 정렬해도:

    final_end_corridor = 0
    limit_reason = MARKER_PAIR_UNCLOSED
    straight performance target = not allowed

local close 후 EXIT phase는 SECOND_DRIVE_FINAL_EXIT_STABLE_FRAMES = 50에서 planner-only final straight로 승격하며 shared drive_course_phase를 강제로 바꾸지 않는다.

## 8. TURN braking

기존 제동식을 유지했다.

    braking_steps = ceil((high_sps^2 - low_sps^2)
                         / (2 * 10000 SPS/s))
    brake margin = 300 steps
    accel re-enable margin = 150 steps

다음 LEFT/RIGHT restriction의 low target은 effective approach speed다. END가 valid final corridor이면 END를 장거리 1800 restriction으로 사용하지 않는다. corridor가 불완전하면 기존 safe END fallback을 유지한다.

## 9. END hard brake

관련 API:

- Motor_DriveBrakeHoldStart()
- Motor_DriveBrakeHoldSetVref()
- Motor_DriveBrakeHoldProcess()
- Motor_DriveBrakeHoldFinish()
- Motor_DriveBrakeHoldIsActive()

상태:

    confirmed END
      -> timer stop, current phase output 유지
      -> HARD 30 ms @ MOTOR_VREF_DAC_BRAKE_HARD (= RUN current)
      -> REDUCED 120 ms @ MOTOR_VREF_DAC_BRAKE_HOLD (= 1024)
      -> Motor_Stop(), Vref/output OFF

HAL_Delay()는 사용하지 않는다. brake active 동안 speed command는 거절한다. 반복 start는 idempotent이고 emergency/fault는 Motor_DriveStop()으로 즉시 full off한다.

V28 guard evidence:

- FirstDrive_IsEndMarkerEvent()가 BOTH, start-ignore, overlap, wide-CROSS, CROSS-tail 조건을 확인한다.
- FirstDrive_HandleMarkerEvent()는 이 helper가 true일 때만 FirstDrive_StopAtEndMarker()를 호출한다.
- First Drive는 기존 Motor_DriveStop() 경로를 유지한다.
- Second Drive도 final corridor가 false이면 normal safe stop으로 돌아간다.

motor host/mock 결과:

    V30 motor brake mock PASS

mock에서 timer 즉시 중지, phase output 유지, speed command 실패, 30/120 ms Vref 전환, output off를 확인했다.

## 10. LCD freeze와 결과 화면

active state 최초 진입 시 한 번만 다음을 표시한다.

    SECOND DRIVE RUNNING
    DISPLAY UPDATE LOCKED
    CENTER = EMERGENCY STOP
    HOLD C TO GO BACK

Menu_SecondDriveIsLiveRunning()이 active/RUNOUT을 식별하고 second_drive_run_screen_locked가 이미 켜졌으면 telemetry getter, snprintf, LCD draw 없이 return한다. COUNTDOWN은 주기 redraw를 허용한다. center 입력은 Menu_Process()의 별도 즉시 stop 경로에 남겨 두었다.

stopped/FAULT 결과 화면:

### Result 1/3

    SECOND DRIVE END 또는 FAULT
    TIME ... STEP ...
    CTR AVG/MAX .../...
    SYNC M... R... FINAL ...
    L/R PAGE  C HOLD:BACK

### Result 2/3

    SPEED LIMIT SHARE
    FAST ...% CURVE ...%
    BRAKE ...% SAFE ...%
    POS ...% REC ...%
    RAW ... SAMPLES MAX ...

### Result 3/3

    END BRAKE V... DONE/N/A
    PAIR OPEN/CLOSED ... SRC ...
    LCR... S... DIR ...
    EXP S... ERR...
    LAST ... REJ N... O... C... B...

실제 LCD hardware render-count 측정은 하지 못했으므로, active render 0회는 Menu_UpdateSecondDrive()/renderer static path의 source inspection으로 확인했다.

## 11. Host/ARM test evidence

### Host

다음 harness를 현재 Main/Src/track.c와 Main/Src/second_drive.c로 재빌드했다.

    /private/tmp/v27_track_test
    /private/tmp/v28_track_harness
    /private/tmp/v29_track_harness
    /private/tmp/v30_track_pair_harness
    /private/tmp/v30_second_drive_planner_harness
    /private/tmp/v30_motor_brake_mock

결과:

    v27: events=5 segments=5 anchors=1 overflow=0 anchor_overflow=0
         ... (existing V27 binary output)
    V28 track harness PASS
    V29 track harness PASS
    V30 track pair harness PASS
    V30 planner local repair PASS idx=8 start=4010 src=2
    V30 motor brake mock PASS

V27 원본 C fixture는 현재 /private/tmp에 남아 있지 않아 기존 V27 binary를 실행했다. V28/V29/V30 track fixture는 현재 track source로 재빌드했다.

### ARM clean build

실행 명령:

    cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=Debug
    cmake --build build/Debug --clean-first -j 8

결과:

    RAM   44,872 B / 272 KB = 16.11%
    FLASH 132,940 B / 512 KB = 25.36%
    Built target 2026_LINE_TRACER_STEP

V29 작업 결과 문서의 baseline:

    RAM   44,288 B / 272 KB = 15.90%
    FLASH 122,604 B / 512 KB = 23.38%

V29 대비:

    RAM   +584 B
    FLASH +10,336 B

정적 확인:

- source 범위 git diff --check 통과
- Main/Src/second_drive.c object에서 새 __aeabi_uldivmod/64-bit division symbol 없음
- brake 경로의 HAL_Delay 없음
- Second Drive planner stack usage:
  - SecondDrivePlanner_GetTargetSps: 120 bytes
  - SecondDrivePlanner_OnEvent: 24 bytes
  - FirstDrive_UpdateMotorCommand: 112 bytes
  - HAL_TIM7_IRQ_Handler: 8 bytes
- map symbol 크기:
  - planner_status: 0x50 = 80 bytes
  - planner_run_stats: 0x1d8 = 472 bytes

clean build에는 V29부터 남아 있던 기존 warning 1개가 있다.

    Drivers/BSP/ST7735/st7735_lcd.c:50:
    unused variable 'text'

V30 변경 파일에서 새 warning/error는 발생하지 않았다.

## 12. 회귀 및 미검증 범위

회귀 확인:

- 기존 Track_SegmentTypeFromEvent() pair semantics를 유지했다.
- 정상 R,R,R,L,R,L,L,END가 마지막 STRAIGHT로 finalize된다.
- 누락 close가 unmatched LEFT로 진단된다.
- V27/V28/V29 track harness가 PASS했다.
- First Drive hard-stop branch는 Second Drive mode branch와 분리되어 있다.
- V28 END guard symbol과 CROSS-tail guard 호출 순서를 유지했다.
- V29 result record/UI source는 그대로 두고 Second Drive UI path만 추가했다.

아직 실차에서 확인하지 못한 항목:

- 실제 marker detector가 마지막 close marker를 어느 reject gate에서 놓치는지
- curve 3000 SPS에서 line loss/outer boost/edge dwell/wheel 6500 clamp
- CROSS 전 fast corridor에서 실제 braking margin과 다음 TURN 정지 거리
- final END hard brake의 정지 거리와 coil hold의 기계적 안정성
- SPI/LCD 실제 render count와 화면 가독성
- ARM board에서 center emergency가 RUNOUT hold 중 즉시 full off하는지

## 13. 상위 모델 전달 사항과 질문

상위 구현 계획 담당 모델에는 다음을 전달한다.

1. V30 구현은 APP_VERSION_NUMBER=30U로 끝냈다.
2. 정상 pair close와 local repair를 geometry source로 분리했으며, local repair는 expected index/segment start/map array를 수정하지 않는다.
3. final END local corridor의 limiter reason은 FAST_LOCAL_CLOSE_REPAIR, 정상 pair corridor는 FAST_END_CORRIDOR로 구분했다.
4. V30 planner stats/trace와 END expected-step fallback을 추가했다.
5. ARM clean build는 통과했지만 기존 ST7735 unused-variable warning 1개가 남아 있다.

확인 질문:

- local repair로 닫힌 final END corridor를 결과 화면/limiter share에서 FAST_LOCAL_CLOSE_REPAIR로 유지할지, FAST_END_CORRIDOR로 통합하고 geometry source만 LOCAL로 표시할지 결정이 필요하다.
- valid final corridor가 아닐 때 END fallback을 fixed 1800 SPS로 유지한 현재 구현이 의도한 안전 정책인지, 기존 overall semantics에 따라 1800×overall로 표시/적용할지 실차 전 확인이 필요하다.
- ST7735의 기존 unused text warning을 V30 범위에서 별도 제거할지 결정이 필요하다.

## 14. 권장 실차 확인 순서

1. 스탠드에서 straight 5600, curve 3000, overall 100%와 brake Vref/timer/output 상태 확인
2. 마지막 R -> R -> R -> L -> R -> L -> L -> END marker sequence를 저속으로 기록
3. close marker가 빠진 map fixture에서 2차 local repair와 SRC LOCAL 확인
4. 동일 marker가 2차에서도 빠지면 PAIR OPEN과 safe fallback 확인
5. 단일 일정 곡률에서 curve profile과 recovery/outer boost/6500 wheel clamp 확인
6. CROSS 포함 긴 직선에서 FAST_CROSS_IN, CROSS reset, 다음 TURN braking 확인
7. 마지막 긴 직선에서 FAST_END_CORRIDOR 또는 FAST_LOCAL_CLOSE_REPAIR와 END hard brake 확인
8. overall 100% → 105% → 110% 순서로 올리고 120%는 라인/모터 안정성 확인 후 시험

실차 결과에서는 Result 2/3 limiter share, Result 3/3 pair/source/local repair/reject/end brake, 실제 stop step error를 함께 기록해야 한다.
