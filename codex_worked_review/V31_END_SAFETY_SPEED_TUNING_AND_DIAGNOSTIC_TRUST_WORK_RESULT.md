# V31 END Safety·Speed Tuning·Diagnostic Trust 작업 결과

작성일: 2026-08-12
대상 프로젝트: /Users/ehoi/STM32CubeIDE/line_tracer_2026
적용 버전: APP_VERSION_NUMBER 31U

## 1. 구현 요약

V31 계획에 따라 V30의 기본 속도 profile은 유지하면서 END 안전 정지 경로, marker 진단 신뢰성, limiter episode 계측, First Drive 곡선 조정 UI와 Second Drive 결과 화면을 확장했다.

- final performance corridor와 confirmed END stop policy를 분리했다.
- pair-open·position gate·phase gate가 final END fallback을 조기 반환으로 우회하지 않도록 target 결정 순서를 통합했다.
- END safe fallback은 1800 SPS cap과 sticky state를 사용한다.
- V28 END guard를 통과한 Second Drive END는 corridor와 관계없이 active brake를 시도한다.
- brake 시작 실패·emergency·fault를 full-off stop mode로 기록한다.
- raw S0/S7 edge가 있는 physical candidate episode만 marker reject 통계에 반영한다.
- limiter histogram의 control tick당 1 sample invariant를 유지하면서 episode count/max streak와 SEEK/pair/position/recovery duration을 추가했다.
- First Drive CURVE를 2200..2600 SPS, 100 SPS step으로 board에서 조정할 수 있게 했고 기본값은 2200 SPS로 유지했다.
- Second Drive 결과를 4페이지로 확장해 END, limiter, candidate diagnostics를 확인할 수 있게 했다.

실차·실제 LCD 하드웨어 시험은 이 환경에서 수행하지 못했다. 아래 PASS는 현재 source 기반 host/mock 또는 ARM compile 결과다.

## 2. 변경 파일

### Firmware source/header

- Main/Inc/app_version.h
  - APP_VERSION_NUMBER를 30U에서 31U로 변경했다.
- Main/Inc/drive.h
  - First Drive curve 상수 FIRST_DRIVE_CURVE_CRUISE_*, floor 상수를 추가했다.
  - FirstDriveConfig_t.curve_cruise_sps와 First Drive candidate episode/reason 통계를 추가했다.
  - FirstDrive_SetCurveCruiseSps(), FirstDrive_GetEffectiveCurveFloorSps()와 V31 host wrapper를 선언했다.
- Main/Src/drive.c
  - candidate episode state machine과 First/Second Drive run record 연결을 추가했다.
  - V28 FirstDrive_IsEndMarkerEvent()/FirstDrive_IsDefensiveCrossTail() guard를 유지했다.
  - confirmed END 이후 Second Drive active brake, brake 실패 full-off, emergency/fault stop mode 기록을 연결했다.
  - configured curve target/floor/APPROACH cap을 적용했다.
- Main/Inc/second_drive.h
  - SECOND_DRIVE_END_APPROACH_SPS 1800U, SECOND_DRIVE_LIMIT_END_APPROACH_SAFE, END policy, stop mode, candidate reject reason, limiter episode/run diagnostic fields를 추가했다.
- Main/Src/second_drive.c
  - END distance fallback/sticky cap, stale final END policy snapshot, limiter episode accounting, candidate episode accounting, resync/local-repair diagnostics와 stop statistics를 구현했다.
- Main/Inc/menu.h, Main/Src/menu.c
  - First Drive curve UI를 추가했다.
  - Second Drive 결과 page 0..3, END policy/stop mode/candidate/longest-safe episode 문자열을 추가했다.
  - active Second Drive running screen은 최초 표시 후 LCD API를 다시 호출하지 않도록 lock을 유지한다.
- Main/Src/motor.c
  - V30 brake state machine을 유지하고 V31 mock이 start 실패와 full-off 경로를 검증하도록 연결했다. V31에서 brake Vref/hold timing 기본값은 변경하지 않았다.

### 재현 가능한 프로젝트 내부 테스트

tests/v31/에 다음을 추가했다.

- run_host_tests.sh: 전체 host compile/run 및 source checks.
- v31_planner_harness.c: END fallback, stop mode, limiter episode, candidate API, resync, local-repair negative tests.
- v31_track_pair_harness.c: 현재 track.c pair semantics regression.
- v31_motor_brake_mock.c: 현재 motor.c mock 기반 brake/full-off test.
- v31_curve_contract_harness.c: curve range/floor contract.
- v31_candidate_harness.c: 현재 drive.c candidate gate/episode test.
- tests/v31/host/{main.h,tim.h,dac.h,gpio.h}: host HAL stub.

## 3. V30/V31 planner 결정 순서

관련 실제 symbol은 다음과 같다.

- SecondDrivePlanner_GetTargetSps() — 최종 target 결정
- SecondDrive_DistanceToFinalEnd() — final END 거리 계산
- SecondDrive_SetPlannerTarget() — status와 target 반영
- planner_end_fallback_active — run 중 sticky END fallback state

V30은 map/sync/segment 상태 뒤에 pair, position, phase, fast gate, segment END 등의 branch가 각각 target을 정하고 조기 return하는 구조였다. 따라서 final END expected 상태라도 pair-open 또는 position/phase/fast gate branch가 먼저 반환하면 END distance fallback이 실행되지 않을 수 있었다.

V31은 다음 순서로 정리했다.

1. map/sync/segment 구조가 완전히 불신 가능한 경우에는 기존 First Drive fallback/SEEK 경로를 유지한다.
2. 유효한 map segment 안에서는 final corridor, curve/pair/position/phase/fast gate를 이용해 nominal target을 먼저 정한다.
3. 다음 TURN/CROSS 제한을 nominal target에 restriction cap으로 적용한다.
4. final_end_expected && !final_corridor이면 SecondDrive_DistanceToFinalEnd()로 END 거리를 계산한다.
5. ceil((high^2-low^2)/(2*10000)) + 300 step window에 진입하면 SECOND_DRIVE_END_APPROACH_SPS로 cap하고 planner_end_fallback_active를 latch한다.
6. latch 이후에는 line centering, position/phase 변화, pair 상태가 target을 다시 fast로 올리거나 limit_reason을 덮지 못한다.
7. 마지막으로 SecondDrive_SetPlannerTarget()을 호출한다.

즉 V31에서 제거된 것은 safety 불신 branch 자체가 아니라, final END fallback보다 앞서 실행되던 우회성 early return이다. map invalid/SEEK와 segment bounds처럼 END 거리를 신뢰할 수 없는 경로는 기존 안전 fallback을 의도적으로 유지했다.

## 4. END fallback test

현재 source로 재빌드한 v31_planner_harness의 원문:

~~~
V31 END fallback fixture: far_distance=5400 far_target=3000 near_distance=1200 near_target=1800 sticky_target=1800 policy=2
V31 corridor-false confirmed END: stop_mode=1 attempted=1 succeeded=1 hold_ms=150
~~~

해석:

- pair-open final END에서 먼 위치는 5400 steps, nominal safe target은 3000 SPS였다.
- END까지 1200 steps로 진입하면 계산된 braking window 안에서 1800 SPS로 제한됐다.
- 다음 centered frame과 position/phase gate 변화에도 sticky_target=1800이 유지됐다.
- policy=2는 SECOND_DRIVE_END_POLICY_SAFE_APPROACH다.
- sticky 이후 limit_reason도 SECOND_DRIVE_LIMIT_END_APPROACH_SAFE로 유지되도록 보완했고 host assertion으로 확인했다.

정상 closed-pair final corridor fixture에서는 final_end_corridor=1, end_fallback_active=0, Second Drive effective straight target 유지를 확인했다. 따라서 performance fast corridor와 END safe approach가 하나의 safe_speed로 합쳐지지 않는다.

## 5. Confirmed END active brake와 stop mode

FirstDrive_StopAtEndMarker(uint32_t confirmed_step)에서 V28 guard를 통과한 confirmed END를 Second Drive stop event로 처리한다.

- FirstDrive_IsEndMarkerEvent()는 MARKER_EVENT_BOTH, start-ignore 이후 step, overlap, center/wide guard를 확인한다.
- FirstDrive_IsDefensiveCrossTail()이 true인 defensive CROSS-tail BOTH는 END로 처리하지 않고 reject한다.
- 따라서 raw BOTH 또는 V28 reject가 FirstDrive_StopAtEndMarker()와 Motor_DriveBrakeHoldStart()를 호출하지 않는다.
- confirmed END에서는 SecondDrivePlanner_IsFinalEndCorridor()를 active brake 조건으로 사용하지 않는다.
- Motor_DriveBrakeHoldStart(MOTOR_VREF_DAC_BRAKE_HARD)를 먼저 시도하고 성공하면 SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE로 기록한다.
- 시작 실패 시 motor full-off, SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE_FAILED_FULL_OFF 기록 후 stopped/finalize로 간다.
- FirstDrive_EmergencyStop()은 SECOND_DRIVE_STOP_MODE_EMERGENCY_FULL_OFF, FirstDrive_SetFault()는 SECOND_DRIVE_STOP_MODE_FAULT_FULL_OFF를 기록하고 Motor_DriveStop()을 호출한다.
- final BOTH event 처리 뒤 planner event index가 다음으로 이동해도 SecondDrivePlanner_RecordEndBrake()가 finalized map의 마지막 BOTH event를 reference로 사용해 expected step/error를 보존한다.

planner fixture 원문에서 corridor false인데도 active brake가 실행된 결과는 다음과 같다.

~~~
stop_mode=1 attempted=1 succeeded=1 hold_ms=150
~~~

stop_mode=1은 SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE다.

## 6. Brake mock 결과

tests/v31/v31_motor_brake_mock.c는 현재 Main/Src/motor.c를 include하고 다음을 검증한다.

- DAC start 실패 시 Motor_DriveBrakeHoldStart()가 false를 반환하고 즉시 DAC stop + phase full-off.
- HARD hold 시작 후 repeated start가 idempotent.
- 30 ms 뒤 reduced hold Vref 전환.
- reduced hold 중 Motor_DriveStop()이 full-off.
- Motor_DriveBrakeHoldFinish()가 full-off.
- timer stop path가 양쪽 timer에 적용됨.

원문 결과:

~~~
V31 motor brake mock PASS
~~~

## 7. Marker candidate episode 진단

First Drive state는 FirstDriveMarkerCandidateRuntime_t로 유지된다.

1. FirstDrive_MarkerCandidateObserve()가 raw line->marker_mask & SENSOR_MARKER_MASK를 먼저 본다.
2. raw edge가 없으면 candidate를 시작하지 않으며 일반 off-center line frame은 reject로 세지 않는다.
3. raw edge가 있는 동안 하나의 physical episode에 edge_union, reject_reason_mask, entry/last step을 누적한다.
4. FirstDrive_GetMarkerEdges()가 NO_LINE, OFF_CENTER, NO_CENTER_MASK, BRIDGE gate 이유를 mask에 추가하고, ProcessMarker가 CROSS-tail suppressed/LOW_CONFIDENCE 이유를 추가한다.
5. accepted direction event면 episode를 accepted로 표시한다.
6. TRACK_MARK_CLEAR_FRAMES clear debounce 뒤 FirstDrive_MarkerCandidateMaybeFinalize()가 episode 하나만 finalize한다. confirmed END가 clear debounce보다 먼저 오면 FirstDrive_MarkerCandidateForceFinalize()가 run 종료 전에 닫는다.
7. rejected episode는 mask의 각 reason count를 한 번씩 증가시키며, last_candidate_reject_reason은 mask에서 가장 낮은 set bit를 선택한다. 이는 다중 reason이 있었을 때 deterministic한 priority다.

reason enum은 NO_LINE, OFF_CENTER, NO_CENTER_MASK, BRIDGE, CROSS_TAIL_SUPPRESSED, LOW_CONFIDENCE, COOLDOWN_OR_DUPLICATE다. Second Drive도 SecondDrivePlanner_RecordMarkerCandidateEpisode()로 동일한 physical episode count와 reason mask를 run stats에 기록한다. 기존 SecondDrivePlanner_RecordMarkerReject()는 legacy single-reason 호출을 하나의 rejected episode로 감싼다.

raw edge 없는 1000 frame test 원문:

~~~
V31 raw candidate gate: no_edge_frames=1000 episodes=0 rejected=0
~~~

candidate accepted/rejected test 원문:

~~~
V31 candidate episodes: off_center=1 mask={OFF_CENTER,NO_LINE} accepted=1 rejected=0
V31 candidate harness PASS
~~~

같은 harness의 assertion으로 off-center raw edge episode 하나는 rejected 1/OFF_CENTER 1, 중간 NO_LINE 추가 시 한 episode 안에서 OFF_CENTER+NO_LINE 두 reason, accepted direction episode는 accepted 1/rejected 0, clear debounce는 episode를 쪼개지 않는 것을 확인했다.

## 8. Limiter episode와 핵심 진단

구현 symbol:

- SecondDrive_RecordLimiterEpisodeSample() — control tick당 최종 reason 1개를 누적
- SecondDrive_FinalizeLimiterEpisode() — reason 전환/run finalize 시 streak를 닫고 max duration 반영
- SecondDrivePlanner_RecordFinalTarget() — recovery_slow가 최종 reason이면 RECOVERY로 덮은 뒤 sample을 1회만 기록

V30의 limiter_samples[SECOND_DRIVE_LIMIT_COUNT]와 transition trace는 유지했다. 추가된 limiter_episode_count[], limiter_max_consecutive_samples[], first/last step과 SEEK/pair-open/position/recovery episode/max duration은 모두 O(1) update이며 동적 할당이 없다.

A10 -> B20 -> A30 fixture에서 position/recovery reason을 사용해 다음을 assertion했다.

- A/position episode count 2, max streak 30.
- B/recovery episode count 1, max streak 20.
- run finalize 시 마지막 active streak flush.
- 모든 reason histogram 합계가 control_samples와 동일.

즉 control tick당 limiter histogram sample은 정확히 1개다. recovery_slow가 true인 tick은 planner nominal reason과 별도로 최종 SECOND_DRIVE_LIMIT_RECOVERY 하나만 누적한다.

## 9. V27 resync 및 local repair negative test

현재 Main/Src/track.c와 Main/Src/second_drive.c를 host compile했다.

~~~
V31 resync fixture: sync=0 expected=2 anchor=0 pair=0
~~~

입력 흐름은 mismatch EDGE -> SECOND_DRIVE_SYNC_SEEK_CROSS -> forward CROSS -> unambiguous current-source anchor -> MAP 복귀다. 결과에서 sync MAP(0), expected event index 2, anchor order 0, replay pair closed를 확인했다.

TestResyncAndNegativeRepair()가 확인한 local repair 거절은 다음 전부다.

- no open pair -> SECOND_DRIVE_LOCAL_REPAIR_NO_OPEN
- wrong-side direction -> SECOND_DRIVE_LOCAL_REPAIR_WRONG_SIDE
- expected event가 ordinary edge -> SECOND_DRIVE_LOCAL_REPAIR_EXPECTED_NOT_BOUNDARY
- confidence 10 -> SECOND_DRIVE_LOCAL_REPAIR_LOW_CONFIDENCE
- cooldown gap -> SECOND_DRIVE_LOCAL_REPAIR_DUPLICATE
- map invalid 및 SEEK non-CROSS -> SECOND_DRIVE_LOCAL_REPAIR_MAP_NOT_SYNC

SecondDrive_TryLocalCloseRepair()에는 segment direction mismatch 분기(SECOND_DRIVE_LOCAL_REPAIR_SEGMENT_MISMATCH)도 남아 있으며, 공개 fixture에서는 먼저 expected boundary/side 조건에 도달하는 입력을 만들지 않아 해당 분기의 별도 출력은 만들지 않았다. 모든 실행 fixture에서 expected index, segment start와 source map 배열을 수정하지 않는 구조를 유지했다.

## 10. First Drive curve tuning contract

상수와 기본값:

~~~
FIRST_DRIVE_CURVE_CRUISE_DEFAULT_SPS 2200
FIRST_DRIVE_CURVE_CRUISE_MIN_SPS     2200
FIRST_DRIVE_CURVE_CRUISE_MAX_SPS     2600
FIRST_DRIVE_CURVE_CRUISE_STEP_SPS     100
FIRST_DRIVE_CURVE_FLOOR_DELTA_SPS     400
FIRST_DRIVE_CURVE_FLOOR_MIN_SPS      1800
~~~

Menu_ChangeFirstDriveCurve()는 READY 상태에서만 L/R로 변경하며 active/armed/countdown/run 상태에서는 FirstDrive_SetCurveCruiseSps()가 거절한다. Menu_RenderFirstDrive()는 BASE %u CURVE %u, FLOOR %u를 표시한다.

FirstDrive_GetTargetBaseSps() 정책:

- APPROACH cap은 계속 2200 SPS다.
- confirmed TURN/EXIT cap은 configured curve다.
- outer floor는 max(1800, curve - 400)다.
- CROSS/marker slow와 recovery 1400/1800 SPS는 curve setting과 독립이다.

v31_curve_contract_harness는 다음 curve floor를 확인했다.

~~~
2200 -> 1800
2300 -> 1900
2400 -> 2000
2500 -> 2100
2600 -> 2200
V31 curve contract harness PASS
~~~

따라서 curve 2200 fixture에서는 V30과 APPROACH 2200, TURN/EXIT 2200, outer floor 1800 조건이 동일하다. curve 2400 fixture는 APPROACH 2200, confirmed TURN/EXIT center 2400, outer floor 2000, recovery straight/turn 1400/1800을 source contract로 고정했다. 실제 sensor trace를 입력한 ARM target trace와 board 동작은 아직 수행하지 않았다.

## 11. Second Drive profile 기본값

Main/Inc/second_drive.h와 Main/Src/second_drive.c의 기본 profile은 다음과 같다.

~~~
straight 5600 SPS
curve    3000 SPS
overall  100%
recovery 1400 / 1800 SPS (First Drive recovery constants)
~~~

V31은 Second Drive 기본 straight/curve/overall을 올리지 않았다. Second Drive UI의 기존 조정 범위 4800..6000, 2600..3600, 90..120%도 유지했다. 실차에서 기본 run이 안정적인 것을 확인하기 전에는 curve profile을 3200 이상으로 변경하지 않는다.

## 12. Second Drive 결과 UI 및 LCD lock

결과 page는 0..3으로 동작한다.

- page 0: run summary.
- page 1: SPEED LIMIT SHARE, END_SAFE를 brake share에 포함한 limiter 비율.
- page 2: END POLICY FAST/SAFE/UNCERTAIN, fallback step/speed/distance, BRAKE ACTIVE/FAIL OFF/EMERG OFF/FAULT OFF, pair/source, expected END step/error/hold.
- page 3: MARK CANDIDATE 4/4, candidate accepted/rejected/reason, longest safe limiter episode와 SEEK/pair/recovery episode count.

실제 source 문자열은 SPEED LIMIT SHARE, END POLICY %s, MARK CANDIDATE 4/4, DISPLAY UPDATE LOCKED다. 사진은 실제 LCD를 연결해 촬영하지 못했으므로 첨부하지 않았다.

Menu_RenderSecondDrive()에서 live state 최초 진입 시에만 LCD API로 static running screen을 그리고 second_drive_run_screen_locked=1로 설정한다. 이후 locked branch는 LCD API 호출 전에 return한다. second_drive_active_render_count는 화면 진입 시 0으로 reset되고 periodic active redraw counter를 증가시키지 않는다. Menu_GetSecondDriveActiveRenderCount()와 source check가 0-count 정책을 확인하지만 실제 SPI/LCD hardware count test는 미실행이다.

## 13. 전체 host/mock test

실행 명령:

~~~
sh tests/v31/run_host_tests.sh
~~~

최종 실행 원문:

~~~
V31 END fallback fixture: far_distance=5400 far_target=3000 near_distance=1200 near_target=1800 sticky_target=1800 policy=2
V31 corridor-false confirmed END: stop_mode=1 attempted=1 succeeded=1 hold_ms=150
V31 resync fixture: sync=0 expected=2 anchor=0 pair=0
V31 planner/diagnostic harness PASS
V31 track pair harness PASS
V31 motor brake mock PASS
V31 curve contract harness PASS
V31 raw candidate gate: no_edge_frames=1000 episodes=0 rejected=0
V31 candidate episodes: off_center=1 mask={OFF_CENTER,NO_LINE} accepted=1 rejected=0
V31 candidate harness PASS
V31 source marker/version checks PASS
~~~

이 test는 임시 binary만 실행하지 않고 현재 workspace의 source/header를 매번 compile한다.

## 14. ARM clean build 및 정적 확인

계획서 지정 명령 그대로 실행했다.

~~~
cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --clean-first -j 8
~~~

결과:

~~~
RAM:   45472 B / 272 KB = 16.33%
FLASH: 138152 B / 512 KB = 26.35%
Built target 2026_LINE_TRACER_STEP
~~~

빌드 중 V31 source/header의 warning/error는 없었다. 기존 별도 warning 1개는 다음이다.

~~~
Drivers/BSP/ST7735/st7735_lcd.c:50:17:
warning: unused variable 'text' [-Wunused-variable]
~~~

추가 정적 확인:

- git diff --check -- Main/Inc Main/Src tests/v31 codex_worked_review 통과.
- grep -R -n HAL_Delay Main/Src/drive.c Main/Src/motor.c Main/Src/second_drive.c 결과 없음.
- SecondDrivePlanner_GetTargetSps disassembly에는 __aeabi_uldivmod 호출이 없다. ELF 전체의 __aeabi_uldivmod는 기존 drive.c 다른 경로에서 참조된다.
- control planner stack에는 LCD API/snprintf가 없다.
- candidate runtime은 static struct/정수 counter만 사용하며 동적 할당이 없다.
- limiter episode update는 reason transition당 O(1)이다.

## 15. V30 대비 Flash/RAM/stack

V30 HEAD tracked artifact와 V31 final ELF를 비교했다.

| 항목 | V30 | V31 | 증감 |
|---|---:|---:|---:|
| ELF text | 132,512 B | 137,724 B | +5,212 B |
| ELF data | 428 B | 428 B | 0 B |
| ELF bss/RAM | 44,440 B | 45,040 B | +600 B |
| linked total (size dec) | 177,380 B | 183,192 B | +5,812 B |
| CMake RAM region | 44,872 B | 45,472 B | +600 B |
| CMake FLASH region | 132,940 B | 138,152 B | +5,212 B |

V30 stack .su 대비:

| 함수 | V30 | V31 | 증감 |
|---|---:|---:|---:|
| SecondDrivePlanner_GetTargetSps | 120 B | 160 B | +40 B |
| SecondDrivePlanner_OnEvent | 24 B | 24 B | 0 B |
| SecondDrivePlanner_RecordFinalTarget | 56 B | 56 B | 0 B |
| FirstDrive_ProcessMarker | 40 B | 40 B | 0 B |
| FirstDrive_ControlTick | 24 B | 24 B | 0 B |
| FirstDrive_StopAtEndMarker | 16 B | 24 B | +8 B |
| Menu_RenderSecondDriveResult | 152 B | 160 B | +8 B |
| Motor_DriveBrakeHoldStart | 16 B | 16 B | 0 B |

map/nm symbol 크기:

~~~
planner_status:    V30 0x50 = 80 B   -> V31 0x64 = 100 B
planner_run_stats: V30 0x1d8 = 472 B -> V31 0x3b8 = 952 B
drive_marker_candidate: V31 0x10 = 16 B
~~~

증가한 RAM은 candidate/END/limiter 진단 기록을 위한 static telemetry이며, 속도 기본값을 올리기 위한 buffer가 아니다.

## 16. 상위 모델 전달사항과 질문

다음 항목은 상위 모델 또는 다음 작업자가 실차 검증 후 판단해야 한다.

1. pair-open/map-uncertain/position gate false 상태에서 confirmed END가 실제 chassis에서 반드시 active brake를 시작하는지 확인할 것.
2. DAC brake Vref, HARD 30 ms/reduced hold 120 ms가 실제 모터 관성·코일 발열·정지거리와 맞는지 측정할 것. V31은 기존 값을 임의 변경하지 않았다.
3. First Drive curve 2400에서 실제 sensor trace를 수집해 marker acceptance, line loss, edge dwell, PD stability를 확인할 것. 2200 결과와 비교하기 전에는 2500/2600을 사용하지 말 것.
4. Second Drive 기본 5600/3000/100% run을 먼저 통과시키고, curve는 3000 -> 3200 -> 3400 순서로 별도 시험할 것. Second Drive straight default를 V31에서 올리지 않았다.
5. 실제 LCD에서 active running 화면 최초 1회 표시와 subsequent periodic redraw 0회를 SPI trace 또는 instrumentation으로 확인할 것.
6. 마지막 close marker R -> R -> R -> L -> R -> L -> L -> END sequence에서 First Drive candidate episode와 finalized pair closed 결과를 확인할 것.

## 17. 권장 실차 시험 순서

1. 스탠드에서 First Drive CURVE 2200, Second Drive 5600/3000/100%, 센서 calibration 상태를 확인한다.
2. 스탠드에서 corridor true/false confirmed END active brake를 확인한다.
3. brake HARD/reduced hold 중 center emergency를 눌러 즉시 full-off인지 확인한다.
4. 주행 화면 LCD redraw lock을 확인한다.
5. 저속으로 R -> R -> R -> L -> R -> L -> L -> END를 주행해 candidate accepted/rejected와 pair diagnostics를 기록한다.
6. 마지막 close 누락/pair-open map을 저속으로 재현해 END 접근 시 END_SAFE와 1800 SPS, confirmed END active brake를 확인한다.
7. V30 기본 profile로 CROSS resync, final corridor/local repair, limiter share/longest episode, END step error를 기록한다.
8. Second Drive curve만 3000 -> 3200 -> 3400으로 단계적으로 올려 lap time, curve limiter, recovery, edge dwell, wheel 6500 clamp, candidate reject를 비교한다.
9. 별도 First Drive run에서 curve를 2200 -> 2300 -> 2400으로 올리며 marker map 품질과 last close acceptance를 먼저 확인한다.
10. MAP/SEEK/recovery 원인이 분리되고 안정성이 확인되기 전에는 safety fallback 또는 기본 속도를 올리지 않는다.
