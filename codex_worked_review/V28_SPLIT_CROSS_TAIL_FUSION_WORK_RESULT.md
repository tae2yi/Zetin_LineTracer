# V28 분리형 CROSS-tail 결합 구현 결과

## 1. 작업 개요

- 기준 계획: `codex_work_plan/V28_SPLIT_CROSS_TAIL_FUSION_IMPLEMENTATION_PLAN.md`
- 기준 구현: 현재 작업 트리의 V27
- 작업일: 2026-08-11
- 펌웨어 버전: `Main/Inc/app_version.h`의 `APP_VERSION_NUMBER 28U`
- 핵심 목표: `CROSS E00` 직후 분리된 S0/S7 이벤트를 같은 물리 교차로의 tail로 결합하고, 독립 종료선 `BOTH`는 유지

실제 차체를 사용할 수 없는 환경이므로 V28 합성 입력과 ARM clean build까지 수행했다. First Drive/Second Drive 실차 완주는 아직 확인하지 못했다.

## 2. 실제 변경 파일

- `Main/Inc/app_version.h`
  - 앱 버전을 27에서 28로 변경
- `Main/Inc/track.h`
  - `TRACK_CROSS_TAIL_MAX_GAP_STEPS 160U` 추가
  - `TrackProcessResult_t`와 `TrackCollectorDiagnostics_t` 추가
  - First/Replay process API를 결과 enum 반환형으로 변경
- `Main/Src/track.c`
  - First/Replay에 공통으로 사용하는 `Track_ProcessInternal()` 구현
  - pending build와 event publish 분리
  - CROSS-tail source/entry step 상관관계, tail evidence 병합, 저장 event 갱신 구현
  - Replay는 First Drive event/segment/anchor 배열을 수정하지 않도록 유지
- `Main/Inc/drive.h`
  - tail suppression/guard/end reject telemetry 추가
  - marker log에 entry/exit 및 wide/overlap evidence 추가
- `Main/Src/drive.c`
  - `TRACK_PROCESS_CROSS_TAIL_MERGED` 처리 추가
  - tail 결과를 planner와 marker phase switch에 전달하지 않도록 처리
  - incomplete CROSS 이후 provisional EDGE 억제
  - BOTH defensive guard와 reject count 추가
  - Init/Start에서 guard와 marker frame 상태 초기화
- `Main/Src/menu.c`
  - First Drive 정지/고장 페이지에 `TAIL N... G... E... A...` 표시
  - marker log에 `I(entry_step)`, `X(exit_step)` 표시
  - Second Drive에서 replay tail count/gap 일부 표시

`Main/Src/second_drive.c`와 `Main/Inc/second_drive.h`는 V27 MAP/SEEK/anchor resync/lookahead를 보존하기 위해 수정하지 않았다. Replay collector가 tail event를 발행하지 않으므로 planner에는 물리 event stream만 전달된다.

## 3. 최종 상수와 선택 근거

```text
TRACK_CROSS_TAIL_MAX_GAP_STEPS = 160
TRACK_MARK_CONFIRM_FRAMES      = 3
TRACK_MARK_CLEAR_FRAMES        = 5
TRACK_MARK_COOLDOWN_STEPS      = 50
FIRST_DRIVE_CROSS_PASS_STEPS   = 510  (기존 값 보존)
```

tail window는 `candidate.entry_step - source_cross.exit_step`으로 계산한다. candidate가 window 안에서 시작하면 clear frame 때문에 완성 시점이 160 step을 넘어도 source context를 유지한다. tail을 결합할 때 source `entry_frame`, `entry_step`, `center_step`, `type`은 변경하지 않고 edge/full union, run evidence, exit, confidence만 보강한다.

160 step은 계획의 실차 로그 gap 89 step을 수용하면서 기존 510-step CROSS pass보다 좁은 값이다. `TRACK_MARK_CLEAR_FRAMES`와 `TRACK_MARK_COOLDOWN_STEPS`는 변경하지 않았다.

## 4. 구현 semantics

공통 collector runtime은 pending, last event, cooldown, cross-tail pending/source exit, pending-start-in-tail flag, suppression count, 마지막 gap/edge를 보유한다.

정상 event는 다음과 같이 처리한다.

1. pending을 candidate로 build한다.
2. incomplete CROSS이면 tail 대기를 시작한다.
3. First Drive에서는 event 배열에 저장하고, Replay에서는 저장하지 않는다.
4. `TRACK_PROCESS_EVENT_READY`를 반환한다.

tail candidate는 다음 조건을 모두 만족해야 한다.

- 직전 공개 event가 CROSS
- 직전 CROSS의 edge union이 아직 E81이 아님
- candidate pending이 source exit 이후 160 step 이내에 시작
- candidate entry가 source exit보다 앞서지 않음
- `max_center_count < MARKER_WIDE_CENTER_COUNT`
- `wide_center_run < MARKER_WIDE_MIN_FRAMES`
- edge union에 S0 또는 S7이 있음

tail이면 저장 event count를 증가시키지 않고 기존 CROSS에 union/run/exit/confidence를 병합한 뒤 `TRACK_PROCESS_CROSS_TAIL_MERGED`를 반환한다. First Drive 저장 배열의 마지막 원소는 type과 원래 center를 확인한 경우에만 갱신한다. 일치하지 않으면 추측으로 배열을 수정하지 않고 overflow/fault 진단 경로로 보낸다.

Drive에서는 merged 결과에 대해 marker log newest CROSS만 refresh하고 `SecondDrivePlanner_OnEvent()` 및 marker type switch를 호출하지 않는다. 따라서 tail BOTH가 새 END가 되거나 두 번째 CROSS anchor가 되지 않는다. 독립 BOTH는 기존 bilateral overlap, start ignore, no-wide-center 조건을 그대로 통과해야 END 정지를 수행한다.

## 5. 합성 테스트 결과

임시 host harness `/private/tmp/v28_track_harness.c`에서 `Main/Src/track.c`를 ARM 의존성 없이 빌드해 실행했다.

```sh
gcc -std=c11 -Wall -Wextra -Werror -I Main/Inc \
  Main/Src/track.c /private/tmp/v28_track_harness.c \
  -o /private/tmp/v28_track_harness
/private/tmp/v28_track_harness
```

결과: `V28 track harness PASS`

검증한 입력과 결과:

- 정상 `0xBD` wide-center + E81: CROSS 1회, E81, confidence 100, tail count 0
- 실차 재현 `0x3C` CROSS E00 + cooldown 이후 E81: CROSS 1개 유지, tail merge 1회, event count 증가 없음, center 보존
- E0 tail 후 E7 tail: 두 결과 모두 merge, 최종 E81, suppression count 2, source window를 sliding하지 않음
- E0/E7이 overlap 없이 분리된 UNKNOWN edge evidence: 새 map event 없이 기존 CROSS에 결합
- incomplete CROSS 이후 160 step 밖의 E81: 독립 BOTH event로 공개
- E81을 이미 포함한 CROSS 이후 BOTH: 거리만으로 억제하지 않고 별도 BOTH로 공개
- tail window 안의 새 wide-center sequence: tail이 아닌 새 CROSS로 공개
- First/Replay 대칭: 결과 type/order가 같고 Replay가 First event 배열을 변경하지 않음
- split CROSS 후 실제 BOTH에서 map finalize: CROSS anchor 1개, CROSS segment 1개, final END segment 1개, anchor center 원래 값 유지

Provisional marker가 실제 `SensorLineMeasurement_t`에서 생성되는 경로와 모터 정지는 host harness 대상이 아니며, 해당 Drive 정적 경로는 compile 및 code review로 확인했다. 실차에서 `CR>AL/AR MPR`가 발생하지 않는지 별도 확인해야 한다.

## 6. Build 및 정적 검증

실행 명령:

```sh
cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --clean-first -j 8
arm-none-eabi-size build/Debug/2026_LINE_TRACER_STEP.elf
arm-none-eabi-nm -S --size-sort build/Debug/2026_LINE_TRACER_STEP.elf | tail -20
```

최종 size:

```text
text: 115,944 B
data:     416 B
bss:   41,304 B
RAM 합계: 41,720 B / 272 KB = 14.98%
Flash 출력: 116,360 B / 512 KB = 22.19%
```

V27 결과 문서의 기준과 비교하면 text/Flash는 약 2,424 B, bss는 약 144 B 증가했다. 증가분은 공통 collector runtime, drive telemetry/marker log evidence, tail 처리 코드에 해당한다.

`git diff --check`는 V28 source 변경 파일에서 통과했다. 새 compiler warning은 없었고, clean build에 기존 경고 하나만 남았다.

```text
Drivers/BSP/ST7735/st7735_lcd.c:50: unused variable 'text'
```

변경 코드에서는 dynamic allocation, float/double, sensor/motor/CubeMX 생성 파일 수정이 없다.

## 7. 상위 모델/실차에 전달할 확인사항

1. 문제 속도의 First Drive에서 7~8번째 교차로를 통과할 때 `TAIL N`이 증가하고 `G`가 실제 `candidate.entry - CROSS.exit` gap으로 기록되는지 확인한다.
2. split CROSS tail에서 `E81`, `A0`가 되고 First Drive가 `CR>AL/AR MPR`로 전이하지 않는지 확인한다.
3. tail suppression 뒤 `event_count`와 map anchor 수가 물리 교차로 수와 일치하는지 확인한다.
4. 최근 incomplete CROSS가 없는 독립 종료선에서 `BOTH`가 반드시 END 정지를 수행하는지 확인한다.
5. CROSS 직후 실제 코스 요소가 160 step 밖에 있는 경우 억제되지 않는지 확인한다.
6. 낮은 속도/슬립 조건에서 여러 교차로의 최대 gap을 기록한다. 160을 조정할 필요가 있으면 510을 재사용하지 말고 기록된 최대 gap과 별도 margin으로 결정한다.
7. V28 map 기반 Second Drive에서 tail event가 planner에 전달되지 않고 MAP SYNC가 유지되는지, 실제 event mismatch 시 V27 SEEK CROSS 복귀가 유지되는지 확인한다.
8. `end_guard_reject_count`가 증가한다면 Track correlation이 놓친 tail의 raw event와 entry/exit log를 함께 확인한다. 정상적인 Track 동작에서는 이 방어 카운터가 0이어야 한다.

## 8. 계획과 다르게 구현한 부분

- 계획의 공통 API 예시는 process 두 함수만 enum 반환으로 제시했지만, flush도 동일 semantics를 유지하도록 `Track_Flush()` 반환형을 enum으로 변경했다. 현재 저장소 내 호출자는 없으며 compile을 통과했다.
- `drive.c`에는 provisional guard의 현재 active 상태와 별도로 source validity를 유지해, guard active window가 만료된 뒤 늦게 완성된 candidate도 candidate entry step 기준으로 defensive 검사할 수 있게 했다. 새 event가 wide CROSS이거나 run이 reset되면 source는 갱신/해제된다.
- Second Drive planner 소스는 수정하지 않았다. First/Replay 공통 collector에서 tail을 공개하지 않는 것이 planner를 재작성하는 것보다 V27 semantics를 안전하게 보존한다고 판단했다.

실차 시험과 V28 최종 완료 판정은 위 확인사항 실행 후 상위 모델이 내려야 한다.
