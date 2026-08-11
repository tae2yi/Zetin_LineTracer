# V29 First Drive 실행 기록 및 결과 UI 구현 결과

## 1. 작업 범위와 결론

- 기준 계획: `codex_work_plan/V29_FIRST_DRIVE_RUN_RECORD_AND_LOG_UI_IMPLEMENTATION_PLAN.md`
- 작업일: 2026-08-11
- 펌웨어 버전: `APP_VERSION_NUMBER 29U`
- V27 anchor resync, V28 split CROSS-tail 병합/END guard의 제어 판단과 상수는 유지했다.

First Drive가 END, 수동 정지, fault로 종료될 때 정지 직전의 제어·센서·마커 evidence를 독립 `FirstDriveRunRecord_t`에 한 번만 확정한다. 정지/고장 메뉴는 이 확정 record만 표시하며, 3개 그룹 × 3페이지로 탐색할 수 있다. control tick에는 O(1) 누적만 추가했고, 평균 속도의 나눗셈은 record를 메뉴가 읽을 때 수행한다.

## 2. 변경 파일

- `Main/Inc/app_version.h`
  - 버전을 29로 올렸다.
- `Main/Inc/track.h`, `Main/Src/track.c`
  - marker event의 `entry_mask`/`exit_mask`를 보존한다.
  - 기존 tail fragment 수와 별도로, tail이 영향을 준 CROSS 수와 최대 tail gap을 누적한다.
  - tail 병합의 V28 공개/비공개 semantics는 바꾸지 않았다.
- `Main/Inc/drive.h`, `Main/Src/drive.c`
  - `FirstDriveStopReason_t`, marker/quality/speed summary, stop snapshot, `FirstDriveRunRecord_t`, `FirstDrive_GetRunRecord()`를 추가했다.
  - `TRACK_PROCESS_EVENT_READY`만 START/LEFT/RIGHT/CROSS/END/UNKNOWN으로 집계한다. `TRACK_PROCESS_CROSS_TAIL_MERGED`는 total에 넣지 않는다.
  - line loss episode/recovery, loss frame 수, turn/non-turn edge dwell 최대, 실제 적용 속도 샘플/최댓값을 O(1)로 누적한다.
  - END/manual/fault에서 공통 finalize를 사용한다. reason/fault 설정 → 모터 정지 전 snapshot/Track finalize/record 확정 → motor/sensor/timer stop 순서다.
  - map valid는 정상 END에서만 structural map 검사까지 통과한 경우에만 1이다.
- `Main/Src/menu.c`
  - 정지/고장 화면을 final record 기반의 Summary/Markers/Debug 3×3 페이지로 교체했다.
  - L/R은 현재 그룹의 페이지 순환, C 짧게는 다음 그룹의 1페이지, C 길게는 메인 복귀다.
  - END/manual은 Summary 1페이지, fault는 Debug 1페이지로 진입한다.

`Main/Src/second_drive.c`는 수정하지 않았다. 따라서 V27 planner/SEEK resync와 V28 replay-tail 차단 경로는 유지된다.

## 3. 실행 기록 semantics

### Marker summary

`total_count = START + LEFT + RIGHT + CROSS + END + UNKNOWN`을 동일한 publish 지점에서 함께 증가시켜 유지한다. START는 `BOTH`이며 center step이 300 미만인 event, END는 기존 V28의 bilateral overlap/no-wide-center/END guard 조건을 모두 통과한 event다. tail fragment는 공개 marker가 아니므로 total에 포함하지 않는다.

summary에는 segment/anchor 수, tail fragment 수, 영향을 받은 CROSS 수, 최대 gap, END guard reject 수, Track/anchor overflow, map valid를 함께 남긴다.

### Quality/speed summary

- loss episode는 정상 line → invalid line 전이에서 시작하고, 기존 5 valid-frame recovery가 완료될 때만 recovery success로 끝난다. 재획득 중 invalid blip은 같은 episode로 유지한다. fault 종료는 success를 증가시키지 않는다.
- edge dwell 최대는 confirmed TURN과 normal/transition으로 분리한다.
- `Motor_DriveStart()` 성공 직후 1회, 이후 `Motor_DriveSetSpeeds()` 성공 직후 1회씩 실제 적용 center/left/right/target speed를 샘플링한다. 실패한 motor command는 샘플에 넣지 않는다.
- record에는 sum/count/max를 저장하고 center 평균은 `FirstDrive_GetRunRecord()`의 메뉴 읽기 경로에서 계산한다.

### Stop snapshot

snapshot은 step/time/state/phase, 현재·마지막 valid mask와 position, PD/steer/limit, target/current center 및 wheel SPS, loss/edge/bridge/boost, 마지막 marker의 type/confidence/union/entry/exit mask/center/run evidence, IRQ/control tick을 포함한다. 이후 live telemetry가 속도 0으로 정리되어도 final record는 바뀌지 않는다.

## 4. 결과 UI

| 그룹 | 페이지 | 표시 내용 |
| --- | --- | --- |
| Summary | 1 | 종료 이유, 시간/step, total/CROSS/END, segment/anchor/map |
| Summary | 2 | loss/recovery/lost frame, edge dwell, tail/reject, unknown/overflow |
| Summary | 3 | center average/max, target/wheel max, BASE/TURN/CROSS/REC speed 상수 |
| Markers | 1 | START/LEFT/RIGHT/CROSS/END/UNKNOWN, segment/anchor/total |
| Markers | 2 | CROSS/anchor, tail event/affected CROSS/max gap, reject/map/overflow |
| Markers | 3 | 최신 순 marker 4개를 사람이 읽는 분류명으로 표시 |
| Debug | 1 | stop reason/fault, final state/phase/position/mask/map |
| Debug | 2 | P/D/steer, steering limit, target/current/wheel speed, loss/edge/bridge/boost |
| Debug | 3 | 마지막 marker raw evidence, tail/reject, IRQ/control/lost-frame count |

## 5. 검증 결과

### Host Track harness

```text
V28 track harness PASS
V29 track harness PASS
```

- 기존 V28 harness: normal/split CROSS, 복수 tail, UNKNOWN tail, window 밖 BOTH, 완전 CROSS 뒤 END, 새 wide CROSS, First/Replay 대칭, map/anchor finalize를 회귀 검증했다.
- V29 harness: entry/exit mask 보존, 두 tail fragment가 event를 추가하지 않는 점, tail fragment 2개가 affected CROSS 1개로 집계되는 점, 최대 gap 보존을 검증했다.

### ARM clean build

```sh
cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --clean-first -j 8
```

결과: 성공.

```text
text  122,188 B
data      416 B
bss    43,872 B
RAM 합계 44,288 B / 272 KB = 15.90%
Flash 출력 122,604 B / 512 KB = 23.38%
```

source 범위 `git diff --check`도 통과했다. clean build에는 기존 `Drivers/BSP/ST7735/st7735_lcd.c:50`의 unused local variable 경고 1개가 남으며, V29 변경 코드에서 발생한 새 경고는 없다.

## 6. 상위 모델/실차 검증에 전달할 사항

1. 정상 END, countdown 중 수동 정지, 주행 중 수동 정지, line-lost/motor/timer/overflow fault 각각에서 reason/fault와 snapshot speed가 모터 정지 후에도 유지되는지 확인한다.
2. 실제 종료 화면에서 9개 페이지가 모두 135px LCD에 읽기 좋게 표시되는지, L/R/C short/C hold 입력이 계획대로 동작하는지 확인한다.
3. 여러 손실/reacquire를 포함한 실차 run에서 `loss episode`와 `recovery success`가 5-frame 규칙과 맞는지 확인한다. 재획득 중 한 frame invalid은 새 episode가 아니어야 한다.
4. 긴 180/270도 코너에서 TURN edge dwell maximum은 증가하되 EDGE_STUCK fault 정책이 V28과 동일하게 유지되는지 확인한다.
5. split CROSS 코스에서 tail fragment 수와 affected CROSS 수를 함께 기록해, 한 CROSS에 여러 tail이 붙는 실제 빈도를 수집한다. `tail_max_gap_steps`가 160 근처면 V28 window 재조정 여부를 상위 모델이 판단한다.
6. map valid가 정상 END에서만 1이고, manual/fault 후에는 Second Drive를 시작할 수 없는지 확인한다.

## 7. 남은 제한

실차 센서·모터·LCD 입력 시험은 이 환경에서 할 수 없었다. 따라서 snapshot의 실제 정지 직전 값과 화면 가독성은 위 실차 항목으로 최종 판정해야 한다. Host test는 Track collector를 직접 검증했고, Drive의 HAL/ISR 정지 경로는 ARM clean build와 코드 경로 점검으로 검증했다.
