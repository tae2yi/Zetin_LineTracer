# V27 교차로 앵커 재동기화 구현 결과 및 상위 모델 전달사항

## 1. 작업 개요

- 기준 계획: `codex_work_plan/V27_CROSS_ANCHOR_RESYNC_IMPLEMENTATION_PLAN.md`
- 작업 버전: V27
- 작업일: 2026-08-10
- 프로젝트: `/Users/ehoi/STM32CubeIDE/line_tracer_2026`
- 펌웨어 버전: `Main/Inc/app_version.h`의 `APP_VERSION_NUMBER 27U`

V26의 First Drive 라인 추종, CROSS/END 분류, 라인 손실 복구, 안전정지 경로는 유지하고 Second Drive의 맵 동기화와 속도 계획을 중심으로 변경했다.

## 2. 구현 완료 내용

### Track 맵 메타데이터와 CROSS 앵커

- `TrackSegment_t`에 `curve_units`를 추가했다.
  - 0: 곡선 아님/판정 불확실
  - 1-6: 45도 단위의 25cm 반경 원호 메타데이터
  - 45도 1단위를 약 500 generated step으로 계산
  - 허용오차를 벗어나도 맵을 무효화하지 않고 진단용 값만 0으로 저장
- 최대 64개의 `TrackCrossAnchor_t`를 추가했다.
  - CROSS event index
  - CROSS 직후 segment index
  - 절대 center step
  - 이전 CROSS와의 거리
- `Track_FinalizeSegments()`에서 segment와 anchor를 함께 확정한다.
- `Track_Reset()`은 event/segment/anchor를 모두 초기화하고, `Track_ReplayReset()`은 First Drive 맵을 보존한다.
- anchor overflow와 anchor 구조 범위를 맵 유효성 검사에 포함했다.

### Second Drive 동기화 상태 머신

- 기존 영구 `fallback_active`를 제거하고 다음 상태로 교체했다.
  - `SECOND_DRIVE_SYNC_MAP`
  - `SECOND_DRIVE_SYNC_SEEK_CROSS`
  - `SECOND_DRIVE_SYNC_INVALID`
- mismatch reason을 추가했다.
  - event type/distance
  - segment overdue
  - map bounds
  - anchor not found/ambiguous
- 검증에 실패한 일반 event에서는 `expected_event_index`와 `segment_index`를 전진시키지 않는다.
- mismatch 또는 segment overdue는 MAP에서 SEEK로 한 번만 전이하고, SEEK 중에는 First Drive target speed envelope를 사용한다.
- SEEK 중 일반 방향 마커는 맵 위치를 바꾸지 않고 무시한다.
- 실제 CROSS가 검출되면 앞으로 최대 3개의 anchor를 거리로 비교한다.
  - 허용오차: `max(recorded / 3, 500 steps)`
  - 후보가 모호하면 snap하지 않고 SEEK를 유지한다.
  - 성공 시 event/segment index를 해당 CROSS 이후 위치로 snap하고 `resync_count`를 증가시킨다.
  - anchor snap은 뒤로 rewind하지 않는다.
- CROSS가 정상 순서/거리로 매칭된 경우에도 confirmed anchor를 갱신한다.

### Second Drive 속도 계획

- SYNC 상태에서 `STRAIGHT`와 `CROSS`를 동일한 fast geometry로 취급한다.
- mapped CROSS는 다음 코너가 충분히 멀 때 Second Drive 직선 설정 속도를 사용할 수 있다.
- First Drive와 SEEK 상태의 기존 CROSS 2400 SPS cap은 유지된다.
- 현재 segment 하나만 보는 방식에서 최대 16개 fast segment를 통과하는 다중 lookahead로 변경했다.
- 다음 `LEFT`, `RIGHT`, `END`까지 누적 거리를 계산해 제동거리가 부족하면 CROSS 전부터 감속한다.
- 64-bit 중간값으로 제동거리 계산을 유지하고, 동적 할당/float/전체 맵 매 1ms 스캔은 추가하지 않았다.

### UI와 버전

- Second Drive 화면에 다음 정보를 표시하도록 변경했다.
  - `MAP SYNC`, `SEEK CROSS`, `MAP INVALID`
  - segment type/index, curve unit, remaining step
  - anchor order/count, expected event index
  - next restriction distance
  - mismatch count, resync count, mismatch reason
- `APP_VERSION_NUMBER`를 26에서 27로 올렸다.

## 3. 변경 파일

- `Main/Inc/app_version.h`
- `Main/Inc/track.h`
- `Main/Src/track.c`
- `Main/Inc/second_drive.h`
- `Main/Src/second_drive.c`
- `Main/Src/menu.c`
- `codex_worked_review/V27_CROSS_ANCHOR_RESYNC_WORK_RESULT.md`

다음 파일은 변경하지 않았다.

- `Main/Src/drive.c`
- `sensor.c/h`, `motor.c/h`, `marker.c/h`
- CubeMX 생성 파일, `.ioc`, linker script
- `.DS_Store`와 사용자 백업 디렉토리

따라서 drive.c의 marker 호출 순서, CROSS pass 510 step, 중앙 광폭 CROSS 조향 억제, First Drive 안전 로직은 기존 경로 그대로다.

## 4. 검증 결과

### Debug clean build

기존 `build/Debug` cache가 이전 프로젝트 경로를 가리키고 있었고 현재 환경에 Ninja가 없어 다음과 같이 현재 프로젝트를 Makefiles로 재구성해 빌드했다.

```sh
cmake --fresh -S . -B build/Debug -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --clean-first -j 8
```

결과: 성공

```text
text: 113,520 B
data:     416 B
bss:   41,160 B
RAM 합계: 41,576 B / 272 KB = 14.93%
Flash 출력: 113,936 B / 512 KB = 21.73%
```

추가된 정적 영역은 대략 다음과 같다.

- `segments`: 0x1800 = 6,144 B
- `cross_anchors`: 0x0400 = 1,024 B
- planner 상태/anchor runtime: 수십 B

빌드 경고는 계획에 기록된 기존 경고 하나뿐이다.

```text
Drivers/BSP/ST7735/st7735_lcd.c:50: unused variable 'text'
```

이번 V27 코드에서 새 warning은 확인되지 않았다.

### 정적/간이 검증

- 변경 소스에 대해 `git diff --check` 통과
- Main 소스에 `fallback_active` 잔여 참조 없음
- Track synthetic sensor sequence로 다음을 확인했다.
  - event 5개 수집
  - segment 5개 생성
  - CROSS anchor 1개 생성
  - 500 step 곡선이 `curve_units = 1`로 기록
  - overflow 없음

## 5. 상위 모델에게 전달할 내용

1. 구현과 Debug clean build는 끝났지만 이 환경에서는 실제 차체/트랙을 사용할 수 없어 실차 검증은 아직 하지 못했다.
2. 반드시 확인할 실차 항목:
   - 정상 Second Drive에서 끝까지 `MAP SYNC` 유지되는지
   - mapped CROSS에서 다음 코너가 먼 경우 2400 SPS로 불필요하게 제한되지 않는지
   - CROSS 직후 20cm 수준으로 코너가 가까운 경우 lookahead 감속이 충분한지
   - 방향 마커 하나 누락/false event 후 `SEEK CROSS -> MAP SYNC` 복귀하는지
   - false CROSS 또는 두 후보가 모호한 경우 snap하지 않는지
   - CROSS 없는 맵, CROSS/END 분류, line lost/watchdog/emergency stop 회귀 여부
3. 설계 확인이 필요한 값:
   - anchor 거리 허용오차 `max(recorded / 3, 500 steps)`
   - anchor 후보 동률 판단 차이 `100 steps`
   - segment lookahead 최대 16개
   실제 generated step 오차와 차체 슬립을 본 뒤 조정할 수 있다.
4. 현재 구현은 성공적인 CROSS 재동기화 직후 `last_mismatch_reason`를 `NONE`으로 지운다. UI에서 마지막 장애 원인을 계속 보존해야 한다면 별도 `last_resync_reason` 필드가 필요하다.
5. 예상 CROSS가 거리 불일치인 경우에도 현재는 CROSS 후보가 명확하면 즉시 resync하고, mismatch count는 증가시킨다. 이 정책이 원하는 진단 의미와 맞는지 확인이 필요하다.

## 6. 결론

V27 구현은 계획의 코드 완료 조건과 Debug clean build 조건을 충족했다. 실제 완료 판정은 위 실차 시험 매트릭스를 실행한 뒤 내려야 한다.
