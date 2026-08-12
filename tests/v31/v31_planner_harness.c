#include "second_drive.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int failures;

void __disable_irq(void) {}
void __enable_irq(void) {}

FirstDriveState_t SecondDrive_GetState(void)
{
	return FIRST_DRIVE_READY;
}

static void Check(bool condition, const char *message)
{
	if (!condition) {
		printf("FAIL: %s\n", message);
		failures++;
	}
}

static void FeedCandidate(uint8_t mask, uint32_t *frame,
		uint32_t start_step)
{
	uint8_t index;

	for (index = 0U; index < TRACK_MARK_CONFIRM_FRAMES; index++) {
		(*frame)++;
		(void)Track_ProcessSensor(mask, *frame,
				start_step + ((uint32_t)index * 10U));
	}
	for (index = 1U; index <= TRACK_MARK_CLEAR_FRAMES; index++) {
		(*frame)++;
		(void)Track_ProcessSensor(0U, *frame,
				start_step + 30U + ((uint32_t)index * 10U));
	}
}

static void BuildMapAtSteps(const uint8_t *masks, const uint32_t *starts,
		uint8_t count)
{
	uint32_t frame = 0U;
	uint8_t index;

	Track_Reset();
	Track_SetStartIgnoreSteps(300U);
	for (index = 0U; index < count; index++) {
		FeedCandidate(masks[index], &frame, starts[index]);
	}
	Track_FinalizeSegments();
}

static void TestEndCorridorAndFallback(void)
{
	static const uint8_t normal_masks[] = { 0x80U, 0x80U, 0x81U };
	static const uint32_t normal_starts[] = { 500U, 1500U, 6000U };
	static const uint8_t pair_masks[] = { 0x80U, 0x81U };
	static const uint32_t pair_starts[] = { 500U, 6000U };
	SecondDrivePlannerStatus_t status;
	SecondDriveRunStats_t stats;
	const TrackMarkerEvent_t *normal_end_event;
	uint16_t target;
	uint16_t far_target;
	uint16_t near_target;
	uint16_t sticky_target;
	uint32_t far_distance;
	uint32_t near_distance;
	uint8_t index;

	BuildMapAtSteps(normal_masks, normal_starts,
			(uint8_t)(sizeof(normal_masks) / sizeof(normal_masks[0])));
	SecondDrivePlanner_Reset();
	SecondDrivePlanner_OnEvent(Track_GetEvent(0U));
	SecondDrivePlanner_OnEvent(Track_GetEvent(1U));
	SecondDrivePlanner_BeginRun();
	for (index = 0U; index < SECOND_DRIVE_FAST_STABLE_FRAMES; index++) {
		target = SecondDrivePlanner_GetTargetSps(3820U, 0,
				FIRST_DRIVE_COURSE_STRAIGHT, 1600U, 5000U, true,
				false, false);
		SecondDrivePlanner_RecordFinalTarget(target, target, 0, 1600U,
				FIRST_DRIVE_COURSE_STRAIGHT, false);
	}
	SecondDrivePlanner_GetStatus(&status);
	Check(status.final_end_expected == 1U,
			"normal map expects final END");
	Check(status.final_end_corridor == 1U,
			"closed pair opens final performance corridor");
	Check(status.end_fallback_active == 0U,
			"normal corridor has no END fallback");
	Check(target == SecondDrive_GetEffectiveStraightSps(),
			"normal corridor keeps straight performance target");
	normal_end_event = Track_GetEvent(2U);
	Check(normal_end_event != NULL, "normal fixture has final END event");
	SecondDrivePlanner_OnEvent(normal_end_event);
	SecondDrivePlanner_RecordEndBrake(normal_end_event->center_step + 12U,
			SecondDrive_GetEffectiveStraightSps());
	SecondDrivePlanner_GetRunStats(&stats);
	Check(stats.expected_end_step == normal_end_event->center_step
			&& stats.end_step_error == 12,
			"confirmed END stores event step after planner index advance");
	SecondDrivePlanner_FinalizeRunStats(100U);

	BuildMapAtSteps(pair_masks, pair_starts,
			(uint8_t)(sizeof(pair_masks) / sizeof(pair_masks[0])));
	SecondDrivePlanner_Reset();
	SecondDrivePlanner_OnEvent(Track_GetEvent(0U));
	SecondDrivePlanner_BeginRun();
	SecondDrivePlanner_GetStatus(&status);
	Check(status.replay_turn_open == 1U,
			"pair-open fixture keeps replay pair open");
	target = SecondDrivePlanner_GetTargetSps(3820U, 0,
			FIRST_DRIVE_COURSE_STRAIGHT, status.segment_start_step + 100U,
			5000U, true, false, false);
	SecondDrivePlanner_RecordFinalTarget(target, target, 0,
			status.segment_start_step + 100U,
			FIRST_DRIVE_COURSE_STRAIGHT, false);
	SecondDrivePlanner_GetStatus(&status);
	far_target = target;
	far_distance = status.end_distance_steps;
	Check(status.final_end_corridor == 0U,
			"pair-open blocks final performance corridor");
	Check(target != SecondDrive_GetEffectiveStraightSps(),
			"pair-open uses safe nominal target before END window");

	/* Reaching the same END with a pair still open must apply the 1800 cap. */
	target = SecondDrivePlanner_GetTargetSps(3820U, 0,
			FIRST_DRIVE_COURSE_STRAIGHT, status.segment_start_step + 4300U,
			5000U, true, false, false);
	SecondDrivePlanner_RecordFinalTarget(target, target, 0,
			status.segment_start_step + 4300U,
			FIRST_DRIVE_COURSE_STRAIGHT, false);
	SecondDrivePlanner_GetStatus(&status);
	near_target = target;
	near_distance = status.end_distance_steps;
	Check(target == SECOND_DRIVE_END_APPROACH_SPS,
			"pair-open END braking window caps target at 1800");
	Check(status.limit_reason == SECOND_DRIVE_LIMIT_END_APPROACH_SAFE,
			"pair-open END braking reason is END_SAFE");
	Check(status.end_fallback_active == 1U,
			"END fallback becomes sticky");

	/* Centering, position gate and phase changes must not release a sticky cap. */
	target = SecondDrivePlanner_GetTargetSps(3820U, 0,
			FIRST_DRIVE_COURSE_STRAIGHT, status.segment_start_step + 4400U,
			5000U, true, false, false);
	sticky_target = target;
	Check(target == SECOND_DRIVE_END_APPROACH_SPS,
			"sticky END fallback blocks return to fast after centering");
	SecondDrivePlanner_GetStatus(&status);
	Check(status.limit_reason == SECOND_DRIVE_LIMIT_END_APPROACH_SAFE,
			"sticky END fallback keeps END_SAFE diagnostic reason");
	target = SecondDrivePlanner_GetTargetSps(3820U, 1000,
			FIRST_DRIVE_COURSE_TURN_LEFT, status.segment_start_step + 4450U,
			5000U, true, false, false);
	Check(target == SECOND_DRIVE_END_APPROACH_SPS,
			"position/phase gate cannot bypass END fallback");

	SecondDrivePlanner_RecordEndBrake(status.segment_start_step + 4500U, 5000U);
	SecondDrivePlanner_RecordStopMode(SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE,
			true, true);
	SecondDrivePlanner_RecordBrakeCompletion(150U, true);
	SecondDrivePlanner_FinalizeRunStats(250U);
	SecondDrivePlanner_GetRunStats(&stats);
	Check(stats.end_fallback_active == 1U,
			"run stats preserve END fallback active state");
	Check(stats.end_fallback_entry_step != 0U,
			"run stats preserve END fallback entry step");
	Check(stats.stop_mode == SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE,
			"confirmed END records active brake stop mode");
	printf("V31 END fallback fixture: far_distance=%lu far_target=%u "
		"near_distance=%lu near_target=%u sticky_target=%u policy=%u\n",
		(unsigned long)far_distance, (unsigned int)far_target,
		(unsigned long)near_distance, (unsigned int)near_target,
		(unsigned int)sticky_target, (unsigned int)stats.end_policy);
	printf("V31 corridor-false confirmed END: stop_mode=%u attempted=%u "
		"succeeded=%u hold_ms=%u\n",
		(unsigned int)stats.stop_mode,
		(unsigned int)stats.brake_start_attempted,
		(unsigned int)stats.brake_start_succeeded,
		(unsigned int)stats.brake_hold_ms);
}

static void TestCandidateAndLimiterStats(void)
{
	SecondDriveRunStats_t stats;
	uint16_t index;
	uint16_t target;
	uint32_t limiter_total = 0U;

	BuildMapAtSteps((const uint8_t[]){ 0x81U },
			(const uint32_t[]){ 10000U }, 1U);
	SecondDrivePlanner_Reset();
	SecondDrivePlanner_BeginRun();
	SecondDrivePlanner_RecordMarkerCandidateEpisode(false,
			(uint8_t)((1U << SECOND_DRIVE_MARKER_REJECT_OFF_CENTER)
				| (1U << SECOND_DRIVE_MARKER_REJECT_NO_LINE)), 10U);
	SecondDrivePlanner_RecordMarkerCandidateEpisode(true, 0U, 20U);
	SecondDrivePlanner_RecordMarkerCandidateEpisode(false,
			(uint8_t)(1U << SECOND_DRIVE_MARKER_REJECT_COOLDOWN_OR_DUPLICATE),
			30U);
	SecondDrivePlanner_GetRunStats(&stats);
	Check(stats.marker_candidate_episode_count == 3U,
			"candidate API counts physical episodes");
	Check(stats.marker_candidate_accepted_count == 1U
			&& stats.marker_candidate_rejected_count == 2U,
			"candidate accepted/rejected counts");
	Check(stats.marker_reject_off_center_count == 1U
			&& stats.marker_reject_no_line_count == 1U
			&& stats.marker_reject_duplicate_count == 1U,
			"candidate reason mask counts each reason once");
	SecondDrivePlanner_FinalizeRunStats(50U);

	/* A10 -> B20 -> A30 verifies episode boundaries and final flush. */
	SecondDrivePlanner_Reset();
	SecondDrivePlanner_BeginRun();
	for (index = 0U; index < 10U; index++) {
		target = SecondDrivePlanner_GetTargetSps(3820U, 1000,
				FIRST_DRIVE_COURSE_STRAIGHT, 100U + index,
				5000U, true, false, false);
		SecondDrivePlanner_RecordFinalTarget(target, target, 1000,
				100U + index, FIRST_DRIVE_COURSE_STRAIGHT, false);
	}
	for (index = 0U; index < 20U; index++) {
		target = SecondDrivePlanner_GetTargetSps(3820U, 0,
				FIRST_DRIVE_COURSE_STRAIGHT, 200U + index,
				5000U, false, true, false);
		SecondDrivePlanner_RecordFinalTarget(target, target, 0,
				200U + index, FIRST_DRIVE_COURSE_STRAIGHT, true);
	}
	for (index = 0U; index < 30U; index++) {
		target = SecondDrivePlanner_GetTargetSps(3820U, 1000,
				FIRST_DRIVE_COURSE_STRAIGHT, 300U + index,
				5000U, true, false, false);
		SecondDrivePlanner_RecordFinalTarget(target, target, 1000,
				300U + index, FIRST_DRIVE_COURSE_STRAIGHT, false);
	}
	SecondDrivePlanner_FinalizeRunStats(400U);
	SecondDrivePlanner_GetRunStats(&stats);
	Check(stats.limiter_episode_count[SECOND_DRIVE_LIMIT_POSITION] == 2U,
			"position limiter has two episodes");
	Check(stats.limiter_episode_count[SECOND_DRIVE_LIMIT_RECOVERY] == 1U,
			"recovery limiter has one episode");
	Check(stats.limiter_max_consecutive_samples[SECOND_DRIVE_LIMIT_POSITION]
			== 30U, "position max streak is 30");
	Check(stats.limiter_max_consecutive_samples[SECOND_DRIVE_LIMIT_RECOVERY]
			== 20U, "recovery max streak is 20");
	for (index = 0U; index < SECOND_DRIVE_LIMIT_COUNT; index++) {
		limiter_total += stats.limiter_samples[index];
	}
	Check(limiter_total == stats.control_samples,
			"limiter histogram has exactly one sample per control tick");
}

static void TestStopModeRecords(void)
{
	static const SecondDriveStopMode_t modes[] = {
		SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE,
		SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE_FAILED_FULL_OFF,
		SECOND_DRIVE_STOP_MODE_EMERGENCY_FULL_OFF,
		SECOND_DRIVE_STOP_MODE_FAULT_FULL_OFF
	};
	SecondDriveRunStats_t stats;
	uint8_t index;

	for (index = 0U; index < (uint8_t)(sizeof(modes) / sizeof(modes[0]));
			index++) {
		BuildMapAtSteps((const uint8_t[]){ 0x81U },
				(const uint32_t[]){ 10000U }, 1U);
		SecondDrivePlanner_Reset();
		SecondDrivePlanner_BeginRun();
		SecondDrivePlanner_RecordStopMode(modes[index],
				modes[index] == SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE
						|| modes[index]
							== SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE_FAILED_FULL_OFF,
				modes[index] == SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE);
		SecondDrivePlanner_FinalizeRunStats(1U);
		SecondDrivePlanner_GetRunStats(&stats);
		Check(stats.stop_mode == modes[index],
				"stop mode is retained in run stats");
		Check(stats.brake_start_succeeded
				== ((modes[index] == SECOND_DRIVE_STOP_MODE_ACTIVE_BRAKE)
						? 1U : 0U),
				"brake success flag matches stop mode");
	}
}

static void TestResyncAndNegativeRepair(void)
{
	static const uint8_t cross_masks[] = { 0x80U, 0x3CU, 0x81U };
	static const uint32_t cross_starts[] = { 500U, 2000U, 5000U };
	SecondDrivePlannerStatus_t status;
	TrackMarkerEvent_t wrong;

	BuildMapAtSteps(cross_masks, cross_starts,
			(uint8_t)(sizeof(cross_masks) / sizeof(cross_masks[0])));
	SecondDrivePlanner_Reset();
	memset(&wrong, 0, sizeof(wrong));
	wrong.type = MARKER_EVENT_EDGE_0;
	wrong.confidence = 60U;
	wrong.center_step = 500U;
	wrong.entry_step = 500U;
	wrong.exit_step = 520U;
	wrong.edge_union = 0x01U;
	SecondDrivePlanner_OnEvent(&wrong);
	SecondDrivePlanner_GetStatus(&status);
	Check(status.sync_state == SECOND_DRIVE_SYNC_SEEK_CROSS,
			"mismatch enters SEEK CROSS");
	SecondDrivePlanner_OnEvent(&wrong);
	SecondDrivePlanner_GetStatus(&status);
	Check(status.local_repair_reject_reason
			== SECOND_DRIVE_LOCAL_REPAIR_MAP_NOT_SYNC,
			"SEEK state rejects non-CROSS local repair");
	SecondDrivePlanner_OnEvent(Track_GetEvent(1U));
	SecondDrivePlanner_GetStatus(&status);
	Check(status.sync_state == SECOND_DRIVE_SYNC_MAP,
			"current-source CROSS resync returns to MAP");
	Check(status.resync_count == 1U && status.current_anchor_order == 0U,
			"resync restores anchor order");
	Check(status.expected_event_index == 2U,
			"resync restores expected event index");
	Check(status.replay_turn_open == 0U,
			"resync resets replay pair");
	printf("V31 resync fixture: sync=%u expected=%u anchor=%u pair=%u\n",
		(unsigned int)status.sync_state,
		(unsigned int)status.expected_event_index,
		(unsigned int)status.current_anchor_order,
		(unsigned int)status.replay_turn_open);

	/* No-open negative. */
	BuildMapAtSteps((const uint8_t[]){ 0x80U, 0x81U },
			(const uint32_t[]){ 500U, 5000U }, 2U);
	SecondDrivePlanner_Reset();
	memset(&wrong, 0, sizeof(wrong));
	wrong.type = MARKER_EVENT_EDGE_0;
	wrong.confidence = 60U;
	wrong.center_step = 500U;
	SecondDrivePlanner_OnEvent(&wrong);
	SecondDrivePlanner_GetStatus(&status);
	Check(status.local_repair_reject_reason
			== SECOND_DRIVE_LOCAL_REPAIR_NO_OPEN,
			"local repair rejects without open pair");

	/* Wrong-side negative: an open RIGHT pair must not be closed by LEFT. */
	BuildMapAtSteps((const uint8_t[]){ 0x80U, 0x81U },
			(const uint32_t[]){ 500U, 5000U }, 2U);
	SecondDrivePlanner_Reset();
	SecondDrivePlanner_OnEvent(Track_GetEvent(0U));
	SecondDrivePlanner_GetStatus(&status);
	memset(&wrong, 0, sizeof(wrong));
	wrong.type = MARKER_EVENT_EDGE_0;
	wrong.confidence = 60U;
	wrong.center_step = status.replay_turn_open_step + 400U;
	SecondDrivePlanner_OnEvent(&wrong);
	SecondDrivePlanner_GetStatus(&status);
	Check(status.local_repair_reject_reason
			== SECOND_DRIVE_LOCAL_REPAIR_WRONG_SIDE,
			"local repair rejects wrong-side direction");

	/* Expected-not-boundary negative: same-side event while ordinary edge is
	 * expected must not close the replay pair. */
	BuildMapAtSteps((const uint8_t[]){ 0x80U, 0x01U, 0x80U, 0x81U },
			(const uint32_t[]){ 500U, 1000U, 1500U, 5000U }, 4U);
	SecondDrivePlanner_Reset();
	SecondDrivePlanner_OnEvent(Track_GetEvent(0U));
	memset(&wrong, 0, sizeof(wrong));
	wrong.type = MARKER_EVENT_EDGE_7;
	wrong.confidence = 60U;
	wrong.center_step = 900U;
	SecondDrivePlanner_OnEvent(&wrong);
	SecondDrivePlanner_GetStatus(&status);
	Check(status.local_repair_reject_reason
			== SECOND_DRIVE_LOCAL_REPAIR_EXPECTED_NOT_BOUNDARY,
			"local repair rejects ordinary expected edge");

	/* Low-confidence and duplicate/cooldown negatives use the final END
	 * boundary, where a same-side local close would otherwise be legal. */
	BuildMapAtSteps((const uint8_t[]){ 0x80U, 0x81U },
			(const uint32_t[]){ 500U, 5000U }, 2U);
	SecondDrivePlanner_Reset();
	SecondDrivePlanner_OnEvent(Track_GetEvent(0U));
	SecondDrivePlanner_GetStatus(&status);
	memset(&wrong, 0, sizeof(wrong));
	wrong.type = MARKER_EVENT_EDGE_7;
	wrong.confidence = 10U;
	wrong.center_step = status.replay_turn_open_step + 400U;
	SecondDrivePlanner_OnEvent(&wrong);
	SecondDrivePlanner_GetStatus(&status);
	Check(status.local_repair_reject_reason
			== SECOND_DRIVE_LOCAL_REPAIR_LOW_CONFIDENCE,
			"local repair rejects low confidence");

	BuildMapAtSteps((const uint8_t[]){ 0x80U, 0x81U },
			(const uint32_t[]){ 500U, 5000U }, 2U);
	SecondDrivePlanner_Reset();
	SecondDrivePlanner_OnEvent(Track_GetEvent(0U));
	SecondDrivePlanner_GetStatus(&status);
	memset(&wrong, 0, sizeof(wrong));
	wrong.type = MARKER_EVENT_EDGE_7;
	wrong.confidence = 60U;
	wrong.center_step = status.replay_turn_open_step
			+ TRACK_MARK_COOLDOWN_STEPS;
	SecondDrivePlanner_OnEvent(&wrong);
	SecondDrivePlanner_GetStatus(&status);
	Check(status.local_repair_reject_reason
			== SECOND_DRIVE_LOCAL_REPAIR_DUPLICATE,
			"local repair rejects duplicate cooldown event");

	/* Invalid and SEEK inputs are explicitly reported as map-not-synchronised
	 * local-repair rejects by the V31 diagnostic path. */
	Track_Reset();
	SecondDrivePlanner_Reset();
	memset(&wrong, 0, sizeof(wrong));
	wrong.type = MARKER_EVENT_EDGE_0;
	wrong.confidence = 60U;
	SecondDrivePlanner_OnEvent(&wrong);
	SecondDrivePlanner_GetStatus(&status);
	Check(status.local_repair_reject_reason
			== SECOND_DRIVE_LOCAL_REPAIR_MAP_NOT_SYNC,
			"map-invalid local repair rejection is explicit");
}

int main(void)
{
	TestEndCorridorAndFallback();
	TestCandidateAndLimiterStats();
	TestStopModeRecords();
	TestResyncAndNegativeRepair();
	if (failures != 0U) {
		printf("%u failures\n", failures);
		return EXIT_FAILURE;
	}
	puts("V31 planner/diagnostic harness PASS");
	return EXIT_SUCCESS;
}
