/*
 * second_drive.c
 *
 * Uses the finalized First Drive segment map as a speed-planning aid.  Motor
 * step distance predicts braking, but only a re-detected physical marker moves
 * the segment index.  If a marker no longer matches the map, map-based speed
 * planning pauses until a forward CROSS anchor can re-establish position.
 */

#include "second_drive.h"

#include "main.h"
#include "marker.h"
#include "motor.h"

#include <stddef.h>
#include <string.h>

#define SECOND_DRIVE_TURN_REFERENCE_SPS          2200U
#define SECOND_DRIVE_DECEL_SPS_PER_SECOND       10000U
#define SECOND_DRIVE_BRAKE_MARGIN_STEPS           300U
#define SECOND_DRIVE_ACCEL_ENABLE_MARGIN_STEPS    150U
#define SECOND_DRIVE_START_MARKER_IGNORE_STEPS    300U
#define SECOND_DRIVE_DISTANCE_TOLERANCE_MIN        200U
#define SECOND_DRIVE_ANCHOR_LOOKAHEAD_COUNT          3U
#define SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_MIN 500U
#define SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_DIV   3U
#define SECOND_DRIVE_ANCHOR_TIE_TOLERANCE_STEPS      100U
#define SECOND_DRIVE_LOOKAHEAD_MAX_SEGMENTS          16U

static SecondDriveConfig_t second_drive_config = {
	.straight_sps = SECOND_DRIVE_DEFAULT_STRAIGHT_SPS,
	.curve_sps = SECOND_DRIVE_DEFAULT_CURVE_SPS,
	.overall_percent = SECOND_DRIVE_DEFAULT_OVERALL_PERCENT
};
static volatile uint16_t second_drive_effective_straight_sps =
		SECOND_DRIVE_DEFAULT_STRAIGHT_SPS;
static volatile uint16_t second_drive_effective_curve_sps =
		SECOND_DRIVE_DEFAULT_CURVE_SPS;
static volatile uint16_t second_drive_effective_approach_sps =
		SECOND_DRIVE_DEFAULT_CURVE_SPS + SECOND_DRIVE_APPROACH_BONUS_SPS;
static volatile uint16_t second_drive_effective_exit_sps =
		SECOND_DRIVE_DEFAULT_CURVE_SPS + SECOND_DRIVE_EXIT_BONUS_SPS;

static volatile SecondDrivePlannerStatus_t planner_status;
static volatile uint32_t planner_segment_start_step;
static volatile uint16_t planner_expected_event_index;
static volatile uint32_t planner_last_anchor_map_step;
static volatile uint32_t planner_last_anchor_run_step;
static volatile bool planner_have_anchor;
static volatile bool planner_replay_turn_open;
static volatile int8_t planner_replay_turn_direction;
static volatile uint32_t planner_replay_turn_open_step;
static volatile int8_t planner_last_direction;
static volatile uint32_t planner_last_direction_step;
static volatile uint32_t planner_last_repair_step;
static volatile int8_t planner_last_repair_direction;
static volatile bool planner_fast_gate_ready;
static volatile uint16_t planner_fast_stable_frames;
static volatile uint16_t planner_final_exit_stable_frames;
static volatile bool planner_final_exit_override;
static volatile bool planner_cross_exit_active;
static volatile bool planner_end_fallback_active;
static volatile SecondDriveRunStats_t planner_run_stats;
static volatile bool planner_run_active;
static volatile SecondDriveLimitReason_t planner_last_trace_reason;
static volatile SecondDriveGeometrySource_t planner_last_trace_source;
static volatile bool planner_limiter_episode_valid;
static volatile SecondDriveLimitReason_t planner_limiter_episode_reason;
static volatile uint32_t planner_limiter_episode_samples;
static volatile uint32_t planner_limiter_episode_start_step;
static volatile uint32_t planner_limiter_episode_last_step;

static bool SecondDrive_StateAllowsConfiguration(FirstDriveState_t state)
{
	return (state == FIRST_DRIVE_OFF) || (state == FIRST_DRIVE_READY)
			|| (state == FIRST_DRIVE_STOPPED) || (state == FIRST_DRIVE_FAULT);
}

static uint16_t SecondDrive_ScalePerformanceSps(uint16_t value,
		bool *clamped)
{
	uint32_t scaled;

	if (value == second_drive_config.straight_sps) {
		if (clamped != NULL) {
			*clamped = ((uint32_t)second_drive_config.straight_sps
					* second_drive_config.overall_percent)
					> ((uint32_t)SECOND_DRIVE_PERFORMANCE_MAX_CENTER_SPS
							* 100U);
		}
		return second_drive_effective_straight_sps;
	}
	/* All other performance targets use their cached profile value.  Keeping
	 * this fallback division-free is important because GetTargetSps() runs from
	 * the 1 kHz control path. */
	scaled = value;

	if (scaled > SECOND_DRIVE_PERFORMANCE_MAX_CENTER_SPS) {
		if (clamped != NULL) {
			*clamped = true;
		}
		scaled = SECOND_DRIVE_PERFORMANCE_MAX_CENTER_SPS;
	} else if (clamped != NULL) {
		*clamped = false;
	}
	return (uint16_t)scaled;
}

static uint16_t SecondDrive_ScaleProfileValue(uint32_t nominal)
{
	uint32_t scaled = (nominal * second_drive_config.overall_percent) / 100U;

	return (scaled > SECOND_DRIVE_PERFORMANCE_MAX_CENTER_SPS)
			? SECOND_DRIVE_PERFORMANCE_MAX_CENTER_SPS : (uint16_t)scaled;
}

static void SecondDrive_RecomputeEffectiveProfile(void)
{
	second_drive_effective_straight_sps = SecondDrive_ScaleProfileValue(
			second_drive_config.straight_sps);
	second_drive_effective_curve_sps = SecondDrive_ScaleProfileValue(
			second_drive_config.curve_sps);
	second_drive_effective_approach_sps = SecondDrive_ScaleProfileValue(
			(uint32_t)second_drive_config.curve_sps
					+ SECOND_DRIVE_APPROACH_BONUS_SPS);
	second_drive_effective_exit_sps = SecondDrive_ScaleProfileValue(
			(uint32_t)second_drive_config.curve_sps
					+ SECOND_DRIVE_EXIT_BONUS_SPS);
}

static void SecondDrive_IncrementU8(volatile uint8_t *value)
{
	if ((*value) < UINT8_MAX) {
		(*value)++;
	}
}

static void SecondDrive_IncrementU16(volatile uint16_t *value)
{
	if ((*value) < UINT16_MAX) {
		(*value)++;
	}
}

static uint32_t SecondDrive_AbsoluteDifference(uint32_t first,
		uint32_t second)
{
	return (first > second) ? (first - second) : (second - first);
}

static uint32_t SecondDrive_DistanceTolerance(uint32_t recorded_distance)
{
	uint32_t tolerance = recorded_distance / 3U;

	return (tolerance < SECOND_DRIVE_DISTANCE_TOLERANCE_MIN)
			? SECOND_DRIVE_DISTANCE_TOLERANCE_MIN : tolerance;
}

static uint32_t SecondDrive_BrakingSteps(uint16_t high_sps,
		uint16_t low_sps)
{
	uint32_t numerator;
	const uint32_t denominator = 2U * SECOND_DRIVE_DECEL_SPS_PER_SECOND;

	if (high_sps <= low_sps) {
		return 0U;
	}
	numerator = ((uint32_t)high_sps * high_sps)
			- ((uint32_t)low_sps * low_sps);
	return (numerator / denominator)
			+ ((numerator % denominator) != 0U ? 1U : 0U);
}

static uint16_t SecondDrive_TargetForSegment(TrackSegmentType_t type)
{
	switch (type) {
	case TRACK_SEGMENT_LEFT:
	case TRACK_SEGMENT_RIGHT:
		/* This is the old safe fallback used only when geometry is uncertain. */
		return SECOND_DRIVE_TURN_REFERENCE_SPS;
	case TRACK_SEGMENT_END:
		return SECOND_DRIVE_END_APPROACH_SPS;
	case TRACK_SEGMENT_CROSS:
	case TRACK_SEGMENT_STRAIGHT:
	default:
		return SecondDrive_GetEffectiveStraightSps();
	}
}

static bool SecondDrive_IsFastGeometry(TrackSegmentType_t type)
{
	return (type == TRACK_SEGMENT_STRAIGHT)
			|| (type == TRACK_SEGMENT_CROSS);
}

static void SecondDrive_LoadSegmentStatus(uint16_t segment_index)
{
	const TrackSegment_t *segment = Track_GetSegment(segment_index);
	const TrackSegment_t *next_segment;

	if (segment == NULL) {
		planner_status.segment_index = segment_index;
		planner_status.segment_type = TRACK_SEGMENT_END;
		planner_status.next_segment_type = TRACK_SEGMENT_END;
		planner_status.curve_units = 0U;
		planner_status.segment_distance_steps = 0U;
		planner_status.segment_travelled_steps = 0U;
		planner_status.segment_remaining_steps = 0U;
		planner_status.next_restriction_distance_steps = 0U;
		return;
	}

	next_segment = Track_GetSegment(segment_index + 1U);
	planner_status.segment_index = segment_index;
	planner_status.segment_type = segment->type;
	planner_status.curve_units = segment->curve_units;
	planner_status.segment_distance_steps = segment->distance_steps;
	planner_status.segment_travelled_steps = 0U;
	planner_status.segment_remaining_steps = segment->distance_steps;
	planner_status.segment_start_step = planner_segment_start_step;
	planner_status.next_segment_type = (next_segment != NULL)
			? next_segment->type : TRACK_SEGMENT_END;
	planner_status.next_restriction_distance_steps = 0U;
}

static bool SecondDrive_MapIndexIsValid(uint16_t segment_index)
{
	return Track_GetSegment(segment_index) != NULL;
}

bool SecondDrivePlanner_MapIsStructurallyValid(void)
{
	uint16_t count = Track_GetSegmentCount();
	uint16_t anchor_index;
	const TrackSegment_t *last_segment;

	if (Track_HasOverflow() || Track_HasAnchorOverflow()
			|| (Track_GetEventCount() == 0U) || (count == 0U)) {
		return false;
	}
	last_segment = Track_GetSegment(count - 1U);
	if ((last_segment == NULL) || (last_segment->type != TRACK_SEGMENT_END)) {
		return false;
	}
	for (anchor_index = 0U;
			anchor_index < Track_GetCrossAnchorCount(); anchor_index++) {
		const TrackCrossAnchor_t *anchor = Track_GetCrossAnchor(anchor_index);
		const TrackMarkerEvent_t *event;

		if (anchor == NULL
				|| (anchor->order != anchor_index)
				|| (anchor->segment_index_after_cross >= count)) {
			return false;
		}
		event = Track_GetEvent(anchor->event_index);
		if ((event == NULL) || (event->type != MARKER_EVENT_CROSS)
				|| (Track_GetSegment(anchor->segment_index_after_cross) == NULL)) {
			return false;
		}
	}
	return true;
}

static void SecondDrive_SetExpectedEventIndex(uint16_t event_index)
{
	planner_expected_event_index = event_index;
	planner_status.expected_event_index = event_index;
}

static void SecondDrive_EnterSeekCross(SecondDriveMismatchReason_t reason)
{
	if (planner_status.sync_state == SECOND_DRIVE_SYNC_INVALID) {
		return;
	}
	if (planner_status.sync_state == SECOND_DRIVE_SYNC_MAP) {
		SecondDrive_IncrementU8(&planner_status.mismatch_count);
		if (planner_run_active) {
			SecondDrive_IncrementU16(&planner_run_stats.mismatch_count);
		}
		planner_status.sync_state = SECOND_DRIVE_SYNC_SEEK_CROSS;
	}
	planner_status.last_mismatch_reason = reason;
}

static bool SecondDrive_EventDirection(const TrackMarkerEvent_t *event,
		int8_t *direction)
{
	if ((event == NULL) || (direction == NULL)) {
		return false;
	}
	if (event->type == MARKER_EVENT_EDGE_0) {
		*direction = -1;
		return true;
	}
	if (event->type == MARKER_EVENT_EDGE_7) {
		*direction = 1;
		return true;
	}
	return false;
}

static void SecondDrive_PublishPairState(void)
{
	planner_status.replay_turn_open = planner_replay_turn_open ? 1U : 0U;
	planner_status.replay_turn_direction = planner_replay_turn_direction;
	planner_status.replay_turn_open_step = planner_replay_turn_open_step;
}

static void SecondDrive_ResetReplayPair(
		SecondDriveGeometrySource_t source)
{
	planner_replay_turn_open = false;
	planner_replay_turn_direction = 0;
	planner_replay_turn_open_step = 0U;
	planner_cross_exit_active = true;
	planner_status.local_close_repair_active = 0U;
	planner_status.geometry_source = source;
	SecondDrive_PublishPairState();
}

static void SecondDrive_ApplyReplayPairEvent(const TrackMarkerEvent_t *event)
{
	int8_t direction;

	if (event == NULL) {
		return;
	}
	planner_status.local_close_repair_active = 0U;
	if (event->type == MARKER_EVENT_CROSS) {
		SecondDrive_ResetReplayPair(SECOND_DRIVE_GEOMETRY_CROSS_RESET);
		planner_last_direction = 0;
		planner_last_direction_step = event->center_step;
		return;
	}
	if (event->type == MARKER_EVENT_BOTH) {
		/* END is a diagnostic boundary.  Never infer a close from centring or
		 * from the bilateral END marker itself. */
		planner_cross_exit_active = false;
		SecondDrive_PublishPairState();
		return;
	}
	if (!SecondDrive_EventDirection(event, &direction)) {
		return;
	}
	planner_last_direction = direction;
	planner_last_direction_step = event->center_step;
	planner_cross_exit_active = false;
	if (!planner_replay_turn_open) {
		planner_replay_turn_open = true;
		planner_replay_turn_direction = direction;
		planner_replay_turn_open_step = event->center_step;
		planner_status.geometry_source = SECOND_DRIVE_GEOMETRY_PAIR_OPEN;
	} else if (planner_replay_turn_direction == direction) {
		planner_replay_turn_open = false;
		planner_replay_turn_direction = 0;
		planner_replay_turn_open_step = 0U;
		planner_status.geometry_source = SECOND_DRIVE_GEOMETRY_PAIR_CLOSE;
	} else {
		planner_replay_turn_open = true;
		planner_replay_turn_direction = direction;
		planner_replay_turn_open_step = event->center_step;
		planner_status.geometry_source = SECOND_DRIVE_GEOMETRY_PAIR_OPEN;
	}
	SecondDrive_PublishPairState();
}

static bool SecondDrive_IsFinalEndExpected(void)
{
	const TrackMarkerEvent_t *expected_event =
			Track_GetEvent(planner_expected_event_index);
	const TrackSegment_t *last_segment;

	if ((expected_event == NULL)
			|| (expected_event->type != MARKER_EVENT_BOTH)
			|| (planner_expected_event_index + 1U != Track_GetEventCount())) {
		return false;
	}
	last_segment = Track_GetSegment(Track_GetSegmentCount() - 1U);
	return (last_segment != NULL)
			&& (last_segment->type == TRACK_SEGMENT_END);
}

static bool SecondDrive_GeometryEvidenceAllowsFast(void)
{
	return (planner_status.geometry_source
			== SECOND_DRIVE_GEOMETRY_MAP_SEGMENT)
			|| (planner_status.geometry_source
					== SECOND_DRIVE_GEOMETRY_PAIR_CLOSE)
			|| (planner_status.geometry_source
					== SECOND_DRIVE_GEOMETRY_CROSS_RESET)
			|| (planner_status.geometry_source
					== SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR);
}

static void SecondDrive_UpdateFastGate(bool line_valid,
		int32_t line_position, FirstDriveCoursePhase_t course_phase)
{
	uint32_t absolute_position = (line_position < 0)
			? (uint32_t)(-line_position) : (uint32_t)line_position;
	bool phase_allowed = (course_phase == FIRST_DRIVE_COURSE_STRAIGHT)
			|| (course_phase == FIRST_DRIVE_COURSE_CROSS);

	if (!line_valid || !phase_allowed) {
		if (planner_fast_gate_ready && planner_run_active
				&& (planner_run_stats.fast_exit_count < UINT16_MAX)) {
			planner_run_stats.fast_exit_count++;
		}
		if (planner_fast_gate_ready) {
			planner_status.fast_gate_ready = 0U;
		}
		planner_fast_gate_ready = false;
		planner_fast_stable_frames = 0U;
		planner_status.centered_stable_frames = 0U;
		return;
	}
	if (absolute_position <= SECOND_DRIVE_FAST_ENTER_POSITION) {
		if (planner_fast_stable_frames < UINT16_MAX) {
			planner_fast_stable_frames++;
		}
		planner_status.centered_stable_frames = planner_fast_stable_frames;
	} else if (absolute_position >= SECOND_DRIVE_FAST_EXIT_POSITION) {
		planner_fast_stable_frames = 0U;
		planner_status.centered_stable_frames = 0U;
		if (planner_fast_gate_ready && planner_run_active
				&& (planner_run_stats.fast_exit_count < UINT16_MAX)) {
			planner_run_stats.fast_exit_count++;
		}
		planner_fast_gate_ready = false;
	} else if (!planner_fast_gate_ready) {
		/* Hysteresis band: a not-yet-entered gate still needs a centered run. */
		planner_fast_stable_frames = 0U;
		planner_status.centered_stable_frames = 0U;
	}
	if (!planner_fast_gate_ready
			&& (planner_fast_stable_frames
					>= SECOND_DRIVE_FAST_STABLE_FRAMES)) {
		planner_fast_gate_ready = true;
		planner_status.fast_gate_ready = 1U;
		if (planner_run_active
				&& (planner_run_stats.fast_entry_count < UINT16_MAX)) {
			planner_run_stats.fast_entry_count++;
		}
	}
	planner_status.fast_gate_ready = planner_fast_gate_ready ? 1U : 0U;
}

void SecondDrivePlanner_Reset(void)
{
	const TrackSegment_t *segment;
	const TrackSegment_t *next_segment;
	const TrackMarkerEvent_t *expected_event;
	SecondDrivePlannerStatus_t reset_status;
	uint16_t expected_index = 0U;
	uint8_t map_valid = SecondDrivePlanner_MapIsStructurallyValid() ? 1U : 0U;

	memset(&reset_status, 0, sizeof(reset_status));
	reset_status.map_valid = map_valid;
	reset_status.sync_state = map_valid ? SECOND_DRIVE_SYNC_MAP
			: SECOND_DRIVE_SYNC_INVALID;
	reset_status.last_mismatch_reason = SECOND_DRIVE_MISMATCH_NONE;
	reset_status.limit_reason = SECOND_DRIVE_LIMIT_NONE;
	reset_status.geometry_source = SECOND_DRIVE_GEOMETRY_UNCERTAIN;
	reset_status.segment_count = Track_GetSegmentCount();
	reset_status.anchor_count = Track_GetCrossAnchorCount();
	reset_status.current_anchor_order = UINT16_MAX;
	segment = Track_GetSegment(0U);
	if (segment != NULL) {
		reset_status.geometry_source = SECOND_DRIVE_GEOMETRY_MAP_SEGMENT;
		reset_status.segment_type = segment->type;
		reset_status.curve_units = segment->curve_units;
		reset_status.segment_distance_steps = segment->distance_steps;
		reset_status.segment_remaining_steps = segment->distance_steps;
	}
	next_segment = Track_GetSegment(1U);
	reset_status.next_segment_type = (next_segment != NULL)
			? next_segment->type : TRACK_SEGMENT_END;
	expected_event = Track_GetEvent(expected_index);
	while ((expected_event != NULL)
			&& (expected_event->type == MARKER_EVENT_BOTH)
			&& (expected_event->center_step
					< SECOND_DRIVE_START_MARKER_IGNORE_STEPS)) {
		expected_index++;
		expected_event = Track_GetEvent(expected_index);
	}
	reset_status.expected_event_index = expected_index;

	__disable_irq();
	planner_status = reset_status;
	planner_segment_start_step = 0U;
	planner_expected_event_index = expected_index;
	planner_last_anchor_map_step = 0U;
	planner_last_anchor_run_step = 0U;
	planner_have_anchor = false;
	planner_replay_turn_open = false;
	planner_replay_turn_direction = 0;
	planner_replay_turn_open_step = 0U;
	planner_last_direction = 0;
	planner_last_direction_step = 0U;
	planner_last_repair_step = 0U;
	planner_last_repair_direction = 0;
	planner_fast_gate_ready = false;
	planner_fast_stable_frames = 0U;
	planner_final_exit_stable_frames = 0U;
	planner_final_exit_override = false;
	planner_cross_exit_active = false;
	planner_end_fallback_active = false;
	planner_last_trace_reason = SECOND_DRIVE_LIMIT_NONE;
	planner_last_trace_source = reset_status.geometry_source;
	planner_limiter_episode_valid = false;
	planner_limiter_episode_reason = SECOND_DRIVE_LIMIT_NONE;
	planner_limiter_episode_samples = 0U;
	planner_limiter_episode_start_step = 0U;
	planner_limiter_episode_last_step = 0U;
	planner_run_active = false;
	__enable_irq();
}

static SecondDriveMismatchReason_t SecondDrive_EventMatchesExpected(
		const TrackMarkerEvent_t *event)
{
	const TrackSegment_t *current_segment;
	const TrackSegment_t *next_segment;
	const TrackMarkerEvent_t *expected_event;
	uint32_t travelled;
	uint32_t distance_error;
	uint32_t tolerance;

	current_segment = Track_GetSegment(planner_status.segment_index);
	next_segment = Track_GetSegment(planner_status.segment_index + 1U);
	expected_event = Track_GetEvent(planner_expected_event_index);
	if ((current_segment == NULL) || (next_segment == NULL)
			|| (expected_event == NULL)) {
		return SECOND_DRIVE_MISMATCH_MAP_BOUNDS;
	}
	if (event->center_step < planner_segment_start_step) {
		return SECOND_DRIVE_MISMATCH_MAP_BOUNDS;
	}
	travelled = event->center_step - planner_segment_start_step;
	tolerance = SecondDrive_DistanceTolerance(
			current_segment->distance_steps);
	distance_error = SecondDrive_AbsoluteDifference(travelled,
			current_segment->distance_steps);
	if (event->type != expected_event->type) {
		return SECOND_DRIVE_MISMATCH_EVENT_TYPE;
	}
	if (distance_error > tolerance) {
		return SECOND_DRIVE_MISMATCH_EVENT_DISTANCE;
	}
	return SECOND_DRIVE_MISMATCH_NONE;
}

static bool SecondDrive_ConfirmCrossAnchor(uint16_t event_index,
		uint16_t next_segment_index, uint32_t run_step)
{
	const TrackCrossAnchor_t *anchor =
			Track_FindCrossAnchorByEventIndex(event_index);

	if ((anchor == NULL)
			|| (anchor->segment_index_after_cross != next_segment_index)
			|| (planner_have_anchor
					&& (anchor->order <= planner_status.current_anchor_order))
			|| !SecondDrive_MapIndexIsValid(next_segment_index)) {
		return false;
	}
	planner_status.current_anchor_order = anchor->order;
	planner_last_anchor_map_step = anchor->center_step;
	planner_last_anchor_run_step = run_step;
	planner_have_anchor = true;
	return true;
}

static bool SecondDrive_AdvanceAfterEvent(const TrackMarkerEvent_t *event)
{
	uint16_t next_segment_index = planner_status.segment_index + 1U;
	const TrackSegment_t *next_segment = Track_GetSegment(next_segment_index);
	uint16_t current_event_index = planner_expected_event_index;

	if (next_segment == NULL) {
		return false;
	}
	if ((event->type == MARKER_EVENT_CROSS)
			&& !SecondDrive_ConfirmCrossAnchor(current_event_index,
					next_segment_index, event->center_step)) {
		return false;
	}
	if (planner_expected_event_index < Track_GetEventCount()) {
		SecondDrive_SetExpectedEventIndex(
			(uint16_t)(planner_expected_event_index + 1U));
	}
	planner_segment_start_step = event->center_step;
	planner_status.segment_start_step = planner_segment_start_step;
	SecondDrive_LoadSegmentStatus(next_segment_index);
	return true;
}

static bool SecondDrive_TryLocalCloseRepair(
		const TrackMarkerEvent_t *event)
{
	const TrackMarkerEvent_t *expected_event;
	const TrackSegment_t *current_segment;
	int8_t direction;
	uint32_t gap_steps;

	if (!SecondDrive_EventDirection(event, &direction)) {
		return false;
	}
	if ((planner_status.map_valid == 0U)
			|| (planner_status.sync_state != SECOND_DRIVE_SYNC_MAP)) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_MAP_NOT_SYNC;
		return false;
	}
	if (event->confidence < 20U) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_LOW_CONFIDENCE;
		return false;
	}
	if (!planner_replay_turn_open) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_NO_OPEN;
		return false;
	}
	if (planner_replay_turn_direction != direction) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_WRONG_SIDE;
		return false;
	}
	expected_event = Track_GetEvent(planner_expected_event_index);
	if ((expected_event == NULL)
			|| ((expected_event->type != MARKER_EVENT_CROSS)
					&& !SecondDrive_IsFinalEndExpected())) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_EXPECTED_NOT_BOUNDARY;
		return false;
	}
	current_segment = Track_GetSegment(planner_status.segment_index);
	if ((current_segment == NULL)
			|| (((direction < 0) ? TRACK_SEGMENT_LEFT : TRACK_SEGMENT_RIGHT)
					!= current_segment->type)) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_SEGMENT_MISMATCH;
		return false;
	}
	if (event->center_step < planner_replay_turn_open_step) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_DUPLICATE;
		return false;
	}
	gap_steps = event->center_step - planner_replay_turn_open_step;
	if ((gap_steps <= TRACK_MARK_COOLDOWN_STEPS)
			|| ((planner_last_repair_step == event->center_step)
					&& (planner_last_repair_direction == direction))) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_DUPLICATE;
		return false;
	}

	/* This is supplemental replay geometry only.  Keep the map event index,
	 * segment start and every finalized First Drive array untouched. */
	planner_replay_turn_open = false;
	planner_replay_turn_direction = 0;
	planner_replay_turn_open_step = 0U;
	planner_cross_exit_active = false;
	planner_last_direction = direction;
	planner_last_direction_step = event->center_step;
	planner_last_repair_step = event->center_step;
	planner_last_repair_direction = direction;
	planner_status.local_close_repair_active = 1U;
	planner_status.local_close_repair_direction = direction;
	planner_status.local_close_repair_step = event->center_step;
	planner_status.local_repair_reject_reason = SECOND_DRIVE_LOCAL_REPAIR_NONE;
	planner_status.geometry_source =
			SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR;
	planner_status.last_mismatch_reason = SECOND_DRIVE_MISMATCH_NONE;
	if (planner_status.local_close_repair_count < UINT16_MAX) {
		planner_status.local_close_repair_count++;
	}
	if (planner_run_active) {
		if (planner_run_stats.local_close_repair_count < UINT16_MAX) {
			planner_run_stats.local_close_repair_count++;
		}
	}
	SecondDrive_PublishPairState();
	return true;
}

typedef enum {
	SECOND_DRIVE_ANCHOR_NO_MATCH = 0,
	SECOND_DRIVE_ANCHOR_MATCH,
	SECOND_DRIVE_ANCHOR_AMBIGUOUS
} SecondDriveAnchorMatchResult_t;

static SecondDriveAnchorMatchResult_t SecondDrive_FindAnchorCandidate(
		const TrackMarkerEvent_t *event, const TrackCrossAnchor_t **match,
		SecondDriveMismatchReason_t *failure_reason)
{
	const TrackCrossAnchor_t *best_anchor = NULL;
	uint32_t best_error = UINT32_MAX;
	uint32_t second_error = UINT32_MAX;
	uint16_t index;
	uint8_t candidates_checked = 0U;
	bool have_best = false;

	if (failure_reason != NULL) {
		*failure_reason = SECOND_DRIVE_MISMATCH_ANCHOR_NOT_FOUND;
	}
	if (match != NULL) {
		*match = NULL;
	}
	for (index = 0U; index < Track_GetCrossAnchorCount(); index++) {
		const TrackCrossAnchor_t *candidate = Track_GetCrossAnchor(index);
		uint32_t observed_distance;
		uint32_t recorded_distance;
		uint32_t tolerance;
		uint32_t error;

		if (candidate == NULL) {
			break;
		}
		if (planner_have_anchor
				&& (candidate->order <= planner_status.current_anchor_order)) {
			continue;
		}
		if (candidate->event_index < planner_expected_event_index) {
			continue;
		}
		if (candidates_checked >= SECOND_DRIVE_ANCHOR_LOOKAHEAD_COUNT) {
			break;
		}
		candidates_checked++;

		if (planner_have_anchor) {
			if ((event->center_step < planner_last_anchor_run_step)
					|| (candidate->center_step < planner_last_anchor_map_step)) {
				continue;
			}
			observed_distance = event->center_step
					- planner_last_anchor_run_step;
			recorded_distance = candidate->center_step
					- planner_last_anchor_map_step;
		} else {
			observed_distance = event->center_step;
			recorded_distance = candidate->center_step;
		}
		tolerance = recorded_distance
				/ SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_DIV;
		if (tolerance < SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_MIN) {
			tolerance = SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_MIN;
		}
		error = SecondDrive_AbsoluteDifference(observed_distance,
				recorded_distance);
		if (error > tolerance) {
			continue;
		}
		if (!have_best || (error < best_error)) {
			second_error = best_error;
			best_error = error;
			best_anchor = candidate;
			have_best = true;
		} else if (error < second_error) {
			second_error = error;
		}
	}

	if (!have_best) {
		return SECOND_DRIVE_ANCHOR_NO_MATCH;
	}
	if ((second_error != UINT32_MAX)
			&& ((second_error <= best_error)
					|| ((second_error - best_error)
							<= SECOND_DRIVE_ANCHOR_TIE_TOLERANCE_STEPS))) {
		if (failure_reason != NULL) {
			*failure_reason = SECOND_DRIVE_MISMATCH_ANCHOR_AMBIGUOUS;
		}
		return SECOND_DRIVE_ANCHOR_AMBIGUOUS;
	}
	if (best_anchor == NULL
			|| !SecondDrive_MapIndexIsValid(
					best_anchor->segment_index_after_cross)) {
		if (failure_reason != NULL) {
			*failure_reason = SECOND_DRIVE_MISMATCH_MAP_BOUNDS;
		}
		return SECOND_DRIVE_ANCHOR_NO_MATCH;
	}
	if (match != NULL) {
		*match = best_anchor;
	}
	return SECOND_DRIVE_ANCHOR_MATCH;
}

static bool SecondDrive_TryResyncAtCross(const TrackMarkerEvent_t *event,
		SecondDriveMismatchReason_t *failure_reason)
{
	const TrackCrossAnchor_t *anchor = NULL;
	SecondDriveAnchorMatchResult_t result;
	uint16_t next_event_index;

	result = SecondDrive_FindAnchorCandidate(event, &anchor, failure_reason);
	if (result != SECOND_DRIVE_ANCHOR_MATCH) {
		return false;
	}
	if (anchor == NULL) {
		if (failure_reason != NULL) {
			*failure_reason = SECOND_DRIVE_MISMATCH_MAP_BOUNDS;
		}
		return false;
	}
	next_event_index = anchor->event_index;
	if (next_event_index < Track_GetEventCount()) {
		next_event_index++;
	}
	SecondDrive_SetExpectedEventIndex(next_event_index);
	planner_segment_start_step = event->center_step;
	planner_status.segment_start_step = planner_segment_start_step;
	planner_status.current_anchor_order = anchor->order;
	planner_last_anchor_map_step = anchor->center_step;
	planner_last_anchor_run_step = event->center_step;
	planner_have_anchor = true;
	planner_status.sync_state = SECOND_DRIVE_SYNC_MAP;
	planner_status.last_mismatch_reason = SECOND_DRIVE_MISMATCH_NONE;
	SecondDrive_IncrementU8(&planner_status.resync_count);
	if (planner_run_active && (planner_run_stats.resync_count < UINT16_MAX)) {
		planner_run_stats.resync_count++;
	}
	SecondDrive_ResetReplayPair(SECOND_DRIVE_GEOMETRY_CROSS_RESET);
	SecondDrive_LoadSegmentStatus(anchor->segment_index_after_cross);
	return true;
}

void SecondDrivePlanner_OnEvent(const TrackMarkerEvent_t *event)
{
	SecondDriveMismatchReason_t reason;
	SecondDriveMismatchReason_t anchor_reason;
	SecondDriveAnchorMatchResult_t anchor_result;
	int8_t event_direction;
	bool final_end_event;

	if (event == NULL) {
		return;
	}
	if (planner_status.map_valid == 0U) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_MAP_NOT_SYNC;
		return;
	}
	if ((event->type == MARKER_EVENT_BOTH)
			&& (event->center_step < SECOND_DRIVE_START_MARKER_IGNORE_STEPS)) {
		return;
	}
	if (event->confidence < 20U) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_LOW_CONFIDENCE;
		SecondDrive_IncrementU16(&planner_status.ignored_event_count);
		return;
	}
	SecondDrive_IncrementU16(&planner_status.replay_event_count);

	if (planner_status.sync_state == SECOND_DRIVE_SYNC_INVALID) {
		planner_status.local_repair_reject_reason =
				SECOND_DRIVE_LOCAL_REPAIR_MAP_NOT_SYNC;
		SecondDrive_IncrementU16(&planner_status.ignored_event_count);
		return;
	}
	if (planner_status.sync_state == SECOND_DRIVE_SYNC_SEEK_CROSS) {
		if (event->type == MARKER_EVENT_CROSS) {
			anchor_reason = SECOND_DRIVE_MISMATCH_ANCHOR_NOT_FOUND;
			if (SecondDrive_TryResyncAtCross(event, &anchor_reason)) {
				return;
			}
			planner_status.last_mismatch_reason = anchor_reason;
		} else {
			planner_status.local_repair_reject_reason =
					SECOND_DRIVE_LOCAL_REPAIR_MAP_NOT_SYNC;
		}
		SecondDrive_IncrementU16(&planner_status.ignored_event_count);
		return;
	}

	reason = SecondDrive_EventMatchesExpected(event);
	if (reason == SECOND_DRIVE_MISMATCH_NONE) {
		final_end_event = (event->type == MARKER_EVENT_BOTH)
				&& SecondDrive_IsFinalEndExpected();
		if (final_end_event) {
			/* Preserve the last planner-policy snapshot before advancing past the
			 * final BOTH event.  RecordEndBrake() then stores this policy together
			 * with the event step instead of depending on a stale next-tick status. */
			planner_status.final_end_expected = 1U;
			if (planner_status.end_policy == SECOND_DRIVE_END_POLICY_NONE) {
				planner_status.end_policy = planner_end_fallback_active
						? SECOND_DRIVE_END_POLICY_SAFE_APPROACH
						: SECOND_DRIVE_END_POLICY_MAP_UNCERTAIN;
			}
			planner_status.end_fallback_active =
					planner_end_fallback_active ? 1U : 0U;
		}
		if (!SecondDrive_AdvanceAfterEvent(event)) {
			SecondDrive_EnterSeekCross(
					SECOND_DRIVE_MISMATCH_MAP_BOUNDS);
			SecondDrive_IncrementU16(&planner_status.ignored_event_count);
			return;
		}
		planner_status.last_mismatch_reason = SECOND_DRIVE_MISMATCH_NONE;
		SecondDrive_ApplyReplayPairEvent(event);
		return;
	}
	if (SecondDrive_EventDirection(event, &event_direction)) {
		/* Directional mismatch is tracked separately from anchor resync. */
		(void)event_direction;
		SecondDrive_IncrementU16(&planner_status.marker_pair_mismatch_count);
		if (SecondDrive_TryLocalCloseRepair(event)) {
			return;
		}
	}

	if (event->type == MARKER_EVENT_CROSS) {
		anchor_reason = SECOND_DRIVE_MISMATCH_ANCHOR_NOT_FOUND;
		anchor_result = SecondDrive_FindAnchorCandidate(event, NULL,
				&anchor_reason);
		if (anchor_result == SECOND_DRIVE_ANCHOR_MATCH) {
			/* The event was unexpected for the current map position, but the
			 * physical CROSS is a strong enough absolute anchor to recover now. */
			SecondDrive_IncrementU8(&planner_status.mismatch_count);
			if (planner_run_active) {
				SecondDrive_IncrementU16(&planner_run_stats.mismatch_count);
			}
			if (SecondDrive_TryResyncAtCross(event, &anchor_reason)) {
				return;
			}
			reason = anchor_reason;
		}
		reason = anchor_reason;
	}
	SecondDrive_EnterSeekCross(reason);
	SecondDrive_IncrementU16(&planner_status.ignored_event_count);
}

static uint32_t SecondDrive_DistanceToNextRestriction(uint32_t current_remaining,
		uint16_t current_segment_index, bool override_current_geometry,
		TrackSegmentType_t *restriction_type)
{
	const TrackSegment_t *current_segment =
			Track_GetSegment(current_segment_index);
	uint64_t total = current_remaining;
	uint32_t offset;

	if (restriction_type != NULL) {
		*restriction_type = TRACK_SEGMENT_STRAIGHT;
	}
	if (current_segment == NULL) {
		if (restriction_type != NULL) {
			*restriction_type = TRACK_SEGMENT_END;
		}
		return 0U;
	}
	if (!override_current_geometry
			&& !SecondDrive_IsFastGeometry(current_segment->type)) {
		if (restriction_type != NULL) {
			*restriction_type = current_segment->type;
		}
		return current_remaining;
	}

	for (offset = 1U; offset <= SECOND_DRIVE_LOOKAHEAD_MAX_SEGMENTS;
			offset++) {
		uint32_t candidate_index = (uint32_t)current_segment_index + offset;
		const TrackSegment_t *candidate;

		if (candidate_index > UINT16_MAX) {
			break;
		}
		candidate = Track_GetSegment((uint16_t)candidate_index);
		if (candidate == NULL) {
			break;
		}
		if (!SecondDrive_IsFastGeometry(candidate->type)) {
			if (restriction_type != NULL) {
				*restriction_type = candidate->type;
			}
			return (total > UINT32_MAX) ? UINT32_MAX
					: (uint32_t)total;
		}
		total += candidate->distance_steps;
	}

	/* No turn/end was found in the bounded horizon.  The next control tick
	 * will scan again after a confirmed event; keep the current straight target
	 * instead of inventing a far-away restriction. */
	if (restriction_type != NULL) {
		*restriction_type = TRACK_SEGMENT_STRAIGHT;
	}
	return UINT32_MAX;
}

static bool SecondDrive_DistanceToFinalEnd(uint32_t current_remaining,
		uint16_t current_segment_index, uint32_t *distance)
{
	uint64_t total = current_remaining;
	uint16_t index;

	if (distance == NULL) {
		return false;
	}
	for (index = current_segment_index;
			index < Track_GetSegmentCount(); index++) {
		const TrackSegment_t *segment = Track_GetSegment(index);

		if (segment == NULL) {
			return false;
		}
		if ((index != current_segment_index)
				&& (segment->type == TRACK_SEGMENT_END)) {
			*distance = (total > UINT32_MAX) ? UINT32_MAX
					: (uint32_t)total;
			return true;
		}
		if (index != current_segment_index) {
			total += segment->distance_steps;
		}
		if (segment->type == TRACK_SEGMENT_END) {
			*distance = (total > UINT32_MAX) ? UINT32_MAX
					: (uint32_t)total;
			return true;
		}
	}
	return false;
}

static uint16_t SecondDrive_SetPlannerTarget(uint16_t target,
		SecondDriveLimitReason_t reason)
{
	planner_status.planner_target_sps = target;
	planner_status.limit_reason = reason;
	return target;
}

static bool SecondDrive_IsCurvePhase(FirstDriveCoursePhase_t phase)
{
	return (phase == FIRST_DRIVE_COURSE_APPROACH_LEFT)
			|| (phase == FIRST_DRIVE_COURSE_APPROACH_RIGHT)
			|| (phase == FIRST_DRIVE_COURSE_TURN_LEFT)
			|| (phase == FIRST_DRIVE_COURSE_TURN_RIGHT)
			|| (phase == FIRST_DRIVE_COURSE_EXIT_LEFT)
			|| (phase == FIRST_DRIVE_COURSE_EXIT_RIGHT);
}

static uint16_t SecondDrive_GetPhaseTarget(
		FirstDriveCoursePhase_t course_phase,
		SecondDriveLimitReason_t *reason)
{
	if (reason == NULL) {
		return SecondDrive_GetEffectiveCurveSps();
	}
	switch (course_phase) {
	case FIRST_DRIVE_COURSE_APPROACH_LEFT:
	case FIRST_DRIVE_COURSE_APPROACH_RIGHT:
		*reason = SECOND_DRIVE_LIMIT_CURVE_APPROACH;
		return SecondDrive_GetEffectiveApproachSps();
	case FIRST_DRIVE_COURSE_EXIT_LEFT:
	case FIRST_DRIVE_COURSE_EXIT_RIGHT:
		*reason = SECOND_DRIVE_LIMIT_CURVE_EXIT;
		return SecondDrive_GetEffectiveExitSps();
	case FIRST_DRIVE_COURSE_TURN_LEFT:
	case FIRST_DRIVE_COURSE_TURN_RIGHT:
		*reason = SECOND_DRIVE_LIMIT_CURVE_CRUISE;
		return SecondDrive_GetEffectiveCurveSps();
	default:
		*reason = SECOND_DRIVE_LIMIT_CURVE_CRUISE;
		return SecondDrive_GetEffectiveCurveSps();
	}
}

static void SecondDrive_UpdateFinalExitOverride(bool final_end_expected,
		bool line_valid, bool recovering, bool provisional_marker,
		int32_t line_position, FirstDriveCoursePhase_t course_phase)
{
	uint32_t absolute_position = (line_position < 0)
			? (uint32_t)(-line_position) : (uint32_t)line_position;
	bool exit_phase = (course_phase == FIRST_DRIVE_COURSE_EXIT_LEFT)
			|| (course_phase == FIRST_DRIVE_COURSE_EXIT_RIGHT);

	if (!final_end_expected || !exit_phase || !line_valid || recovering
			|| provisional_marker || planner_replay_turn_open
			|| ((planner_status.geometry_source
					!= SECOND_DRIVE_GEOMETRY_PAIR_CLOSE)
				&& (planner_status.geometry_source
						!= SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR))) {
		planner_final_exit_stable_frames = 0U;
		planner_final_exit_override = false;
		return;
	}
	if (absolute_position <= SECOND_DRIVE_FAST_ENTER_POSITION) {
		if (planner_final_exit_stable_frames < UINT16_MAX) {
			planner_final_exit_stable_frames++;
		}
	} else {
		planner_final_exit_stable_frames = 0U;
		planner_final_exit_override = false;
	}
	planner_status.centered_stable_frames = planner_final_exit_stable_frames;
	if (planner_final_exit_stable_frames
			>= SECOND_DRIVE_FINAL_EXIT_STABLE_FRAMES) {
		planner_final_exit_override = true;
	}
}

static SecondDriveLimitReason_t SecondDrive_GetFastLimitReason(
		bool performance_clamped, bool cross_corridor)
{
	if (performance_clamped) {
		return SECOND_DRIVE_LIMIT_MAX_CLAMP;
	}
	if (planner_cross_exit_active) {
		return SECOND_DRIVE_LIMIT_FAST_CROSS_EXIT;
	}
	if (cross_corridor) {
		return (planner_status.geometry_source
				== SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR)
				? SECOND_DRIVE_LIMIT_FAST_LOCAL_CLOSE_REPAIR
				: SECOND_DRIVE_LIMIT_FAST_CROSS_APPROACH;
	}
	return SECOND_DRIVE_LIMIT_FAST_STRAIGHT;
}

uint16_t SecondDrivePlanner_GetTargetSps(uint16_t first_drive_target_sps,
		int32_t line_position, FirstDriveCoursePhase_t course_phase,
		uint32_t average_step, uint16_t current_sps, bool line_valid,
		bool recovering, bool provisional_marker)
{
	const TrackSegment_t *segment;
	const TrackMarkerEvent_t *expected_event;
	TrackSegmentType_t restriction_type;
	SecondDriveLimitReason_t phase_reason;
	SecondDriveLimitReason_t nominal_reason = SECOND_DRIVE_LIMIT_SEGMENT_UNCERTAIN;
	uint32_t travelled;
	uint32_t remaining;
	uint32_t distance_to_restriction;
	uint32_t end_distance;
	uint32_t braking_steps;
	uint16_t nominal_target;
	uint16_t planner_target;
	uint16_t restriction_target;
	uint64_t braking_limit;
	uint32_t absolute_position = (line_position < 0)
			? (uint32_t)(-line_position) : (uint32_t)line_position;
	bool cross_corridor = false;
	bool final_corridor = false;
	bool override_current_geometry = false;
	bool performance_clamped = false;
	bool fast_eligible = false;
	bool final_end_expected = false;
	bool end_distance_valid = false;
	bool turn_restriction_active = false;

	planner_status.cross_approach_corridor = 0U;
	planner_status.final_end_corridor = 0U;
	planner_status.final_end_expected = 0U;
	planner_status.end_policy = SECOND_DRIVE_END_POLICY_NONE;
	planner_status.end_distance_steps = 0U;
	planner_status.expected_marker_distance_steps = 0U;
	planner_status.next_restriction_distance_steps = 0U;
	SecondDrive_UpdateFastGate(line_valid, line_position, course_phase);

	if ((planner_status.map_valid == 0U)
			|| (planner_status.sync_state != SECOND_DRIVE_SYNC_MAP)) {
		planner_status.geometry_source = SECOND_DRIVE_GEOMETRY_UNCERTAIN;
		planner_status.nominal_target_sps = first_drive_target_sps;
		planner_status.planner_target_sps = first_drive_target_sps;
	planner_status.limit_reason = (planner_status.map_valid == 0U)
				? SECOND_DRIVE_LIMIT_MAP_INVALID
				: SECOND_DRIVE_LIMIT_SEEK_CROSS;
		return first_drive_target_sps;
	}

	segment = Track_GetSegment(planner_status.segment_index);
	if (segment == NULL) {
		SecondDrive_EnterSeekCross(SECOND_DRIVE_MISMATCH_MAP_BOUNDS);
		planner_status.nominal_target_sps = first_drive_target_sps;
		planner_status.planner_target_sps = first_drive_target_sps;
		planner_status.limit_reason = SECOND_DRIVE_LIMIT_SEGMENT_UNCERTAIN;
		return first_drive_target_sps;
	}
	if (average_step < planner_segment_start_step) {
		SecondDrive_EnterSeekCross(SECOND_DRIVE_MISMATCH_MAP_BOUNDS);
		planner_status.nominal_target_sps = first_drive_target_sps;
		planner_status.planner_target_sps = first_drive_target_sps;
		planner_status.limit_reason = SECOND_DRIVE_LIMIT_SEGMENT_UNCERTAIN;
		return first_drive_target_sps;
	}
	travelled = average_step - planner_segment_start_step;
	remaining = (travelled < segment->distance_steps)
			? (segment->distance_steps - travelled) : 0U;
	planner_status.segment_travelled_steps = travelled;
	planner_status.segment_remaining_steps = remaining;
	if ((travelled > segment->distance_steps)
			&& ((travelled - segment->distance_steps)
					> SecondDrive_DistanceTolerance(segment->distance_steps))) {
		SecondDrive_EnterSeekCross(SECOND_DRIVE_MISMATCH_SEGMENT_OVERDUE);
		planner_status.nominal_target_sps = first_drive_target_sps;
		planner_status.planner_target_sps = first_drive_target_sps;
		planner_status.limit_reason = SECOND_DRIVE_LIMIT_SEGMENT_UNCERTAIN;
		return first_drive_target_sps;
	}
	expected_event = Track_GetEvent(planner_expected_event_index);
	if ((expected_event != NULL)
			&& (expected_event->center_step >= planner_segment_start_step)) {
		planner_status.expected_marker_distance_steps =
				expected_event->center_step - planner_segment_start_step;
	}
	final_end_expected = SecondDrive_IsFinalEndExpected();
	planner_status.final_end_expected = final_end_expected ? 1U : 0U;
	SecondDrive_UpdateFinalExitOverride(
			final_end_expected, line_valid, recovering,
			provisional_marker, line_position, course_phase);

	final_corridor = final_end_expected
			&& !planner_replay_turn_open
			&& SecondDrive_GeometryEvidenceAllowsFast()
			&& line_valid && !recovering && !provisional_marker
			&& ((course_phase == FIRST_DRIVE_COURSE_STRAIGHT
					&& planner_fast_gate_ready)
				|| planner_final_exit_override);
	if (final_corridor
			&& !SecondDrive_IsFastGeometry(segment->type)
			&& (planner_status.geometry_source
					!= SECOND_DRIVE_GEOMETRY_PAIR_CLOSE)
			&& (planner_status.geometry_source
					!= SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR)) {
		final_corridor = false;
	}
	planner_status.final_end_corridor = final_corridor ? 1U : 0U;
	if (final_end_expected) {
		end_distance_valid = SecondDrive_DistanceToFinalEnd(remaining,
				planner_status.segment_index, &end_distance);
		if (end_distance_valid) {
			planner_status.end_distance_steps = end_distance;
			planner_status.end_policy = final_corridor
					? SECOND_DRIVE_END_POLICY_FAST_CORRIDOR
					: SECOND_DRIVE_END_POLICY_SAFE_APPROACH;
		} else {
			planner_status.end_policy =
					SECOND_DRIVE_END_POLICY_MAP_UNCERTAIN;
		}
	}

	cross_corridor = (expected_event != NULL)
			&& (expected_event->type == MARKER_EVENT_CROSS)
			&& !planner_replay_turn_open
			&& SecondDrive_GeometryEvidenceAllowsFast()
			&& line_valid && !recovering && !provisional_marker
			&& (course_phase == FIRST_DRIVE_COURSE_STRAIGHT)
			&& planner_fast_gate_ready;
	if (cross_corridor
			&& !SecondDrive_IsFastGeometry(segment->type)
			&& (planner_status.geometry_source
					!= SECOND_DRIVE_GEOMETRY_PAIR_CLOSE)
			&& (planner_status.geometry_source
					!= SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR)) {
		cross_corridor = false;
	}
	planner_status.cross_approach_corridor = cross_corridor ? 1U : 0U;
	override_current_geometry = cross_corridor
			&& !SecondDrive_IsFastGeometry(segment->type);

	/* Select a nominal target first.  All branches below converge on the
	 * restriction caps so an END fallback cannot be skipped by a curve, pair,
	 * position, or phase early return. */
	if (final_corridor && !planner_end_fallback_active) {
		nominal_target = SecondDrive_ScalePerformanceSps(
				second_drive_config.straight_sps, &performance_clamped);
		nominal_reason = performance_clamped ? SECOND_DRIVE_LIMIT_MAX_CLAMP
				: ((planner_status.geometry_source
						== SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR)
					? SECOND_DRIVE_LIMIT_FAST_LOCAL_CLOSE_REPAIR
					: SECOND_DRIVE_LIMIT_FAST_END_CORRIDOR);
	} else if (SecondDrive_IsCurvePhase(course_phase)
			&& (!SecondDrive_IsFastGeometry(segment->type)
					|| !planner_fast_gate_ready || planner_replay_turn_open)) {
		nominal_target = SecondDrive_GetPhaseTarget(course_phase, &phase_reason);
		nominal_reason = phase_reason;
	} else {
		fast_eligible = (SecondDrive_IsFastGeometry(segment->type)
			|| cross_corridor)
			&& line_valid && !recovering && !provisional_marker
			&& !planner_replay_turn_open
			&& planner_fast_gate_ready
			&& ((course_phase == FIRST_DRIVE_COURSE_STRAIGHT)
					|| (course_phase == FIRST_DRIVE_COURSE_CROSS));
		if (!fast_eligible) {
			if (planner_replay_turn_open) {
				nominal_target = SecondDrive_GetPhaseTarget(course_phase,
						&phase_reason);
				nominal_reason = SECOND_DRIVE_LIMIT_MARKER_PAIR_UNCLOSED;
			} else if (!line_valid
					|| (absolute_position > SECOND_DRIVE_FAST_ENTER_POSITION)) {
				nominal_target = first_drive_target_sps;
				nominal_reason = SECOND_DRIVE_LIMIT_POSITION;
			} else if (course_phase != FIRST_DRIVE_COURSE_STRAIGHT) {
				nominal_target = first_drive_target_sps;
				nominal_reason = SECOND_DRIVE_LIMIT_PHASE_NOT_STRAIGHT;
			} else if (segment->type == TRACK_SEGMENT_END) {
				nominal_target = SECOND_DRIVE_END_APPROACH_SPS;
				nominal_reason = SECOND_DRIVE_LIMIT_END_BRAKE;
			} else {
				nominal_target = first_drive_target_sps;
				nominal_reason = SECOND_DRIVE_LIMIT_SEGMENT_UNCERTAIN;
			}
		} else {
			distance_to_restriction = SecondDrive_DistanceToNextRestriction(
					remaining, planner_status.segment_index,
					override_current_geometry, &restriction_type);
			planner_status.next_restriction_distance_steps =
					distance_to_restriction;
			nominal_target = SecondDrive_ScalePerformanceSps(
					second_drive_config.straight_sps, &performance_clamped);
			nominal_reason = SecondDrive_GetFastLimitReason(
					performance_clamped, cross_corridor);
			if (distance_to_restriction != UINT32_MAX) {
				restriction_target = (restriction_type == TRACK_SEGMENT_LEFT
						|| restriction_type == TRACK_SEGMENT_RIGHT)
						? SecondDrive_GetEffectiveApproachSps()
						: SecondDrive_TargetForSegment(restriction_type);
				braking_steps = SecondDrive_BrakingSteps(
						(nominal_target > current_sps) ? nominal_target
								: current_sps, restriction_target);
				braking_limit = (uint64_t)braking_steps
						+ SECOND_DRIVE_BRAKE_MARGIN_STEPS
						+ SECOND_DRIVE_ACCEL_ENABLE_MARGIN_STEPS;
				if ((uint64_t)distance_to_restriction <= braking_limit) {
					turn_restriction_active = true;
				}
			}
		}
	}

	/* END safe fallback is a sticky restriction.  It is deliberately evaluated
	 * after nominal phase/pair/position selection, so no safety branch can
	 * return before it. */
	planner_status.nominal_target_sps = nominal_target;
	planner_target = nominal_target;
	planner_status.end_fallback_active = planner_end_fallback_active ? 1U : 0U;
	if (final_end_expected && !final_corridor && end_distance_valid) {
		uint16_t high_sps = (nominal_target > current_sps)
				? nominal_target : current_sps;

		braking_steps = SecondDrive_BrakingSteps(high_sps,
				SECOND_DRIVE_END_APPROACH_SPS);
		braking_limit = (uint64_t)braking_steps
				+ SECOND_DRIVE_BRAKE_MARGIN_STEPS;
		if (planner_end_fallback_active
				|| ((uint64_t)end_distance <= braking_limit)) {
			if (!planner_end_fallback_active) {
				planner_end_fallback_active = true;
				planner_status.end_fallback_entry_step = average_step;
				planner_status.end_fallback_entry_sps = high_sps;
				if (planner_run_active) {
					planner_run_stats.end_fallback_entry_step = average_step;
					planner_run_stats.end_fallback_entry_sps = high_sps;
				}
			}
			planner_status.end_fallback_active = 1U;
			planner_target = (nominal_target > SECOND_DRIVE_END_APPROACH_SPS)
					? SECOND_DRIVE_END_APPROACH_SPS : nominal_target;
			nominal_reason = SECOND_DRIVE_LIMIT_END_APPROACH_SAFE;
		}
	}
	if (planner_end_fallback_active) {
		planner_target = (planner_target > SECOND_DRIVE_END_APPROACH_SPS)
				? SECOND_DRIVE_END_APPROACH_SPS : planner_target;
		nominal_reason = SECOND_DRIVE_LIMIT_END_APPROACH_SAFE;
		planner_status.limit_reason = SECOND_DRIVE_LIMIT_END_APPROACH_SAFE;
	} else if (turn_restriction_active && !final_corridor) {
		planner_target = restriction_target;
		nominal_reason = SECOND_DRIVE_LIMIT_TURN_BRAKE;
	}
	return SecondDrive_SetPlannerTarget(planner_target, nominal_reason);
}

static void SecondDrive_IncrementU32(volatile uint32_t *value)
{
	if ((value != NULL) && (*value < UINT32_MAX)) {
		(*value)++;
	}
}

static void SecondDrive_FinalizeLimiterEpisode(void)
{
	uint32_t duration_steps;
	uint32_t duration_ms;
	SecondDriveLimitReason_t reason;

	if (!planner_limiter_episode_valid || !planner_run_active) {
		return;
	}
	reason = planner_limiter_episode_reason;
	if (reason >= SECOND_DRIVE_LIMIT_COUNT) {
		planner_limiter_episode_valid = false;
		return;
	}
	if (planner_limiter_episode_samples
			> planner_run_stats.limiter_max_consecutive_samples[reason]) {
		planner_run_stats.limiter_max_consecutive_samples[reason] =
				planner_limiter_episode_samples;
	}
	duration_ms = planner_limiter_episode_samples;
	duration_steps = (planner_limiter_episode_last_step
			>= planner_limiter_episode_start_step)
			? (planner_limiter_episode_last_step
					- planner_limiter_episode_start_step) : 0U;
	switch (reason) {
	case SECOND_DRIVE_LIMIT_SEEK_CROSS:
		if (duration_ms > planner_run_stats.seek_max_ms) {
			planner_run_stats.seek_max_ms = duration_ms;
		}
		if (duration_steps > planner_run_stats.seek_max_steps) {
			planner_run_stats.seek_max_steps = duration_steps;
		}
		break;
	case SECOND_DRIVE_LIMIT_MARKER_PAIR_UNCLOSED:
		if (duration_ms > planner_run_stats.pair_open_max_ms) {
			planner_run_stats.pair_open_max_ms = duration_ms;
		}
		break;
	case SECOND_DRIVE_LIMIT_POSITION:
		if (duration_ms > planner_run_stats.position_limit_max_ms) {
			planner_run_stats.position_limit_max_ms = duration_ms;
		}
		break;
	case SECOND_DRIVE_LIMIT_RECOVERY:
		if (duration_ms > planner_run_stats.recovery_max_ms) {
			planner_run_stats.recovery_max_ms = duration_ms;
		}
		break;
	default:
		break;
	}
	planner_limiter_episode_valid = false;
	planner_limiter_episode_samples = 0U;
}

static void SecondDrive_RecordLimiterEpisodeSample(
		SecondDriveLimitReason_t reason, uint32_t average_step)
{
	if (!planner_run_active || (reason >= SECOND_DRIVE_LIMIT_COUNT)) {
		return;
	}
	if (!planner_limiter_episode_valid
			|| (planner_limiter_episode_reason != reason)) {
		SecondDrive_FinalizeLimiterEpisode();
		planner_limiter_episode_valid = true;
		planner_limiter_episode_reason = reason;
		planner_limiter_episode_samples = 1U;
		planner_limiter_episode_start_step = average_step;
		planner_limiter_episode_last_step = average_step;
		SecondDrive_IncrementU16(
				&planner_run_stats.limiter_episode_count[reason]);
		if (planner_run_stats.limiter_episode_count[reason] == 1U) {
			planner_run_stats.limiter_first_step[reason] = average_step;
		}
		if (reason == SECOND_DRIVE_LIMIT_SEEK_CROSS
				&& (planner_run_stats.seek_episode_count < UINT16_MAX)) {
			planner_run_stats.seek_episode_count++;
		} else if (reason == SECOND_DRIVE_LIMIT_MARKER_PAIR_UNCLOSED
				&& (planner_run_stats.pair_open_episode_count < UINT16_MAX)) {
			planner_run_stats.pair_open_episode_count++;
		} else if (reason == SECOND_DRIVE_LIMIT_POSITION
				&& (planner_run_stats.position_limit_episode_count
						< UINT16_MAX)) {
			planner_run_stats.position_limit_episode_count++;
		} else if (reason == SECOND_DRIVE_LIMIT_RECOVERY
				&& (planner_run_stats.recovery_episode_count < UINT16_MAX)) {
			planner_run_stats.recovery_episode_count++;
		}
	} else {
		SecondDrive_IncrementU32(&planner_limiter_episode_samples);
		planner_limiter_episode_last_step = average_step;
	}
	planner_run_stats.limiter_last_step[reason] = average_step;
}

static void SecondDrive_AppendLimitTrace(uint16_t final_target_sps,
		int16_t line_position, uint32_t average_step,
		FirstDriveCoursePhase_t course_phase,
		SecondDriveLimitReason_t reason)
{
	uint8_t index;
	SecondDriveLimitTraceEntry_t *entry;

	if (!planner_run_active) {
		return;
	}
	if ((planner_run_stats.trace_count > 0U)
			&& (planner_last_trace_reason == reason)
			&& (planner_last_trace_source == planner_status.geometry_source)) {
		return;
	}
	index = planner_run_stats.trace_head;
	entry = (SecondDriveLimitTraceEntry_t *)&planner_run_stats.trace[index];
	entry->step = average_step;
	entry->segment_index = planner_status.segment_index;
	entry->segment_type = planner_status.segment_type;
	entry->phase = course_phase;
	entry->reason = reason;
	entry->geometry_source = planner_status.geometry_source;
	entry->replay_turn_open = planner_replay_turn_open ? 1U : 0U;
	entry->replay_turn_direction = planner_replay_turn_direction;
	entry->requested_sps = planner_status.planner_target_sps;
	entry->final_sps = final_target_sps;
	entry->line_position = line_position;
	entry->distance_to_restriction_steps =
			planner_status.next_restriction_distance_steps;
	entry->expected_event_index = planner_status.expected_event_index;
	entry->sync_state = planner_status.sync_state;
	entry->final_end_expected = planner_status.final_end_expected;
	planner_run_stats.trace_head = (uint8_t)((index + 1U)
			% SECOND_DRIVE_LIMIT_TRACE_DEPTH);
	if (planner_run_stats.trace_count < SECOND_DRIVE_LIMIT_TRACE_DEPTH) {
		planner_run_stats.trace_count++;
	}
	planner_last_trace_reason = reason;
	planner_last_trace_source = planner_status.geometry_source;
}

void SecondDrivePlanner_BeginRun(void)
{
	__disable_irq();
	memset((void *)&planner_run_stats, 0, sizeof(planner_run_stats));
	planner_run_active = true;
	planner_last_trace_reason = SECOND_DRIVE_LIMIT_NONE;
	planner_last_trace_source = planner_status.geometry_source;
	planner_limiter_episode_valid = false;
	planner_limiter_episode_reason = SECOND_DRIVE_LIMIT_NONE;
	planner_limiter_episode_samples = 0U;
	planner_limiter_episode_start_step = 0U;
	planner_limiter_episode_last_step = 0U;
	planner_run_stats.marker_candidate_last_reject_reason =
			SECOND_DRIVE_MARKER_REJECT_COUNT;
	__enable_irq();
}

void SecondDrivePlanner_RecordFinalTarget(uint16_t final_target_sps,
		uint16_t actual_center_sps, int16_t line_position,
		uint32_t average_step, FirstDriveCoursePhase_t course_phase,
		bool recovery_slow)
{
	SecondDriveLimitReason_t reason = planner_status.limit_reason;

	if (recovery_slow) {
		reason = SECOND_DRIVE_LIMIT_RECOVERY;
	}
	if (reason >= SECOND_DRIVE_LIMIT_COUNT) {
		reason = SECOND_DRIVE_LIMIT_SEGMENT_UNCERTAIN;
	}
	planner_status.final_target_sps = final_target_sps;
	planner_status.limit_reason = reason;
	if (!planner_run_active) {
		return;
	}
	SecondDrive_IncrementU32(&planner_run_stats.control_samples);
	planner_run_stats.center_sps_sum += actual_center_sps;
	if (actual_center_sps > planner_run_stats.center_sps_max) {
		planner_run_stats.center_sps_max = actual_center_sps;
	}
	if (final_target_sps > planner_run_stats.target_sps_max) {
		planner_run_stats.target_sps_max = final_target_sps;
	}
	SecondDrive_IncrementU32(&planner_run_stats.limiter_samples[reason]);
	SecondDrive_RecordLimiterEpisodeSample(reason, average_step);
	SecondDrive_AppendLimitTrace(final_target_sps, line_position,
			average_step, course_phase, reason);
}

void SecondDrivePlanner_RecordMarkerReject(
		SecondDriveMarkerRejectReason_t reason)
{
	if (reason < SECOND_DRIVE_MARKER_REJECT_COUNT) {
		SecondDrivePlanner_RecordMarkerCandidateEpisode(false,
				(uint8_t)(1U << (uint8_t)reason), 0U);
	}
}

void SecondDrivePlanner_RecordMarkerCandidateEpisode(bool accepted,
		uint8_t reject_reason_mask, uint32_t step)
{
	uint8_t reason;

	if (!planner_run_active) {
		return;
	}
	SecondDrive_IncrementU16(&planner_run_stats.marker_candidate_episode_count);
	planner_run_stats.marker_candidate_last_step = step;
	if (accepted) {
		SecondDrive_IncrementU16(
				&planner_run_stats.marker_candidate_accepted_count);
		return;
	}
	SecondDrive_IncrementU16(
			&planner_run_stats.marker_candidate_rejected_count);
	if (reject_reason_mask == 0U) {
		reject_reason_mask = (uint8_t)(1U <<
				SECOND_DRIVE_MARKER_REJECT_COOLDOWN_OR_DUPLICATE);
	}
	for (reason = 0U; reason < SECOND_DRIVE_MARKER_REJECT_COUNT; reason++) {
		if ((reject_reason_mask & (uint8_t)(1U << reason)) == 0U) {
			continue;
		}
		switch ((SecondDriveMarkerRejectReason_t)reason) {
		case SECOND_DRIVE_MARKER_REJECT_NO_LINE:
			SecondDrive_IncrementU16(
					&planner_run_stats.marker_reject_no_line_count);
			break;
		case SECOND_DRIVE_MARKER_REJECT_OFF_CENTER:
			SecondDrive_IncrementU16(
					&planner_run_stats.marker_reject_off_center_count);
			break;
		case SECOND_DRIVE_MARKER_REJECT_NO_CENTER_MASK:
			SecondDrive_IncrementU16(
					&planner_run_stats.marker_reject_no_center_mask_count);
			break;
		case SECOND_DRIVE_MARKER_REJECT_BRIDGE:
			SecondDrive_IncrementU16(
					&planner_run_stats.marker_reject_bridge_count);
			break;
		case SECOND_DRIVE_MARKER_REJECT_CROSS_TAIL_SUPPRESSED:
			SecondDrive_IncrementU16(
				&planner_run_stats.marker_reject_cross_tail_count);
			break;
		case SECOND_DRIVE_MARKER_REJECT_LOW_CONFIDENCE:
			SecondDrive_IncrementU16(
				&planner_run_stats.marker_reject_low_confidence_count);
			break;
		case SECOND_DRIVE_MARKER_REJECT_COOLDOWN_OR_DUPLICATE:
			SecondDrive_IncrementU16(
				&planner_run_stats.marker_reject_duplicate_count);
			break;
		default:
			break;
		}
	}
	for (reason = 0U; reason < SECOND_DRIVE_MARKER_REJECT_COUNT; reason++) {
		if ((reject_reason_mask & (uint8_t)(1U << reason)) != 0U) {
			planner_run_stats.marker_candidate_last_reject_reason = reason;
			break;
		}
	}
}

bool SecondDrivePlanner_IsFinalEndCorridor(void)
{
	return planner_status.final_end_corridor != 0U;
}

void SecondDrivePlanner_RecordEndBrake(uint32_t step, uint16_t entry_sps)
{
	const TrackMarkerEvent_t *expected_event = NULL;
	uint16_t event_count = Track_GetEventCount();

	if (planner_expected_event_index < event_count) {
		expected_event = Track_GetEvent(planner_expected_event_index);
	}
	/* A confirmed END is processed before FirstDrive_StopAtEndMarker(), and
	 * the normal expected-event path advances the planner index past the final
	 * BOTH event.  Preserve the finalized map's last event as the reference in
	 * that case so the stop-distance diagnostic remains meaningful. */
	if ((expected_event == NULL) && (event_count > 0U)) {
		const TrackMarkerEvent_t *last_event =
				Track_GetEvent((uint16_t)(event_count - 1U));

		if ((last_event != NULL) && (last_event->type == MARKER_EVENT_BOTH)) {
			expected_event = last_event;
		}
	}

	if (!planner_run_active) {
		return;
	}
	planner_status.confirmed_end_stop = 1U;
	planner_run_stats.final_end_expected = planner_status.final_end_expected;
	planner_run_stats.end_policy = planner_status.end_policy;
	planner_run_stats.end_fallback_active =
			planner_status.end_fallback_active;
	planner_run_stats.end_fallback_entry_step =
			planner_status.end_fallback_entry_step;
	planner_run_stats.end_fallback_entry_sps =
			planner_status.end_fallback_entry_sps;
	planner_run_stats.end_distance_steps = planner_status.end_distance_steps;
	planner_run_stats.end_brake_step = step;
	planner_run_stats.end_brake_entry_sps = entry_sps;
	if (expected_event != NULL) {
		planner_run_stats.expected_end_step = expected_event->center_step;
		planner_run_stats.end_step_error = (int32_t)step
				- (int32_t)expected_event->center_step;
	}
	if (planner_replay_turn_open
			&& (planner_run_stats.unmatched_turn_at_end_count < UINT16_MAX)) {
		planner_run_stats.unmatched_turn_at_end_count++;
	}
}

void SecondDrivePlanner_RecordStopMode(SecondDriveStopMode_t mode,
		bool brake_start_attempted, bool brake_start_succeeded)
{
	planner_status.stop_mode = mode;
	planner_status.brake_start_attempted = brake_start_attempted ? 1U : 0U;
	planner_status.brake_start_succeeded = brake_start_succeeded ? 1U : 0U;
	if (!planner_run_active) {
		return;
	}
	planner_run_stats.stop_mode = mode;
	planner_run_stats.brake_start_attempted =
			brake_start_attempted ? 1U : 0U;
	planner_run_stats.brake_start_succeeded =
			brake_start_succeeded ? 1U : 0U;
}

void SecondDrivePlanner_RecordBrakeCompletion(uint16_t hold_ms,
		bool completed)
{
	if (!planner_run_active) {
		return;
	}
	planner_run_stats.brake_hold_ms = hold_ms;
	planner_run_stats.end_brake_completed = completed ? 1U : 0U;
}

void SecondDrivePlanner_FinalizeRunStats(uint32_t elapsed_ms)
{
	if (!planner_run_active) {
		return;
	}
	SecondDrive_FinalizeLimiterEpisode();
	planner_run_stats.final_end_expected = planner_status.final_end_expected;
	planner_run_stats.end_policy = planner_status.end_policy;
	planner_run_stats.end_fallback_active =
			planner_status.end_fallback_active;
	planner_run_stats.end_distance_steps = planner_status.end_distance_steps;
	if (planner_run_stats.stop_mode == SECOND_DRIVE_STOP_MODE_NONE) {
		planner_run_stats.stop_mode = planner_status.stop_mode;
	}
	planner_run_stats.elapsed_ms = elapsed_ms;
	planner_run_stats.valid = 1U;
	planner_run_active = false;
}

void SecondDrivePlanner_GetRunStats(SecondDriveRunStats_t *stats)
{
	if (stats == NULL) {
		return;
	}
	__disable_irq();
	*stats = (const SecondDriveRunStats_t)planner_run_stats;
	__enable_irq();
}

void SecondDrivePlanner_GetStatus(SecondDrivePlannerStatus_t *status)
{
	if (status == NULL) {
		return;
	}
	__disable_irq();
	*status = (const SecondDrivePlannerStatus_t)planner_status;
	__enable_irq();
}

const SecondDriveConfig_t *SecondDrive_GetConfig(void)
{
	return &second_drive_config;
}

uint16_t SecondDrive_GetEffectiveStraightSps(void)
{
	return second_drive_effective_straight_sps;
}

uint16_t SecondDrive_GetEffectiveCurveSps(void)
{
	return second_drive_effective_curve_sps;
}

uint16_t SecondDrive_GetEffectiveApproachSps(void)
{
	return second_drive_effective_approach_sps;
}

uint16_t SecondDrive_GetEffectiveExitSps(void)
{
	return second_drive_effective_exit_sps;
}

bool SecondDrive_SetStraightSps(uint16_t straight_sps)
{
	if (!SecondDrive_StateAllowsConfiguration(SecondDrive_GetState())
			|| (straight_sps < SECOND_DRIVE_STRAIGHT_MIN_SPS)
			|| (straight_sps > SECOND_DRIVE_STRAIGHT_MAX_SPS)) {
		return false;
	}
	second_drive_config.straight_sps = straight_sps;
	SecondDrive_RecomputeEffectiveProfile();
	return true;
}

bool SecondDrive_SetCurveSps(uint16_t curve_sps)
{
	if (!SecondDrive_StateAllowsConfiguration(SecondDrive_GetState())
			|| (curve_sps < SECOND_DRIVE_CURVE_MIN_SPS)
			|| (curve_sps > SECOND_DRIVE_CURVE_MAX_SPS)) {
		return false;
	}
	second_drive_config.curve_sps = curve_sps;
	SecondDrive_RecomputeEffectiveProfile();
	return true;
}

bool SecondDrive_SetOverallPercent(uint8_t overall_percent)
{
	if (!SecondDrive_StateAllowsConfiguration(SecondDrive_GetState())
			|| (overall_percent < SECOND_DRIVE_OVERALL_MIN_PERCENT)
			|| (overall_percent > SECOND_DRIVE_OVERALL_MAX_PERCENT)) {
		return false;
	}
	second_drive_config.overall_percent = overall_percent;
	SecondDrive_RecomputeEffectiveProfile();
	return true;
}
