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

#define SECOND_DRIVE_DEFAULT_STRAIGHT_SPS       5200U
#define SECOND_DRIVE_DEFAULT_OVERALL_PERCENT     100U
#define SECOND_DRIVE_TURN_REFERENCE_SPS          2200U
#define SECOND_DRIVE_END_APPROACH_SPS            1800U
#define SECOND_DRIVE_DECEL_SPS_PER_SECOND       10000U
#define SECOND_DRIVE_BRAKE_MARGIN_STEPS           300U
#define SECOND_DRIVE_ACCEL_ENABLE_MARGIN_STEPS    150U
#define SECOND_DRIVE_POSITION_FAST_LIMIT          700
#define SECOND_DRIVE_START_MARKER_IGNORE_STEPS    300U
#define SECOND_DRIVE_DISTANCE_TOLERANCE_MIN        200U
#define SECOND_DRIVE_ANCHOR_LOOKAHEAD_COUNT          3U
#define SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_MIN 500U
#define SECOND_DRIVE_ANCHOR_DISTANCE_TOLERANCE_DIV   3U
#define SECOND_DRIVE_ANCHOR_TIE_TOLERANCE_STEPS      100U
#define SECOND_DRIVE_LOOKAHEAD_MAX_SEGMENTS          16U

static SecondDriveConfig_t second_drive_config = {
	.straight_sps = SECOND_DRIVE_DEFAULT_STRAIGHT_SPS,
	.overall_percent = SECOND_DRIVE_DEFAULT_OVERALL_PERCENT
};

static volatile SecondDrivePlannerStatus_t planner_status;
static volatile uint32_t planner_segment_start_step;
static volatile uint16_t planner_expected_event_index;
static volatile uint32_t planner_last_anchor_map_step;
static volatile uint32_t planner_last_anchor_run_step;
static volatile bool planner_have_anchor;

static bool SecondDrive_StateAllowsConfiguration(FirstDriveState_t state)
{
	return (state == FIRST_DRIVE_OFF) || (state == FIRST_DRIVE_READY)
			|| (state == FIRST_DRIVE_STOPPED) || (state == FIRST_DRIVE_FAULT);
}

static uint16_t SecondDrive_ScaleSps(uint16_t value)
{
	uint32_t scaled = ((uint32_t)value
			* second_drive_config.overall_percent) / 100U;

	if (scaled > MOTOR_DRIVE_MAX_SPS) {
		scaled = MOTOR_DRIVE_MAX_SPS;
	}
	return (uint16_t)scaled;
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
	uint64_t numerator;
	uint64_t denominator = 2ULL * SECOND_DRIVE_DECEL_SPS_PER_SECOND;

	if (high_sps <= low_sps) {
		return 0U;
	}
	numerator = ((uint64_t)high_sps * high_sps)
			- ((uint64_t)low_sps * low_sps);
	return (uint32_t)((numerator + denominator - 1ULL) / denominator);
}

static uint16_t SecondDrive_TargetForSegment(TrackSegmentType_t type)
{
	switch (type) {
	case TRACK_SEGMENT_LEFT:
	case TRACK_SEGMENT_RIGHT:
		return SecondDrive_ScaleSps(SECOND_DRIVE_TURN_REFERENCE_SPS);
	case TRACK_SEGMENT_END:
		return SecondDrive_ScaleSps(SECOND_DRIVE_END_APPROACH_SPS);
	case TRACK_SEGMENT_CROSS:
	case TRACK_SEGMENT_STRAIGHT:
	default:
		/* A mapped CROSS is a straight-through geometry class. */
		return second_drive_config.straight_sps;
	}
}

static bool SecondDrive_IsFastGeometry(TrackSegmentType_t type)
{
	return (type == TRACK_SEGMENT_STRAIGHT)
			|| (type == TRACK_SEGMENT_CROSS);
}

static bool SecondDrive_CourseAllowsFast(TrackSegmentType_t type,
		FirstDriveCoursePhase_t course_phase)
{
	if (type == TRACK_SEGMENT_STRAIGHT) {
		return course_phase == FIRST_DRIVE_COURSE_STRAIGHT;
	}
	if (type == TRACK_SEGMENT_CROSS) {
		return (course_phase == FIRST_DRIVE_COURSE_STRAIGHT)
				|| (course_phase == FIRST_DRIVE_COURSE_CROSS);
	}
	return false;
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
		planner_status.sync_state = SECOND_DRIVE_SYNC_SEEK_CROSS;
	}
	planner_status.last_mismatch_reason = reason;
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
	reset_status.segment_count = Track_GetSegmentCount();
	reset_status.anchor_count = Track_GetCrossAnchorCount();
	reset_status.current_anchor_order = UINT16_MAX;
	segment = Track_GetSegment(0U);
	if (segment != NULL) {
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
	SecondDrive_LoadSegmentStatus(next_segment_index);
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
	planner_status.current_anchor_order = anchor->order;
	planner_last_anchor_map_step = anchor->center_step;
	planner_last_anchor_run_step = event->center_step;
	planner_have_anchor = true;
	planner_status.sync_state = SECOND_DRIVE_SYNC_MAP;
	planner_status.last_mismatch_reason = SECOND_DRIVE_MISMATCH_NONE;
	SecondDrive_IncrementU8(&planner_status.resync_count);
	SecondDrive_LoadSegmentStatus(anchor->segment_index_after_cross);
	return true;
}

void SecondDrivePlanner_OnEvent(const TrackMarkerEvent_t *event)
{
	SecondDriveMismatchReason_t reason;
	SecondDriveMismatchReason_t anchor_reason;
	SecondDriveAnchorMatchResult_t anchor_result;

	if ((event == NULL) || (event->confidence < 20U)
			|| (planner_status.map_valid == 0U)) {
		return;
	}
	if ((event->type == MARKER_EVENT_BOTH)
			&& (event->center_step < SECOND_DRIVE_START_MARKER_IGNORE_STEPS)) {
		return;
	}
	SecondDrive_IncrementU16(&planner_status.replay_event_count);

	if (planner_status.sync_state == SECOND_DRIVE_SYNC_INVALID) {
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
		}
		SecondDrive_IncrementU16(&planner_status.ignored_event_count);
		return;
	}

	reason = SecondDrive_EventMatchesExpected(event);
	if (reason == SECOND_DRIVE_MISMATCH_NONE) {
		if (!SecondDrive_AdvanceAfterEvent(event)) {
			SecondDrive_EnterSeekCross(
					SECOND_DRIVE_MISMATCH_MAP_BOUNDS);
			SecondDrive_IncrementU16(&planner_status.ignored_event_count);
			return;
		}
		planner_status.last_mismatch_reason = SECOND_DRIVE_MISMATCH_NONE;
		return;
	}

	if (event->type == MARKER_EVENT_CROSS) {
		anchor_reason = SECOND_DRIVE_MISMATCH_ANCHOR_NOT_FOUND;
		anchor_result = SecondDrive_FindAnchorCandidate(event, NULL,
				&anchor_reason);
		if (anchor_result == SECOND_DRIVE_ANCHOR_MATCH) {
			/* The event was unexpected for the current map position, but the
			 * physical CROSS is a strong enough absolute anchor to recover now. */
			SecondDrive_IncrementU8(&planner_status.mismatch_count);
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
		uint16_t current_segment_index, TrackSegmentType_t *restriction_type)
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
	if (!SecondDrive_IsFastGeometry(current_segment->type)) {
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

uint16_t SecondDrivePlanner_GetTargetSps(uint16_t first_drive_target_sps,
		int32_t line_position, FirstDriveCoursePhase_t course_phase,
		uint32_t average_step, uint16_t current_sps)
{
	const TrackSegment_t *segment;
	TrackSegmentType_t restriction_type;
	uint32_t travelled;
	uint32_t remaining;
	uint32_t distance_to_restriction;
	uint32_t braking_steps;
	uint16_t scaled_target;
	uint16_t restriction_target;
	uint64_t braking_limit;
	uint32_t absolute_position = (line_position < 0)
			? (uint32_t)(-line_position) : (uint32_t)line_position;

	if ((planner_status.map_valid == 0U)
			|| (planner_status.sync_state != SECOND_DRIVE_SYNC_MAP)) {
		return first_drive_target_sps;
	}

	segment = Track_GetSegment(planner_status.segment_index);
	if (segment == NULL) {
		SecondDrive_EnterSeekCross(SECOND_DRIVE_MISMATCH_MAP_BOUNDS);
		return first_drive_target_sps;
	}
	if (average_step < planner_segment_start_step) {
		SecondDrive_EnterSeekCross(SECOND_DRIVE_MISMATCH_MAP_BOUNDS);
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
		return first_drive_target_sps;
	}

	distance_to_restriction = SecondDrive_DistanceToNextRestriction(remaining,
			planner_status.segment_index, &restriction_type);
	planner_status.next_restriction_distance_steps = distance_to_restriction;
	scaled_target = SecondDrive_ScaleSps(first_drive_target_sps);
	if (!SecondDrive_IsFastGeometry(segment->type)
			|| !SecondDrive_CourseAllowsFast(segment->type, course_phase)
			|| (absolute_position > SECOND_DRIVE_POSITION_FAST_LIMIT)) {
		return scaled_target;
	}
	if (distance_to_restriction == UINT32_MAX) {
		return second_drive_config.straight_sps;
	}

	restriction_target = SecondDrive_TargetForSegment(restriction_type);
	braking_steps = SecondDrive_BrakingSteps(
			(second_drive_config.straight_sps > current_sps)
					? second_drive_config.straight_sps : current_sps,
			restriction_target);
	braking_limit = (uint64_t)braking_steps
			+ SECOND_DRIVE_BRAKE_MARGIN_STEPS
			+ SECOND_DRIVE_ACCEL_ENABLE_MARGIN_STEPS;
	if ((uint64_t)distance_to_restriction > braking_limit) {
		return second_drive_config.straight_sps;
	}
	return restriction_target;
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

bool SecondDrive_SetStraightSps(uint16_t straight_sps)
{
	if (!SecondDrive_StateAllowsConfiguration(SecondDrive_GetState())
			|| (straight_sps < SECOND_DRIVE_STRAIGHT_MIN_SPS)
			|| (straight_sps > SECOND_DRIVE_STRAIGHT_MAX_SPS)) {
		return false;
	}
	second_drive_config.straight_sps = straight_sps;
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
	return true;
}
