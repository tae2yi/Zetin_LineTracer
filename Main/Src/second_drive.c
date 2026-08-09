/*
 * second_drive.c
 *
 * Uses the finalized First Drive segment map as a speed-planning aid.  Motor
 * step distance predicts braking, but only a re-detected physical marker moves
 * the segment index.  Any map mismatch permanently falls back to the proven
 * First Drive speed envelope for the remainder of the run.
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
#define SECOND_DRIVE_CROSS_REFERENCE_SPS         2400U
#define SECOND_DRIVE_END_APPROACH_SPS            1800U
#define SECOND_DRIVE_DECEL_SPS_PER_SECOND       10000U
#define SECOND_DRIVE_BRAKE_MARGIN_STEPS           300U
#define SECOND_DRIVE_ACCEL_ENABLE_MARGIN_STEPS    150U
#define SECOND_DRIVE_MIN_FAST_STRAIGHT_STEPS      700U
#define SECOND_DRIVE_POSITION_FAST_LIMIT          700
#define SECOND_DRIVE_START_MARKER_IGNORE_STEPS    300U
#define SECOND_DRIVE_DISTANCE_TOLERANCE_MIN        200U

static SecondDriveConfig_t second_drive_config = {
	.straight_sps = SECOND_DRIVE_DEFAULT_STRAIGHT_SPS,
	.overall_percent = SECOND_DRIVE_DEFAULT_OVERALL_PERCENT
};

static volatile SecondDrivePlannerStatus_t planner_status;
static volatile uint32_t planner_segment_start_step;
static volatile uint16_t planner_expected_event_index;

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
	case TRACK_SEGMENT_CROSS:
		return SecondDrive_ScaleSps(SECOND_DRIVE_CROSS_REFERENCE_SPS);
	case TRACK_SEGMENT_END:
		return SecondDrive_ScaleSps(SECOND_DRIVE_END_APPROACH_SPS);
	case TRACK_SEGMENT_STRAIGHT:
	default:
		return second_drive_config.straight_sps;
	}
}

bool SecondDrivePlanner_MapIsStructurallyValid(void)
{
	uint16_t count = Track_GetSegmentCount();
	const TrackSegment_t *last_segment;

	if (Track_HasOverflow() || (Track_GetEventCount() == 0U) || (count == 0U)) {
		return false;
	}
	last_segment = Track_GetSegment(count - 1U);
	return (last_segment != NULL)
			&& (last_segment->type == TRACK_SEGMENT_END);
}

void SecondDrivePlanner_Reset(void)
{
	const TrackSegment_t *segment;
	const TrackSegment_t *next_segment;
	const TrackMarkerEvent_t *expected_event;
	SecondDrivePlannerStatus_t reset_status;
	uint16_t expected_index = 0U;

	memset(&reset_status, 0, sizeof(reset_status));
	reset_status.map_valid = SecondDrivePlanner_MapIsStructurallyValid() ? 1U : 0U;
	reset_status.segment_count = Track_GetSegmentCount();
	segment = Track_GetSegment(0U);
	if (segment != NULL) {
		reset_status.segment_type = segment->type;
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

	__disable_irq();
	planner_status = reset_status;
	planner_segment_start_step = 0U;
	planner_expected_event_index = expected_index;
	__enable_irq();
}

void SecondDrivePlanner_OnEvent(const TrackMarkerEvent_t *event)
{
	const TrackSegment_t *current_segment;
	const TrackSegment_t *next_segment;
	const TrackMarkerEvent_t *expected_event;
	uint32_t travelled;
	uint32_t distance_error;
	uint32_t tolerance;
	bool mismatch = false;
	uint16_t next_index;

	if ((event == NULL) || (event->confidence < 20U)
			|| (planner_status.map_valid == 0U)) {
		return;
	}
	if ((event->type == MARKER_EVENT_BOTH)
			&& (event->center_step < SECOND_DRIVE_START_MARKER_IGNORE_STEPS)) {
		return;
	}

	planner_status.replay_event_count++;
	current_segment = Track_GetSegment(planner_status.segment_index);
	next_index = planner_status.segment_index + 1U;
	next_segment = Track_GetSegment(next_index);
	expected_event = Track_GetEvent(planner_expected_event_index);
	travelled = event->center_step - planner_segment_start_step;

	if ((current_segment == NULL) || (next_segment == NULL)
			|| (expected_event == NULL)) {
		mismatch = true;
	} else {
		tolerance = SecondDrive_DistanceTolerance(
				current_segment->distance_steps);
		distance_error = (travelled > current_segment->distance_steps)
				? (travelled - current_segment->distance_steps)
				: (current_segment->distance_steps - travelled);
		if ((event->type != expected_event->type)
				|| (distance_error > tolerance)) {
			mismatch = true;
		}
	}

	if (mismatch) {
		planner_status.fallback_active = 1U;
		if (planner_status.mismatch_count < UINT8_MAX) {
			planner_status.mismatch_count++;
		}
	}

	if (next_segment != NULL) {
		if (planner_expected_event_index < Track_GetEventCount()) {
			planner_expected_event_index++;
		}
		planner_status.segment_index = next_index;
		planner_status.segment_type = next_segment->type;
		planner_status.segment_distance_steps = next_segment->distance_steps;
		planner_status.segment_travelled_steps = 0U;
		planner_status.segment_remaining_steps = next_segment->distance_steps;
		planner_segment_start_step = event->center_step;
		next_segment = Track_GetSegment(next_index + 1U);
		planner_status.next_segment_type = (next_segment != NULL)
				? next_segment->type : TRACK_SEGMENT_END;
	}
}

uint16_t SecondDrivePlanner_GetTargetSps(uint16_t first_drive_target_sps,
		int32_t line_position, FirstDriveCoursePhase_t course_phase,
		uint32_t average_step, uint16_t current_sps)
{
	const TrackSegment_t *segment;
	const TrackSegment_t *next_segment;
	uint32_t travelled;
	uint32_t remaining;
	uint32_t tolerance;
	uint32_t braking_steps;
	uint16_t scaled_target;
	uint16_t next_target;
	uint32_t absolute_position = (line_position < 0)
			? (uint32_t)(-line_position) : (uint32_t)line_position;

	if ((planner_status.map_valid == 0U)
			|| (planner_status.fallback_active != 0U)) {
		return first_drive_target_sps;
	}

	segment = Track_GetSegment(planner_status.segment_index);
	if (segment == NULL) {
		planner_status.fallback_active = 1U;
		return first_drive_target_sps;
	}
	travelled = average_step - planner_segment_start_step;
	remaining = (travelled < segment->distance_steps)
			? (segment->distance_steps - travelled) : 0U;
	planner_status.segment_travelled_steps = travelled;
	planner_status.segment_remaining_steps = remaining;
	tolerance = SecondDrive_DistanceTolerance(segment->distance_steps);
	if (travelled > (segment->distance_steps + tolerance)) {
		planner_status.fallback_active = 1U;
		if (planner_status.mismatch_count < UINT8_MAX) {
			planner_status.mismatch_count++;
		}
		return first_drive_target_sps;
	}

	scaled_target = SecondDrive_ScaleSps(first_drive_target_sps);
	if ((segment->type != TRACK_SEGMENT_STRAIGHT)
			|| (course_phase != FIRST_DRIVE_COURSE_STRAIGHT)
			|| (absolute_position > SECOND_DRIVE_POSITION_FAST_LIMIT)
			|| (segment->distance_steps
					< SECOND_DRIVE_MIN_FAST_STRAIGHT_STEPS)) {
		return scaled_target;
	}

	next_segment = Track_GetSegment(planner_status.segment_index + 1U);
	next_target = (next_segment != NULL)
			? SecondDrive_TargetForSegment(next_segment->type)
			: SecondDrive_TargetForSegment(TRACK_SEGMENT_END);
	braking_steps = SecondDrive_BrakingSteps(
			(second_drive_config.straight_sps > current_sps)
					? second_drive_config.straight_sps : current_sps,
			next_target);
	if (remaining > (braking_steps + SECOND_DRIVE_BRAKE_MARGIN_STEPS
			+ SECOND_DRIVE_ACCEL_ENABLE_MARGIN_STEPS)) {
		return second_drive_config.straight_sps;
	}
	return next_target;
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
