/*
 * drive.c
 *
 * First Drive V25 controller: V21 driving behavior with expanded track memory,
 * continuous speed/steering-ratio control, corner-marker FSM and state-aware
 * fail-safe line-loss handling.
 */

#include "drive.h"

#include "main.h"
#include "motor.h"
#include "second_drive.h"
#include "sensor.h"
#include "tim.h"
#include "track.h"

#include <stddef.h>
#include <string.h>

#define FIRST_DRIVE_VREF_DAC             MOTOR_VREF_DAC_RUN
#define FIRST_DRIVE_CONTROL_DIVIDER         2U /* TIM7 is 2kHz, control 1kHz */
#define FIRST_DRIVE_START_DELAY_MS         2000U
#define FIRST_DRIVE_MOTOR_START_SPS         400U
#define FIRST_DRIVE_ACCEL_SPS_PER_MS          8U /* approximately 3.1 m/s^2 */
#define FIRST_DRIVE_DECEL_SPS_PER_MS         10U /* approximately 3.9 m/s^2 */
#define FIRST_DRIVE_LINE_LOST_GRACE_MS       3U
#define FIRST_DRIVE_LINE_LOST_STRAIGHT_MS   60U
#define FIRST_DRIVE_LINE_LOST_TRANSITION_MS 80U
#define FIRST_DRIVE_LINE_LOST_TURN_MS      120U
#define FIRST_DRIVE_LINE_LOST_BRIDGE_MS    140U
#define FIRST_DRIVE_LINE_LOST_OUTER_EVIDENCE_MS 250U
#define FIRST_DRIVE_LINE_UNSTABLE_FAULT_MS 150U
#define FIRST_DRIVE_REACQUIRE_FRAMES         5U
#define FIRST_DRIVE_SENSOR_STALE_FAULT     10U
#define FIRST_DRIVE_D_FILTER_SHIFT          2U
#define FIRST_DRIVE_DERIVATIVE_LIMIT      2000
#define FIRST_DRIVE_STEER_RATIO_Q15      32768
#define FIRST_DRIVE_STEER_RATIO_NORMAL_Q15 14746 /* 0.45 */
#define FIRST_DRIVE_STEER_RATIO_RECOVERY_Q15 19661 /* 0.60 */
#define FIRST_DRIVE_STEER_RATIO_SLEW_Q15    384 /* 0.0117 per ms */
#define FIRST_DRIVE_APPROACH_FF_Q15         1966 /* 0.06 */
#define FIRST_DRIVE_TURN_FF_Q15             2949 /* 0.09 */
#define FIRST_DRIVE_OUTER_GAIN_POS          1500
#define FIRST_DRIVE_POSITION_DEADBAND       100
#define FIRST_DRIVE_STARTUP_TIMEOUT_MS      20U
#define FIRST_DRIVE_COUNTDOWN_STALE_MS      20U
#define FIRST_DRIVE_CONTROL_WATCHDOG_MS     10U
#define FIRST_DRIVE_STEER_SIGN             -1 /* S0/negative is physical left */
#define FIRST_DRIVE_CURVE_MIN_SPS        1800U
#define FIRST_DRIVE_TURN_MAX_SPS         2200U
#define FIRST_DRIVE_MARKER_SLOW_SPS      2400U
#define FIRST_DRIVE_RECOVERY_STRAIGHT_SPS 1400U
#define FIRST_DRIVE_RECOVERY_TURN_SPS     1800U
#define FIRST_DRIVE_CLUSTER_MAX_JUMP       1400
#define FIRST_DRIVE_MARKER_HISTORY_POS     1200
#define FIRST_DRIVE_BRIDGE_ENTER_POS       1500
#define FIRST_DRIVE_OUTER_BOOST_ENTER_POS  2200
#define FIRST_DRIVE_OUTER_BOOST_EXIT_POS   1800
#define FIRST_DRIVE_OUTER_BOOST_FRAMES       15U
#define FIRST_DRIVE_MARKER_CENTER_POS      1000
#define FIRST_DRIVE_MARKER_CENTER_MASK     0x3CU
#define FIRST_DRIVE_EDGE_STUCK_POS         2600
#define FIRST_DRIVE_EDGE_RELEASE_POS       2200
#define FIRST_DRIVE_EDGE_STRAIGHT_FAULT_MS   60U
#define FIRST_DRIVE_EDGE_TRANSITION_FAULT_MS 200U
#define FIRST_DRIVE_EDGE_PROGRESS_POS        200U
#define FIRST_DRIVE_TURN_ENTER_POS         700
#define FIRST_DRIVE_TURN_EXIT_POS          400
#define FIRST_DRIVE_TURN_EXIT_FRAMES        30U
#define FIRST_DRIVE_MARKER_SLOW_STEPS       600U
#define FIRST_DRIVE_CROSS_PASS_STEPS        510U
#define FIRST_DRIVE_START_MARKER_IGNORE_STEPS 300U
#define FIRST_DRIVE_KP_MAX_Q10            1024
#define FIRST_DRIVE_KD_MAX_Q10             512
#define FIRST_DRIVE_TRIM_MAX_SPS            300

static FirstDriveConfig_t first_drive_config = {
	.launch_sps = MOTOR_SPEED_TEST_RAMP_START_SPS,
	/* 5 cm wheel, 400 half-steps/rev: 3820 SPS is approximately 1.5 m/s. */
	.base_sps = 3820U,
	.minimum_sps = MOTOR_DRIVE_MIN_SPS,
	.maximum_sps = 5200U,
	.ramp_sps_per_10ms = 20U,
	.kp_q10 = 450,
	.kd_q10 = 0,
	.trim_sps = 0
};

static volatile FirstDriveState_t drive_state = FIRST_DRIVE_OFF;
static volatile FirstDriveFault_t drive_fault = FIRST_DRIVE_FAULT_NONE;
static volatile FirstDriveTelemetry_t drive_telemetry;
typedef enum {
	DRIVE_RUN_FIRST = 0,
	DRIVE_RUN_SECOND
} DriveRunMode_t;
static volatile DriveRunMode_t drive_run_mode = DRIVE_RUN_FIRST;
static volatile bool drive_map_ready = false;
static volatile bool drive_control_enabled = false;
static volatile uint32_t drive_control_irq_count;
static volatile uint32_t drive_control_tick_count;
static uint32_t drive_start_tick;
static uint32_t drive_watchdog_tick;
static uint32_t drive_watchdog_irq_count;
static uint32_t drive_last_frame;
static uint16_t drive_current_base_sps;
static int32_t drive_last_position;
static int32_t drive_last_control_position;
static int32_t drive_filtered_derivative;
static int32_t drive_target_steer_q15;
static int32_t drive_applied_steer_q15;
static int8_t drive_bridge_recovery_direction;
static uint16_t drive_outer_boost_frames;
static bool drive_outer_boost_active;
static uint16_t drive_line_lost_frames;
static uint8_t drive_line_reacquire_frames;
static uint32_t drive_line_lost_tick;
static bool drive_line_lost_active;
static uint32_t drive_line_invalid_tick;
static uint16_t drive_line_invalid_limit_ms;
static bool drive_line_invalid_active;
static uint16_t drive_sensor_stale_frames;
static uint8_t drive_timer_divider;
static int8_t drive_expected_turn;
static FirstDriveCoursePhase_t drive_course_phase;
static uint8_t drive_turn_center_frames;
static uint8_t drive_marker_left_frames;
static uint8_t drive_marker_right_frames;
static int8_t drive_provisional_marker_direction;
static uint32_t drive_provisional_marker_step;
static uint32_t drive_marker_slow_until_step;
static uint32_t drive_cross_until_step;
static uint32_t drive_countdown_tick;
static uint32_t drive_countdown_frame_tick;
static uint32_t drive_edge_stuck_tick;
static uint32_t drive_edge_best_position;
static bool drive_edge_stuck_active;
static int8_t drive_edge_stuck_side;

static bool Drive_HasCompletedMap(void)
{
	return drive_map_ready && !Track_HasOverflow()
			&& SecondDrivePlanner_MapIsStructurallyValid();
}

static void FirstDrive_SetFault(FirstDriveFault_t fault)
{
	if (drive_run_mode == DRIVE_RUN_FIRST) {
		drive_map_ready = false;
	}
	drive_fault = fault;
	drive_state = FIRST_DRIVE_FAULT;
	drive_control_enabled = false;
	Motor_DriveStop();
	Sensor_Stop();
	HAL_TIM_Base_Stop_IT(&htim7);

	drive_telemetry.state = FIRST_DRIVE_FAULT;
	drive_telemetry.fault = fault;
}

static void FirstDrive_StopAtEndMarker(void)
{
	drive_telemetry.end_candidate = 1U;
	drive_control_enabled = false;
	Motor_DriveStop();
	Sensor_Stop();
	HAL_TIM_Base_Stop_IT(&htim7);
	if (drive_run_mode == DRIVE_RUN_FIRST) {
		Track_FinalizeSegments();
		drive_map_ready = !Track_HasOverflow()
				&& SecondDrivePlanner_MapIsStructurallyValid();
	}

	drive_current_base_sps = 0U;
	drive_state = FIRST_DRIVE_STOPPED;
	drive_telemetry.state = FIRST_DRIVE_STOPPED;
	drive_telemetry.fault = FIRST_DRIVE_FAULT_NONE;
	drive_telemetry.base_sps = 0U;
	drive_telemetry.centre_sps = 0U;
	drive_telemetry.target_centre_sps = 0U;
	drive_telemetry.left_sps = 0U;
	drive_telemetry.right_sps = 0U;
}

static bool FirstDrive_IsControlState(FirstDriveState_t state)
{
	return (state == FIRST_DRIVE_LAUNCH)
			|| (state == FIRST_DRIVE_FOLLOW)
			|| (state == FIRST_DRIVE_TURN_LEFT)
			|| (state == FIRST_DRIVE_TURN_RIGHT)
			|| (state == FIRST_DRIVE_CROSS_PASS);
}

static bool FirstDrive_StepBefore(uint32_t step, uint32_t limit)
{
	return (int32_t)(limit - step) > 0;
}

static uint32_t FirstDrive_AbsolutePosition(int32_t position)
{
	return (position < 0) ? (uint32_t)(-position) : (uint32_t)position;
}

static uint32_t FirstDrive_PositionDistance(int32_t first, int32_t second)
{
	int32_t difference = first - second;

	return (difference < 0) ? (uint32_t)(-difference)
			: (uint32_t)difference;
}

static int32_t FirstDrive_ApplyPositionDeadband(int32_t position)
{
	return (FirstDrive_AbsolutePosition(position)
			<= FIRST_DRIVE_POSITION_DEADBAND) ? 0 : position;
}

static int32_t FirstDrive_SlewValue(int32_t current, int32_t target,
		int32_t limit)
{
	if (target > (current + limit)) {
		return current + limit;
	}
	if (target < (current - limit)) {
		return current - limit;
	}
	return target;
}

static uint16_t FirstDrive_RampCommonSpeed(uint16_t target)
{
	if (drive_current_base_sps < target) {
		uint32_t next = (uint32_t)drive_current_base_sps
				+ FIRST_DRIVE_ACCEL_SPS_PER_MS;

		drive_current_base_sps = (next > target)
				? target : (uint16_t)next;
	} else if (drive_current_base_sps > target) {
		uint16_t difference = drive_current_base_sps - target;

		drive_current_base_sps -=
				(difference > FIRST_DRIVE_DECEL_SPS_PER_MS)
				? FIRST_DRIVE_DECEL_SPS_PER_MS : difference;
	}
	return drive_current_base_sps;
}

static int8_t FirstDrive_CourseDirection(FirstDriveCoursePhase_t phase)
{
	switch (phase) {
	case FIRST_DRIVE_COURSE_APPROACH_LEFT:
	case FIRST_DRIVE_COURSE_TURN_LEFT:
	case FIRST_DRIVE_COURSE_EXIT_LEFT:
		return -1;
	case FIRST_DRIVE_COURSE_APPROACH_RIGHT:
	case FIRST_DRIVE_COURSE_TURN_RIGHT:
	case FIRST_DRIVE_COURSE_EXIT_RIGHT:
		return 1;
	default:
		return 0;
	}
}

static void FirstDrive_RecordPhaseTransition(
		FirstDriveCoursePhase_t from_phase,
		FirstDriveCoursePhase_t to_phase,
		FirstDrivePhaseReason_t reason, uint32_t average_step)
{
	uint8_t index;

	if (from_phase == to_phase) {
		return;
	}
	for (index = FIRST_DRIVE_PHASE_LOG_DEPTH - 1U; index > 0U; index--) {
		drive_telemetry.phase_log[index] =
				drive_telemetry.phase_log[index - 1U];
	}
	drive_telemetry.phase_log[0].from_phase = from_phase;
	drive_telemetry.phase_log[0].to_phase = to_phase;
	drive_telemetry.phase_log[0].reason = reason;
	drive_telemetry.phase_log[0].step = average_step;
	if (drive_telemetry.phase_log_count < FIRST_DRIVE_PHASE_LOG_DEPTH) {
		drive_telemetry.phase_log_count++;
	}
}

static void FirstDrive_RecordMarkerEvent(const TrackMarkerEvent_t *event)
{
	uint8_t index;

	if (event == NULL) {
		return;
	}
	for (index = FIRST_DRIVE_MARKER_LOG_DEPTH - 1U; index > 0U; index--) {
		drive_telemetry.marker_log[index] =
				drive_telemetry.marker_log[index - 1U];
	}
	drive_telemetry.marker_log[0].type = (uint8_t)event->type;
	drive_telemetry.marker_log[0].confidence = event->confidence;
	drive_telemetry.marker_log[0].edge_union = event->edge_union;
	drive_telemetry.marker_log[0].step = event->center_step;
	if (drive_telemetry.marker_log_count < FIRST_DRIVE_MARKER_LOG_DEPTH) {
		drive_telemetry.marker_log_count++;
	}
}

static void FirstDrive_SetCoursePhase(FirstDriveCoursePhase_t phase,
		uint32_t average_step, FirstDrivePhaseReason_t reason)
{
	FirstDriveCoursePhase_t previous_phase = drive_course_phase;

	FirstDrive_RecordPhaseTransition(previous_phase, phase, reason,
			average_step);
	drive_course_phase = phase;
	drive_turn_center_frames = 0U;

	switch (phase) {
	case FIRST_DRIVE_COURSE_APPROACH_LEFT:
		drive_expected_turn = -1;
		break;
	case FIRST_DRIVE_COURSE_APPROACH_RIGHT:
		drive_expected_turn = 1;
		break;
	case FIRST_DRIVE_COURSE_TURN_LEFT:
		drive_expected_turn = -1;
		drive_state = FIRST_DRIVE_TURN_LEFT;
		break;
	case FIRST_DRIVE_COURSE_TURN_RIGHT:
		drive_expected_turn = 1;
		drive_state = FIRST_DRIVE_TURN_RIGHT;
		break;
	case FIRST_DRIVE_COURSE_CROSS:
		drive_expected_turn = 0;
		drive_state = FIRST_DRIVE_CROSS_PASS;
		break;
	case FIRST_DRIVE_COURSE_EXIT_LEFT:
		drive_expected_turn = -1;
		drive_state = FIRST_DRIVE_TURN_LEFT;
		break;
	case FIRST_DRIVE_COURSE_EXIT_RIGHT:
		drive_expected_turn = 1;
		drive_state = FIRST_DRIVE_TURN_RIGHT;
		break;
	case FIRST_DRIVE_COURSE_STRAIGHT:
	default:
		drive_expected_turn = 0;
		if ((drive_state == FIRST_DRIVE_TURN_LEFT)
				|| (drive_state == FIRST_DRIVE_TURN_RIGHT)
				|| (drive_state == FIRST_DRIVE_CROSS_PASS)) {
			drive_state = FIRST_DRIVE_FOLLOW;
		}
		break;
	}
	drive_telemetry.course_phase = drive_course_phase;
}

static void FirstDrive_HandleDirectionalMarker(int8_t direction,
		uint32_t average_step, FirstDrivePhaseReason_t trigger_reason)
{
	int8_t current_direction = FirstDrive_CourseDirection(drive_course_phase);
	FirstDriveCoursePhase_t next_phase;
	FirstDrivePhaseReason_t transition_reason = trigger_reason;

	if ((direction != -1) && (direction != 1)) {
		return;
	}
	drive_marker_slow_until_step = average_step
			+ FIRST_DRIVE_MARKER_SLOW_STEPS;

	if (current_direction != direction) {
		next_phase = (direction < 0)
				? FIRST_DRIVE_COURSE_APPROACH_LEFT
				: FIRST_DRIVE_COURSE_APPROACH_RIGHT;
	} else if ((drive_course_phase == FIRST_DRIVE_COURSE_TURN_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_TURN_RIGHT)) {
		next_phase = (direction < 0)
				? FIRST_DRIVE_COURSE_EXIT_LEFT
				: FIRST_DRIVE_COURSE_EXIT_RIGHT;
		transition_reason = FIRST_DRIVE_PHASE_REASON_MARKER_EXIT;
	} else if ((drive_course_phase == FIRST_DRIVE_COURSE_EXIT_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_EXIT_RIGHT)) {
		next_phase = FIRST_DRIVE_COURSE_STRAIGHT;
		transition_reason = FIRST_DRIVE_PHASE_REASON_MARKER_STRAIGHT;
	} else {
		/* The completed Track event repeats the provisional edge hit. */
		return;
	}
	FirstDrive_SetCoursePhase(next_phase, average_step, transition_reason);
}

static void FirstDrive_FilterMarkerSpill(SensorLineMeasurement_t *line,
		int32_t reference_position)
{
	uint8_t strict_edges;
	uint8_t spill_mask = 0U;
	uint8_t effective_mask;

	if (line == NULL) {
		return;
	}
	strict_edges = line->marker_mask & SENSOR_MARKER_MASK;
	effective_mask = line->raw_line_mask;

	/* Direction tape can cover S1/S2 or S5/S6 as well as the dedicated outer
	 * sensor.  Quarantine that side only when tracking history is central and
	 * another central inner response exists.  A real curve already tracked at
	 * the same outer side must remain available to posture control. */
	if (FirstDrive_AbsolutePosition(reference_position)
			<= FIRST_DRIVE_MARKER_HISTORY_POS) {
		if (((strict_edges & 0x01U) != 0U)
				&& ((effective_mask & 0x38U) != 0U)) {
			spill_mask |= 0x06U;
		}
		if (((strict_edges & 0x80U) != 0U)
				&& ((effective_mask & 0x1CU) != 0U)) {
			spill_mask |= 0x60U;
		}
	}

	if (spill_mask != 0U) {
		effective_mask &= (uint8_t)~spill_mask;
		(void)Sensor_UseLineMask(line, effective_mask, spill_mask);
	}
}

static bool FirstDrive_SelectTrackedCluster(SensorLineMeasurement_t *line)
{
	uint32_t best_distance = UINT32_MAX;
	uint8_t best_index = 0U;
	bool found = false;
	uint8_t index;

	if (line == NULL) {
		return false;
	}
	for (index = 0U; index < line->cluster_count; index++) {
		const SensorLineCluster_t *cluster = &line->clusters[index];
		uint32_t distance;

		if (cluster->strength < SENSOR_LINE_STRENGTH_MIN) {
			continue;
		}
		distance = FirstDrive_PositionDistance(cluster->position,
				drive_last_position);
		if (!drive_line_lost_active
				&& (distance > FIRST_DRIVE_CLUSTER_MAX_JUMP)) {
			continue;
		}
		if (!found || (distance < best_distance)
				|| ((distance == best_distance)
						&& (cluster->strength
								> line->clusters[best_index].strength))) {
			best_index = index;
			best_distance = distance;
			found = true;
		}
	}

	if (!found || !Sensor_UseLineCluster(line, best_index)) {
		line->position = (int16_t)drive_last_position;
		line->strength = 0U;
		line->selected_mask = 0U;
		line->center_count = 0U;
		line->line_valid = false;
		return false;
	}
	return true;
}

static void FirstDrive_UpdateBridgeRecovery(
		const SensorLineMeasurement_t *line)
{
	uint8_t outer_edges;
	int8_t candidate = 0;

	if (line == NULL) {
		drive_bridge_recovery_direction = 0;
		return;
	}
	if (line->line_valid) {
		/* S1..S6 has reacquired the line.  S0/S7 never supplies a position. */
		drive_bridge_recovery_direction = 0;
		return;
	}

	outer_edges = line->marker_mask & SENSOR_MARKER_MASK;
	if ((outer_edges == 0x01U)
			&& (drive_last_position <= -FIRST_DRIVE_BRIDGE_ENTER_POS)) {
		candidate = -1;
	} else if ((outer_edges == 0x80U)
			&& (drive_last_position >= FIRST_DRIVE_BRIDGE_ENTER_POS)) {
		candidate = 1;
	}

	/* Both outer sensors, the opposite sensor, or no outer evidence returns to
	 * the normal hard safety window immediately. */
	drive_bridge_recovery_direction = candidate;
}

static uint8_t FirstDrive_GetMarkerEdges(
		const SensorLineMeasurement_t *line)
{
	uint8_t marker_edges;

	if ((line == NULL) || !line->line_valid
			|| (drive_bridge_recovery_direction != 0)) {
		return 0U;
	}

	marker_edges = line->marker_mask & SENSOR_MARKER_MASK;
	/* A direction marker must be beside a simultaneously visible, central
	 * S1..S6 running line.  An S0/S7 response reached from outer S1/S6 is the
	 * main line crossing the PCB's sensor-free bridge, not a marker. */
	if ((FirstDrive_AbsolutePosition(line->position)
			> FIRST_DRIVE_MARKER_CENTER_POS)
			|| ((line->selected_mask & FIRST_DRIVE_MARKER_CENTER_MASK) == 0U)) {
		return 0U;
	}
	return marker_edges;
}

static void FirstDrive_SetProvisionalMarker(int8_t direction,
		uint32_t average_step)
{
	drive_provisional_marker_direction = direction;
	if (direction != 0) {
		drive_provisional_marker_step = average_step;
	}
	drive_telemetry.provisional_marker_direction = direction;
}

static void FirstDrive_ProcessMarker(const SensorLineMeasurement_t *line,
		uint32_t frame_number, uint32_t average_step)
{
	uint8_t marker_mask;
	uint8_t marker_edges;
	const TrackMarkerEvent_t *event;
	bool left_marker_now;
	bool right_marker_now;
	int8_t event_direction = 0;

	if ((drive_provisional_marker_direction != 0)
			&& !FirstDrive_StepBefore(average_step,
					drive_provisional_marker_step
							+ FIRST_DRIVE_MARKER_SLOW_STEPS)) {
		FirstDrive_SetProvisionalMarker(0, average_step);
	}

	marker_edges = FirstDrive_GetMarkerEdges(line);
	/* S0/S7 are marker-only; selected_mask contains S1..S6 exclusively. */
	marker_mask = (line->selected_mask & (uint8_t)~0x81U) | marker_edges;

	/* Do not wait until the complete 5 cm marker has passed.  Three consecutive
	 * frames of an isolated edge beside a still-valid centre line are enough to
	 * begin braking and publish the expected direction.  Track_ProcessSensor()
	 * continues collecting the full event for debouncing and diagnostics. */
	left_marker_now = line->line_valid && !line->edge_only
			&& ((marker_edges & 0x01U) != 0U)
			&& ((marker_edges & 0x80U) == 0U);
	right_marker_now = line->line_valid && !line->edge_only
			&& ((marker_edges & 0x80U) != 0U)
			&& ((marker_edges & 0x01U) == 0U);
	if (left_marker_now) {
		if (drive_marker_left_frames < UINT8_MAX) {
			drive_marker_left_frames++;
		}
	} else {
		drive_marker_left_frames = 0U;
	}
	if (right_marker_now) {
		if (drive_marker_right_frames < UINT8_MAX) {
			drive_marker_right_frames++;
		}
	} else {
		drive_marker_right_frames = 0U;
	}
	if (drive_marker_left_frames == TRACK_MARK_CONFIRM_FRAMES) {
		FirstDrive_HandleDirectionalMarker(-1, average_step,
				FIRST_DRIVE_PHASE_REASON_MARKER_PROVISIONAL);
		FirstDrive_SetProvisionalMarker(-1, average_step);
	} else if (drive_marker_right_frames == TRACK_MARK_CONFIRM_FRAMES) {
		FirstDrive_HandleDirectionalMarker(1, average_step,
				FIRST_DRIVE_PHASE_REASON_MARKER_PROVISIONAL);
		FirstDrive_SetProvisionalMarker(1, average_step);
	}

	if ((drive_run_mode == DRIVE_RUN_FIRST)
			? !Track_ProcessSensor(marker_mask, frame_number, average_step)
			: !Track_ProcessReplaySensor(marker_mask, frame_number, average_step)) {
		return;
	}
	event = (drive_run_mode == DRIVE_RUN_FIRST)
			? Track_GetLastEvent() : Track_GetLastReplayEvent();
	if (drive_run_mode == DRIVE_RUN_FIRST) {
		drive_telemetry.event_count = Track_GetEventCount();
	} else {
		drive_telemetry.event_count++;
	}
	if (event != NULL) {
		drive_telemetry.last_marker_type = (uint8_t)event->type;
		drive_telemetry.last_marker_confidence = event->confidence;
		drive_telemetry.last_marker_edge_union = event->edge_union;
		FirstDrive_RecordMarkerEvent(event);
	}
	if ((event == NULL) || (event->confidence < 20U)) {
		return;
	}
	if (drive_run_mode == DRIVE_RUN_SECOND) {
		SecondDrivePlanner_OnEvent(event);
	}

	switch (event->type) {
	case MARKER_EVENT_EDGE_0:
		event_direction = -1;
		break;
	case MARKER_EVENT_EDGE_7:
		event_direction = 1;
		break;
	case MARKER_EVENT_CROSS:
		drive_cross_until_step = average_step
				+ FIRST_DRIVE_CROSS_PASS_STEPS;
		FirstDrive_SetCoursePhase(FIRST_DRIVE_COURSE_CROSS,
				average_step, FIRST_DRIVE_PHASE_REASON_CROSS_MARKER);
		FirstDrive_SetProvisionalMarker(0, average_step);
		break;
	case MARKER_EVENT_BOTH:
		FirstDrive_SetProvisionalMarker(0, average_step);
		/* The start and finish marks are bilateral.  Require real simultaneous
		 * S0/S7 overlap so two unrelated one-sided events cannot end the run. */
		if ((event->center_step >= FIRST_DRIVE_START_MARKER_IGNORE_STEPS)
				&& (event->both_overlap_run
						>= TRACK_MARKER_MIN_OVERLAP)
				&& (event->max_center_count
						< MARKER_WIDE_CENTER_COUNT)
				&& (event->wide_center_run < MARKER_WIDE_MIN_FRAMES)) {
			FirstDrive_StopAtEndMarker();
		}
		break;
	default:
		break;
	}

	if (event_direction != 0) {
		if (drive_provisional_marker_direction == event_direction) {
			/* This is the completed form of the already-applied early event.
			 * A confirmed entry marker also latches TURN even when feedforward
			 * kept the running line near the centre throughout APPROACH. */
			FirstDrive_SetProvisionalMarker(0, average_step);
			if (((event_direction < 0)
					&& (drive_course_phase
							== FIRST_DRIVE_COURSE_APPROACH_LEFT))
					|| ((event_direction > 0)
						&& (drive_course_phase
								== FIRST_DRIVE_COURSE_APPROACH_RIGHT))) {
				FirstDrive_SetCoursePhase((event_direction < 0)
						? FIRST_DRIVE_COURSE_TURN_LEFT
						: FIRST_DRIVE_COURSE_TURN_RIGHT,
						average_step,
						FIRST_DRIVE_PHASE_REASON_MARKER_CONFIRMED);
			}
		} else {
			FirstDrive_HandleDirectionalMarker(event_direction, average_step,
					FIRST_DRIVE_PHASE_REASON_MARKER_CONFIRMED);
		}
	}
}

static void FirstDrive_UpdateCourseState(int32_t position, bool line_valid,
		uint32_t average_step)
{
	uint32_t absolute_position = FirstDrive_AbsolutePosition(position);

	if (drive_course_phase == FIRST_DRIVE_COURSE_CROSS) {
		if (!FirstDrive_StepBefore(average_step, drive_cross_until_step)) {
			FirstDrive_SetCoursePhase(FIRST_DRIVE_COURSE_STRAIGHT,
					average_step,
					FIRST_DRIVE_PHASE_REASON_CROSS_EXPIRED);
		}
		return;
	}

	if ((drive_course_phase == FIRST_DRIVE_COURSE_APPROACH_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_APPROACH_RIGHT)) {
		if (line_valid
				&& (((drive_course_phase
						== FIRST_DRIVE_COURSE_APPROACH_LEFT)
						&& (position <= -FIRST_DRIVE_TURN_ENTER_POS))
					|| ((drive_course_phase
							== FIRST_DRIVE_COURSE_APPROACH_RIGHT)
							&& (position >= FIRST_DRIVE_TURN_ENTER_POS)))) {
			FirstDrive_SetCoursePhase((position < 0)
					? FIRST_DRIVE_COURSE_TURN_LEFT
					: FIRST_DRIVE_COURSE_TURN_RIGHT, average_step,
					FIRST_DRIVE_PHASE_REASON_POSITION_ENTER);
		} else if (!FirstDrive_StepBefore(average_step,
				drive_marker_slow_until_step)) {
			FirstDrive_SetCoursePhase(FIRST_DRIVE_COURSE_STRAIGHT,
					average_step,
					FIRST_DRIVE_PHASE_REASON_APPROACH_EXPIRED);
		}
		return;
	}

	if ((drive_course_phase == FIRST_DRIVE_COURSE_TURN_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_TURN_RIGHT)) {
		/* A well-followed long corner can keep the line centred for hundreds of
		 * milliseconds.  Centring therefore proves tracking quality, not corner
		 * completion.  Only the matching exit marker may release TURN. */
		drive_turn_center_frames = 0U;
		return;
	}

	if ((drive_course_phase == FIRST_DRIVE_COURSE_EXIT_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_EXIT_RIGHT)) {
		if (line_valid && (absolute_position <= FIRST_DRIVE_TURN_EXIT_POS)) {
			if (drive_turn_center_frames < UINT8_MAX) {
				drive_turn_center_frames++;
			}
		} else {
			drive_turn_center_frames = 0U;
		}
		if (drive_turn_center_frames >= FIRST_DRIVE_TURN_EXIT_FRAMES) {
			FirstDrive_SetCoursePhase(FIRST_DRIVE_COURSE_STRAIGHT,
					average_step,
					FIRST_DRIVE_PHASE_REASON_EXIT_CENTERED);
		}
		return;
	}
}

static bool FirstDrive_UpdateEdgeWatchdog(int32_t position, bool line_valid,
		uint32_t now)
{
	uint32_t absolute_position = FirstDrive_AbsolutePosition(position);
	int8_t side = (position < 0) ? -1 : 1;
	uint32_t dwell;
	uint32_t limit;
	bool confirmed_turn;

	if (!line_valid || (absolute_position <= FIRST_DRIVE_EDGE_RELEASE_POS)) {
		drive_edge_stuck_active = false;
		drive_edge_stuck_side = 0;
		drive_edge_best_position = 0U;
		drive_telemetry.edge_dwell_ms = 0U;
		return true;
	}
	if (!drive_edge_stuck_active) {
		if (absolute_position < FIRST_DRIVE_EDGE_STUCK_POS) {
			drive_telemetry.edge_dwell_ms = 0U;
			return true;
		}
		drive_edge_stuck_active = true;
		drive_edge_stuck_side = side;
		drive_edge_best_position = absolute_position;
		drive_edge_stuck_tick = now;
	} else if (side != drive_edge_stuck_side) {
		drive_edge_stuck_side = side;
		drive_edge_best_position = absolute_position;
		drive_edge_stuck_tick = now;
	} else if ((absolute_position + FIRST_DRIVE_EDGE_PROGRESS_POS)
			<= drive_edge_best_position) {
		/* Moving at least one fifth of a sensor interval back toward the
		 * centre is real recovery.  Restart the no-progress timer so a long
		 * 270-degree corner is not treated as a fixed edge merely because it
		 * legitimately spends time on S1/S6. */
		drive_edge_best_position = absolute_position;
		drive_edge_stuck_tick = now;
	}

	dwell = now - drive_edge_stuck_tick;
	drive_telemetry.edge_dwell_ms = (dwell > UINT16_MAX)
			? UINT16_MAX : (uint16_t)dwell;
	confirmed_turn = (drive_course_phase == FIRST_DRIVE_COURSE_TURN_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_TURN_RIGHT);
	/* A constant-radius corner can legitimately hold S1/S6 for longer than a
	 * fixed timer while the line remains valid and steering has the correct
	 * turn phase.  Keep measuring the dwell for diagnostics, but let the
	 * independent 120 ms TURN line-loss watchdog stop a real departure. */
	if (confirmed_turn) {
		return true;
	}
	limit = (FirstDrive_CourseDirection(drive_course_phase) != 0)
			? FIRST_DRIVE_EDGE_TRANSITION_FAULT_MS
			: FIRST_DRIVE_EDGE_STRAIGHT_FAULT_MS;
	if (dwell >= limit) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_EDGE_STUCK);
		return false;
	}
	return true;
}

static uint16_t FirstDrive_ClampSps(int32_t value)
{
	uint16_t floor_sps = (drive_current_base_sps
			< first_drive_config.minimum_sps)
			? drive_current_base_sps : first_drive_config.minimum_sps;
	uint16_t maximum_sps = (drive_run_mode == DRIVE_RUN_SECOND)
			? MOTOR_DRIVE_MAX_SPS : first_drive_config.maximum_sps;

	if (value < (int32_t)floor_sps) {
		return floor_sps;
	}
	if (value > (int32_t)maximum_sps) {
		return maximum_sps;
	}
	return (uint16_t)value;
}

static uint16_t FirstDrive_GetTargetBaseSps(int32_t position,
		uint32_t average_step)
{
	uint32_t absolute_position;
	uint16_t floor_sps;
	uint32_t reduction;
	uint16_t result;

	absolute_position = FirstDrive_AbsolutePosition(position);
	if (absolute_position > (uint32_t)SENSOR_LINE_POSITION_MAX) {
		absolute_position = (uint32_t)SENSOR_LINE_POSITION_MAX;
	}
	floor_sps = (first_drive_config.base_sps < FIRST_DRIVE_CURVE_MIN_SPS)
			? first_drive_config.base_sps : FIRST_DRIVE_CURVE_MIN_SPS;
	if (first_drive_config.base_sps <= floor_sps) {
		result = first_drive_config.base_sps;
	} else {
		/* At the centre keep the launch/base speed.  At the outermost sensor,
		 * lower the common speed toward the corner envelope before applying the
		 * PD differential. */
		reduction = ((uint32_t)(first_drive_config.base_sps - floor_sps)
				* absolute_position) / (uint32_t)SENSOR_LINE_POSITION_MAX;
		result = (uint16_t)(first_drive_config.base_sps - reduction);
	}

	if (FirstDrive_StepBefore(average_step, drive_marker_slow_until_step)
			&& (result > FIRST_DRIVE_MARKER_SLOW_SPS)) {
		result = FIRST_DRIVE_MARKER_SLOW_SPS;
	}
	if (((drive_course_phase == FIRST_DRIVE_COURSE_APPROACH_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_APPROACH_RIGHT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_TURN_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_TURN_RIGHT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_EXIT_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_EXIT_RIGHT))
			&& (result > FIRST_DRIVE_TURN_MAX_SPS)) {
		result = FIRST_DRIVE_TURN_MAX_SPS;
	}
	if ((drive_course_phase == FIRST_DRIVE_COURSE_CROSS)
			&& (result > FIRST_DRIVE_MARKER_SLOW_SPS)) {
		result = FIRST_DRIVE_MARKER_SLOW_SPS;
	}
	if (drive_run_mode == DRIVE_RUN_SECOND) {
		result = SecondDrivePlanner_GetTargetSps(result, position,
				drive_course_phase, average_step, drive_current_base_sps);
	}
	return result;
}

static uint16_t FirstDrive_GetLineLostLimitMs(void)
{
	uint32_t absolute_position = FirstDrive_AbsolutePosition(
			drive_last_position);

	/* This longer window is available only while the line disappears from
	 * S1/S6 and continues on the matching dedicated outer sensor. */
	if (drive_bridge_recovery_direction != 0) {
		return FIRST_DRIVE_LINE_LOST_OUTER_EVIDENCE_MS;
	}

	/* The PCB has a real no-sensor bridge between the central S1..S6 array and
	 * each outer marker sensor.  Entering it from an outer inner sensor is a
	 * planned blind transfer: keep the last steering slightly longer, but stay
	 * below the independent 150 ms unstable-line safety limit. */
	if (absolute_position >= FIRST_DRIVE_BRIDGE_ENTER_POS) {
		return FIRST_DRIVE_LINE_LOST_BRIDGE_MS;
	}
	if ((FirstDrive_CourseDirection(drive_course_phase) != 0)
			|| (absolute_position >= FIRST_DRIVE_EDGE_RELEASE_POS)) {
		return FIRST_DRIVE_LINE_LOST_TURN_MS;
	}
	if (absolute_position <= FIRST_DRIVE_TURN_ENTER_POS) {
		return FIRST_DRIVE_LINE_LOST_STRAIGHT_MS;
	}
	return FIRST_DRIVE_LINE_LOST_TRANSITION_MS;
}

static int32_t FirstDrive_GetTurnFeedforwardQ15(int32_t position)
{
	int8_t direction = FirstDrive_CourseDirection(drive_course_phase);
	int32_t magnitude = 0;

	if ((drive_course_phase == FIRST_DRIVE_COURSE_APPROACH_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_APPROACH_RIGHT)) {
		magnitude = FIRST_DRIVE_APPROACH_FF_Q15;
	} else if (((drive_course_phase == FIRST_DRIVE_COURSE_TURN_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_TURN_RIGHT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_EXIT_LEFT)
			|| (drive_course_phase == FIRST_DRIVE_COURSE_EXIT_RIGHT))
			&& (((direction < 0) && (position < -FIRST_DRIVE_TURN_EXIT_POS))
					|| ((direction > 0)
							&& (position > FIRST_DRIVE_TURN_EXIT_POS)))) {
		magnitude = FIRST_DRIVE_TURN_FF_Q15;
	}

	/* Positive steer makes the chassis turn left; course direction uses
	 * -1 for left and +1 for right. */
	return -(int32_t)direction * magnitude;
}

static bool FirstDrive_UpdateMotorCommand(const SensorLineMeasurement_t *line,
		uint32_t now, uint32_t average_step)
{
	int32_t position;
	int32_t control_position;
	int32_t derivative = 0;
	int32_t p_term = drive_telemetry.p_term;
	int32_t d_term = drive_telemetry.d_term;
	int32_t target_ratio_q15;
	int32_t steer;
	int32_t centre_sps;
	int32_t applied_trim;
	int32_t left;
	int32_t right;
	uint16_t target_centre_sps;
	uint16_t lost_limit_ms;
	uint16_t unstable_limit_ms = FIRST_DRIVE_LINE_UNSTABLE_FAULT_MS;
	int32_t steer_limit_q15 = drive_outer_boost_active
			? FIRST_DRIVE_STEER_RATIO_RECOVERY_Q15
			: FIRST_DRIVE_STEER_RATIO_NORMAL_Q15;
	uint32_t lost_ms = 0U;
	bool recovery_slow;

	if (line->line_valid) {
		/* Any trustworthy inner/continuous-edge position breaks a hard
		 * consecutive-loss interval.  Five such frames are still required to
		 * leave the slower recovery state. */
		drive_line_invalid_active = false;
		drive_line_invalid_tick = 0U;
		drive_line_invalid_limit_ms = 0U;
		if ((drive_state == FIRST_DRIVE_CROSS_PASS)
				&& (line->center_count >= MARKER_WIDE_CENTER_COUNT)) {
			/* At an intersection the rule is straight ahead.  Do not let the
			 * horizontal branch pull the weighted position away from the incoming
			 * heading. */
			position = drive_last_position;
			control_position = drive_last_control_position;
			derivative = 0;
		} else {
			position = line->position;
			control_position = FirstDrive_ApplyPositionDeadband(position);
			/* Do not create a derivative kick on the first few frames after a
			 * recovery.  P immediately uses the reacquired position while D settles. */
			derivative = drive_line_lost_active ? 0
					: (control_position - drive_last_control_position);
		}
		if (derivative > FIRST_DRIVE_DERIVATIVE_LIMIT) {
			derivative = FIRST_DRIVE_DERIVATIVE_LIMIT;
		} else if (derivative < -FIRST_DRIVE_DERIVATIVE_LIMIT) {
			derivative = -FIRST_DRIVE_DERIVATIVE_LIMIT;
		}
		drive_filtered_derivative += (derivative
				- drive_filtered_derivative) >> FIRST_DRIVE_D_FILTER_SHIFT;
		drive_last_position = position;
		drive_last_control_position = control_position;
		drive_telemetry.last_valid_position = (int16_t)position;
		drive_telemetry.last_valid_sensor_mask = line->state_mask;
		drive_telemetry.last_valid_line_mask = line->selected_mask;

		if (drive_line_lost_active) {
			if (drive_line_reacquire_frames < UINT8_MAX) {
				drive_line_reacquire_frames++;
			}
			if (drive_line_reacquire_frames
					>= FIRST_DRIVE_REACQUIRE_FRAMES) {
				drive_line_lost_active = false;
				drive_line_lost_frames = 0U;
				drive_line_reacquire_frames = 0U;
			}
		} else {
			drive_line_lost_frames = 0U;
			drive_line_reacquire_frames = 0U;
		}
	} else {
		position = drive_last_position;
		control_position = drive_last_control_position;
		/* A derivative and a new steering target do not exist without a new line
		 * position.  Hold the last normalized steering ratio continuously. */
		if (!drive_line_lost_active) {
			drive_line_lost_active = true;
			drive_line_lost_tick = now;
			drive_line_lost_frames = 0U;
		}
		if (!drive_line_invalid_active) {
			drive_line_invalid_active = true;
			drive_line_invalid_tick = now;
			/* Freeze the safety allowance at the start of this consecutive
			 * invalid interval.  A later course-phase transition must not
			 * retroactively shorten an already-running recovery window. */
			drive_line_invalid_limit_ms = FirstDrive_GetLineLostLimitMs();
		}
		if (drive_line_lost_frames < UINT16_MAX) {
			drive_line_lost_frames++;
		}
		drive_line_reacquire_frames = 0U;
		drive_telemetry.lost_count++;
	}

	if (!line->line_valid && (drive_bridge_recovery_direction != 0)) {
		unstable_limit_ms = FIRST_DRIVE_LINE_LOST_OUTER_EVIDENCE_MS;
		if (drive_line_invalid_active) {
			/* S0/S7 can appear a few frames after S1/S6 disappears because the
			 * PCB bridge itself has no sensor.  Promote the already-running blind
			 * interval once matching outer evidence arrives. */
			drive_line_invalid_limit_ms =
					FIRST_DRIVE_LINE_LOST_OUTER_EVIDENCE_MS;
		}
	} else if (drive_line_invalid_active
			&& (drive_line_invalid_limit_ms
					> FIRST_DRIVE_LINE_UNSTABLE_FAULT_MS)) {
		/* Outer evidence disappeared.  Do not retain the extended allowance
		 * merely because it was present at the first invalid frame. */
		drive_line_invalid_limit_ms = FIRST_DRIVE_LINE_UNSTABLE_FAULT_MS;
	}

	FirstDrive_UpdateCourseState(position, line->line_valid, average_step);
	lost_limit_ms = drive_line_invalid_active
			? drive_line_invalid_limit_ms : FirstDrive_GetLineLostLimitMs();
	if (drive_line_invalid_active) {
		lost_ms = now - drive_line_invalid_tick;
	}
	drive_telemetry.line_lost_ms = (lost_ms > UINT16_MAX)
			? UINT16_MAX : (uint16_t)lost_ms;
	drive_telemetry.line_lost_limit_ms = lost_limit_ms;
	if ((drive_line_invalid_active
			&& (lost_ms >= lost_limit_ms))
			|| (drive_line_lost_active
					&& ((now - drive_line_lost_tick)
							>= unstable_limit_ms))) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_LINE_LOST);
		return false;
	}
	recovery_slow = drive_line_lost_active
			&& ((now - drive_line_lost_tick)
					>= FIRST_DRIVE_LINE_LOST_GRACE_MS);
	if (!FirstDrive_UpdateEdgeWatchdog(position, line->line_valid, now)) {
		return false;
	}

	if (line->line_valid) {
		uint32_t absolute_control_position = FirstDrive_AbsolutePosition(
				control_position);

		if (absolute_control_position >= FIRST_DRIVE_OUTER_BOOST_ENTER_POS) {
			if (drive_outer_boost_frames < UINT16_MAX) {
				drive_outer_boost_frames++;
			}
		} else if (absolute_control_position
				<= FIRST_DRIVE_OUTER_BOOST_EXIT_POS) {
			drive_outer_boost_frames = 0U;
			drive_outer_boost_active = false;
		}
		if (drive_outer_boost_frames >= FIRST_DRIVE_OUTER_BOOST_FRAMES) {
			drive_outer_boost_active = true;
		}
		if (drive_outer_boost_active) {
			steer_limit_q15 = FIRST_DRIVE_STEER_RATIO_RECOVERY_Q15;
		}

		p_term = (first_drive_config.kp_q10 * control_position) / 1024;
		if (FirstDrive_AbsolutePosition(control_position)
				>= FIRST_DRIVE_OUTER_GAIN_POS) {
			p_term = (p_term * 5) / 4;
		}
		d_term = (first_drive_config.kd_q10
				* drive_filtered_derivative) / 1024;
		target_ratio_q15 = (((p_term + d_term) * FIRST_DRIVE_STEER_SIGN)
				* FIRST_DRIVE_STEER_RATIO_Q15)
				/ (int32_t)first_drive_config.base_sps;
		target_ratio_q15 += FirstDrive_GetTurnFeedforwardQ15(position);
		if (drive_outer_boost_active) {
			int32_t recovery_target = (control_position < 0)
					? FIRST_DRIVE_STEER_RATIO_RECOVERY_Q15
					: -FIRST_DRIVE_STEER_RATIO_RECOVERY_Q15;

			/* The failure log already showed the normal 0.45 target saturated.
			 * Raising only the clamp would therefore change nothing. */
			if (((recovery_target > 0) && (target_ratio_q15 < recovery_target))
					|| ((recovery_target < 0)
							&& (target_ratio_q15 > recovery_target))) {
				target_ratio_q15 = recovery_target;
			}
		}
		if (target_ratio_q15 > steer_limit_q15) {
			target_ratio_q15 = steer_limit_q15;
		} else if (target_ratio_q15
				< -steer_limit_q15) {
			target_ratio_q15 = -steer_limit_q15;
		}
		drive_target_steer_q15 = target_ratio_q15;
	} else if (drive_bridge_recovery_direction != 0) {
		steer_limit_q15 = FIRST_DRIVE_STEER_RATIO_RECOVERY_Q15;
		drive_outer_boost_active = true;
		drive_target_steer_q15 = (drive_bridge_recovery_direction < 0)
				? FIRST_DRIVE_STEER_RATIO_RECOVERY_Q15
				: -FIRST_DRIVE_STEER_RATIO_RECOVERY_Q15;
	}

	target_centre_sps = FirstDrive_GetTargetBaseSps(position, average_step);
	if (recovery_slow) {
		uint16_t recovery_limit =
				((FirstDrive_CourseDirection(drive_course_phase) != 0)
						|| (FirstDrive_AbsolutePosition(position)
								>= FIRST_DRIVE_EDGE_RELEASE_POS))
				? FIRST_DRIVE_RECOVERY_TURN_SPS
				: FIRST_DRIVE_RECOVERY_STRAIGHT_SPS;

		if (target_centre_sps > recovery_limit) {
			target_centre_sps = recovery_limit;
		}
	}
	centre_sps = FirstDrive_RampCommonSpeed(target_centre_sps);
	drive_applied_steer_q15 = FirstDrive_SlewValue(
			drive_applied_steer_q15, drive_target_steer_q15,
			FIRST_DRIVE_STEER_RATIO_SLEW_Q15);
	steer = (drive_applied_steer_q15 * centre_sps)
			/ FIRST_DRIVE_STEER_RATIO_Q15;
	/* The menu trim is specified at straight-line base speed.  Scale it with
	 * common speed so it remains a wheel-bias correction instead of dominating
	 * a slow recovery or tight corner. */
	applied_trim = ((int32_t)first_drive_config.trim_sps * centre_sps)
			/ (int32_t)first_drive_config.base_sps;
	left = centre_sps - steer + applied_trim;
	right = centre_sps + steer - applied_trim;
	/* Clamp each wheel independently.  The previous pair correction raised
	 * the slow wheel back to 4800 SPS and erased the corner response. */

	drive_telemetry.left_sps = FirstDrive_ClampSps(left);
	drive_telemetry.right_sps = FirstDrive_ClampSps(right);
	drive_telemetry.line_position = line->line_valid
			? line->position : (int16_t)drive_last_position;
	drive_telemetry.line_valid = line->line_valid ? 1U : 0U;
	drive_telemetry.recovering = drive_line_lost_active ? 1U : 0U;
	drive_telemetry.edge_only = line->edge_only ? 1U : 0U;
	drive_telemetry.reacquire_count = drive_line_reacquire_frames;
	drive_telemetry.marker_direction = drive_expected_turn;
	drive_telemetry.lost_streak = drive_line_lost_frames;
	drive_telemetry.line_strength = line->strength;
	drive_telemetry.p_term = p_term;
	drive_telemetry.d_term = d_term;
	drive_telemetry.steer = steer;
	drive_telemetry.centre_sps = (uint16_t)centre_sps;
	drive_telemetry.target_centre_sps = target_centre_sps;
	drive_telemetry.target_steer_permille = (int16_t)(
			(drive_target_steer_q15 * 1000)
			/ FIRST_DRIVE_STEER_RATIO_Q15);
	drive_telemetry.applied_steer_permille = (int16_t)(
			(drive_applied_steer_q15 * 1000)
			/ FIRST_DRIVE_STEER_RATIO_Q15);
	drive_telemetry.bridge_recovery_direction =
			drive_bridge_recovery_direction;
	drive_telemetry.outer_boost_active = drive_outer_boost_active ? 1U : 0U;
	drive_telemetry.steer_limit_permille = (uint16_t)(
			(steer_limit_q15 * 1000) / FIRST_DRIVE_STEER_RATIO_Q15);
	drive_telemetry.course_phase = drive_course_phase;
	if ((drive_state == FIRST_DRIVE_LAUNCH)
			&& (drive_current_base_sps >= target_centre_sps)
			&& (drive_course_phase == FIRST_DRIVE_COURSE_STRAIGHT)) {
		drive_state = FIRST_DRIVE_FOLLOW;
	}

	if (!Motor_DriveSetSpeeds(drive_telemetry.left_sps,
			drive_telemetry.right_sps)) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_MOTOR_COMMAND);
		return false;
	}
	return true;
}

static bool FirstDrive_LaunchAfterCountdown(uint32_t now)
{
	SensorLineMeasurement_t line;

	if (!Sensor_GetLineMeasurement(&line)) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_SENSOR_STALE);
		return false;
	}
	FirstDrive_FilterMarkerSpill(&line, drive_last_position);
	(void)FirstDrive_SelectTrackedCluster(&line);
	if (!line.line_valid) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_START_NO_LINE);
		return false;
	}

	Motor_DriveResetStepCounts();
	drive_last_frame = Sensor_GetFrameCount();
	drive_current_base_sps = FIRST_DRIVE_MOTOR_START_SPS;
	drive_last_position = line.position;
	drive_last_control_position = FirstDrive_ApplyPositionDeadband(
			line.position);
	drive_filtered_derivative = 0;
	drive_target_steer_q15 = 0;
	drive_applied_steer_q15 = 0;
	drive_bridge_recovery_direction = 0;
	drive_outer_boost_frames = 0U;
	drive_outer_boost_active = false;
	drive_line_lost_frames = 0U;
	drive_line_reacquire_frames = 0U;
	drive_line_lost_tick = 0U;
	drive_line_lost_active = false;
	drive_line_invalid_tick = 0U;
	drive_line_invalid_limit_ms = 0U;
	drive_line_invalid_active = false;
	drive_sensor_stale_frames = 0U;
	drive_timer_divider = 0U;
	drive_edge_stuck_tick = 0U;
	drive_edge_best_position = 0U;
	drive_edge_stuck_active = false;
	drive_edge_stuck_side = 0;
	drive_telemetry.edge_dwell_ms = 0U;

	if (!Motor_DriveStart(FIRST_DRIVE_MOTOR_START_SPS,
			FIRST_DRIVE_MOTOR_START_SPS, FIRST_DRIVE_VREF_DAC)) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_MOTOR_COMMAND);
		return false;
	}

	drive_start_tick = now;
	drive_state = FIRST_DRIVE_LAUNCH;
	drive_telemetry.state = FIRST_DRIVE_LAUNCH;
	drive_telemetry.countdown_ms = 0U;
	drive_telemetry.line_position = line.position;
	drive_telemetry.sensor_mask = line.state_mask;
	drive_telemetry.raw_line_mask = line.raw_line_mask;
	drive_telemetry.line_mask = line.selected_mask;
	drive_telemetry.marker_spill_mask = line.spill_mask;
	drive_telemetry.cluster_count = line.cluster_count;
	drive_telemetry.center_count = line.center_count;
	drive_telemetry.line_valid = 1U;
	drive_telemetry.line_strength = line.strength;
	drive_telemetry.last_valid_position = line.position;
	drive_telemetry.last_valid_sensor_mask = line.state_mask;
	drive_telemetry.last_valid_line_mask = line.selected_mask;
	drive_telemetry.course_phase = drive_course_phase;
	drive_telemetry.base_sps = drive_current_base_sps;
	drive_telemetry.centre_sps = drive_current_base_sps;
	drive_telemetry.target_centre_sps = first_drive_config.base_sps;
	drive_watchdog_irq_count = drive_control_irq_count;
	drive_watchdog_tick = now;
	drive_control_enabled = true;
	return true;
}

static void FirstDrive_UpdateCountdown(uint32_t now)
{
	SensorLineMeasurement_t line;
	uint32_t frame_number = Sensor_GetFrameCount();
	uint32_t elapsed = now - drive_countdown_tick;

	if (frame_number != drive_last_frame) {
		drive_last_frame = frame_number;
		drive_countdown_frame_tick = now;
		if (!Sensor_GetLineMeasurement(&line)) {
			FirstDrive_SetFault(FIRST_DRIVE_FAULT_SENSOR_STALE);
			return;
		}
		FirstDrive_FilterMarkerSpill(&line, drive_last_position);
		(void)FirstDrive_SelectTrackedCluster(&line);
		drive_telemetry.line_position = line.position;
		drive_telemetry.sensor_mask = line.state_mask;
		drive_telemetry.raw_line_mask = line.raw_line_mask;
		drive_telemetry.line_mask = line.selected_mask;
		drive_telemetry.marker_spill_mask = line.spill_mask;
		drive_telemetry.cluster_count = line.cluster_count;
		drive_telemetry.center_count = line.center_count;
		drive_telemetry.line_valid = line.line_valid ? 1U : 0U;
		drive_telemetry.edge_only = line.edge_only ? 1U : 0U;
		drive_telemetry.line_strength = line.strength;
		if (line.line_valid) {
			drive_last_position = line.position;
			drive_last_control_position = FirstDrive_ApplyPositionDeadband(
					line.position);
			drive_telemetry.last_valid_position = line.position;
			drive_telemetry.last_valid_sensor_mask = line.state_mask;
			drive_telemetry.last_valid_line_mask = line.selected_mask;
		}
	} else if ((now - drive_countdown_frame_tick)
			>= FIRST_DRIVE_COUNTDOWN_STALE_MS) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_SENSOR_STALE);
		return;
	}

	drive_telemetry.countdown_ms = (elapsed < FIRST_DRIVE_START_DELAY_MS)
			? (uint16_t)(FIRST_DRIVE_START_DELAY_MS - elapsed) : 0U;
	if (elapsed >= FIRST_DRIVE_START_DELAY_MS) {
		(void)FirstDrive_LaunchAfterCountdown(now);
	}
}

static void FirstDrive_ProcessNewFrame(uint32_t frame_number,
		uint32_t average_step, uint32_t now)
{
	SensorLineMeasurement_t line;

	if (!Sensor_GetLineMeasurement(&line)) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_SENSOR_STALE);
		return;
	}

	drive_sensor_stale_frames = 0U;
	FirstDrive_FilterMarkerSpill(&line, drive_last_position);
	(void)FirstDrive_SelectTrackedCluster(&line);
	FirstDrive_UpdateBridgeRecovery(&line);
	drive_telemetry.sensor_mask = line.state_mask;
	drive_telemetry.raw_line_mask = line.raw_line_mask;
	drive_telemetry.line_mask = line.selected_mask;
	drive_telemetry.marker_spill_mask = line.spill_mask;
	drive_telemetry.cluster_count = line.cluster_count;
	drive_telemetry.center_count = line.center_count;
	drive_telemetry.average_steps = average_step;
	FirstDrive_ProcessMarker(&line, frame_number, average_step);
	if (!drive_control_enabled) {
		return;
	}
	if (Track_HasOverflow()) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_TRACK_OVERFLOW);
		return;
	}

	if (!FirstDrive_UpdateMotorCommand(&line, now, average_step)) {
		return;
	}
}

static void FirstDrive_ControlTick(void)
{
	uint32_t frame_number;
	uint32_t average_step;
	uint32_t now = HAL_GetTick();

	if (!drive_control_enabled) {
		return;
	}

	frame_number = Sensor_GetFrameCount();
	average_step = Motor_DriveGetAverageSteps();
	drive_telemetry.average_steps = average_step;
	drive_telemetry.elapsed_ms = now - drive_start_tick;

	if (frame_number == drive_last_frame) {
		drive_sensor_stale_frames++;
		if (drive_sensor_stale_frames >= FIRST_DRIVE_SENSOR_STALE_FAULT) {
			FirstDrive_SetFault(FIRST_DRIVE_FAULT_SENSOR_STALE);
		}
		return;
	}
	drive_sensor_stale_frames = 0U;
	drive_last_frame = frame_number;

	if (FirstDrive_IsControlState(drive_state)) {
		FirstDrive_ProcessNewFrame(frame_number, average_step, now);
	}
}

static bool Drive_InitCommon(DriveRunMode_t requested_mode)
{
	bool map_valid = (requested_mode == DRIVE_RUN_FIRST)
			|| Drive_HasCompletedMap();

	HAL_TIM_Base_Stop_IT(&htim7);
	Motor_DriveStop();
	Sensor_Stop();
	Sensor_StateReset();
	drive_run_mode = requested_mode;
	memset((void *)&drive_telemetry, 0, sizeof(drive_telemetry));
	drive_state = FIRST_DRIVE_READY;
	drive_fault = FIRST_DRIVE_FAULT_NONE;
	drive_control_enabled = false;
	drive_last_frame = 0U;
	drive_current_base_sps = 0U;
	drive_last_position = 0;
	drive_last_control_position = 0;
	drive_filtered_derivative = 0;
	drive_target_steer_q15 = 0;
	drive_applied_steer_q15 = 0;
	drive_bridge_recovery_direction = 0;
	drive_outer_boost_frames = 0U;
	drive_outer_boost_active = false;
	drive_line_lost_frames = 0U;
	drive_line_reacquire_frames = 0U;
	drive_line_lost_tick = 0U;
	drive_line_lost_active = false;
	drive_line_invalid_tick = 0U;
	drive_line_invalid_limit_ms = 0U;
	drive_line_invalid_active = false;
	drive_sensor_stale_frames = 0U;
	drive_timer_divider = 0U;
	drive_expected_turn = 0;
	drive_course_phase = FIRST_DRIVE_COURSE_STRAIGHT;
	drive_turn_center_frames = 0U;
	drive_marker_left_frames = 0U;
	drive_marker_right_frames = 0U;
	drive_provisional_marker_direction = 0;
	drive_provisional_marker_step = 0U;
	drive_marker_slow_until_step = 0U;
	drive_cross_until_step = 0U;
	drive_countdown_tick = 0U;
	drive_countdown_frame_tick = 0U;
	drive_edge_stuck_tick = 0U;
	drive_edge_best_position = 0U;
	drive_edge_stuck_active = false;
	drive_edge_stuck_side = 0;
	if (requested_mode == DRIVE_RUN_FIRST) {
		Track_Reset();
		drive_map_ready = false;
	} else {
		Track_ReplayReset();
		SecondDrivePlanner_Reset();
	}
	drive_control_irq_count = 0U;
	drive_control_tick_count = 0U;
	drive_watchdog_irq_count = 0U;
	drive_watchdog_tick = HAL_GetTick();
	drive_telemetry.state = FIRST_DRIVE_READY;
	drive_telemetry.fault = FIRST_DRIVE_FAULT_NONE;
	drive_telemetry.course_phase = FIRST_DRIVE_COURSE_STRAIGHT;
	if (!map_valid) {
		drive_state = FIRST_DRIVE_FAULT;
		drive_fault = FIRST_DRIVE_FAULT_NO_TRACK;
		drive_telemetry.state = FIRST_DRIVE_FAULT;
		drive_telemetry.fault = FIRST_DRIVE_FAULT_NO_TRACK;
		return false;
	}
	return true;
}

void FirstDrive_Init(void)
{
	(void)Drive_InitCommon(DRIVE_RUN_FIRST);
}

bool SecondDrive_Init(void)
{
	return Drive_InitCommon(DRIVE_RUN_SECOND);
}

static bool Drive_ArmCommon(DriveRunMode_t requested_mode)
{
	if ((drive_run_mode != requested_mode)
			|| !Sensor_IsCalibrationComplete()
			|| (drive_state != FIRST_DRIVE_READY)) {
		if (!Sensor_IsCalibrationComplete()) {
			drive_fault = FIRST_DRIVE_FAULT_NO_CALIBRATION;
		}
		return false;
	}

	drive_state = FIRST_DRIVE_ARMED;
	drive_telemetry.state = FIRST_DRIVE_ARMED;
	drive_telemetry.fault = FIRST_DRIVE_FAULT_NONE;
	return true;
}

bool FirstDrive_Arm(void)
{
	return Drive_ArmCommon(DRIVE_RUN_FIRST);
}

bool SecondDrive_Arm(void)
{
	if (!Drive_HasCompletedMap()) {
		drive_fault = FIRST_DRIVE_FAULT_NO_TRACK;
		drive_state = FIRST_DRIVE_FAULT;
		drive_telemetry.state = FIRST_DRIVE_FAULT;
		drive_telemetry.fault = FIRST_DRIVE_FAULT_NO_TRACK;
		return false;
	}
	return Drive_ArmCommon(DRIVE_RUN_SECOND);
}

static bool Drive_StartCommon(DriveRunMode_t requested_mode)
{
	SensorLineMeasurement_t initial_line;
	uint32_t startup_tick;
	uint32_t startup_irq_count;
	uint32_t first_frame;

	if ((drive_run_mode != requested_mode)
			|| (drive_state != FIRST_DRIVE_ARMED)
			|| !Sensor_IsCalibrationComplete()) {
		return false;
	}
	if ((requested_mode == DRIVE_RUN_SECOND) && !Drive_HasCompletedMap()) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_NO_TRACK);
		return false;
	}

	Motor_DriveStop();
	Sensor_Stop();
	Sensor_StateReset();
	Motor_DriveResetStepCounts();
	drive_last_frame = 0U;
	drive_current_base_sps = FIRST_DRIVE_MOTOR_START_SPS;
	drive_last_position = 0;
	drive_last_control_position = 0;
	drive_filtered_derivative = 0;
	drive_target_steer_q15 = 0;
	drive_applied_steer_q15 = 0;
	drive_bridge_recovery_direction = 0;
	drive_outer_boost_frames = 0U;
	drive_outer_boost_active = false;
	drive_line_lost_frames = 0U;
	drive_line_reacquire_frames = 0U;
	drive_line_lost_tick = 0U;
	drive_line_lost_active = false;
	drive_line_invalid_tick = 0U;
	drive_line_invalid_limit_ms = 0U;
	drive_line_invalid_active = false;
	drive_sensor_stale_frames = 0U;
	drive_control_enabled = false;
	drive_fault = FIRST_DRIVE_FAULT_NONE;
	/* Each run owns a fresh diagnostic snapshot.  Otherwise a fault that
	 * happens during startup can display masks, loss time or motor commands
	 * left over from the preceding run. */
	memset((void *)&drive_telemetry, 0, sizeof(drive_telemetry));
	drive_timer_divider = 0U;
	drive_control_irq_count = 0U;
	drive_control_tick_count = 0U;
	drive_expected_turn = 0;
	drive_course_phase = FIRST_DRIVE_COURSE_STRAIGHT;
	drive_turn_center_frames = 0U;
	drive_marker_left_frames = 0U;
	drive_marker_right_frames = 0U;
	drive_provisional_marker_direction = 0;
	drive_provisional_marker_step = 0U;
	drive_marker_slow_until_step = 0U;
	drive_cross_until_step = 0U;
	drive_countdown_tick = 0U;
	drive_countdown_frame_tick = 0U;
	drive_edge_stuck_tick = 0U;
	drive_edge_best_position = 0U;
	drive_edge_stuck_active = false;
	drive_edge_stuck_side = 0;
	if (requested_mode == DRIVE_RUN_FIRST) {
		Track_Reset();
		Track_SetStartIgnoreSteps(FIRST_DRIVE_START_MARKER_IGNORE_STEPS);
		drive_map_ready = false;
	} else {
		Track_ReplayReset();
		SecondDrivePlanner_Reset();
	}

	Sensor_SetLightMode(SENSOR_LIGHT_PAIR, 0U);
	Sensor_Start();
	if (!Sensor_IsRunning()) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_SENSOR_STALE);
		return false;
	}

	/* Prove that both the control interrupt path and the sensor pipeline are
	 * alive while the motors are still off.  HAL_TIM_Base_Start_IT() alone
	 * cannot detect a missing NVIC entry or IRQ handler. */
	__HAL_TIM_SET_COUNTER(&htim7, 0U);
	__HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
	startup_irq_count = drive_control_irq_count;
	startup_tick = HAL_GetTick();
	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_CONTROL_TIMER);
		return false;
	}
	do {
		first_frame = Sensor_GetFrameCount();
		if ((drive_control_irq_count != startup_irq_count)
				&& (first_frame != 0U)) {
			break;
		}
	} while ((HAL_GetTick() - startup_tick)
			< FIRST_DRIVE_STARTUP_TIMEOUT_MS);

	if (drive_control_irq_count == startup_irq_count) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_CONTROL_TIMER);
		return false;
	}
	if (first_frame == 0U) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_SENSOR_STALE);
		return false;
	}
	drive_last_frame = first_frame;
	if (!Sensor_GetLineMeasurement(&initial_line)) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_SENSOR_STALE);
		return false;
	}
	FirstDrive_FilterMarkerSpill(&initial_line, 0);
	(void)FirstDrive_SelectTrackedCluster(&initial_line);
	if (!initial_line.line_valid) {
		FirstDrive_SetFault(FIRST_DRIVE_FAULT_START_NO_LINE);
		return false;
	}
	drive_last_position = initial_line.position;
	drive_last_control_position = FirstDrive_ApplyPositionDeadband(
			initial_line.position);
	drive_filtered_derivative = 0;
	drive_telemetry.line_position = initial_line.position;
	drive_telemetry.sensor_mask = initial_line.state_mask;
	drive_telemetry.raw_line_mask = initial_line.raw_line_mask;
	drive_telemetry.line_mask = initial_line.selected_mask;
	drive_telemetry.marker_spill_mask = initial_line.spill_mask;
	drive_telemetry.cluster_count = initial_line.cluster_count;
	drive_telemetry.center_count = initial_line.center_count;
	drive_telemetry.line_valid = 1U;
	drive_telemetry.line_strength = initial_line.strength;
	drive_telemetry.last_valid_position = initial_line.position;
	drive_telemetry.last_valid_sensor_mask = initial_line.state_mask;
	drive_telemetry.last_valid_line_mask = initial_line.selected_mask;
	drive_telemetry.course_phase = drive_course_phase;

	drive_countdown_tick = HAL_GetTick();
	drive_countdown_frame_tick = drive_countdown_tick;
	drive_state = FIRST_DRIVE_COUNTDOWN;
	drive_telemetry.state = FIRST_DRIVE_COUNTDOWN;
	drive_telemetry.countdown_ms = FIRST_DRIVE_START_DELAY_MS;
	drive_telemetry.base_sps = drive_current_base_sps;
	drive_telemetry.centre_sps = drive_current_base_sps;
	drive_telemetry.target_centre_sps = first_drive_config.base_sps;
	drive_control_enabled = false;
	return true;
}

bool FirstDrive_Start(void)
{
	return Drive_StartCommon(DRIVE_RUN_FIRST);
}

bool SecondDrive_Start(void)
{
	return Drive_StartCommon(DRIVE_RUN_SECOND);
}

void FirstDrive_EmergencyStop(void)
{
	if ((drive_run_mode == DRIVE_RUN_FIRST)
			&& (drive_state != FIRST_DRIVE_STOPPED)) {
		drive_map_ready = false;
	}
	drive_control_enabled = false;
	HAL_TIM_Base_Stop_IT(&htim7);
	Motor_DriveStop();
	Sensor_Stop();
	if (drive_state != FIRST_DRIVE_FAULT) {
		drive_state = FIRST_DRIVE_STOPPED;
	}
	drive_telemetry.state = drive_state;
}

void FirstDrive_Stop(void)
{
	FirstDrive_EmergencyStop();
}

void SecondDrive_EmergencyStop(void)
{
	if (drive_run_mode == DRIVE_RUN_SECOND) {
		FirstDrive_EmergencyStop();
	}
}

void SecondDrive_Stop(void)
{
	SecondDrive_EmergencyStop();
}

void FirstDrive_Process(void)
{
	uint32_t now = HAL_GetTick();
	uint32_t irq_count;

	if (drive_state == FIRST_DRIVE_COUNTDOWN) {
		FirstDrive_UpdateCountdown(now);
	}
	if (drive_control_enabled) {
		irq_count = drive_control_irq_count;
		if (irq_count != drive_watchdog_irq_count) {
			drive_watchdog_irq_count = irq_count;
			drive_watchdog_tick = now;
		} else if ((now - drive_watchdog_tick)
				>= FIRST_DRIVE_CONTROL_WATCHDOG_MS) {
			FirstDrive_SetFault(FIRST_DRIVE_FAULT_CONTROL_TIMER);
		}
	}
	drive_telemetry.state = drive_state;
	drive_telemetry.fault = drive_fault;
	drive_telemetry.base_sps = drive_current_base_sps;
	drive_telemetry.average_steps = Motor_DriveGetAverageSteps();
	drive_telemetry.control_irq_count = drive_control_irq_count;
	drive_telemetry.control_tick_count = drive_control_tick_count;
}

void SecondDrive_Process(void)
{
	if (drive_run_mode == DRIVE_RUN_SECOND) {
		FirstDrive_Process();
	}
}

FirstDriveState_t FirstDrive_GetState(void)
{
	return drive_state;
}

FirstDriveFault_t FirstDrive_GetFault(void)
{
	return drive_fault;
}

FirstDriveState_t SecondDrive_GetState(void)
{
	return (drive_run_mode == DRIVE_RUN_SECOND)
			? drive_state : FIRST_DRIVE_OFF;
}

FirstDriveFault_t SecondDrive_GetFault(void)
{
	return (drive_run_mode == DRIVE_RUN_SECOND)
			? drive_fault : FIRST_DRIVE_FAULT_NONE;
}

void FirstDrive_GetTelemetry(FirstDriveTelemetry_t *telemetry)
{
	if (telemetry == NULL) {
		return;
	}
	__disable_irq();
	*telemetry = (const FirstDriveTelemetry_t)drive_telemetry;
	__enable_irq();
}

void SecondDrive_GetTelemetry(SecondDriveTelemetry_t *telemetry)
{
	if (telemetry == NULL) {
		return;
	}
	FirstDrive_GetTelemetry(&telemetry->drive);
	SecondDrivePlanner_GetStatus(&telemetry->planner);
}

const FirstDriveConfig_t *FirstDrive_GetConfig(void)
{
	return &first_drive_config;
}

bool FirstDrive_SetPdGains(int32_t kp_q10, int32_t kd_q10)
{
	if (drive_control_enabled || (drive_state == FIRST_DRIVE_ARMED)
			|| (kp_q10 < 0) || (kp_q10 > FIRST_DRIVE_KP_MAX_Q10)
			|| (kd_q10 < 0) || (kd_q10 > FIRST_DRIVE_KD_MAX_Q10)) {
		return false;
	}
	first_drive_config.kp_q10 = kp_q10;
	first_drive_config.kd_q10 = kd_q10;
	return true;
}

bool FirstDrive_SetMotorTrim(int16_t trim_sps)
{
	if (drive_control_enabled || (drive_state == FIRST_DRIVE_ARMED)
			|| (trim_sps < -FIRST_DRIVE_TRIM_MAX_SPS)
			|| (trim_sps > FIRST_DRIVE_TRIM_MAX_SPS)) {
		return false;
	}
	first_drive_config.trim_sps = trim_sps;
	return true;
}

void HAL_TIM7_IRQ_Handler(void)
{
	drive_control_irq_count++;
	drive_timer_divider++;
	if (drive_timer_divider >= FIRST_DRIVE_CONTROL_DIVIDER) {
		drive_timer_divider = 0U;
		drive_control_tick_count++;
		FirstDrive_ControlTick();
	}
}
