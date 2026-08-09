/*
 * test_drive.c
 */

#include "test_drive.h"

#include "marker.h"
#include "motor.h"
#include "sensor.h"
#include "test_drive_logic.h"

#include "stm32h5xx_hal.h"

#include <limits.h>
#include <stddef.h>

#define TEST_DRIVE_CONTROL_FRAMES        5U
#define TEST_DRIVE_LOG_FRAMES            RUN_LOG_RECORD_PERIOD_MS
#define TEST_DRIVE_START_VALID_FRAMES    20U
#define TEST_DRIVE_START_TIMEOUT_FRAMES  1000U
#define TEST_DRIVE_LINE_LOST_FRAMES      250U
#define TEST_DRIVE_SENSOR_STALL_MS        250U
#define TEST_DRIVE_MIN_MOTOR_SPS          MOTOR_SPEED_TEST_MIN_SPS
#define TEST_DRIVE_PROCESS_FRAME_BUDGET     8U

_Static_assert(RUN_LOG_MAX_RECORDS
		>= ((TEST_DRIVE_MAX_RUN_MS + RUN_LOG_RECORD_PERIOD_MS - 1U)
				/ RUN_LOG_RECORD_PERIOD_MS),
		"run log must hold a complete maximum-duration test drive");

static bool test_drive_active;
static bool test_drive_motor_running;
static bool test_drive_sensor0_is_left;
static uint16_t test_drive_base_sps;
static uint16_t test_drive_left_sps;
static uint16_t test_drive_right_sps;
static uint32_t test_drive_frame_cursor;
static uint32_t test_drive_start_tick;
static uint32_t test_drive_last_frame_tick;
static uint32_t test_drive_duration_ms;
static uint32_t test_drive_dropped_frames;
static uint32_t test_drive_valid_start_frames;
static uint32_t test_drive_lost_frames;
static uint8_t test_drive_state_mask;
static int16_t test_drive_line_position;
static int16_t test_drive_last_reliable_position;
static int16_t test_drive_previous_control_position;
static MarkerDetector_t test_drive_marker;
static RunLogStopReason_t test_drive_stop_reason;

static void TestDrive_StopInternal(RunLogStopReason_t reason)
{
	uint32_t duration_ms;

	if (!test_drive_active) {
		return;
	}
	if (MarkerDetector_Flush(&test_drive_marker)) {
		(void)RunLog_AppendEvent(&test_drive_marker.last_event);
	}
	Motor_DriveStop();
	Sensor_Stop();
	test_drive_motor_running = false;
	test_drive_active = false;
	test_drive_stop_reason = reason;
	duration_ms = HAL_GetTick() - test_drive_start_tick;
	test_drive_duration_ms = duration_ms;
	RunLog_Finalize(duration_ms, test_drive_dropped_frames, reason);
}

static bool TestDrive_ApplyControl(int16_t position)
{
	uint16_t target_left;
	uint16_t target_right;

	TestDriveLogic_CalculateTargets(test_drive_base_sps,
			test_drive_sensor0_is_left, position,
			&test_drive_previous_control_position,
			&target_left, &target_right);
	test_drive_left_sps = TestDriveLogic_Slew(test_drive_left_sps,
			target_left);
	test_drive_right_sps = TestDriveLogic_Slew(test_drive_right_sps,
			target_right);
	return Motor_DriveSetSpeeds(test_drive_left_sps,
			test_drive_right_sps);
}

static void TestDrive_ProcessFrame(const uint16_t raw[SENSOR_COUNT],
		uint32_t frame_number)
{
	uint16_t normalized[SENSOR_COUNT];
	bool line_valid;
	bool wide_center;
	bool marker_completed;
	uint8_t flags;

	Sensor_NormalizeFrame(raw, normalized);
	test_drive_state_mask = Sensor_ApplyStateHysteresis(normalized,
			test_drive_state_mask);
	line_valid = TestDriveLogic_CalculateLinePosition(normalized,
			test_drive_state_mask, test_drive_last_reliable_position,
			&test_drive_line_position, &wide_center);
	if (line_valid && !wide_center) {
		test_drive_last_reliable_position = test_drive_line_position;
	}

	marker_completed = MarkerDetector_Update(&test_drive_marker,
			test_drive_state_mask, frame_number);
	if (marker_completed
			&& !RunLog_AppendEvent(&test_drive_marker.last_event)) {
		TestDrive_StopInternal(RUN_LOG_STOP_LOG_FULL);
		return;
	}

	if (!test_drive_motor_running) {
		if (line_valid && !wide_center) {
			test_drive_valid_start_frames++;
		} else {
			test_drive_valid_start_frames = 0U;
		}
		if (test_drive_valid_start_frames
				>= TEST_DRIVE_START_VALID_FRAMES) {
			test_drive_left_sps = TEST_DRIVE_MIN_MOTOR_SPS;
			test_drive_right_sps = TEST_DRIVE_MIN_MOTOR_SPS;
			test_drive_previous_control_position =
					test_drive_sensor0_is_left
					? test_drive_line_position
					: (int16_t)-test_drive_line_position;
			if (!Motor_DriveStart(test_drive_left_sps,
					test_drive_right_sps, MOTOR_SPEED_TEST_DAC)) {
				TestDrive_StopInternal(RUN_LOG_STOP_MOTOR_ERROR);
				return;
			}
			test_drive_motor_running = true;
		} else if (frame_number >= TEST_DRIVE_START_TIMEOUT_FRAMES) {
			TestDrive_StopInternal(RUN_LOG_STOP_LINE_LOST);
			return;
		}
	}

	if (test_drive_motor_running) {
		if (line_valid) {
			test_drive_lost_frames = 0U;
		} else {
			test_drive_lost_frames++;
			if (test_drive_lost_frames >= TEST_DRIVE_LINE_LOST_FRAMES) {
				TestDrive_StopInternal(RUN_LOG_STOP_LINE_LOST);
				return;
			}
		}
		if ((frame_number % TEST_DRIVE_CONTROL_FRAMES) == 0U) {
			if (!TestDrive_ApplyControl(test_drive_line_position)) {
				TestDrive_StopInternal(RUN_LOG_STOP_MOTOR_ERROR);
				return;
			}
		}
	}

	if ((frame_number % TEST_DRIVE_LOG_FRAMES) == 0U) {
		flags = (test_drive_marker.collecting
				? RUN_LOG_FLAG_MARK_COLLECTING : 0U)
				| (line_valid ? RUN_LOG_FLAG_LINE_VALID : 0U)
				| (test_drive_motor_running
						? RUN_LOG_FLAG_MOTOR_RUNNING : 0U)
				| (wide_center ? RUN_LOG_FLAG_WIDE_CENTER : 0U)
				| (uint8_t)(((uint8_t)test_drive_marker.last_event.type
						& 0x07U) << RUN_LOG_FLAG_EVENT_SHIFT);
		if (!RunLog_AppendRecord(raw, test_drive_line_position,
				test_drive_motor_running ? test_drive_left_sps : 0U,
				test_drive_motor_running ? test_drive_right_sps : 0U,
				test_drive_state_mask, flags)) {
			TestDrive_StopInternal(RUN_LOG_STOP_LOG_FULL);
			return;
		}
	}

	if (frame_number >= TEST_DRIVE_MAX_RUN_MS) {
		TestDrive_StopInternal(RUN_LOG_STOP_TIMEOUT);
	} else if (RunLog_IsFull()) {
		TestDrive_StopInternal(RUN_LOG_STOP_LOG_FULL);
	}
}

bool TestDrive_Start(uint16_t base_steps_per_second, bool sensor0_is_left)
{
	uint16_t calibration_low[SENSOR_COUNT];
	uint16_t calibration_high[SENSOR_COUNT];

	if (test_drive_active || !Sensor_IsCalibrationComplete()
			|| (base_steps_per_second < TEST_DRIVE_MIN_BASE_SPS)
			|| (base_steps_per_second > TEST_DRIVE_MAX_BASE_SPS)) {
		return false;
	}

	Motor_DriveStop();
	Sensor_Stop();
	Sensor_GetCalibration(calibration_low, calibration_high);
	RunLog_Reset(base_steps_per_second, sensor0_is_left,
			Sensor_GetCalibrationValidMask(), calibration_low,
			calibration_high);
	MarkerDetector_Init(&test_drive_marker);
	test_drive_base_sps = base_steps_per_second;
	test_drive_sensor0_is_left = sensor0_is_left;
	test_drive_left_sps = 0U;
	test_drive_right_sps = 0U;
	test_drive_frame_cursor = 0U;
	test_drive_start_tick = HAL_GetTick();
	test_drive_last_frame_tick = test_drive_start_tick;
	test_drive_duration_ms = 0U;
	test_drive_dropped_frames = 0U;
	test_drive_valid_start_frames = 0U;
	test_drive_lost_frames = 0U;
	test_drive_state_mask = 0U;
	test_drive_line_position = 0;
	test_drive_last_reliable_position = 0;
	test_drive_previous_control_position = 0;
	test_drive_stop_reason = RUN_LOG_STOP_NONE;
	test_drive_motor_running = false;

	Sensor_SetLightMode(SENSOR_LIGHT_PAIR, 0U);
	Sensor_Start();
	if (!Sensor_IsRunning()) {
		RunLog_Finalize(0U, 0U, RUN_LOG_STOP_SENSOR_ERROR);
		test_drive_stop_reason = RUN_LOG_STOP_SENSOR_ERROR;
		return false;
	}
	test_drive_active = true;
	return true;
}

void TestDrive_Process(void)
{
	uint16_t raw[SENSOR_COUNT];
	uint8_t processed_frames = 0U;

	/* Always return to the menu loop so the emergency-stop button remains
	 * responsive even if sensor frames arrive faster than expected. */
	while (test_drive_active
			&& (processed_frames < TEST_DRIVE_PROCESS_FRAME_BUDGET)) {
		uint32_t previous_cursor = test_drive_frame_cursor;

		if (!Sensor_GetFrameRawAfter(&test_drive_frame_cursor, raw)) {
			break;
		}
		test_drive_last_frame_tick = HAL_GetTick();
		if (test_drive_frame_cursor > (previous_cursor + 1U)) {
			test_drive_dropped_frames += test_drive_frame_cursor
					- previous_cursor - 1U;
			TestDrive_StopInternal(RUN_LOG_STOP_SENSOR_DROP);
			break;
		}
		TestDrive_ProcessFrame(raw, test_drive_frame_cursor);
		processed_frames++;
	}

	if (test_drive_active
			&& ((HAL_GetTick() - test_drive_last_frame_tick)
					>= TEST_DRIVE_SENSOR_STALL_MS)) {
		TestDrive_StopInternal(RUN_LOG_STOP_SENSOR_ERROR);
	}
}

void TestDrive_RequestStop(RunLogStopReason_t reason)
{
	if (reason == RUN_LOG_STOP_NONE) {
		reason = RUN_LOG_STOP_USER;
	}
	TestDrive_StopInternal(reason);
}

bool TestDrive_IsActive(void)
{
	return test_drive_active;
}

void TestDrive_GetStatus(TestDriveStatus_t *status)
{
	if (status == NULL) {
		return;
	}
	status->active = test_drive_active;
	status->motor_running = test_drive_motor_running;
	status->elapsed_ms = test_drive_active
			? (HAL_GetTick() - test_drive_start_tick)
			: test_drive_duration_ms;
	status->record_count = RunLog_GetRecordCount();
	status->event_count = RunLog_GetEventCount();
	status->dropped_frames = test_drive_dropped_frames;
	status->left_steps_per_second = test_drive_left_sps;
	status->right_steps_per_second = test_drive_right_sps;
	status->line_position = test_drive_line_position;
	status->sensor_state_mask = test_drive_state_mask;
	status->stop_reason = test_drive_stop_reason;
}

const char *TestDrive_GetStopReasonName(RunLogStopReason_t reason)
{
	switch (reason) {
	case RUN_LOG_STOP_USER:
		return "USER";
	case RUN_LOG_STOP_LINE_LOST:
		return "LINE LOST";
	case RUN_LOG_STOP_SENSOR_DROP:
		return "SENSOR DROP";
	case RUN_LOG_STOP_TIMEOUT:
		return "TIMEOUT";
	case RUN_LOG_STOP_LOG_FULL:
		return "LOG FULL";
	case RUN_LOG_STOP_MOTOR_ERROR:
		return "MOTOR ERROR";
	case RUN_LOG_STOP_SENSOR_ERROR:
		return "SENSOR ERROR";
	case RUN_LOG_STOP_SAVE_ERROR:
		return "SAVE ERROR";
	default:
		return "NONE";
	}
}
