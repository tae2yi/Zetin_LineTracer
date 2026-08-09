/*
 * test_drive.h
 *
 * Conservative diagnostic line follower. It is intentionally independent
 * from FIRST DRIVE and never makes a driving decision from a marker event.
 */

#ifndef INC_TEST_DRIVE_H_
#define INC_TEST_DRIVE_H_

#include "run_log.h"

#include <stdbool.h>
#include <stdint.h>

#define TEST_DRIVE_MIN_BASE_SPS          150U
#define TEST_DRIVE_MAX_BASE_SPS          200U
#define TEST_DRIVE_MAX_RUN_MS            60000U

typedef struct {
	bool active;
	bool motor_running;
	uint32_t elapsed_ms;
	uint32_t record_count;
	uint32_t event_count;
	uint32_t dropped_frames;
	uint16_t left_steps_per_second;
	uint16_t right_steps_per_second;
	int16_t line_position;
	uint8_t sensor_state_mask;
	RunLogStopReason_t stop_reason;
} TestDriveStatus_t;

bool TestDrive_Start(uint16_t base_steps_per_second, bool sensor0_is_left);
void TestDrive_Process(void);
void TestDrive_RequestStop(RunLogStopReason_t reason);
bool TestDrive_IsActive(void);
void TestDrive_GetStatus(TestDriveStatus_t *status);
const char *TestDrive_GetStopReasonName(RunLogStopReason_t reason);

#endif /* INC_TEST_DRIVE_H_ */
