/*
 * drive.h
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	FIRST_DRIVE_OFF = 0,
	FIRST_DRIVE_READY,
	FIRST_DRIVE_ARMED,
	FIRST_DRIVE_COUNTDOWN,
	FIRST_DRIVE_LAUNCH,
	FIRST_DRIVE_FOLLOW,
	FIRST_DRIVE_TURN_LEFT,
	FIRST_DRIVE_TURN_RIGHT,
	FIRST_DRIVE_CROSS_PASS,
	FIRST_DRIVE_END_CANDIDATE,
	FIRST_DRIVE_RUNOUT,
	FIRST_DRIVE_STOPPED,
	FIRST_DRIVE_FAULT
} FirstDriveState_t;

typedef enum {
	FIRST_DRIVE_FAULT_NONE = 0,
	FIRST_DRIVE_FAULT_NO_CALIBRATION,
	FIRST_DRIVE_FAULT_START_NO_LINE,
	FIRST_DRIVE_FAULT_SENSOR_STALE,
	FIRST_DRIVE_FAULT_LINE_LOST,
	FIRST_DRIVE_FAULT_MOTOR_COMMAND,
	FIRST_DRIVE_FAULT_CONTROL_TIMER,
	FIRST_DRIVE_FAULT_TRACK_OVERFLOW,
	FIRST_DRIVE_FAULT_EDGE_STUCK,
	FIRST_DRIVE_FAULT_NO_TRACK
} FirstDriveFault_t;

typedef enum {
	FIRST_DRIVE_COURSE_STRAIGHT = 0,
	FIRST_DRIVE_COURSE_APPROACH_LEFT,
	FIRST_DRIVE_COURSE_APPROACH_RIGHT,
	FIRST_DRIVE_COURSE_TURN_LEFT,
	FIRST_DRIVE_COURSE_TURN_RIGHT,
	FIRST_DRIVE_COURSE_EXIT_LEFT,
	FIRST_DRIVE_COURSE_EXIT_RIGHT,
	FIRST_DRIVE_COURSE_CROSS
} FirstDriveCoursePhase_t;

#define FIRST_DRIVE_PHASE_LOG_DEPTH 3U
#define FIRST_DRIVE_MARKER_LOG_DEPTH 5U

typedef enum {
	FIRST_DRIVE_PHASE_REASON_NONE = 0,
	FIRST_DRIVE_PHASE_REASON_MARKER_PROVISIONAL,
	FIRST_DRIVE_PHASE_REASON_MARKER_CONFIRMED,
	FIRST_DRIVE_PHASE_REASON_POSITION_ENTER,
	FIRST_DRIVE_PHASE_REASON_APPROACH_EXPIRED,
	FIRST_DRIVE_PHASE_REASON_MARKER_EXIT,
	FIRST_DRIVE_PHASE_REASON_MARKER_STRAIGHT,
	FIRST_DRIVE_PHASE_REASON_CROSS_MARKER,
	FIRST_DRIVE_PHASE_REASON_CROSS_EXPIRED,
	FIRST_DRIVE_PHASE_REASON_EXIT_CENTERED
} FirstDrivePhaseReason_t;

typedef struct {
	FirstDriveCoursePhase_t from_phase;
	FirstDriveCoursePhase_t to_phase;
	FirstDrivePhaseReason_t reason;
	uint32_t step;
} FirstDrivePhaseLogEntry_t;

typedef struct {
	uint8_t type;
	uint8_t confidence;
	uint8_t edge_union;
	uint32_t step;
} FirstDriveMarkerLogEntry_t;

typedef struct {
	uint16_t launch_sps;
	uint16_t base_sps;
	uint16_t minimum_sps;
	uint16_t maximum_sps;
	uint16_t ramp_sps_per_10ms;
	int32_t kp_q10;
	int32_t kd_q10;
	int16_t trim_sps;
} FirstDriveConfig_t;

typedef struct {
	FirstDriveState_t state;
	FirstDriveFault_t fault;
	FirstDriveCoursePhase_t course_phase;
	int16_t line_position;
	int16_t last_valid_position;
	uint8_t sensor_mask;
	uint8_t raw_line_mask;
	uint8_t line_mask;
	uint8_t marker_spill_mask;
	uint8_t last_valid_sensor_mask;
	uint8_t last_valid_line_mask;
	uint8_t cluster_count;
	uint8_t center_count;
	uint8_t line_valid;
	uint8_t recovering;
	uint8_t end_candidate;
	uint8_t edge_only;
	uint8_t reacquire_count;
	int8_t bridge_recovery_direction;
	uint8_t outer_boost_active;
	int8_t marker_direction;
	int8_t provisional_marker_direction;
	uint8_t last_marker_type;
	uint8_t last_marker_confidence;
	uint8_t last_marker_edge_union;
	uint8_t phase_log_count;
	uint8_t marker_log_count;
	uint16_t left_sps;
	uint16_t right_sps;
	uint16_t base_sps;
	uint16_t centre_sps;
	uint16_t target_centre_sps;
	uint16_t lost_streak;
	uint16_t line_lost_ms;
	uint16_t line_lost_limit_ms;
	uint16_t edge_dwell_ms;
	uint32_t line_strength;
	int32_t p_term;
	int32_t d_term;
	int32_t steer;
	int16_t target_steer_permille;
	int16_t applied_steer_permille;
	uint16_t steer_limit_permille;
	uint32_t average_steps;
	uint32_t event_count;
	uint32_t lost_count;
	uint32_t elapsed_ms;
	uint16_t countdown_ms;
	uint32_t control_irq_count;
	uint32_t control_tick_count;
	FirstDrivePhaseLogEntry_t phase_log[FIRST_DRIVE_PHASE_LOG_DEPTH];
	FirstDriveMarkerLogEntry_t marker_log[FIRST_DRIVE_MARKER_LOG_DEPTH];
} FirstDriveTelemetry_t;

void FirstDrive_Init(void);
bool FirstDrive_Arm(void);
bool FirstDrive_Start(void);
void FirstDrive_Process(void);
void FirstDrive_EmergencyStop(void);
void FirstDrive_Stop(void);
FirstDriveState_t FirstDrive_GetState(void);
FirstDriveFault_t FirstDrive_GetFault(void);
void FirstDrive_GetTelemetry(FirstDriveTelemetry_t *telemetry);
const FirstDriveConfig_t *FirstDrive_GetConfig(void);
bool FirstDrive_SetPdGains(int32_t kp_q10, int32_t kd_q10);
bool FirstDrive_SetMotorTrim(int16_t trim_sps);

/* Called from the existing TIM7 period callback. */
void HAL_TIM7_IRQ_Handler(void);


#endif /* INC_DRIVE_H_ */
