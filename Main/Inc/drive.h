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

#include "track.h"

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

typedef enum {
	FIRST_DRIVE_STOP_REASON_NONE = 0,
	FIRST_DRIVE_STOP_REASON_END_MARKER,
	FIRST_DRIVE_STOP_REASON_MANUAL,
	FIRST_DRIVE_STOP_REASON_FAULT
} FirstDriveStopReason_t;

typedef enum {
	FIRST_DRIVE_MARKER_CLASS_START = 0,
	FIRST_DRIVE_MARKER_CLASS_LEFT,
	FIRST_DRIVE_MARKER_CLASS_RIGHT,
	FIRST_DRIVE_MARKER_CLASS_CROSS,
	FIRST_DRIVE_MARKER_CLASS_END,
	FIRST_DRIVE_MARKER_CLASS_UNKNOWN
} FirstDriveMarkerClass_t;

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
	uint8_t valid;
	uint8_t type;
	uint8_t summary_type;
	uint8_t confidence;
	uint8_t edge_union;
	uint8_t full_union;
	uint8_t entry_mask;
	uint8_t exit_mask;
	uint32_t step;
	uint32_t entry_step;
	uint32_t exit_step;
	uint32_t center_step;
	uint8_t max_center_count;
	uint16_t wide_center_run;
	uint16_t both_overlap_run;
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
	uint16_t cross_tail_suppressed_count;
	uint32_t last_cross_tail_gap_steps;
	uint8_t last_cross_tail_edge_union;
	uint8_t cross_tail_guard_active;
	uint16_t end_guard_reject_count;
	uint16_t cross_tail_affected_count;
	uint32_t max_cross_tail_gap_steps;
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

typedef struct {
	uint16_t total_count;
	uint16_t start_count;
	uint16_t left_count;
	uint16_t right_count;
	uint16_t cross_count;
	uint16_t end_count;
	uint16_t unknown_count;
	uint16_t segment_count;
	uint16_t anchor_count;
	uint16_t tail_event_count;
	uint16_t tail_cross_count;
	uint32_t tail_max_gap_steps;
	uint16_t end_guard_reject_count;
	uint8_t track_overflow;
	uint8_t anchor_overflow;
	uint8_t map_valid;
} FirstDriveMarkerSummary_t;

typedef struct {
	uint16_t loss_episode_count;
	uint16_t recovery_success_count;
	uint16_t max_loss_ms;
	uint32_t lost_frame_count;
	uint16_t max_edge_dwell_normal_ms;
	uint16_t max_edge_dwell_turn_ms;
} FirstDriveQualitySummary_t;

typedef struct {
	uint64_t center_speed_sum_sps;
	uint32_t sample_count;
	uint16_t center_avg_sps;
	uint16_t center_max_sps;
	uint16_t target_center_max_sps;
	uint16_t left_max_sps;
	uint16_t right_max_sps;
	uint16_t base_sps;
	uint16_t turn_sps;
	uint16_t cross_sps;
	uint16_t recovery_straight_sps;
	uint16_t recovery_turn_sps;
} FirstDriveSpeedSummary_t;

typedef struct {
	uint32_t center_step;
	uint32_t elapsed_ms;
	FirstDriveState_t state;
	FirstDriveCoursePhase_t course_phase;
	uint8_t sensor_mask;
	uint8_t raw_line_mask;
	uint8_t line_mask;
	uint8_t marker_spill_mask;
	uint8_t last_valid_sensor_mask;
	uint8_t last_valid_line_mask;
	int16_t line_position;
	int16_t last_valid_position;
	int32_t p_term;
	int32_t d_term;
	int32_t steer;
	int16_t target_steer_permille;
	int16_t applied_steer_permille;
	uint16_t steer_limit_permille;
	uint16_t base_sps;
	uint16_t target_centre_sps;
	uint16_t current_centre_sps;
	uint16_t left_sps;
	uint16_t right_sps;
	uint16_t line_lost_ms;
	uint16_t line_lost_limit_ms;
	uint16_t edge_dwell_ms;
	int8_t bridge_recovery_direction;
	uint8_t outer_boost_active;
	FirstDriveMarkerLogEntry_t last_marker;
	uint32_t control_irq_count;
	uint32_t control_tick_count;
} FirstDriveStopSnapshot_t;

typedef struct {
	uint8_t valid;
	FirstDriveStopReason_t stop_reason;
	FirstDriveFault_t fault;
	FirstDriveStopSnapshot_t stop;
	FirstDriveMarkerSummary_t markers;
	FirstDriveQualitySummary_t quality;
	FirstDriveSpeedSummary_t speed;
	uint8_t phase_log_count;
	uint8_t marker_log_count;
	FirstDrivePhaseLogEntry_t phase_log[FIRST_DRIVE_PHASE_LOG_DEPTH];
	FirstDriveMarkerLogEntry_t marker_log[FIRST_DRIVE_MARKER_LOG_DEPTH];
} FirstDriveRunRecord_t;

void FirstDrive_Init(void);
bool FirstDrive_Arm(void);
bool FirstDrive_Start(void);
void FirstDrive_Process(void);
void FirstDrive_EmergencyStop(void);
void FirstDrive_Stop(void);
FirstDriveState_t FirstDrive_GetState(void);
FirstDriveFault_t FirstDrive_GetFault(void);
void FirstDrive_GetTelemetry(FirstDriveTelemetry_t *telemetry);
void FirstDrive_GetRunRecord(FirstDriveRunRecord_t *record);
const FirstDriveConfig_t *FirstDrive_GetConfig(void);
bool FirstDrive_SetPdGains(int32_t kp_q10, int32_t kd_q10);
bool FirstDrive_SetMotorTrim(int16_t trim_sps);

/* Called from the existing TIM7 period callback. */
void HAL_TIM7_IRQ_Handler(void);


#endif /* INC_DRIVE_H_ */
