/*
 * second_drive.h
 *
 * Second-run track-map playback and speed planning.
 */

#ifndef INC_SECOND_DRIVE_H_
#define INC_SECOND_DRIVE_H_

#include <stdbool.h>
#include <stdint.h>

#include "drive.h"
#include "track.h"

#define SECOND_DRIVE_DEFAULT_STRAIGHT_SPS        5600U
#define SECOND_DRIVE_DEFAULT_CURVE_SPS           3000U
#define SECOND_DRIVE_APPROACH_BONUS_SPS           300U
#define SECOND_DRIVE_EXIT_BONUS_SPS               800U
#define SECOND_DRIVE_PERFORMANCE_MAX_CENTER_SPS  6200U
#define SECOND_DRIVE_DEFAULT_OVERALL_PERCENT      100U

#define SECOND_DRIVE_STRAIGHT_MIN_SPS           4800U
#define SECOND_DRIVE_STRAIGHT_MAX_SPS           6000U
#define SECOND_DRIVE_STRAIGHT_STEP_SPS           100U
#define SECOND_DRIVE_CURVE_MIN_SPS              2600U
#define SECOND_DRIVE_CURVE_MAX_SPS              3600U
#define SECOND_DRIVE_CURVE_STEP_SPS              100U
#define SECOND_DRIVE_OVERALL_MIN_PERCENT      90U
#define SECOND_DRIVE_OVERALL_MAX_PERCENT     120U
#define SECOND_DRIVE_OVERALL_STEP_PERCENT      5U
#define SECOND_DRIVE_FAST_ENTER_POSITION        500
#define SECOND_DRIVE_FAST_EXIT_POSITION         900
#define SECOND_DRIVE_FAST_STABLE_FRAMES          30U
#define SECOND_DRIVE_FINAL_EXIT_STABLE_FRAMES    50U
#define SECOND_DRIVE_LIMIT_TRACE_DEPTH           16U

typedef struct {
	uint16_t straight_sps;
	uint16_t curve_sps;
	uint8_t overall_percent;
} SecondDriveConfig_t;

typedef enum {
	SECOND_DRIVE_SYNC_MAP = 0,
	SECOND_DRIVE_SYNC_SEEK_CROSS,
	SECOND_DRIVE_SYNC_INVALID
} SecondDriveSyncState_t;

typedef enum {
	SECOND_DRIVE_MISMATCH_NONE = 0,
	SECOND_DRIVE_MISMATCH_EVENT_TYPE,
	SECOND_DRIVE_MISMATCH_EVENT_DISTANCE,
	SECOND_DRIVE_MISMATCH_SEGMENT_OVERDUE,
	SECOND_DRIVE_MISMATCH_MAP_BOUNDS,
	SECOND_DRIVE_MISMATCH_ANCHOR_NOT_FOUND,
	SECOND_DRIVE_MISMATCH_ANCHOR_AMBIGUOUS
} SecondDriveMismatchReason_t;

typedef enum {
	SECOND_DRIVE_GEOMETRY_MAP_SEGMENT = 0,
	SECOND_DRIVE_GEOMETRY_PAIR_OPEN,
	SECOND_DRIVE_GEOMETRY_PAIR_CLOSE,
	SECOND_DRIVE_GEOMETRY_CROSS_RESET,
	SECOND_DRIVE_GEOMETRY_LOCAL_CLOSE_REPAIR,
	SECOND_DRIVE_GEOMETRY_UNCERTAIN
} SecondDriveGeometrySource_t;

typedef enum {
	SECOND_DRIVE_LIMIT_NONE = 0,
	SECOND_DRIVE_LIMIT_FAST_STRAIGHT,
	SECOND_DRIVE_LIMIT_FAST_CROSS_APPROACH,
	SECOND_DRIVE_LIMIT_FAST_CROSS_EXIT,
	SECOND_DRIVE_LIMIT_FAST_END_CORRIDOR,
	SECOND_DRIVE_LIMIT_FAST_LOCAL_CLOSE_REPAIR,
	SECOND_DRIVE_LIMIT_CURVE_APPROACH,
	SECOND_DRIVE_LIMIT_CURVE_CRUISE,
	SECOND_DRIVE_LIMIT_CURVE_EXIT,
	SECOND_DRIVE_LIMIT_MAP_INVALID,
	SECOND_DRIVE_LIMIT_SEEK_CROSS,
	SECOND_DRIVE_LIMIT_SEGMENT_UNCERTAIN,
	SECOND_DRIVE_LIMIT_MARKER_PAIR_UNCLOSED,
	SECOND_DRIVE_LIMIT_PHASE_NOT_STRAIGHT,
	SECOND_DRIVE_LIMIT_POSITION,
	SECOND_DRIVE_LIMIT_TURN_BRAKE,
	SECOND_DRIVE_LIMIT_RECOVERY,
	SECOND_DRIVE_LIMIT_MAX_CLAMP,
	SECOND_DRIVE_LIMIT_END_BRAKE,
	SECOND_DRIVE_LIMIT_COUNT
} SecondDriveLimitReason_t;

typedef enum {
	SECOND_DRIVE_LOCAL_REPAIR_NONE = 0,
	SECOND_DRIVE_LOCAL_REPAIR_NO_OPEN,
	SECOND_DRIVE_LOCAL_REPAIR_WRONG_SIDE,
	SECOND_DRIVE_LOCAL_REPAIR_EXPECTED_NOT_BOUNDARY,
	SECOND_DRIVE_LOCAL_REPAIR_LOW_CONFIDENCE,
	SECOND_DRIVE_LOCAL_REPAIR_DUPLICATE,
	SECOND_DRIVE_LOCAL_REPAIR_MAP_NOT_SYNC,
	SECOND_DRIVE_LOCAL_REPAIR_SEGMENT_MISMATCH
} SecondDriveLocalRepairRejectReason_t;

typedef enum {
	SECOND_DRIVE_MARKER_REJECT_NO_LINE = 0,
	SECOND_DRIVE_MARKER_REJECT_OFF_CENTER,
	SECOND_DRIVE_MARKER_REJECT_NO_CENTER_MASK,
	SECOND_DRIVE_MARKER_REJECT_BRIDGE
} SecondDriveMarkerRejectReason_t;

typedef struct {
	uint8_t map_valid;
	SecondDriveSyncState_t sync_state;
	SecondDriveMismatchReason_t last_mismatch_reason;
	uint8_t mismatch_count;
	uint8_t resync_count;
	uint16_t ignored_event_count;
	uint16_t replay_event_count;
	uint16_t expected_event_index;
	uint16_t segment_index;
	uint16_t segment_count;
	uint16_t anchor_count;
	uint16_t current_anchor_order;
	TrackSegmentType_t segment_type;
	TrackSegmentType_t next_segment_type;
	uint8_t curve_units;
	uint32_t segment_distance_steps;
	uint32_t segment_travelled_steps;
	uint32_t segment_remaining_steps;
	uint32_t segment_start_step;
	uint32_t next_restriction_distance_steps;
	uint16_t nominal_target_sps;
	uint16_t planner_target_sps;
	uint16_t final_target_sps;
	SecondDriveLimitReason_t limit_reason;
	SecondDriveGeometrySource_t geometry_source;
	uint8_t fast_gate_ready;
	uint8_t cross_approach_corridor;
	uint8_t final_end_corridor;
	uint8_t replay_turn_open;
	int8_t replay_turn_direction;
	uint8_t local_close_repair_active;
	int8_t local_close_repair_direction;
	uint16_t local_close_repair_count;
	uint16_t marker_pair_mismatch_count;
	SecondDriveLocalRepairRejectReason_t local_repair_reject_reason;
	uint16_t centered_stable_frames;
	uint32_t replay_turn_open_step;
	uint32_t local_close_repair_step;
	uint32_t expected_marker_distance_steps;
} SecondDrivePlannerStatus_t;

typedef struct {
	uint32_t step;
	uint16_t segment_index;
	TrackSegmentType_t segment_type;
	FirstDriveCoursePhase_t phase;
	SecondDriveLimitReason_t reason;
	SecondDriveGeometrySource_t geometry_source;
	uint8_t replay_turn_open;
	int8_t replay_turn_direction;
	uint16_t requested_sps;
	uint16_t final_sps;
	int16_t line_position;
} SecondDriveLimitTraceEntry_t;

typedef struct {
	uint8_t valid;
	uint32_t elapsed_ms;
	uint32_t control_samples;
	uint64_t center_sps_sum;
	uint16_t center_sps_max;
	uint16_t target_sps_max;
	uint32_t limiter_samples[SECOND_DRIVE_LIMIT_COUNT];
	uint16_t fast_entry_count;
	uint16_t fast_exit_count;
	uint16_t mismatch_count;
	uint16_t resync_count;
	uint16_t local_close_repair_count;
	uint16_t unmatched_turn_at_end_count;
	uint16_t marker_reject_no_line_count;
	uint16_t marker_reject_off_center_count;
	uint16_t marker_reject_no_center_mask_count;
	uint16_t marker_reject_bridge_count;
	uint32_t end_brake_step;
	uint16_t end_brake_entry_sps;
	uint16_t brake_hold_ms;
	uint8_t end_brake_completed;
	uint32_t expected_end_step;
	int32_t end_step_error;
	uint8_t trace_count;
	uint8_t trace_head;
	SecondDriveLimitTraceEntry_t trace[SECOND_DRIVE_LIMIT_TRACE_DEPTH];
} SecondDriveRunStats_t;

typedef struct {
	FirstDriveTelemetry_t drive;
	SecondDrivePlannerStatus_t planner;
} SecondDriveTelemetry_t;

bool SecondDrive_Init(void);
bool SecondDrive_Arm(void);
bool SecondDrive_Start(void);
void SecondDrive_Process(void);
void SecondDrive_EmergencyStop(void);
void SecondDrive_Stop(void);
FirstDriveState_t SecondDrive_GetState(void);
FirstDriveFault_t SecondDrive_GetFault(void);
void SecondDrive_GetTelemetry(SecondDriveTelemetry_t *telemetry);
const SecondDriveConfig_t *SecondDrive_GetConfig(void);
bool SecondDrive_SetStraightSps(uint16_t straight_sps);
bool SecondDrive_SetCurveSps(uint16_t curve_sps);
bool SecondDrive_SetOverallPercent(uint8_t overall_percent);
uint16_t SecondDrive_GetEffectiveStraightSps(void);
uint16_t SecondDrive_GetEffectiveCurveSps(void);
uint16_t SecondDrive_GetEffectiveApproachSps(void);
uint16_t SecondDrive_GetEffectiveExitSps(void);

/* Controller integration used by drive.c. */
bool SecondDrivePlanner_MapIsStructurallyValid(void);
void SecondDrivePlanner_Reset(void);
void SecondDrivePlanner_BeginRun(void);
void SecondDrivePlanner_OnEvent(const TrackMarkerEvent_t *event);
uint16_t SecondDrivePlanner_GetTargetSps(uint16_t first_drive_target_sps,
		int32_t line_position, FirstDriveCoursePhase_t course_phase,
		uint32_t average_step, uint16_t current_sps, bool line_valid,
		bool recovering, bool provisional_marker);
void SecondDrivePlanner_RecordFinalTarget(uint16_t final_target_sps,
		uint16_t actual_center_sps, int16_t line_position,
		uint32_t average_step, FirstDriveCoursePhase_t course_phase,
		bool recovery_slow);
void SecondDrivePlanner_RecordMarkerReject(
		SecondDriveMarkerRejectReason_t reason);
bool SecondDrivePlanner_IsFinalEndCorridor(void);
void SecondDrivePlanner_RecordEndBrake(uint32_t step, uint16_t entry_sps);
void SecondDrivePlanner_RecordBrakeCompletion(uint16_t hold_ms,
		bool completed);
void SecondDrivePlanner_FinalizeRunStats(uint32_t elapsed_ms);
void SecondDrivePlanner_GetRunStats(SecondDriveRunStats_t *stats);
void SecondDrivePlanner_GetStatus(SecondDrivePlannerStatus_t *status);

#endif /* INC_SECOND_DRIVE_H_ */
