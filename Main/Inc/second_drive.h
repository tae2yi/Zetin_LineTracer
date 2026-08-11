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

#define SECOND_DRIVE_STRAIGHT_MIN_SPS       4000U
#define SECOND_DRIVE_STRAIGHT_MAX_SPS       6200U
#define SECOND_DRIVE_STRAIGHT_STEP_SPS       100U
#define SECOND_DRIVE_OVERALL_MIN_PERCENT      80U
#define SECOND_DRIVE_OVERALL_MAX_PERCENT     120U
#define SECOND_DRIVE_OVERALL_STEP_PERCENT      5U

typedef struct {
	uint16_t straight_sps;
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
	uint32_t next_restriction_distance_steps;
} SecondDrivePlannerStatus_t;

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
bool SecondDrive_SetOverallPercent(uint8_t overall_percent);

/* Controller integration used by drive.c. */
bool SecondDrivePlanner_MapIsStructurallyValid(void);
void SecondDrivePlanner_Reset(void);
void SecondDrivePlanner_OnEvent(const TrackMarkerEvent_t *event);
uint16_t SecondDrivePlanner_GetTargetSps(uint16_t first_drive_target_sps,
		int32_t line_position, FirstDriveCoursePhase_t course_phase,
		uint32_t average_step, uint16_t current_sps);
void SecondDrivePlanner_GetStatus(SecondDrivePlannerStatus_t *status);

#endif /* INC_SECOND_DRIVE_H_ */
