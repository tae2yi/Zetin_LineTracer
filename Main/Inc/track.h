/*
 * track.h
 *
 * RAM-only marker and segment recording for First Drive.
 */

#ifndef INC_TRACK_H_
#define INC_TRACK_H_

#include <stdbool.h>
#include <stdint.h>

#include "marker.h"

#define TRACK_MAX_EVENTS            512U
#define TRACK_MAX_SEGMENTS          512U
#define TRACK_MAX_CROSS_ANCHORS      64U
#define TRACK_MARK_CONFIRM_FRAMES   3U
#define TRACK_MARK_CLEAR_FRAMES      5U
#define TRACK_MARK_COOLDOWN_STEPS     50U
#define TRACK_CROSS_TAIL_MAX_GAP_STEPS 160U
#define TRACK_MARKER_MIN_OVERLAP     2U
#define TRACK_CURVE_UNIT_NOMINAL_STEPS 500U
#define TRACK_CURVE_UNIT_MIN           1U
#define TRACK_CURVE_UNIT_MAX           6U
#define TRACK_CURVE_UNIT_ERROR_MIN_STEPS 150U

typedef struct {
	MarkerEventType_t type;
	uint8_t edge_union;
	uint8_t full_union;
	uint8_t max_center_count;
	uint16_t edge0_run;
	uint16_t edge7_run;
	uint16_t both_overlap_run;
	uint16_t wide_center_run;
	uint8_t entry_mask;
	uint8_t exit_mask;
	uint32_t entry_frame;
	uint32_t exit_frame;
	uint32_t entry_step;
	uint32_t exit_step;
	uint32_t center_step;
	uint8_t confidence;
} TrackMarkerEvent_t;

typedef enum {
	TRACK_SEGMENT_STRAIGHT = 0,
	TRACK_SEGMENT_LEFT,
	TRACK_SEGMENT_RIGHT,
	TRACK_SEGMENT_CROSS,
	TRACK_SEGMENT_END
} TrackSegmentType_t;

typedef struct {
	TrackSegmentType_t type;
	uint32_t distance_steps;
	uint8_t curve_units; /* 0 unknown/not curve, 1..6 = 45..270 degrees */
} TrackSegment_t;

typedef struct {
	uint16_t order;
	uint16_t event_index;
	uint16_t segment_index_after_cross;
	uint32_t center_step;
	uint32_t distance_from_previous_cross;
} TrackCrossAnchor_t;

typedef enum {
	TRACK_PROCESS_NONE = 0,
	TRACK_PROCESS_EVENT_READY,
	TRACK_PROCESS_CROSS_TAIL_MERGED
} TrackProcessResult_t;

typedef struct {
	uint16_t cross_tail_suppressed_count;
	uint16_t cross_tail_affected_count;
	uint32_t last_cross_tail_gap_steps;
	uint32_t max_cross_tail_gap_steps;
	uint8_t last_cross_tail_edge_union;
	uint8_t cross_tail_pending;
} TrackCollectorDiagnostics_t;

typedef struct {
	uint8_t turn_open;
	int8_t turn_direction;
	int8_t last_direction;
	uint32_t last_direction_step;
	uint8_t unmatched_turn_at_end;
} TrackMapPairDiagnostics_t;

void Track_Reset(void);
void Track_SetStartIgnoreSteps(uint32_t minimum_step);
TrackProcessResult_t Track_ProcessSensor(uint8_t sensor_mask,
		uint32_t frame_number,
		uint32_t average_step);
TrackProcessResult_t Track_Flush(void);
const TrackMarkerEvent_t *Track_GetLastEvent(void);
void Track_ReplayReset(void);
TrackProcessResult_t Track_ProcessReplaySensor(uint8_t sensor_mask,
		uint32_t frame_number,
		uint32_t average_step);
const TrackMarkerEvent_t *Track_GetLastReplayEvent(void);
void Track_GetCollectorDiagnostics(bool replay,
		TrackCollectorDiagnostics_t *diagnostics);
uint16_t Track_GetEventCount(void);
uint16_t Track_GetSegmentCount(void);
const TrackMarkerEvent_t *Track_GetEvent(uint16_t index);
const TrackSegment_t *Track_GetSegment(uint16_t index);
uint16_t Track_GetCrossAnchorCount(void);
const TrackCrossAnchor_t *Track_GetCrossAnchor(uint16_t index);
const TrackCrossAnchor_t *Track_FindCrossAnchorByEventIndex(
		uint16_t event_index);
bool Track_HasOverflow(void);
bool Track_HasAnchorOverflow(void);
void Track_FinalizeSegments(void);
void Track_GetMapPairDiagnostics(TrackMapPairDiagnostics_t *diagnostics);

#endif /* INC_TRACK_H_ */
