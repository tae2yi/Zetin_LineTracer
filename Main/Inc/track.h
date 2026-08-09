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
#define TRACK_MARK_CONFIRM_FRAMES   3U
#define TRACK_MARK_CLEAR_FRAMES      5U
#define TRACK_MARK_COOLDOWN_STEPS     50U
#define TRACK_MARKER_MIN_OVERLAP     2U

typedef struct {
	MarkerEventType_t type;
	uint8_t edge_union;
	uint8_t full_union;
	uint8_t max_center_count;
	uint16_t edge0_run;
	uint16_t edge7_run;
	uint16_t both_overlap_run;
	uint16_t wide_center_run;
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
} TrackSegment_t;

void Track_Reset(void);
void Track_SetStartIgnoreSteps(uint32_t minimum_step);
bool Track_ProcessSensor(uint8_t sensor_mask, uint32_t frame_number,
		uint32_t average_step);
bool Track_Flush(void);
const TrackMarkerEvent_t *Track_GetLastEvent(void);
void Track_ReplayReset(void);
bool Track_ProcessReplaySensor(uint8_t sensor_mask, uint32_t frame_number,
		uint32_t average_step);
const TrackMarkerEvent_t *Track_GetLastReplayEvent(void);
uint16_t Track_GetEventCount(void);
uint16_t Track_GetSegmentCount(void);
const TrackMarkerEvent_t *Track_GetEvent(uint16_t index);
const TrackSegment_t *Track_GetSegment(uint16_t index);
bool Track_HasOverflow(void);
void Track_FinalizeSegments(void);

#endif /* INC_TRACK_H_ */
