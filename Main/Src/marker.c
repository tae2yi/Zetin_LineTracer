/*
 * marker.c
 *
 * A marker is classified from the order and duration of sensor groups, not
 * from a single 8-bit frame.  Bits 1..6 are the forward line sensors while
 * bits 0 and 7 are the two rear marker sensors.
 */

#include "marker.h"

#include <stddef.h>
#include <string.h>

static uint8_t Marker_Popcount8(uint8_t value)
{
	uint8_t count = 0U;

	while (value != 0U) {
		count += value & 0x01U;
		value >>= 1U;
	}
	return count;
}

uint8_t Marker_CountCenter(uint8_t sensor_mask)
{
	return Marker_Popcount8(sensor_mask & MARKER_CENTER_MASK);
}

void MarkerDetector_Init(MarkerDetector_t *detector)
{
	if (detector == NULL) {
		return;
	}
	memset(detector, 0, sizeof(*detector));
}

static void MarkerDetector_Start(MarkerDetector_t *detector,
		uint32_t frame_number)
{
	detector->collecting = true;
	detector->edge_union = 0U;
	detector->full_union = 0U;
	detector->max_center_count = 0U;
	detector->wide_run = 0U;
	detector->max_wide_run = 0U;
	detector->quiet_frames = 0U;
	detector->start_frame = frame_number;
	detector->last_active_frame = frame_number;
}

static void MarkerDetector_Finish(MarkerDetector_t *detector)
{
	MarkerEvent_t *event = &detector->last_event;

	if (detector->max_wide_run >= MARKER_WIDE_MIN_FRAMES) {
		event->type = MARKER_EVENT_CROSS;
	} else if (detector->edge_union == MARKER_EDGE_MASK) {
		event->type = MARKER_EVENT_BOTH;
	} else if (detector->edge_union == 0x01U) {
		event->type = MARKER_EVENT_EDGE_0;
	} else if (detector->edge_union == 0x80U) {
		event->type = MARKER_EVENT_EDGE_7;
	} else {
		event->type = MARKER_EVENT_UNKNOWN;
	}

	event->edge_union = detector->edge_union;
	event->full_union = detector->full_union;
	event->max_center_count = detector->max_center_count;
	event->max_wide_run = detector->max_wide_run;
	event->start_frame = detector->start_frame;
	event->end_frame = detector->last_active_frame;
	event->duration_frames = (detector->last_active_frame
			>= detector->start_frame)
			? (detector->last_active_frame - detector->start_frame + 1U)
			: 0U;
	detector->event_count++;
	detector->collecting = false;
	detector->quiet_frames = 0U;
	detector->wide_run = 0U;
}

bool MarkerDetector_Update(MarkerDetector_t *detector, uint8_t sensor_mask,
		uint32_t frame_number)
{
	uint8_t center_count;
	uint8_t edges;
	bool wide;
	bool active;
	uint16_t clear_frames;
	uint32_t event_age;

	if (detector == NULL) {
		return false;
	}

	center_count = Marker_CountCenter(sensor_mask);
	edges = sensor_mask & MARKER_EDGE_MASK;
	wide = center_count >= MARKER_WIDE_CENTER_COUNT;
	active = wide || (edges != 0U);

	if (!detector->collecting) {
		if (!active) {
			return false;
		}
		MarkerDetector_Start(detector, frame_number);
	}

	detector->full_union |= sensor_mask;
	detector->edge_union |= edges;
	if (center_count > detector->max_center_count) {
		detector->max_center_count = center_count;
	}

	if (wide) {
		if (detector->wide_run < UINT16_MAX) {
			detector->wide_run++;
		}
		if (detector->wide_run > detector->max_wide_run) {
			detector->max_wide_run = detector->wide_run;
		}
	} else {
		detector->wide_run = 0U;
	}

	if (active) {
		detector->last_active_frame = frame_number;
		detector->quiet_frames = 0U;
	} else if (detector->quiet_frames < UINT16_MAX) {
		detector->quiet_frames++;
	}

	clear_frames = (detector->max_wide_run >= MARKER_WIDE_MIN_FRAMES)
			? MARKER_CROSS_CLEAR_FRAMES : MARKER_EDGE_CLEAR_FRAMES;
	event_age = frame_number - detector->start_frame + 1U;
	if ((detector->quiet_frames >= clear_frames)
			|| (event_age >= MARKER_EVENT_TIMEOUT_FRAMES)) {
		MarkerDetector_Finish(detector);
		return true;
	}

	return false;
}

bool MarkerDetector_Flush(MarkerDetector_t *detector)
{
	if ((detector == NULL) || !detector->collecting) {
		return false;
	}
	MarkerDetector_Finish(detector);
	return true;
}
