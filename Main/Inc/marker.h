/*
 * marker.h
 *
 * Temporal marker-event collector shared by MARK DIAGNOSTIC and drive logic.
 */

#ifndef INC_MARKER_H_
#define INC_MARKER_H_

#include <stdbool.h>
#include <stdint.h>

#define MARKER_CENTER_MASK             0x7EU
#define MARKER_EDGE_MASK               0x81U
#define MARKER_WIDE_CENTER_COUNT       4U
#define MARKER_WIDE_MIN_FRAMES         3U
#define MARKER_EDGE_CLEAR_FRAMES       350U
#define MARKER_CROSS_CLEAR_FRAMES      500U
#define MARKER_EVENT_TIMEOUT_FRAMES    3000U

typedef enum {
	MARKER_EVENT_NONE = 0,
	MARKER_EVENT_EDGE_0,
	MARKER_EVENT_EDGE_7,
	MARKER_EVENT_BOTH,
	MARKER_EVENT_CROSS,
	MARKER_EVENT_UNKNOWN
} MarkerEventType_t;

typedef struct {
	MarkerEventType_t type;
	uint8_t edge_union;
	uint8_t full_union;
	uint8_t max_center_count;
	uint16_t max_wide_run;
	uint32_t duration_frames;
	uint32_t start_frame;
	uint32_t end_frame;
} MarkerEvent_t;

typedef struct {
	bool collecting;
	uint8_t edge_union;
	uint8_t full_union;
	uint8_t max_center_count;
	uint16_t wide_run;
	uint16_t max_wide_run;
	uint16_t quiet_frames;
	uint32_t start_frame;
	uint32_t last_active_frame;
	uint32_t event_count;
	MarkerEvent_t last_event;
} MarkerDetector_t;

void MarkerDetector_Init(MarkerDetector_t *detector);
bool MarkerDetector_Update(MarkerDetector_t *detector, uint8_t sensor_mask,
		uint32_t frame_number);
bool MarkerDetector_Flush(MarkerDetector_t *detector);
uint8_t Marker_CountCenter(uint8_t sensor_mask);

#endif /* INC_MARKER_H_ */
