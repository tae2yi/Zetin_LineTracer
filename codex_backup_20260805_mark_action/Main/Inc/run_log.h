/*
 * run_log.h
 *
 * Fixed-size RAM black box used by TEST DRIVE.
 */

#ifndef INC_RUN_LOG_H_
#define INC_RUN_LOG_H_

#include "marker.h"
#include "sensor.h"

#include <stdbool.h>
#include <stdint.h>

#define RUN_LOG_RECORD_PERIOD_MS        10U
#define RUN_LOG_MAX_RECORDS             6144U
#define RUN_LOG_MAX_EVENTS              128U

#define RUN_LOG_FLAG_MARK_COLLECTING    0x01U
#define RUN_LOG_FLAG_LINE_VALID         0x02U
#define RUN_LOG_FLAG_MOTOR_RUNNING      0x04U
#define RUN_LOG_FLAG_WIDE_CENTER        0x08U
#define RUN_LOG_FLAG_EVENT_SHIFT        4U

typedef enum {
	RUN_LOG_STOP_NONE = 0,
	RUN_LOG_STOP_USER,
	RUN_LOG_STOP_LINE_LOST,
	RUN_LOG_STOP_SENSOR_DROP,
	RUN_LOG_STOP_TIMEOUT,
	RUN_LOG_STOP_LOG_FULL,
	RUN_LOG_STOP_MOTOR_ERROR,
	RUN_LOG_STOP_SENSOR_ERROR
} RunLogStopReason_t;

typedef struct {
	uint16_t raw[SENSOR_COUNT];
	int16_t line_position;
	uint16_t left_steps_per_second;
	uint16_t right_steps_per_second;
	uint8_t state_mask;
	uint8_t flags;
} RunLogRecord_t;

typedef struct {
	uint32_t sequence;
	uint32_t start_frame;
	uint32_t end_frame;
	uint32_t duration_frames;
	uint8_t type;
	uint8_t edge_union;
	uint8_t full_union;
	uint8_t max_center_count;
	uint16_t max_wide_run;
	uint16_t reserved0;
	uint32_t reserved1[2];
} RunLogEvent_t;

void RunLog_Reset(void);
bool RunLog_AppendRecord(const uint16_t raw[SENSOR_COUNT],
		int16_t line_position, uint16_t left_steps_per_second,
		uint16_t right_steps_per_second, uint8_t state_mask,
		uint8_t flags);
bool RunLog_AppendEvent(const MarkerEvent_t *event);
bool RunLog_IsFull(void);
uint32_t RunLog_GetRecordCount(void);
uint32_t RunLog_GetEventCount(void);
const RunLogRecord_t *RunLog_GetRecords(void);
const RunLogEvent_t *RunLog_GetEvents(void);

#endif /* INC_RUN_LOG_H_ */
