/*
 * run_log.h
 *
 * Fixed-size RAM black box used by TEST DRIVE. Flash storage uses the same
 * versioned structures so a dump can be decoded without the running firmware.
 */

#ifndef INC_RUN_LOG_H_
#define INC_RUN_LOG_H_

#include "marker.h"
#include "sensor.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RUN_LOG_MAGIC                   0x54444C47UL /* "TDLG" */
#define RUN_LOG_FORMAT_VERSION          1U
#define RUN_LOG_RECORD_PERIOD_MS        10U
#define RUN_LOG_MAX_RECORDS             6144U
#define RUN_LOG_MAX_EVENTS              128U
#define RUN_LOG_FLASH_HEADER_OFFSET     0x0000UL
#define RUN_LOG_FLASH_EVENTS_OFFSET     0x0100UL
#define RUN_LOG_FLASH_RECORDS_OFFSET    0x2000UL
#define RUN_LOG_FLASH_BASE_ADDRESS      0x08040000UL
#define RUN_LOG_FLASH_REGION_SIZE       0x00040000UL

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
	RUN_LOG_STOP_SENSOR_ERROR,
	RUN_LOG_STOP_SAVE_ERROR
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

typedef struct {
	uint32_t magic;
	uint16_t format_version;
	uint16_t header_size;
	uint16_t record_size;
	uint16_t event_size;
	uint16_t record_period_ms;
	uint16_t reserved0;
	uint32_t record_count;
	uint32_t event_count;
	uint32_t dropped_frames;
	uint32_t duration_ms;
	uint16_t base_steps_per_second;
	uint8_t sensor0_is_left;
	uint8_t stop_reason;
	uint8_t calibration_valid_mask;
	uint8_t reserved1[3];
	uint16_t calibration_low[SENSOR_COUNT];
	uint16_t calibration_high[SENSOR_COUNT];
	uint32_t records_offset;
	uint32_t events_offset;
	uint32_t data_crc32;
	uint32_t header_crc32;
	uint8_t reserved2[168];
} RunLogHeader_t;

void RunLog_Reset(uint16_t base_steps_per_second, bool sensor0_is_left,
		uint8_t calibration_valid_mask,
		const uint16_t calibration_low[SENSOR_COUNT],
		const uint16_t calibration_high[SENSOR_COUNT]);
bool RunLog_AppendRecord(const uint16_t raw[SENSOR_COUNT],
		int16_t line_position, uint16_t left_steps_per_second,
		uint16_t right_steps_per_second, uint8_t state_mask,
		uint8_t flags);
bool RunLog_AppendEvent(const MarkerEvent_t *event);
void RunLog_Finalize(uint32_t duration_ms, uint32_t dropped_frames,
		RunLogStopReason_t stop_reason);
bool RunLog_IsFull(void);
uint32_t RunLog_GetRecordCount(void);
uint32_t RunLog_GetEventCount(void);
const RunLogRecord_t *RunLog_GetRecords(void);
const RunLogEvent_t *RunLog_GetEvents(void);
void RunLog_BuildHeader(RunLogHeader_t *header);

uint32_t RunLog_Crc32(const void *data, size_t length);
uint32_t RunLog_CalculateDataCrc(const RunLogRecord_t *records,
		uint32_t record_count, const RunLogEvent_t *events,
		uint32_t event_count);
uint32_t RunLog_CalculateHeaderCrc(const RunLogHeader_t *header);
bool RunLog_ValidateHeader(const RunLogHeader_t *header);

#endif /* INC_RUN_LOG_H_ */
