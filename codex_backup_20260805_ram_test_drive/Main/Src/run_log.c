/*
 * run_log.c
 */

#include "run_log.h"

#include <string.h>

_Static_assert(sizeof(RunLogRecord_t) == 24U,
		"RunLogRecord_t format changed");
_Static_assert(sizeof(RunLogEvent_t) == 32U,
		"RunLogEvent_t format changed");
_Static_assert(sizeof(RunLogHeader_t) == 256U,
		"RunLogHeader_t format changed");
_Static_assert((RUN_LOG_FLASH_RECORDS_OFFSET
		+ (RUN_LOG_MAX_RECORDS * sizeof(RunLogRecord_t)))
		<= RUN_LOG_FLASH_REGION_SIZE, "record region exceeds flash log area");
_Static_assert((RUN_LOG_FLASH_EVENTS_OFFSET
		+ (RUN_LOG_MAX_EVENTS * sizeof(RunLogEvent_t)))
		<= RUN_LOG_FLASH_RECORDS_OFFSET, "event and record regions overlap");

static RunLogRecord_t run_log_records[RUN_LOG_MAX_RECORDS];
static RunLogEvent_t run_log_events[RUN_LOG_MAX_EVENTS];
static uint32_t run_log_record_count;
static uint32_t run_log_event_count;
static uint32_t run_log_duration_ms;
static uint32_t run_log_dropped_frames;
static uint16_t run_log_base_steps_per_second;
static uint8_t run_log_sensor0_is_left;
static uint8_t run_log_calibration_valid_mask;
static uint16_t run_log_calibration_low[SENSOR_COUNT];
static uint16_t run_log_calibration_high[SENSOR_COUNT];
static RunLogStopReason_t run_log_stop_reason;

static uint32_t RunLog_Crc32Update(uint32_t crc, const uint8_t *data,
		size_t length)
{
	size_t index;

	for (index = 0U; index < length; index++) {
		uint8_t bit;

		crc ^= data[index];
		for (bit = 0U; bit < 8U; bit++) {
			crc = (crc >> 1U) ^ ((crc & 1U) != 0U
					? 0xEDB88320UL : 0U);
		}
	}
	return crc;
}

void RunLog_Reset(uint16_t base_steps_per_second, bool sensor0_is_left,
		uint8_t calibration_valid_mask,
		const uint16_t calibration_low[SENSOR_COUNT],
		const uint16_t calibration_high[SENSOR_COUNT])
{
	uint8_t index;

	run_log_record_count = 0U;
	run_log_event_count = 0U;
	run_log_duration_ms = 0U;
	run_log_dropped_frames = 0U;
	run_log_base_steps_per_second = base_steps_per_second;
	run_log_sensor0_is_left = sensor0_is_left ? 1U : 0U;
	run_log_calibration_valid_mask = calibration_valid_mask;
	run_log_stop_reason = RUN_LOG_STOP_NONE;
	for (index = 0U; index < SENSOR_COUNT; index++) {
		run_log_calibration_low[index] = calibration_low[index];
		run_log_calibration_high[index] = calibration_high[index];
	}
}

bool RunLog_AppendRecord(const uint16_t raw[SENSOR_COUNT],
		int16_t line_position, uint16_t left_steps_per_second,
		uint16_t right_steps_per_second, uint8_t state_mask,
		uint8_t flags)
{
	RunLogRecord_t *record;
	uint8_t index;

	if ((raw == NULL) || (run_log_record_count >= RUN_LOG_MAX_RECORDS)) {
		return false;
	}
	record = &run_log_records[run_log_record_count++];
	for (index = 0U; index < SENSOR_COUNT; index++) {
		record->raw[index] = raw[index];
	}
	record->line_position = line_position;
	record->left_steps_per_second = left_steps_per_second;
	record->right_steps_per_second = right_steps_per_second;
	record->state_mask = state_mask;
	record->flags = flags;
	return true;
}

bool RunLog_AppendEvent(const MarkerEvent_t *event)
{
	RunLogEvent_t *stored;

	if ((event == NULL) || (run_log_event_count >= RUN_LOG_MAX_EVENTS)) {
		return false;
	}
	stored = &run_log_events[run_log_event_count];
	memset(stored, 0, sizeof(*stored));
	stored->sequence = run_log_event_count + 1U;
	stored->start_frame = event->start_frame;
	stored->end_frame = event->end_frame;
	stored->duration_frames = event->duration_frames;
	stored->type = (uint8_t)event->type;
	stored->edge_union = event->edge_union;
	stored->full_union = event->full_union;
	stored->max_center_count = event->max_center_count;
	stored->max_wide_run = event->max_wide_run;
	run_log_event_count++;
	return true;
}

void RunLog_Finalize(uint32_t duration_ms, uint32_t dropped_frames,
		RunLogStopReason_t stop_reason)
{
	run_log_duration_ms = duration_ms;
	run_log_dropped_frames = dropped_frames;
	run_log_stop_reason = stop_reason;
}

bool RunLog_IsFull(void)
{
	return run_log_record_count >= RUN_LOG_MAX_RECORDS;
}

uint32_t RunLog_GetRecordCount(void)
{
	return run_log_record_count;
}

uint32_t RunLog_GetEventCount(void)
{
	return run_log_event_count;
}

const RunLogRecord_t *RunLog_GetRecords(void)
{
	return run_log_records;
}

const RunLogEvent_t *RunLog_GetEvents(void)
{
	return run_log_events;
}

uint32_t RunLog_Crc32(const void *data, size_t length)
{
	if ((data == NULL) && (length != 0U)) {
		return 0U;
	}
	return RunLog_Crc32Update(0xFFFFFFFFUL,
			(const uint8_t *)data, length) ^ 0xFFFFFFFFUL;
}

uint32_t RunLog_CalculateDataCrc(const RunLogRecord_t *records,
		uint32_t record_count, const RunLogEvent_t *events,
		uint32_t event_count)
{
	uint32_t crc = 0xFFFFFFFFUL;

	if (((records == NULL) && (record_count != 0U))
			|| ((events == NULL) && (event_count != 0U))) {
		return 0U;
	}
	crc = RunLog_Crc32Update(crc, (const uint8_t *)records,
			(size_t)record_count * sizeof(*records));
	crc = RunLog_Crc32Update(crc, (const uint8_t *)events,
			(size_t)event_count * sizeof(*events));
	return crc ^ 0xFFFFFFFFUL;
}

uint32_t RunLog_CalculateHeaderCrc(const RunLogHeader_t *header)
{
	RunLogHeader_t copy;

	if (header == NULL) {
		return 0U;
	}
	copy = *header;
	copy.header_crc32 = 0U;
	return RunLog_Crc32(&copy, sizeof(copy));
}

void RunLog_BuildHeader(RunLogHeader_t *header)
{
	uint8_t index;

	if (header == NULL) {
		return;
	}
	memset(header, 0, sizeof(*header));
	header->magic = RUN_LOG_MAGIC;
	header->format_version = RUN_LOG_FORMAT_VERSION;
	header->header_size = sizeof(*header);
	header->record_size = sizeof(RunLogRecord_t);
	header->event_size = sizeof(RunLogEvent_t);
	header->record_period_ms = RUN_LOG_RECORD_PERIOD_MS;
	header->record_count = run_log_record_count;
	header->event_count = run_log_event_count;
	header->dropped_frames = run_log_dropped_frames;
	header->duration_ms = run_log_duration_ms;
	header->base_steps_per_second = run_log_base_steps_per_second;
	header->sensor0_is_left = run_log_sensor0_is_left;
	header->stop_reason = (uint8_t)run_log_stop_reason;
	header->calibration_valid_mask = run_log_calibration_valid_mask;
	for (index = 0U; index < SENSOR_COUNT; index++) {
		header->calibration_low[index] = run_log_calibration_low[index];
		header->calibration_high[index] = run_log_calibration_high[index];
	}
	header->records_offset = RUN_LOG_FLASH_RECORDS_OFFSET;
	header->events_offset = RUN_LOG_FLASH_EVENTS_OFFSET;
	header->data_crc32 = RunLog_CalculateDataCrc(run_log_records,
			run_log_record_count, run_log_events, run_log_event_count);
	header->header_crc32 = RunLog_CalculateHeaderCrc(header);
}

bool RunLog_ValidateHeader(const RunLogHeader_t *header)
{
	if ((header == NULL) || (header->magic != RUN_LOG_MAGIC)
			|| (header->format_version != RUN_LOG_FORMAT_VERSION)
			|| (header->header_size != sizeof(*header))
			|| (header->record_size != sizeof(RunLogRecord_t))
			|| (header->event_size != sizeof(RunLogEvent_t))
			|| (header->record_period_ms != RUN_LOG_RECORD_PERIOD_MS)
			|| (header->record_count > RUN_LOG_MAX_RECORDS)
			|| (header->event_count > RUN_LOG_MAX_EVENTS)
			|| (header->records_offset != RUN_LOG_FLASH_RECORDS_OFFSET)
			|| (header->events_offset != RUN_LOG_FLASH_EVENTS_OFFSET)) {
		return false;
	}
	return header->header_crc32 == RunLog_CalculateHeaderCrc(header);
}
