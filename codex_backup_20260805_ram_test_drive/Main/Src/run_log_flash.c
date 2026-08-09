/*
 * run_log_flash.c
 *
 * Bank 2 is erased/programmed only after TEST DRIVE has stopped every motor
 * and sensor timer. The header is written last, so an interrupted save never
 * looks like a valid log.
 */

#include "run_log_flash.h"

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern uint8_t __test_log_start__;
extern uint8_t __test_log_end__;

_Static_assert((RUN_LOG_FLASH_BASE_ADDRESS & 0x0FUL) == 0U,
		"flash log base must be quad-word aligned");
_Static_assert((RUN_LOG_FLASH_REGION_SIZE % FLASH_SECTOR_SIZE) == 0U,
		"flash log area must contain complete sectors");

static bool RunLogFlash_ConfigurationIsSafe(void)
{
	return ((uintptr_t)&__test_log_start__ == RUN_LOG_FLASH_BASE_ADDRESS)
			&& ((uintptr_t)&__test_log_end__
					== (RUN_LOG_FLASH_BASE_ADDRESS
							+ RUN_LOG_FLASH_REGION_SIZE))
			&& (FLASH_SIZE == 0x00080000UL)
			&& ((FLASH->OPTSR_CUR & FLASH_OPTSR_SWAP_BANK) == 0U);
}

static HAL_StatusTypeDef RunLogFlash_ProgramBuffer(uint32_t address,
		const void *data, size_t length)
{
	const uint8_t *source = (const uint8_t *)data;
	size_t offset = 0U;

	while (offset < length) {
		uint32_t quad_word[4] = {
			0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL
		};
		size_t remaining = length - offset;
		size_t amount = (remaining < sizeof(quad_word))
				? remaining : sizeof(quad_word);

		memcpy(quad_word, source + offset, amount);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
				address + (uint32_t)offset,
				(uint32_t)(uintptr_t)quad_word) != HAL_OK) {
			return HAL_ERROR;
		}
		offset += sizeof(quad_word);
	}
	return HAL_OK;
}

bool RunLogFlash_ReadHeader(RunLogHeader_t *header, bool verify_data)
{
	const RunLogRecord_t *records;
	const RunLogEvent_t *events;
	uint32_t data_crc;

	if ((header == NULL) || !RunLogFlash_ConfigurationIsSafe()) {
		return false;
	}
	memcpy(header, (const void *)(uintptr_t)(RUN_LOG_FLASH_BASE_ADDRESS
			+ RUN_LOG_FLASH_HEADER_OFFSET), sizeof(*header));
	if (!RunLog_ValidateHeader(header)) {
		return false;
	}
	if (!verify_data) {
		return true;
	}

	records = (const RunLogRecord_t *)(uintptr_t)
			(RUN_LOG_FLASH_BASE_ADDRESS + header->records_offset);
	events = (const RunLogEvent_t *)(uintptr_t)
			(RUN_LOG_FLASH_BASE_ADDRESS + header->events_offset);
	data_crc = RunLog_CalculateDataCrc(records, header->record_count,
			events, header->event_count);
	return data_crc == header->data_crc32;
}

RunLogFlashStatus_t RunLogFlash_Save(void)
{
	FLASH_EraseInitTypeDef erase = { 0 };
	RunLogHeader_t header;
	RunLogHeader_t verify_header;
	uint32_t sector_error = 0xFFFFFFFFUL;
	HAL_StatusTypeDef status;
	RunLogFlashStatus_t result = RUN_LOG_FLASH_OK;

	if (!RunLogFlash_ConfigurationIsSafe()) {
		return RUN_LOG_FLASH_CONFIG_ERROR;
	}
	RunLog_BuildHeader(&header);

	if (HAL_FLASH_Unlock() != HAL_OK) {
		return RUN_LOG_FLASH_UNLOCK_ERROR;
	}

	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.Banks = FLASH_BANK_2;
	erase.Sector = FLASH_SECTOR_0;
	erase.NbSectors = FLASH_SECTOR_NB;
	status = HAL_FLASHEx_Erase(&erase, &sector_error);
	if ((status != HAL_OK) || (sector_error != 0xFFFFFFFFUL)) {
		result = RUN_LOG_FLASH_ERASE_ERROR;
		goto finish;
	}

	status = RunLogFlash_ProgramBuffer(RUN_LOG_FLASH_BASE_ADDRESS
			+ RUN_LOG_FLASH_RECORDS_OFFSET, RunLog_GetRecords(),
			(size_t)header.record_count * sizeof(RunLogRecord_t));
	if (status != HAL_OK) {
		result = RUN_LOG_FLASH_PROGRAM_ERROR;
		goto finish;
	}
	status = RunLogFlash_ProgramBuffer(RUN_LOG_FLASH_BASE_ADDRESS
			+ RUN_LOG_FLASH_EVENTS_OFFSET, RunLog_GetEvents(),
			(size_t)header.event_count * sizeof(RunLogEvent_t));
	if (status != HAL_OK) {
		result = RUN_LOG_FLASH_PROGRAM_ERROR;
		goto finish;
	}

	/* Commit marker: the validated header is intentionally programmed last. */
	status = RunLogFlash_ProgramBuffer(RUN_LOG_FLASH_BASE_ADDRESS
			+ RUN_LOG_FLASH_HEADER_OFFSET, &header, sizeof(header));
	if (status != HAL_OK) {
		result = RUN_LOG_FLASH_PROGRAM_ERROR;
	}

finish:
	if (HAL_FLASH_Lock() != HAL_OK) {
		if (result == RUN_LOG_FLASH_OK) {
			result = RUN_LOG_FLASH_PROGRAM_ERROR;
		}
	}
	if ((result == RUN_LOG_FLASH_OK)
			&& (!RunLogFlash_ReadHeader(&verify_header, true)
					|| (verify_header.record_count != header.record_count)
					|| (verify_header.event_count != header.event_count))) {
		result = RUN_LOG_FLASH_VERIFY_ERROR;
	}
	return result;
}
