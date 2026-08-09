/*
 * run_log_flash.h
 */

#ifndef INC_RUN_LOG_FLASH_H_
#define INC_RUN_LOG_FLASH_H_

#include "run_log.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	RUN_LOG_FLASH_OK = 0,
	RUN_LOG_FLASH_CONFIG_ERROR,
	RUN_LOG_FLASH_UNLOCK_ERROR,
	RUN_LOG_FLASH_ERASE_ERROR,
	RUN_LOG_FLASH_PROGRAM_ERROR,
	RUN_LOG_FLASH_VERIFY_ERROR
} RunLogFlashStatus_t;

RunLogFlashStatus_t RunLogFlash_Save(void);
bool RunLogFlash_ReadHeader(RunLogHeader_t *header, bool verify_data);

#endif /* INC_RUN_LOG_FLASH_H_ */
