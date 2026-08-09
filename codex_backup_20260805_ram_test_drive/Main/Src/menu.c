/*
 * menu.c
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#include "menu.h"
#include "button.h"
#include "custom_lcd.h"
#include "marker.h"
#include "motor.h"
#include "run_log_flash.h"
#include "sensor.h"
#include "st7789_lcd.h"
#include "test_drive.h"

#include <stdio.h>
#include <string.h>

#define MENU_SCREEN_WIDTH       240U
#define MENU_SCREEN_HEIGHT      135U
#define MENU_VISIBLE_ITEMS      5U
#define MENU_ITEM_HEIGHT        18U
#define MENU_ITEM_START_Y       22U
#define MENU_FOOTER_Y           121U
#define SENSOR_RAW_UPDATE_MS    100U
#define SENSOR_BAR_BASE_Y       113U
#define SENSOR_BAR_MAX_HEIGHT   40U
#define SENSOR_BAR_WIDTH        20U
#define SENSOR_BAR_PITCH        29U
#define SENSOR_VALUE_PITCH      56U
#define SENSOR_STATUS_TIMEOUT   500U
#define SENSOR_NORMAL_UPDATE_MS 100U
#define SENSOR_STATE_UPDATE_MS  100U
#define SENSOR_STATE_BOX_Y      48U
#define SENSOR_STATE_BOX_WIDTH  22U
#define SENSOR_STATE_BOX_HEIGHT 28U
#define SENSOR_STATE_TRACK_X    10U
#define SENSOR_STATE_TRACK_Y    112U
#define SENSOR_STATE_TRACK_WIDTH 220U
#define MARK_DIAG_UPDATE_MS       50U
#define MARK_DIAG_LINE_COUNT       6U
#define MARK_DIAG_TEXT_LENGTH     40U
#define MOTOR_PHASE_TIMEOUT_MS   5000U
#define MOTOR_PHASE_BOX_Y        64U
#define MOTOR_PHASE_BOX_WIDTH    34U
#define MOTOR_PHASE_BOX_HEIGHT   28U
#define MOTOR_SPEED_TIMEOUT_MS   3000U
#define MOTOR_SPEED_BAR_X        10U
#define MOTOR_SPEED_BAR_Y        68U
#define MOTOR_SPEED_BAR_WIDTH    32U
#define MOTOR_SPEED_BAR_HEIGHT   16U
#define MOTOR_SPEED_BAR_PITCH    36U
#define TEST_DRIVE_COUNTDOWN_MS  3000U
#define CALIBRATION_WARMUP_MS   100U
#define CALIBRATION_CAPTURE_MS  8000U
#define CALIBRATION_UPDATE_MS   100U
#define CALIBRATION_BAR_X       8U
#define CALIBRATION_BAR_Y       98U
#define CALIBRATION_BAR_WIDTH   224U
#define CALIBRATION_BAR_HEIGHT  8U

typedef enum {
	MENU_VIEW_MAIN = 0,
	MENU_VIEW_SENSOR_RAW,
	MENU_VIEW_CALIBRATION,
	MENU_VIEW_SENSOR_NORMAL,
	MENU_VIEW_SENSOR_STATE,
	MENU_VIEW_MARK_DIAGNOSTIC,
	MENU_VIEW_MOTOR_PHASE,
	MENU_VIEW_MOTOR_SPEED,
	MENU_VIEW_TEST_DRIVE,
	MENU_VIEW_DETAIL
} MenuView_t;

typedef enum {
	CALIBRATION_READY = 0,
	CALIBRATION_COLLECTING,
	CALIBRATION_DONE
} CalibrationState_t;

static const char *const menu_items[] = {
	"SENSOR RAW",
	"CALIBRATION",
	"SENSOR NORMAL",
	"SENSOR STATE",
	"MARK DIAGNOSTIC",
	"MOTOR PHASE",
	"MOTOR SPEED",
	"TEST DRIVE",
	"FIRST DRIVE",
	"SECOND DRIVE"
};

#define MENU_ITEM_COUNT ((uint8_t)(sizeof(menu_items) / sizeof(menu_items[0])))

static MenuView_t current_view = MENU_VIEW_MAIN;
static uint8_t selected_item = 0U;
static uint8_t first_visible_item = 0U;
static uint32_t sensor_raw_update_tick = 0U;
static uint32_t sensor_last_sample_count = 0U;
static uint32_t sensor_last_sample_tick = 0U;
static uint16_t sensor_displayed_raw[SENSOR_COUNT];
static uint16_t sensor_displayed_bar_height[SENSOR_COUNT];
static uint16_t sensor_displayed_bar_color[SENSOR_COUNT];
static uint8_t sensor_displayed_status = 0xFFU;
static uint8_t sensor_displayed_pin_status = 0xFFU;
static SensorLightMode_t sensor_light_mode = SENSOR_LIGHT_PAIR;
static uint8_t sensor_light_parameter = 0U;
static CalibrationState_t calibration_state = CALIBRATION_READY;
static uint32_t calibration_start_tick = 0U;
static uint32_t calibration_update_tick = 0U;
static uint32_t calibration_last_sample_count = 0U;
static uint16_t calibration_displayed_range[SENSOR_COUNT];
static uint32_t sensor_normal_update_tick = 0U;
static uint32_t sensor_normal_last_sample_count = 0U;
static uint32_t sensor_normal_last_sample_tick = 0U;
static uint16_t sensor_displayed_normal[SENSOR_COUNT];
static uint16_t sensor_normal_bar_height[SENSOR_COUNT];
static uint16_t sensor_normal_bar_color[SENSOR_COUNT];
static uint8_t sensor_normal_displayed_status = 0xFFU;
static uint32_t sensor_state_update_tick = 0U;
static uint32_t sensor_state_last_sample_count = 0U;
static uint32_t sensor_state_last_sample_tick = 0U;
static uint16_t sensor_state_displayed_mask = 0xFFFFU;
static int16_t sensor_state_displayed_position = INT16_MIN;
static uint8_t sensor_state_displayed_status = 0xFFU;
static uint8_t sensor_state_displayed_lost = 0xFFU;
static uint16_t sensor_state_marker_x = 0xFFFFU;
static MarkerDetector_t mark_diag_detector;
static uint32_t mark_diag_frame_cursor = 0U;
static uint32_t mark_diag_dropped_frames = 0U;
static uint32_t mark_diag_update_tick = 0U;
static uint32_t mark_diag_last_frame_tick = 0U;
static uint8_t mark_diag_state_mask = 0U;
static uint8_t mark_diag_current_center_count = 0U;
static uint8_t mark_diag_displayed_status = 0xFFU;
static char mark_diag_display_lines[MARK_DIAG_LINE_COUNT]
		[MARK_DIAG_TEXT_LENGTH];
static uint16_t mark_diag_display_colors[MARK_DIAG_LINE_COUNT];
static MotorTarget_t motor_phase_target = MOTOR_TARGET_LEFT;
static uint8_t motor_phase_index = 0U;
static uint8_t motor_phase_armed = 0U;
static uint8_t motor_phase_error = 0U;
static uint32_t motor_phase_action_tick = 0U;
static const uint16_t motor_speed_steps[] = {
	100U, 150U, 200U
};
#define MOTOR_SPEED_LEVEL_COUNT ((uint8_t)(sizeof(motor_speed_steps) \
		/ sizeof(motor_speed_steps[0])))
static uint8_t motor_speed_level = 0U;
static uint8_t motor_speed_running = 0U;
static uint8_t motor_speed_error = 0U;
static uint32_t motor_speed_action_tick = 0U;
typedef enum {
	TEST_DRIVE_UI_READY = 0,
	TEST_DRIVE_UI_COUNTDOWN,
	TEST_DRIVE_UI_RUNNING,
	TEST_DRIVE_UI_RESULT
} TestDriveUiState_t;
static const uint16_t test_drive_speed_steps[] = { 150U, 175U, 200U };
#define TEST_DRIVE_SPEED_LEVEL_COUNT ((uint8_t)(sizeof(test_drive_speed_steps) \
		/ sizeof(test_drive_speed_steps[0])))
static TestDriveUiState_t test_drive_ui_state = TEST_DRIVE_UI_READY;
static uint8_t test_drive_speed_level = 0U;
static uint8_t test_drive_sensor0_is_left = 1U;
static uint32_t test_drive_countdown_end_tick = 0U;
static uint8_t test_drive_countdown_display = 0xFFU;
static RunLogFlashStatus_t test_drive_flash_status = RUN_LOG_FLASH_OK;
static TestDriveStatus_t test_drive_last_status;
static RunLogHeader_t test_drive_saved_header;

static void Menu_FillScreen(uint16_t color)
{
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 0U,
			MENU_SCREEN_WIDTH, MENU_SCREEN_HEIGHT, color);
}

static void Menu_DrawText(uint16_t x, uint16_t y, uint8_t size,
		uint16_t foreground, uint16_t background, const char *text)
{
	uint16_t old_point_color = ST7789_WRAPPER_POINT_COLOR;
	uint16_t old_back_color = ST7789_WRAPPER_BACK_COLOR;

	ST7789_WRAPPER_POINT_COLOR = foreground;
	ST7789_WRAPPER_BACK_COLOR = background;
	ST7789_WRAPPER_ShowString(x, y, MENU_SCREEN_WIDTH - x,
			size, size, (uint8_t *)text);
	ST7789_WRAPPER_POINT_COLOR = old_point_color;
	ST7789_WRAPPER_BACK_COLOR = old_back_color;
}

static void Menu_UpdateVisibleWindow(void)
{
	if (selected_item < first_visible_item) {
		first_visible_item = selected_item;
	} else if (selected_item >= (first_visible_item + MENU_VISIBLE_ITEMS)) {
		first_visible_item = selected_item - (MENU_VISIBLE_ITEMS - 1U);
	}
}

static void Menu_DrawMain(void)
{
	char text[32];
	uint8_t row;

	Menu_FillScreen(LCD_COLOR_BLACK);

	snprintf(text, sizeof(text), "LINE TRACER  %u/%u",
			(unsigned int)(selected_item + 1U), (unsigned int)MENU_ITEM_COUNT);
	Menu_DrawText(8U, 2U, 16U, LCD_COLOR_CYAN, LCD_COLOR_BLACK, text);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 19U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	for (row = 0U; row < MENU_VISIBLE_ITEMS; row++) {
		uint8_t item = first_visible_item + row;
		uint16_t y = MENU_ITEM_START_Y + ((uint16_t)row * MENU_ITEM_HEIGHT);
		uint16_t foreground = LCD_COLOR_WHITE;
		uint16_t background = LCD_COLOR_BLACK;

		if (item >= MENU_ITEM_COUNT) {
			break;
		}

		if (item == selected_item) {
			foreground = LCD_COLOR_BLACK;
			background = LCD_COLOR_CYAN;
			ST7789_LCD_Driver.FillRect(&st7789_pObj, 2U, y - 1U,
					MENU_SCREEN_WIDTH - 4U, MENU_ITEM_HEIGHT, background);
		}

		snprintf(text, sizeof(text), "%c %u. %s",
				(item == selected_item) ? '>' : ' ',
				(unsigned int)(item + 1U), menu_items[item]);
		Menu_DrawText(8U, y, 16U, foreground, background, text);
	}

	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "L/R MOVE   C SELECT");
}

static void Menu_DrawDetail(void)
{
	char text[32];

	Menu_FillScreen(LCD_COLOR_BLACK);

	snprintf(text, sizeof(text), "%u. %s",
			(unsigned int)(selected_item + 1U), menu_items[selected_item]);
	Menu_DrawText(8U, 6U, 16U, LCD_COLOR_CYAN, LCD_COLOR_BLACK, text);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 25U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	Menu_DrawText(8U, 43U, 16U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, "READY FOR NEXT STEP");
	Menu_DrawText(8U, 70U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "OUTPUTS ARE NOT STARTED");
	Menu_DrawText(8U, 91U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "SAFE PLACEHOLDER SCREEN");
	Menu_DrawText(8U, 118U, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
}

static void Menu_UpdateSensorRaw(uint8_t force_update)
{
	char text[40];
	uint16_t raw[SENSOR_COUNT];
	uint32_t now = HAL_GetTick();
	uint32_t samples;
	uint8_t status;
	uint8_t index;

	if ((force_update == 0U)
			&& ((now - sensor_raw_update_tick) < SENSOR_RAW_UPDATE_MS)) {
		return;
	}
	sensor_raw_update_tick = now;

	Sensor_GetRaw(raw);
	samples = Sensor_GetSampleCount();
	if (!Sensor_IsRunning()) {
		status = 3U;
	} else if (samples != sensor_last_sample_count) {
		sensor_last_sample_count = samples;
		sensor_last_sample_tick = now;
		status = 1U;
	} else if (samples == 0U) {
		status = 0U;
	} else if ((now - sensor_last_sample_tick) >= SENSOR_STATUS_TIMEOUT) {
		status = 2U;
	} else {
		status = 1U;
	}

	if (status != sensor_displayed_status) {
		const char *status_text;
		uint16_t status_color;

		if (status == 0U) {
			status_text = "WAIT";
			status_color = LCD_COLOR_YELLOW;
		} else if (status == 1U) {
			status_text = "LIVE";
			status_color = LCD_COLOR_GREEN;
		} else if (status == 2U) {
			status_text = "STOP";
			status_color = LCD_COLOR_RED;
		} else {
			status_text = "ERROR";
			status_color = LCD_COLOR_RED;
		}

		ST7789_LCD_Driver.FillRect(&st7789_pObj, 180U, 2U,
				56U, 16U, LCD_COLOR_BLACK);
		Menu_DrawText(184U, 4U, 12U,
				status_color, LCD_COLOR_BLACK, status_text);
		sensor_displayed_status = status;
	}

	if (sensor_light_mode == SENSOR_LIGHT_FIXED) {
		uint8_t pin_status = Sensor_GetLightPinStatus(sensor_light_parameter);

		if (pin_status != sensor_displayed_pin_status) {
			uint16_t pin_color = ((pin_status & 0x07U) == 0x07U)
					? LCD_COLOR_GREEN : LCD_COLOR_RED;

			ST7789_LCD_Driver.FillRect(&st7789_pObj, 142U, 23U,
					94U, 12U, LCD_COLOR_BLACK);
			snprintf(text, sizeof(text), "M%u O%u I%u",
					(unsigned int)((pin_status >> 0U) & 0x01U),
					(unsigned int)((pin_status >> 1U) & 0x01U),
					(unsigned int)((pin_status >> 2U) & 0x01U));
			Menu_DrawText(146U, 23U, 12U,
					pin_color, LCD_COLOR_BLACK, text);
			sensor_displayed_pin_status = pin_status;
		}
	}

	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t value = raw[index];
		uint16_t height;
		uint16_t color;
		uint16_t x = 8U + ((uint16_t)index * SENSOR_BAR_PITCH);
		uint16_t value_x = 8U + ((uint16_t)(index & 0x03U)
				* SENSOR_VALUE_PITCH);
		uint16_t value_y = 36U + ((uint16_t)(index >> 2U) * 16U);

		if (value != sensor_displayed_raw[index]) {
			snprintf(text, sizeof(text), "%u:%4u",
					(unsigned int)index, (unsigned int)value);
			Menu_DrawText(value_x, value_y, 12U,
					LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
			sensor_displayed_raw[index] = value;
		}

		if (value > SENSOR_RAW_MAX) {
			value = SENSOR_RAW_MAX;
		}
		height = (uint16_t)(((uint32_t)value * SENSOR_BAR_MAX_HEIGHT)
				/ SENSOR_RAW_MAX);

		if (value < (SENSOR_RAW_MAX / 3U)) {
			color = LCD_COLOR_GREEN;
		} else if (value < ((SENSOR_RAW_MAX * 2U) / 3U)) {
			color = LCD_COLOR_YELLOW;
		} else {
			color = LCD_COLOR_RED;
		}

		if (color != sensor_displayed_bar_color[index]) {
			ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
					SENSOR_BAR_BASE_Y - SENSOR_BAR_MAX_HEIGHT,
					SENSOR_BAR_WIDTH, SENSOR_BAR_MAX_HEIGHT,
					LCD_COLOR_BLACK);
			if (height > 0U) {
				ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
						SENSOR_BAR_BASE_Y - height, SENSOR_BAR_WIDTH,
						height, color);
			}
		} else if (height > sensor_displayed_bar_height[index]) {
			ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
					SENSOR_BAR_BASE_Y - height, SENSOR_BAR_WIDTH,
					height - sensor_displayed_bar_height[index], color);
		} else if (height < sensor_displayed_bar_height[index]) {
			ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
					SENSOR_BAR_BASE_Y - sensor_displayed_bar_height[index],
					SENSOR_BAR_WIDTH,
					sensor_displayed_bar_height[index] - height,
					LCD_COLOR_BLACK);
		}

		sensor_displayed_bar_height[index] = height;
		sensor_displayed_bar_color[index] = color;
	}
}

static void Menu_ResetSensorRawDisplay(void)
{
	uint8_t index;

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 35U,
			MENU_SCREEN_WIDTH - 8U, 80U, LCD_COLOR_BLACK);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 6U, SENSOR_BAR_BASE_Y,
			MENU_SCREEN_WIDTH - 12U, 1U, LCD_COLOR_GRAY);

	sensor_raw_update_tick = 0U;
	sensor_last_sample_count = 0U;
	sensor_last_sample_tick = HAL_GetTick();
	sensor_displayed_status = 0xFFU;
	sensor_displayed_pin_status = 0xFFU;
	for (index = 0U; index < SENSOR_COUNT; index++) {
		sensor_displayed_raw[index] = 0xFFFFU;
		sensor_displayed_bar_height[index] = 0U;
		sensor_displayed_bar_color[index] = LCD_COLOR_BLACK;
	}

	Menu_UpdateSensorRaw(1U);
}

static void Menu_DrawSensorLightMode(void)
{
	char text[32];

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 23U,
			MENU_SCREEN_WIDTH - 8U, 12U, LCD_COLOR_BLACK);
	if (sensor_light_mode == SENSOR_LIGHT_PAIR) {
		snprintf(text, sizeof(text), "PAIR OFFSET +%u",
				(unsigned int)sensor_light_parameter);
	} else if (sensor_light_mode == SENSOR_LIGHT_OFF) {
		snprintf(text, sizeof(text), "AMBIENT  IR OFF");
	} else {
		snprintf(text, sizeof(text), "FIXED PAIR %u+%u",
				(unsigned int)sensor_light_parameter,
				(unsigned int)(7U - sensor_light_parameter));
	}

	Menu_DrawText(8U, 23U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, text);
}

static void Menu_ApplySensorLightMode(void)
{
	Sensor_SetLightMode(sensor_light_mode, sensor_light_parameter);
	Menu_DrawSensorLightMode();
	Menu_ResetSensorRawDisplay();
}

static void Menu_DrawSensorRaw(void)
{
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "1. SENSOR RAW");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);
	Menu_DrawSensorLightMode();
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "L/R SET  C MODE  HOLD BACK");
	Menu_ResetSensorRawDisplay();
}

static void Menu_ChangeSensorParameter(int8_t direction)
{
	uint8_t parameter_count;

	if (sensor_light_mode == SENSOR_LIGHT_OFF) {
		return;
	}
	parameter_count = (sensor_light_mode == SENSOR_LIGHT_FIXED)
			? SENSOR_EMITTER_PAIR_COUNT : SENSOR_COUNT;

	if (direction < 0) {
		sensor_light_parameter = (sensor_light_parameter == 0U)
				? (parameter_count - 1U) : (sensor_light_parameter - 1U);
	} else {
		sensor_light_parameter = (sensor_light_parameter + 1U)
				% parameter_count;
	}

	Menu_ApplySensorLightMode();
}

static void Menu_ChangeSensorLightMode(void)
{
	if (sensor_light_mode == SENSOR_LIGHT_PAIR) {
		sensor_light_mode = SENSOR_LIGHT_OFF;
	} else if (sensor_light_mode == SENSOR_LIGHT_OFF) {
		sensor_light_mode = SENSOR_LIGHT_FIXED;
		sensor_light_parameter = 0U;
	} else {
		sensor_light_mode = SENSOR_LIGHT_PAIR;
		sensor_light_parameter = 0U;
	}

	Menu_ApplySensorLightMode();
}

static void Menu_DrawCalibrationHeader(void)
{
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "2. CALIBRATION");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);
}

static void Menu_DrawCalibrationReady(void)
{
	Menu_DrawCalibrationHeader();
	Menu_DrawText(8U, 31U, 16U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, "SWEEP ACROSS WHITE LINE");
	Menu_DrawText(8U, 53U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "ALL SENSORS: BLACK + WHITE");
	Menu_DrawText(8U, 72U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "MOVE LEFT <-> RIGHT FOR 8 SEC");
	Menu_DrawText(8U, 94U, 16U,
			LCD_COLOR_GREEN, LCD_COLOR_BLACK, "PRESS C TO START");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
}

static void Menu_DrawCalibrationCollecting(void)
{
	uint8_t index;

	Menu_DrawCalibrationHeader();
	Menu_DrawText(8U, 76U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "SWEEP LEFT/RIGHT - RANGE MUST GROW");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, CALIBRATION_BAR_X,
			CALIBRATION_BAR_Y, CALIBRATION_BAR_WIDTH,
			CALIBRATION_BAR_HEIGHT, LCD_COLOR_GRAY);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, CALIBRATION_BAR_X + 1U,
			CALIBRATION_BAR_Y + 1U, CALIBRATION_BAR_WIDTH - 2U,
			CALIBRATION_BAR_HEIGHT - 2U, LCD_COLOR_BLACK);
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "C FINISH EARLY   HOLD BACK");
	calibration_update_tick = 0U;
	for (index = 0U; index < SENSOR_COUNT; index++) {
		calibration_displayed_range[index] = 0xFFFFU;
	}
}

static void Menu_UpdateCalibrationCollecting(uint8_t force_update)
{
	char text[32];
	uint16_t low[SENSOR_COUNT];
	uint16_t high[SENSOR_COUNT];
	uint32_t now = HAL_GetTick();
	uint32_t elapsed = now - calibration_start_tick;
	uint32_t active_elapsed = 0U;
	uint32_t samples = Sensor_GetSampleCount();
	uint16_t progress_width;
	uint8_t index;

	if ((elapsed >= CALIBRATION_WARMUP_MS)
			&& (samples != calibration_last_sample_count)) {
		Sensor_CalibrationCapture();
		calibration_last_sample_count = samples;
	}

	if (elapsed > CALIBRATION_WARMUP_MS) {
		active_elapsed = elapsed - CALIBRATION_WARMUP_MS;
	}
	if (active_elapsed > CALIBRATION_CAPTURE_MS) {
		active_elapsed = CALIBRATION_CAPTURE_MS;
	}

	if ((force_update == 0U)
			&& ((now - calibration_update_tick) < CALIBRATION_UPDATE_MS)) {
		return;
	}
	calibration_update_tick = now;
	Sensor_GetCalibration(low, high);

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 8U, 25U,
			112U, 14U, LCD_COLOR_BLACK);
	snprintf(text, sizeof(text), "CAPTURE %u.%us",
			(unsigned int)((CALIBRATION_CAPTURE_MS - active_elapsed) / 1000U),
			(unsigned int)(((CALIBRATION_CAPTURE_MS - active_elapsed) % 1000U)
					/ 100U));
	Menu_DrawText(8U, 25U, 12U,
			LCD_COLOR_GREEN, LCD_COLOR_BLACK, text);

	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t range = (low[index] == UINT16_MAX)
				? 0U : (uint16_t)(high[index] - low[index]);
		uint16_t x = 8U + ((uint16_t)(index & 0x03U) * 56U);
		uint16_t y = 43U + ((uint16_t)(index >> 2U) * 16U);
		uint16_t color = (range >= SENSOR_CALIBRATION_MIN_RANGE)
				? LCD_COLOR_GREEN : LCD_COLOR_YELLOW;

		if (range != calibration_displayed_range[index]) {
			snprintf(text, sizeof(text), "%u:%4u",
					(unsigned int)index, (unsigned int)range);
			Menu_DrawText(x, y, 12U, color, LCD_COLOR_BLACK, text);
			calibration_displayed_range[index] = range;
		}
	}

	progress_width = (uint16_t)(((uint32_t)(CALIBRATION_BAR_WIDTH - 2U)
			* active_elapsed) / CALIBRATION_CAPTURE_MS);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, CALIBRATION_BAR_X + 1U,
			CALIBRATION_BAR_Y + 1U, CALIBRATION_BAR_WIDTH - 2U,
			CALIBRATION_BAR_HEIGHT - 2U, LCD_COLOR_BLACK);
	if (progress_width > 0U) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj, CALIBRATION_BAR_X + 1U,
				CALIBRATION_BAR_Y + 1U, progress_width,
				CALIBRATION_BAR_HEIGHT - 2U, LCD_COLOR_GREEN);
	}
}

static void Menu_DrawCalibrationResult(uint8_t valid_mask)
{
	char text[32];
	uint16_t low[SENSOR_COUNT];
	uint16_t high[SENSOR_COUNT];
	uint8_t valid_count = 0U;
	uint8_t index;

	Sensor_GetCalibration(low, high);
	Menu_DrawCalibrationHeader();
	for (index = 0U; index < SENSOR_COUNT; index++) {
		if ((valid_mask & (1U << index)) != 0U) {
			valid_count++;
		}
	}
	snprintf(text, sizeof(text), "%s  %u/8 OK",
			(valid_count == SENSOR_COUNT) ? "DONE" : "RETRY",
			(unsigned int)valid_count);
	Menu_DrawText(8U, 25U, 16U,
			(valid_count == SENSOR_COUNT) ? LCD_COLOR_GREEN : LCD_COLOR_RED,
			LCD_COLOR_BLACK, text);

	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t x = (index < 4U) ? 8U : 124U;
		uint16_t y = 48U + ((uint16_t)(index & 0x03U) * 15U);
		uint16_t color = ((valid_mask & (1U << index)) != 0U)
				? LCD_COLOR_WHITE : LCD_COLOR_RED;
		uint16_t shown_low = (low[index] == UINT16_MAX) ? 0U : low[index];

		snprintf(text, sizeof(text), "%u:%4u-%4u",
				(unsigned int)index, (unsigned int)shown_low,
				(unsigned int)high[index]);
		Menu_DrawText(x, y, 12U, color, LCD_COLOR_BLACK, text);
	}

	Menu_DrawText(8U, 108U, 12U,
			(valid_count == SENSOR_COUNT) ? LCD_COLOR_GREEN : LCD_COLOR_YELLOW,
			LCD_COLOR_BLACK,
			(valid_count == SENSOR_COUNT) ? "RAM CAL ACTIVE" : "MOVE MORE ON RETRY");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "C RETRY   HOLD C BACK");
}

static void Menu_StartCalibration(void)
{
	sensor_light_mode = SENSOR_LIGHT_PAIR;
	sensor_light_parameter = 0U;
	Sensor_SetLightMode(sensor_light_mode, sensor_light_parameter);
	Sensor_CalibrationReset();
	Sensor_Start();
	calibration_start_tick = HAL_GetTick();
	calibration_last_sample_count = Sensor_GetSampleCount();
	calibration_state = CALIBRATION_COLLECTING;
	Menu_DrawCalibrationCollecting();
	Menu_UpdateCalibrationCollecting(1U);
}

static void Menu_FinishCalibration(void)
{
	uint8_t valid_mask;

	Sensor_CalibrationCapture();
	valid_mask = Sensor_CalibrationFinish();
	Sensor_Stop();
	calibration_state = CALIBRATION_DONE;
	Menu_DrawCalibrationResult(valid_mask);
}

static void Menu_UpdateSensorNormal(uint8_t force_update)
{
	char text[32];
	uint16_t normalized[SENSOR_COUNT];
	uint32_t now = HAL_GetTick();
	uint32_t samples;
	uint8_t status;
	uint8_t index;

	if (!Sensor_IsCalibrationComplete()) {
		return;
	}
	if ((force_update == 0U)
			&& ((now - sensor_normal_update_tick) < SENSOR_NORMAL_UPDATE_MS)) {
		return;
	}
	sensor_normal_update_tick = now;
	Sensor_GetNormalized(normalized);
	samples = Sensor_GetSampleCount();
	if (!Sensor_IsRunning()) {
		status = 3U;
	} else if (samples != sensor_normal_last_sample_count) {
		sensor_normal_last_sample_count = samples;
		sensor_normal_last_sample_tick = now;
		status = 1U;
	} else if (samples == 0U) {
		status = 0U;
	} else if ((now - sensor_normal_last_sample_tick)
			>= SENSOR_STATUS_TIMEOUT) {
		status = 2U;
	} else {
		status = 1U;
	}

	if (status != sensor_normal_displayed_status) {
		const char *status_text;
		uint16_t status_color;

		if (status == 0U) {
			status_text = "WAIT";
			status_color = LCD_COLOR_YELLOW;
		} else if (status == 1U) {
			status_text = "LIVE";
			status_color = LCD_COLOR_GREEN;
		} else if (status == 2U) {
			status_text = "STOP";
			status_color = LCD_COLOR_RED;
		} else {
			status_text = "ERROR";
			status_color = LCD_COLOR_RED;
		}

		ST7789_LCD_Driver.FillRect(&st7789_pObj, 180U, 2U,
				56U, 16U, LCD_COLOR_BLACK);
		Menu_DrawText(184U, 4U, 12U,
				status_color, LCD_COLOR_BLACK, status_text);
		sensor_normal_displayed_status = status;
	}

	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t value = normalized[index];
		uint16_t height;
		uint16_t color;
		uint16_t x = 8U + ((uint16_t)index * SENSOR_BAR_PITCH);
		uint16_t value_x = 8U + ((uint16_t)(index & 0x03U)
				* SENSOR_VALUE_PITCH);
		uint16_t value_y = 36U + ((uint16_t)(index >> 2U) * 16U);

		if (value != sensor_displayed_normal[index]) {
			snprintf(text, sizeof(text), "%u:%4u",
					(unsigned int)index, (unsigned int)value);
			Menu_DrawText(value_x, value_y, 12U,
					LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
			sensor_displayed_normal[index] = value;
		}

		if (value > SENSOR_NORMALIZED_MAX) {
			value = SENSOR_NORMALIZED_MAX;
		}
		height = (uint16_t)(((uint32_t)value * SENSOR_BAR_MAX_HEIGHT)
				/ SENSOR_NORMALIZED_MAX);
		if (value < 250U) {
			color = LCD_COLOR_GRAY;
		} else if (value < 650U) {
			color = LCD_COLOR_CYAN;
		} else {
			color = LCD_COLOR_WHITE;
		}

		if (color != sensor_normal_bar_color[index]) {
			ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
					SENSOR_BAR_BASE_Y - SENSOR_BAR_MAX_HEIGHT,
					SENSOR_BAR_WIDTH, SENSOR_BAR_MAX_HEIGHT,
					LCD_COLOR_BLACK);
			if (height > 0U) {
				ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
						SENSOR_BAR_BASE_Y - height, SENSOR_BAR_WIDTH,
						height, color);
			}
		} else if (height > sensor_normal_bar_height[index]) {
			ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
					SENSOR_BAR_BASE_Y - height, SENSOR_BAR_WIDTH,
					height - sensor_normal_bar_height[index], color);
		} else if (height < sensor_normal_bar_height[index]) {
			ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
					SENSOR_BAR_BASE_Y - sensor_normal_bar_height[index],
					SENSOR_BAR_WIDTH,
					sensor_normal_bar_height[index] - height,
					LCD_COLOR_BLACK);
		}

		sensor_normal_bar_height[index] = height;
		sensor_normal_bar_color[index] = color;
	}
}

static void Menu_ResetSensorNormalDisplay(void)
{
	uint8_t index;

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 35U,
			MENU_SCREEN_WIDTH - 8U, 80U, LCD_COLOR_BLACK);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 6U, SENSOR_BAR_BASE_Y,
			MENU_SCREEN_WIDTH - 12U, 1U, LCD_COLOR_GRAY);
	sensor_normal_update_tick = 0U;
	sensor_normal_last_sample_count = 0U;
	sensor_normal_last_sample_tick = HAL_GetTick();
	sensor_normal_displayed_status = 0xFFU;
	for (index = 0U; index < SENSOR_COUNT; index++) {
		sensor_displayed_normal[index] = 0xFFFFU;
		sensor_normal_bar_height[index] = 0U;
		sensor_normal_bar_color[index] = LCD_COLOR_BLACK;
	}
	Menu_UpdateSensorNormal(1U);
}

static void Menu_DrawSensorNormal(void)
{
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "3. SENSOR NORMAL");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	if (!Sensor_IsCalibrationComplete()) {
		Menu_DrawText(8U, 38U, 16U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "CALIBRATION REQUIRED");
		Menu_DrawText(8U, 68U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "PRESS C TO OPEN CALIBRATION");
		Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
				LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
		return;
	}

	Menu_DrawText(8U, 23U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "BLACK 0       WHITE 1000");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
	Menu_ResetSensorNormalDisplay();
}

static void Menu_UpdateSensorState(uint8_t force_update)
{
	char text[32];
	uint32_t now = HAL_GetTick();
	uint32_t samples;
	uint8_t status;
	uint8_t state_mask;
	uint8_t lost;
	uint8_t index;
	int16_t position = 0;
	int16_t displayed_position;

	if (!Sensor_IsCalibrationComplete()) {
		return;
	}
	if ((force_update == 0U)
			&& ((now - sensor_state_update_tick) < SENSOR_STATE_UPDATE_MS)) {
		return;
	}
	sensor_state_update_tick = now;
	samples = Sensor_GetSampleCount();
	if (!Sensor_IsRunning()) {
		status = 3U;
	} else if (samples != sensor_state_last_sample_count) {
		sensor_state_last_sample_count = samples;
		sensor_state_last_sample_tick = now;
		status = 1U;
	} else if (samples == 0U) {
		status = 0U;
	} else if ((now - sensor_state_last_sample_tick)
			>= SENSOR_STATUS_TIMEOUT) {
		status = 2U;
	} else {
		status = 1U;
	}

	if (status != sensor_state_displayed_status) {
		const char *status_text;
		uint16_t status_color;

		if (status == 0U) {
			status_text = "WAIT";
			status_color = LCD_COLOR_YELLOW;
		} else if (status == 1U) {
			status_text = "LIVE";
			status_color = LCD_COLOR_GREEN;
		} else if (status == 2U) {
			status_text = "STOP";
			status_color = LCD_COLOR_RED;
		} else {
			status_text = "ERROR";
			status_color = LCD_COLOR_RED;
		}

		ST7789_LCD_Driver.FillRect(&st7789_pObj, 180U, 2U,
				56U, 16U, LCD_COLOR_BLACK);
		Menu_DrawText(184U, 4U, 12U,
				status_color, LCD_COLOR_BLACK, status_text);
		sensor_state_displayed_status = status;
	}

	state_mask = Sensor_UpdateState(&position);
	lost = (state_mask == 0U) ? 1U : 0U;
	if (state_mask != sensor_state_displayed_mask) {
		for (index = 0U; index < SENSOR_COUNT; index++) {
			uint8_t bit = (uint8_t)(1U << index);
			uint8_t is_white = ((state_mask & bit) != 0U) ? 1U : 0U;
			uint16_t x = 7U + ((uint16_t)index * SENSOR_BAR_PITCH);
			uint16_t box_color = is_white
					? LCD_COLOR_WHITE : LCD_COLOR_GREEN_DARK;

			if ((sensor_state_displayed_mask == 0xFFFFU)
					|| ((state_mask ^ sensor_state_displayed_mask) & bit)) {
				ST7789_LCD_Driver.FillRect(&st7789_pObj, x,
						SENSOR_STATE_BOX_Y, SENSOR_STATE_BOX_WIDTH,
						SENSOR_STATE_BOX_HEIGHT, box_color);
				ST7789_LCD_Driver.FillRect(&st7789_pObj, x + 2U,
						SENSOR_STATE_BOX_Y + 2U,
						SENSOR_STATE_BOX_WIDTH - 4U,
						SENSOR_STATE_BOX_HEIGHT - 4U,
						is_white ? LCD_COLOR_WHITE : LCD_COLOR_BLACK);
				Menu_DrawText(x + 7U, SENSOR_STATE_BOX_Y + 8U, 12U,
						is_white ? LCD_COLOR_BLACK : LCD_COLOR_GREEN,
						is_white ? LCD_COLOR_WHITE : LCD_COLOR_BLACK,
						is_white ? "W" : "B");
			}
		}

		for (index = 0U; index < SENSOR_COUNT; index++) {
			text[index] = ((state_mask & (1U << index)) != 0U)
					? '1' : '0';
		}
		text[SENSOR_COUNT] = '\0';
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 80U,
				232U, 14U, LCD_COLOR_BLACK);
		Menu_DrawText(8U, 80U, 12U,
				LCD_COLOR_WHITE, LCD_COLOR_BLACK, "S0->S7");
		Menu_DrawText(80U, 80U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK, text);
		sensor_state_displayed_mask = state_mask;
	}

	/* Ten-unit display steps keep the numeric field readable while live. */
	displayed_position = (int16_t)((position / 10) * 10);
	if ((lost != sensor_state_displayed_lost)
			|| (!lost && (displayed_position
					!= sensor_state_displayed_position))) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 96U,
				232U, 14U, LCD_COLOR_BLACK);
		if (lost) {
			Menu_DrawText(8U, 96U, 12U,
					LCD_COLOR_RED, LCD_COLOR_BLACK, "LINE LOST");
		} else {
			snprintf(text, sizeof(text), "LINE POS %+5d",
					(int)displayed_position);
			Menu_DrawText(8U, 96U, 12U,
					LCD_COLOR_CYAN, LCD_COLOR_BLACK, text);
		}
		sensor_state_displayed_position = displayed_position;
		sensor_state_displayed_lost = lost;
	}

	if (sensor_state_marker_x != 0xFFFFU) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj,
				sensor_state_marker_x, SENSOR_STATE_TRACK_Y - 3U,
				5U, 7U, LCD_COLOR_BLACK);
		ST7789_LCD_Driver.FillRect(&st7789_pObj,
				SENSOR_STATE_TRACK_X, SENSOR_STATE_TRACK_Y,
				SENSOR_STATE_TRACK_WIDTH, 1U, LCD_COLOR_GRAY);
	}
	if (!lost) {
		uint16_t marker_x = SENSOR_STATE_TRACK_X
				+ (uint16_t)(((int32_t)(position - SENSOR_LINE_POSITION_MIN)
						* (SENSOR_STATE_TRACK_WIDTH - 5U))
						/ (SENSOR_LINE_POSITION_MAX
								- SENSOR_LINE_POSITION_MIN));

		ST7789_LCD_Driver.FillRect(&st7789_pObj, marker_x,
				SENSOR_STATE_TRACK_Y - 3U, 5U, 7U, LCD_COLOR_YELLOW);
		sensor_state_marker_x = marker_x;
	} else {
		sensor_state_marker_x = 0xFFFFU;
	}
}

static void Menu_ResetSensorStateDisplay(void)
{
	uint8_t index;

	sensor_state_update_tick = 0U;
	sensor_state_last_sample_count = 0U;
	sensor_state_last_sample_tick = HAL_GetTick();
	sensor_state_displayed_mask = 0xFFFFU;
	sensor_state_displayed_position = INT16_MIN;
	sensor_state_displayed_status = 0xFFU;
	sensor_state_displayed_lost = 0xFFU;
	sensor_state_marker_x = 0xFFFFU;
	Sensor_StateReset();
	for (index = 0U; index < SENSOR_COUNT; index++) {
		char text[2] = {(char)('0' + index), '\0'};
		Menu_DrawText(14U + ((uint16_t)index * SENSOR_BAR_PITCH),
				34U, 12U, LCD_COLOR_GRAY, LCD_COLOR_BLACK, text);
	}
	ST7789_LCD_Driver.FillRect(&st7789_pObj,
			SENSOR_STATE_TRACK_X, SENSOR_STATE_TRACK_Y,
			SENSOR_STATE_TRACK_WIDTH, 1U, LCD_COLOR_GRAY);
	Menu_UpdateSensorState(1U);
}

static void Menu_DrawSensorState(void)
{
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "4. SENSOR STATE");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	if (!Sensor_IsCalibrationComplete()) {
		Menu_DrawText(8U, 38U, 16U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "CALIBRATION REQUIRED");
		Menu_DrawText(8U, 68U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"PRESS C TO OPEN CALIBRATION");
		Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
				LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
		return;
	}

	Menu_DrawText(8U, 23U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "WHITE ON 600  OFF 400");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
	Menu_ResetSensorStateDisplay();
}

static const char *Menu_GetMarkerEventName(MarkerEventType_t type)
{
	if (type == MARKER_EVENT_EDGE_0) {
		return "S0";
	}
	if (type == MARKER_EVENT_EDGE_7) {
		return "S7";
	}
	if (type == MARKER_EVENT_BOTH) {
		return "BOTH";
	}
	if (type == MARKER_EVENT_CROSS) {
		return "CROSS";
	}
	if (type == MARKER_EVENT_UNKNOWN) {
		return "UNKNOWN";
	}
	return "NONE";
}

static uint16_t Menu_GetMarkerEventColor(MarkerEventType_t type)
{
	if (type == MARKER_EVENT_CROSS) {
		return LCD_COLOR_CYAN;
	}
	if (type == MARKER_EVENT_BOTH) {
		return LCD_COLOR_WHITE;
	}
	if (type == MARKER_EVENT_EDGE_0) {
		return LCD_COLOR_ORANGE;
	}
	if (type == MARKER_EVENT_EDGE_7) {
		return LCD_COLOR_GREEN;
	}
	if (type == MARKER_EVENT_UNKNOWN) {
		return LCD_COLOR_RED;
	}
	return LCD_COLOR_GRAY;
}

static void Menu_DrawMarkDiagnosticLine(uint8_t line, uint16_t y,
		uint16_t color, const char *text)
{
	if (line >= MARK_DIAG_LINE_COUNT) {
		return;
	}
	if ((mark_diag_display_colors[line] == color)
			&& (strcmp(mark_diag_display_lines[line], text) == 0)) {
		return;
	}

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, y,
			MENU_SCREEN_WIDTH - 8U, 14U, LCD_COLOR_BLACK);
	Menu_DrawText(8U, y, 12U, color, LCD_COLOR_BLACK, text);
	strncpy(mark_diag_display_lines[line], text,
			MARK_DIAG_TEXT_LENGTH - 1U);
	mark_diag_display_lines[line][MARK_DIAG_TEXT_LENGTH - 1U] = '\0';
	mark_diag_display_colors[line] = color;
}

static uint8_t Menu_ProcessMarkDiagnosticFrames(void)
{
	uint16_t raw[SENSOR_COUNT];
	uint16_t normalized[SENSOR_COUNT];
	uint8_t event_updated = 0U;
	uint8_t processed = 0U;

	while (1) {
		uint32_t previous_cursor = mark_diag_frame_cursor;

		if (!Sensor_GetFrameRawAfter(&mark_diag_frame_cursor, raw)) {
			break;
		}
		if (mark_diag_frame_cursor > (previous_cursor + 1U)) {
			mark_diag_dropped_frames += mark_diag_frame_cursor
					- previous_cursor - 1U;
		}

		Sensor_NormalizeFrame(raw, normalized);
		mark_diag_state_mask = Sensor_ApplyStateHysteresis(normalized,
				mark_diag_state_mask);
		mark_diag_current_center_count =
				Marker_CountCenter(mark_diag_state_mask);
		if (MarkerDetector_Update(&mark_diag_detector,
				mark_diag_state_mask, mark_diag_frame_cursor)) {
			event_updated = 1U;
		}
		processed = 1U;
	}

	if (processed != 0U) {
		mark_diag_last_frame_tick = HAL_GetTick();
	}
	return event_updated;
}

static void Menu_UpdateMarkDiagnostic(uint8_t force_update)
{
	char text[MARK_DIAG_TEXT_LENGTH];
	char full_bits[SENSOR_COUNT + 1U];
	char center_bits[7];
	char edge_text[3];
	uint32_t now = HAL_GetTick();
	uint32_t frame_count;
	uint8_t status;
	uint8_t index;
	uint8_t event_updated;
	const MarkerEvent_t *event = &mark_diag_detector.last_event;

	if (!Sensor_IsCalibrationComplete()) {
		return;
	}
	event_updated = Menu_ProcessMarkDiagnosticFrames();
	if ((force_update == 0U) && (event_updated == 0U)
			&& ((now - mark_diag_update_tick) < MARK_DIAG_UPDATE_MS)) {
		return;
	}
	mark_diag_update_tick = now;
	frame_count = Sensor_GetFrameCount();

	if (!Sensor_IsRunning()) {
		status = 3U;
	} else if (frame_count == 0U) {
		status = 0U;
	} else if ((now - mark_diag_last_frame_tick) >= SENSOR_STATUS_TIMEOUT) {
		status = 2U;
	} else {
		status = 1U;
	}
	if (status != mark_diag_displayed_status) {
		const char *status_text;
		uint16_t status_color;

		if (status == 0U) {
			status_text = "WAIT";
			status_color = LCD_COLOR_YELLOW;
		} else if (status == 1U) {
			status_text = "LIVE";
			status_color = LCD_COLOR_GREEN;
		} else if (status == 2U) {
			status_text = "STOP";
			status_color = LCD_COLOR_RED;
		} else {
			status_text = "ERROR";
			status_color = LCD_COLOR_RED;
		}
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 180U, 2U,
				56U, 16U, LCD_COLOR_BLACK);
		Menu_DrawText(184U, 4U, 12U, status_color,
				LCD_COLOR_BLACK, status_text);
		mark_diag_displayed_status = status;
	}

	for (index = 0U; index < SENSOR_COUNT; index++) {
		full_bits[index] = ((mark_diag_state_mask & (1U << index)) != 0U)
				? '1' : '0';
		if ((index >= 1U) && (index <= 6U)) {
			center_bits[index - 1U] = full_bits[index];
		}
	}
	full_bits[SENSOR_COUNT] = '\0';
	center_bits[6] = '\0';
	edge_text[0] = ((mark_diag_state_mask & 0x01U) != 0U) ? '0' : '-';
	edge_text[1] = ((mark_diag_state_mask & 0x80U) != 0U) ? '7' : '-';
	edge_text[2] = '\0';

	snprintf(text, sizeof(text), "FULL %s  MID %s",
			full_bits, center_bits);
	Menu_DrawMarkDiagnosticLine(0U, 24U, LCD_COLOR_WHITE, text);
	snprintf(text, sizeof(text), "MID N%u MAX%u  EDGE %s",
			(unsigned int)mark_diag_current_center_count,
			(unsigned int)mark_diag_detector.max_center_count, edge_text);
	Menu_DrawMarkDiagnosticLine(1U, 40U,
			(mark_diag_current_center_count >= MARKER_WIDE_CENTER_COUNT)
					? LCD_COLOR_CYAN : LCD_COLOR_GRAY, text);
	snprintf(text, sizeof(text), "FSM %-7s F%08lu",
			mark_diag_detector.collecting ? "COLLECT" : "IDLE",
			(unsigned long)frame_count);
	Menu_DrawMarkDiagnosticLine(2U, 56U,
			mark_diag_detector.collecting
					? LCD_COLOR_YELLOW : LCD_COLOR_GREEN, text);
	snprintf(text, sizeof(text), "LAST %03lu %-7s",
			(unsigned long)mark_diag_detector.event_count,
			Menu_GetMarkerEventName(event->type));
	Menu_DrawMarkDiagnosticLine(3U, 72U,
			Menu_GetMarkerEventColor(event->type), text);
	snprintf(text, sizeof(text), "U%02X E%02X M%u W%u",
			(unsigned int)event->full_union,
			(unsigned int)event->edge_union,
			(unsigned int)event->max_center_count,
			(unsigned int)event->max_wide_run);
	Menu_DrawMarkDiagnosticLine(4U, 88U,
			Menu_GetMarkerEventColor(event->type), text);
	snprintf(text, sizeof(text), "DUR %lums  DROP %lu",
			(unsigned long)event->duration_frames,
			(unsigned long)mark_diag_dropped_frames);
	Menu_DrawMarkDiagnosticLine(5U, 104U,
			(mark_diag_dropped_frames == 0U)
					? LCD_COLOR_GRAY : LCD_COLOR_RED, text);
}

static void Menu_ResetMarkDiagnostic(void)
{
	uint8_t line;

	MarkerDetector_Init(&mark_diag_detector);
	mark_diag_frame_cursor = Sensor_GetFrameCount();
	mark_diag_dropped_frames = 0U;
	mark_diag_update_tick = 0U;
	mark_diag_last_frame_tick = HAL_GetTick();
	mark_diag_state_mask = 0U;
	mark_diag_current_center_count = 0U;
	mark_diag_displayed_status = 0xFFU;
	for (line = 0U; line < MARK_DIAG_LINE_COUNT; line++) {
		mark_diag_display_lines[line][0] = '\0';
		mark_diag_display_colors[line] = 0xFFFFU;
	}
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 23U,
			MENU_SCREEN_WIDTH - 8U, 96U, LCD_COLOR_BLACK);
	Menu_UpdateMarkDiagnostic(1U);
}

static void Menu_DrawMarkDiagnostic(void)
{
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "5. MARK DIAGNOSTIC");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	if (!Sensor_IsCalibrationComplete()) {
		Menu_DrawText(8U, 38U, 16U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "CALIBRATION REQUIRED");
		Menu_DrawText(8U, 68U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"PRESS C TO OPEN CALIBRATION");
		Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
				LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
		return;
	}

	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "C RESET   HOLD C BACK");
	Menu_ResetMarkDiagnostic();
}

static const char *Menu_GetMotorTargetName(MotorTarget_t target)
{
	if (target == MOTOR_TARGET_RIGHT) {
		return "RIGHT";
	}
	if (target == MOTOR_TARGET_BOTH) {
		return "BOTH";
	}
	return "LEFT";
}

static void Menu_DrawMotorPhase(void)
{
	char text[32];
	uint8_t pattern = Motor_GetPhasePattern(motor_phase_index);
	uint8_t index;

	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "6. MOTOR PHASE");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	if (motor_phase_error != 0U) {
		Menu_DrawText(176U, 4U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "ERROR");
	} else if (motor_phase_armed != 0U) {
		Menu_DrawText(176U, 4U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "ARMED");
	} else {
		Menu_DrawText(164U, 4U, 12U,
				LCD_COLOR_GREEN, LCD_COLOR_BLACK, "SAFE OFF");
	}

	snprintf(text, sizeof(text), "TARGET %-5s   PHASE %u/7",
			Menu_GetMotorTargetName(motor_phase_target),
			(unsigned int)motor_phase_index);
	Menu_DrawText(8U, 25U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
	Menu_DrawText(8U, 43U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "CONTROL ORDER: 1  3  2  4");

	for (index = 0U; index < 4U; index++) {
		uint16_t x = 31U + ((uint16_t)index * 48U);
		uint8_t active = ((pattern & (1U << index)) != 0U) ? 1U : 0U;
		uint16_t border = active ? LCD_COLOR_RED : LCD_COLOR_GREEN_DARK;
		uint16_t inside = active ? LCD_COLOR_RED_DARK : LCD_COLOR_BLACK;

		ST7789_LCD_Driver.FillRect(&st7789_pObj, x, MOTOR_PHASE_BOX_Y,
				MOTOR_PHASE_BOX_WIDTH, MOTOR_PHASE_BOX_HEIGHT, border);
		ST7789_LCD_Driver.FillRect(&st7789_pObj, x + 2U,
				MOTOR_PHASE_BOX_Y + 2U, MOTOR_PHASE_BOX_WIDTH - 4U,
				MOTOR_PHASE_BOX_HEIGHT - 4U, inside);
		Menu_DrawText(x + 13U, MOTOR_PHASE_BOX_Y + 8U, 12U,
				active ? LCD_COLOR_WHITE : LCD_COLOR_GREEN,
				inside, active ? "1" : "0");
	}

	if (motor_phase_error != 0U) {
		Menu_DrawText(8U, 96U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "DAC START FAILED - OUTPUT OFF");
	} else if (motor_phase_armed != 0U) {
		Menu_DrawText(8U, 96U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"L/R STEP  C OFF  AUTO OFF 5S");
	} else {
		Menu_DrawText(8U, 96U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"L/R TARGET  C ARM  DAC 512");
	}
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C EMERGENCY BACK");
}

static void Menu_ChangeMotorPhaseTarget(int8_t direction)
{
	if (motor_phase_armed != 0U) {
		return;
	}

	if (direction < 0) {
		motor_phase_target = (motor_phase_target == MOTOR_TARGET_LEFT)
				? MOTOR_TARGET_BOTH
				: (MotorTarget_t)(motor_phase_target - 1);
	} else {
		motor_phase_target = (motor_phase_target == MOTOR_TARGET_BOTH)
				? MOTOR_TARGET_LEFT
				: (MotorTarget_t)(motor_phase_target + 1);
	}
	motor_phase_error = 0U;
	Menu_DrawMotorPhase();
}

static void Menu_ChangeMotorPhase(int8_t direction)
{
	if (motor_phase_armed == 0U) {
		return;
	}

	if (direction < 0) {
		motor_phase_index = (motor_phase_index == 0U)
				? (MOTOR_PHASE_COUNT - 1U) : (motor_phase_index - 1U);
	} else {
		motor_phase_index = (motor_phase_index + 1U) % MOTOR_PHASE_COUNT;
	}
	Motor_PhaseTestSetPhase(motor_phase_index);
	motor_phase_action_tick = HAL_GetTick();
	Menu_DrawMotorPhase();
}

static void Menu_ToggleMotorPhaseArm(void)
{
	if (motor_phase_armed != 0U) {
		Motor_PhaseTestDisarm();
		motor_phase_armed = 0U;
		motor_phase_error = 0U;
	} else if (Motor_PhaseTestArm(motor_phase_target,
			motor_phase_index, MOTOR_PHASE_TEST_DAC)) {
		motor_phase_armed = 1U;
		motor_phase_error = 0U;
		motor_phase_action_tick = HAL_GetTick();
	} else {
		motor_phase_armed = 0U;
		motor_phase_error = 1U;
	}
	Menu_DrawMotorPhase();
}

static void Menu_UpdateMotorPhaseSafety(void)
{
	if ((motor_phase_armed != 0U)
			&& ((HAL_GetTick() - motor_phase_action_tick)
					>= MOTOR_PHASE_TIMEOUT_MS)) {
		Motor_PhaseTestDisarm();
		motor_phase_armed = 0U;
		motor_phase_error = 0U;
		Menu_DrawMotorPhase();
	}
}

static void Menu_DrawMotorSpeed(void)
{
	char text[40];
	uint8_t index;
	uint16_t speed = motor_speed_steps[motor_speed_level];

	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "7. MOTOR SPEED");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	if (motor_speed_error != 0U) {
		Menu_DrawText(176U, 4U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "ERROR");
	} else if (motor_speed_running != 0U) {
		Menu_DrawText(188U, 4U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "RUN");
	} else {
		Menu_DrawText(164U, 4U, 12U,
				LCD_COLOR_GREEN, LCD_COLOR_BLACK, "SAFE OFF");
	}

	Menu_DrawText(8U, 25U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, "FORWARD  PHASE L-  R+");
	snprintf(text, sizeof(text), "SPEED %4u HALFSTEP/S  LV %u/%u",
			(unsigned int)speed, (unsigned int)(motor_speed_level + 1U),
			(unsigned int)MOTOR_SPEED_LEVEL_COUNT);
	Menu_DrawText(8U, 43U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);

	for (index = 0U; index < MOTOR_SPEED_LEVEL_COUNT; index++) {
		uint16_t color = (index <= motor_speed_level)
				? LCD_COLOR_YELLOW : LCD_COLOR_GREEN_DARK;
		ST7789_LCD_Driver.FillRect(&st7789_pObj,
				MOTOR_SPEED_BAR_X + ((uint16_t)index * MOTOR_SPEED_BAR_PITCH),
				MOTOR_SPEED_BAR_Y, MOTOR_SPEED_BAR_WIDTH,
				MOTOR_SPEED_BAR_HEIGHT, color);
	}

	if (motor_speed_error != 0U) {
		Menu_DrawText(8U, 92U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "START FAILED - OUTPUT OFF");
	} else {
		snprintf(text, sizeof(text), "DAC %u   AUTO OFF %uS",
				(unsigned int)MOTOR_SPEED_TEST_DAC,
				(unsigned int)(MOTOR_SPEED_TIMEOUT_MS / 1000U));
		Menu_DrawText(8U, 92U, 12U,
				LCD_COLOR_GRAY, LCD_COLOR_BLACK, text);
	}
	Menu_DrawText(8U, 106U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
			(motor_speed_running != 0U)
					? "C STOP   L/R LOCKED WHILE RUN"
					: "L/R SPEED   C RUN");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C EMERGENCY BACK");
}

static void Menu_ChangeMotorSpeed(int8_t direction)
{
	if (motor_speed_running != 0U) {
		return;
	}

	if (direction < 0) {
		if (motor_speed_level > 0U) {
			motor_speed_level--;
		}
	} else if (motor_speed_level < (MOTOR_SPEED_LEVEL_COUNT - 1U)) {
		motor_speed_level++;
	}
	motor_speed_error = 0U;
	Menu_DrawMotorSpeed();
}

static void Menu_ToggleMotorSpeed(void)
{
	if (motor_speed_running != 0U) {
		Motor_SpeedTestStop();
		motor_speed_running = 0U;
		motor_speed_error = 0U;
	} else if (Motor_SpeedTestStart(motor_speed_steps[motor_speed_level],
			MOTOR_SPEED_TEST_DAC)) {
		motor_speed_running = 1U;
		motor_speed_error = 0U;
		motor_speed_action_tick = HAL_GetTick();
	} else {
		motor_speed_running = 0U;
		motor_speed_error = 1U;
	}
	Menu_DrawMotorSpeed();
}

static void Menu_UpdateMotorSpeedSafety(void)
{
	if ((motor_speed_running != 0U)
			&& ((HAL_GetTick() - motor_speed_action_tick)
					>= MOTOR_SPEED_TIMEOUT_MS)) {
		Motor_SpeedTestStop();
		motor_speed_running = 0U;
		motor_speed_error = 0U;
		Menu_DrawMotorSpeed();
	}
}

static void Menu_DrawTestDriveReady(void)
{
	char text[40];

	Motor_DriveStop();
	Sensor_Stop();
	test_drive_ui_state = TEST_DRIVE_UI_READY;

	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "8. TEST DRIVE");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	if (!Sensor_IsCalibrationComplete()) {
		Menu_DrawText(8U, 37U, 16U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "CALIBRATION REQUIRED");
		Menu_DrawText(8U, 68U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"PRESS C TO OPEN CALIBRATION");
		Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
				LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
		return;
	}

	Menu_DrawText(166U, 4U, 12U,
			LCD_COLOR_GREEN, LCD_COLOR_BLACK, "SAFE OFF");
	Menu_DrawText(8U, 25U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK,
			"LOW SPEED LOGGING - NO MARK ACTION");
	snprintf(text, sizeof(text), "SPEED %u SPS   MAP S0=%s",
			(unsigned int)test_drive_speed_steps[test_drive_speed_level],
			test_drive_sensor0_is_left != 0U ? "LEFT" : "RIGHT");
	Menu_DrawText(8U, 42U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, text);
	Menu_DrawText(8U, 58U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "LIMIT 60S  SAMPLE 10MS");
	/* Never touch the reserved flash-log bank while entering this menu.
	 * Flash access is deferred until after a completed, safely stopped run. */
	Menu_DrawText(8U, 74U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "FLASH CHECK DEFERRED");
	Menu_DrawText(8U, 91U, 12U,
			LCD_COLOR_ORANGE, LCD_COLOR_BLACK,
			"L MAP   R SPEED   C 3S START");
	Menu_DrawText(8U, 106U, 12U,
			LCD_COLOR_RED, LCD_COLOR_BLACK, "NEXT RUN REPLACES OLD LOG");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
}

static void Menu_DrawTestDriveCountdown(uint8_t seconds)
{
	char text[24];

	if (seconds == test_drive_countdown_display) {
		return;
	}
	test_drive_countdown_display = seconds;
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "8. TEST DRIVE");
	Menu_DrawText(180U, 4U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "ARMED");
	snprintf(text, sizeof(text), "START IN %u", (unsigned int)seconds);
	Menu_DrawText(48U, 42U, 24U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, text);
	Menu_DrawText(24U, 80U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK,
			"PLACE VEHICLE ON WHITE LINE");
	Menu_DrawText(35U, 102U, 12U,
			LCD_COLOR_RED, LCD_COLOR_BLACK, "ANY BUTTON CANCELS");
}

static void Menu_DrawTestDriveRunning(void)
{
	char text[40];

	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "8. TEST DRIVE");
	Menu_DrawText(188U, 4U, 12U,
			LCD_COLOR_RED, LCD_COLOR_BLACK, "RUN");
	Menu_DrawText(17U, 31U, 16U,
			LCD_COLOR_RED, LCD_COLOR_BLACK, "PRESS C = EMERGENCY STOP");
	snprintf(text, sizeof(text), "SPEED %u SPS  S0=%s",
			(unsigned int)test_drive_speed_steps[test_drive_speed_level],
			test_drive_sensor0_is_left != 0U ? "LEFT" : "RIGHT");
	Menu_DrawText(40U, 59U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, text);
	Menu_DrawText(29U, 79U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK,
			"WAIT LINE, THEN MOTOR STARTS");
	Menu_DrawText(34U, 102U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK,
			"LCD FROZEN WHILE MOVING");
}

static void Menu_DrawTestDriveResult(void)
{
	char text[40];
	uint16_t save_color = (test_drive_flash_status == RUN_LOG_FLASH_OK)
			? LCD_COLOR_GREEN : LCD_COLOR_RED;

	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "8. TEST DRIVE");
	Menu_DrawText(174U, 4U, 12U, save_color, LCD_COLOR_BLACK,
			test_drive_flash_status == RUN_LOG_FLASH_OK ? "SAVED" : "ERROR");
	snprintf(text, sizeof(text), "STOP %-12s",
			TestDrive_GetStopReasonName(test_drive_last_status.stop_reason));
	Menu_DrawText(8U, 26U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, text);
	snprintf(text, sizeof(text), "TIME %lums   REC %lu",
			(unsigned long)test_drive_last_status.elapsed_ms,
			(unsigned long)test_drive_last_status.record_count);
	Menu_DrawText(8U, 43U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
	snprintf(text, sizeof(text), "EVENT %lu   DROP %lu",
			(unsigned long)test_drive_last_status.event_count,
			(unsigned long)test_drive_last_status.dropped_frames);
	Menu_DrawText(8U, 60U, 12U,
			test_drive_last_status.dropped_frames == 0U
					? LCD_COLOR_GREEN : LCD_COLOR_RED,
			LCD_COLOR_BLACK, text);
	if (test_drive_flash_status == RUN_LOG_FLASH_OK) {
		snprintf(text, sizeof(text), "CRC %08lX  FLASH OK",
				(unsigned long)test_drive_saved_header.data_crc32);
		Menu_DrawText(8U, 79U, 12U,
				LCD_COLOR_GREEN, LCD_COLOR_BLACK, text);
	} else {
		snprintf(text, sizeof(text), "FLASH ERROR CODE %u",
				(unsigned int)test_drive_flash_status);
		Menu_DrawText(8U, 79U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, text);
	}
	Menu_DrawText(8U, 101U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK,
			"C READY   LOG SURVIVES RESET");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
}

static void Menu_SaveTestDriveResult(void)
{
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "8. TEST DRIVE");
	Menu_DrawText(74U, 48U, 20U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "SAVING LOG");
	Menu_DrawText(38U, 82U, 12U,
			LCD_COLOR_RED, LCD_COLOR_BLACK, "DO NOT TURN POWER OFF");

	Motor_DriveStop();
	Sensor_Stop();
	TestDrive_GetStatus(&test_drive_last_status);
	test_drive_flash_status = RunLogFlash_Save();
	if ((test_drive_flash_status == RUN_LOG_FLASH_OK)
			&& !RunLogFlash_ReadHeader(&test_drive_saved_header, true)) {
		test_drive_flash_status = RUN_LOG_FLASH_VERIFY_ERROR;
	}
	test_drive_ui_state = TEST_DRIVE_UI_RESULT;
	Menu_DrawTestDriveResult();
}

static void Menu_BeginTestDriveCountdown(void)
{
	Motor_DriveStop();
	Sensor_Stop();
	test_drive_ui_state = TEST_DRIVE_UI_COUNTDOWN;
	test_drive_countdown_end_tick = HAL_GetTick()
			+ TEST_DRIVE_COUNTDOWN_MS;
	test_drive_countdown_display = 0xFFU;
	Menu_DrawTestDriveCountdown(3U);
}

static void Menu_UpdateTestDriveCountdown(UserInput_t input)
{
	uint32_t now = HAL_GetTick();
	uint32_t remaining;
	uint8_t seconds;

	if (input != INPUT_CMD_NONE) {
		Menu_DrawTestDriveReady();
		return;
	}
	if ((int32_t)(test_drive_countdown_end_tick - now) <= 0) {
		Menu_DrawTestDriveRunning();
		if (TestDrive_Start(
				test_drive_speed_steps[test_drive_speed_level],
				test_drive_sensor0_is_left != 0U)) {
			test_drive_ui_state = TEST_DRIVE_UI_RUNNING;
		} else {
			TestDrive_GetStatus(&test_drive_last_status);
			test_drive_flash_status = RUN_LOG_FLASH_CONFIG_ERROR;
			test_drive_ui_state = TEST_DRIVE_UI_RESULT;
			Menu_DrawTestDriveResult();
		}
		return;
	}
	remaining = test_drive_countdown_end_tick - now;
	seconds = (uint8_t)((remaining + 999U) / 1000U);
	Menu_DrawTestDriveCountdown(seconds);
}

static void Menu_ProcessTestDrive(UserInput_t input)
{
	if (test_drive_ui_state == TEST_DRIVE_UI_READY) {
		if (input == INPUT_CMD_K_HOLD) {
			Motor_DriveStop();
			Sensor_Stop();
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (!Sensor_IsCalibrationComplete()
				&& (input == INPUT_CMD_K_SINGLE)) {
			calibration_state = CALIBRATION_READY;
			current_view = MENU_VIEW_CALIBRATION;
			Menu_DrawCalibrationReady();
		} else if (input == INPUT_CMD_L_SINGLE) {
			test_drive_sensor0_is_left ^= 1U;
			Menu_DrawTestDriveReady();
		} else if (input == INPUT_CMD_R_SINGLE) {
			test_drive_speed_level = (test_drive_speed_level + 1U)
					% TEST_DRIVE_SPEED_LEVEL_COUNT;
			Menu_DrawTestDriveReady();
		} else if (input == INPUT_CMD_K_SINGLE) {
			Menu_BeginTestDriveCountdown();
		}
	} else if (test_drive_ui_state == TEST_DRIVE_UI_COUNTDOWN) {
		Menu_UpdateTestDriveCountdown(input);
	} else if (test_drive_ui_state == TEST_DRIVE_UI_RUNNING) {
		if (Button_IsCenterPressed()) {
			TestDrive_RequestStop(RUN_LOG_STOP_USER);
		} else {
			TestDrive_Process();
		}
		if (!TestDrive_IsActive()) {
			Menu_SaveTestDriveResult();
		}
	} else if (input == INPUT_CMD_K_HOLD) {
		Motor_DriveStop();
		Sensor_Stop();
		current_view = MENU_VIEW_MAIN;
		Menu_DrawMain();
	} else if (input == INPUT_CMD_K_SINGLE) {
		Menu_DrawTestDriveReady();
	}
}

static void Menu_MoveSelection(int8_t direction)
{
	if (direction < 0) {
		selected_item = (selected_item == 0U)
				? (MENU_ITEM_COUNT - 1U) : (selected_item - 1U);
	} else {
		selected_item = (selected_item + 1U) % MENU_ITEM_COUNT;
	}

	Menu_UpdateVisibleWindow();
	Menu_DrawMain();
}

void Main_Menu(void)
{
	static uint8_t init_done = 0U;
	UserInput_t input;

	if (init_done == 0U) {
		Custom_LCD_Init(LCD_TYPE_ST7789);
		Menu_DrawMain();
		init_done = 1U;
	}

	input = Button_Get_Input();
	if (current_view == MENU_VIEW_MAIN) {
		if ((input == INPUT_CMD_L_SINGLE) || (input == INPUT_CMD_L_HOLD)) {
			Menu_MoveSelection(-1);
		} else if ((input == INPUT_CMD_R_SINGLE) || (input == INPUT_CMD_R_HOLD)) {
			Menu_MoveSelection(1);
		} else if (input == INPUT_CMD_K_SINGLE) {
			if (selected_item == 0U) {
				sensor_light_mode = SENSOR_LIGHT_PAIR;
				sensor_light_parameter = 0U;
				Sensor_SetLightMode(sensor_light_mode,
						sensor_light_parameter);
				Sensor_Start();
				current_view = MENU_VIEW_SENSOR_RAW;
				Menu_DrawSensorRaw();
			} else if (selected_item == 1U) {
				Sensor_Stop();
				calibration_state = CALIBRATION_READY;
				current_view = MENU_VIEW_CALIBRATION;
				Menu_DrawCalibrationReady();
			} else if (selected_item == 2U) {
				Sensor_Stop();
				if (Sensor_IsCalibrationComplete()) {
					sensor_light_mode = SENSOR_LIGHT_PAIR;
					sensor_light_parameter = 0U;
					Sensor_SetLightMode(sensor_light_mode,
							sensor_light_parameter);
					Sensor_Start();
				}
				current_view = MENU_VIEW_SENSOR_NORMAL;
				Menu_DrawSensorNormal();
			} else if (selected_item == 3U) {
				Sensor_Stop();
				if (Sensor_IsCalibrationComplete()) {
					sensor_light_mode = SENSOR_LIGHT_PAIR;
					sensor_light_parameter = 0U;
					Sensor_SetLightMode(sensor_light_mode,
							sensor_light_parameter);
					Sensor_Start();
				}
				current_view = MENU_VIEW_SENSOR_STATE;
				Menu_DrawSensorState();
			} else if (selected_item == 4U) {
				Sensor_Stop();
				Motor_SpeedTestStop();
				if (Sensor_IsCalibrationComplete()) {
					sensor_light_mode = SENSOR_LIGHT_PAIR;
					sensor_light_parameter = 0U;
					Sensor_SetLightMode(sensor_light_mode,
							sensor_light_parameter);
					Sensor_Start();
				}
				current_view = MENU_VIEW_MARK_DIAGNOSTIC;
				Menu_DrawMarkDiagnostic();
			} else if (selected_item == 5U) {
				Sensor_Stop();
				Motor_PhaseTestDisarm();
				motor_phase_target = MOTOR_TARGET_LEFT;
				motor_phase_index = 0U;
				motor_phase_armed = 0U;
				motor_phase_error = 0U;
				current_view = MENU_VIEW_MOTOR_PHASE;
				Menu_DrawMotorPhase();
			} else if (selected_item == 6U) {
				Sensor_Stop();
				Motor_SpeedTestStop();
				motor_speed_level = 0U;
				motor_speed_running = 0U;
				motor_speed_error = 0U;
				current_view = MENU_VIEW_MOTOR_SPEED;
				Menu_DrawMotorSpeed();
			} else if (selected_item == 7U) {
				Sensor_Stop();
				Motor_DriveStop();
				test_drive_ui_state = TEST_DRIVE_UI_READY;
				current_view = MENU_VIEW_TEST_DRIVE;
				Menu_DrawTestDriveReady();
			} else {
				current_view = MENU_VIEW_DETAIL;
				Menu_DrawDetail();
			}
		}
	} else if (current_view == MENU_VIEW_SENSOR_RAW) {
		if (input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (input == INPUT_CMD_K_SINGLE) {
			Menu_ChangeSensorLightMode();
		} else if (input == INPUT_CMD_L_SINGLE) {
			Menu_ChangeSensorParameter(-1);
		} else if (input == INPUT_CMD_R_SINGLE) {
			Menu_ChangeSensorParameter(1);
		} else {
			Menu_UpdateSensorRaw(0U);
		}
	} else if (current_view == MENU_VIEW_CALIBRATION) {
		if (input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (input == INPUT_CMD_K_SINGLE) {
			if (calibration_state == CALIBRATION_COLLECTING) {
				Menu_FinishCalibration();
			} else {
				Menu_StartCalibration();
			}
		} else if (calibration_state == CALIBRATION_COLLECTING) {
			uint32_t elapsed = HAL_GetTick() - calibration_start_tick;

			Menu_UpdateCalibrationCollecting(0U);
			if (elapsed >= (CALIBRATION_WARMUP_MS
					+ CALIBRATION_CAPTURE_MS)) {
				Menu_FinishCalibration();
			}
		}
	} else if (current_view == MENU_VIEW_SENSOR_NORMAL) {
		if (input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (!Sensor_IsCalibrationComplete()
				&& (input == INPUT_CMD_K_SINGLE)) {
			Sensor_Stop();
			calibration_state = CALIBRATION_READY;
			current_view = MENU_VIEW_CALIBRATION;
			Menu_DrawCalibrationReady();
		} else {
			Menu_UpdateSensorNormal(0U);
		}
	} else if (current_view == MENU_VIEW_SENSOR_STATE) {
		if (input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (!Sensor_IsCalibrationComplete()
				&& (input == INPUT_CMD_K_SINGLE)) {
			Sensor_Stop();
			calibration_state = CALIBRATION_READY;
			current_view = MENU_VIEW_CALIBRATION;
			Menu_DrawCalibrationReady();
		} else {
			Menu_UpdateSensorState(0U);
		}
	} else if (current_view == MENU_VIEW_MARK_DIAGNOSTIC) {
		if (input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (!Sensor_IsCalibrationComplete()
				&& (input == INPUT_CMD_K_SINGLE)) {
			Sensor_Stop();
			calibration_state = CALIBRATION_READY;
			current_view = MENU_VIEW_CALIBRATION;
			Menu_DrawCalibrationReady();
		} else if (input == INPUT_CMD_K_SINGLE) {
			Menu_ResetMarkDiagnostic();
		} else {
			Menu_UpdateMarkDiagnostic(0U);
		}
	} else if (current_view == MENU_VIEW_MOTOR_PHASE) {
		if (input == INPUT_CMD_K_HOLD) {
			Motor_PhaseTestDisarm();
			motor_phase_armed = 0U;
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (input == INPUT_CMD_K_SINGLE) {
			Menu_ToggleMotorPhaseArm();
		} else if ((input == INPUT_CMD_L_SINGLE)
				|| (input == INPUT_CMD_L_HOLD)) {
			if (motor_phase_armed != 0U) {
				Menu_ChangeMotorPhase(-1);
			} else {
				Menu_ChangeMotorPhaseTarget(-1);
			}
		} else if ((input == INPUT_CMD_R_SINGLE)
				|| (input == INPUT_CMD_R_HOLD)) {
			if (motor_phase_armed != 0U) {
				Menu_ChangeMotorPhase(1);
			} else {
				Menu_ChangeMotorPhaseTarget(1);
			}
		} else {
			Menu_UpdateMotorPhaseSafety();
		}
	} else if (current_view == MENU_VIEW_MOTOR_SPEED) {
		if (input == INPUT_CMD_K_HOLD) {
			Motor_SpeedTestStop();
			motor_speed_running = 0U;
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (input == INPUT_CMD_K_SINGLE) {
			Menu_ToggleMotorSpeed();
		} else if ((input == INPUT_CMD_L_SINGLE)
				|| (input == INPUT_CMD_L_HOLD)) {
			Menu_ChangeMotorSpeed(-1);
		} else if ((input == INPUT_CMD_R_SINGLE)
				|| (input == INPUT_CMD_R_HOLD)) {
			Menu_ChangeMotorSpeed(1);
		} else {
			Menu_UpdateMotorSpeedSafety();
		}
	} else if (current_view == MENU_VIEW_TEST_DRIVE) {
		Menu_ProcessTestDrive(input);
	} else if (input == INPUT_CMD_K_HOLD) {
		current_view = MENU_VIEW_MAIN;
		Menu_DrawMain();
	}
}
