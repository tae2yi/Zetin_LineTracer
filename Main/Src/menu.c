/*
 * menu.c
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#include "menu.h"
#include "app_version.h"
#include "button.h"
#include "custom_lcd.h"
#include "drive.h"
#include "marker.h"
#include "motor.h"
#include "second_drive.h"
#include "sensor.h"
#include "st7789_lcd.h"
#include "track.h"

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
#define SENSOR_STATE_UPDATE_MS  100U
#define SENSOR_STATE_BOX_Y      48U
#define SENSOR_STATE_BOX_WIDTH  22U
#define SENSOR_STATE_BOX_HEIGHT 28U
#define SENSOR_STATE_TRACK_X    10U
#define SENSOR_STATE_TRACK_Y    112U
#define SENSOR_STATE_TRACK_WIDTH 220U
#define MOTOR_PHASE_TIMEOUT_MS   5000U
#define MOTOR_PHASE_BOX_Y        64U
#define MOTOR_PHASE_BOX_WIDTH    34U
#define MOTOR_PHASE_BOX_HEIGHT   28U
#define MOTOR_SPEED_ARM_TIMEOUT_MS 5000U
#define MOTOR_SPEED_TARGET_HOLD_MS 5000U
#define MOTOR_SPEED_UPDATE_MS     100U
#define FIRST_DRIVE_UPDATE_MS     100U
#define FIRST_DRIVE_ARM_TIMEOUT_MS 5000U
#define FIRST_DRIVE_LINE_COUNT       5U
#define FIRST_DRIVE_LINE_CHARS      37U
#define FIRST_DRIVE_FAULT_PAGE_COUNT 4U
#define FIRST_DRIVE_PREVIEW_X        10U
#define FIRST_DRIVE_PREVIEW_Y        62U
#define FIRST_DRIVE_PREVIEW_WIDTH   220U
#define SECOND_DRIVE_UPDATE_MS      100U
#define SECOND_DRIVE_ARM_TIMEOUT_MS 5000U
#define PD_TUNING_KP_STEP_Q10        10
#define PD_TUNING_KD_STEP_Q10         5
#define PD_TUNING_KP_MAX_Q10       1024
#define PD_TUNING_KD_MAX_Q10        512
#define PD_TUNING_TRIM_STEP_SPS       10
#define PD_TUNING_TRIM_MAX_SPS       300
#define MOTOR_SPEED_BAR_X        10U
#define MOTOR_SPEED_BAR_Y        78U
#define MOTOR_SPEED_BAR_WIDTH    48U
#define MOTOR_SPEED_BAR_HEIGHT   10U
#define MOTOR_SPEED_BAR_PITCH    55U
#define CALIBRATION_WARMUP_MS      500U
#define CALIBRATION_BLACK_MS      3000U
#define CALIBRATION_PREP_MS       1000U
#define CALIBRATION_WHITE_MIN_MS  8000U
#define CALIBRATION_WHITE_MAX_MS 12000U
#define CALIBRATION_BLACK_END_MS \
	(CALIBRATION_WARMUP_MS + CALIBRATION_BLACK_MS)
#define CALIBRATION_WHITE_START_MS \
	(CALIBRATION_BLACK_END_MS + CALIBRATION_PREP_MS)
#define CALIBRATION_TOTAL_MAX_MS \
	(CALIBRATION_WHITE_START_MS + CALIBRATION_WHITE_MAX_MS)
#define CALIBRATION_UPDATE_MS   100U
#define CALIBRATION_BAR_X       8U
#define CALIBRATION_BAR_Y       98U
#define CALIBRATION_BAR_WIDTH   224U
#define CALIBRATION_BAR_HEIGHT  8U

typedef enum {
	MENU_VIEW_MAIN = 0,
	MENU_VIEW_SENSOR_RAW,
	MENU_VIEW_CALIBRATION,
	MENU_VIEW_SENSOR_STATE,
	MENU_VIEW_MOTOR_PHASE,
	MENU_VIEW_MOTOR_SPEED,
	MENU_VIEW_PD_TUNING,
	MENU_VIEW_FIRST_DRIVE,
	MENU_VIEW_SECOND_DRIVE
} MenuView_t;

typedef enum {
	CALIBRATION_READY = 0,
	CALIBRATION_COLLECTING,
	CALIBRATION_DONE
} CalibrationState_t;

static const char *const menu_items[] = {
	"SENSOR RAW",
	"CALIBRATION",
	"SENSOR STATE",
	"MOTOR PHASE",
	"MOTOR SPEED",
	"PD TUNING",
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
static uint32_t calibration_frame_cursor = 0U;
static uint8_t calibration_black_finished = 0U;
static uint8_t calibration_auto_finish_ready = 0U;
static uint8_t calibration_displayed_phase = 0xFFU;
static uint8_t calibration_displayed_coverage = 0xFFU;
static uint16_t calibration_displayed_range[SENSOR_COUNT];
static uint32_t sensor_state_update_tick = 0U;
static uint32_t sensor_state_last_sample_count = 0U;
static uint32_t sensor_state_last_sample_tick = 0U;
static uint16_t sensor_state_displayed_mask = 0xFFFFU;
static int16_t sensor_state_displayed_position = INT16_MIN;
static uint8_t sensor_state_displayed_status = 0xFFU;
static uint8_t sensor_state_displayed_lost = 0xFFU;
static uint16_t sensor_state_marker_x = 0xFFFFU;
static MotorTarget_t motor_phase_target = MOTOR_TARGET_LEFT;
static uint8_t motor_phase_index = 0U;
static uint8_t motor_phase_armed = 0U;
static uint8_t motor_phase_error = 0U;
static uint32_t motor_phase_action_tick = 0U;
static const uint16_t motor_speed_steps[] = {
	4800U, 5200U, 5600U, 6000U
};
#define MOTOR_SPEED_LEVEL_COUNT ((uint8_t)(sizeof(motor_speed_steps) \
		/ sizeof(motor_speed_steps[0])))
static uint8_t motor_speed_level = 0U;
static uint8_t motor_speed_armed = 0U;
static uint8_t motor_speed_running = 0U;
static uint8_t motor_speed_error = 0U;
static uint32_t motor_speed_action_tick = 0U;
static uint32_t motor_speed_hold_tick = 0U;
static uint32_t motor_speed_update_tick = 0U;
static uint8_t pd_tuning_selected = 0U;
static uint32_t first_drive_arm_tick = 0U;
static uint32_t first_drive_update_tick = 0U;
static uint32_t first_drive_preview_tick = 0U;
static uint16_t first_drive_preview_marker_x = 0xFFFFU;
static uint8_t first_drive_preview_valid = 0xFFU;
static uint8_t first_drive_fault_page = 0U;
static FirstDriveState_t first_drive_displayed_state = FIRST_DRIVE_OFF;
static FirstDriveFault_t first_drive_displayed_fault = FIRST_DRIVE_FAULT_NONE;
static char first_drive_displayed_lines[FIRST_DRIVE_LINE_COUNT]
		[FIRST_DRIVE_LINE_CHARS + 1U];
static uint16_t first_drive_displayed_line_y[FIRST_DRIVE_LINE_COUNT];
static uint16_t first_drive_displayed_line_color[FIRST_DRIVE_LINE_COUNT];
static uint8_t first_drive_displayed_line_valid[FIRST_DRIVE_LINE_COUNT];
static uint8_t second_drive_selected = 0U;
static uint32_t second_drive_arm_tick = 0U;
static uint32_t second_drive_update_tick = 0U;

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
	char version[12];
	uint16_t version_x;
	uint8_t row;

	Menu_FillScreen(LCD_COLOR_BLACK);

	snprintf(text, sizeof(text), "LINE TRACER  %u/%u",
			(unsigned int)(selected_item + 1U), (unsigned int)MENU_ITEM_COUNT);
	Menu_DrawText(8U, 2U, 16U, LCD_COLOR_CYAN, LCD_COLOR_BLACK, text);
	snprintf(version, sizeof(version), "V%u", (unsigned int)APP_VERSION_NUMBER);
	version_x = MENU_SCREEN_WIDTH - 8U
			- ((uint16_t)strlen(version) * 8U);
	Menu_DrawText(version_x, 2U, 16U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, version);
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

#if 0 /* Removed legacy placeholder menu. */
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
#endif

static const char *Menu_FirstDriveStateName(FirstDriveState_t state)
{
	switch (state) {
	case FIRST_DRIVE_ARMED: return "ARMED";
	case FIRST_DRIVE_COUNTDOWN: return "WAIT 2S";
	case FIRST_DRIVE_LAUNCH: return "LAUNCH";
	case FIRST_DRIVE_FOLLOW: return "FOLLOW";
	case FIRST_DRIVE_TURN_LEFT: return "TURN L";
	case FIRST_DRIVE_TURN_RIGHT: return "TURN R";
	case FIRST_DRIVE_CROSS_PASS: return "CROSS";
	case FIRST_DRIVE_END_CANDIDATE: return "END?";
	case FIRST_DRIVE_RUNOUT: return "RUNOUT";
	case FIRST_DRIVE_STOPPED: return "STOP";
	case FIRST_DRIVE_FAULT: return "FAULT";
	case FIRST_DRIVE_READY: return "READY";
	default: return "OFF";
	}
}

static const char *Menu_FirstDriveFaultName(FirstDriveFault_t fault)
{
	switch (fault) {
	case FIRST_DRIVE_FAULT_NO_CALIBRATION: return "NO CAL";
	case FIRST_DRIVE_FAULT_START_NO_LINE: return "NO LINE";
	case FIRST_DRIVE_FAULT_SENSOR_STALE: return "SENSOR";
	case FIRST_DRIVE_FAULT_LINE_LOST: return "LINE LOST";
	case FIRST_DRIVE_FAULT_MOTOR_COMMAND: return "MOTOR";
	case FIRST_DRIVE_FAULT_CONTROL_TIMER: return "CTRL TIM";
	case FIRST_DRIVE_FAULT_TRACK_OVERFLOW: return "TRACK FULL";
	case FIRST_DRIVE_FAULT_EDGE_STUCK: return "EDGE STUCK";
	case FIRST_DRIVE_FAULT_NO_TRACK: return "NO TRACK";
	default: return "NONE";
	}
}

static const char *Menu_FirstDriveCourseName(FirstDriveCoursePhase_t phase)
{
	switch (phase) {
	case FIRST_DRIVE_COURSE_APPROACH_LEFT: return "AL";
	case FIRST_DRIVE_COURSE_APPROACH_RIGHT: return "AR";
	case FIRST_DRIVE_COURSE_TURN_LEFT: return "TL";
	case FIRST_DRIVE_COURSE_TURN_RIGHT: return "TR";
	case FIRST_DRIVE_COURSE_EXIT_LEFT: return "XL";
	case FIRST_DRIVE_COURSE_EXIT_RIGHT: return "XR";
	case FIRST_DRIVE_COURSE_CROSS: return "CR";
	default: return "ST";
	}
}

static const char *Menu_FirstDriveMarkerName(uint8_t marker_type)
{
	switch ((MarkerEventType_t)marker_type) {
	case MARKER_EVENT_EDGE_0: return "E0";
	case MARKER_EVENT_EDGE_7: return "E7";
	case MARKER_EVENT_BOTH: return "BT";
	case MARKER_EVENT_CROSS: return "CR";
	case MARKER_EVENT_UNKNOWN: return "UN";
	default: return "--";
	}
}

static const char *Menu_FirstDrivePhaseReasonName(
		FirstDrivePhaseReason_t reason)
{
	switch (reason) {
	case FIRST_DRIVE_PHASE_REASON_MARKER_PROVISIONAL: return "MPR";
	case FIRST_DRIVE_PHASE_REASON_MARKER_CONFIRMED: return "MCF";
	case FIRST_DRIVE_PHASE_REASON_POSITION_ENTER: return "POS";
	case FIRST_DRIVE_PHASE_REASON_APPROACH_EXPIRED: return "ATO";
	case FIRST_DRIVE_PHASE_REASON_MARKER_EXIT: return "MEX";
	case FIRST_DRIVE_PHASE_REASON_MARKER_STRAIGHT: return "MST";
	case FIRST_DRIVE_PHASE_REASON_CROSS_MARKER: return "CMK";
	case FIRST_DRIVE_PHASE_REASON_CROSS_EXPIRED: return "CTO";
	case FIRST_DRIVE_PHASE_REASON_EXIT_CENTERED: return "CEN";
	default: return "---";
	}
}

static void Menu_DrawPdTuning(void)
{
	char text[40];
	const FirstDriveConfig_t *config = FirstDrive_GetConfig();
	uint16_t kp_milli = (uint16_t)(((uint32_t)config->kp_q10 * 1000U) / 1024U);
	uint16_t kd_milli = (uint16_t)(((uint32_t)config->kd_q10 * 1000U) / 1024U);
	uint16_t kp_fg = (pd_tuning_selected == 0U)
			? LCD_COLOR_BLACK : LCD_COLOR_WHITE;
	uint16_t kp_bg = (pd_tuning_selected == 0U)
			? LCD_COLOR_CYAN : LCD_COLOR_BLACK;
	uint16_t kd_fg = (pd_tuning_selected == 1U)
			? LCD_COLOR_BLACK : LCD_COLOR_WHITE;
	uint16_t kd_bg = (pd_tuning_selected == 1U)
			? LCD_COLOR_CYAN : LCD_COLOR_BLACK;
	uint16_t trim_fg = (pd_tuning_selected == 2U)
			? LCD_COLOR_BLACK : LCD_COLOR_WHITE;
	uint16_t trim_bg = (pd_tuning_selected == 2U)
			? LCD_COLOR_CYAN : LCD_COLOR_BLACK;

	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "6. DRIVE TUNING");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 6U, 27U,
			228U, 22U, kp_bg);
	snprintf(text, sizeof(text), "KP %4ld /1024  = %u.%03u",
			(long)config->kp_q10, (unsigned int)(kp_milli / 1000U),
			(unsigned int)(kp_milli % 1000U));
	Menu_DrawText(12U, 30U, 16U, kp_fg, kp_bg, text);

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 6U, 53U,
			228U, 22U, kd_bg);
	snprintf(text, sizeof(text), "KD %4ld /1024  = %u.%03u",
			(long)config->kd_q10, (unsigned int)(kd_milli / 1000U),
			(unsigned int)(kd_milli % 1000U));
	Menu_DrawText(12U, 56U, 16U, kd_fg, kd_bg, text);

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 6U, 79U,
			228U, 22U, trim_bg);
	snprintf(text, sizeof(text), "TRIM %+4d SPS  (+ = L FAST)",
			(int)config->trim_sps);
	Menu_DrawText(12U, 82U, 12U, trim_fg, trim_bg, text);

	Menu_DrawText(8U, 108U, 12U, LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
			"L/R ADJUST   C NEXT");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
}

static void Menu_ChangePdGain(int8_t direction)
{
	const FirstDriveConfig_t *config = FirstDrive_GetConfig();
	int32_t kp_q10 = config->kp_q10;
	int32_t kd_q10 = config->kd_q10;
	int16_t trim_sps = config->trim_sps;

	if (pd_tuning_selected == 0U) {
		kp_q10 += (direction < 0) ? -PD_TUNING_KP_STEP_Q10
				: PD_TUNING_KP_STEP_Q10;
		if (kp_q10 < 0) {
			kp_q10 = 0;
		} else if (kp_q10 > PD_TUNING_KP_MAX_Q10) {
			kp_q10 = PD_TUNING_KP_MAX_Q10;
		}
	} else if (pd_tuning_selected == 1U) {
		kd_q10 += (direction < 0) ? -PD_TUNING_KD_STEP_Q10
				: PD_TUNING_KD_STEP_Q10;
		if (kd_q10 < 0) {
			kd_q10 = 0;
		} else if (kd_q10 > PD_TUNING_KD_MAX_Q10) {
			kd_q10 = PD_TUNING_KD_MAX_Q10;
		}
	} else {
		trim_sps += (direction < 0) ? -PD_TUNING_TRIM_STEP_SPS
				: PD_TUNING_TRIM_STEP_SPS;
		if (trim_sps < -PD_TUNING_TRIM_MAX_SPS) {
			trim_sps = -PD_TUNING_TRIM_MAX_SPS;
		} else if (trim_sps > PD_TUNING_TRIM_MAX_SPS) {
			trim_sps = PD_TUNING_TRIM_MAX_SPS;
		}
	}
	(void)FirstDrive_SetPdGains(kp_q10, kd_q10);
	(void)FirstDrive_SetMotorTrim(trim_sps);
	Menu_DrawPdTuning();
}

static bool Menu_FirstDriveIsActive(FirstDriveState_t state)
{
	return (state == FIRST_DRIVE_COUNTDOWN)
			|| (state == FIRST_DRIVE_LAUNCH)
			|| (state == FIRST_DRIVE_FOLLOW)
			|| (state == FIRST_DRIVE_TURN_LEFT)
			|| (state == FIRST_DRIVE_TURN_RIGHT)
			|| (state == FIRST_DRIVE_CROSS_PASS)
			|| (state == FIRST_DRIVE_END_CANDIDATE)
			|| (state == FIRST_DRIVE_RUNOUT);
}

static void Menu_ResetFirstDriveLines(void)
{
	uint8_t index;

	for (index = 0U; index < FIRST_DRIVE_LINE_COUNT; index++) {
		first_drive_displayed_lines[index][0] = '\0';
		first_drive_displayed_line_y[index] = 0U;
		first_drive_displayed_line_color[index] = LCD_COLOR_BLACK;
		first_drive_displayed_line_valid[index] = 0U;
	}
}

static void Menu_DrawFirstDriveLine(uint8_t index, uint16_t y,
		uint16_t color, const char *text)
{
	char padded[FIRST_DRIVE_LINE_CHARS + 1U];
	char character[2];
	size_t length;
	uint8_t column;

	if ((index >= FIRST_DRIVE_LINE_COUNT) || (text == NULL)) {
		return;
	}

	memset(padded, ' ', FIRST_DRIVE_LINE_CHARS);
	padded[FIRST_DRIVE_LINE_CHARS] = '\0';
	length = strlen(text);
	if (length > FIRST_DRIVE_LINE_CHARS) {
		length = FIRST_DRIVE_LINE_CHARS;
	}
	memcpy(padded, text, length);

	if (first_drive_displayed_line_valid[index]
			&& (first_drive_displayed_line_y[index] == y)
			&& (first_drive_displayed_line_color[index] == color)
			&& (strcmp(first_drive_displayed_lines[index], padded) == 0)) {
		return;
	}

	if (first_drive_displayed_line_valid[index]
			&& (first_drive_displayed_line_y[index] == y)
			&& (first_drive_displayed_line_color[index] == color)) {
		character[1] = '\0';
		for (column = 0U; column < FIRST_DRIVE_LINE_CHARS; column++) {
			if (first_drive_displayed_lines[index][column]
					!= padded[column]) {
				character[0] = padded[column];
				Menu_DrawText(8U + ((uint16_t)column * 6U), y, 12U,
						color, LCD_COLOR_BLACK, character);
			}
		}
	} else {
		Menu_DrawText(8U, y, 12U, color, LCD_COLOR_BLACK, padded);
	}
	memcpy(first_drive_displayed_lines[index], padded, sizeof(padded));
	first_drive_displayed_line_y[index] = y;
	first_drive_displayed_line_color[index] = color;
	first_drive_displayed_line_valid[index] = 1U;
}

static void Menu_EnsureFirstDrivePreview(void)
{
	FirstDriveState_t state = FirstDrive_GetState();

	if (!Sensor_IsCalibrationComplete()
			|| ((state != FIRST_DRIVE_READY) && (state != FIRST_DRIVE_ARMED))
			|| Sensor_IsRunning()) {
		return;
	}
	Sensor_SetLightMode(SENSOR_LIGHT_PAIR, 0U);
	Sensor_StateReset();
	Sensor_Start();
}

static void Menu_DrawFirstDrivePreviewScale(void)
{
	uint8_t index;

	ST7789_LCD_Driver.FillRect(&st7789_pObj, FIRST_DRIVE_PREVIEW_X,
			FIRST_DRIVE_PREVIEW_Y, FIRST_DRIVE_PREVIEW_WIDTH, 2U,
			LCD_COLOR_GRAY);
	for (index = SENSOR_LINE_SENSOR_FIRST;
			index <= SENSOR_LINE_SENSOR_LAST; index++) {
		uint16_t tick_x = FIRST_DRIVE_PREVIEW_X
				+ (uint16_t)(((uint32_t)(index
						- SENSOR_LINE_SENSOR_FIRST)
						* (FIRST_DRIVE_PREVIEW_WIDTH - 1U))
						/ (SENSOR_LINE_SENSOR_LAST
								- SENSOR_LINE_SENSOR_FIRST));
		ST7789_LCD_Driver.FillRect(&st7789_pObj, tick_x,
				FIRST_DRIVE_PREVIEW_Y - 3U, 1U, 8U, LCD_COLOR_GRAY);
	}
}

static void Menu_UpdateFirstDrivePreview(uint8_t force_update)
{
	char text[40];
	SensorLineMeasurement_t line;
	uint32_t now = HAL_GetTick();
	bool measured;

	if ((force_update == 0U)
			&& ((now - first_drive_preview_tick) < FIRST_DRIVE_UPDATE_MS)) {
		return;
	}
	first_drive_preview_tick = now;
	Menu_EnsureFirstDrivePreview();
	measured = Sensor_IsRunning() && Sensor_GetLineMeasurement(&line);

	if (force_update != 0U) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 43U,
				MENU_SCREEN_WIDTH - 8U, 45U, LCD_COLOR_BLACK);
		Menu_DrawText(8U, 44U, 12U, LCD_COLOR_GRAY, LCD_COLOR_BLACK, "S1");
		Menu_DrawText(218U, 44U, 12U, LCD_COLOR_GRAY, LCD_COLOR_BLACK, "S6");
		first_drive_preview_valid = 0xFFU;
		first_drive_preview_marker_x = 0xFFFFU;
	} else if ((first_drive_preview_valid == 1U)
			&& (first_drive_preview_marker_x != 0xFFFFU)) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj,
				first_drive_preview_marker_x, FIRST_DRIVE_PREVIEW_Y - 6U,
				3U, 14U, LCD_COLOR_BLACK);
	} else if (first_drive_preview_valid == 0U) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 114U, 52U,
				14U, 17U, LCD_COLOR_BLACK);
	}
	Menu_DrawFirstDrivePreviewScale();
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 74U,
			MENU_SCREEN_WIDTH - 8U, 14U, LCD_COLOR_BLACK);

	if (measured && line.line_valid) {
		int32_t offset = (int32_t)line.position - SENSOR_LINE_POSITION_MIN;
		uint16_t marker_x = FIRST_DRIVE_PREVIEW_X
				+ (uint16_t)(((uint32_t)offset
						* (FIRST_DRIVE_PREVIEW_WIDTH - 1U))
						/ (SENSOR_LINE_POSITION_MAX
								- SENSOR_LINE_POSITION_MIN));

		if (marker_x > FIRST_DRIVE_PREVIEW_X) {
			marker_x--;
		}
		ST7789_LCD_Driver.FillRect(&st7789_pObj, marker_x,
				FIRST_DRIVE_PREVIEW_Y - 6U, 3U, 14U, LCD_COLOR_YELLOW);
		first_drive_preview_marker_x = marker_x;
		first_drive_preview_valid = 1U;
		snprintf(text, sizeof(text), "POS %+5d M%02X L%02X STR%lu",
				(int)line.position, (unsigned int)line.state_mask,
				(unsigned int)line.selected_mask,
				(unsigned long)line.strength);
		Menu_DrawText(8U, 75U, 12U, LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
	} else {
		Menu_DrawText(116U, 53U, 16U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "X");
		Menu_DrawText(8U, 75U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "NO LINE - MOVE SENSOR OVER WHITE");
		first_drive_preview_marker_x = 0xFFFFU;
		first_drive_preview_valid = 0U;
	}
}

static void Menu_RenderFirstDrive(bool redraw_layout)
{
	char text[48];
	FirstDriveTelemetry_t telemetry;
	FirstDriveState_t state;
	FirstDriveFault_t fault;

	FirstDrive_GetTelemetry(&telemetry);
	state = telemetry.state;
	fault = telemetry.fault;
	if (redraw_layout) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 24U,
				MENU_SCREEN_WIDTH - 8U, 94U, LCD_COLOR_BLACK);
		Menu_ResetFirstDriveLines();
	}
	if (redraw_layout || (state != first_drive_displayed_state)
			|| (fault != first_drive_displayed_fault)) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 154U, 1U,
				82U, 18U, LCD_COLOR_BLACK);
		Menu_DrawText(160U, 4U, 12U,
				(state == FIRST_DRIVE_FAULT) ? LCD_COLOR_RED : LCD_COLOR_GREEN,
				LCD_COLOR_BLACK, Menu_FirstDriveStateName(state));
	}

	if ((state == FIRST_DRIVE_READY) || (state == FIRST_DRIVE_ARMED)) {
		if (!Sensor_IsCalibrationComplete()) {
			Menu_DrawFirstDriveLine(0U, 35U, LCD_COLOR_RED,
					"CALIBRATION REQUIRED");
			Menu_DrawFirstDriveLine(1U, 57U, LCD_COLOR_YELLOW,
					"C OPEN CALIBRATION");
		} else {
				snprintf(text, sizeof(text), "KP%ld KD%ld T%+d B%u",
						(long)FirstDrive_GetConfig()->kp_q10,
						(long)FirstDrive_GetConfig()->kd_q10,
						(int)FirstDrive_GetConfig()->trim_sps,
						(unsigned int)FirstDrive_GetConfig()->base_sps);
			Menu_DrawFirstDriveLine(0U, 27U, LCD_COLOR_GREEN, text);
			Menu_UpdateFirstDrivePreview(1U);
			Menu_DrawFirstDriveLine(1U, 93U,
					(state == FIRST_DRIVE_ARMED)
							? LCD_COLOR_YELLOW : LCD_COLOR_WHITE,
					(state == FIRST_DRIVE_ARMED)
							? "ARMED - C START  AUTO CANCEL 5S"
							: "C ARM  (START REQUIRES VALID LINE)");
			Menu_DrawFirstDriveLine(2U, 108U, LCD_COLOR_CYAN,
					"HOLD C BACK");
		}
	} else if (state == FIRST_DRIVE_COUNTDOWN) {
		snprintf(text, sizeof(text), "START IN %u.%01u SEC",
				(unsigned int)(telemetry.countdown_ms / 1000U),
				(unsigned int)((telemetry.countdown_ms % 1000U) / 100U));
		Menu_DrawFirstDriveLine(0U, 32U, LCD_COLOR_YELLOW, text);
		snprintf(text, sizeof(text), "P%+5d M%02X R%02X L%02X Q%02X",
				(int)telemetry.line_position,
				(unsigned int)telemetry.sensor_mask,
				(unsigned int)telemetry.raw_line_mask,
				(unsigned int)telemetry.line_mask,
				(unsigned int)telemetry.marker_spill_mask);
		Menu_DrawFirstDriveLine(1U, 51U,
				telemetry.line_valid ? LCD_COLOR_WHITE : LCD_COLOR_RED, text);
		snprintf(text, sizeof(text), "KP%ld KD%ld T%+d B%u",
				(long)FirstDrive_GetConfig()->kp_q10,
				(long)FirstDrive_GetConfig()->kd_q10,
				(int)FirstDrive_GetConfig()->trim_sps,
				(unsigned int)FirstDrive_GetConfig()->base_sps);
		Menu_DrawFirstDriveLine(2U, 70U, LCD_COLOR_GREEN, text);
		Menu_DrawFirstDriveLine(3U, 89U, LCD_COLOR_GRAY,
				"MOTORS OFF - CHECK ALIGNMENT");
		Menu_DrawFirstDriveLine(4U, 110U, LCD_COLOR_YELLOW,
				"CENTER PRESS = CANCEL");
	} else if (Menu_FirstDriveIsActive(state)) {
		snprintf(text, sizeof(text), "P%+5d M%02X R%02X L%02X Q%02X",
				(int)telemetry.line_position,
				(unsigned int)telemetry.sensor_mask,
				(unsigned int)telemetry.raw_line_mask,
				(unsigned int)telemetry.line_mask,
				(unsigned int)telemetry.marker_spill_mask);
		Menu_DrawFirstDriveLine(0U, 32U,
				telemetry.line_valid ? LCD_COLOR_WHITE : LCD_COLOR_YELLOW,
				text);
		snprintf(text, sizeof(text), "P%+5ld D%+5ld ST%+5ld",
				(long)telemetry.p_term, (long)telemetry.d_term,
				(long)telemetry.steer);
		Menu_DrawFirstDriveLine(1U, 51U, LCD_COLOR_WHITE, text);
		snprintf(text, sizeof(text), "V%4u>%4u L/R %4u/%4u",
				(unsigned int)telemetry.target_centre_sps,
				(unsigned int)telemetry.centre_sps,
				(unsigned int)telemetry.left_sps,
				(unsigned int)telemetry.right_sps);
		Menu_DrawFirstDriveLine(2U, 70U, LCD_COLOR_GREEN, text);
		snprintf(text, sizeof(text), "PH%s U%+d/%+d L%u/%u B%c",
				Menu_FirstDriveCourseName(telemetry.course_phase),
				(int)telemetry.target_steer_permille,
				(int)telemetry.applied_steer_permille,
				(unsigned int)telemetry.line_lost_ms,
				(unsigned int)telemetry.line_lost_limit_ms,
				(telemetry.bridge_recovery_direction < 0) ? 'L'
						: ((telemetry.bridge_recovery_direction > 0) ? 'R' : '-'));
		Menu_DrawFirstDriveLine(3U, 89U, LCD_COLOR_GRAY, text);
		Menu_DrawFirstDriveLine(4U, 110U, LCD_COLOR_YELLOW,
				"CENTER PRESS = STOP");
	} else if (((state == FIRST_DRIVE_FAULT)
			|| (state == FIRST_DRIVE_STOPPED))
			&& (first_drive_fault_page == 1U)) {
		uint8_t log_index;
		char provisional = (telemetry.provisional_marker_direction < 0)
				? 'L' : ((telemetry.provisional_marker_direction > 0)
						? 'R' : '-');

		Menu_DrawFirstDriveLine(0U, 32U, LCD_COLOR_CYAN,
				"PHASE LOG  L/R PAGE");
		snprintf(text, sizeof(text), "MK%s C%u N%lu P%c E%02X",
				Menu_FirstDriveMarkerName(telemetry.last_marker_type),
				(unsigned int)telemetry.last_marker_confidence,
				(unsigned long)telemetry.event_count,
				provisional,
				(unsigned int)telemetry.last_marker_edge_union);
		Menu_DrawFirstDriveLine(1U, 53U, LCD_COLOR_GREEN, text);
		for (log_index = 0U; log_index < FIRST_DRIVE_PHASE_LOG_DEPTH;
				log_index++) {
			if (log_index < telemetry.phase_log_count) {
				const FirstDrivePhaseLogEntry_t *entry =
						&telemetry.phase_log[log_index];

				snprintf(text, sizeof(text), "%u %s>%s %s S%lu",
						(unsigned int)log_index,
						Menu_FirstDriveCourseName(entry->from_phase),
						Menu_FirstDriveCourseName(entry->to_phase),
						Menu_FirstDrivePhaseReasonName(entry->reason),
						(unsigned long)entry->step);
			} else {
				snprintf(text, sizeof(text), "%u -- NO TRANSITION --",
						(unsigned int)log_index);
			}
			Menu_DrawFirstDriveLine((uint8_t)(log_index + 2U),
					(uint16_t)(74U + ((uint16_t)log_index * 18U)),
					(log_index == 0U) ? LCD_COLOR_YELLOW : LCD_COLOR_WHITE,
					text);
		}
	} else if (((state == FIRST_DRIVE_FAULT)
			|| (state == FIRST_DRIVE_STOPPED))
			&& (first_drive_fault_page >= 2U)) {
		uint8_t first_index = (first_drive_fault_page == 2U) ? 0U : 3U;
		uint8_t entry_count = (first_drive_fault_page == 2U) ? 3U : 2U;
		uint8_t row;

		Menu_DrawFirstDriveLine(0U, 30U, LCD_COLOR_CYAN,
				(first_drive_fault_page == 2U)
						? "MARK EVENTS 1/2  NEW>OLD"
						: "MARK EVENTS 2/2  NEW>OLD");
		for (row = 0U; row < entry_count; row++) {
			uint8_t log_index = (uint8_t)(first_index + row);

			if (log_index < telemetry.marker_log_count) {
				const FirstDriveMarkerLogEntry_t *entry =
						&telemetry.marker_log[log_index];

				snprintf(text, sizeof(text), "%u %s C%u E%02X I%lu X%lu",
						(unsigned int)log_index,
						Menu_FirstDriveMarkerName(entry->type),
						(unsigned int)entry->confidence,
						(unsigned int)entry->edge_union,
						(unsigned long)entry->entry_step,
						(unsigned long)entry->exit_step);
			} else {
				snprintf(text, sizeof(text), "%u -- NO EVENT --",
						(unsigned int)log_index);
			}
			Menu_DrawFirstDriveLine((uint8_t)(row + 1U),
					(uint16_t)(50U + ((uint16_t)row * 18U)),
					(log_index == 0U) ? LCD_COLOR_YELLOW : LCD_COLOR_WHITE,
					text);
		}
		if (first_drive_fault_page == 2U) {
			Menu_DrawFirstDriveLine(4U, 110U, LCD_COLOR_CYAN,
					"L/R PAGE");
		} else {
			snprintf(text, sizeof(text), "TOTAL N%lu  L/R PAGE",
					(unsigned long)telemetry.event_count);
			Menu_DrawFirstDriveLine(3U, 92U, LCD_COLOR_GREEN, text);
			Menu_DrawFirstDriveLine(4U, 110U, LCD_COLOR_CYAN,
					"HOLD C BACK");
		}
	} else {
		snprintf(text, sizeof(text), "%s  FAULT %s",
				Menu_FirstDriveStateName(state), Menu_FirstDriveFaultName(fault));
		Menu_DrawFirstDriveLine(0U, 32U,
				(state == FIRST_DRIVE_FAULT) ? LCD_COLOR_RED : LCD_COLOR_GREEN,
				text);
		snprintf(text, sizeof(text), "LAST P%+5d L%02X NOW M%02X L%02X Q%02X",
				(int)telemetry.last_valid_position,
				(unsigned int)telemetry.last_valid_line_mask,
				(unsigned int)telemetry.sensor_mask,
				(unsigned int)telemetry.line_mask,
				(unsigned int)telemetry.marker_spill_mask);
		Menu_DrawFirstDriveLine(1U, 53U, LCD_COLOR_WHITE, text);
		snprintf(text, sizeof(text), "PH%s V%u/%u U%+d/%+d",
				Menu_FirstDriveCourseName(telemetry.course_phase),
				(unsigned int)telemetry.target_centre_sps,
				(unsigned int)telemetry.centre_sps,
				(int)telemetry.target_steer_permille,
				(int)telemetry.applied_steer_permille);
		Menu_DrawFirstDriveLine(2U, 74U, LCD_COLOR_WHITE, text);
		if (fault == FIRST_DRIVE_FAULT_EDGE_STUCK) {
			snprintf(text, sizeof(text), "EDGE %u LR %u/%u",
					(unsigned int)telemetry.edge_dwell_ms,
					(unsigned int)telemetry.left_sps,
					(unsigned int)telemetry.right_sps);
		} else {
			snprintf(text, sizeof(text), "LOSS %u/%u LR %u/%u",
					(unsigned int)telemetry.line_lost_ms,
					(unsigned int)telemetry.line_lost_limit_ms,
					(unsigned int)telemetry.left_sps,
					(unsigned int)telemetry.right_sps);
		}
		Menu_DrawFirstDriveLine(3U, 95U, LCD_COLOR_GRAY, text);
		snprintf(text, sizeof(text), "TAIL N%u G%lu E%02X A%u",
				(unsigned int)telemetry.cross_tail_suppressed_count,
				(unsigned long)telemetry.last_cross_tail_gap_steps,
				(unsigned int)telemetry.last_cross_tail_edge_union,
				(unsigned int)telemetry.cross_tail_guard_active);
		Menu_DrawFirstDriveLine(4U, 110U, LCD_COLOR_CYAN, text);
	}
	first_drive_displayed_state = state;
	first_drive_displayed_fault = fault;
}

static void Menu_DrawFirstDrive(void)
{
	first_drive_fault_page = 0U;
	first_drive_displayed_state = FIRST_DRIVE_OFF;
	first_drive_displayed_fault = FIRST_DRIVE_FAULT_NONE;
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "7. FIRST DRIVE");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);
	Menu_RenderFirstDrive(true);
}

static void Menu_UpdateFirstDrive(void)
{
	uint32_t now = HAL_GetTick();
	FirstDriveState_t state;
	FirstDriveFault_t fault;

	state = FirstDrive_GetState();
	fault = FirstDrive_GetFault();
	if ((state == FIRST_DRIVE_ARMED)
			&& ((now - first_drive_arm_tick) >= FIRST_DRIVE_ARM_TIMEOUT_MS)) {
		FirstDrive_Init();
		Menu_EnsureFirstDrivePreview();
		state = FirstDrive_GetState();
		fault = FirstDrive_GetFault();
	}
	if ((state != first_drive_displayed_state)
			|| (fault != first_drive_displayed_fault)) {
		bool redraw_layout = (state != first_drive_displayed_state)
				&& !(Menu_FirstDriveIsActive(state)
						&& Menu_FirstDriveIsActive(first_drive_displayed_state));

		Menu_RenderFirstDrive(redraw_layout);
		first_drive_displayed_state = state;
		first_drive_displayed_fault = fault;
		first_drive_update_tick = now;
		return;
	}
	if ((now - first_drive_update_tick) < FIRST_DRIVE_UPDATE_MS) {
		return;
	}
	first_drive_update_tick = now;
	if (Menu_FirstDriveIsActive(state)) {
		Menu_RenderFirstDrive(false);
	} else if ((state == FIRST_DRIVE_READY) || (state == FIRST_DRIVE_ARMED)) {
		Menu_UpdateFirstDrivePreview(0U);
	}
}

static const char *Menu_SecondDriveSegmentName(TrackSegmentType_t type)
{
	switch (type) {
	case TRACK_SEGMENT_LEFT: return "LEFT";
	case TRACK_SEGMENT_RIGHT: return "RIGHT";
	case TRACK_SEGMENT_CROSS: return "CROSS";
	case TRACK_SEGMENT_END: return "END";
	default: return "STRAIGHT";
	}
}

static const char *Menu_SecondDriveSyncName(SecondDriveSyncState_t state)
{
	switch (state) {
	case SECOND_DRIVE_SYNC_SEEK_CROSS: return "SEEK CROSS";
	case SECOND_DRIVE_SYNC_INVALID: return "MAP INVALID";
	case SECOND_DRIVE_SYNC_MAP:
	default: return "MAP SYNC";
	}
}

static const char *Menu_SecondDriveMismatchName(
		SecondDriveMismatchReason_t reason)
{
	switch (reason) {
	case SECOND_DRIVE_MISMATCH_EVENT_TYPE: return "TYPE";
	case SECOND_DRIVE_MISMATCH_EVENT_DISTANCE: return "DIST";
	case SECOND_DRIVE_MISMATCH_SEGMENT_OVERDUE: return "OVERDUE";
	case SECOND_DRIVE_MISMATCH_MAP_BOUNDS: return "BOUNDS";
	case SECOND_DRIVE_MISMATCH_ANCHOR_NOT_FOUND: return "ANCHOR NF";
	case SECOND_DRIVE_MISMATCH_ANCHOR_AMBIGUOUS: return "ANCHOR AMB";
	case SECOND_DRIVE_MISMATCH_NONE:
	default: return "NONE";
	}
}

static void Menu_RenderSecondDrive(void)
{
	char text[48];
	SecondDriveTelemetry_t telemetry;
	const SecondDriveConfig_t *config = SecondDrive_GetConfig();
	FirstDriveState_t state;
	FirstDriveFault_t fault;

	SecondDrive_GetTelemetry(&telemetry);
	state = telemetry.drive.state;
	fault = telemetry.drive.fault;
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 4U, 24U,
			MENU_SCREEN_WIDTH - 8U, 94U, LCD_COLOR_BLACK);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 154U, 1U,
			82U, 18U, LCD_COLOR_BLACK);
	Menu_DrawText(160U, 4U, 12U,
			(state == FIRST_DRIVE_FAULT) ? LCD_COLOR_RED : LCD_COLOR_GREEN,
			LCD_COLOR_BLACK, Menu_FirstDriveStateName(state));

	if ((state == FIRST_DRIVE_READY) || (state == FIRST_DRIVE_ARMED)) {
		uint16_t straight_fg = ((state == FIRST_DRIVE_READY)
				&& (second_drive_selected == 0U))
				? LCD_COLOR_BLACK : LCD_COLOR_WHITE;
		uint16_t straight_bg = ((state == FIRST_DRIVE_READY)
				&& (second_drive_selected == 0U))
				? LCD_COLOR_CYAN : LCD_COLOR_BLACK;
		uint16_t overall_fg = ((state == FIRST_DRIVE_READY)
				&& (second_drive_selected == 1U))
				? LCD_COLOR_BLACK : LCD_COLOR_WHITE;
		uint16_t overall_bg = ((state == FIRST_DRIVE_READY)
				&& (second_drive_selected == 1U))
				? LCD_COLOR_CYAN : LCD_COLOR_BLACK;

		snprintf(text, sizeof(text), "%s E%u S%u A%u",
				Menu_SecondDriveSyncName(telemetry.planner.sync_state),
				(unsigned int)Track_GetEventCount(),
				(unsigned int)telemetry.planner.segment_count,
				(unsigned int)telemetry.planner.anchor_count);
		Menu_DrawText(8U, 27U, 12U,
				(telemetry.planner.sync_state == SECOND_DRIVE_SYNC_MAP)
						? LCD_COLOR_GREEN : LCD_COLOR_RED,
				LCD_COLOR_BLACK, text);
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 6U, 43U,
				228U, 23U, straight_bg);
		snprintf(text, sizeof(text), "STRAIGHT  %4u SPS",
				(unsigned int)config->straight_sps);
		Menu_DrawText(12U, 47U, 16U, straight_fg, straight_bg, text);
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 6U, 70U,
				228U, 23U, overall_bg);
		snprintf(text, sizeof(text), "ALL SPEED   %3u %%",
				(unsigned int)config->overall_percent);
		Menu_DrawText(12U, 74U, 16U, overall_fg, overall_bg, text);
		if (!Sensor_IsCalibrationComplete()) {
			Menu_DrawText(8U, 101U, 12U, LCD_COLOR_RED, LCD_COLOR_BLACK,
					"CAL REQUIRED - C OPEN CAL");
		} else if (state == FIRST_DRIVE_ARMED) {
			Menu_DrawText(8U, 101U, 12U, LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
					"ARMED - C START  AUTO CANCEL 5S");
		} else if (second_drive_selected == 0U) {
			Menu_DrawText(8U, 101U, 12U, LCD_COLOR_CYAN, LCD_COLOR_BLACK,
					"L/R ADJUST   C NEXT");
		} else {
			Menu_DrawText(8U, 101U, 12U, LCD_COLOR_CYAN, LCD_COLOR_BLACK,
					"L/R ADJUST   C ARM");
		}
	} else if (state == FIRST_DRIVE_COUNTDOWN) {
		snprintf(text, sizeof(text), "START IN %u.%01u SEC",
				(unsigned int)(telemetry.drive.countdown_ms / 1000U),
				(unsigned int)((telemetry.drive.countdown_ms % 1000U) / 100U));
		Menu_DrawText(8U, 30U, 16U, LCD_COLOR_YELLOW, LCD_COLOR_BLACK, text);
		snprintf(text, sizeof(text), "STRAIGHT %u  ALL %u%%",
				(unsigned int)config->straight_sps,
				(unsigned int)config->overall_percent);
		Menu_DrawText(8U, 55U, 16U, LCD_COLOR_GREEN, LCD_COLOR_BLACK, text);
		snprintf(text, sizeof(text), "MAP E%u S%u A%u  LINE %s",
				(unsigned int)Track_GetEventCount(),
				(unsigned int)telemetry.planner.segment_count,
				(unsigned int)telemetry.planner.anchor_count,
				telemetry.drive.line_valid ? "OK" : "LOST");
		Menu_DrawText(8U, 80U, 12U, LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
		Menu_DrawText(8U, 105U, 12U, LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"CENTER PRESS = CANCEL");
	} else if (Menu_FirstDriveIsActive(state)) {
		const bool has_anchor =
				telemetry.planner.current_anchor_order != UINT16_MAX;

		snprintf(text, sizeof(text), "SEG %u/%u %s U%u R%lu",
				(unsigned int)(telemetry.planner.segment_index + 1U),
				(unsigned int)telemetry.planner.segment_count,
				Menu_SecondDriveSegmentName(telemetry.planner.segment_type),
				(unsigned int)telemetry.planner.curve_units,
				(unsigned long)telemetry.planner.segment_remaining_steps);
		Menu_DrawText(8U, 27U, 12U,
				(telemetry.planner.sync_state == SECOND_DRIVE_SYNC_MAP)
						? LCD_COLOR_CYAN : LCD_COLOR_YELLOW,
				LCD_COLOR_BLACK, text);
		snprintf(text, sizeof(text), "V%4u>%4u  L/R %4u/%4u",
				(unsigned int)telemetry.drive.target_centre_sps,
				(unsigned int)telemetry.drive.centre_sps,
				(unsigned int)telemetry.drive.left_sps,
				(unsigned int)telemetry.drive.right_sps);
		Menu_DrawText(8U, 48U, 12U, LCD_COLOR_GREEN, LCD_COLOR_BLACK, text);
		snprintf(text, sizeof(text), "P%+5d PH%s LOSS %u/%u",
				(int)telemetry.drive.line_position,
				Menu_FirstDriveCourseName(telemetry.drive.course_phase),
				(unsigned int)telemetry.drive.line_lost_ms,
				(unsigned int)telemetry.drive.line_lost_limit_ms);
		Menu_DrawText(8U, 69U, 12U, LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
		if (has_anchor) {
				snprintf(text, sizeof(text), "A%u/%u E%u NR%lu T%u",
						(unsigned int)telemetry.planner.current_anchor_order,
						(unsigned int)telemetry.planner.anchor_count,
						(unsigned int)telemetry.planner.expected_event_index,
						(unsigned long)telemetry.planner.next_restriction_distance_steps,
						(unsigned int)telemetry.drive.cross_tail_suppressed_count);
			} else {
				snprintf(text, sizeof(text), "A-/%u E%u NR%lu T%u",
						(unsigned int)telemetry.planner.anchor_count,
						(unsigned int)telemetry.planner.expected_event_index,
						(unsigned long)telemetry.planner.next_restriction_distance_steps,
						(unsigned int)telemetry.drive.cross_tail_suppressed_count);
		}
		Menu_DrawText(8U, 90U, 12U,
				(telemetry.planner.sync_state == SECOND_DRIVE_SYNC_MAP)
						? LCD_COLOR_GREEN : LCD_COLOR_YELLOW,
				LCD_COLOR_BLACK, text);
		snprintf(text, sizeof(text), "%s M%u R%u %s",
				Menu_SecondDriveSyncName(telemetry.planner.sync_state),
				(unsigned int)telemetry.planner.mismatch_count,
				(unsigned int)telemetry.planner.resync_count,
				Menu_SecondDriveMismatchName(
						telemetry.planner.last_mismatch_reason));
		strncat(text, " C=STOP", sizeof(text) - strlen(text) - 1U);
		Menu_DrawText(8U, 107U, 12U,
				(telemetry.planner.sync_state == SECOND_DRIVE_SYNC_MAP)
						? LCD_COLOR_GREEN : LCD_COLOR_YELLOW,
				LCD_COLOR_BLACK, text);
	} else {
		snprintf(text, sizeof(text), "%s  FAULT %s",
				Menu_FirstDriveStateName(state), Menu_FirstDriveFaultName(fault));
		Menu_DrawText(8U, 29U, 16U,
				(state == FIRST_DRIVE_FAULT) ? LCD_COLOR_RED : LCD_COLOR_GREEN,
				LCD_COLOR_BLACK, text);
		snprintf(text, sizeof(text), "SEG %u/%u %s  STEP %lu",
				(unsigned int)(telemetry.planner.segment_index + 1U),
				(unsigned int)telemetry.planner.segment_count,
				Menu_SecondDriveSegmentName(telemetry.planner.segment_type),
				(unsigned long)telemetry.drive.average_steps);
		Menu_DrawText(8U, 55U, 12U, LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
		snprintf(text, sizeof(text), "%s M%u R%u",
				Menu_SecondDriveSyncName(telemetry.planner.sync_state),
				(unsigned int)telemetry.planner.mismatch_count,
				(unsigned int)telemetry.planner.resync_count);
		Menu_DrawText(8U, 77U, 12U,
				(telemetry.planner.sync_state == SECOND_DRIVE_SYNC_MAP)
						? LCD_COLOR_CYAN : LCD_COLOR_YELLOW,
				LCD_COLOR_BLACK, text);
		snprintf(text, sizeof(text), "REASON %s E%u T%u G%lu",
				Menu_SecondDriveMismatchName(
						telemetry.planner.last_mismatch_reason),
				(unsigned int)telemetry.planner.replay_event_count,
				(unsigned int)telemetry.drive.cross_tail_suppressed_count,
				(unsigned long)telemetry.drive.last_cross_tail_gap_steps);
		Menu_DrawText(8U, 98U, 12U, LCD_COLOR_GRAY, LCD_COLOR_BLACK, text);
	}
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
}

static void Menu_DrawSecondDrive(void)
{
	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "8. SECOND DRIVE");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);
	Menu_RenderSecondDrive();
}

static void Menu_ChangeSecondDriveSetting(int8_t direction)
{
	const SecondDriveConfig_t *config = SecondDrive_GetConfig();

	if (SecondDrive_GetState() != FIRST_DRIVE_READY) {
		return;
	}
	if (second_drive_selected == 0U) {
		int32_t value = config->straight_sps
				+ ((direction < 0) ? -(int32_t)SECOND_DRIVE_STRAIGHT_STEP_SPS
						: (int32_t)SECOND_DRIVE_STRAIGHT_STEP_SPS);

		if (value < SECOND_DRIVE_STRAIGHT_MIN_SPS) {
			value = SECOND_DRIVE_STRAIGHT_MIN_SPS;
		} else if (value > SECOND_DRIVE_STRAIGHT_MAX_SPS) {
			value = SECOND_DRIVE_STRAIGHT_MAX_SPS;
		}
		(void)SecondDrive_SetStraightSps((uint16_t)value);
	} else {
		int32_t value = config->overall_percent
				+ ((direction < 0)
						? -(int32_t)SECOND_DRIVE_OVERALL_STEP_PERCENT
						: (int32_t)SECOND_DRIVE_OVERALL_STEP_PERCENT);

		if (value < SECOND_DRIVE_OVERALL_MIN_PERCENT) {
			value = SECOND_DRIVE_OVERALL_MIN_PERCENT;
		} else if (value > SECOND_DRIVE_OVERALL_MAX_PERCENT) {
			value = SECOND_DRIVE_OVERALL_MAX_PERCENT;
		}
		(void)SecondDrive_SetOverallPercent((uint8_t)value);
	}
	Menu_RenderSecondDrive();
}

static void Menu_UpdateSecondDrive(void)
{
	uint32_t now = HAL_GetTick();

	if ((SecondDrive_GetState() == FIRST_DRIVE_ARMED)
			&& ((now - second_drive_arm_tick)
					>= SECOND_DRIVE_ARM_TIMEOUT_MS)) {
		(void)SecondDrive_Init();
		second_drive_selected = 0U;
		Menu_RenderSecondDrive();
		second_drive_update_tick = now;
		return;
	}
	if ((now - second_drive_update_tick) >= SECOND_DRIVE_UPDATE_MS) {
		second_drive_update_tick = now;
		Menu_RenderSecondDrive();
	}
}

static void Menu_UpdateSensorRaw(uint8_t force_update)
{
	char text[40];
	uint16_t raw[SENSOR_COUNT];
	uint16_t values[SENSOR_COUNT];
	uint32_t now = HAL_GetTick();
	uint32_t samples;
	uint16_t display_max;
	bool normalized_view;
	uint8_t status;
	uint8_t index;

	if ((force_update == 0U)
			&& ((now - sensor_raw_update_tick) < SENSOR_RAW_UPDATE_MS)) {
		return;
	}
	sensor_raw_update_tick = now;

	Sensor_GetRaw(raw);
	normalized_view = Sensor_IsCalibrationComplete()
			&& (sensor_light_mode == SENSOR_LIGHT_PAIR)
			&& (sensor_light_parameter == 0U);
	if (normalized_view) {
		Sensor_NormalizeFrame(raw, values);
		display_max = SENSOR_NORMALIZED_MAX;
	} else {
		memcpy(values, raw, sizeof(values));
		display_max = SENSOR_ADC_OVERSAMPLED_MAX;
	}
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
		uint16_t value = values[index];
		uint16_t height;
		uint16_t color;
		uint16_t x = 8U + ((uint16_t)index * SENSOR_BAR_PITCH);
		uint16_t value_x = 8U + ((uint16_t)(index & 0x03U)
				* SENSOR_VALUE_PITCH);
		uint16_t value_y = 36U + ((uint16_t)(index >> 2U) * 16U);

		if (value != sensor_displayed_raw[index]) {
			snprintf(text, sizeof(text), normalized_view ? "%u:%4u" : "%u:%5u",
					(unsigned int)index, (unsigned int)value);
			Menu_DrawText(value_x, value_y, 12U,
					LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
			sensor_displayed_raw[index] = value;
		}

		if (value > display_max) {
			value = display_max;
		}
		height = (uint16_t)(((uint32_t)value * SENSOR_BAR_MAX_HEIGHT)
				/ display_max);

		if (normalized_view) {
			if (value <= SENSOR_STATE_WHITE_OFF) {
				color = LCD_COLOR_GRAY;
			} else if (value >= SENSOR_STATE_WHITE_ON) {
				color = LCD_COLOR_WHITE;
			} else {
				color = LCD_COLOR_CYAN;
			}
		} else if (value < (display_max / 3U)) {
			color = LCD_COLOR_GRAY;
		} else if (value < ((display_max * 2U) / 3U)) {
			color = LCD_COLOR_CYAN;
		} else {
			color = LCD_COLOR_WHITE;
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
		if (Sensor_IsCalibrationComplete() && (sensor_light_parameter == 0U)) {
			snprintf(text, sizeof(text), "PAIR +0  DIFF 0-1000");
		} else {
			snprintf(text, sizeof(text), "PAIR +%u  ADC 0-16380",
					(unsigned int)sensor_light_parameter);
		}
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
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, "START ON BLACK FLOOR");
	Menu_DrawText(8U, 53U, 12U,
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "BLACK 3S, THEN SWEEP WHITE 8-12S");
	Menu_DrawText(8U, 72U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "AUTO STOP: 8/8 AFTER 8S");
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
			LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "KEEP ALL SENSORS ON BLACK");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, CALIBRATION_BAR_X,
			CALIBRATION_BAR_Y, CALIBRATION_BAR_WIDTH,
			CALIBRATION_BAR_HEIGHT, LCD_COLOR_GRAY);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, CALIBRATION_BAR_X + 1U,
			CALIBRATION_BAR_Y + 1U, CALIBRATION_BAR_WIDTH - 2U,
			CALIBRATION_BAR_HEIGHT - 2U, LCD_COLOR_BLACK);
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "AUTO QUALITY STOP   HOLD C BACK");
	calibration_update_tick = 0U;
	calibration_displayed_phase = 0xFFU;
	calibration_displayed_coverage = 0xFFU;
	for (index = 0U; index < SENSOR_COUNT; index++) {
		calibration_displayed_range[index] = 0xFFFFU;
	}
}

static void Menu_UpdateCalibrationCollecting(uint8_t force_update)
{
	char text[32];
	uint16_t raw[SENSOR_COUNT];
	uint16_t low[SENSOR_COUNT];
	uint16_t high[SENSOR_COUNT];
	uint32_t now = HAL_GetTick();
	uint32_t elapsed = now - calibration_start_tick;
	uint32_t phase_elapsed;
	uint32_t phase_duration;
	uint16_t progress_width;
	uint8_t coverage_mask;
	uint8_t coverage_count = 0U;
	uint8_t phase;
	uint8_t index;

	if (elapsed < CALIBRATION_BLACK_END_MS) {
		phase = 0U;
	} else if (elapsed < CALIBRATION_WHITE_START_MS) {
		phase = 1U;
	} else {
		phase = 2U;
	}

	/* Drain each completed sensor frame once.  The previous implementation
	 * repeatedly averaged whichever frame happened to be newest at 100 ms UI
	 * intervals, so the result depended on display timing rather than samples. */
	while (Sensor_GetFrameFilteredAfter(&calibration_frame_cursor, raw)) {
		if ((elapsed >= CALIBRATION_WARMUP_MS)
				&& (elapsed < CALIBRATION_BLACK_END_MS)) {
			Sensor_CalibrationCaptureBlack(raw);
		} else if (elapsed >= CALIBRATION_WHITE_START_MS) {
			if (calibration_black_finished == 0U) {
				calibration_black_finished = Sensor_CalibrationFinishBlack()
						? 1U : 0U;
			}
			if (calibration_black_finished != 0U) {
				Sensor_CalibrationCaptureWhite(raw);
			}
		}
	}
	if ((elapsed >= CALIBRATION_BLACK_END_MS)
			&& (calibration_black_finished == 0U)) {
		calibration_black_finished = Sensor_CalibrationFinishBlack()
				? 1U : 0U;
	}

	coverage_mask = Sensor_GetCalibrationCoverageMask();
	for (index = 0U; index < SENSOR_COUNT; index++) {
		if ((coverage_mask & (uint8_t)(1U << index)) != 0U) {
			coverage_count++;
		}
	}
	if (elapsed >= CALIBRATION_WHITE_START_MS) {
		uint32_t white_elapsed = elapsed - CALIBRATION_WHITE_START_MS;

		if (((white_elapsed >= CALIBRATION_WHITE_MIN_MS)
				&& (coverage_mask == 0xFFU))
				|| (white_elapsed >= CALIBRATION_WHITE_MAX_MS)) {
			calibration_auto_finish_ready = 1U;
		}
	}

	if ((force_update == 0U)
			&& ((now - calibration_update_tick) < CALIBRATION_UPDATE_MS)) {
		return;
	}
	calibration_update_tick = now;
	Sensor_GetCalibration(low, high);
	if (phase != calibration_displayed_phase) {
		ST7789_LCD_Driver.FillRect(&st7789_pObj, 8U, 25U,
				224U, 65U, LCD_COLOR_BLACK);
		Menu_DrawText(8U, 76U, 12U, LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				(phase == 0U) ? "KEEP ALL SENSORS ON BLACK"
						: ((phase == 1U) ? "GET READY TO SWEEP WHITE"
								: "SWEEP WHITE LINE ACROSS ALL"));
		for (index = 0U; index < SENSOR_COUNT; index++) {
			calibration_displayed_range[index] = 0xFFFFU;
		}
		calibration_displayed_phase = phase;
	}

	if (phase == 0U) {
		phase_elapsed = (elapsed > CALIBRATION_WARMUP_MS)
				? (elapsed - CALIBRATION_WARMUP_MS) : 0U;
		phase_duration = CALIBRATION_BLACK_MS;
	} else if (phase == 1U) {
		phase_elapsed = elapsed - CALIBRATION_BLACK_END_MS;
		phase_duration = CALIBRATION_PREP_MS;
	} else {
		phase_elapsed = elapsed - CALIBRATION_WHITE_START_MS;
		phase_duration = CALIBRATION_WHITE_MAX_MS;
	}
	if (phase_elapsed > phase_duration) {
		phase_elapsed = phase_duration;
	}

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 8U, 25U,
			224U, 14U, LCD_COLOR_BLACK);
	if ((phase == 0U) && (elapsed < CALIBRATION_WARMUP_MS)) {
		snprintf(text, sizeof(text), "SENSOR WARMUP %u.%us",
				(unsigned int)((CALIBRATION_WARMUP_MS - elapsed) / 1000U),
				(unsigned int)(((CALIBRATION_WARMUP_MS - elapsed) % 1000U)
						/ 100U));
	} else if (phase == 0U) {
		snprintf(text, sizeof(text), "BLACK BASE %u.%us",
				(unsigned int)((phase_duration - phase_elapsed) / 1000U),
				(unsigned int)(((phase_duration - phase_elapsed) % 1000U)
						/ 100U));
	} else if (phase == 1U) {
		snprintf(text, sizeof(text), "WHITE PREP %u.%us",
				(unsigned int)((phase_duration - phase_elapsed) / 1000U),
				(unsigned int)(((phase_duration - phase_elapsed) % 1000U)
						/ 100U));
	} else {
		snprintf(text, sizeof(text), "WHITE %u/8  %u.%us",
				(unsigned int)coverage_count,
				(unsigned int)(phase_elapsed / 1000U),
				(unsigned int)((phase_elapsed % 1000U) / 100U));
	}
	Menu_DrawText(8U, 25U, 12U,
			LCD_COLOR_GREEN, LCD_COLOR_BLACK, text);

	if ((phase == 2U) && (coverage_mask != calibration_displayed_coverage)) {
		for (index = 0U; index < SENSOR_COUNT; index++) {
			calibration_displayed_range[index] = 0xFFFFU;
		}
	}
	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t range = (high[index] >= low[index])
				? (uint16_t)(high[index] - low[index]) : 0U;
		uint16_t shown = (phase < 2U) ? low[index] : range;
		uint16_t x = 8U + ((uint16_t)(index & 0x03U) * 56U);
		uint16_t y = 43U + ((uint16_t)(index >> 2U) * 16U);
		uint16_t color = (phase < 2U) ? LCD_COLOR_CYAN
				: (((coverage_mask & (uint8_t)(1U << index)) != 0U)
						? LCD_COLOR_GREEN : LCD_COLOR_YELLOW);

		if (shown != calibration_displayed_range[index]) {
			snprintf(text, sizeof(text), (phase < 2U) ? "%u:%5u" : "%u:+%4u",
					(unsigned int)index, (unsigned int)shown);
			Menu_DrawText(x, y, 12U, color, LCD_COLOR_BLACK, text);
			calibration_displayed_range[index] = shown;
		}
	}
	calibration_displayed_coverage = coverage_mask;

	progress_width = (uint16_t)(((uint32_t)(CALIBRATION_BAR_WIDTH - 2U)
			* phase_elapsed) / phase_duration);
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
		uint16_t range = (high[index] >= low[index])
				? (uint16_t)(high[index] - low[index]) : 0U;

		snprintf(text, sizeof(text), "%u:%5u+%5u",
				(unsigned int)index, (unsigned int)low[index],
				(unsigned int)range);
		Menu_DrawText(x, y, 12U, color, LCD_COLOR_BLACK, text);
	}

	Menu_DrawText(8U, 108U, 12U,
			(valid_count == SENSOR_COUNT) ? LCD_COLOR_GREEN : LCD_COLOR_YELLOW,
			LCD_COLOR_BLACK,
			(valid_count == SENSOR_COUNT) ? "OUTPUT: BLACK 0 / WHITE 1000"
					: "CHECK BLACK BASE, THEN SWEEP MORE");
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
	calibration_frame_cursor = Sensor_GetFrameCount();
	calibration_black_finished = 0U;
	calibration_auto_finish_ready = 0U;
	calibration_state = CALIBRATION_COLLECTING;
	Menu_DrawCalibrationCollecting();
	Menu_UpdateCalibrationCollecting(1U);
}

static void Menu_FinishCalibration(void)
{
	uint8_t valid_mask;

	if (calibration_black_finished == 0U) {
		calibration_black_finished = Sensor_CalibrationFinishBlack()
				? 1U : 0U;
	}
	valid_mask = Sensor_CalibrationFinish();
	Sensor_Stop();
	calibration_state = CALIBRATION_DONE;
	Menu_DrawCalibrationResult(valid_mask);
}

#if 0 /* SENSOR NORMAL was removed; calibrated decisions live in SENSOR STATE. */
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
#endif

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
	/* S0/S7 may be active as markers while the actual S1..S6 line is lost. */
	lost = ((state_mask & SENSOR_LINE_MASK) == 0U) ? 1U : 0U;
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
					LCD_COLOR_WHITE, LCD_COLOR_BLACK, "L S0->S7 R");
			Menu_DrawText(88U, 80U, 12U,
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
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "3. SENSOR STATE");
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
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "BLACK=LOW  HYST 40/60%");
	Menu_DrawText(8U, MENU_FOOTER_Y, 12U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "HOLD C TO GO BACK");
	Menu_ResetSensorStateDisplay();
}

#if 0 /* MARK DIAGNOSTIC was removed while sensor/calibration is validated. */
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
#endif

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
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "4. MOTOR PHASE");
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
	snprintf(text, sizeof(text), "DAC %u  CURRENT ~%umA",
			(unsigned int)MOTOR_PHASE_TEST_DAC,
			(unsigned int)MOTOR_VREF_CURRENT_MA);
	Menu_DrawText(8U, 43U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, text);

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
				"L/R TARGET  C ARM  AUTO OFF 5S");
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

static uint16_t Menu_MotorSpeedMpsX100(uint16_t half_steps_per_second)
{
	/* 50 mm wheel, 400 half-steps/revolution:
	 * m/s x 100 = half-step/s x pi x 0.05 / 400 x 100. */
	return (uint16_t)((((uint32_t)half_steps_per_second * 3927U)
			+ 50000U) / 100000U);
}

static void Menu_DrawMotorSpeedRuntime(void)
{
	char text[40];
	uint16_t current_sps = Motor_SpeedTestGetCurrentSps();
	uint16_t target_sps = motor_speed_steps[motor_speed_level];
	uint16_t actual_mps_x100 = Menu_MotorSpeedMpsX100(current_sps);
	const char *phase = (current_sps < target_sps) ? "RAMP" : "HOLD";

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 158U, 2U,
			80U, 18U, LCD_COLOR_BLACK);
	if (motor_speed_error != 0U) {
		Menu_DrawText(176U, 4U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "ERROR");
	} else if (motor_speed_running != 0U) {
		Menu_DrawText(188U, 4U, 12U,
				LCD_COLOR_RED, LCD_COLOR_BLACK, "RUN");
	} else if (motor_speed_armed != 0U) {
		Menu_DrawText(176U, 4U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK, "ARMED");
	} else {
		Menu_DrawText(164U, 4U, 12U,
				LCD_COLOR_GREEN, LCD_COLOR_BLACK, "SAFE OFF");
	}

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 8U, 58U,
			224U, 15U, LCD_COLOR_BLACK);
	if (motor_speed_running != 0U) {
		snprintf(text, sizeof(text), "ACTUAL %4u HS/S  %u.%02u %s",
				(unsigned int)current_sps,
				(unsigned int)(actual_mps_x100 / 100U),
				(unsigned int)(actual_mps_x100 % 100U), phase);
	} else {
		snprintf(text, sizeof(text), "ACTUAL    0 HS/S  OUTPUT OFF");
	}
	Menu_DrawText(8U, 59U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
}

static void Menu_DrawMotorSpeed(void)
{
	char text[40];
	uint8_t index;
	uint16_t speed = motor_speed_steps[motor_speed_level];
	uint16_t mps_x100 = Menu_MotorSpeedMpsX100(speed);

	Menu_FillScreen(LCD_COLOR_BLACK);
	Menu_DrawText(8U, 2U, 16U,
			LCD_COLOR_CYAN, LCD_COLOR_BLACK, "5. MOTOR SPEED");
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0U, 21U,
			MENU_SCREEN_WIDTH, 1U, LCD_COLOR_CYAN);

	Menu_DrawText(8U, 25U, 12U,
			LCD_COLOR_GRAY, LCD_COLOR_BLACK, "5CM WHEEL  400 HALFSTEP/REV");
	snprintf(text, sizeof(text), "TARGET %4u HS/S  %u.%02u M/S  %u/%u",
			(unsigned int)speed, (unsigned int)(mps_x100 / 100U),
			(unsigned int)(mps_x100 % 100U),
			(unsigned int)(motor_speed_level + 1U),
			(unsigned int)MOTOR_SPEED_LEVEL_COUNT);
	Menu_DrawText(8U, 42U, 12U,
			LCD_COLOR_WHITE, LCD_COLOR_BLACK, text);
	Menu_DrawMotorSpeedRuntime();

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
				LCD_COLOR_RED, LCD_COLOR_BLACK, "RAMP FAILED - OUTPUT OFF");
	} else {
		snprintf(text, sizeof(text), "DAC %u  ~%u.%02uA/PH  HOLD %uS",
				(unsigned int)MOTOR_SPEED_TEST_DAC,
				(unsigned int)(MOTOR_VREF_CURRENT_MA / 1000U),
				(unsigned int)(MOTOR_VREF_CURRENT_MA % 1000U / 10U),
				(unsigned int)(MOTOR_SPEED_TARGET_HOLD_MS / 1000U));
		Menu_DrawText(8U, 92U, 12U,
				LCD_COLOR_GRAY, LCD_COLOR_BLACK, text);
	}
	if (motor_speed_running != 0U) {
		Menu_DrawText(8U, 106U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"C STOP   L/R LOCKED WHILE RUN");
	} else if (motor_speed_armed != 0U) {
		Menu_DrawText(8U, 106U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"ARMED: C START   AUTO CANCEL 5S");
	} else {
		Menu_DrawText(8U, 106U, 12U,
				LCD_COLOR_YELLOW, LCD_COLOR_BLACK,
				"L/R SPEED   C ARM");
	}
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
	motor_speed_armed = 0U;
	motor_speed_error = 0U;
	Menu_DrawMotorSpeed();
}

static void Menu_ToggleMotorSpeed(void)
{
	if (motor_speed_running != 0U) {
		Motor_SpeedTestStop();
		motor_speed_running = 0U;
		motor_speed_armed = 0U;
		motor_speed_error = 0U;
		motor_speed_hold_tick = 0U;
	} else if (motor_speed_armed == 0U) {
		motor_speed_armed = 1U;
		motor_speed_error = 0U;
		motor_speed_action_tick = HAL_GetTick();
	} else if (Motor_SpeedTestStart(motor_speed_steps[motor_speed_level],
			MOTOR_SPEED_TEST_DAC)) {
		motor_speed_running = 1U;
		motor_speed_armed = 0U;
		motor_speed_error = 0U;
		motor_speed_hold_tick = 0U;
		motor_speed_update_tick = 0U;
	} else {
		motor_speed_running = 0U;
		motor_speed_armed = 0U;
		motor_speed_error = 1U;
	}
	Menu_DrawMotorSpeed();
}

static void Menu_UpdateMotorSpeedSafety(void)
{
	uint32_t now = HAL_GetTick();
	uint16_t current_sps;

	if ((motor_speed_armed != 0U)
			&& ((now - motor_speed_action_tick)
					>= MOTOR_SPEED_ARM_TIMEOUT_MS)) {
		motor_speed_armed = 0U;
		Menu_DrawMotorSpeed();
		return;
	}
	if (motor_speed_running == 0U) {
		return;
	}
	if (!Motor_SpeedTestProcess() || !Motor_SpeedTestIsRunning()) {
		Motor_SpeedTestStop();
		motor_speed_running = 0U;
		motor_speed_error = 1U;
		Menu_DrawMotorSpeed();
		return;
	}

	current_sps = Motor_SpeedTestGetCurrentSps();
	if (current_sps >= motor_speed_steps[motor_speed_level]) {
		if (motor_speed_hold_tick == 0U) {
			motor_speed_hold_tick = now;
		} else if ((now - motor_speed_hold_tick)
				>= MOTOR_SPEED_TARGET_HOLD_MS) {
			Motor_SpeedTestStop();
			motor_speed_running = 0U;
			motor_speed_error = 0U;
			motor_speed_hold_tick = 0U;
			Menu_DrawMotorSpeed();
			return;
		}
	}
	if ((now - motor_speed_update_tick) >= MOTOR_SPEED_UPDATE_MS) {
		motor_speed_update_tick = now;
		Menu_DrawMotorSpeedRuntime();
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
				current_view = MENU_VIEW_SENSOR_STATE;
				Menu_DrawSensorState();
			} else if (selected_item == 3U) {
				Sensor_Stop();
				Motor_PhaseTestDisarm();
				motor_phase_target = MOTOR_TARGET_LEFT;
				motor_phase_index = 0U;
				motor_phase_armed = 0U;
				motor_phase_error = 0U;
				current_view = MENU_VIEW_MOTOR_PHASE;
				Menu_DrawMotorPhase();
			} else if (selected_item == 4U) {
				Sensor_Stop();
				Motor_SpeedTestStop();
				motor_speed_level = 0U;
				motor_speed_armed = 0U;
				motor_speed_running = 0U;
				motor_speed_error = 0U;
				motor_speed_hold_tick = 0U;
				motor_speed_update_tick = 0U;
				current_view = MENU_VIEW_MOTOR_SPEED;
				Menu_DrawMotorSpeed();
			} else if (selected_item == 5U) {
				Sensor_Stop();
				Motor_SpeedTestStop();
				FirstDrive_Stop();
				pd_tuning_selected = 0U;
				current_view = MENU_VIEW_PD_TUNING;
				Menu_DrawPdTuning();
			} else if (selected_item == 6U) {
				Motor_SpeedTestStop();
				FirstDrive_Init();
				Menu_EnsureFirstDrivePreview();
				first_drive_update_tick = HAL_GetTick();
				first_drive_preview_tick = 0U;
				current_view = MENU_VIEW_FIRST_DRIVE;
				Menu_DrawFirstDrive();
			} else {
				Motor_SpeedTestStop();
				(void)SecondDrive_Init();
				second_drive_selected = 0U;
				second_drive_arm_tick = 0U;
				second_drive_update_tick = HAL_GetTick();
				current_view = MENU_VIEW_SECOND_DRIVE;
				Menu_DrawSecondDrive();
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
			if (calibration_state != CALIBRATION_COLLECTING) {
				Menu_StartCalibration();
			}
		} else if (calibration_state == CALIBRATION_COLLECTING) {
			Menu_UpdateCalibrationCollecting(0U);
			if (calibration_auto_finish_ready != 0U) {
				Menu_FinishCalibration();
			}
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
			motor_speed_armed = 0U;
			motor_speed_running = 0U;
			motor_speed_hold_tick = 0U;
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
	} else if (current_view == MENU_VIEW_PD_TUNING) {
		if (input == INPUT_CMD_K_HOLD) {
			current_view = MENU_VIEW_MAIN;
			Menu_DrawMain();
		} else if (input == INPUT_CMD_K_SINGLE) {
			pd_tuning_selected++;
			if (pd_tuning_selected >= 3U) {
				pd_tuning_selected = 0U;
			}
			Menu_DrawPdTuning();
		} else if ((input == INPUT_CMD_L_SINGLE)
				|| (input == INPUT_CMD_L_HOLD)) {
			Menu_ChangePdGain(-1);
		} else if ((input == INPUT_CMD_R_SINGLE)
				|| (input == INPUT_CMD_R_HOLD)) {
			Menu_ChangePdGain(1);
		}
	} else if (current_view == MENU_VIEW_FIRST_DRIVE) {
		FirstDrive_Process();
		FirstDriveState_t first_state = FirstDrive_GetState();

		/* Do not wait for the 500ms long-press event while motors run. */
		if (Menu_FirstDriveIsActive(first_state)
				&& Button_IsCenterPressed()) {
			FirstDrive_EmergencyStop();
			Button_IgnoreCenterUntilRelease();
			Menu_RenderFirstDrive(true);
		} else if (((first_state == FIRST_DRIVE_FAULT)
				|| (first_state == FIRST_DRIVE_STOPPED))
				&& ((input == INPUT_CMD_L_SINGLE)
						|| (input == INPUT_CMD_L_HOLD)
						|| (input == INPUT_CMD_R_SINGLE)
						|| (input == INPUT_CMD_R_HOLD))) {
			if ((input == INPUT_CMD_L_SINGLE)
					|| (input == INPUT_CMD_L_HOLD)) {
				first_drive_fault_page = (first_drive_fault_page == 0U)
						? (FIRST_DRIVE_FAULT_PAGE_COUNT - 1U)
						: (uint8_t)(first_drive_fault_page - 1U);
			} else {
				first_drive_fault_page = (uint8_t)
						((first_drive_fault_page + 1U)
								% FIRST_DRIVE_FAULT_PAGE_COUNT);
			}
			Menu_RenderFirstDrive(true);
		} else if (input == INPUT_CMD_K_HOLD) {
			if (Menu_FirstDriveIsActive(first_state)) {
				FirstDrive_EmergencyStop();
				Menu_RenderFirstDrive(true);
			} else {
				FirstDrive_Stop();
				current_view = MENU_VIEW_MAIN;
				Menu_DrawMain();
			}
		} else if (input == INPUT_CMD_K_SINGLE) {
			first_state = FirstDrive_GetState();
			if ((first_state == FIRST_DRIVE_READY)
					&& !Sensor_IsCalibrationComplete()) {
				calibration_state = CALIBRATION_READY;
				current_view = MENU_VIEW_CALIBRATION;
				Menu_DrawCalibrationReady();
			} else if (first_state == FIRST_DRIVE_READY) {
				if (FirstDrive_Arm()) {
					first_drive_arm_tick = HAL_GetTick();
					Menu_RenderFirstDrive(true);
				}
			} else if (first_state == FIRST_DRIVE_ARMED) {
				if (!FirstDrive_Start()) {
					Menu_RenderFirstDrive(true);
				}
			} else if (Menu_FirstDriveIsActive(first_state)) {
				FirstDrive_EmergencyStop();
				Menu_RenderFirstDrive(true);
			}
		} else {
			Menu_UpdateFirstDrive();
		}
	} else if (current_view == MENU_VIEW_SECOND_DRIVE) {
		FirstDriveState_t second_state;

		SecondDrive_Process();
		second_state = SecondDrive_GetState();
		/* Match First Drive's immediate physical emergency-stop path. */
		if (Menu_FirstDriveIsActive(second_state)
				&& Button_IsCenterPressed()) {
			SecondDrive_EmergencyStop();
			Button_IgnoreCenterUntilRelease();
			Menu_RenderSecondDrive();
		} else if (input == INPUT_CMD_K_HOLD) {
			if (Menu_FirstDriveIsActive(second_state)) {
				SecondDrive_EmergencyStop();
				Menu_RenderSecondDrive();
			} else {
				SecondDrive_Stop();
				current_view = MENU_VIEW_MAIN;
				Menu_DrawMain();
			}
		} else if ((second_state == FIRST_DRIVE_READY)
				&& ((input == INPUT_CMD_L_SINGLE)
						|| (input == INPUT_CMD_L_HOLD))) {
			Menu_ChangeSecondDriveSetting(-1);
		} else if ((second_state == FIRST_DRIVE_READY)
				&& ((input == INPUT_CMD_R_SINGLE)
						|| (input == INPUT_CMD_R_HOLD))) {
			Menu_ChangeSecondDriveSetting(1);
		} else if (input == INPUT_CMD_K_SINGLE) {
			if ((second_state == FIRST_DRIVE_READY)
					&& !Sensor_IsCalibrationComplete()) {
				SecondDrive_Stop();
				calibration_state = CALIBRATION_READY;
				current_view = MENU_VIEW_CALIBRATION;
				Menu_DrawCalibrationReady();
			} else if (second_state == FIRST_DRIVE_READY) {
				if (second_drive_selected == 0U) {
					second_drive_selected = 1U;
					Menu_RenderSecondDrive();
				} else if (SecondDrive_Arm()) {
					second_drive_arm_tick = HAL_GetTick();
					Menu_RenderSecondDrive();
				}
			} else if (second_state == FIRST_DRIVE_ARMED) {
				if (!SecondDrive_Start()) {
					Menu_RenderSecondDrive();
				}
			} else if (Menu_FirstDriveIsActive(second_state)) {
				SecondDrive_EmergencyStop();
				Menu_RenderSecondDrive();
			}
		} else {
			Menu_UpdateSecondDrive();
		}
	} else if (input == INPUT_CMD_K_HOLD) {
		current_view = MENU_VIEW_MAIN;
		Menu_DrawMain();
	}
}
