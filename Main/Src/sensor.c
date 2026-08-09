/*
 * sensor.c
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#include "sensor.h"

#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

#include <string.h>

#define TIM_IR                 (&htim6)
#define TIM_SENSOR_DELAY       (&htim3)
#define ADC_SENSOR             (&hadc1)
#define TIM_IR_PAIR_PERIOD     124U
#define TIM_IR_FIXED_PERIOD    999U
#define SENSOR_POSITION_STEP   1000
#define SENSOR_POSITION_FLOOR  200U
#define SENSOR_MEDIAN_LENGTH   3U
#define SENSOR_CALIBRATION_HISTOGRAM_BINS 512U
#define SENSOR_CALIBRATION_HISTOGRAM_SHIFT 5U
#define SENSOR_CALIBRATION_BLACK_MIN_FRAMES 512U
#define SENSOR_CALIBRATION_WHITE_TOP_COUNT 32U
#define SENSOR_CALIBRATION_WHITE_TRIM_COUNT 4U
#define SENSOR_CALIBRATION_WHITE_MIN_DELTA \
	(SENSOR_CALIBRATION_MIN_RANGE / 2U)
#define SENSOR_CALIBRATION_MIN_SNR 6U

#define TIM_IR_IRQ_Handler     HAL_TIM6_IRQ_Handler
#define SENSOR_IRQ_Handler     HAL_ADC1_IRQ_Handler

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} IR_TypeDef;

typedef struct {
	uint8_t next_adc_index;
	uint8_t active_adc_index;
	uint8_t active_ir_index;
	uint16_t sensor_raw[SENSOR_COUNT];
} IR_Data_t;

typedef struct {
	volatile uint32_t number;
	volatile uint16_t raw[SENSOR_COUNT];
} SensorFrameSlot_t;

static volatile IR_Data_t ir_data;
static volatile bool sensor_running = false;
static volatile bool sample_pending = false;
static volatile bool emitter_enabled = false;
static volatile uint32_t sample_count = 0U;
static volatile uint32_t sensor_frame_count = 0U;
static volatile SensorFrameSlot_t
		sensor_frames[SENSOR_FRAME_BUFFER_COUNT];
static volatile SensorLightMode_t light_mode = SENSOR_LIGHT_PAIR;
static volatile uint8_t light_parameter = 0U;
static volatile uint8_t light_pin_status[SENSOR_EMITTER_PAIR_COUNT];
static uint16_t calibration_low[SENSOR_COUNT];
static uint16_t calibration_high[SENSOR_COUNT];
static uint32_t calibration_black_sum[SENSOR_COUNT];
static uint16_t calibration_black_histogram[SENSOR_COUNT]
		[SENSOR_CALIBRATION_HISTOGRAM_BINS];
static uint16_t calibration_white_top[SENSOR_COUNT]
		[SENSOR_CALIBRATION_WHITE_TOP_COUNT];
static uint16_t calibration_white_count[SENSOR_COUNT];
static uint16_t calibration_black_noise[SENSOR_COUNT];
static uint32_t calibration_black_frame_count = 0U;
static bool calibration_black_ready = false;
static uint8_t calibration_valid_mask = 0U;
static bool calibration_complete = false;
static uint8_t sensor_state_mask = 0U;

static void Sensor_PublishFrame(void)
{
	uint32_t next_frame = sensor_frame_count + 1U;
	volatile SensorFrameSlot_t *slot =
			&sensor_frames[next_frame & (SENSOR_FRAME_BUFFER_COUNT - 1U)];
	uint8_t index;

	/* number == 0 marks a slot being written.  The barriers make the frame
	 * payload visible before its sequence number is published. */
	slot->number = 0U;
	__DMB();
	for (index = 0U; index < SENSOR_COUNT; index++) {
		slot->raw[index] = ir_data.sensor_raw[index];
	}
	__DMB();
	slot->number = next_frame;
	__DMB();
	sensor_frame_count = next_frame;
}

static bool Sensor_CopyFrame(uint32_t frame_number,
		uint16_t values[SENSOR_COUNT])
{
	volatile SensorFrameSlot_t *slot =
			&sensor_frames[frame_number & (SENSOR_FRAME_BUFFER_COUNT - 1U)];
	uint32_t number_before;
	uint32_t number_after;
	uint8_t index;

	number_before = slot->number;
	if ((frame_number == 0U) || (number_before != frame_number)) {
		return false;
	}
	__DMB();
	for (index = 0U; index < SENSOR_COUNT; index++) {
		values[index] = slot->raw[index];
	}
	__DMB();
	number_after = slot->number;
	return number_after == number_before;
}

static uint16_t Sensor_Median3(uint16_t first, uint16_t second,
		uint16_t third)
{
	if (first > second) {
		uint16_t temporary = first;
		first = second;
		second = temporary;
	}
	if (second > third) {
		uint16_t temporary = second;
		second = third;
		third = temporary;
	}
	if (first > second) {
		second = first;
	}
	return second;
}

static bool Sensor_CopyFilteredFrame(uint32_t frame_number,
		uint16_t values[SENSOR_COUNT])
{
	uint16_t current[SENSOR_COUNT];
	uint16_t previous[SENSOR_COUNT];
	uint16_t oldest[SENSOR_COUNT];
	uint8_t index;

	if (!Sensor_CopyFrame(frame_number, current)) {
		return false;
	}
	/* The first two frames, or an oldest frame whose predecessors have already
	 * left the ring, are still coherent.  Use them without inventing history. */
	if ((frame_number < SENSOR_MEDIAN_LENGTH)
			|| !Sensor_CopyFrame(frame_number - 1U, previous)
			|| !Sensor_CopyFrame(frame_number - 2U, oldest)) {
		memcpy(values, current, sizeof(current));
		return true;
	}
	for (index = 0U; index < SENSOR_COUNT; index++) {
		values[index] = Sensor_Median3(oldest[index], previous[index],
				current[index]);
	}
	return true;
}

static uint16_t Sensor_BlackQuantile(uint8_t sensor_index,
		uint32_t percentile)
{
	uint32_t target;
	uint32_t accumulated = 0U;
	uint16_t bin;

	if ((sensor_index >= SENSOR_COUNT)
			|| (calibration_black_frame_count == 0U)) {
		return 0U;
	}
	target = ((calibration_black_frame_count * percentile) + 99U) / 100U;
	for (bin = 0U; bin < SENSOR_CALIBRATION_HISTOGRAM_BINS; bin++) {
		accumulated += calibration_black_histogram[sensor_index][bin];
		if (accumulated >= target) {
			uint32_t upper = (((uint32_t)bin + 1U)
					<< SENSOR_CALIBRATION_HISTOGRAM_SHIFT) - 1U;

			return (uint16_t)((upper > SENSOR_ADC_OVERSAMPLED_MAX)
					? SENSOR_ADC_OVERSAMPLED_MAX : upper);
		}
	}
	return SENSOR_ADC_OVERSAMPLED_MAX;
}

static uint16_t Sensor_GetWhiteReference(uint8_t sensor_index)
{
	uint16_t count = calibration_white_count[sensor_index];
	uint16_t trimmed_count;
	uint32_t sum = 0U;
	uint16_t index;

	if (count == 0U) {
		return calibration_low[sensor_index];
	}
	trimmed_count = (count > SENSOR_CALIBRATION_WHITE_TRIM_COUNT)
			? (uint16_t)(count - SENSOR_CALIBRATION_WHITE_TRIM_COUNT)
			: count;
	for (index = 0U; index < trimmed_count; index++) {
		sum += calibration_white_top[sensor_index][index];
	}
	return (uint16_t)((sum + (trimmed_count / 2U)) / trimmed_count);
}

/*
 * U9 (ULN2803) drives the emitters as four mirrored pairs:
 * IR0 -> sensors 0/7, IR1 -> 1/6, IR2 -> 2/5, IR3 -> 3/4.
 * MCU pins IR4..IR7 are not connected to U9 emitter inputs.
 */
static const IR_TypeDef ir_outputs[SENSOR_EMITTER_PAIR_COUNT] = {
	{ IR_0_GPIO_Port, IR_0_Pin },
	{ IR_1_GPIO_Port, IR_1_Pin },
	{ IR_2_GPIO_Port, IR_2_Pin },
	{ IR_3_GPIO_Port, IR_3_Pin }
};

static uint8_t IR_GetPairIndex(uint8_t sensor_index)
{
	sensor_index &= 0x07U;
	return (sensor_index < SENSOR_EMITTER_PAIR_COUNT)
			? sensor_index : (uint8_t)(7U - sensor_index);
}

static void IR_Enable(uint8_t index)
{
	GPIO_TypeDef *port = ir_outputs[index].port;
	uint32_t pin_number = 0U;
	uint32_t pin_mask = ir_outputs[index].pin;
	uint8_t status = 0U;

	HAL_GPIO_WritePin(ir_outputs[index].port, ir_outputs[index].pin,
			GPIO_PIN_SET);

	/* Latch the state while the short emitter pulse is actually HIGH. */
	while ((pin_mask & 0x01U) == 0U) {
		pin_mask >>= 1U;
		pin_number++;
	}
	if (((port->MODER >> (pin_number * 2U)) & 0x03U) == 0x01U) {
		status |= 0x01U;
	}
	if ((port->ODR & ir_outputs[index].pin) != 0U) {
		status |= 0x02U;
	}
	__DSB();
	__NOP();
	__NOP();
	if ((port->IDR & ir_outputs[index].pin) != 0U) {
		status |= 0x04U;
	}
	light_pin_status[index] = status;
}

static void IR_Disable(uint8_t index)
{
	HAL_GPIO_WritePin(ir_outputs[index].port, ir_outputs[index].pin,
			GPIO_PIN_RESET);
}

static void IR_DisableAll(void)
{
	uint8_t index;

	for (index = 0U; index < SENSOR_EMITTER_PAIR_COUNT; index++) {
		IR_Disable(index);
	}
}

void TIM_IR_IRQ_Handler(void)
{
	uint8_t adc_index;
	uint8_t ir_index = 0U;

	if (!sensor_running) {
		return;
	}

	/* Fail-safe: never leave an emitter enabled after a missed conversion. */
	if (sample_pending && emitter_enabled) {
		IR_Disable(ir_data.active_ir_index);
		emitter_enabled = false;
		sample_pending = false;
	}

	adc_index = ir_data.next_adc_index;
	ir_data.active_adc_index = adc_index;
	ir_data.next_adc_index = (adc_index + 1U) & 0x07U;

	if (light_mode == SENSOR_LIGHT_PAIR) {
		ir_index = IR_GetPairIndex(
				(adc_index + light_parameter) & 0x07U);
	} else if (light_mode == SENSOR_LIGHT_FIXED) {
		ir_index = light_parameter & 0x03U;
	}

	if (light_mode != SENSOR_LIGHT_OFF) {
		ir_data.active_ir_index = ir_index;
		IR_Enable(ir_index);
		emitter_enabled = true;
	}
	sample_pending = true;

	/* TIM3 creates the short settling delay and then triggers ADC1. */
	__HAL_TIM_SET_COUNTER(TIM_SENSOR_DELAY, 0U);
	__HAL_TIM_CLEAR_FLAG(TIM_SENSOR_DELAY, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE(TIM_SENSOR_DELAY);
}

void SENSOR_IRQ_Handler(void)
{
	uint8_t index;
	uint32_t adc_raw;

	if (!sensor_running) {
		return;
	}

	index = ir_data.active_adc_index;
	adc_raw = HAL_ADC_GetValue(ADC_SENSOR);

	if (emitter_enabled) {
		IR_Disable(ir_data.active_ir_index);
		emitter_enabled = false;
	}
	sample_pending = false;
	/* Preserve the complete 4x-oversampled ADC result.  Reducing this to an
	 * 8-bit display value here used to erase the small black/white difference
	 * before calibration had a chance to amplify it. */
	if (adc_raw > SENSOR_ADC_OVERSAMPLED_MAX) {
		adc_raw = SENSOR_ADC_OVERSAMPLED_MAX;
	}
	ir_data.sensor_raw[index] = (uint16_t)adc_raw;
	sample_count++;
	if (index == (SENSOR_COUNT - 1U)) {
		Sensor_PublishFrame();
	}
}

void Sensor_Start(void)
{
	uint8_t index;

	if (sensor_running) {
		return;
	}

	IR_DisableAll();
	ir_data.next_adc_index = 0U;
	ir_data.active_adc_index = 0U;
	ir_data.active_ir_index = 0U;
	sample_pending = false;
	emitter_enabled = false;
	sample_count = 0U;
	sensor_frame_count = 0U;

	for (index = 0U; index < SENSOR_COUNT; index++) {
		ir_data.sensor_raw[index] = 0U;
		if (index < SENSOR_EMITTER_PAIR_COUNT) {
			light_pin_status[index] = 0U;
		}
	}
	for (index = 0U; index < SENSOR_FRAME_BUFFER_COUNT; index++) {
		sensor_frames[index].number = 0U;
	}

	if (HAL_ADCEx_Calibration_Start(ADC_SENSOR, ADC_SINGLE_ENDED) != HAL_OK) {
		return;
	}

	if (HAL_ADC_Start_IT(ADC_SENSOR) != HAL_OK) {
		return;
	}

	sensor_running = true;
	__HAL_TIM_SET_AUTORELOAD(TIM_IR,
			(light_mode == SENSOR_LIGHT_FIXED)
			? TIM_IR_FIXED_PERIOD : TIM_IR_PAIR_PERIOD);
	__HAL_TIM_SET_COUNTER(TIM_IR, 0U);
	__HAL_TIM_CLEAR_FLAG(TIM_IR, TIM_FLAG_UPDATE);
	if (HAL_TIM_Base_Start_IT(TIM_IR) != HAL_OK) {
		sensor_running = false;
		HAL_ADC_Stop_IT(ADC_SENSOR);
		IR_DisableAll();
	}
}

void Sensor_Stop(void)
{
	sensor_running = false;
	HAL_TIM_Base_Stop_IT(TIM_IR);
	__HAL_TIM_DISABLE(TIM_SENSOR_DELAY);
	HAL_ADC_Stop_IT(ADC_SENSOR);
	IR_DisableAll();
	sample_pending = false;
	emitter_enabled = false;
}

bool Sensor_IsRunning(void)
{
	return sensor_running;
}

void Sensor_SetLightMode(SensorLightMode_t mode, uint8_t parameter)
{
	bool restart = sensor_running;

	if (restart) {
		Sensor_Stop();
	}

	light_mode = mode;
	light_parameter = (mode == SENSOR_LIGHT_FIXED)
			? (parameter & 0x03U) : (parameter & 0x07U);

	if (restart) {
		Sensor_Start();
	}
}

void Sensor_GetRaw(uint16_t values[SENSOR_COUNT])
{
	uint32_t latest_frame;
	uint8_t attempt;
	uint8_t index;

	/* Prefer a coherent completed frame.  Retry if the ISR happens to replace
	 * its ring-buffer slot while it is being copied. */
	for (attempt = 0U; attempt < 3U; attempt++) {
		latest_frame = sensor_frame_count;
		__DMB();
		if ((latest_frame != 0U)
				&& Sensor_CopyFrame(latest_frame, values)) {
			return;
		}
	}

	for (index = 0U; index < SENSOR_COUNT; index++) {
		/* Before the first full frame, return the latest individual samples. */
		values[index] = ir_data.sensor_raw[index];
	}
}

uint32_t Sensor_GetSampleCount(void)
{
	return sample_count;
}

uint32_t Sensor_GetFrameCount(void)
{
	return sensor_frame_count;
}

bool Sensor_GetFrameRawAfter(uint32_t *cursor,
		uint16_t values[SENSOR_COUNT])
{
	uint32_t latest_frame;
	uint32_t target_frame;

	if ((cursor == NULL) || (values == NULL)) {
		return false;
	}

	latest_frame = sensor_frame_count;
	__DMB();
	if (*cursor >= latest_frame) {
		return false;
	}

	target_frame = *cursor + 1U;
	if ((latest_frame - target_frame) >= SENSOR_FRAME_BUFFER_COUNT) {
		target_frame = latest_frame - SENSOR_FRAME_BUFFER_COUNT + 1U;
	}
	if (!Sensor_CopyFrame(target_frame, values)) {
		return false;
	}

	*cursor = target_frame;
	return true;
}

bool Sensor_GetFrameFilteredAfter(uint32_t *cursor,
		uint16_t values[SENSOR_COUNT])
{
	uint32_t latest_frame;
	uint32_t target_frame;

	if ((cursor == NULL) || (values == NULL)) {
		return false;
	}
	latest_frame = sensor_frame_count;
	__DMB();
	if (*cursor >= latest_frame) {
		return false;
	}
	target_frame = *cursor + 1U;
	if ((latest_frame - target_frame) >= SENSOR_FRAME_BUFFER_COUNT) {
		target_frame = latest_frame - SENSOR_FRAME_BUFFER_COUNT + 1U;
	}
	if (!Sensor_CopyFilteredFrame(target_frame, values)) {
		return false;
	}
	*cursor = target_frame;
	return true;
}

uint8_t Sensor_GetLightPinStatus(uint8_t index)
{
	return light_pin_status[IR_GetPairIndex(index)];
}

void Sensor_CalibrationReset(void)
{
	uint8_t index;

	memset(calibration_black_histogram, 0,
			sizeof(calibration_black_histogram));
	memset(calibration_white_top, 0, sizeof(calibration_white_top));
	for (index = 0U; index < SENSOR_COUNT; index++) {
		calibration_low[index] = 0U;
		calibration_high[index] = 0U;
		calibration_black_sum[index] = 0U;
		calibration_black_noise[index] = 0U;
		calibration_white_count[index] = 0U;
	}
	calibration_black_frame_count = 0U;
	calibration_black_ready = false;
	calibration_valid_mask = 0U;
	calibration_complete = false;
}

void Sensor_CalibrationCaptureBlack(const uint16_t raw[SENSOR_COUNT])
{
	uint8_t index;

	if ((raw == NULL) || calibration_black_ready) {
		return;
	}
	calibration_black_frame_count++;
	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t bin = raw[index] >> SENSOR_CALIBRATION_HISTOGRAM_SHIFT;

		if (bin >= SENSOR_CALIBRATION_HISTOGRAM_BINS) {
			bin = SENSOR_CALIBRATION_HISTOGRAM_BINS - 1U;
		}
		if (calibration_black_histogram[index][bin] < UINT16_MAX) {
			calibration_black_histogram[index][bin]++;
		}
		calibration_black_sum[index] += raw[index];
		calibration_low[index] = (uint16_t)((calibration_black_sum[index]
				+ (calibration_black_frame_count / 2U))
				/ calibration_black_frame_count);
		calibration_high[index] = calibration_low[index];
	}
}

bool Sensor_CalibrationFinishBlack(void)
{
	uint8_t index;

	if (calibration_black_frame_count < SENSOR_CALIBRATION_BLACK_MIN_FRAMES) {
		return false;
	}
	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t median = Sensor_BlackQuantile(index, 50U);
		uint16_t upper = Sensor_BlackQuantile(index, 99U);

		/* P95 becomes normalized zero.  A hand shadow or isolated ADC spike can
		 * no longer raise the black baseline as an absolute maximum would. */
		calibration_low[index] = Sensor_BlackQuantile(index, 95U);
		calibration_black_noise[index] = (upper >= median)
				? (uint16_t)(upper - median) : 0U;
		calibration_high[index] = calibration_low[index];
	}
	calibration_black_ready = true;
	return true;
}

void Sensor_CalibrationCaptureWhite(const uint16_t raw[SENSOR_COUNT])
{
	uint8_t index;

	if ((raw == NULL) || !calibration_black_ready) {
		return;
	}
	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t count = calibration_white_count[index];
		uint16_t position;
		uint32_t minimum_white = (uint32_t)calibration_low[index]
				+ SENSOR_CALIBRATION_WHITE_MIN_DELTA;

		/* Ignore ordinary black-floor samples.  The retained array contains only
		 * the strongest 32 observations and is kept in ascending order. */
		if (raw[index] < minimum_white) {
			continue;
		}
		if (count < SENSOR_CALIBRATION_WHITE_TOP_COUNT) {
			position = count;
			while ((position > 0U)
					&& (calibration_white_top[index][position - 1U]
							> raw[index])) {
				calibration_white_top[index][position] =
						calibration_white_top[index][position - 1U];
				position--;
			}
			calibration_white_top[index][position] = raw[index];
			calibration_white_count[index] = count + 1U;
		} else if (raw[index] > calibration_white_top[index][0]) {
			position = 0U;
			while (((position + 1U) < SENSOR_CALIBRATION_WHITE_TOP_COUNT)
					&& (calibration_white_top[index][position + 1U]
							< raw[index])) {
				calibration_white_top[index][position] =
						calibration_white_top[index][position + 1U];
				position++;
			}
			calibration_white_top[index][position] = raw[index];
		}
		calibration_high[index] = Sensor_GetWhiteReference(index);
	}
}

uint8_t Sensor_GetCalibrationCoverageMask(void)
{
	uint8_t mask = 0U;
	uint8_t index;

	if (!calibration_black_ready) {
		return 0U;
	}
	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t range = (calibration_high[index] >= calibration_low[index])
				? (uint16_t)(calibration_high[index] - calibration_low[index])
				: 0U;

		if ((calibration_white_count[index]
				>= SENSOR_CALIBRATION_WHITE_TOP_COUNT)
				&& (range >= SENSOR_CALIBRATION_MIN_RANGE)
				&& ((calibration_black_noise[index] == 0U)
						|| (range >= ((uint32_t)calibration_black_noise[index]
								* SENSOR_CALIBRATION_MIN_SNR)))) {
			mask |= (uint8_t)(1U << index);
		}
	}
	return mask;
}

uint8_t Sensor_CalibrationFinish(void)
{
	uint8_t index;
	uint8_t mask = 0U;
	uint8_t coverage_mask = Sensor_GetCalibrationCoverageMask();

	if (!calibration_black_ready) {
		calibration_valid_mask = 0U;
		calibration_complete = false;
		return 0U;
	}

	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint16_t range;

		calibration_high[index] = Sensor_GetWhiteReference(index);
		range = (calibration_high[index] >= calibration_low[index])
				? (uint16_t)(calibration_high[index] - calibration_low[index])
				: 0U;
		if ((calibration_high[index] >= calibration_low[index])
				&& ((coverage_mask & (uint8_t)(1U << index)) != 0U)
				&& (range >= SENSOR_CALIBRATION_MIN_RANGE)
				&& ((calibration_black_noise[index] == 0U)
						|| (range >= ((uint32_t)calibration_black_noise[index]
								* SENSOR_CALIBRATION_MIN_SNR)))) {
			mask |= (uint8_t)(1U << index);
		}
	}
	calibration_valid_mask = mask;
	calibration_complete = (mask == 0xFFU);
	return mask;
}

bool Sensor_IsCalibrationComplete(void)
{
	return calibration_complete;
}

uint8_t Sensor_GetCalibrationValidMask(void)
{
	return calibration_valid_mask;
}

void Sensor_GetCalibration(uint16_t low[SENSOR_COUNT],
		uint16_t high[SENSOR_COUNT])
{
	uint8_t index;

	for (index = 0U; index < SENSOR_COUNT; index++) {
		low[index] = calibration_low[index];
		high[index] = calibration_high[index];
	}
}

bool Sensor_GetRawThresholds(uint8_t index, uint16_t *black_max,
		uint16_t *white_min)
{
	uint32_t range;

	if ((index >= SENSOR_COUNT) || (black_max == NULL) || (white_min == NULL)
			|| ((calibration_valid_mask & (1U << index)) == 0U)) {
		return false;
	}

	range = calibration_high[index] - calibration_low[index];
	*black_max = (uint16_t)(calibration_low[index]
			+ ((range * SENSOR_TRACK_WHITE_OFF) / SENSOR_NORMALIZED_MAX));
	*white_min = (uint16_t)(calibration_low[index]
			+ ((range * SENSOR_TRACK_WHITE_ON) / SENSOR_NORMALIZED_MAX));
	return true;
}

uint16_t Sensor_NormalizeRaw(uint8_t index, uint16_t raw)
{
	uint16_t low;
	uint16_t high;

	if ((index >= SENSOR_COUNT)
			|| ((calibration_valid_mask & (1U << index)) == 0U)) {
		return 0U;
	}

	low = calibration_low[index];
	high = calibration_high[index];
	if (raw <= low) {
		return 0U;
	}
	if (raw >= high) {
		return SENSOR_NORMALIZED_MAX;
	}

	/* Amplify only the reflectance above the measured black-floor baseline.
	 * The absolute ADC/DC level is never multiplied, so a high baseline cannot
	 * force the normalized signal into saturation. */
	return (uint16_t)((((uint32_t)(raw - low) * SENSOR_NORMALIZED_MAX)
			+ ((high - low) / 2U)) / (high - low));
}

void Sensor_NormalizeFrame(const uint16_t raw[SENSOR_COUNT],
		uint16_t normalized[SENSOR_COUNT])
{
	uint8_t index;

	for (index = 0U; index < SENSOR_COUNT; index++) {
		normalized[index] = Sensor_NormalizeRaw(index, raw[index]);
	}
}

void Sensor_GetNormalized(uint16_t values[SENSOR_COUNT])
{
	uint16_t raw[SENSOR_COUNT];
	uint32_t latest_frame = sensor_frame_count;

	__DMB();
	if ((latest_frame == 0U)
			|| !Sensor_CopyFilteredFrame(latest_frame, raw)) {
		Sensor_GetRaw(raw);
	}
	Sensor_NormalizeFrame(raw, values);
}

uint8_t Sensor_ApplyStateHysteresis(
		const uint16_t normalized[SENSOR_COUNT], uint8_t previous_mask)
{
	uint8_t index;
	uint8_t mask = previous_mask;

	for (index = 0U; index < SENSOR_COUNT; index++) {
		uint8_t bit = (uint8_t)(1U << index);

		if (normalized[index] >= SENSOR_TRACK_WHITE_ON) {
			mask |= bit;
		} else if (normalized[index] <= SENSOR_TRACK_WHITE_OFF) {
			mask &= (uint8_t)~bit;
		}
	}
	return mask;
}

void Sensor_StateReset(void)
{
	sensor_state_mask = 0U;
}

uint8_t Sensor_UpdateState(int16_t *line_position)
{
	uint16_t normalized[SENSOR_COUNT];
	int32_t weighted_sum = 0;
	uint32_t strength_sum = 0U;
	uint8_t index;

	if (line_position != NULL) {
		*line_position = 0;
	}
	if (!calibration_complete) {
		sensor_state_mask = 0U;
		return sensor_state_mask;
	}

	Sensor_GetNormalized(normalized);
	sensor_state_mask = Sensor_ApplyStateHysteresis(normalized,
			sensor_state_mask);
	for (index = SENSOR_LINE_SENSOR_FIRST;
			index <= SENSOR_LINE_SENSOR_LAST; index++) {
		/* S0/S7 are direction-marker sensors only.  Position is the analogue
		 * centre of S1..S6, with S1=-2500 and S6=+2500. */
		if (normalized[index] > SENSOR_POSITION_FLOOR) {
			uint16_t strength = normalized[index] - SENSOR_POSITION_FLOOR;
			int32_t position = SENSOR_LINE_POSITION_MIN
					+ ((int32_t)(index - SENSOR_LINE_SENSOR_FIRST)
							* SENSOR_POSITION_STEP);

			strength_sum += strength;
			weighted_sum += position * strength;
		}
	}

	if (((sensor_state_mask & SENSOR_LINE_MASK) != 0U)
			&& (strength_sum != 0U)
			&& (line_position != NULL)) {
		*line_position = (int16_t)(weighted_sum / (int32_t)strength_sum);
	}

	return sensor_state_mask;
}

static uint8_t Sensor_CountMaskBits(uint8_t mask)
{
	uint8_t count = 0U;

	while (mask != 0U) {
		count += mask & 0x01U;
		mask >>= 1U;
	}
	return count;
}

static void Sensor_RefineLineCluster(SensorLineCluster_t *cluster,
		const SensorLineMeasurement_t *measurement, uint8_t line_mask)
{
	int32_t weighted_sum = 0;
	uint32_t strength_sum = 0U;
	uint8_t index;

	if ((cluster == NULL) || (measurement == NULL)) {
		return;
	}

	for (index = cluster->first_sensor; index <= cluster->last_sensor;
			index++) {
		int32_t position = SENSOR_LINE_POSITION_MIN
				+ ((int32_t)(index - SENSOR_LINE_SENSOR_FIRST)
						* SENSOR_LINE_POSITION_STEP);

		strength_sum += measurement->line_strength[index];
		weighted_sum += position * measurement->line_strength[index];
	}

	/* Use one quiet S1..S6 neighbour as an analogue shoulder.  Never cross
	 * into S0/S7 and never borrow a sensor quarantined as marker spill. */
	if (cluster->first_sensor > SENSOR_LINE_SENSOR_FIRST) {
		uint8_t shoulder = cluster->first_sensor - 1U;
		uint8_t bit = (uint8_t)(1U << shoulder);

		if (((line_mask & bit) == 0U)
				&& ((measurement->spill_mask & bit) == 0U)) {
			int32_t position = SENSOR_LINE_POSITION_MIN
					+ ((int32_t)(shoulder - SENSOR_LINE_SENSOR_FIRST)
							* SENSOR_LINE_POSITION_STEP);

			strength_sum += measurement->line_strength[shoulder];
			weighted_sum += position
					* measurement->line_strength[shoulder];
		}
	}
	if (cluster->last_sensor < SENSOR_LINE_SENSOR_LAST) {
		uint8_t shoulder = cluster->last_sensor + 1U;
		uint8_t bit = (uint8_t)(1U << shoulder);

		if (((line_mask & bit) == 0U)
				&& ((measurement->spill_mask & bit) == 0U)) {
			int32_t position = SENSOR_LINE_POSITION_MIN
					+ ((int32_t)(shoulder - SENSOR_LINE_SENSOR_FIRST)
							* SENSOR_LINE_POSITION_STEP);

			strength_sum += measurement->line_strength[shoulder];
			weighted_sum += position
					* measurement->line_strength[shoulder];
		}
	}

	cluster->strength = strength_sum;
	cluster->position = (strength_sum != 0U)
			? (int16_t)(weighted_sum / (int32_t)strength_sum)
			: (int16_t)(SENSOR_LINE_POSITION_MIN
					+ ((int32_t)(cluster->first_sensor
							- SENSOR_LINE_SENSOR_FIRST)
							* SENSOR_LINE_POSITION_STEP));
}

bool Sensor_UseLineCluster(SensorLineMeasurement_t *measurement,
		uint8_t cluster_index)
{
	const SensorLineCluster_t *cluster;

	if ((measurement == NULL)
			|| (cluster_index >= measurement->cluster_count)) {
		return false;
	}
	cluster = &measurement->clusters[cluster_index];
	measurement->position = cluster->position;
	measurement->strength = cluster->strength;
	measurement->selected_mask = cluster->mask;
	measurement->center_count = Sensor_CountMaskBits(cluster->mask);
	measurement->edge_mask = measurement->marker_mask & SENSOR_MARKER_MASK;
	measurement->edge_only = false;
	measurement->line_valid = cluster->strength
			>= SENSOR_LINE_STRENGTH_MIN;
	return measurement->line_valid;
}

bool Sensor_UseLineMask(SensorLineMeasurement_t *measurement,
		uint8_t line_mask, uint8_t spill_mask)
{
	uint8_t selected_cluster = 0U;
	uint32_t selected_distance = UINT32_MAX;
	uint8_t index;

	if (measurement == NULL) {
		return false;
	}
	line_mask &= SENSOR_LINE_MASK;
	measurement->position = 0;
	measurement->strength = 0U;
	measurement->selected_mask = 0U;
	measurement->spill_mask = spill_mask & SENSOR_LINE_MASK;
	measurement->center_count = 0U;
	measurement->cluster_count = 0U;
	measurement->edge_only = false;
	measurement->line_valid = false;
	memset(measurement->clusters, 0, sizeof(measurement->clusters));

	index = SENSOR_LINE_SENSOR_FIRST;
	while (index <= SENSOR_LINE_SENSOR_LAST) {
		SensorLineCluster_t *cluster;

		if ((line_mask & (uint8_t)(1U << index)) == 0U) {
			index++;
			continue;
		}
		if (measurement->cluster_count >= SENSOR_LINE_MAX_CLUSTERS) {
			break;
		}
		cluster = &measurement->clusters[measurement->cluster_count];
		cluster->first_sensor = index;
		while ((index <= SENSOR_LINE_SENSOR_LAST)
				&& ((line_mask & (uint8_t)(1U << index)) != 0U)) {
			cluster->mask |= (uint8_t)(1U << index);
			cluster->last_sensor = index;
			index++;
		}
		Sensor_RefineLineCluster(cluster, measurement, line_mask);
		measurement->cluster_count++;
	}

	if (measurement->cluster_count == 0U) {
		return false;
	}

	/* Without drive history, the centre-nearest inner group is the safest
	 * preview/start candidate.  First Drive can replace it using continuity. */
	for (index = 0U; index < measurement->cluster_count; index++) {
		uint32_t distance = (measurement->clusters[index].position < 0)
				? (uint32_t)(-measurement->clusters[index].position)
				: (uint32_t)measurement->clusters[index].position;

		if ((distance < selected_distance)
				|| ((distance == selected_distance)
						&& (measurement->clusters[index].strength
								> measurement->clusters[selected_cluster].strength))) {
			selected_cluster = index;
			selected_distance = distance;
		}
	}
	return Sensor_UseLineCluster(measurement, selected_cluster);
}

bool Sensor_GetLineMeasurement(SensorLineMeasurement_t *measurement)
{
	uint16_t normalized[SENSOR_COUNT];
	uint8_t state_mask;
	uint8_t line_mask;
	uint8_t index;

	if (measurement == NULL) {
		return false;
	}
	memset(measurement, 0, sizeof(*measurement));
	if (!calibration_complete) {
		return false;
	}

	Sensor_GetNormalized(normalized);
	sensor_state_mask = Sensor_ApplyStateHysteresis(normalized,
			sensor_state_mask);
	state_mask = sensor_state_mask;
	measurement->state_mask = state_mask;
	line_mask = state_mask & SENSOR_LINE_MASK;
	measurement->raw_line_mask = line_mask;

	for (index = 0U; index < SENSOR_COUNT; index++) {
		if (normalized[index] >= SENSOR_MARKER_WHITE_MIN) {
			measurement->marker_mask |= (uint8_t)(1U << index);
		}
	}

	/* Cache analogue S1..S6 strength so First Drive can rebuild the inner
	 * groups after quarantining marker spill.  S0/S7 are never cached here. */
	for (index = SENSOR_LINE_SENSOR_FIRST;
			index <= SENSOR_LINE_SENSOR_LAST; index++) {
		if (normalized[index] > SENSOR_LINE_POSITION_FLOOR) {
			measurement->line_strength[index] = normalized[index]
					- SENSOR_LINE_POSITION_FLOOR;
		}
	}

	measurement->edge_mask = measurement->marker_mask & SENSOR_MARKER_MASK;
	(void)Sensor_UseLineMask(measurement, line_mask, 0U);
	return true;
}

void Sensor_Test_Raw(void)
{
}

void Sensor_Test_Normalized(void)
{
}

void Sensor_Test_State(void)
{
}
