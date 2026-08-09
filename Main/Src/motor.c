/*
 * motor.c
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#include "motor.h"

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "dac.h"

#include <stddef.h>

#define TIM_MOTOR_L &htim1
#define TIM_MOTOR_R &htim8

#define MOTOR_TIMER_TARGET_HZ 1000000UL
#define MOTOR_SPEED_RAMP_INTERVAL_MS 10U
#define MOTOR_SPEED_RAMP_STEP_SPS 50U

#define MOTOR_L_IRQ_Handler HAL_TIM1_IRQ_Handler
#define MOTOR_R_IRQ_Handler HAL_TIM8_IRQ_Handler

#define Check_Bit(num, bitMask)	((num & bitMask) ? GPIO_PIN_SET : GPIO_PIN_RESET)

typedef struct {
	GPIO_TypeDef *Port;
	uint16_t Pin;
} Motor_TypeDef;

static const Motor_TypeDef Motor_L[4] = { { .Port = MTR_L1_GPIO_Port, .Pin = MTR_L1_Pin }, {
		.Port = MTR_L3_GPIO_Port, .Pin = MTR_L3_Pin }, { .Port =
MTR_L2_GPIO_Port, .Pin = MTR_L2_Pin }, { .Port = MTR_L4_GPIO_Port, .Pin =
MTR_L4_Pin }, };

static const Motor_TypeDef Motor_R[4] = { { .Port = MTR_R1_GPIO_Port, .Pin = MTR_R1_Pin }, {
		.Port = MTR_R3_GPIO_Port, .Pin = MTR_R3_Pin }, { .Port =
MTR_R2_GPIO_Port, .Pin = MTR_R2_Pin }, { .Port = MTR_R4_GPIO_Port, .Pin =
MTR_R4_Pin }, };

static const uint8_t stepSequence[MOTOR_PHASE_COUNT] = { 0b0001, 0b0011,
		0b0010, 0b0110, 0b0100,
		0b1100, 0b1000, 0b1001 };
static bool phase_test_armed = false;
static MotorTarget_t phase_test_target = MOTOR_TARGET_LEFT;
static volatile uint8_t motor_l_phase = 0U;
static volatile uint8_t motor_r_phase = 0U;
static volatile int8_t motor_l_direction = 1;
static volatile int8_t motor_r_direction = 1;
static volatile uint32_t motor_l_step_count = 0U;
static volatile uint32_t motor_r_step_count = 0U;
static volatile bool motor_drive_running = false;
static bool motor_speed_test_running = false;
static uint16_t motor_speed_test_current_sps = 0U;
static uint16_t motor_speed_test_target_sps = 0U;
static uint32_t motor_speed_test_ramp_tick = 0U;

static void Motor_WritePattern(const Motor_TypeDef motor[4], uint8_t pattern)
{
	uint8_t index;

	for (index = 0U; index < 4U; index++) {
		HAL_GPIO_WritePin(motor[index].Port, motor[index].Pin,
				Check_Bit(pattern, (uint8_t)(1U << index)));
	}
}

static void Motor_AllOutputsOff(void)
{
	Motor_WritePattern(Motor_L, 0U);
	Motor_WritePattern(Motor_R, 0U);
}

static void Motor_ApplyTargetPattern(MotorTarget_t target, uint8_t pattern)
{
	Motor_WritePattern(Motor_L,
			(target != MOTOR_TARGET_RIGHT) ? pattern : 0U);
	Motor_WritePattern(Motor_R,
			(target != MOTOR_TARGET_LEFT) ? pattern : 0U);
}

void MOTOR_L_IRQ_Handler()
{
	if (motor_l_direction < 0) {
		motor_l_phase = (motor_l_phase == 0U)
				? (MOTOR_PHASE_COUNT - 1U) : (motor_l_phase - 1U);
	} else {
		motor_l_phase = (motor_l_phase + 1U) & 0x07U;
	}
	Motor_WritePattern(Motor_L, stepSequence[motor_l_phase]);
	motor_l_step_count++;
}

void MOTOR_R_IRQ_Handler()
{
	if (motor_r_direction < 0) {
		motor_r_phase = (motor_r_phase == 0U)
				? (MOTOR_PHASE_COUNT - 1U) : (motor_r_phase - 1U);
	} else {
		motor_r_phase = (motor_r_phase + 1U) & 0x07U;
	}
	Motor_WritePattern(Motor_R, stepSequence[motor_r_phase]);
	motor_r_step_count++;
}

static bool Motor_ConfigureStepTimer(TIM_HandleTypeDef *timer,
		uint16_t steps_per_second)
{
	uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();
	uint32_t prescaler;
	uint32_t timer_tick_hz;
	uint32_t reload;

	if ((steps_per_second < MOTOR_DRIVE_MIN_SPS)
			|| (steps_per_second > MOTOR_DRIVE_MAX_SPS)
			|| (timer_clock_hz < MOTOR_TIMER_TARGET_HZ)) {
		return false;
	}

	prescaler = (timer_clock_hz / MOTOR_TIMER_TARGET_HZ) - 1U;
	timer_tick_hz = timer_clock_hz / (prescaler + 1U);
	reload = timer_tick_hz / steps_per_second;
	if ((reload == 0U) || (reload > 65536U)) {
		return false;
	}

	__HAL_TIM_DISABLE(timer);
	__HAL_TIM_SET_PRESCALER(timer, prescaler);
	__HAL_TIM_SET_AUTORELOAD(timer, reload - 1U);
	__HAL_TIM_SET_COUNTER(timer, 0U);
	if (HAL_TIM_GenerateEvent(timer, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
		return false;
	}
	__HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
	return true;
}

static bool Motor_CalculateReload(uint16_t steps_per_second,
		uint32_t *prescaler, uint32_t *reload)
{
	uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();
	uint32_t timer_tick_hz;

	if ((prescaler == NULL) || (reload == NULL)
			|| (steps_per_second < MOTOR_DRIVE_MIN_SPS)
			|| (steps_per_second > MOTOR_DRIVE_MAX_SPS)
			|| (timer_clock_hz < MOTOR_TIMER_TARGET_HZ)) {
		return false;
	}

	*prescaler = (timer_clock_hz / MOTOR_TIMER_TARGET_HZ) - 1U;
	timer_tick_hz = timer_clock_hz / (*prescaler + 1U);
	*reload = timer_tick_hz / steps_per_second;
	return (*reload != 0U) && (*reload <= 65536U);
}

static bool Motor_UpdateStepTimers(uint16_t left_steps_per_second,
		uint16_t right_steps_per_second)
{
	uint32_t left_prescaler;
	uint32_t right_prescaler;
	uint32_t left_reload;
	uint32_t right_reload;

	if (!Motor_CalculateReload(left_steps_per_second,
			&left_prescaler, &left_reload)
			|| !Motor_CalculateReload(right_steps_per_second,
					&right_prescaler, &right_reload)
			|| (left_prescaler != right_prescaler)) {
		return false;
	}

	/* TIM1/TIM8 use ARR preload. The new period takes effect at the next
	 * update event without resetting the phase or counter. */
	__HAL_TIM_SET_PRESCALER(TIM_MOTOR_L, left_prescaler);
	__HAL_TIM_SET_PRESCALER(TIM_MOTOR_R, right_prescaler);
	__HAL_TIM_SET_AUTORELOAD(TIM_MOTOR_L, left_reload - 1U);
	__HAL_TIM_SET_AUTORELOAD(TIM_MOTOR_R, right_reload - 1U);
	return true;
}

__STATIC_INLINE void Motor_Vref_Set(uint16_t value) {
	// value 값은 최대 12-bit => 4095 ~ 0 => 3.3 ~ 0 V
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value);

}

void Motor_Start() {
	phase_test_armed = false;
	motor_l_phase = 0U;
	motor_r_phase = 0U;
	motor_l_direction = 1;
	motor_r_direction = 1;
	// Vref 전압 활성화
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	// Vref 전압 설정 (value 값 조절해야 함)
//	Motor_Vref_Set(value);

	HAL_TIM_Base_Start_IT(TIM_MOTOR_L);
	HAL_TIM_Base_Start_IT(TIM_MOTOR_R);
}

void Motor_Stop() {
	HAL_TIM_Base_Stop_IT(TIM_MOTOR_L);
	HAL_TIM_Base_Stop_IT(TIM_MOTOR_R);
	Motor_Vref_Set(0);
	Motor_AllOutputsOff();
	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
	phase_test_armed = false;
	motor_drive_running = false;
	motor_speed_test_running = false;
	motor_speed_test_current_sps = 0U;
	motor_speed_test_target_sps = 0U;
}

uint8_t Motor_GetPhasePattern(uint8_t phase)
{
	return stepSequence[phase & 0x07U];
}

bool Motor_PhaseTestArm(MotorTarget_t target, uint8_t phase,
		uint16_t vref_dac)
{
	if (target > MOTOR_TARGET_BOTH) {
		return false;
	}
	if (vref_dac > MOTOR_VREF_DAC_ABSOLUTE_MAX) {
		vref_dac = MOTOR_VREF_DAC_ABSOLUTE_MAX;
	}

	/* Stop any timer-driven output before entering the manual phase test. */
	Motor_Stop();
	if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) {
		Motor_AllOutputsOff();
		return false;
	}

	/* Establish the phase at zero current, then apply the conservative test
	 * reference. This avoids a transient with an undefined input pattern. */
	Motor_Vref_Set(0U);
	phase_test_target = target;
	Motor_ApplyTargetPattern(target, Motor_GetPhasePattern(phase));
	Motor_Vref_Set(vref_dac);
	phase_test_armed = true;
	return true;
}

void Motor_PhaseTestSetPhase(uint8_t phase)
{
	if (!phase_test_armed) {
		return;
	}
	Motor_ApplyTargetPattern(phase_test_target,
			Motor_GetPhasePattern(phase));
}

void Motor_PhaseTestDisarm(void)
{
	Motor_Stop();
}

bool Motor_SpeedTestStart(uint16_t steps_per_second, uint16_t vref_dac)
{
	if ((steps_per_second < MOTOR_SPEED_TEST_MIN_SPS)
			|| (steps_per_second > MOTOR_SPEED_TEST_MAX_SPS)) {
		return false;
	}
	if (vref_dac > MOTOR_VREF_DAC_ABSOLUTE_MAX) {
		vref_dac = MOTOR_VREF_DAC_ABSOLUTE_MAX;
	}

	Motor_Stop();
	if (!Motor_ConfigureStepTimer(TIM_MOTOR_L,
			MOTOR_SPEED_TEST_RAMP_START_SPS)
			|| !Motor_ConfigureStepTimer(TIM_MOTOR_R,
					MOTOR_SPEED_TEST_RAMP_START_SPS)) {
		return false;
	}
	if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) {
		Motor_AllOutputsOff();
		return false;
	}

	/* The assembled motors are mirrored. Phase-test results showed that
	 * forward motion requires decreasing the left phase and increasing the
	 * right phase. Establish both initial phases at zero current first. */
	motor_l_phase = 0U;
	motor_r_phase = 0U;
	motor_l_direction = -1;
	motor_r_direction = 1;
	Motor_Vref_Set(0U);
	Motor_WritePattern(Motor_L, Motor_GetPhasePattern(motor_l_phase));
	Motor_WritePattern(Motor_R, Motor_GetPhasePattern(motor_r_phase));
	Motor_Vref_Set(vref_dac);

	if ((HAL_TIM_Base_Start_IT(TIM_MOTOR_L) != HAL_OK)
			|| (HAL_TIM_Base_Start_IT(TIM_MOTOR_R) != HAL_OK)) {
		Motor_Stop();
		return false;
	}
	motor_speed_test_current_sps = MOTOR_SPEED_TEST_RAMP_START_SPS;
	motor_speed_test_target_sps = steps_per_second;
	motor_speed_test_ramp_tick = HAL_GetTick();
	motor_speed_test_running = true;
	return true;
}

bool Motor_SpeedTestProcess(void)
{
	uint32_t now;
	uint32_t intervals;
	uint32_t increment;
	uint16_t new_sps;

	if (!motor_speed_test_running) {
		return false;
	}
	if (motor_speed_test_current_sps >= motor_speed_test_target_sps) {
		return true;
	}

	now = HAL_GetTick();
	intervals = (now - motor_speed_test_ramp_tick)
			/ MOTOR_SPEED_RAMP_INTERVAL_MS;
	if (intervals == 0U) {
		return true;
	}
	motor_speed_test_ramp_tick += intervals * MOTOR_SPEED_RAMP_INTERVAL_MS;
	increment = intervals * MOTOR_SPEED_RAMP_STEP_SPS;
	if (increment >= (uint32_t)(motor_speed_test_target_sps
			- motor_speed_test_current_sps)) {
		new_sps = motor_speed_test_target_sps;
	} else {
		new_sps = motor_speed_test_current_sps + (uint16_t)increment;
	}

	if (!Motor_UpdateStepTimers(new_sps, new_sps)) {
		Motor_Stop();
		return false;
	}
	motor_speed_test_current_sps = new_sps;
	return true;
}

bool Motor_SpeedTestIsRunning(void)
{
	return motor_speed_test_running;
}

uint16_t Motor_SpeedTestGetCurrentSps(void)
{
	return motor_speed_test_current_sps;
}

void Motor_SpeedTestStop(void)
{
	Motor_Stop();
}

bool Motor_DriveStart(uint16_t left_steps_per_second,
		uint16_t right_steps_per_second, uint16_t vref_dac)
{
	if (vref_dac > MOTOR_VREF_DAC_ABSOLUTE_MAX) {
		vref_dac = MOTOR_VREF_DAC_ABSOLUTE_MAX;
	}

	Motor_Stop();
	Motor_DriveResetStepCounts();
	if (!Motor_ConfigureStepTimer(TIM_MOTOR_L, left_steps_per_second)
			|| !Motor_ConfigureStepTimer(TIM_MOTOR_R,
					right_steps_per_second)) {
		return false;
	}
	if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) {
		Motor_AllOutputsOff();
		return false;
	}

	/* Forward on the mirrored chassis is L phase decreasing, R increasing. */
	motor_l_phase = 0U;
	motor_r_phase = 0U;
	motor_l_direction = -1;
	motor_r_direction = 1;
	Motor_Vref_Set(0U);
	Motor_WritePattern(Motor_L, Motor_GetPhasePattern(motor_l_phase));
	Motor_WritePattern(Motor_R, Motor_GetPhasePattern(motor_r_phase));
	Motor_Vref_Set(vref_dac);

	if ((HAL_TIM_Base_Start_IT(TIM_MOTOR_L) != HAL_OK)
			|| (HAL_TIM_Base_Start_IT(TIM_MOTOR_R) != HAL_OK)) {
		Motor_Stop();
		return false;
	}
	motor_drive_running = true;
	return true;
}

bool Motor_DriveSetSpeeds(uint16_t left_steps_per_second,
		uint16_t right_steps_per_second)
{
	if (!motor_drive_running) {
		return false;
	}
	return Motor_UpdateStepTimers(left_steps_per_second,
			right_steps_per_second);
}

void Motor_DriveStop(void)
{
	Motor_Stop();
}

bool Motor_DriveIsRunning(void)
{
	return motor_drive_running;
}

void Motor_DriveResetStepCounts(void)
{
	__disable_irq();
	motor_l_step_count = 0U;
	motor_r_step_count = 0U;
	__enable_irq();
}

void Motor_DriveGetStepCounts(uint32_t *left_steps, uint32_t *right_steps)
{
	if ((left_steps == NULL) || (right_steps == NULL)) {
		return;
	}

	__disable_irq();
	*left_steps = motor_l_step_count;
	*right_steps = motor_r_step_count;
	__enable_irq();
}

uint32_t Motor_DriveGetAverageSteps(void)
{
	uint32_t left_steps;
	uint32_t right_steps;

	Motor_DriveGetStepCounts(&left_steps, &right_steps);
	return (left_steps + right_steps) / 2U;
}
