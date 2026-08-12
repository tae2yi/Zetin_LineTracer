#include "main.h"

#include <stdio.h>
#include <stdlib.h>

uint32_t mock_tick;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
DAC_HandleTypeDef hdac1;
static uint32_t gpio_state_mask;
static uint32_t timer_stop_count;
static bool dac_start_fail;
static unsigned int failures;

uint32_t HAL_GetTick(void)
{
	return mock_tick;
}

uint32_t HAL_RCC_GetPCLK2Freq(void)
{
	return 72000000U;
}

HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer,
		uint32_t event)
{
	(void)timer;
	(void)event;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{
	timer->running = true;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer)
{
	timer->running = false;
	timer_stop_count++;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *dac, uint32_t channel)
{
	(void)channel;
	if (dac_start_fail) {
		return HAL_ERROR;
	}
	dac->running = true;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef *dac, uint32_t channel)
{
	(void)channel;
	dac->running = false;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *dac, uint32_t channel,
		uint32_t alignment, uint32_t value)
{
	(void)channel;
	(void)alignment;
	dac->value = (uint16_t)value;
	return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
	uint32_t bit = 1U << ((uint32_t)(uintptr_t)port - 1U);

	(void)pin;
	if (state == GPIO_PIN_SET) {
		gpio_state_mask |= bit;
	} else {
		gpio_state_mask &= ~bit;
	}
}

#include "../../Main/Src/motor.c"

static void Check(bool condition, const char *message)
{
	if (!condition) {
		printf("FAIL: %s\n", message);
		failures++;
	}
}

int main(void)
{
	uint32_t energized;

	Check(Motor_DriveStart(3000U, 3000U, MOTOR_VREF_DAC_RUN),
			"drive start");
	energized = gpio_state_mask;
	Check(energized != 0U && Motor_DriveIsRunning(),
			"motor is running with phase outputs");

	/* Brake-start failure must immediately use the full-off path. */
	dac_start_fail = true;
	Check(!Motor_DriveBrakeHoldStart(MOTOR_VREF_DAC_BRAKE_HARD),
			"brake start failure is reported");
	Check(!Motor_DriveBrakeHoldIsActive() && !hdac1.running
			&& (gpio_state_mask == 0U),
			"brake start failure is full off");
	dac_start_fail = false;

	Check(Motor_DriveStart(3000U, 3000U, MOTOR_VREF_DAC_RUN),
			"restart after failed brake");
	mock_tick = 1000U;
	Check(Motor_DriveBrakeHoldStart(MOTOR_VREF_DAC_BRAKE_HARD),
			"active brake starts");
	Check(Motor_DriveBrakeHoldStart(MOTOR_VREF_DAC_BRAKE_HARD),
			"repeated brake start is idempotent");
	Check(gpio_state_mask == energized,
			"active brake holds the last phase");
	mock_tick += 30U;
	Motor_DriveBrakeHoldProcess();
	Check(hdac1.value == MOTOR_VREF_DAC_BRAKE_HOLD,
			"hard hold transitions to reduced hold");

	/* Emergency/fault-equivalent full-off while reduced hold is active. */
	Motor_DriveStop();
	Check(!Motor_DriveBrakeHoldIsActive() && !hdac1.running
			&& (gpio_state_mask == 0U),
			"stop during reduced hold is full off");

	Check(Motor_DriveStart(3000U, 3000U, MOTOR_VREF_DAC_RUN),
			"second restart");
	mock_tick += 1U;
	Check(Motor_DriveBrakeHoldStart(MOTOR_VREF_DAC_BRAKE_HARD),
			"second active brake starts");
	Motor_DriveBrakeHoldFinish();
	Check(!Motor_DriveBrakeHoldIsActive() && !hdac1.running
			&& (gpio_state_mask == 0U),
			"emergency finish is full off");
	Check(timer_stop_count >= 8U, "brake paths stop both timers");

	if (failures != 0U) {
		printf("%u failures\n", failures);
		return EXIT_FAILURE;
	}
	puts("V31 motor brake mock PASS");
	return EXIT_SUCCESS;
}
