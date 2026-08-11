/*
 * motor.h
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PHASE_COUNT          8U
/*
 * The PCB feeds the SLA7026M REF pins through a 1 kOhm/1 kOhm divider and
 * uses 0.68 Ohm current-sense resistors.  With a 3.3 V DAC this setting is
 * approximately 0.91 A/phase (3.3 V * 1536 / 4095 / 2 / 0.68 Ohm).
 *
 * 512 DAC counts only produced about 0.30 A/phase, which was insufficient
 * for the loaded chassis and caused the motors to lose synchronism.  Keep
 * the current in one named run-time setting so the speed test and First
 * Drive cannot silently use different torque settings.
 */
#define MOTOR_VREF_DAC_RUN         1536U
#define MOTOR_VREF_DAC_BRAKE_HARD  MOTOR_VREF_DAC_RUN
#define MOTOR_VREF_DAC_BRAKE_HOLD  1024U
#define MOTOR_VREF_CURRENT_MA      910U
#define MOTOR_PHASE_TEST_DAC       MOTOR_VREF_DAC_RUN
#define MOTOR_SPEED_TEST_DAC       MOTOR_VREF_DAC_RUN
#define MOTOR_SPEED_TEST_MIN_SPS   4800U
#define MOTOR_SPEED_TEST_MAX_SPS   6000U
#define MOTOR_SPEED_TEST_RAMP_START_SPS 800U
#define MOTOR_VREF_DAC_ABSOLUTE_MAX 2480U
#define MOTOR_DRIVE_MIN_SPS         100U
#define MOTOR_DRIVE_MAX_SPS         6500U

typedef enum {
	MOTOR_TARGET_LEFT = 0,
	MOTOR_TARGET_RIGHT,
	MOTOR_TARGET_BOTH
} MotorTarget_t;

void Motor_Start(void);
void Motor_Stop(void);
bool Motor_PhaseTestArm(MotorTarget_t target, uint8_t phase,
		uint16_t vref_dac);
void Motor_PhaseTestSetPhase(uint8_t phase);
void Motor_PhaseTestDisarm(void);
uint8_t Motor_GetPhasePattern(uint8_t phase);
bool Motor_SpeedTestStart(uint16_t steps_per_second, uint16_t vref_dac);
bool Motor_SpeedTestProcess(void);
bool Motor_SpeedTestIsRunning(void);
uint16_t Motor_SpeedTestGetCurrentSps(void);
void Motor_SpeedTestStop(void);
bool Motor_DriveStart(uint16_t left_steps_per_second,
		uint16_t right_steps_per_second, uint16_t vref_dac);
bool Motor_DriveSetSpeeds(uint16_t left_steps_per_second,
		uint16_t right_steps_per_second);
bool Motor_DriveBrakeHoldStart(uint16_t initial_vref_dac);
void Motor_DriveBrakeHoldSetVref(uint16_t vref_dac);
void Motor_DriveBrakeHoldProcess(void);
void Motor_DriveBrakeHoldFinish(void);
bool Motor_DriveBrakeHoldIsActive(void);
void Motor_DriveStop(void);
bool Motor_DriveIsRunning(void);
void Motor_DriveResetStepCounts(void);
void Motor_DriveGetStepCounts(uint32_t *left_steps, uint32_t *right_steps);
uint32_t Motor_DriveGetAverageSteps(void);

#endif /* INC_MOTOR_H_ */
