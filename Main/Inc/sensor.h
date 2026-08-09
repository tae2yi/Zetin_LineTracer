/*
 * sensor.h
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_COUNT    8U
#define SENSOR_EMITTER_PAIR_COUNT 4U
#define SENSOR_ADC_OVERSAMPLED_MAX 16380U
#define SENSOR_NORMALIZED_MAX 1000U
/* Native 4x-oversampled ADC counts.  The sensor-board analogue gain now
 * provides a large enough black/white span that very small ranges should be
 * rejected instead of being digitally amplified into noise. */
#define SENSOR_CALIBRATION_MIN_RANGE 800U
#define SENSOR_TRACK_WHITE_ON        500U
#define SENSOR_TRACK_WHITE_OFF       300U
#define SENSOR_MARKER_WHITE_MIN      600U
#define SENSOR_STATE_WHITE_ON  SENSOR_TRACK_WHITE_ON
#define SENSOR_STATE_WHITE_OFF SENSOR_TRACK_WHITE_OFF
#define SENSOR_LINE_MASK          0x7EU
#define SENSOR_MARKER_MASK        0x81U
#define SENSOR_LINE_POSITION_MIN (-2500)
#define SENSOR_LINE_POSITION_MAX  2500
#define SENSOR_FRAME_BUFFER_COUNT 64U
#define SENSOR_LINE_SENSOR_FIRST  1U
#define SENSOR_LINE_SENSOR_LAST   6U
#define SENSOR_LINE_POSITION_STEP 1000
#define SENSOR_LINE_POSITION_FLOOR 200U
#define SENSOR_LINE_STRENGTH_MIN  200U
#define SENSOR_LINE_MAX_CLUSTERS    3U

typedef enum {
	SENSOR_LIGHT_PAIR = 0,
	SENSOR_LIGHT_OFF,
	SENSOR_LIGHT_FIXED
} SensorLightMode_t;

typedef struct {
	int16_t position;
	uint32_t strength;
	uint8_t mask;
	uint8_t first_sensor;
	uint8_t last_sensor;
} SensorLineCluster_t;

typedef struct {
	int16_t position;
	uint32_t strength;
	uint8_t state_mask;
	uint8_t marker_mask;
	uint8_t raw_line_mask;
	uint8_t selected_mask;
	uint8_t spill_mask;
	uint8_t center_count;
	uint8_t edge_mask;
	uint8_t cluster_count;
	bool edge_only;
	bool line_valid;
	uint16_t line_strength[SENSOR_COUNT];
	SensorLineCluster_t clusters[SENSOR_LINE_MAX_CLUSTERS];
} SensorLineMeasurement_t;

void Sensor_Start(void);
void Sensor_Stop(void);
bool Sensor_IsRunning(void);
void Sensor_SetLightMode(SensorLightMode_t mode, uint8_t parameter);
void Sensor_GetRaw(uint16_t values[SENSOR_COUNT]);
uint32_t Sensor_GetSampleCount(void);
uint32_t Sensor_GetFrameCount(void);
bool Sensor_GetFrameRawAfter(uint32_t *cursor,
		uint16_t values[SENSOR_COUNT]);
bool Sensor_GetFrameFilteredAfter(uint32_t *cursor,
		uint16_t values[SENSOR_COUNT]);
uint8_t Sensor_GetLightPinStatus(uint8_t index);

void Sensor_CalibrationReset(void);
void Sensor_CalibrationCaptureBlack(
		const uint16_t raw[SENSOR_COUNT]);
bool Sensor_CalibrationFinishBlack(void);
void Sensor_CalibrationCaptureWhite(
		const uint16_t raw[SENSOR_COUNT]);
uint8_t Sensor_CalibrationFinish(void);
uint8_t Sensor_GetCalibrationCoverageMask(void);
bool Sensor_IsCalibrationComplete(void);
uint8_t Sensor_GetCalibrationValidMask(void);
void Sensor_GetCalibration(uint16_t low[SENSOR_COUNT],
		uint16_t high[SENSOR_COUNT]);
bool Sensor_GetRawThresholds(uint8_t index, uint16_t *black_max,
		uint16_t *white_min);
uint16_t Sensor_NormalizeRaw(uint8_t index, uint16_t raw);
void Sensor_NormalizeFrame(const uint16_t raw[SENSOR_COUNT],
		uint16_t normalized[SENSOR_COUNT]);
void Sensor_GetNormalized(uint16_t values[SENSOR_COUNT]);
uint8_t Sensor_ApplyStateHysteresis(
		const uint16_t normalized[SENSOR_COUNT], uint8_t previous_mask);
void Sensor_StateReset(void);
uint8_t Sensor_UpdateState(int16_t *line_position);
bool Sensor_GetLineMeasurement(SensorLineMeasurement_t *measurement);
bool Sensor_UseLineMask(SensorLineMeasurement_t *measurement,
		uint8_t line_mask, uint8_t spill_mask);
bool Sensor_UseLineCluster(SensorLineMeasurement_t *measurement,
		uint8_t cluster_index);

void Sensor_Test_Raw(void);
void Sensor_Test_Normalized(void);
void Sensor_Test_State(void);



#endif /* INC_SENSOR_H_ */
