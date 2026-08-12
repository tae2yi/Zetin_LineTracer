#include "drive.h"

#include <stdio.h>
#include <stdlib.h>

static unsigned int failures;

static void Check(bool condition, const char *message)
{
	if (!condition) {
		printf("FAIL: %s\n", message);
		failures++;
	}
}

int main(void)
{
	uint16_t curve;

	Check(FIRST_DRIVE_CURVE_CRUISE_DEFAULT_SPS == 2200U,
			"curve default is V30 2200 SPS");
	Check(FIRST_DRIVE_CURVE_CRUISE_MIN_SPS == 2200U
			&& FIRST_DRIVE_CURVE_CRUISE_MAX_SPS == 2600U,
			"curve range is 2200..2600 SPS");
	Check(FIRST_DRIVE_CURVE_CRUISE_STEP_SPS == 100U,
			"curve step is 100 SPS");
	for (curve = FIRST_DRIVE_CURVE_CRUISE_MIN_SPS;
			curve <= FIRST_DRIVE_CURVE_CRUISE_MAX_SPS;
			curve += FIRST_DRIVE_CURVE_CRUISE_STEP_SPS) {
		uint16_t floor_sps = (curve > FIRST_DRIVE_CURVE_FLOOR_DELTA_SPS)
				? (uint16_t)(curve - FIRST_DRIVE_CURVE_FLOOR_DELTA_SPS) : 0U;

		if (floor_sps < FIRST_DRIVE_CURVE_FLOOR_MIN_SPS) {
			floor_sps = FIRST_DRIVE_CURVE_FLOOR_MIN_SPS;
		}
		Check((curve == 2200U && floor_sps == 1800U)
				|| (curve == 2300U && floor_sps == 1900U)
				|| (curve == 2400U && floor_sps == 2000U)
				|| (curve == 2500U && floor_sps == 2100U)
				|| (curve == 2600U && floor_sps == 2200U),
				"curve floor contract");
	}
	Check(FIRST_DRIVE_CURVE_FLOOR_MIN_SPS == 1800U,
			"curve floor minimum remains 1800 SPS");
	if (failures != 0U) {
		printf("%u failures\n", failures);
		return EXIT_FAILURE;
	}
	puts("V31 curve contract harness PASS");
	return EXIT_SUCCESS;
}
