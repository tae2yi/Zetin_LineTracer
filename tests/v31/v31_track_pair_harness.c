#include "track.h"

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

static void FeedCandidate(uint8_t mask, uint32_t *frame,
		uint32_t start_step)
{
	uint8_t index;

	for (index = 0U; index < TRACK_MARK_CONFIRM_FRAMES; index++) {
		(*frame)++;
		(void)Track_ProcessSensor(mask, *frame,
				start_step + ((uint32_t)index * 10U));
	}
	for (index = 1U; index <= TRACK_MARK_CLEAR_FRAMES; index++) {
		(*frame)++;
		(void)Track_ProcessSensor(0U, *frame,
				start_step + 30U + ((uint32_t)index * 10U));
	}
}

static void BuildSequence(const uint8_t *masks, uint8_t count)
{
	uint32_t frame = 0U;
	uint8_t index;

	Track_Reset();
	Track_SetStartIgnoreSteps(300U);
	for (index = 0U; index < count; index++) {
		FeedCandidate(masks[index], &frame,
				500U + ((uint32_t)index * 500U));
	}
	Track_FinalizeSegments();
}

int main(void)
{
	static const uint8_t full[] = {
		0x80U, 0x80U, 0x80U, 0x01U,
		0x80U, 0x01U, 0x01U, 0x81U
	};
	static const uint8_t missing_close[] = {
		0x80U, 0x80U, 0x80U, 0x01U,
		0x80U, 0x01U, 0x81U
	};
	static const TrackSegmentType_t expected[] = {
		TRACK_SEGMENT_RIGHT, TRACK_SEGMENT_STRAIGHT,
		TRACK_SEGMENT_RIGHT, TRACK_SEGMENT_LEFT,
		TRACK_SEGMENT_RIGHT, TRACK_SEGMENT_LEFT,
		TRACK_SEGMENT_STRAIGHT
	};
	TrackMapPairDiagnostics_t diagnostics;
	uint8_t index;

	BuildSequence(full, (uint8_t)(sizeof(full) / sizeof(full[0])));
	Check(Track_GetSegmentCount() == 9U, "full sequence segment count");
	for (index = 0U; index < (uint8_t)(sizeof(expected)
			/ sizeof(expected[0])); index++) {
		const TrackSegment_t *segment = Track_GetSegment((uint16_t)index + 1U);

		Check((segment != NULL) && (segment->type == expected[index]),
				"full sequence segment semantics");
	}
	Track_GetMapPairDiagnostics(&diagnostics);
	Check(diagnostics.turn_open == 0U, "full sequence pair closed");
	Check(diagnostics.unmatched_turn_at_end == 0U,
			"full sequence no unmatched turn");

	BuildSequence(missing_close,
			(uint8_t)(sizeof(missing_close) / sizeof(missing_close[0])));
	Track_GetMapPairDiagnostics(&diagnostics);
	Check(diagnostics.turn_open == 1U, "missing close leaves pair open");
	Check(diagnostics.turn_direction == -1,
			"missing close leaves unmatched LEFT");
	Check(diagnostics.unmatched_turn_at_end == 1U,
			"missing close diagnostic set");
	Check(Track_GetSegment(Track_GetSegmentCount() - 2U)->type
			== TRACK_SEGMENT_LEFT, "missing close tail is LEFT before END");

	if (failures != 0U) {
		printf("%u failures\n", failures);
		return EXIT_FAILURE;
	}
	puts("V31 track pair harness PASS");
	return EXIT_SUCCESS;
}
