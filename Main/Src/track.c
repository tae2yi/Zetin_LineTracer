/*
 * track.c
 *
 * First Drive marker collector.  It records a debounced event and leaves the
 * action policy to drive.c, where the line follower can temporarily select a
 * turn, cross-pass, or end-runout state.
 */

#include "track.h"

#include <stddef.h>
#include <string.h>

typedef struct {
	bool collecting;
	uint8_t edge_union;
	uint8_t full_union;
	uint8_t max_center_count;
	uint16_t edge0_run;
	uint16_t edge7_run;
	uint16_t both_overlap_run;
	uint16_t wide_center_run;
	uint16_t max_edge0_run;
	uint16_t max_edge7_run;
	uint16_t max_both_overlap_run;
	uint16_t max_wide_center_run;
	uint16_t quiet_frames;
	uint16_t confirm_frames;
	uint32_t entry_frame;
	uint32_t last_active_frame;
	uint32_t entry_step;
	uint32_t last_active_step;
} TrackPendingEvent_t;

static TrackMarkerEvent_t events[TRACK_MAX_EVENTS];
static TrackSegment_t segments[TRACK_MAX_SEGMENTS];
static TrackMarkerEvent_t last_event;
static TrackPendingEvent_t pending;
static TrackMarkerEvent_t replay_last_event;
static TrackPendingEvent_t replay_pending;
static uint16_t event_count;
static uint16_t segment_count;
static uint32_t cooldown_until_step;
static uint32_t replay_cooldown_until_step;
static uint32_t start_ignore_steps;
static bool overflow;

static uint8_t Track_CountCenter(uint8_t mask)
{
	uint8_t count = 0U;
	uint8_t index;

	for (index = 1U; index <= 6U; index++) {
		if ((mask & (uint8_t)(1U << index)) != 0U) {
			count++;
		}
	}
	return count;
}

static bool Track_IsActive(uint8_t mask, uint8_t center_count)
{
	return ((mask & MARKER_EDGE_MASK) != 0U)
			|| (center_count >= MARKER_WIDE_CENTER_COUNT);
}

static void Track_StartPending(uint32_t frame_number, uint32_t step)
{
	memset(&pending, 0, sizeof(pending));
	pending.collecting = true;
	pending.entry_frame = frame_number;
	pending.last_active_frame = frame_number;
	pending.entry_step = step;
	pending.last_active_step = step;
}

static MarkerEventType_t Track_ClassifyPending(
		const TrackPendingEvent_t *pending_event)
{
	/* A transverse crossing can illuminate S0 and S7 together as well as the
	 * inner sensors.  Sustained wide-centre evidence therefore takes priority
	 * over bilateral edge evidence. */
	if (pending_event->max_wide_center_run >= MARKER_WIDE_MIN_FRAMES) {
		return MARKER_EVENT_CROSS;
	}
	if (pending_event->max_both_overlap_run >= TRACK_MARKER_MIN_OVERLAP) {
		return MARKER_EVENT_BOTH;
	}
	if (pending_event->edge_union == 0x01U) {
		return MARKER_EVENT_EDGE_0;
	}
	if (pending_event->edge_union == 0x80U) {
		return MARKER_EVENT_EDGE_7;
	}
	return MARKER_EVENT_UNKNOWN;
}

static uint8_t Track_CalculateConfidence(
		const TrackPendingEvent_t *pending_event)
{
	uint8_t confidence = 0U;

	if (pending_event->max_center_count >= MARKER_WIDE_CENTER_COUNT) {
		confidence += 40U;
	}
	if (pending_event->max_edge0_run >= TRACK_MARK_CONFIRM_FRAMES) {
		confidence += 20U;
	}
	if (pending_event->max_edge7_run >= TRACK_MARK_CONFIRM_FRAMES) {
		confidence += 20U;
	}
	if (pending_event->max_both_overlap_run >= TRACK_MARKER_MIN_OVERLAP) {
		confidence += 20U;
	}
	return confidence;
}

static bool Track_FinishPending(TrackPendingEvent_t *pending_event,
		TrackMarkerEvent_t *completed_event, uint32_t *cooldown_step,
		bool store_event)
{
	TrackMarkerEvent_t event;

	if ((pending_event == NULL) || (completed_event == NULL)
			|| (cooldown_step == NULL) || !pending_event->collecting) {
		return false;
	}
	if (pending_event->confirm_frames < TRACK_MARK_CONFIRM_FRAMES) {
		memset(pending_event, 0, sizeof(*pending_event));
		return false;
	}

	memset(&event, 0, sizeof(event));
	event.type = Track_ClassifyPending(pending_event);
	event.edge_union = pending_event->edge_union;
	event.full_union = pending_event->full_union;
	event.max_center_count = pending_event->max_center_count;
	event.edge0_run = pending_event->max_edge0_run;
	event.edge7_run = pending_event->max_edge7_run;
	event.both_overlap_run = pending_event->max_both_overlap_run;
	event.wide_center_run = pending_event->max_wide_center_run;
	event.entry_frame = pending_event->entry_frame;
	event.exit_frame = pending_event->last_active_frame;
	event.entry_step = pending_event->entry_step;
	event.exit_step = pending_event->last_active_step;
	event.center_step = pending_event->entry_step
			+ ((pending_event->last_active_step
					- pending_event->entry_step) / 2U);
	event.confidence = Track_CalculateConfidence(pending_event);

	*completed_event = event;
	if (store_event) {
		if (event_count < TRACK_MAX_EVENTS) {
			events[event_count++] = event;
		} else {
			overflow = true;
		}
	}

	*cooldown_step = event.exit_step + TRACK_MARK_COOLDOWN_STEPS;
	memset(pending_event, 0, sizeof(*pending_event));
	return true;
}

void Track_Reset(void)
{
	memset(events, 0, sizeof(events));
	memset(segments, 0, sizeof(segments));
	memset(&last_event, 0, sizeof(last_event));
	memset(&pending, 0, sizeof(pending));
	Track_ReplayReset();
	event_count = 0U;
	segment_count = 0U;
	cooldown_until_step = 0U;
	start_ignore_steps = 0U;
	overflow = false;
}

void Track_SetStartIgnoreSteps(uint32_t minimum_step)
{
	start_ignore_steps = minimum_step;
}

bool Track_ProcessSensor(uint8_t sensor_mask, uint32_t frame_number,
		uint32_t average_step)
{
	uint8_t center_count = Track_CountCenter(sensor_mask);
	uint8_t edges = sensor_mask & MARKER_EDGE_MASK;
	bool active = Track_IsActive(sensor_mask, center_count);
	bool event_finished = false;

	if (!pending.collecting) {
		if (!active || (average_step < cooldown_until_step)) {
			return false;
		}
		Track_StartPending(frame_number, average_step);
	}

	pending.full_union |= sensor_mask;
	pending.edge_union |= edges;
	if (center_count > pending.max_center_count) {
		pending.max_center_count = center_count;
	}

	if ((edges & 0x01U) != 0U) {
		if (pending.edge0_run < UINT16_MAX) {
			pending.edge0_run++;
		}
	} else {
		pending.edge0_run = 0U;
	}
	if (pending.edge0_run > pending.max_edge0_run) {
		pending.max_edge0_run = pending.edge0_run;
	}
	if ((edges & 0x80U) != 0U) {
		if (pending.edge7_run < UINT16_MAX) {
			pending.edge7_run++;
		}
	} else {
		pending.edge7_run = 0U;
	}
	if (pending.edge7_run > pending.max_edge7_run) {
		pending.max_edge7_run = pending.edge7_run;
	}
	if (edges == MARKER_EDGE_MASK) {
		if (pending.both_overlap_run < UINT16_MAX) {
			pending.both_overlap_run++;
		}
	} else {
		pending.both_overlap_run = 0U;
	}
	if (pending.both_overlap_run > pending.max_both_overlap_run) {
		pending.max_both_overlap_run = pending.both_overlap_run;
	}
	if (center_count >= MARKER_WIDE_CENTER_COUNT) {
		if (pending.wide_center_run < UINT16_MAX) {
			pending.wide_center_run++;
		}
		if (pending.wide_center_run > pending.max_wide_center_run) {
			pending.max_wide_center_run = pending.wide_center_run;
		}
	} else {
		pending.wide_center_run = 0U;
	}

	if (active) {
		pending.last_active_frame = frame_number;
		pending.last_active_step = average_step;
		pending.quiet_frames = 0U;
		if (pending.confirm_frames < UINT16_MAX) {
			pending.confirm_frames++;
		}
	} else if (pending.quiet_frames < UINT16_MAX) {
		pending.quiet_frames++;
	}

	if (pending.quiet_frames >= TRACK_MARK_CLEAR_FRAMES) {
		event_finished = Track_FinishPending(&pending, &last_event,
				&cooldown_until_step, true);
	}
	return event_finished;
}

static bool Track_ProcessReplayInternal(uint8_t sensor_mask,
		uint32_t frame_number, uint32_t average_step)
{
	uint8_t center_count = Track_CountCenter(sensor_mask);
	uint8_t edges = sensor_mask & MARKER_EDGE_MASK;
	bool active = Track_IsActive(sensor_mask, center_count);
	bool event_finished = false;

	if (!replay_pending.collecting) {
		if (!active || (average_step < replay_cooldown_until_step)) {
			return false;
		}
		memset(&replay_pending, 0, sizeof(replay_pending));
		replay_pending.collecting = true;
		replay_pending.entry_frame = frame_number;
		replay_pending.last_active_frame = frame_number;
		replay_pending.entry_step = average_step;
		replay_pending.last_active_step = average_step;
	}

	replay_pending.full_union |= sensor_mask;
	replay_pending.edge_union |= edges;
	if (center_count > replay_pending.max_center_count) {
		replay_pending.max_center_count = center_count;
	}

	if ((edges & 0x01U) != 0U) {
		if (replay_pending.edge0_run < UINT16_MAX) {
			replay_pending.edge0_run++;
		}
	} else {
		replay_pending.edge0_run = 0U;
	}
	if (replay_pending.edge0_run > replay_pending.max_edge0_run) {
		replay_pending.max_edge0_run = replay_pending.edge0_run;
	}
	if ((edges & 0x80U) != 0U) {
		if (replay_pending.edge7_run < UINT16_MAX) {
			replay_pending.edge7_run++;
		}
	} else {
		replay_pending.edge7_run = 0U;
	}
	if (replay_pending.edge7_run > replay_pending.max_edge7_run) {
		replay_pending.max_edge7_run = replay_pending.edge7_run;
	}
	if (edges == MARKER_EDGE_MASK) {
		if (replay_pending.both_overlap_run < UINT16_MAX) {
			replay_pending.both_overlap_run++;
		}
	} else {
		replay_pending.both_overlap_run = 0U;
	}
	if (replay_pending.both_overlap_run
			> replay_pending.max_both_overlap_run) {
		replay_pending.max_both_overlap_run = replay_pending.both_overlap_run;
	}
	if (center_count >= MARKER_WIDE_CENTER_COUNT) {
		if (replay_pending.wide_center_run < UINT16_MAX) {
			replay_pending.wide_center_run++;
		}
		if (replay_pending.wide_center_run
				> replay_pending.max_wide_center_run) {
			replay_pending.max_wide_center_run = replay_pending.wide_center_run;
		}
	} else {
		replay_pending.wide_center_run = 0U;
	}

	if (active) {
		replay_pending.last_active_frame = frame_number;
		replay_pending.last_active_step = average_step;
		replay_pending.quiet_frames = 0U;
		if (replay_pending.confirm_frames < UINT16_MAX) {
			replay_pending.confirm_frames++;
		}
	} else if (replay_pending.quiet_frames < UINT16_MAX) {
		replay_pending.quiet_frames++;
	}

	if (replay_pending.quiet_frames >= TRACK_MARK_CLEAR_FRAMES) {
		event_finished = Track_FinishPending(&replay_pending,
				&replay_last_event, &replay_cooldown_until_step, false);
	}
	return event_finished;
}

bool Track_Flush(void)
{
	return Track_FinishPending(&pending, &last_event,
			&cooldown_until_step, true);
}

const TrackMarkerEvent_t *Track_GetLastEvent(void)
{
	return &last_event;
}

void Track_ReplayReset(void)
{
	memset(&replay_last_event, 0, sizeof(replay_last_event));
	memset(&replay_pending, 0, sizeof(replay_pending));
	replay_cooldown_until_step = 0U;
}

bool Track_ProcessReplaySensor(uint8_t sensor_mask, uint32_t frame_number,
		uint32_t average_step)
{
	return Track_ProcessReplayInternal(sensor_mask, frame_number, average_step);
}

const TrackMarkerEvent_t *Track_GetLastReplayEvent(void)
{
	return &replay_last_event;
}

uint16_t Track_GetEventCount(void)
{
	return event_count;
}

uint16_t Track_GetSegmentCount(void)
{
	return segment_count;
}

const TrackMarkerEvent_t *Track_GetEvent(uint16_t index)
{
	return (index < event_count) ? &events[index] : NULL;
}

const TrackSegment_t *Track_GetSegment(uint16_t index)
{
	return (index < segment_count) ? &segments[index] : NULL;
}

bool Track_HasOverflow(void)
{
	return overflow;
}

static TrackSegmentType_t Track_SegmentTypeFromEvent(
		MarkerEventType_t type, bool *turn_open, int8_t *turn_direction)
{
	int8_t direction;

	switch (type) {
	case MARKER_EVENT_EDGE_0:
	case MARKER_EVENT_EDGE_7:
		/* A direction mark opens a curve.  The next mark on the same side
		 * closes that curve and starts the following straight section. */
		direction = (type == MARKER_EVENT_EDGE_0) ? -1 : 1;
		if (*turn_open && (*turn_direction == direction)) {
			*turn_open = false;
			*turn_direction = 0;
			return TRACK_SEGMENT_STRAIGHT;
		}
		*turn_open = true;
		*turn_direction = direction;
		return (direction < 0) ? TRACK_SEGMENT_LEFT : TRACK_SEGMENT_RIGHT;
	case MARKER_EVENT_CROSS:
		*turn_open = false;
		*turn_direction = 0;
		return TRACK_SEGMENT_CROSS;
	case MARKER_EVENT_BOTH:
		return TRACK_SEGMENT_END;
	default:
		*turn_open = false;
		*turn_direction = 0;
		return TRACK_SEGMENT_STRAIGHT;
	}
}

void Track_FinalizeSegments(void)
{
	uint16_t index;
	uint16_t first_index = 0U;
	bool turn_open = false;
	int8_t turn_direction = 0;

	segment_count = 0U;
	if (event_count == 0U) {
		return;
	}
	while ((first_index < event_count)
			&& (events[first_index].type == MARKER_EVENT_BOTH)
			&& (events[first_index].center_step < start_ignore_steps)) {
		first_index++;
	}
	if (first_index >= event_count) {
		return;
	}

	if ((events[first_index].center_step > 0U)
			&& (segment_count < TRACK_MAX_SEGMENTS)) {
		segments[segment_count].type = TRACK_SEGMENT_STRAIGHT;
		segments[segment_count].distance_steps = events[first_index].center_step;
		segment_count++;
	}

	for (index = first_index; index < event_count; index++) {
		TrackSegmentType_t type = Track_SegmentTypeFromEvent(events[index].type,
				&turn_open, &turn_direction);
		uint32_t distance = 0U;

		if (type == TRACK_SEGMENT_END) {
			if (segment_count < TRACK_MAX_SEGMENTS) {
				segments[segment_count].type = TRACK_SEGMENT_END;
				segments[segment_count].distance_steps = 0U;
				segment_count++;
			}
			break;
		}
		if ((index + 1U) >= event_count) {
			break;
		}
		distance = events[index + 1U].center_step
				- events[index].center_step;
		if (segment_count >= TRACK_MAX_SEGMENTS) {
			overflow = true;
			break;
		}
		segments[segment_count].type = type;
		segments[segment_count].distance_steps = distance;
		segment_count++;
	}
}
