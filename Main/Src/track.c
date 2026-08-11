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
	uint8_t entry_mask;
	uint8_t exit_mask;
	uint16_t quiet_frames;
	uint16_t confirm_frames;
	uint32_t entry_frame;
	uint32_t last_active_frame;
	uint32_t entry_step;
	uint32_t last_active_step;
} TrackPendingEvent_t;

typedef struct {
	TrackPendingEvent_t pending;
	TrackMarkerEvent_t last_event;
	uint32_t cooldown_until_step;
	bool cross_tail_pending;
	uint32_t cross_tail_source_exit_step;
	bool pending_started_in_cross_tail;
	uint16_t cross_tail_suppressed_count;
	uint16_t cross_tail_affected_count;
	uint32_t last_cross_tail_gap_steps;
	uint32_t max_cross_tail_gap_steps;
	uint8_t last_cross_tail_edge_union;
	bool cross_tail_current_affected;
} TrackCollectorRuntime_t;

static TrackMarkerEvent_t events[TRACK_MAX_EVENTS];
static TrackSegment_t segments[TRACK_MAX_SEGMENTS];
static TrackCrossAnchor_t cross_anchors[TRACK_MAX_CROSS_ANCHORS];
static TrackCollectorRuntime_t first_collector;
static TrackCollectorRuntime_t replay_collector;
static uint16_t event_count;
static uint16_t segment_count;
static uint16_t cross_anchor_count;
static uint32_t start_ignore_steps;
static bool overflow;
static bool cross_anchor_overflow;
static TrackMapPairDiagnostics_t map_pair_diagnostics;

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

static void Track_ClearPending(TrackCollectorRuntime_t *runtime)
{
	if (runtime == NULL) {
		return;
	}
	memset(&runtime->pending, 0, sizeof(runtime->pending));
	runtime->pending_started_in_cross_tail = false;
}

static void Track_StartPending(TrackCollectorRuntime_t *runtime,
		uint32_t frame_number, uint32_t step)
{
	if (runtime == NULL) {
		return;
	}
	Track_ClearPending(runtime);
	runtime->pending.collecting = true;
	runtime->pending.entry_frame = frame_number;
	runtime->pending.last_active_frame = frame_number;
	runtime->pending.entry_step = step;
	runtime->pending.last_active_step = step;
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

static uint8_t Track_CalculateEventConfidence(
		const TrackMarkerEvent_t *event)
{
	uint8_t confidence = 0U;

	if ((event != NULL)
			&& (event->max_center_count >= MARKER_WIDE_CENTER_COUNT)) {
		confidence += 40U;
	}
	if ((event != NULL)
			&& (event->edge0_run >= TRACK_MARK_CONFIRM_FRAMES)) {
		confidence += 20U;
	}
	if ((event != NULL)
			&& (event->edge7_run >= TRACK_MARK_CONFIRM_FRAMES)) {
		confidence += 20U;
	}
	if ((event != NULL)
			&& (event->both_overlap_run >= TRACK_MARKER_MIN_OVERLAP)) {
		confidence += 20U;
	}
	return confidence;
}

static bool Track_BuildCompletedEvent(TrackCollectorRuntime_t *runtime,
		TrackMarkerEvent_t *completed_event)
{
	TrackMarkerEvent_t event;
	TrackPendingEvent_t *pending_event;
	uint32_t active_span = 0U;

	if ((runtime == NULL) || (completed_event == NULL)) {
		return false;
	}
	pending_event = &runtime->pending;
	if (!pending_event->collecting) {
		return false;
	}
	if (pending_event->confirm_frames < TRACK_MARK_CONFIRM_FRAMES) {
		Track_ClearPending(runtime);
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
	event.entry_mask = pending_event->entry_mask;
	event.exit_mask = pending_event->exit_mask;
	event.entry_frame = pending_event->entry_frame;
	event.exit_frame = pending_event->last_active_frame;
	event.entry_step = pending_event->entry_step;
	event.exit_step = pending_event->last_active_step;
	if (pending_event->last_active_step >= pending_event->entry_step) {
		active_span = pending_event->last_active_step
				- pending_event->entry_step;
	}
	event.center_step = pending_event->entry_step + (active_span / 2U);
	event.confidence = Track_CalculateEventConfidence(&event);

	*completed_event = event;
	/* Keep pending_started_in_cross_tail alive until the candidate is
	 * correlated.  Its value describes the candidate entry, not the cleared
	 * pending buffer. */
	memset(&runtime->pending, 0, sizeof(runtime->pending));
	return true;
}

static bool Track_StoreEvent(const TrackMarkerEvent_t *event)
{
	if (event == NULL) {
		return false;
	}
	if (event_count >= TRACK_MAX_EVENTS) {
		overflow = true;
		return false;
	}
	events[event_count++] = *event;
	return true;
}

static bool Track_UpdateLastStoredCross(const TrackMarkerEvent_t *event)
{
	TrackMarkerEvent_t *stored_event;

	if ((event == NULL) || (event_count == 0U)) {
		return false;
	}
	stored_event = &events[event_count - 1U];
	if ((stored_event->type != MARKER_EVENT_CROSS)
			|| (stored_event->center_step != event->center_step)) {
		return false;
	}
	*stored_event = *event;
	return true;
}

static uint16_t Track_MaxU16(uint16_t first, uint16_t second)
{
	return (first > second) ? first : second;
}

static void Track_MergeCrossTail(TrackMarkerEvent_t *cross,
		const TrackMarkerEvent_t *tail)
{
	if ((cross == NULL) || (tail == NULL)) {
		return;
	}
	cross->edge_union |= tail->edge_union;
	cross->full_union |= tail->full_union;
	if (tail->max_center_count > cross->max_center_count) {
		cross->max_center_count = tail->max_center_count;
	}
	cross->edge0_run = Track_MaxU16(cross->edge0_run, tail->edge0_run);
	cross->edge7_run = Track_MaxU16(cross->edge7_run, tail->edge7_run);
	cross->both_overlap_run = Track_MaxU16(cross->both_overlap_run,
			tail->both_overlap_run);
	cross->wide_center_run = Track_MaxU16(cross->wide_center_run,
			tail->wide_center_run);
	if (tail->exit_frame > cross->exit_frame) {
		cross->exit_frame = tail->exit_frame;
		cross->exit_mask = tail->exit_mask;
	}
	if (tail->exit_step > cross->exit_step) {
		cross->exit_step = tail->exit_step;
	}
	cross->confidence = Track_CalculateEventConfidence(cross);
}

static bool Track_IsCrossTailCandidate(
		const TrackCollectorRuntime_t *runtime,
		const TrackMarkerEvent_t *candidate, uint32_t *gap_steps)
{
	uint32_t gap;

	if ((runtime == NULL) || (candidate == NULL)
			|| !runtime->cross_tail_pending
			|| (runtime->last_event.type != MARKER_EVENT_CROSS)
			|| !runtime->pending_started_in_cross_tail
			|| (candidate->entry_step < runtime->cross_tail_source_exit_step)) {
		return false;
	}
	gap = candidate->entry_step - runtime->cross_tail_source_exit_step;
	if (gap > TRACK_CROSS_TAIL_MAX_GAP_STEPS) {
		return false;
	}
	if ((candidate->max_center_count >= MARKER_WIDE_CENTER_COUNT)
			|| (candidate->wide_center_run >= MARKER_WIDE_MIN_FRAMES)
			|| ((candidate->edge_union & MARKER_EDGE_MASK) == 0U)) {
		return false;
	}
	if (gap_steps != NULL) {
		*gap_steps = gap;
	}
	return true;
}

static void Track_ExpireCrossTailIfNeeded(TrackCollectorRuntime_t *runtime,
		uint32_t average_step)
{
	if ((runtime == NULL) || !runtime->cross_tail_pending) {
		return;
	}
	/* A candidate which started inside the window owns its source context
	 * until completion, even if its clear frames finish beyond 160 steps. */
	if (runtime->pending.collecting
			&& runtime->pending_started_in_cross_tail) {
		return;
	}
	if ((average_step >= runtime->cross_tail_source_exit_step)
			&& ((average_step - runtime->cross_tail_source_exit_step)
					> TRACK_CROSS_TAIL_MAX_GAP_STEPS)) {
		runtime->cross_tail_pending = false;
		runtime->cross_tail_current_affected = false;
	}
}

static TrackProcessResult_t Track_CompleteCandidate(
		TrackCollectorRuntime_t *runtime, TrackMarkerEvent_t *candidate,
		bool store_event)
{
	uint32_t gap_steps;

	if ((runtime == NULL) || (candidate == NULL)) {
		return TRACK_PROCESS_NONE;
	}
	if (Track_IsCrossTailCandidate(runtime, candidate, &gap_steps)) {
		TrackMarkerEvent_t merged = runtime->last_event;

		Track_MergeCrossTail(&merged, candidate);
		if (store_event && !Track_UpdateLastStoredCross(&merged)) {
			/* A runtime/map invariant was broken.  Do not publish an event that
			 * could turn a collector defect into a false END marker. */
			overflow = true;
			runtime->cross_tail_pending = false;
			runtime->pending_started_in_cross_tail = false;
			return TRACK_PROCESS_NONE;
		}
		runtime->last_event = merged;
		runtime->last_cross_tail_gap_steps = gap_steps;
		runtime->last_cross_tail_edge_union = candidate->edge_union;
		if (gap_steps > runtime->max_cross_tail_gap_steps) {
			runtime->max_cross_tail_gap_steps = gap_steps;
		}
		if (runtime->cross_tail_suppressed_count < UINT16_MAX) {
			runtime->cross_tail_suppressed_count++;
		}
		if (!runtime->cross_tail_current_affected) {
			runtime->cross_tail_current_affected = true;
			if (runtime->cross_tail_affected_count < UINT16_MAX) {
				runtime->cross_tail_affected_count++;
			}
		}
		runtime->cooldown_until_step = candidate->exit_step
				+ TRACK_MARK_COOLDOWN_STEPS;
		if ((runtime->last_event.edge_union & MARKER_EDGE_MASK)
				== MARKER_EDGE_MASK) {
			runtime->cross_tail_pending = false;
			runtime->cross_tail_current_affected = false;
		}
		runtime->pending_started_in_cross_tail = false;
		return TRACK_PROCESS_CROSS_TAIL_MERGED;
	}

	runtime->cross_tail_pending = false;
	runtime->cross_tail_current_affected = false;
	runtime->last_event = *candidate;
	runtime->pending_started_in_cross_tail = false;
	if (store_event) {
		(void)Track_StoreEvent(candidate);
	}
	runtime->cooldown_until_step = candidate->exit_step
			+ TRACK_MARK_COOLDOWN_STEPS;
	if ((candidate->type == MARKER_EVENT_CROSS)
			&& ((candidate->edge_union & MARKER_EDGE_MASK)
					!= MARKER_EDGE_MASK)) {
		runtime->cross_tail_pending = true;
		runtime->cross_tail_source_exit_step = candidate->exit_step;
	}
	return TRACK_PROCESS_EVENT_READY;
}

static TrackProcessResult_t Track_ProcessInternal(
		TrackCollectorRuntime_t *runtime, uint8_t sensor_mask,
		uint32_t frame_number, uint32_t average_step, bool store_event)
{
	TrackPendingEvent_t *pending_event;
	TrackMarkerEvent_t candidate;
	uint8_t center_count;
	uint8_t edges;
	bool active;
	bool tail_start;

	if (runtime == NULL) {
		return TRACK_PROCESS_NONE;
	}
	Track_ExpireCrossTailIfNeeded(runtime, average_step);
	pending_event = &runtime->pending;
	center_count = Track_CountCenter(sensor_mask);
	edges = sensor_mask & MARKER_EDGE_MASK;
	active = Track_IsActive(sensor_mask, center_count);

	if (!pending_event->collecting) {
		if (!active || (average_step < runtime->cooldown_until_step)) {
			return TRACK_PROCESS_NONE;
		}
		tail_start = runtime->cross_tail_pending
				&& (average_step >= runtime->cross_tail_source_exit_step)
				&& ((average_step - runtime->cross_tail_source_exit_step)
						<= TRACK_CROSS_TAIL_MAX_GAP_STEPS);
		Track_StartPending(runtime, frame_number, average_step);
		runtime->pending_started_in_cross_tail = tail_start;
		runtime->pending.entry_mask = sensor_mask;
		runtime->pending.exit_mask = sensor_mask;
	}

	pending_event->full_union |= sensor_mask;
	pending_event->edge_union |= edges;
	if (center_count > pending_event->max_center_count) {
		pending_event->max_center_count = center_count;
	}

	if ((edges & 0x01U) != 0U) {
		if (pending_event->edge0_run < UINT16_MAX) {
			pending_event->edge0_run++;
		}
	} else {
		pending_event->edge0_run = 0U;
	}
	if (pending_event->edge0_run > pending_event->max_edge0_run) {
		pending_event->max_edge0_run = pending_event->edge0_run;
	}
	if ((edges & 0x80U) != 0U) {
		if (pending_event->edge7_run < UINT16_MAX) {
			pending_event->edge7_run++;
		}
	} else {
		pending_event->edge7_run = 0U;
	}
	if (pending_event->edge7_run > pending_event->max_edge7_run) {
		pending_event->max_edge7_run = pending_event->edge7_run;
	}
	if (edges == MARKER_EDGE_MASK) {
		if (pending_event->both_overlap_run < UINT16_MAX) {
			pending_event->both_overlap_run++;
		}
	} else {
		pending_event->both_overlap_run = 0U;
	}
	if (pending_event->both_overlap_run
			> pending_event->max_both_overlap_run) {
		pending_event->max_both_overlap_run = pending_event->both_overlap_run;
	}

	if (center_count >= MARKER_WIDE_CENTER_COUNT) {
		if (pending_event->wide_center_run < UINT16_MAX) {
			pending_event->wide_center_run++;
		}
		if (pending_event->wide_center_run
				> pending_event->max_wide_center_run) {
			pending_event->max_wide_center_run = pending_event->wide_center_run;
		}
	} else {
		pending_event->wide_center_run = 0U;
	}

	if (active) {
		pending_event->exit_mask = sensor_mask;
		pending_event->last_active_frame = frame_number;
		pending_event->last_active_step = average_step;
		pending_event->quiet_frames = 0U;
		if (pending_event->confirm_frames < UINT16_MAX) {
			pending_event->confirm_frames++;
		}
	} else if (pending_event->quiet_frames < UINT16_MAX) {
		pending_event->quiet_frames++;
	}

	if (pending_event->quiet_frames < TRACK_MARK_CLEAR_FRAMES) {
		return TRACK_PROCESS_NONE;
	}
	if (!Track_BuildCompletedEvent(runtime, &candidate)) {
		return TRACK_PROCESS_NONE;
	}
	return Track_CompleteCandidate(runtime, &candidate, store_event);
}

static TrackProcessResult_t Track_FlushInternal(
		TrackCollectorRuntime_t *runtime, bool store_event)
{
	TrackMarkerEvent_t candidate;

	if (!Track_BuildCompletedEvent(runtime, &candidate)) {
		return TRACK_PROCESS_NONE;
	}
	return Track_CompleteCandidate(runtime, &candidate, store_event);
}

void Track_Reset(void)
{
	memset(events, 0, sizeof(events));
	memset(segments, 0, sizeof(segments));
	memset(cross_anchors, 0, sizeof(cross_anchors));
	memset(&first_collector, 0, sizeof(first_collector));
	event_count = 0U;
	segment_count = 0U;
	cross_anchor_count = 0U;
	start_ignore_steps = 0U;
	overflow = false;
	cross_anchor_overflow = false;
	memset(&map_pair_diagnostics, 0, sizeof(map_pair_diagnostics));
	Track_ReplayReset();
}

void Track_SetStartIgnoreSteps(uint32_t minimum_step)
{
	start_ignore_steps = minimum_step;
}

TrackProcessResult_t Track_ProcessSensor(uint8_t sensor_mask,
		uint32_t frame_number, uint32_t average_step)
{
	return Track_ProcessInternal(&first_collector, sensor_mask, frame_number,
			average_step, true);
}

TrackProcessResult_t Track_Flush(void)
{
	return Track_FlushInternal(&first_collector, true);
}

const TrackMarkerEvent_t *Track_GetLastEvent(void)
{
	return &first_collector.last_event;
}

void Track_ReplayReset(void)
{
	memset(&replay_collector, 0, sizeof(replay_collector));
}

TrackProcessResult_t Track_ProcessReplaySensor(uint8_t sensor_mask,
		uint32_t frame_number, uint32_t average_step)
{
	return Track_ProcessInternal(&replay_collector, sensor_mask, frame_number,
			average_step, false);
}

const TrackMarkerEvent_t *Track_GetLastReplayEvent(void)
{
	return &replay_collector.last_event;
}

void Track_GetCollectorDiagnostics(bool replay,
		TrackCollectorDiagnostics_t *diagnostics)
{
	const TrackCollectorRuntime_t *runtime = replay
			? &replay_collector : &first_collector;

	if (diagnostics == NULL) {
		return;
	}
	diagnostics->cross_tail_suppressed_count =
			runtime->cross_tail_suppressed_count;
	diagnostics->cross_tail_affected_count =
			runtime->cross_tail_affected_count;
	diagnostics->last_cross_tail_gap_steps =
			runtime->last_cross_tail_gap_steps;
	diagnostics->max_cross_tail_gap_steps =
			runtime->max_cross_tail_gap_steps;
	diagnostics->last_cross_tail_edge_union =
			runtime->last_cross_tail_edge_union;
	diagnostics->cross_tail_pending = runtime->cross_tail_pending ? 1U : 0U;
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

uint16_t Track_GetCrossAnchorCount(void)
{
	return cross_anchor_count;
}

const TrackCrossAnchor_t *Track_GetCrossAnchor(uint16_t index)
{
	return (index < cross_anchor_count) ? &cross_anchors[index] : NULL;
}

const TrackCrossAnchor_t *Track_FindCrossAnchorByEventIndex(uint16_t event_index)
{
	uint16_t index;

	for (index = 0U; index < cross_anchor_count; index++) {
		if (cross_anchors[index].event_index == event_index) {
			return &cross_anchors[index];
		}
	}
	return NULL;
}

bool Track_HasOverflow(void)
{
	return overflow;
}

bool Track_HasAnchorOverflow(void)
{
	return cross_anchor_overflow;
}

void Track_GetMapPairDiagnostics(TrackMapPairDiagnostics_t *diagnostics)
{
	if (diagnostics == NULL) {
		return;
	}
	*diagnostics = map_pair_diagnostics;
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

static uint8_t Track_CalculateCurveUnits(TrackSegmentType_t type,
		uint32_t distance_steps)
{
	uint32_t units;
	uint32_t expected;
	uint32_t error;
	uint32_t tolerance;

	if ((type != TRACK_SEGMENT_LEFT) && (type != TRACK_SEGMENT_RIGHT)) {
		return 0U;
	}

	units = distance_steps / TRACK_CURVE_UNIT_NOMINAL_STEPS;
	if ((distance_steps % TRACK_CURVE_UNIT_NOMINAL_STEPS)
			>= (TRACK_CURVE_UNIT_NOMINAL_STEPS / 2U)) {
		units++;
	}
	if (units < TRACK_CURVE_UNIT_MIN) {
		units = TRACK_CURVE_UNIT_MIN;
	} else if (units > TRACK_CURVE_UNIT_MAX) {
		units = TRACK_CURVE_UNIT_MAX;
	}

	expected = units * TRACK_CURVE_UNIT_NOMINAL_STEPS;
	error = (distance_steps > expected) ? (distance_steps - expected)
			: (expected - distance_steps);
	tolerance = expected / 4U;
	if (tolerance < TRACK_CURVE_UNIT_ERROR_MIN_STEPS) {
		tolerance = TRACK_CURVE_UNIT_ERROR_MIN_STEPS;
	}
	return (error <= tolerance) ? (uint8_t)units : 0U;
}

static bool Track_AddCrossAnchor(uint16_t event_index,
		uint16_t segment_index_after_cross, uint32_t center_step,
		uint32_t distance_from_previous_cross)
{
	TrackCrossAnchor_t *anchor;

	if (cross_anchor_count >= TRACK_MAX_CROSS_ANCHORS) {
		cross_anchor_overflow = true;
		return false;
	}
	if (segment_index_after_cross >= TRACK_MAX_SEGMENTS) {
		cross_anchor_overflow = true;
		return false;
	}
	anchor = &cross_anchors[cross_anchor_count];
	anchor->order = cross_anchor_count;
	anchor->event_index = event_index;
	anchor->segment_index_after_cross = segment_index_after_cross;
	anchor->center_step = center_step;
	anchor->distance_from_previous_cross = distance_from_previous_cross;
	cross_anchor_count++;
	return true;
}

void Track_FinalizeSegments(void)
{
	uint16_t index;
	uint16_t first_index = 0U;
	uint32_t previous_cross_step = 0U;
	bool have_previous_cross = false;
	bool turn_open = false;
	int8_t turn_direction = 0;

	segment_count = 0U;
	cross_anchor_count = 0U;
	cross_anchor_overflow = false;
	memset(&map_pair_diagnostics, 0, sizeof(map_pair_diagnostics));
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
		segments[segment_count].curve_units = 0U;
		segment_count++;
	}

	for (index = first_index; index < event_count; index++) {
		TrackSegmentType_t type = Track_SegmentTypeFromEvent(events[index].type,
				&turn_open, &turn_direction);
		uint32_t distance = 0U;

		if (events[index].type == MARKER_EVENT_EDGE_0
				|| events[index].type == MARKER_EVENT_EDGE_7) {
			map_pair_diagnostics.last_direction =
					(events[index].type == MARKER_EVENT_EDGE_0) ? -1 : 1;
			map_pair_diagnostics.last_direction_step =
					events[index].center_step;
		}

		if (type == TRACK_SEGMENT_END) {
			if (segment_count < TRACK_MAX_SEGMENTS) {
				segments[segment_count].type = TRACK_SEGMENT_END;
				segments[segment_count].distance_steps = 0U;
				segments[segment_count].curve_units = 0U;
				segment_count++;
			} else {
				overflow = true;
			}
			break;
		}
		if ((index + 1U) >= event_count) {
			break;
		}
		if (events[index + 1U].center_step < events[index].center_step) {
			overflow = true;
			distance = 0U;
		} else {
			distance = events[index + 1U].center_step
					- events[index].center_step;
		}
		if (segment_count >= TRACK_MAX_SEGMENTS) {
			overflow = true;
			break;
		}
		segments[segment_count].type = type;
		segments[segment_count].distance_steps = distance;
		segments[segment_count].curve_units =
				Track_CalculateCurveUnits(type, distance);
		if (events[index].type == MARKER_EVENT_CROSS) {
			uint32_t distance_from_previous_cross = events[index].center_step;

			if (have_previous_cross) {
				if (events[index].center_step < previous_cross_step) {
					cross_anchor_overflow = true;
				} else {
					distance_from_previous_cross = events[index].center_step
							- previous_cross_step;
				}
			}
			(void)Track_AddCrossAnchor((uint16_t)index, segment_count,
					events[index].center_step, distance_from_previous_cross);
			previous_cross_step = events[index].center_step;
			have_previous_cross = true;
		}
		segment_count++;
	}
	map_pair_diagnostics.turn_open = turn_open ? 1U : 0U;
	map_pair_diagnostics.turn_direction = turn_direction;
	map_pair_diagnostics.unmatched_turn_at_end = turn_open ? 1U : 0U;
}
