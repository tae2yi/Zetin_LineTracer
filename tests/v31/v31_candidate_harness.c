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

static void ClearCandidate(uint32_t step)
{
	FirstDrive_HostTestCandidateClear(step);
}

int main(void)
{
	FirstDriveMarkerSummary_t summary;
	uint16_t frame;

	/* No raw S0/S7 edge: off-centre line frames are not candidates. */
	FirstDrive_HostTestCandidateReset();
	for (frame = 0U; frame < 1000U; frame++) {
		FirstDrive_HostTestCandidateFrame(0U, true, 1000, 0U, false,
				(uint32_t)frame);
	}
	ClearCandidate(1000U);
	FirstDrive_HostTestGetCandidateSummary(&summary);
	Check(summary.candidate_episode_count == 0U
			&& summary.candidate_rejected_count == 0U,
			"1000 off-centre frames without raw edge stay uncounted");
	printf("V31 raw candidate gate: no_edge_frames=1000 episodes=%u rejected=%u\n",
		(unsigned int)summary.candidate_episode_count,
		(unsigned int)summary.candidate_rejected_count);

	/* One physical edge episode, rejected by the off-centre gate. */
	FirstDrive_HostTestCandidateReset();
	for (frame = 0U; frame < 30U; frame++) {
		FirstDrive_HostTestCandidateFrame(0x01U, true, 2000, 0x3CU, false,
				100U + (uint32_t)frame);
	}
	ClearCandidate(130U);
	FirstDrive_HostTestGetCandidateSummary(&summary);
	Check(summary.candidate_episode_count == 1U
			&& summary.candidate_rejected_count == 1U
			&& summary.candidate_reject_off_center_count == 1U,
			"off-centre edge is one rejected episode");

	/* The same physical episode accumulates both OFF_CENTER and NO_LINE. */
	FirstDrive_HostTestCandidateReset();
	FirstDrive_HostTestCandidateFrame(0x01U, true, 2000, 0x3CU, false, 200U);
	FirstDrive_HostTestCandidateFrame(0x01U, false, 0, 0x3CU, false, 201U);
	ClearCandidate(202U);
	FirstDrive_HostTestGetCandidateSummary(&summary);
	Check(summary.candidate_episode_count == 1U
			&& summary.candidate_rejected_count == 1U
			&& summary.candidate_reject_off_center_count == 1U
			&& summary.candidate_reject_no_line_count == 1U,
			"candidate reason mask retains two gate reasons");

	/* Accepted direction event: clear debounce must not split one marker. */
	FirstDrive_HostTestCandidateReset();
	for (frame = 0U; frame < 3U; frame++) {
		FirstDrive_HostTestCandidateFrame(0x01U, true, 0, 0x3CU,
				frame == 0U, 300U + (uint32_t)frame);
	}
	ClearCandidate(303U);
	FirstDrive_HostTestGetCandidateSummary(&summary);
	Check(summary.candidate_episode_count == 1U
			&& summary.candidate_accepted_count == 1U
			&& summary.candidate_rejected_count == 0U,
			"accepted marker is one accepted episode");
	printf("V31 candidate episodes: off_center=1 mask={OFF_CENTER,NO_LINE} "
		"accepted=1 rejected=0\n");

	if (failures != 0U) {
		printf("%u failures\n", failures);
		return EXIT_FAILURE;
	}
	puts("V31 candidate harness PASS");
	return EXIT_SUCCESS;
}
