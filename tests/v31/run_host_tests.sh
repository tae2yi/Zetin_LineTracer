#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_root=$(CDPATH= cd -- "$script_dir/../.." && pwd)
test_tmp=$(mktemp -d "${TMPDIR:-/tmp}/line_tracer_v31_host.XXXXXX")
trap 'rm -rf "$test_tmp"' EXIT

host_flags="-std=c11 -Wall -Wextra -Werror"
include_flags="-I$script_dir/host -I$repo_root/Main/Inc"

cc $host_flags $include_flags \
	"$script_dir/v31_planner_harness.c" \
	"$repo_root/Main/Src/track.c" \
	"$repo_root/Main/Src/second_drive.c" \
	-o "$test_tmp/v31_planner_harness"
"$test_tmp/v31_planner_harness"

cc $host_flags $include_flags \
	"$script_dir/v31_track_pair_harness.c" \
	"$repo_root/Main/Src/track.c" \
	-o "$test_tmp/v31_track_pair_harness"
"$test_tmp/v31_track_pair_harness"

cc $host_flags $include_flags \
	"$script_dir/v31_motor_brake_mock.c" \
	-o "$test_tmp/v31_motor_brake_mock"
"$test_tmp/v31_motor_brake_mock"

cc $host_flags $include_flags \
	"$script_dir/v31_curve_contract_harness.c" \
	-o "$test_tmp/v31_curve_contract_harness"
"$test_tmp/v31_curve_contract_harness"

cc $host_flags -DV31_HOST_TEST -ffunction-sections -fdata-sections \
	$include_flags \
	"$script_dir/v31_candidate_harness.c" \
	"$repo_root/Main/Src/drive.c" \
	"$repo_root/Main/Src/track.c" \
	"$repo_root/Main/Src/second_drive.c" \
	-Wl,-dead_strip -o "$test_tmp/v31_candidate_harness"
"$test_tmp/v31_candidate_harness"

grep -q "marker_edges == 0U" "$repo_root/Main/Src/drive.c"
grep -q "FirstDrive_MarkerCandidateForceFinalize" "$repo_root/Main/Src/drive.c"
grep -q "APP_VERSION_NUMBER 31U" "$repo_root/Main/Inc/app_version.h"
grep -q "second_drive_run_screen_locked" "$repo_root/Main/Src/menu.c"
grep -q "second_drive_active_render_count = 0U" "$repo_root/Main/Src/menu.c"
printf '%s\n' "V31 source marker/version checks PASS"
