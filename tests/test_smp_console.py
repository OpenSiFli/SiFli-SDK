# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Offline negative verification of the SMP console testcase.

The fixtures include the output contract and a captured SF32LB57GY result block.
These tests verify host matching; hardware acceptance is a separate board run.
All verdicts come from the real ConsoleMatcher and the actual testcase YAML.
"""

import itertools
import sys
import unittest
from pathlib import Path

import yaml

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

from sdk_test.harness.console import ConsoleMatcher, parse_record  # noqa: E402


CONTRACT_LINES = (
    "SMP_BEGIN\n",
    "SMP_RESULT rounds=32 threads=64 migrations=2048 errors=0 "
    "heap_restored=1 cpus=3 ticks=37\n",
    "SMP_PASS\n",
)
CONTRACT_TRANSCRIPT = "".join(CONTRACT_LINES)
# Actual legacy/NC UART result from SF32LB57GY, 2026-11-10. The shell prompt
# shares the result line. Some SDK UART lines use CRCRLF, covered separately.
CAPTURED_TRANSCRIPT = (
    "SMP_BEGIN\r\n"
    "msh />SMP_RESULT rounds=32 threads=64 migrations=2048 errors=0 "
    "heap_restored=1 cpus=3 ticks=353\r\n"
    "SMP_PASS\r\n"
)


class SmpConsoleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = REPOSITORY_ROOT / "example" / "multicore" / "smp" / "testcase.yaml"
        cases = yaml.safe_load(path.read_text(encoding="utf-8"))["tests"]
        cls.case = next(case for case in cases if case["id"] == "smp.console")

    def matcher(self):
        return ConsoleMatcher(parse_record(self.case["harness_config"]["record"]))

    def verdict(self, transcript, chunk_size=None):
        matcher = self.matcher()
        chunk_size = chunk_size or max(1, len(transcript))
        for start in range(0, len(transcript), chunk_size):
            matcher.feed(transcript[start:start + chunk_size])
        return matcher.state()

    def assert_rejected(self, transcript):
        # Single-byte input also exercises the accumulated-buffer behavior used
        # when the UART splits a marker or a numeric field across reads.
        for chunk_size in (None, 1):
            with self.subTest(chunk_size=chunk_size):
                state = self.verdict(transcript, chunk_size)
                self.assertFalse(state.passed, transcript)

    def test_complete_contract_matches_across_serial_chunks(self):
        for newline, ticks, chunk_size in itertools.product(
            ("\n", "\r\n", "\r\r\n"), ("0", "37", "123456789"), (None, 1, 7, 64)
        ):
            with self.subTest(newline=newline, ticks=ticks, chunk_size=chunk_size):
                transcript = (
                    "Boot banner\n" + CONTRACT_TRANSCRIPT.replace("ticks=37", f"ticks={ticks}")
                ).replace("\n", newline)
                state = self.verdict(transcript, chunk_size)
                self.assertTrue(state.passed, state.detail)

    def test_captured_57gy_result_matches_with_the_shell_prompt(self):
        for transcript, chunk_size in itertools.product(
            (CAPTURED_TRANSCRIPT, CAPTURED_TRANSCRIPT.replace("msh />", "msh >")),
            (None, 1, 7),
        ):
            with self.subTest(transcript=transcript, chunk_size=chunk_size):
                state = self.verdict(transcript, chunk_size)
                self.assertTrue(state.passed, state.detail)

    def test_boot_noise_before_the_result_is_allowed(self):
        transcript = CONTRACT_TRANSCRIPT.replace("SMP_BEGIN\n", "SMP_BEGIN\nboard ready\n")
        state = self.verdict(transcript)
        self.assertTrue(state.passed, state.detail)

    def test_only_the_complete_terminal_pass_finishes_the_case(self):
        matcher = self.matcher()
        for character in CONTRACT_TRANSCRIPT[:-1]:
            matcher.feed(character)
            self.assertFalse(matcher.complete)
        matcher.feed(CONTRACT_TRANSCRIPT[-1])
        self.assertTrue(matcher.complete)

    def test_first_middle_and_last_stage_are_required(self):
        for missing in range(len(CONTRACT_LINES)):
            with self.subTest(missing_stage=missing):
                self.assert_rejected("".join(
                    line for index, line in enumerate(CONTRACT_LINES) if index != missing
                ))

    def test_stages_cannot_be_shuffled(self):
        for order in itertools.permutations(range(len(CONTRACT_LINES))):
            if order == (0, 1, 2):
                continue
            with self.subTest(order=order):
                self.assert_rejected("".join(CONTRACT_LINES[index] for index in order))

    def test_failure_marker_cannot_be_skipped(self):
        for position in range(len(CONTRACT_LINES)):
            with self.subTest(position=position):
                lines = list(CONTRACT_LINES)
                lines.insert(position, "SMP_FAIL injected\n")
                self.assert_rejected("".join(lines))

    def test_wrong_results_cannot_be_hidden_by_pass(self):
        replacements = (
            ("rounds=32", "rounds=31"),
            ("threads=64", "threads=63"),
            ("migrations=2048", "migrations=2047"),
            ("errors=0", "errors=1"),
            ("errors=0", "errors=10"),
            ("heap_restored=1", "heap_restored=0"),
            ("cpus=3", "cpus=1"),
            ("cpus=3", "cpus=2"),
            ("ticks=37", "ticks=unknown"),
        )
        for original, broken in replacements:
            with self.subTest(broken=broken):
                self.assert_rejected(CONTRACT_TRANSCRIPT.replace(original, broken))

    def test_an_incomplete_boot_cannot_borrow_from_a_later_good_boot(self):
        for missing in range(len(CONTRACT_LINES)):
            with self.subTest(missing_stage=missing):
                incomplete = "".join(
                    line for index, line in enumerate(CONTRACT_LINES) if index != missing
                )
                self.assert_rejected(incomplete + "Boot banner\n" + CONTRACT_TRANSCRIPT)

    def test_complementary_partial_boots_do_not_make_one_pass(self):
        self.assert_rejected(
            CONTRACT_LINES[0] + CONTRACT_LINES[1]
            + "Boot banner\n" + CONTRACT_LINES[0] + CONTRACT_LINES[2]
        )

    def test_a_failed_result_cannot_borrow_a_later_correct_result(self):
        self.assert_rejected(
            CONTRACT_LINES[0] + CONTRACT_LINES[1].replace("errors=0", "errors=1")
            + CONTRACT_LINES[1] + CONTRACT_LINES[2]
        )


if __name__ == "__main__":
    unittest.main()
