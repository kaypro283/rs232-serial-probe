import unittest

import serial_probe as sp


def make_candidate_result(
    received: bytes,
    *,
    status: str | None = None,
    error: str | None = None,
    bytes_sent: int | None = None,
) -> sp.CandidateResult:
    payload = sp.generate_phase0_payload()
    score = sp.score_received(payload.data, received)
    if status is None:
        if error:
            status = "error"
        elif not received:
            status = "no-data"
        elif score.score >= 99.0:
            status = "exact"
        elif score.score >= 90.0:
            status = "strong"
        elif score.score >= 50.0:
            status = "partial"
        else:
            status = "weak"
    return sp.CandidateResult(
        index=1,
        total=1,
        settings=sp.phase0_baseline_settings(9600),
        bytes_sent=payload.byte_count if bytes_sent is None else bytes_sent,
        bytes_received=len(received),
        bytes_drained_before=0,
        score=score.score,
        repeatability=1.0 if score.score >= 98.0 else 0.0,
        status=status,
        error=error,
        elapsed_sec=0.0,
        timing=sp.zero_timing_breakdown(),
        metrics=score.metrics,
        trials=[],
    )


class Phase0LivenessTests(unittest.TestCase):
    def test_exact_phase0_payload_is_alive(self) -> None:
        payload = sp.generate_phase0_payload()
        result = make_candidate_result(payload.data)

        decision = sp.classify_phase0_liveness(result, payload.byte_count)

        self.assertTrue(decision.alive)
        self.assertEqual(decision.reason, "VALID PROBE STRUCTURE")

    def test_no_data_is_not_alive(self) -> None:
        payload = sp.generate_phase0_payload()
        result = make_candidate_result(b"", status="no-data")

        decision = sp.classify_phase0_liveness(result, payload.byte_count)

        self.assertFalse(decision.alive)
        self.assertEqual(decision.reason, "NO DATA")

    def test_valid_line_without_marker_is_not_alive(self) -> None:
        payload = sp.generate_phase0_payload()
        line = next(
            line
            for line in payload.data.splitlines(keepends=True)
            if line.startswith(b"LINE ")
        )
        result = make_candidate_result(line)

        decision = sp.classify_phase0_liveness(result, payload.byte_count)

        self.assertFalse(decision.alive)
        self.assertEqual(decision.reason, "NO PROBE MARKER")

    def test_stale_output_is_not_alive_even_with_matching_payload(self) -> None:
        payload = sp.generate_phase0_payload()
        result = make_candidate_result(payload.data, status="stale-output")

        decision = sp.classify_phase0_liveness(result, payload.byte_count)

        self.assertFalse(decision.alive)
        self.assertEqual(decision.reason, "OUTPUT NOT QUIET")

    def test_excess_extra_output_is_not_alive(self) -> None:
        payload = sp.generate_phase0_payload()
        extra = b"X" * (sp.phase0_extra_byte_limit(payload.byte_count) + 1)
        result = make_candidate_result(payload.data + extra)

        decision = sp.classify_phase0_liveness(result, payload.byte_count)

        self.assertFalse(decision.alive)
        self.assertEqual(decision.reason, "EXTRA OUTPUT")

    def test_phase0_candidate_filter_keeps_only_alive_bauds(self) -> None:
        candidates = sp.exhaustive_candidates([9600, 4800])
        report = sp.BaudLivenessReport(
            ran=True,
            tested_bauds=[9600, 4800],
            alive_bauds=[9600],
            fallback_to_all_bauds=False,
            fallback_reason=None,
            candidate_count_before=len(candidates),
            candidate_count_after=80,
            elapsed_sec=0.0,
            results=[],
        )

        filtered = sp.candidates_after_phase0_liveness(candidates, report)

        self.assertEqual(len(filtered), 80)
        self.assertEqual({candidate.baud for candidate in filtered}, {9600})

    def test_phase0_candidate_filter_preserves_all_on_fallback(self) -> None:
        candidates = sp.exhaustive_candidates([9600, 4800])
        report = sp.BaudLivenessReport(
            ran=True,
            tested_bauds=[9600, 4800],
            alive_bauds=[],
            fallback_to_all_bauds=True,
            fallback_reason="NO ALIVE BAUDS",
            candidate_count_before=len(candidates),
            candidate_count_after=len(candidates),
            elapsed_sec=0.0,
            results=[],
        )

        filtered = sp.candidates_after_phase0_liveness(candidates, report)

        self.assertEqual(filtered, candidates)


class TurboDiscoveryTimingTests(unittest.TestCase):
    def test_turbo_read_timeout_adapts_by_baud(self) -> None:
        options = sp.dataclasses.replace(
            sp.default_scan_options(),
            turbo_discovery_enabled=True,
        )
        high = sp.effective_discovery_timing(
            options,
            sp.phase0_baseline_settings(38400),
            options.payload_bytes,
        )
        low = sp.effective_discovery_timing(
            options,
            sp.phase0_baseline_settings(110),
            options.payload_bytes,
        )

        self.assertLess(high.read_timeout, sp.DEFAULT_READ_TIMEOUT)
        self.assertGreater(low.read_timeout, high.read_timeout)

    def test_turbo_prioritizes_common_frame_before_low_value_frame(self) -> None:
        options = sp.dataclasses.replace(
            sp.default_scan_options(),
            turbo_discovery_enabled=True,
            min_baud=9600,
            max_baud=9600,
        )
        low_value = sp.SerialSettings(9600, 7, "space", 2, "dsr/dtr")
        likely = sp.SerialSettings(9600, 8, "none", 1, "none")

        ordered = sp.prioritize_discovery_candidates([low_value, likely], options)

        self.assertEqual(ordered[0], likely)

    def test_exact_payload_scoring_still_returns_perfect_score(self) -> None:
        payload = sp.generate_payload(sp.minimum_payload_size())

        score = sp.score_received(payload.data, payload.data)

        self.assertEqual(score.score, sp.PERFECT_SCORE)
        self.assertTrue(score.metrics.end_marker_present)

    def test_receive_completion_detects_exact_payload(self) -> None:
        payload = sp.generate_payload(sp.minimum_payload_size())

        self.assertTrue(sp.receive_completion_detected(payload.data, payload.data))
        self.assertFalse(
            sp.receive_completion_detected(
                payload.data[:-10],
                payload.data,
            )
        )


if __name__ == "__main__":
    unittest.main()
