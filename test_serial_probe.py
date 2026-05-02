import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import serial_probe as sp


def make_candidate_result(
    received: bytes,
    *,
    status: str | None = None,
    error: str | None = None,
    bytes_sent: int | None = None,
    settings: sp.SerialSettings | None = None,
    payload: sp.ProbePayload | None = None,
    index: int = 1,
    total: int = 1,
) -> sp.CandidateResult:
    payload = sp.generate_phase0_payload() if payload is None else payload
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
        index=index,
        total=total,
        settings=settings or sp.phase0_baseline_settings(9600),
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


def make_exploratory_selection(
    *,
    results: list[sp.CandidateResult] | None = None,
    narrowed_candidates: list[sp.SerialSettings] | None = None,
    viable_candidates: list[sp.SerialSettings] | None = None,
    fallback_reason: str | None = "NO STRICT SHORTLIST",
) -> sp.ExploratorySelection:
    results = [] if results is None else results
    return sp.ExploratorySelection(
        results=results,
        ranked_results=sorted(results, key=sp.result_sort_key, reverse=True),
        shortlist_results=[],
        narrowed_candidates=[] if narrowed_candidates is None else narrowed_candidates,
        elapsed_sec=0.0,
        fallback_reason=fallback_reason,
        notes=[],
        cutoff_score=None,
        truncated=False,
        baud_focus=sp.empty_baud_focus_report(True),
        phase0_liveness=sp.empty_baud_liveness_report(0),
        viable_candidates=[] if viable_candidates is None else viable_candidates,
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


class Phase2ViabilityTests(unittest.TestCase):
    def test_exploratory_thresholds_match_phase2_filter_policy(self) -> None:
        self.assertEqual(sp.EXPLORATORY_MIN_NARROW_SCORE, 30.0)
        self.assertEqual(sp.EXPLORATORY_SHORTLIST_LIMIT, 12)

    def test_phase2_viability_helper_classifies_signal_statuses(self) -> None:
        for status in ("weak", "partial", "strong", "exact"):
            with self.subTest(status=status):
                result = make_candidate_result(b"x", status=status)

                self.assertTrue(sp.is_phase2_viable_signal(result))

        for status in ("error", "no-data", "stale-output", "partial-write"):
            with self.subTest(status=status):
                result = make_candidate_result(b"x", status=status)

                self.assertFalse(sp.is_phase2_viable_signal(result))

        errored_signal = make_candidate_result(
            b"x",
            status="strong",
            error="write timeout",
        )

        self.assertFalse(sp.is_phase2_viable_signal(errored_signal))

    def test_exploratory_selection_carries_ranked_viable_candidates(self) -> None:
        payload = sp.generate_payload(sp.minimum_payload_size())
        exact_settings = sp.SerialSettings(9600, 8, "none", 1, "none")
        weak_settings = sp.SerialSettings(9600, 7, "even", 1, "none")
        stale_settings = sp.SerialSettings(9600, 8, "odd", 1, "none")
        error_settings = sp.SerialSettings(9600, 8, "mark", 1, "none")
        results = [
            make_candidate_result(
                payload.data,
                status="exact",
                settings=exact_settings,
                payload=payload,
            ),
            make_candidate_result(
                b"x",
                status="weak",
                settings=weak_settings,
                payload=payload,
            ),
            make_candidate_result(
                payload.data,
                status="stale-output",
                settings=stale_settings,
                payload=payload,
            ),
            make_candidate_result(
                payload.data,
                status="strong",
                error="driver error",
                settings=error_settings,
                payload=payload,
            ),
        ]

        selection = sp.select_exploratory_candidates(
            results=results,
            all_candidates=[
                exact_settings,
                weak_settings,
                stale_settings,
                error_settings,
            ],
            elapsed_sec=0.0,
            baud_focus=sp.empty_baud_focus_report(True),
            phase0_liveness=sp.empty_baud_liveness_report(4),
        )

        self.assertEqual(
            selection.viable_candidates,
            [exact_settings, weak_settings],
        )

    def test_phase2_uses_viable_candidates_when_narrowing_unavailable(self) -> None:
        all_candidates = sp.exhaustive_candidates([9600])[:4]
        viable_candidates = [all_candidates[2], all_candidates[0]]
        selection = make_exploratory_selection(
            viable_candidates=viable_candidates,
            fallback_reason="BEST EXPLORATORY SCORE IS BELOW STRICT CUTOFF",
        )

        candidates, source = sp.phase2_candidates_after_exploratory(
            all_candidates,
            selection,
            narrowing_accepted=False,
        )

        self.assertEqual(candidates, viable_candidates)
        self.assertEqual(source, sp.PHASE2_CANDIDATE_SOURCE_VIABLE)

    def test_phase2_uses_viable_candidates_when_narrowing_declined(self) -> None:
        all_candidates = sp.exhaustive_candidates([9600])[:5]
        narrowed_candidates = [all_candidates[4]]
        viable_candidates = [all_candidates[1], all_candidates[3]]
        selection = make_exploratory_selection(
            narrowed_candidates=narrowed_candidates,
            viable_candidates=viable_candidates,
            fallback_reason=None,
        )

        candidates, source = sp.phase2_candidates_after_exploratory(
            all_candidates,
            selection,
            narrowing_accepted=False,
        )

        self.assertEqual(candidates, viable_candidates)
        self.assertEqual(source, sp.PHASE2_CANDIDATE_SOURCE_VIABLE)

    def test_phase2_keeps_narrowed_candidates_when_accepted(self) -> None:
        all_candidates = sp.exhaustive_candidates([9600])[:5]
        narrowed_candidates = [all_candidates[4]]
        viable_candidates = [all_candidates[1], all_candidates[3]]
        selection = make_exploratory_selection(
            narrowed_candidates=narrowed_candidates,
            viable_candidates=viable_candidates,
            fallback_reason=None,
        )

        candidates, source = sp.phase2_candidates_after_exploratory(
            all_candidates,
            selection,
            narrowing_accepted=True,
        )

        self.assertEqual(candidates, narrowed_candidates)
        self.assertEqual(source, sp.PHASE2_CANDIDATE_SOURCE_NARROWED)


class RunScanPhase2FlowTests(unittest.TestCase):
    def run_scan_with_selection(
        self,
        selection: sp.ExploratorySelection,
        prompt_answers: list[bool],
    ) -> tuple[list[sp.SerialSettings], dict[str, object], str, list[str]]:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            options = sp.dataclasses.replace(
                sp.default_scan_options(),
                min_baud=9600,
                max_baud=9600,
                auto_validate_top_matches=False,
                ask_on_top_match=False,
                json_report=temp_path / "report.json",
                csv_report=temp_path / "summary.csv",
                log_file=temp_path / "scan.log",
            )

            def fake_run_candidate(**kwargs: object) -> sp.CandidateResult:
                payload = kwargs["payload"]
                self.assertIsInstance(payload, sp.ProbePayload)
                return make_candidate_result(
                    payload.data,
                    settings=kwargs["settings"],
                    payload=payload,
                    index=kwargs["index"],
                    total=kwargs["total"],
                )

            with (
                mock.patch.object(
                    sp,
                    "import_or_install_pyserial",
                    return_value=SimpleNamespace(VERSION="test"),
                ),
                mock.patch.object(sp, "prompt_scan_mode", return_value="manual"),
                mock.patch.object(
                    sp,
                    "prompt_yes_no_question",
                    side_effect=prompt_answers,
                ),
                mock.patch.object(sp, "run_exploratory_scan", return_value=selection),
                mock.patch.object(sp, "run_candidate", side_effect=fake_run_candidate)
                as run_candidate,
                mock.patch.object(sp, "write_json_report") as write_json_report,
                mock.patch.object(sp, "write_csv_report"),
                mock.patch("builtins.print") as print_mock,
            ):
                self.assertEqual(sp.run_scan(options), 0)

            selected_settings = [
                call.kwargs["settings"] for call in run_candidate.call_args_list
            ]
            metadata = write_json_report.call_args.args[1]
            log_text = (temp_path / "scan.log").read_text(encoding="utf-8")
            printed = [
                " ".join(str(arg) for arg in call.args)
                for call in print_mock.call_args_list
            ]
            logger = sp.logging.getLogger("serial_probe")
            for handler in list(logger.handlers):
                logger.removeHandler(handler)
                handler.close()
            return selected_settings, metadata, log_text, printed

    def test_run_scan_uses_viable_candidates_when_narrowing_unavailable(self) -> None:
        all_candidates = sp.exhaustive_candidates([9600])
        viable_candidates = [all_candidates[3], all_candidates[7]]
        selection = make_exploratory_selection(
            narrowed_candidates=[],
            viable_candidates=viable_candidates,
            fallback_reason="NO STRICT QUICK SHORTLIST",
        )

        selected_settings, metadata, log_text, printed = self.run_scan_with_selection(
            selection,
            prompt_answers=[False, True],
        )

        self.assertEqual(selected_settings, viable_candidates)
        self.assertEqual(
            metadata["phase2_candidate_source"],
            sp.PHASE2_CANDIDATE_SOURCE_VIABLE,
        )
        self.assertIn("phase 2 candidate source=exploratory-viable-signals", log_text)
        self.assertTrue(
            any("PHASE 2 SIGNAL-ONLY MODE:" in line for line in printed)
        )

    def test_run_scan_uses_viable_candidates_when_narrowing_declined(self) -> None:
        all_candidates = sp.exhaustive_candidates([9600])
        narrowed_candidates = [all_candidates[12]]
        viable_candidates = [all_candidates[5], all_candidates[2]]
        selection = make_exploratory_selection(
            narrowed_candidates=narrowed_candidates,
            viable_candidates=viable_candidates,
            fallback_reason=None,
        )

        selected_settings, metadata, log_text, _ = self.run_scan_with_selection(
            selection,
            prompt_answers=[False, True, False],
        )

        self.assertEqual(selected_settings, viable_candidates)
        self.assertEqual(
            metadata["exploratory_mode"]["candidate_source"],
            sp.PHASE2_CANDIDATE_SOURCE_VIABLE,
        )
        self.assertIn("operator declined exploratory narrowing", log_text)
        self.assertIn("phase 2 candidate source=exploratory-viable-signals", log_text)


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
