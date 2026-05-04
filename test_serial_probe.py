import builtins
import dataclasses
import datetime as dt
import logging
from pathlib import Path

import pytest
from pytest import CaptureFixture, MonkeyPatch

import serial_probe


def fake_input_from(values: list[str]):
    iterator = iter(values)

    def fake_input(_prompt: str) -> str:
        return next(iterator)

    return fake_input


class TypeErrorWriter:
    def write(self, data: bytes) -> int:
        raise TypeError("programming error")


class OSErrorWriter:
    def write(self, data: bytes) -> int:
        raise OSError("serial device unavailable")


class ValueErrorSerialModule:
    SEVENBITS = 7
    EIGHTBITS = 8
    PARITY_NONE = "N"
    PARITY_EVEN = "E"
    PARITY_ODD = "O"
    PARITY_MARK = "M"
    PARITY_SPACE = "S"
    STOPBITS_ONE = 1
    STOPBITS_TWO = 2

    @staticmethod
    def Serial(**_kwargs: object) -> object:
        raise ValueError("unsupported parity mode")


def test_receive_completion_detects_complete_eight_bit_payload() -> None:
    nonce = serial_probe.representative_nonce()
    payload = serial_probe.generate_eight_bit_payload(
        serial_probe.DEFAULT_EIGHT_BIT_PAYLOAD_BYTES,
        nonce,
    )

    assert serial_probe.receive_completion_detected(payload.data, payload.data)
    assert not serial_probe.receive_completion_detected(payload.data[:-1], payload.data)


def test_serial_write_os_error_is_reported_as_io_error() -> None:
    payload = serial_probe.generate_payload(serial_probe.DEFAULT_PAYLOAD_BYTES)

    bytes_sent, error, elapsed = serial_probe.write_payload_only(
        in_serial=OSErrorWriter(),
        settings=serial_probe.SerialSettings(9600, 8, "none", 1, "none"),
        payload=payload,
        progress_interval=1.0,
        prefix="[TEST]",
        logger=logging.getLogger("test"),
    )

    assert bytes_sent == 0
    assert error == "serial device unavailable"
    assert elapsed >= 0.0


def test_serial_open_value_error_is_reported_as_configuration_error() -> None:
    settings = serial_probe.SerialSettings(9600, 8, "mark", 1, "none")

    with pytest.raises(serial_probe.SerialConfigurationError) as exc_info:
        serial_probe.open_serial_port(
            ValueErrorSerialModule,
            "COM1",
            settings,
            1.0,
        )

    assert "COM1 9600 8M1 FLOW=NONE" in str(exc_info.value)
    assert "unsupported parity mode" in str(exc_info.value)


def test_serial_write_type_error_is_not_hidden_as_io_error() -> None:
    payload = serial_probe.generate_payload(serial_probe.DEFAULT_PAYLOAD_BYTES)

    with pytest.raises(TypeError, match="programming error"):
        serial_probe.write_payload_only(
            in_serial=TypeErrorWriter(),
            settings=serial_probe.SerialSettings(9600, 8, "none", 1, "none"),
            payload=payload,
            progress_interval=1.0,
            prefix="[TEST]",
            logger=logging.getLogger("test"),
        )


def test_import_pyserial_missing_reports_install_command_without_install(
    monkeypatch: MonkeyPatch,
    capsys: CaptureFixture[str],
) -> None:
    real_import = builtins.__import__

    def missing_serial_import(
        name: str,
        globals_: dict[str, object] | None = None,
        locals_: dict[str, object] | None = None,
        fromlist: tuple[str, ...] = (),
        level: int = 0,
    ) -> object:
        if name == "serial":
            raise ImportError("No module named serial")
        return real_import(name, globals_, locals_, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", missing_serial_import)

    with pytest.raises(SystemExit) as exc_info:
        serial_probe.import_pyserial()

    assert exc_info.value.code == 2
    assert (
        "INSTALL PYSERIAL WITH: PYTHON -M PIP INSTALL PYSERIAL"
        in capsys.readouterr().out
    )


def test_main_menu_uses_numbered_options_only(
    capsys: CaptureFixture[str],
) -> None:
    serial_probe.print_commands()

    output = capsys.readouterr().out

    assert "  7  MEMORY TEST" in output
    assert "  8  CURRENT SETTINGS" in output
    assert "  9  HELP" in output
    assert "  0  QUIT" in output
    assert "S  CURRENT SETTINGS" not in output
    assert "?  HELP" not in output


def test_main_menu_rejects_letter_shortcuts(
    monkeypatch: MonkeyPatch,
    capsys: CaptureFixture[str],
) -> None:
    monkeypatch.setattr(builtins, "input", fake_input_from(["s", "?", "0"]))

    selection = serial_probe.interactive_menu(serial_probe.default_scan_options())
    output = capsys.readouterr().out

    assert selection is None
    assert output.count("ENTER A NUMBER FROM 0 THROUGH 9.") == 2
    assert "OPTION 2 PORTS" not in output


def test_current_settings_show_option_2_ports_and_bauds(
    monkeypatch: MonkeyPatch,
) -> None:
    captured_lines: list[str] = []

    def capture_paged_lines(
        lines: list[str],
        page_lines: int = serial_probe.PAGE_BODY_LINES,
        pause_at_end: bool = True,
        return_label: str = "RETURN",
    ) -> None:
        captured_lines.extend(lines)

    monkeypatch.setattr(serial_probe, "print_paged_lines", capture_paged_lines)
    options = dataclasses.replace(
        serial_probe.default_scan_options(),
        in_port="COM2",
        out_port="COM6",
        input_baud=38400,
        output_baud=9600,
    )

    serial_probe.print_configuration(options)
    output = "\n".join(captured_lines)

    assert "OPTION 2 PORTS:" in output
    assert "COM2 INPUT -> COM6 OUTPUT" in output
    assert "OPTION 2 BAUDS:" in output
    assert "INPUT 38400 / OUTPUT 9600" in output
    assert "SCAN BAUD RANGE:" in output
    assert "ASKED IN START SCAN 1 OR 3" in output


def test_configure_ports_updates_ports_and_bauds(monkeypatch: MonkeyPatch) -> None:
    monkeypatch.setattr(
        builtins,
        "input",
        fake_input_from(["COM2", "38400", "COM6", "9600"]),
    )

    options = serial_probe.configure_ports(serial_probe.default_scan_options())

    assert options.in_port == "COM2"
    assert options.input_baud == 38400
    assert options.out_port == "COM6"
    assert options.output_baud == 9600


def test_start_scan_discovery_prompts_for_baud_range(
    monkeypatch: MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        builtins,
        "input",
        fake_input_from(["1", "1", "1200", "9600"]),
    )

    selection = serial_probe.interactive_menu(serial_probe.default_scan_options())

    assert selection is not None
    assert selection.action == "scan"
    assert selection.workflow == "discovery"
    assert selection.options.min_baud == 1200
    assert selection.options.max_baud == 9600


def test_start_scan_bank2_uses_configured_bauds_without_range_prompt(
    monkeypatch: MonkeyPatch,
) -> None:
    options = dataclasses.replace(
        serial_probe.default_scan_options(),
        input_baud=38400,
        output_baud=19200,
        min_baud=300,
        max_baud=9600,
    )
    monkeypatch.setattr(builtins, "input", fake_input_from(["1", "2"]))

    selection = serial_probe.interactive_menu(options)

    assert selection is not None
    assert selection.action == "scan"
    assert selection.workflow == "bank2"
    assert selection.options.input_baud == 38400
    assert selection.options.output_baud == 19200
    assert selection.options.min_baud == 300
    assert selection.options.max_baud == 9600


def test_memory_test_bauds_use_configured_port_bauds() -> None:
    options = dataclasses.replace(
        serial_probe.default_scan_options(),
        input_baud=38400,
        output_baud=9600,
        min_baud=300,
        max_baud=1200,
    )

    assert serial_probe.default_memory_test_bauds(options) == (38400, 9600)


def test_memory_target_presets_and_64k_defaults() -> None:
    assert serial_probe.BUFFER_PURGE_CAPACITY_BYTES == 64 * serial_probe.KIB_BYTES
    assert (
        serial_probe.DEFAULT_MAX_DRAIN_BYTES
        >= serial_probe.BUFFER_PURGE_CAPACITY_BYTES * 2
    )
    assert serial_probe.MEMORY_TEST_MAX_TARGET_KIB == 64
    assert serial_probe.parse_memory_test_target_choice("") == 64 * serial_probe.KIB_BYTES
    assert serial_probe.parse_memory_test_target_choice("1") == 16 * serial_probe.KIB_BYTES
    assert serial_probe.parse_memory_test_target_choice("64k") == 64 * serial_probe.KIB_BYTES
    assert serial_probe.parse_memory_test_target_choice("5") == "custom"
    assert (
        serial_probe.memory_test_mode_label(
            serial_probe.MEMORY_TEST_MODE_FILL,
            32 * serial_probe.KIB_BYTES,
        )
        == "FILL 32K"
    )
    assert (
        serial_probe.memory_test_fill_payload_bytes(
            38400,
            19200,
            64 * serial_probe.KIB_BYTES,
        )
        == 128 * serial_probe.KIB_BYTES
    )


def test_memory_loop_status_and_summary_use_neutral_terms() -> None:
    payload = serial_probe.generate_payload(serial_probe.DEFAULT_PAYLOAD_BYTES)
    settings = serial_probe.memory_test_settings(38400, 9600)
    config = serial_probe.MemoryTestConfig(
        in_port="COM1",
        out_port="COM5",
        input_baud=38400,
        output_baud=9600,
        mode=serial_probe.MEMORY_TEST_MODE_IMAGE,
        payload_bytes=payload.byte_count,
        target_bytes=64 * serial_probe.KIB_BYTES,
        loop_count=1,
        accept_full_result=False,
        stop_on_unexpected=True,
        switch_note="",
        run_id="MTEST",
    )
    result = serial_probe.MemoryTestResult(
        config=config,
        settings=settings,
        payload=payload,
        purge=serial_probe.DrainResult(0, 0.0, True, "quiet", None),
        candidate=serial_probe.fake_clean_candidate_result(settings, payload),
        loop_index=1,
        loop_total=1,
        started_at="",
        completed_at="",
        elapsed_sec=0.0,
    )

    diagnosis = serial_probe.memory_test_diagnosis(result)

    assert serial_probe.memory_test_loop_status(diagnosis) == "OK"
    assert serial_probe.memory_test_result_expected(result)
    assert serial_probe.memory_test_summary_status(config, [result], False) == "OK"


def test_memory_test_high_bit_ascii_output_reports_frame_issue() -> None:
    nonce = serial_probe.ProbeNonce("RUNTEST", "MEMORY", "L001", None)
    payload = serial_probe.generate_payload(1024, nonce)
    received = bytes(byte | 0x80 for byte in payload.data[:768])
    score = serial_probe.score_received(payload.data, received)
    trial = serial_probe.TrialResult(
        burst_index=1,
        bytes_sent=payload.byte_count,
        bytes_received=len(received),
        bytes_drained_before=0,
        drain_status="quiet",
        score=score.score,
        metrics=score.metrics,
        status="weak",
        error=None,
        elapsed_sec=0.0,
        timing=serial_probe.zero_timing_breakdown(),
        received_preview_ascii="",
        received_preview_hex="",
        score_classification=score.classification,
        evidence=score.evidence,
        nonce_summary=nonce.compact(),
        payload_mode=payload.payload_mode,
        bytes_received_at_write_done=100,
    )
    settings = serial_probe.memory_test_settings(38400, 19200)
    candidate = serial_probe.aggregate_candidate_result(
        index=1,
        total=1,
        settings=settings,
        trials=[trial],
        elapsed_sec=0.0,
    )
    result = serial_probe.MemoryTestResult(
        config=serial_probe.MemoryTestConfig(
            in_port="COM1",
            out_port="COM5",
            input_baud=38400,
            output_baud=19200,
            mode=serial_probe.MEMORY_TEST_MODE_FILL,
            payload_bytes=payload.byte_count,
            target_bytes=512,
            loop_count=1,
            accept_full_result=True,
            stop_on_unexpected=True,
            switch_note="",
            run_id=nonce.run_id,
        ),
        settings=settings,
        payload=payload,
        purge=serial_probe.DrainResult(0, 0.0, True, "quiet", None),
        candidate=candidate,
        loop_index=1,
        loop_total=1,
        started_at="",
        completed_at="",
        elapsed_sec=0.0,
    )

    diagnosis = serial_probe.memory_test_diagnosis(result)

    assert diagnosis.code == "probable-frame-mismatch"
    assert serial_probe.memory_test_loop_status(diagnosis) == "FRAME"
    assert diagnosis.ram_check == "NOT JUDGED"


def test_bank2_behavior_payload_classes_cover_control_ranges() -> None:
    payloads = dict(serial_probe.bank2_behavior_probe_payloads("B2RUN", 1, None))

    assert {"PRINT_CTRL", "ASCII_SWEEP", "CTL7_SAFE", "CTL7_FULL"} <= set(payloads)
    assert b"\x1b@" in payloads["PRINT_CTRL"]
    assert b"\x1bE" in payloads["PRINT_CTRL"]
    assert b"\x1bF" in payloads["PRINT_CTRL"]
    assert bytes(range(0x20, 0x7F)) in payloads["ASCII_SWEEP"]
    control_safe = payloads["CTL7_SAFE"]
    control_full = payloads["CTL7_FULL"]

    assert b"\x11" not in control_safe
    assert b"\x13" not in control_safe
    assert b"\x11" in control_full
    assert b"\x13" in control_full


def test_raw_byte_diff_metrics_and_xon_summary() -> None:
    settings = serial_probe.SerialSettings(9600, 8, "none", 1, "none")

    def result(name: str, exact: bool) -> serial_probe.Bank2BehaviorProbeResult:
        return serial_probe.Bank2BehaviorProbeResult(
            name=name,
            settings=settings,
            bytes_sent=4,
            bytes_received=4 if exact else 3,
            sent_hash="00000001",
            received_hash="00000001" if exact else "00000002",
            first_mismatch_offset=None if exact else 3,
            missing_bytes=0 if exact else 1,
            extra_bytes=0,
            exact_match=exact,
            form_feed_inserted=False,
            cr_lf_changed=False,
            received_preview_ascii="",
            received_preview_hex="",
            status="exact" if exact else "partial",
            reason="EXACT BYTE MATCH." if exact else "PARTIAL OUTPUT.",
            error=None,
            elapsed_sec=0.0,
        )

    assert serial_probe.first_mismatch_offset(b"ABCD", b"ABX") == 2
    assert serial_probe.first_mismatch_offset(b"ABCD", b"ABCD") is None
    assert (
        serial_probe.bank2_behavior_summary(
            [result("CTL7_SAFE", True), result("CTL7_FULL", False)]
        )
        == "XON/XOFF CONTROL BYTES AFFECT RAW PATH"
    )


def test_bank2_report_columns_are_fixed_width(
    capsys: CaptureFixture[str],
) -> None:
    settings = serial_probe.DualSerialSettings(
        serial_probe.SerialSettings(38400, 8, "even", 1, "none"),
        serial_probe.SerialSettings(19200, 8, "even", 1, "none"),
    )
    payload = serial_probe.generate_payload(serial_probe.DEFAULT_PAYLOAD_BYTES)
    ascii_result = serial_probe.fake_clean_candidate_result(settings, payload)
    behavior = serial_probe.Bank2BehaviorProbeResult(
        name="CR_ONLY",
        settings=settings,
        bytes_sent=105,
        bytes_received=105,
        sent_hash="9BEE4EF1",
        received_hash="9BEE4EF1",
        first_mismatch_offset=None,
        missing_bytes=0,
        extra_bytes=0,
        exact_match=True,
        form_feed_inserted=False,
        cr_lf_changed=False,
        received_preview_ascii="",
        received_preview_hex="",
        status="exact",
        reason="EXACT BYTE MATCH.",
        error=None,
        elapsed_sec=0.0,
    )
    result = serial_probe.Bank2CharacterizationResult(
        switch_note="",
        known_baud_text="IN 38400 / OUT 19200",
        ascii_results=[ascii_result],
        eight_bit_results=[],
        flow_results=[],
        behavior_results=[behavior],
        etx_ack_results=[],
        stale_data_seen=False,
        conclusion="8-bit clean; raw bytes exact",
        run_id="B2TEST",
        flow_skip_reason=None,
    )

    serial_probe.print_bank2_report(result, Path("serial_probe_report.txt"))

    lines = capsys.readouterr().out.splitlines()
    known_line = next(line for line in lines if "KNOWN BAUD/PAIR:" in line)
    probe_line = next(line for line in lines if "PROBE FRAME:" in line)
    assert known_line.index("IN 38400") == probe_line.index("IN 8E1")

    raw_header = next(line for line in lines if "RAW PROBE" in line)
    raw_row = next(line for line in lines if line.strip().startswith("CR_ONLY"))
    assert raw_header == serial_probe.bank2_raw_console_header()
    assert raw_row == serial_probe.bank2_raw_console_row(behavior)
    assert len(serial_probe.bank2_raw_report_header()) == serial_probe.REPORT_WIDTH
    assert len(serial_probe.bank2_raw_report_row(behavior)) == serial_probe.REPORT_WIDTH


def test_terminal_progress_rows_fit_80_columns() -> None:
    assert serial_probe.TERMINAL_COLUMNS == 80
    assert serial_probe.SCREEN_WIDTH == 80
    assert serial_probe.REPORT_WIDTH == 80
    assert (
        serial_probe.PROGRESS_WIDTH + serial_probe.PROGRESS_TIMESTAMP_WIDTH
        == serial_probe.TERMINAL_COLUMNS
    )
    assert all(
        len(line) == serial_probe.TERMINAL_COLUMNS
        for line in serial_probe.banner_lines()
    )

    payload = serial_probe.generate_payload(512)
    single = dataclasses.replace(
        serial_probe.fake_clean_candidate_result(
            serial_probe.SerialSettings(38400, 8, "even", 1, "dsr/dtr"),
            payload,
        ),
        index=70,
        total=70,
        bytes_sent=64 * serial_probe.KIB_BYTES,
        bytes_received=64 * serial_probe.KIB_BYTES,
        bytes_drained_before=128 * serial_probe.KIB_BYTES,
    )
    dual = dataclasses.replace(
        single,
        settings=serial_probe.DualSerialSettings(
            serial_probe.SerialSettings(38400, 8, "even", 1, "dsr/dtr"),
            serial_probe.SerialSettings(19200, 8, "even", 2, "xon/xoff"),
        ),
    )
    phase0 = serial_probe.DualBaudLivenessResult(
        input_baud=38400,
        output_baud=19200,
        alive=False,
        reason="NO VALID PROBE LINE",
        settings=serial_probe.dual_phase0_settings(38400, 19200),
        score=7.46,
        status="weak",
        error=None,
        bytes_sent=128,
        bytes_received=64 * serial_probe.KIB_BYTES,
        bytes_drained_before=128 * serial_probe.KIB_BYTES,
        elapsed_sec=0.0,
        metrics=single.metrics,
    )
    eta = serial_probe.format_scan_eta(
        completed=70,
        total=70,
        started_monotonic=0.0,
        now_monotonic=3600.0,
        clock_now=dt.datetime(2026, 5, 3, 20, 0, 0),
    )

    rows = [
        serial_probe.format_progress(single),
        serial_probe.format_dual_progress(dual),
        serial_probe.format_dual_phase0_progress(phase0, 70, 70),
        eta,
    ]

    assert all(len(row) <= serial_probe.TERMINAL_COLUMNS for row in rows)
