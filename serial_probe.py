#!/usr/bin/env python3
"""Discover likely serial settings for a serial-to-serial printer buffer.

The script assumes a device under test sits between the transmit and receive
ports. It opens both PC serial ports with matching candidate settings, sends
deterministic probe payloads into the buffer input, scores what is received
from the buffer output, and appends a compact text report for each run.
"""

from __future__ import annotations

import atexit
import base64
import ctypes
import dataclasses
import datetime as dt
import logging
import os
import platform
import statistics
import subprocess
import sys
import threading
import textwrap
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Sequence

BAUD_RATES: list[int] = [
    75,
    110,
    150,
    300,
    600,
    1200,
    2400,
    4800,
    9600,
    19200,
    38400,
]
DATA_BITS: list[int] = [8, 7]
PARITIES: list[str] = ["none", "even", "odd", "mark", "space"]
STOP_BITS: list[int] = [1, 2]
FLOW_CONTROLS: list[str] = ["none", "xon/xoff", "rts/cts", "dsr/dtr"]
DEFAULT_BURSTS = 1
DEFAULT_PAYLOAD_BYTES = 180
DEFAULT_READ_TIMEOUT = 2.0
DEFAULT_SETTLE_MS = 50
DEFAULT_PROGRESS_INTERVAL = 1.0
DEFAULT_PRE_DRAIN_TIMEOUT = 0.5
DEFAULT_PRE_DRAIN_QUIET = 0.1
DEFAULT_MAX_DRAIN_BYTES = 32_768
TURBO_DISCOVERY_ENABLED_DEFAULT = False
BUFFER_PURGE_ENABLED = True
BUFFER_PURGE_CAPACITY_BYTES = 16 * 1024
BUFFER_PURGE_QUIET_SECONDS = 1.5
BUFFER_PURGE_PER_BAUD_MAX_SECONDS = 2.5
BUFFER_PURGE_PROGRESS_INTERVAL = 1.0
FLOW_VALIDATE_PAYLOAD_BYTES = 1024
FLOW_VALIDATE_READ_TIMEOUT = 2.0
FLOW_VALIDATE_HOLD_SECONDS = 1.0
FLOW_VALIDATE_RELEASE_SETTLE_SECONDS = 0.15
TURBO_SETTLE_MS = 10
TURBO_PRE_DRAIN_TIMEOUT = 0.25
TURBO_PRE_DRAIN_QUIET = 0.05
TURBO_READ_TIMEOUT_HIGH_BAUD = 0.35
TURBO_READ_TIMEOUT_MID_BAUD = 0.45
TURBO_READ_TIMEOUT_LOW_BAUD = 0.70
TURBO_READ_TIMEOUT_VERY_LOW_BAUD = 1.00
TURBO_COMPLETION_QUIET = 0.05
DEFAULT_COMPLETION_QUIET = 0.10
PHASE0_PAYLOAD_BYTES = 128
PHASE0_READ_TIMEOUT = 1.0
PHASE0_SETTLE_MS = 10
PHASE0_BURSTS = 1
PHASE0_PROGRESS_INTERVAL = 1.0
PHASE0_PRE_DRAIN_TIMEOUT = 1.5
PHASE0_PRE_DRAIN_QUIET = 0.25
PHASE0_MAX_DRAIN_BYTES = DEFAULT_MAX_DRAIN_BYTES
PHASE0_MIN_ALIVE_SCORE = 90.0
PHASE0_MIN_LINE_INTEGRITY = 1.0
PHASE0_MAX_EXTRA_BYTES = 16
PHASE0_MAX_EXTRA_BYTES_RATIO = 0.25
PHASE0_BASELINE_DATA_BITS = 8
PHASE0_BASELINE_PARITY = "even"
PHASE0_BASELINE_STOP_BITS = 1
PHASE0_BASELINE_FLOW_CONTROL = "none"
DUAL_PHASE0_BAUD_PAIR_LIMIT = 6
DUAL_PHASE0_FALLBACK_PAIR_LIMIT = 4
EXPLORATORY_PAYLOAD_BYTES = 160
EXPLORATORY_READ_TIMEOUT = 0.4
EXPLORATORY_SETTLE_MS = 20
EXPLORATORY_BURSTS = 1
EXPLORATORY_PROGRESS_INTERVAL = 1.0
EXPLORATORY_PRE_DRAIN_TIMEOUT = 0.5
EXPLORATORY_PRE_DRAIN_QUIET = 0.1
EXPLORATORY_MAX_DRAIN_BYTES = DEFAULT_MAX_DRAIN_BYTES
EXPLORATORY_SHORTLIST_LIMIT = 12
EXPLORATORY_MIN_NARROW_SCORE = 30.0
EXPLORATORY_SCORE_TOLERANCE = 10.0
EXPLORATORY_SUMMARY_ROWS = 8
PHASE2_VIABLE_SIGNAL_STATUSES = {"weak", "partial", "strong", "exact"}
PHASE2_CANDIDATE_SOURCE_NARROWED = "exploratory-narrowed"
PHASE2_CANDIDATE_SOURCE_VIABLE = "exploratory-viable-signals"
PHASE2_CANDIDATE_SOURCE_FULL = "all-selected-settings"
BAUD_FOCUS_ENABLED_DEFAULT = True
BAUD_FOCUS_STRONG_SCORE_THRESHOLD = 90.0
BAUD_FOCUS_LEAD_GAP_THRESHOLD = 20.0
BAUD_FOCUS_MIN_STRONG_RESULTS = 3
BAUD_FOCUS_MIN_SAMPLES = 8
FNV_OFFSET_32 = 0x811C9DC5
FNV_PRIME_32 = 0x01000193
ProgressCallback = Callable[[str], None]
ANSI_GREEN = "\033[92m"
ANSI_RESET = "\033[0m"
STD_OUTPUT_HANDLE = -11
ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
SCREEN_WIDTH = 72
REPORT_WIDTH = 78
PROGRESS_WIDTH = 70
TERMINAL_COLUMNS = 80
PAGE_BODY_LINES = 22
RECOMMENDATION_MIN_SCORE = 90.0
TOP_MATCH_MIN_SCORE = 99.0
TIE_SCORE_TOLERANCE = 0.5
PERFECT_SCORE = 100.0
OPERATOR_BREAK_EXIT_CODE = 130


def flow_control_code(flow_control: str) -> str:
    """Return a short flow-control code for compact dual-bank displays."""
    return {
        "none": "NONE",
        "xon/xoff": "XON",
        "rts/cts": "RTS",
        "dsr/dtr": "DSR",
    }[flow_control]


class ReturnToMainMenuAfterReport(Exception):
    """Signal that the operator asked for the main menu after report writing."""


class QuitProgramAfterReport(Exception):
    """Signal that the operator asked to quit after report writing."""


@dataclass(frozen=True)
class SerialSettings:
    """A complete serial configuration candidate."""

    baud: int
    data_bits: int
    parity: str
    stop_bits: int
    flow_control: str

    def parity_code(self) -> str:
        """Return the common one-letter parity code."""
        return {
            "none": "N",
            "even": "E",
            "odd": "O",
            "mark": "M",
            "space": "S",
        }[self.parity]

    def label(self) -> str:
        """Return a compact human-readable settings label."""
        flow = self.flow_control.upper()
        return (
            f"{self.baud} {self.data_bits}{self.parity_code()}"
            f"{self.stop_bits} FLOW={flow}"
        )


@dataclass(frozen=True)
class DualSerialSettings:
    """Independent input and output serial settings for a dual-bank buffer."""

    input_settings: SerialSettings
    output_settings: SerialSettings

    @property
    def baud(self) -> int:
        """Return input baud for write-side timing helpers."""
        return self.input_settings.baud

    @property
    def data_bits(self) -> int:
        """Return input data bits for write-side timing helpers."""
        return self.input_settings.data_bits

    @property
    def parity(self) -> str:
        """Return input parity for write-side timing helpers."""
        return self.input_settings.parity

    @property
    def stop_bits(self) -> int:
        """Return input stop bits for write-side timing helpers."""
        return self.input_settings.stop_bits

    @property
    def flow_control(self) -> str:
        """Return input flow control for compatibility with result helpers."""
        return self.input_settings.flow_control

    def parity_code(self) -> str:
        """Return the input-side parity code for compatibility helpers."""
        return self.input_settings.parity_code()

    def input_mode(self) -> str:
        """Return compact input-side baud/frame/flow text."""
        return (
            f"{self.input_settings.baud} "
            f"{self.input_settings.data_bits}{self.input_settings.parity_code()}"
            f"{self.input_settings.stop_bits} "
            f"{flow_control_code(self.input_settings.flow_control)}"
        )

    def output_mode(self) -> str:
        """Return compact output-side baud/frame/flow text."""
        return (
            f"{self.output_settings.baud} "
            f"{self.output_settings.data_bits}{self.output_settings.parity_code()}"
            f"{self.output_settings.stop_bits} "
            f"{flow_control_code(self.output_settings.flow_control)}"
        )

    def label(self) -> str:
        """Return a compact human-readable dual-bank setting label."""
        return f"IN {self.input_mode()} -> OUT {self.output_mode()}"


@dataclass(frozen=True)
class ProbePayload:
    """Generated deterministic probe bytes and expected line metadata."""

    data: bytes
    line_count: int
    byte_count: int
    body_hash: str


@dataclass(frozen=True)
class ScoreMetrics:
    """Quality metrics for one received burst."""

    exact_byte_match_ratio: float
    line_integrity_ratio: float
    missing_bytes: int
    extra_bytes: int
    printable_ascii_ratio: float
    length_ratio: float
    start_marker_present: bool
    end_marker_present: bool


@dataclass(frozen=True)
class ScoreResult:
    """Score and metrics for one received burst."""

    score: float
    metrics: ScoreMetrics


@dataclass(frozen=True)
class DrainResult:
    """Result from draining stale output before sending a burst."""

    bytes_drained: int
    elapsed_sec: float
    quiet: bool
    reason: str
    error: str | None


@dataclass(frozen=True)
class TrialResult:
    """Result from one payload burst under one candidate setting."""

    burst_index: int
    bytes_sent: int
    bytes_received: int
    bytes_drained_before: int
    drain_status: str
    score: float
    metrics: ScoreMetrics
    status: str
    error: str | None
    elapsed_sec: float
    timing: "TimingBreakdown"
    received_preview_ascii: str
    received_preview_hex: str


@dataclass(frozen=True)
class TimingBreakdown:
    """Wall-clock stage timing for one candidate or burst."""

    open_setup_sec: float
    drain_sec: float
    write_sec: float
    read_wait_sec: float
    other_sec: float


@dataclass(frozen=True)
class CandidateResult:
    """Aggregated result for one serial settings candidate."""

    index: int
    total: int
    settings: SerialSettings | DualSerialSettings
    bytes_sent: int
    bytes_received: int
    bytes_drained_before: int
    score: float
    repeatability: float
    status: str
    error: str | None
    elapsed_sec: float
    timing: TimingBreakdown
    metrics: ScoreMetrics
    trials: list[TrialResult]


@dataclass(frozen=True)
class ScanOptions:
    """Runtime options that drive a scan."""

    in_port: str
    out_port: str
    min_baud: int
    max_baud: int
    payload_bytes: int
    read_timeout: float
    settle_ms: int
    top: int
    text_report: Path
    log_file: Path
    switch_note: str
    bursts: int
    progress_interval: float
    no_pre_drain: bool
    pre_drain_timeout: float
    pre_drain_quiet: float
    max_drain_bytes: int
    turbo_discovery_enabled: bool
    ask_on_top_match: bool
    auto_validate_top_matches: bool
    validate_size_1_bytes: int
    validate_size_2_tie_bytes: int
    auto_validate_flow_control: bool
    flow_validate_size_bytes: int
    baud_focus_enabled: bool
    baud_focus_strong_score_threshold: float
    baud_focus_lead_gap_threshold: float
    baud_focus_min_strong_results: int
    baud_focus_min_samples: int


@dataclass(frozen=True)
class EffectiveTiming:
    """Effective phase-1 timing after applying discovery speed policy."""

    read_timeout: float
    settle_ms: int
    pre_drain_quiet: float
    pre_drain_timeout: float
    completion_quiet: float


@dataclass(frozen=True)
class BaudFocusStats:
    """Observed exploratory score pattern for one baud rate."""

    baud: int
    samples: int
    strong_results: int
    best_score: float


@dataclass(frozen=True)
class BaudFocusDecision:
    """Current baud-focus confidence decision."""

    baud: int | None
    reason: str | None
    disabled_reason: str | None


@dataclass(frozen=True)
class BaudFocusReport:
    """Run summary for exploratory baud-focused narrowing."""

    enabled: bool
    engaged: bool
    focused_baud: int | None
    deferred_other_bauds: bool
    deferred_candidate_count: int
    deferred_bauds: list[int]
    engage_reason: str | None
    release_reason: str | None
    disabled_reason: str | None
    tested_before_engage: int | None


@dataclass(frozen=True)
class Phase0LivenessDecision:
    """Boolean liveness decision for one Phase 0 baud probe."""

    alive: bool
    reason: str


@dataclass(frozen=True)
class BaudLivenessResult:
    """Phase 0 result for one baseline baud probe."""

    baud: int
    alive: bool
    reason: str
    settings: SerialSettings
    score: float
    status: str
    error: str | None
    bytes_sent: int
    bytes_received: int
    bytes_drained_before: int
    elapsed_sec: float
    metrics: ScoreMetrics


@dataclass(frozen=True)
class BaudLivenessReport:
    """Run summary for the Phase 0 baud liveness sweep."""

    ran: bool
    tested_bauds: list[int]
    alive_bauds: list[int]
    fallback_to_all_bauds: bool
    fallback_reason: str | None
    candidate_count_before: int
    candidate_count_after: int
    elapsed_sec: float
    results: list[BaudLivenessResult]


@dataclass(frozen=True)
class DualBaudLivenessResult:
    """Phase 0 result for one input/output baud pair."""

    input_baud: int
    output_baud: int
    alive: bool
    reason: str
    settings: DualSerialSettings
    score: float
    status: str
    error: str | None
    bytes_sent: int
    bytes_received: int
    bytes_drained_before: int
    elapsed_sec: float
    metrics: ScoreMetrics


@dataclass(frozen=True)
class DualBaudLivenessReport:
    """Run summary for the dual-bank Phase 0 baud matrix."""

    ran: bool
    tested_pairs: list[tuple[int, int]]
    total_pairs: int
    alive_pairs: list[tuple[int, int]]
    selected_pairs: list[tuple[int, int]]
    fallback_reason: str | None
    elapsed_sec: float
    results: list[DualBaudLivenessResult]


@dataclass(frozen=True)
class ExploratorySelection:
    """Ranked exploratory findings and the optional full-scan candidate subset."""

    results: list[CandidateResult]
    ranked_results: list[CandidateResult]
    shortlist_results: list[CandidateResult]
    narrowed_candidates: list[SerialSettings]
    elapsed_sec: float
    fallback_reason: str | None
    notes: list[str]
    cutoff_score: float | None
    truncated: bool
    baud_focus: BaudFocusReport
    phase0_liveness: BaudLivenessReport
    viable_candidates: list[SerialSettings] = field(default_factory=list)


@dataclass(frozen=True)
class MemoryTestResult:
    """Result from one memory test transfer size."""

    size_bytes: int
    size_label: str
    method: str
    settings: SerialSettings
    bytes_sent: int
    bytes_received: int
    bytes_cleared_before: int
    bytes_seen_before_release: int
    score: float
    indicator: str
    status: str
    error: str | None
    elapsed_sec: float
    metrics: ScoreMetrics


@dataclass(frozen=True)
class FlowControlValidationResult:
    """Result from one post-scan flow-control validation."""

    flow_control: str
    method: str
    settings: SerialSettings
    bytes_sent: int
    bytes_received: int
    bytes_seen_while_held: int
    score: float
    indicator: str
    status: str
    reason: str
    error: str | None
    elapsed_sec: float
    metrics: ScoreMetrics


def fnv1a32(data: bytes) -> int:
    """Return a deterministic FNV-1a 32-bit hash for bytes."""
    value = FNV_OFFSET_32
    for byte in data:
        value ^= byte
        value = (value * FNV_PRIME_32) & 0xFFFFFFFF
    return value


def make_probe_line(line_number: int, kind: str, data: str) -> bytes:
    """Build one checksummed ASCII line for the probe payload."""
    block_number = (line_number - 1) % 32
    left = (
        f"LINE {line_number:06d} BLOCK={block_number:02d} "
        f"TYPE={kind} DATA={data}"
    ).encode("ascii")
    checksum = fnv1a32(left)
    return left + f" HASH={checksum:08X}\r\n".encode("ascii")


def repeated_ascii_pattern(line_number: int, length: int) -> str:
    """Return a deterministic printable ASCII pattern of exactly length chars."""
    alphabet = (
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789"
        " .,:;!?/+*-_=#@[](){}<>"
    )
    offset = line_number % len(alphabet)
    rotated = alphabet[offset:] + alphabet[:offset]
    repeats = (length // len(rotated)) + 1
    return (rotated * repeats)[:length]


def minimum_payload_size() -> int:
    """Return the smallest payload size this generator can represent."""
    start = b"<<<SERIAL_PROBE_BEGIN VERSION=1 TARGET_BYTES=00000000>>>\r\n"
    end = b"<<<SERIAL_PROBE_END LINES=000000 HASH=00000000>>>\r\n"
    smallest_line = make_probe_line(1, "PAD", "")
    return len(start) + len(end) + len(smallest_line)


def generate_payload(payload_bytes: int) -> ProbePayload:
    """Generate an ASCII-only probe payload of exactly payload_bytes bytes.

    The payload contains fixed-width start/end markers and checksummed line
    records. It is deterministic and suitable for unit tests.
    """
    min_size = minimum_payload_size()
    if payload_bytes < min_size:
        raise ValueError(f"payload-bytes must be at least {min_size}")

    start = (
        f"<<<SERIAL_PROBE_BEGIN VERSION=1 TARGET_BYTES={payload_bytes:08d}>>>\r\n"
    ).encode("ascii")
    end_len = len(b"<<<SERIAL_PROBE_END LINES=000000 HASH=00000000>>>\r\n")
    lines: list[bytes] = []
    line_number = 1
    current_len = len(start)

    sample_full_line = make_probe_line(1, "DATA", repeated_ascii_pattern(1, 96))
    min_pad_len = len(make_probe_line(1, "PAD", ""))

    while True:
        candidate = make_probe_line(
            line_number,
            "DATA",
            repeated_ascii_pattern(line_number, 96),
        )
        remaining_after = payload_bytes - current_len - len(candidate) - end_len
        if remaining_after == 0 or remaining_after >= min_pad_len:
            lines.append(candidate)
            current_len += len(candidate)
            line_number += 1
            if remaining_after == 0:
                break
            continue

        remaining_for_pad = payload_bytes - current_len - end_len
        if remaining_for_pad <= 0:
            break

        block_number = (line_number - 1) % 32
        pad_prefix_len = len(
            f"LINE {line_number:06d} BLOCK={block_number:02d} "
            "TYPE=PAD DATA=".encode("ascii")
        )
        pad_suffix_len = len(b" HASH=00000000\r\n")
        pad_data_len = remaining_for_pad - pad_prefix_len - pad_suffix_len
        if pad_data_len < 0:
            # Remove one regular line, then fill the larger remaining space.
            if not lines:
                raise ValueError(f"payload-bytes must be at least {min_size}")
            removed = lines.pop()
            current_len -= len(removed)
            line_number -= 1
            remaining_for_pad = payload_bytes - current_len - end_len
            pad_data_len = remaining_for_pad - pad_prefix_len - pad_suffix_len
            if pad_data_len < 0:
                raise ValueError("could not fit final pad line into payload")
        pad_data = repeated_ascii_pattern(line_number, pad_data_len)
        lines.append(make_probe_line(line_number, "PAD", pad_data))
        current_len += len(lines[-1])
        line_number += 1
        break

    if not lines:
        # Defensive fallback; normal control flow always emits at least one line.
        lines.append(sample_full_line)
        current_len += len(sample_full_line)
        line_number += 1

    line_count = len(lines)
    body = start + b"".join(lines)
    body_hash = fnv1a32(body)
    end = (
        f"<<<SERIAL_PROBE_END LINES={line_count:06d} HASH={body_hash:08X}>>>\r\n"
    ).encode("ascii")
    payload = body + end

    if len(payload) != payload_bytes:
        raise AssertionError(
            f"payload generator produced {len(payload)} bytes, expected {payload_bytes}"
        )
    if any(byte not in (10, 13) and not 32 <= byte <= 126 for byte in payload):
        raise AssertionError("payload contains non-printable bytes")

    return ProbePayload(
        data=payload,
        line_count=line_count,
        byte_count=len(payload),
        body_hash=f"{body_hash:08X}",
    )


def parse_valid_probe_lines(data: bytes) -> dict[int, int]:
    """Return valid line numbers and checksums found in probe-like bytes."""
    valid: dict[int, int] = {}
    for raw_line in data.splitlines():
        if not raw_line.startswith(b"LINE ") or b" HASH=" not in raw_line:
            continue
        try:
            left, hash_text = raw_line.rsplit(b" HASH=", 1)
            line_number = int(left[5:11])
            reported_hash = int(hash_text[:8], 16)
        except (ValueError, IndexError):
            continue
        if fnv1a32(left) == reported_hash:
            valid[line_number] = reported_hash
    return valid


def printable_ascii_ratio(data: bytes) -> float:
    """Return the share of bytes that are printable ASCII, CR, LF, or TAB."""
    if not data:
        return 0.0
    printable = sum(1 for byte in data if byte in (9, 10, 13) or 32 <= byte <= 126)
    return printable / len(data)


def exact_byte_match_ratio(expected: bytes, received: bytes) -> float:
    """Return same-position byte equality over the longer byte sequence length."""
    denominator = max(len(expected), len(received), 1)
    same_positions = sum(
        1 for expected_byte, received_byte in zip(expected, received) if expected_byte == received_byte
    )
    return same_positions / denominator


def score_received(expected: bytes, received: bytes) -> ScoreResult:
    """Score received bytes against the expected probe payload."""
    expected_len = len(expected)
    received_len = len(received)
    missing = max(expected_len - received_len, 0)
    extra = max(received_len - expected_len, 0)
    length_ratio = min(expected_len, received_len) / max(expected_len, received_len, 1)
    exact_ratio = exact_byte_match_ratio(expected, received)
    expected_lines = parse_valid_probe_lines(expected)
    received_lines = parse_valid_probe_lines(received)
    matching_lines = sum(
        1
        for line_number, checksum in received_lines.items()
        if expected_lines.get(line_number) == checksum
    )
    line_ratio = matching_lines / max(len(expected_lines), 1)
    ascii_ratio = printable_ascii_ratio(received)
    start_marker = b"<<<SERIAL_PROBE_BEGIN" in received
    end_marker = b"<<<SERIAL_PROBE_END" in received
    marker_ratio = (int(start_marker) + int(end_marker)) / 2

    if not received:
        confidence = 0.0
    else:
        confidence = (
            55.0 * exact_ratio
            + 25.0 * line_ratio
            + 10.0 * ascii_ratio
            + 5.0 * length_ratio
            + 5.0 * marker_ratio
        )
        if (
            expected_len == received_len
            and exact_ratio == 1.0
            and line_ratio == 1.0
            and start_marker
            and end_marker
        ):
            confidence = 100.0

    metrics = ScoreMetrics(
        exact_byte_match_ratio=exact_ratio,
        line_integrity_ratio=line_ratio,
        missing_bytes=missing,
        extra_bytes=extra,
        printable_ascii_ratio=ascii_ratio,
        length_ratio=length_ratio,
        start_marker_present=start_marker,
        end_marker_present=end_marker,
    )
    return ScoreResult(score=max(0.0, min(100.0, confidence)), metrics=metrics)


def available_bauds(min_baud: int, max_baud: int) -> list[int]:
    """Return configured baud rates within inclusive min/max limits."""
    if min_baud > max_baud:
        raise ValueError("MINIMUM BAUD CANNOT BE GREATER THAN MAXIMUM BAUD")
    selected = [baud for baud in BAUD_RATES if min_baud <= baud <= max_baud]
    if not selected:
        raise ValueError("NO PROGRAM BAUD RATES ARE IN THAT RANGE")
    return selected


def scan_bauds(min_baud: int, max_baud: int) -> list[int]:
    """Return baud rates in the order used by the scan."""
    return list(reversed(available_bauds(min_baud, max_baud)))


def exhaustive_candidates(bauds: Sequence[int]) -> list[SerialSettings]:
    """Return the full Cartesian candidate list in the documented order."""
    return [
        SerialSettings(baud, data_bits, parity, stop_bits, flow)
        for baud in bauds
        for data_bits in DATA_BITS
        for parity in PARITIES
        for stop_bits in STOP_BITS
        for flow in FLOW_CONTROLS
    ]


def generate_candidates(
    min_baud: int,
    max_baud: int,
) -> list[SerialSettings]:
    """Generate the full Cartesian candidate list for a scan."""
    bauds = scan_bauds(min_baud, max_baud)
    return exhaustive_candidates(bauds)


def normalize_port_name(port: str) -> str:
    """Normalize a Windows COM port name for same-port checks."""
    normalized = port.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized


def ensure_distinct_ports(in_port: str, out_port: str) -> None:
    """Raise if the input and output ports resolve to the same port name."""
    if normalize_port_name(in_port) == normalize_port_name(out_port):
        raise ValueError("INPUT AND OUTPUT PORTS MUST NOT BE THE SAME")


def import_or_install_pyserial() -> Any:
    """Import pyserial, attempting installation if it is missing."""
    try:
        import serial  # type: ignore[import-not-found]

        return serial
    except ImportError:
        install_command = [sys.executable, "-m", "pip", "install", "pyserial"]
        print("PYSERIAL MISSING; TRYING: " + " ".join(install_command))
        try:
            subprocess.check_call(install_command)
        except (OSError, subprocess.CalledProcessError):
            print("INSTALL PYSERIAL WITH: PYTHON -M PIP INSTALL PYSERIAL")
            raise SystemExit(2)

    try:
        import serial  # type: ignore[import-not-found]

        return serial
    except ImportError:
        print("INSTALL PYSERIAL WITH: PYTHON -M PIP INSTALL PYSERIAL")
        raise SystemExit(2)


def serial_constants(serial_module: Any, settings: SerialSettings) -> dict[str, Any]:
    """Map plain settings values to pyserial constants and flow flags."""
    bytesize = {
        7: serial_module.SEVENBITS,
        8: serial_module.EIGHTBITS,
    }[settings.data_bits]
    parity = {
        "none": serial_module.PARITY_NONE,
        "even": serial_module.PARITY_EVEN,
        "odd": serial_module.PARITY_ODD,
        "mark": serial_module.PARITY_MARK,
        "space": serial_module.PARITY_SPACE,
    }[settings.parity]
    stopbits = {
        1: serial_module.STOPBITS_ONE,
        2: serial_module.STOPBITS_TWO,
    }[settings.stop_bits]
    return {
        "baudrate": settings.baud,
        "bytesize": bytesize,
        "parity": parity,
        "stopbits": stopbits,
        "xonxoff": settings.flow_control == "xon/xoff",
        "rtscts": settings.flow_control == "rts/cts",
        "dsrdtr": settings.flow_control == "dsr/dtr",
    }


def estimated_frame_bits(settings: SerialSettings | DualSerialSettings) -> float:
    """Estimate serial frame width in bits per transmitted byte."""
    parity_bits = 0 if settings.parity == "none" else 1
    return 1 + settings.data_bits + parity_bits + settings.stop_bits


def write_chunk_size(settings: SerialSettings | DualSerialSettings) -> int:
    """Choose a write chunk size that behaves well at very low baud rates."""
    bytes_per_second = max(1.0, settings.baud / estimated_frame_bits(settings))
    quarter_second = int(bytes_per_second * 0.25)
    return max(8, min(2048, quarter_second))


def open_serial_port(
    serial_module: Any,
    port: str,
    settings: SerialSettings,
    read_timeout: float,
) -> Any:
    """Open a serial port for one candidate setting."""
    constants = serial_constants(serial_module, settings)
    return serial_module.Serial(
        port=port,
        timeout=min(0.05, max(read_timeout, 0.001)),
        write_timeout=max(3.0, read_timeout),
        inter_byte_timeout=0.05,
        **constants,
    )


def reset_serial_buffers(serial_port: Any) -> None:
    """Best-effort reset of serial input and output buffers."""
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()


def preview_ascii(data: bytes, limit: int = 96) -> str:
    """Return a compact printable ASCII preview for logs and reports."""
    preview = data[:limit]
    chars = []
    for byte in preview:
        if byte in (10, 13):
            chars.append(" ")
        elif 32 <= byte <= 126:
            chars.append(chr(byte))
        else:
            chars.append(".")
    return "".join(chars)


def preview_hex(data: bytes, limit: int = 32) -> str:
    """Return a compact hex preview for logs and reports."""
    return data[:limit].hex(" ")


def enable_terminal_style() -> None:
    """Use green ANSI text for an early terminal look when stdout is a terminal."""
    if os.environ.get("NO_COLOR"):
        return
    wants_color = (
        sys.stdout.isatty()
        or os.environ.get("PYCHARM_HOSTED")
        or os.environ.get("FORCE_COLOR")
    )
    if not wants_color:
        return
    if os.name == "nt":
        try:
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.GetStdHandle(STD_OUTPUT_HANDLE)
            mode = ctypes.c_uint32()
            if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
                kernel32.SetConsoleMode(
                    handle,
                    mode.value | ENABLE_VIRTUAL_TERMINAL_PROCESSING,
                )
        except Exception:
            pass
    print(ANSI_GREEN, end="")
    atexit.register(lambda: print(ANSI_RESET, end=""))


def print_banner() -> None:
    """Print the terminal-style program banner."""
    for line in banner_lines():
        print(line)


def border_line(width: int = SCREEN_WIDTH) -> str:
    """Return an asterisk border line."""
    return "*" * width


def bordered_text(text: str, width: int = SCREEN_WIDTH) -> str:
    """Return one centered text line inside an asterisk border."""
    inner_width = max(width - 4, 1)
    cleaned = text[:inner_width]
    return f"* {cleaned.center(inner_width)} *"


def banner_lines() -> list[str]:
    """Return the terminal-style program banner lines."""
    return [
        border_line(SCREEN_WIDTH),
        bordered_text("SERIAL PROBE 1.0  -  PRINTER BUFFER SETUP", SCREEN_WIDTH),
        border_line(SCREEN_WIDTH),
    ]


def print_paged_lines(
    lines: Sequence[str],
    page_lines: int = PAGE_BODY_LINES,
) -> None:
    """Print lines with a simple 80x25-friendly page pause."""
    all_lines = list(lines)
    if page_lines <= 0:
        page_lines = len(all_lines)
    for index, line in enumerate(all_lines, start=1):
        print(line)
        if index >= len(all_lines) or index % page_lines != 0:
            continue
        try:
            choice = input("PRESS ENTER FOR MORE, Q TO STOP: ")
        except EOFError:
            print()
            for rest in all_lines[index:]:
                print(rest)
            return
        if choice.lstrip("\ufeff").strip().lower() in {"q", "quit", "0"}:
            return
        print()


def read_operator_input(prompt: str) -> str:
    """Read terminal input, tolerating occasional bad console bytes."""
    try:
        return input(prompt)
    except UnicodeDecodeError:
        return ""


def wrapped_value_lines(
    prefix: str,
    value: object,
    width: int = TERMINAL_COLUMNS,
) -> list[str]:
    """Return prefixed lines wrapped to the terminal width."""
    available = max(20, width - len(prefix))
    wrapped = textwrap.wrap(
        str(value),
        width=available,
        break_long_words=True,
    ) or [""]
    return [
        prefix + wrapped[0],
        *(" " * len(prefix) + line for line in wrapped[1:]),
    ]


def print_wrapped_value(prefix: str, value: object) -> None:
    """Print a prefixed value wrapped to the terminal width."""
    for line in wrapped_value_lines(prefix, value):
        print(line)


def print_report_title(title: str) -> None:
    """Print a terminal-style report section title."""
    print(border_line(REPORT_WIDTH))
    print(bordered_text(title, REPORT_WIDTH))
    print(border_line(REPORT_WIDTH))


def format_duration(seconds: float) -> str:
    """Return a compact human-readable duration."""
    if seconds < 1:
        return f"{seconds * 1000:.0f}MS"
    rounded = int(round(seconds))
    hours, remainder = divmod(rounded, 3600)
    minutes, secs = divmod(remainder, 60)
    if hours:
        return f"{hours}H{minutes:02d}M{secs:02d}S"
    if minutes:
        return f"{minutes}M{secs:02d}S"
    return f"{secs}S"


def format_finish_clock(remaining_seconds: float, now: dt.datetime | None = None) -> str:
    """Return an approximate local clock time for scan completion."""
    current = now if now is not None else dt.datetime.now().astimezone()
    finish = current + dt.timedelta(seconds=max(0.0, remaining_seconds))
    if finish.date() == current.date():
        return finish.strftime("%H:%M:%S")
    return finish.strftime("%Y-%m-%d %H:%M:%S")


def byte_size_label(byte_count: int) -> str:
    """Return an early-terminal-friendly byte count label."""
    if byte_count % 1024 == 0:
        return f"{byte_count // 1024}K"
    return f"{byte_count} BYTES"


def result_indicator(score: float, status: str, error: str | None = None) -> str:
    """Return a short human-readable success indicator for console output."""
    if status == "stale-output":
        return "STALE"
    if status == "partial-write":
        return "FAIL"
    if error or status == "error":
        return "ERROR"
    if status == "no-data":
        return "FAIL"
    if score >= 99.0:
        return "PASS"
    if score >= 90.0:
        return "GOOD"
    if score >= 50.0:
        return "PARTIAL"
    return "FAIL"


def estimated_transmit_seconds(settings: SerialSettings, byte_count: int) -> float:
    """Estimate physical transmit time for byte_count at candidate settings."""
    return (byte_count * estimated_frame_bits(settings)) / max(settings.baud, 1)


def zero_timing_breakdown() -> TimingBreakdown:
    """Return an empty timing breakdown."""
    return TimingBreakdown(
        open_setup_sec=0.0,
        drain_sec=0.0,
        write_sec=0.0,
        read_wait_sec=0.0,
        other_sec=0.0,
    )


def timing_total(timing: TimingBreakdown) -> float:
    """Return the summed stage time for a timing breakdown."""
    return (
        timing.open_setup_sec
        + timing.drain_sec
        + timing.write_sec
        + timing.read_wait_sec
        + timing.other_sec
    )


def combine_timing_breakdowns(timings: Sequence[TimingBreakdown]) -> TimingBreakdown:
    """Return stage totals for multiple timing breakdowns."""
    if not timings:
        return zero_timing_breakdown()
    return TimingBreakdown(
        open_setup_sec=sum(timing.open_setup_sec for timing in timings),
        drain_sec=sum(timing.drain_sec for timing in timings),
        write_sec=sum(timing.write_sec for timing in timings),
        read_wait_sec=sum(timing.read_wait_sec for timing in timings),
        other_sec=sum(timing.other_sec for timing in timings),
    )


def timing_with_other(
    elapsed_sec: float,
    open_setup_sec: float,
    drain_sec: float,
    write_sec: float,
    read_wait_sec: float,
) -> TimingBreakdown:
    """Return timing with unassigned time folded into the other bucket."""
    known = open_setup_sec + drain_sec + write_sec + read_wait_sec
    return TimingBreakdown(
        open_setup_sec=open_setup_sec,
        drain_sec=drain_sec,
        write_sec=write_sec,
        read_wait_sec=read_wait_sec,
        other_sec=max(0.0, elapsed_sec - known),
    )


def turbo_read_timeout_for_baud(settings: SerialSettings) -> float:
    """Return the turbo read-quiet timeout floor for a baud class."""
    if settings.baud <= 300:
        return TURBO_READ_TIMEOUT_VERY_LOW_BAUD
    if settings.baud <= 1200:
        return TURBO_READ_TIMEOUT_LOW_BAUD
    if settings.baud <= 4800:
        return TURBO_READ_TIMEOUT_MID_BAUD
    return TURBO_READ_TIMEOUT_HIGH_BAUD


def effective_discovery_timing(
    options: ScanOptions,
    settings: SerialSettings,
    payload_bytes: int,
) -> EffectiveTiming:
    """Return phase-1 timing for one setting after turbo/adaptive policy."""
    if not options.turbo_discovery_enabled:
        return EffectiveTiming(
            read_timeout=options.read_timeout,
            settle_ms=options.settle_ms,
            pre_drain_quiet=options.pre_drain_quiet,
            pre_drain_timeout=options.pre_drain_timeout,
            completion_quiet=min(options.read_timeout, DEFAULT_COMPLETION_QUIET),
        )

    payload_margin = min(0.25, estimated_transmit_seconds(settings, payload_bytes) * 0.02)
    read_timeout = turbo_read_timeout_for_baud(settings) + payload_margin
    settle_ms = min(options.settle_ms, TURBO_SETTLE_MS)
    return EffectiveTiming(
        read_timeout=read_timeout,
        settle_ms=settle_ms,
        pre_drain_quiet=min(options.pre_drain_quiet, TURBO_PRE_DRAIN_QUIET),
        pre_drain_timeout=min(options.pre_drain_timeout, TURBO_PRE_DRAIN_TIMEOUT),
        completion_quiet=min(read_timeout, TURBO_COMPLETION_QUIET),
    )


def effective_timing_range_label(options: ScanOptions, candidates: Sequence[SerialSettings]) -> str:
    """Return a compact operator label for effective phase-1 timing."""
    if not candidates:
        return "NO SETTINGS"
    timings = [
        effective_discovery_timing(options, candidate, options.payload_bytes)
        for candidate in candidates
    ]
    read_values = [timing.read_timeout for timing in timings]
    settle_values = [timing.settle_ms for timing in timings]
    drain_quiet_values = [timing.pre_drain_quiet for timing in timings]
    drain_limit_values = [timing.pre_drain_timeout for timing in timings]
    return (
        f"READ={min(read_values):.2f}..{max(read_values):.2f}S, "
        f"PAUSE={min(settle_values)}..{max(settle_values)}MS, "
        f"CLEAR={min(drain_limit_values):.2f}S/"
        f"{min(drain_quiet_values):.2f}S"
    )


def receive_completion_detected(received: bytes, expected: bytes) -> bool:
    """Return True when a received payload has enough structure to stop early."""
    if len(received) < len(expected):
        return False
    if b"<<<SERIAL_PROBE_END" not in received:
        return False
    if len(received) == len(expected):
        return True
    score = score_received(expected, received)
    return (
        score.score >= TOP_MATCH_MIN_SCORE
        and score.metrics.end_marker_present
        and score.metrics.line_integrity_ratio >= 1.0
    )


def discovery_frame_priority(settings: SerialSettings) -> tuple[int, int, int, int]:
    """Return a priority key that puts common printer-buffer frames first."""
    parity_rank = {
        "none": 0,
        "even": 1,
        "odd": 2,
        "mark": 4,
        "space": 5,
    }.get(settings.parity, 9)
    flow_rank = {
        "none": 0,
        "xon/xoff": 1,
        "rts/cts": 2,
        "dsr/dtr": 3,
    }.get(settings.flow_control, 9)
    data_rank = 0 if settings.data_bits == 8 else 1
    stop_rank = 0 if settings.stop_bits == 1 else 1
    if settings.data_bits == 8 and settings.parity == "none" and settings.stop_bits == 1:
        frame_rank = 0
    elif settings.data_bits == 8 and settings.parity in {"even", "odd"} and settings.stop_bits == 1:
        frame_rank = 1
    elif settings.data_bits == 7 and settings.parity in {"even", "odd"} and settings.stop_bits == 1:
        frame_rank = 2
    else:
        frame_rank = 3 + data_rank + stop_rank + parity_rank
    return frame_rank, flow_rank, parity_rank, stop_rank


def prioritize_discovery_candidates(
    candidates: Sequence[SerialSettings],
    options: ScanOptions,
) -> list[SerialSettings]:
    """Return candidates in turbo discovery priority order when enabled."""
    if not options.turbo_discovery_enabled:
        return list(candidates)
    baud_order = scan_bauds(options.min_baud, options.max_baud)
    baud_rank = {baud: index for index, baud in enumerate(baud_order)}
    return sorted(
        candidates,
        key=lambda candidate: (
            baud_rank.get(candidate.baud, len(baud_rank)),
            discovery_frame_priority(candidate),
        ),
    )


def console_progress(message: str) -> None:
    """Print a timestamped live progress message."""
    prefix = f"{time.strftime('%H:%M:%S')} "
    width = max(20, TERMINAL_COLUMNS - len(prefix))
    lines = str(message).splitlines() or [""]
    for line_index, line in enumerate(lines):
        wrapped = textwrap.wrap(
            line,
            width=width,
            break_long_words=True,
            replace_whitespace=False,
        ) or [""]
        for wrap_index, part in enumerate(wrapped):
            lead = prefix if line_index == 0 and wrap_index == 0 else " " * len(prefix)
            print(f"{lead}{part}", flush=True)


def print_progress_legend() -> None:
    """Print a concise explanation of live progress fields."""
    print("OPERATOR LEGEND")
    print("  [ITEM/TOTAL SETTING]  CURRENT SERIAL SETTING.")
    print("  TEST X/Y              SEND/READ PASS FOR THAT SETTING.")
    print("  WRITING A/B           BYTES WRITTEN TO INPUT PORT.")
    print("  RECEIVED=N            BYTES READ FROM OUTPUT PORT.")
    print("  CLEARED=N             OLD OUTPUT REMOVED BEFORE SEND.")
    print("  QUIET=S/T             OUTPUT QUIET TIME AND LIMIT.")
    print("  PASS GOOD PART FAIL STALE ERROR  QUICK RESULT.")
    print("  SCORE                 0-100 MATCH CONFIDENCE.")
    print("  SCAN TIME             ELAPSED, LEFT, FINISH TIME.")
    print("  CTRL+C                OPERATOR BREAK MENU.")


def received_length(received: bytearray, lock: threading.Lock) -> int:
    """Return the current received byte count under lock."""
    with lock:
        return len(received)


def drain_output_until_quiet(
    out_serial: Any,
    quiet_seconds: float,
    max_seconds: float,
    max_bytes: int,
    progress_interval: float,
    progress: ProgressCallback | None,
    prefix: str,
    logger: logging.Logger,
) -> DrainResult:
    """Drain output bytes until the output side is quiet or a limit is reached."""
    started = time.monotonic()
    last_data_time = started
    next_progress_at = started + max(progress_interval, 0.1)
    bytes_drained = 0

    if max_seconds <= 0:
        return DrainResult(0, 0.0, True, "disabled", None)

    if progress:
        progress(
            f"{prefix}: CLEAR OLD OUTPUT UNTIL {quiet_seconds:.1f}S QUIET "
            f"(MAX {max_seconds:.1f}S, MAX {max_bytes} BYTES)"
        )

    try:
        out_serial.reset_input_buffer()
        while True:
            now = time.monotonic()
            if bytes_drained >= max_bytes:
                elapsed = now - started
                return DrainResult(bytes_drained, elapsed, False, "max-bytes", None)
            if now - started >= max_seconds:
                elapsed = now - started
                return DrainResult(bytes_drained, elapsed, False, "timeout", None)

            waiting = getattr(out_serial, "in_waiting", 0)
            remaining = max_bytes - bytes_drained
            read_size = min(max(int(waiting), 1), 4096, remaining)
            chunk = out_serial.read(read_size)
            now = time.monotonic()

            if chunk:
                bytes_drained += len(chunk)
                last_data_time = now
            elif now - last_data_time >= quiet_seconds:
                elapsed = now - started
                return DrainResult(bytes_drained, elapsed, True, "quiet", None)

            if progress and now >= next_progress_at:
                silence = max(0.0, now - last_data_time)
                progress(
                    f"{prefix}: CLEARING OLD OUTPUT CLEARED={bytes_drained}, "
                    f"QUIET={silence:.1f}/{quiet_seconds:.1f}S"
                )
                next_progress_at = now + max(progress_interval, 0.1)
    except Exception as exc:  # pyserial raises driver-specific subclasses.
        logger.debug("pre-drain failed: %s", exc)
        elapsed = time.monotonic() - started
        return DrainResult(bytes_drained, elapsed, False, "error", str(exc))


def execute_burst(
    in_serial: Any,
    out_serial: Any,
    settings: SerialSettings | DualSerialSettings,
    payload: ProbePayload,
    burst_index: int,
    burst_total: int,
    candidate_index: int,
    candidate_total: int,
    read_timeout: float,
    completion_quiet: float,
    settle_ms: int,
    progress_interval: float,
    no_pre_drain: bool,
    pre_drain_timeout: float,
    pre_drain_quiet: float,
    max_drain_bytes: int,
    logger: logging.Logger,
    progress: ProgressCallback | None,
) -> TrialResult:
    """Send one probe burst and score the received bytes."""
    started = time.monotonic()
    expected = payload.data
    received = bytearray()
    reader_errors: list[str] = []
    stop_event = threading.Event()
    writer_done = threading.Event()
    received_lock = threading.Lock()
    last_data_time = time.monotonic()
    progress_interval = max(progress_interval, 0.1)
    completion_quiet = max(0.01, min(completion_quiet, read_timeout))
    prefix = (
        f"[{candidate_index:04d}/{candidate_total:04d} {settings.label()}] "
        f"TEST {burst_index}/{burst_total}"
    )
    chunk_size = write_chunk_size(settings)

    setup_started = time.monotonic()
    if progress:
        progress(f"{prefix}: RESET BUFFERS; PAUSE {settle_ms} MS")
    reset_serial_buffers(in_serial)
    reset_serial_buffers(out_serial)
    time.sleep(settle_ms / 1000.0)
    setup_elapsed = time.monotonic() - setup_started

    drain = DrainResult(0, 0.0, True, "disabled", None)
    if not no_pre_drain:
        drain = drain_output_until_quiet(
            out_serial=out_serial,
            quiet_seconds=pre_drain_quiet,
            max_seconds=pre_drain_timeout,
            max_bytes=max_drain_bytes,
            progress_interval=progress_interval,
            progress=progress,
            prefix=prefix,
            logger=logger,
        )
        if drain.quiet:
            if progress:
                progress(
                    f"{prefix}: OUTPUT QUIET; CLEARED={drain.bytes_drained} "
                    f"BYTES IN {format_duration(drain.elapsed_sec)}"
                )
        else:
            error = (
                "OUTPUT DID NOT GO QUIET BEFORE TEST SEND "
                f"(REASON={drain.reason.upper()}, CLEARED={drain.bytes_drained})"
            )
            if drain.error:
                error = f"{error}: {drain.error}"
            logger.info("%s: %s", prefix, error)
            if progress:
                progress(
                    f"{prefix}: RESULT STALE SCORE=0.00; SEND SKIPPED; "
                    f"CLEARED={drain.bytes_drained}, REASON={drain.reason.upper()}"
                )
            empty_score = score_received(expected, b"")
            elapsed = time.monotonic() - started
            return TrialResult(
                burst_index=burst_index,
                bytes_sent=0,
                bytes_received=0,
                bytes_drained_before=drain.bytes_drained,
                drain_status=drain.reason,
                score=0.0,
                metrics=empty_score.metrics,
                status="stale-output",
                error=error,
                elapsed_sec=elapsed,
                timing=timing_with_other(
                    elapsed,
                    setup_elapsed,
                    drain.elapsed_sec,
                    0.0,
                    0.0,
                ),
                received_preview_ascii="",
                received_preview_hex="",
            )

    def reader() -> None:
        nonlocal last_data_time
        while not stop_event.is_set():
            try:
                waiting = getattr(out_serial, "in_waiting", 0)
                read_size = min(max(int(waiting), 1), 4096)
                chunk = out_serial.read(read_size)
            except Exception as exc:  # pyserial raises driver-specific subclasses.
                reader_errors.append(str(exc))
                stop_event.set()
                break

            now = time.monotonic()
            if chunk:
                with received_lock:
                    received.extend(chunk)
                last_data_time = now
                continue

            if writer_done.is_set():
                with received_lock:
                    snapshot = bytes(received)
                complete = receive_completion_detected(snapshot, expected)
                quiet_target = completion_quiet if complete else read_timeout
                if (now - last_data_time) >= quiet_target:
                    stop_event.set()
                    break

    reader_thread = threading.Thread(target=reader, name="serial-probe-reader", daemon=True)
    reader_thread.start()

    bytes_sent = 0
    write_error: str | None = None
    write_started = time.monotonic()
    if progress:
        estimated = estimated_transmit_seconds(settings, len(expected))
        progress(
            f"{prefix}: SEND {len(expected)} BYTES ON {settings.label()} "
            f"(CHUNK={chunk_size}, ABOUT {format_duration(estimated)})"
        )
    try:
        next_progress_at = time.monotonic() + progress_interval
        while bytes_sent < len(expected):
            chunk = expected[bytes_sent : bytes_sent + chunk_size]
            written = in_serial.write(chunk)
            if written is None:
                written = len(chunk)
            if written <= 0:
                raise RuntimeError("serial write returned zero bytes")
            bytes_sent += int(written)
            now = time.monotonic()
            if progress and now >= next_progress_at:
                percent = (bytes_sent / len(expected)) * 100.0
                progress(
                    f"{prefix}: WRITING {bytes_sent}/{len(expected)} BYTES "
                    f"({percent:5.1f}%), "
                    f"RECEIVED={received_length(received, received_lock)}, "
                    f"CLEARED={drain.bytes_drained}"
                )
                next_progress_at = now + progress_interval
        if progress:
            progress(f"{prefix}: FLUSH OUTPUT BYTES")
        in_serial.flush()
    except Exception as exc:  # pyserial raises driver-specific subclasses.
        write_error = str(exc)
        logger.debug("burst %s write failed: %s", burst_index, write_error)
    finally:
        writer_done.set()
    write_elapsed = time.monotonic() - write_started

    if progress:
        progress(
            f"{prefix}: WRITE DONE, SENT={bytes_sent}; "
            f"WAIT {read_timeout:.2f}S QUIET ON {settings.label()}"
        )

    wait_deadline = time.monotonic() + max(read_timeout + (settle_ms / 1000.0) + 2.0, 2.0)
    next_wait_progress_at = time.monotonic() + progress_interval
    read_wait_started = time.monotonic()
    while reader_thread.is_alive():
        reader_thread.join(timeout=0.2)
        now = time.monotonic()
        if progress and now >= next_wait_progress_at:
            silence = max(0.0, now - last_data_time)
            with received_lock:
                snapshot = bytes(received)
            complete = receive_completion_detected(snapshot, expected)
            quiet_target = completion_quiet if complete else read_timeout
            progress(
                f"{prefix}: READING RECEIVED={received_length(received, received_lock)} "
                f"BYTES, QUIET={silence:.2f}/{quiet_target:.2f}S"
            )
            next_wait_progress_at = now + progress_interval
        if now >= wait_deadline:
            stop_event.set()
            break

    if reader_thread.is_alive():
        stop_event.set()
        reader_thread.join(timeout=1.0)
    read_wait_elapsed = time.monotonic() - read_wait_started

    elapsed = time.monotonic() - started
    received_bytes = bytes(received)
    score = score_received(expected, received_bytes)

    error = write_error or (reader_errors[0] if reader_errors else None)
    if error:
        status = "error"
    elif not received_bytes:
        status = "no-data"
    elif score.score >= 99.0:
        status = "exact"
    elif score.score >= 90.0:
        status = "strong"
    elif score.score >= 50.0:
        status = "partial"
    else:
        status = "weak"

    if progress:
        indicator = result_indicator(score.score, status, error)
        progress(
            f"{prefix}: RESULT {indicator} SCORE={score.score:.2f} ({status.upper()}); "
            f"SENT={bytes_sent}, RECEIVED={len(received_bytes)}, "
            f"CLEARED={drain.bytes_drained}, "
            f"EXACT={score.metrics.exact_byte_match_ratio:.3f}, "
            f"LINES={score.metrics.line_integrity_ratio:.3f}, "
            f"ASCII={score.metrics.printable_ascii_ratio:.3f}"
        )

    return TrialResult(
        burst_index=burst_index,
        bytes_sent=bytes_sent,
        bytes_received=len(received_bytes),
        bytes_drained_before=drain.bytes_drained,
        drain_status=drain.reason,
        score=score.score,
        metrics=score.metrics,
        status=status,
        error=error,
        elapsed_sec=elapsed,
        timing=timing_with_other(
            elapsed,
            setup_elapsed,
            drain.elapsed_sec,
            write_elapsed,
            read_wait_elapsed,
        ),
        received_preview_ascii=preview_ascii(received_bytes),
        received_preview_hex=preview_hex(received_bytes),
    )


def aggregate_metrics(trials: Sequence[TrialResult]) -> ScoreMetrics:
    """Aggregate burst metrics into one candidate metric object."""
    if not trials:
        return ScoreMetrics(
            exact_byte_match_ratio=0.0,
            line_integrity_ratio=0.0,
            missing_bytes=0,
            extra_bytes=0,
            printable_ascii_ratio=0.0,
            length_ratio=0.0,
            start_marker_present=False,
            end_marker_present=False,
        )

    return ScoreMetrics(
        exact_byte_match_ratio=statistics.fmean(
            trial.metrics.exact_byte_match_ratio for trial in trials
        ),
        line_integrity_ratio=statistics.fmean(
            trial.metrics.line_integrity_ratio for trial in trials
        ),
        missing_bytes=sum(trial.metrics.missing_bytes for trial in trials),
        extra_bytes=sum(trial.metrics.extra_bytes for trial in trials),
        printable_ascii_ratio=statistics.fmean(
            trial.metrics.printable_ascii_ratio for trial in trials
        ),
        length_ratio=statistics.fmean(trial.metrics.length_ratio for trial in trials),
        start_marker_present=all(trial.metrics.start_marker_present for trial in trials),
        end_marker_present=all(trial.metrics.end_marker_present for trial in trials),
    )


def aggregate_candidate_result(
    index: int,
    total: int,
    settings: SerialSettings | DualSerialSettings,
    trials: list[TrialResult],
    elapsed_sec: float,
    opening_error: str | None = None,
) -> CandidateResult:
    """Aggregate one candidate's burst trials into a candidate result."""
    if opening_error is not None:
        return CandidateResult(
            index=index,
            total=total,
            settings=settings,
            bytes_sent=0,
            bytes_received=0,
            bytes_drained_before=0,
            score=0.0,
            repeatability=0.0,
            status="error",
            error=opening_error,
            elapsed_sec=elapsed_sec,
            timing=timing_with_other(elapsed_sec, elapsed_sec, 0.0, 0.0, 0.0),
            metrics=aggregate_metrics([]),
            trials=[],
        )

    scores = [trial.score for trial in trials]
    score = statistics.fmean(scores) if scores else 0.0
    high_quality_trials = sum(1 for trial in trials if trial.score >= 98.0)
    repeatability = high_quality_trials / max(len(trials), 1)
    errors = [trial.error for trial in trials if trial.error]
    stale_trials = [trial for trial in trials if trial.status == "stale-output"]
    metrics = aggregate_metrics(trials)

    if stale_trials:
        status = "stale-output"
    elif errors:
        status = "error"
    elif not trials or sum(trial.bytes_received for trial in trials) == 0:
        status = "no-data"
    elif score >= 99.0 and repeatability == 1.0:
        status = "exact"
    elif score >= 90.0:
        status = "strong"
    elif score >= 50.0:
        status = "partial"
    else:
        status = "weak"

    timing = combine_timing_breakdowns([trial.timing for trial in trials])
    timing = TimingBreakdown(
        open_setup_sec=timing.open_setup_sec,
        drain_sec=timing.drain_sec,
        write_sec=timing.write_sec,
        read_wait_sec=timing.read_wait_sec,
        other_sec=timing.other_sec + max(0.0, elapsed_sec - timing_total(timing)),
    )
    return CandidateResult(
        index=index,
        total=total,
        settings=settings,
        bytes_sent=sum(trial.bytes_sent for trial in trials),
        bytes_received=sum(trial.bytes_received for trial in trials),
        bytes_drained_before=sum(trial.bytes_drained_before for trial in trials),
        score=score,
        repeatability=repeatability,
        status=status,
        error="; ".join(errors) if errors else None,
        elapsed_sec=elapsed_sec,
        timing=timing,
        metrics=metrics,
        trials=trials,
    )


def run_candidate(
    serial_module: Any,
    index: int,
    total: int,
    settings: SerialSettings,
    options: ScanOptions,
    payload: ProbePayload,
    logger: logging.Logger,
    progress: ProgressCallback | None = None,
) -> CandidateResult:
    """Run one candidate by reopening both ports and executing bursts."""
    started = time.monotonic()
    trials: list[TrialResult] = []
    effective_timing = effective_discovery_timing(options, settings, payload.byte_count)
    logger.info("candidate %s/%s: %s", index, total, settings.label())
    if progress:
        per_burst = estimated_transmit_seconds(settings, payload.byte_count)
        total_estimate = per_burst * options.bursts
        progress(border_line(PROGRESS_WIDTH))
        progress(
            bordered_text(
                f"SETTING {index}/{total}  {settings.label()}",
                PROGRESS_WIDTH,
            )
        )
        progress(border_line(PROGRESS_WIDTH))
        progress(
            f"[{index:04d}/{total:04d} {settings.label()}] TESTING | "
            f"TEST={payload.byte_count} BYTES, COUNT={options.bursts}, "
            f"SEND={format_duration(per_burst)}/TEST "
            f"({format_duration(total_estimate)} TOTAL), "
            f"READ={effective_timing.read_timeout:.2f}S"
        )
        progress(
            f"[{index:04d}/{total:04d} {settings.label()}] OPEN OUT {options.out_port} "
            f"AND IN {options.in_port}"
        )

    try:
        open_started = time.monotonic()
        with open_serial_port(
            serial_module, options.out_port, settings, effective_timing.read_timeout
        ) as out_serial:
            with open_serial_port(
                serial_module, options.in_port, settings, effective_timing.read_timeout
            ) as in_serial:
                open_elapsed = time.monotonic() - open_started
                if progress:
                    progress(
                        f"[{index:04d}/{total:04d} {settings.label()}] PORTS OPEN; "
                        "START TESTS"
                    )
                for burst_index in range(1, options.bursts + 1):
                    trial = execute_burst(
                        in_serial=in_serial,
                        out_serial=out_serial,
                        settings=settings,
                        payload=payload,
                        burst_index=burst_index,
                        burst_total=options.bursts,
                        candidate_index=index,
                        candidate_total=total,
                        read_timeout=effective_timing.read_timeout,
                        completion_quiet=effective_timing.completion_quiet,
                        settle_ms=effective_timing.settle_ms,
                        progress_interval=options.progress_interval,
                        no_pre_drain=options.no_pre_drain,
                        pre_drain_timeout=effective_timing.pre_drain_timeout,
                        pre_drain_quiet=effective_timing.pre_drain_quiet,
                        max_drain_bytes=options.max_drain_bytes,
                        logger=logger,
                        progress=progress,
                    )
                    trials.append(trial)
                    logger.debug(
                        "candidate %s burst %s: sent=%s recv=%s drained=%s score=%.2f status=%s error=%s",
                        index,
                        burst_index,
                        trial.bytes_sent,
                        trial.bytes_received,
                        trial.bytes_drained_before,
                        trial.score,
                        trial.status,
                        trial.error,
                    )
    except Exception as exc:
        elapsed = time.monotonic() - started
        logger.exception("candidate %s failed before trials", index)
        if progress:
            progress(
                f"[{index:04d}/{total:04d} {settings.label()}] "
                f"ERROR OPEN/RUN: {exc}"
            )
        result = aggregate_candidate_result(
            index=index,
            total=total,
            settings=settings,
            trials=[],
            elapsed_sec=elapsed,
            opening_error=str(exc),
        )
        logger.info(
            "candidate %s/%s timing: total=%.3fs open_setup=%.3fs drain=%.3fs write=%.3fs read_wait=%.3fs other=%.3fs",
            index,
            total,
            result.elapsed_sec,
            result.timing.open_setup_sec,
            result.timing.drain_sec,
            result.timing.write_sec,
            result.timing.read_wait_sec,
            result.timing.other_sec,
        )
        return result

    elapsed = time.monotonic() - started
    result = aggregate_candidate_result(
        index=index,
        total=total,
        settings=settings,
        trials=trials,
        elapsed_sec=elapsed,
    )
    result = dataclasses.replace(
        result,
        timing=TimingBreakdown(
            open_setup_sec=result.timing.open_setup_sec + open_elapsed,
            drain_sec=result.timing.drain_sec,
            write_sec=result.timing.write_sec,
            read_wait_sec=result.timing.read_wait_sec,
            other_sec=max(
                0.0,
                result.elapsed_sec
                - (
                    result.timing.open_setup_sec
                    + open_elapsed
                    + result.timing.drain_sec
                    + result.timing.write_sec
                    + result.timing.read_wait_sec
                ),
            ),
        ),
    )
    logger.info(
        "candidate %s/%s timing: total=%.3fs open_setup=%.3fs drain=%.3fs write=%.3fs read_wait=%.3fs other=%.3fs",
        index,
        total,
        result.elapsed_sec,
        result.timing.open_setup_sec,
        result.timing.drain_sec,
        result.timing.write_sec,
        result.timing.read_wait_sec,
        result.timing.other_sec,
    )
    return result


def run_dual_candidate(
    serial_module: Any,
    index: int,
    total: int,
    settings: DualSerialSettings,
    options: ScanOptions,
    payload: ProbePayload,
    logger: logging.Logger,
    progress: ProgressCallback | None = None,
) -> CandidateResult:
    """Run one dual-bank candidate using independent input/output settings."""
    started = time.monotonic()
    trials: list[TrialResult] = []
    input_settings = settings.input_settings
    output_settings = settings.output_settings
    effective_timing = effective_discovery_timing(
        options,
        input_settings,
        payload.byte_count,
    )
    logger.info("dual candidate %s/%s: %s", index, total, settings.label())
    if progress:
        per_burst = estimated_transmit_seconds(input_settings, payload.byte_count)
        total_estimate = per_burst * options.bursts
        progress(border_line(PROGRESS_WIDTH))
        progress(
            bordered_text(
                f"DUAL {index}/{total}",
                PROGRESS_WIDTH,
            )
        )
        progress(border_line(PROGRESS_WIDTH))
        progress(
            f"[{index:04d}/{total:04d} {settings.label()}] TESTING | "
            f"TEST={payload.byte_count} BYTES, COUNT={options.bursts}, "
            f"SEND={format_duration(per_burst)}/TEST "
            f"({format_duration(total_estimate)} TOTAL), "
            f"READ={effective_timing.read_timeout:.2f}S"
        )
        progress(
            f"[{index:04d}/{total:04d} {settings.label()}] "
            f"OPEN OUT {options.out_port} AS {output_settings.label()} "
            f"AND IN {options.in_port} AS {input_settings.label()}"
        )

    try:
        open_started = time.monotonic()
        with open_serial_port(
            serial_module,
            options.out_port,
            output_settings,
            effective_timing.read_timeout,
        ) as out_serial:
            with open_serial_port(
                serial_module,
                options.in_port,
                input_settings,
                effective_timing.read_timeout,
            ) as in_serial:
                open_elapsed = time.monotonic() - open_started
                if progress:
                    progress(
                        f"[{index:04d}/{total:04d} {settings.label()}] "
                        "PORTS OPEN; START TESTS"
                    )
                for burst_index in range(1, options.bursts + 1):
                    trial = execute_burst(
                        in_serial=in_serial,
                        out_serial=out_serial,
                        settings=settings,
                        payload=payload,
                        burst_index=burst_index,
                        burst_total=options.bursts,
                        candidate_index=index,
                        candidate_total=total,
                        read_timeout=effective_timing.read_timeout,
                        completion_quiet=effective_timing.completion_quiet,
                        settle_ms=effective_timing.settle_ms,
                        progress_interval=options.progress_interval,
                        no_pre_drain=options.no_pre_drain,
                        pre_drain_timeout=effective_timing.pre_drain_timeout,
                        pre_drain_quiet=effective_timing.pre_drain_quiet,
                        max_drain_bytes=options.max_drain_bytes,
                        logger=logger,
                        progress=progress,
                    )
                    trials.append(trial)
                    logger.debug(
                        "dual candidate %s burst %s: sent=%s recv=%s drained=%s score=%.2f status=%s error=%s",
                        index,
                        burst_index,
                        trial.bytes_sent,
                        trial.bytes_received,
                        trial.bytes_drained_before,
                        trial.score,
                        trial.status,
                        trial.error,
                    )
    except Exception as exc:
        elapsed = time.monotonic() - started
        logger.exception("dual candidate %s failed before trials", index)
        if progress:
            progress(
                f"[{index:04d}/{total:04d} {settings.label()}] "
                f"ERROR OPEN/RUN: {exc}"
            )
        result = aggregate_candidate_result(
            index=index,
            total=total,
            settings=settings,
            trials=[],
            elapsed_sec=elapsed,
            opening_error=str(exc),
        )
        logger.info(
            "dual candidate %s/%s timing: total=%.3fs open_setup=%.3fs drain=%.3fs write=%.3fs read_wait=%.3fs other=%.3fs",
            index,
            total,
            result.elapsed_sec,
            result.timing.open_setup_sec,
            result.timing.drain_sec,
            result.timing.write_sec,
            result.timing.read_wait_sec,
            result.timing.other_sec,
        )
        return result

    elapsed = time.monotonic() - started
    result = aggregate_candidate_result(
        index=index,
        total=total,
        settings=settings,
        trials=trials,
        elapsed_sec=elapsed,
    )
    result = dataclasses.replace(
        result,
        timing=TimingBreakdown(
            open_setup_sec=result.timing.open_setup_sec + open_elapsed,
            drain_sec=result.timing.drain_sec,
            write_sec=result.timing.write_sec,
            read_wait_sec=result.timing.read_wait_sec,
            other_sec=max(
                0.0,
                result.elapsed_sec
                - (
                    result.timing.open_setup_sec
                    + open_elapsed
                    + result.timing.drain_sec
                    + result.timing.write_sec
                    + result.timing.read_wait_sec
                ),
            ),
        ),
    )
    logger.info(
        "dual candidate %s/%s timing: total=%.3fs open_setup=%.3fs drain=%.3fs write=%.3fs read_wait=%.3fs other=%.3fs",
        index,
        total,
        result.elapsed_sec,
        result.timing.open_setup_sec,
        result.timing.drain_sec,
        result.timing.write_sec,
        result.timing.read_wait_sec,
        result.timing.other_sec,
    )
    return result


def result_sort_key(result: CandidateResult) -> tuple[float, float, float, int]:
    """Return descending sort key fields for ranking candidates."""
    return (
        result.score,
        result.metrics.line_integrity_ratio,
        result.metrics.exact_byte_match_ratio,
        result.bytes_received,
    )


def ranked_top_results(
    results: Sequence[CandidateResult],
    top: int,
) -> list[CandidateResult]:
    """Return top non-zero results, including all perfect scores when tied."""
    ranked = [
        result
        for result in sorted(results, key=result_sort_key, reverse=True)
        if result.score > 0.0
    ]
    perfect_count = sum(1 for result in ranked if result.score >= PERFECT_SCORE)
    display_count = max(top, perfect_count)
    return ranked[:display_count]


def setup_logging(log_file: Path) -> logging.Logger:
    """Configure file logging and return the scan logger."""
    log_file.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("serial_probe")
    logger.setLevel(logging.DEBUG)
    for existing_handler in list(logger.handlers):
        logger.removeHandler(existing_handler)
        existing_handler.close()
    handler = logging.FileHandler(log_file, encoding="utf-8")
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    )
    logger.addHandler(handler)
    return logger


def bytes_to_jsonable(value: bytes | bytearray | memoryview) -> str | dict[str, Any]:
    """Return a JSON-safe representation of raw bytes."""
    data = bytes(value)
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        text = ""
    if text and all(char in "\r\n\t" or 32 <= ord(char) <= 126 for char in text):
        return text
    return {
        "encoding": "base64",
        "byte_count": len(data),
        "data": base64.b64encode(data).decode("ascii"),
    }


def dataclass_to_jsonable(value: Any) -> Any:
    """Convert values into JSON-serializable plain objects."""
    if dataclasses.is_dataclass(value):
        return {
            key: dataclass_to_jsonable(item)
            for key, item in dataclasses.asdict(value).items()
        }
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes_to_jsonable(value)
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (list, tuple)):
        return [dataclass_to_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {key: dataclass_to_jsonable(item) for key, item in value.items()}
    return value


def candidate_table_lines(
    results: Sequence[CandidateResult],
    top: int,
    title: str,
) -> list[str]:
    """Return compact ranked result table lines for text reports."""
    ranked = ranked_top_results(results, top)
    lines = [
        title,
        border_line(REPORT_WIDTH),
        "RK SCORE   BAUD MODE FLOW       SENT   READ  CLR  EXCT LINE RESULT",
        border_line(REPORT_WIDTH),
    ]
    for rank, result in enumerate(ranked, start=1):
        mode = (
            f"{result.settings.data_bits}"
            f"{result.settings.parity_code()}"
            f"{result.settings.stop_bits}"
        )
        indicator = result_indicator(result.score, result.status, result.error)
        lines.append(
            f"{rank:>2} "
            f"{result.score:>5.1f} "
            f"{result.settings.baud:>6} "
            f"{mode:<4} "
            f"{result.settings.flow_control.upper():<8} "
            f"{result.bytes_sent:>6} "
            f"{result.bytes_received:>6} "
            f"{result.bytes_drained_before:>4} "
            f"{result.metrics.exact_byte_match_ratio:>4.2f} "
            f"{result.metrics.line_integrity_ratio:>4.2f} "
            f"{indicator}"
        )
    lines.append(border_line(REPORT_WIDTH))
    if not ranked:
        lines.append("NO NON-ZERO RESULTS.")
    return lines


def result_count_summary(results: Sequence[CandidateResult]) -> str:
    """Return PASS/GOOD/PARTIAL/FAIL/STALE/ERROR counts."""
    counts: dict[str, int] = {}
    for result in results:
        indicator = result_indicator(result.score, result.status, result.error)
        counts[indicator] = counts.get(indicator, 0) + 1
    return ", ".join(
        f"{name}={counts.get(name, 0)}"
        for name in ("PASS", "GOOD", "PARTIAL", "FAIL", "STALE", "ERROR")
    )


def scan_type_label(scan_mode: str) -> str:
    """Return the operator-facing scan type label."""
    return "QUICK" if scan_mode == "quick" else "FULL"


def text_report_summary_lines(
    results: Sequence[CandidateResult],
    metadata: dict[str, Any],
) -> list[str]:
    """Return summary and interpretation lines for one report section."""
    ranked = sorted(results, key=result_sort_key, reverse=True)
    best = ranked[0] if ranked else None
    tied = top_tied_results(results)
    lines = [
        f"SETTINGS TESTED: {len(results)}/{metadata.get('phase2_candidate_count', len(results))}",
        f"RESULT COUNTS:   {result_count_summary(results)}",
        f"FINDING:         {confidence_summary(best, len(tied))}",
    ]
    if best is not None:
        lines.append(f"BEST SETTING:    {best.settings.label()}")
        lines.append(
            f"BEST SCORE:      {best.score:.2f} "
            f"SENT/READ={best.bytes_sent}/{best.bytes_received}"
        )
    if len(tied) > 1:
        lines.append(
            f"AMBIGUOUS:       {len(tied)} SETTINGS TIED WITHIN "
            f"{TIE_SCORE_TOLERANCE:.1f} POINTS."
        )
        lines.append("ACTION:          DO NOT TREAT ROW 1 AS UNIQUE.")
    elif is_recommendable_result(best):
        lines.append("ACTION:          RECORD THIS AS THE LIKELY SWITCH RESULT.")
    elif has_any_signal(results):
        lines.append("ACTION:          PARTIAL RESULT ONLY; REPEAT OR CHECK WIRING.")
    else:
        lines.append("ACTION:          NO WORKING SETTING FOUND FOR THIS SWITCH STATE.")
    return lines


def write_text_report(
    path: Path,
    metadata: dict[str, Any],
    results: Sequence[CandidateResult],
    validation_results: Sequence[CandidateResult] | None = None,
    flow_validation_results: Sequence[FlowControlValidationResult] | None = None,
) -> None:
    """Append one compact old-school text report entry."""
    path.parent.mkdir(parents=True, exist_ok=True)
    validation_results = [] if validation_results is None else list(validation_results)
    flow_validation_results = (
        []
        if flow_validation_results is None
        else list(flow_validation_results)
    )
    options = metadata.get("options", {})
    exploratory = metadata.get("exploratory_mode", {})
    phase0 = exploratory.get("phase0_baud_liveness", {})
    baud_focus = exploratory.get("baud_focus", {})
    created = dt.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")
    switch_note = str(metadata.get("switch_note") or "").strip()
    lines = [
        "",
        border_line(REPORT_WIDTH),
        bordered_text("SERIAL PROBE RUN REPORT", REPORT_WIDTH),
        border_line(REPORT_WIDTH),
        f"APPENDED:        {created}",
        f"STARTED UTC:     {metadata.get('started_at', '')}",
        f"COM PATH:        {metadata.get('in_port')} -> BUFFER -> {metadata.get('out_port')}",
        f"SWITCH NOTE:     {switch_note if switch_note else '(NOT ENTERED)'}",
        f"SCAN TYPE:       {scan_type_label(str(metadata.get('scan_mode', '')))}",
        f"BAUD RANGE:      {options.get('min_baud')}..{options.get('max_baud')}",
        f"TEST BYTES:      {options.get('payload_bytes')} X {options.get('bursts')}",
        f"REPORT STATUS:   {metadata.get('recommendation_status')}",
        "",
        "ASSUMPTIONS:",
        "  BUFFER IS PHYSICALLY BETWEEN THE TWO COM PORTS.",
        "  BOTH BUFFER SWITCH BANKS ARE SET THE SAME WAY FOR THIS RUN.",
        "  THE PROGRAM SETS BOTH PC COM PORTS; DEVICE MANAGER DEFAULTS ARE NOT USED.",
        "",
    ]
    if metadata.get("operator_break_action"):
        lines.extend(
            [
                f"OPERATOR BREAK:  {metadata.get('operator_break_stage', 'SCAN')}",
                f"BREAK ACTION:    {str(metadata.get('operator_break_action')).upper()}",
                "",
            ]
        )
    if phase0:
        alive = ",".join(str(baud) for baud in phase0.get("alive_bauds", []))
        lines.extend(
            [
                f"PHASE 0 BAUDS:   {alive if alive else '(NONE)'}",
                f"PHASE 0 FALLBACK:{' YES' if phase0.get('fallback_to_all_bauds') else ' NO'}",
            ]
        )
    if baud_focus:
        lines.append(
            "QUICK BAUD FOCUS: "
            + (
                f"ENGAGED {baud_focus.get('focused_baud')}"
                if baud_focus.get("engaged")
                else "NOT ENGAGED"
            )
        )
    lines.extend(
        [
            "",
            "PHASE 1 SUMMARY:",
            *text_report_summary_lines(results, metadata),
            "",
            *candidate_table_lines(results, int(metadata.get("top", 15)), "PHASE 1 TOP RESULTS"),
        ]
    )
    if validation_results:
        validation_metadata = dict(metadata)
        validation_metadata["phase2_candidate_count"] = len(validation_results)
        lines.extend(
            [
                "",
                "VALIDATION SUMMARY:",
                *text_report_summary_lines(validation_results, validation_metadata),
                "",
                *candidate_table_lines(
                    validation_results,
                    min(int(metadata.get("top", 15)), len(validation_results)),
                    "VALIDATION TOP RESULTS",
                ),
            ]
        )
    if flow_validation_results:
        lines.extend(
            [
                "",
                *flow_validation_report_lines(flow_validation_results),
            ]
        )
    lines.extend(
        [
            "",
            "INTERPRETATION NOTES:",
            "  BAUD RATE IS USUALLY THE MOST RELIABLE FIELD FROM THIS TEST.",
            "  7/8 DATA BITS, PARITY, STOP BITS, AND FLOW CONTROL CAN TIE IF THE BUFFER",
            "  OR TEST DATA DOES NOT FORCE THOSE FEATURES TO MATTER.",
            "  TRUST FLOW CONTROL ONLY AFTER FLOW VALIDATION OR A STRESS/MEMORY TEST.",
            border_line(REPORT_WIDTH),
        ]
    )
    with path.open("a", encoding="utf-8") as report_file:
        report_file.write("\n".join(lines) + "\n")


def format_progress(result: CandidateResult) -> str:
    """Return one console progress line for a candidate result."""
    status = result.status.upper() if not result.error else f"{result.status.upper()}: {result.error[:60]}"
    indicator = result_indicator(result.score, result.status, result.error)
    return (
        f"[{result.index:04d}/{result.total:04d}] RESULT {indicator} "
        f"{result.settings.label():32s} "
        f"SENT={result.bytes_sent:7d} READ={result.bytes_received:7d} "
        f"CLR={result.bytes_drained_before:7d} "
        f"SCORE={result.score:6.2f} {status}"
    )


def format_scan_eta(
    completed: int,
    total: int,
    started_monotonic: float,
    now_monotonic: float | None = None,
    clock_now: dt.datetime | None = None,
) -> str:
    """Return a live elapsed/remaining/finish estimate for the scan."""
    now = time.monotonic() if now_monotonic is None else now_monotonic
    elapsed = max(0.0, now - started_monotonic)
    completed = max(0, min(completed, total))
    remaining_count = max(total - completed, 0)
    if completed <= 0:
        return (
            f"SCAN TIME {completed:04d}/{total:04d}: "
            f"ELAPSED={format_duration(elapsed)} LEFT=? FINISH=?"
        )

    average = elapsed / completed
    remaining_seconds = average * remaining_count
    finish = format_finish_clock(remaining_seconds, clock_now)
    return (
        f"SCAN TIME {completed:04d}/{total:04d}: "
        f"ELAPSED={format_duration(elapsed)} "
        f"AVG={format_duration(average)}/SET "
        f"LEFT={format_duration(remaining_seconds)} "
        f"FINISH={finish}"
    )


def print_ranked_table(
    results: Sequence[CandidateResult],
    top: int,
    report_title: str = "SERIAL PROBE FINAL REPORT",
) -> None:
    """Print a ranked non-zero table to stdout."""
    ranked = ranked_top_results(results, top)
    print()
    print_report_title(report_title)
    print("TOP OBSERVED RESULTS (NON-ZERO SCORES)")
    print(border_line(REPORT_WIDTH))
    print(
        "RK SCORE   BAUD MODE FLOW       SENT   READ  CLR  EXCT LINE RESULT"
    )
    print(border_line(REPORT_WIDTH))
    for rank, result in enumerate(ranked, start=1):
        mode = (
            f"{result.settings.data_bits}"
            f"{result.settings.parity_code()}"
            f"{result.settings.stop_bits}"
        )
        indicator = result_indicator(result.score, result.status, result.error)
        print(
            f"{rank:>2} "
            f"{result.score:>5.1f} "
            f"{result.settings.baud:>6} "
            f"{mode:<4} "
            f"{result.settings.flow_control.upper():<8} "
            f"{result.bytes_sent:>6} "
            f"{result.bytes_received:>6} "
            f"{result.bytes_drained_before:>4} "
            f"{result.metrics.exact_byte_match_ratio:>4.2f} "
            f"{result.metrics.line_integrity_ratio:>4.2f} "
            f"{indicator}"
        )
    print(border_line(REPORT_WIDTH))


def is_recommendable_result(result: CandidateResult | None) -> bool:
    """Return True when a result is strong enough to call a recommendation."""
    if result is None or result.error:
        return False
    if result.status in {"error", "no-data", "stale-output", "partial-write", "weak"}:
        return False
    return result.score >= RECOMMENDATION_MIN_SCORE


def is_top_match_result(result: CandidateResult | None) -> bool:
    """Return True when a result is strong enough to pause the scan."""
    if not is_recommendable_result(result):
        return False
    if result.status != "exact":
        return False
    return (
        result.score >= TOP_MATCH_MIN_SCORE
        and result.repeatability >= 1.0
        and result.metrics.missing_bytes == 0
        and result.metrics.extra_bytes == 0
    )


def top_tied_results(results: Sequence[CandidateResult]) -> list[CandidateResult]:
    """Return recommendable results that are effectively tied for first place."""
    ranked = sorted(results, key=result_sort_key, reverse=True)
    best = ranked[0] if ranked else None
    if not is_recommendable_result(best):
        return []
    return [
        result
        for result in ranked
        if is_recommendable_result(result)
        and (best.score - result.score) <= TIE_SCORE_TOLERANCE
    ]


def has_any_signal(results: Sequence[CandidateResult]) -> bool:
    """Return True when any result had enough data quality to be useful."""
    return any(
        not result.error
        and result.status not in {"error", "no-data", "stale-output", "partial-write"}
        and result.score >= 50.0
        for result in results
    )


def scan_recommendation_status(results: Sequence[CandidateResult]) -> str:
    """Return a machine-readable recommendation status for reports."""
    ranked = sorted(results, key=result_sort_key, reverse=True)
    best = ranked[0] if ranked else None
    if best is None:
        return "no-candidates"
    if is_recommendable_result(best):
        if len(top_tied_results(results)) > 1:
            return "multiple-candidates"
        return "recommended"
    if has_any_signal(results):
        return "partial-only"
    return "no-working-setting"


def confidence_summary(result: CandidateResult | None, tied_count: int = 0) -> str:
    """Return a short interpretation of the best result."""
    if result is None:
        return "NO SETTINGS TESTED."
    if result.status == "stale-output":
        return "NO MATCH. OUTPUT WAS NOT QUIET BEFORE TEST."
    if result.error:
        return "NO MATCH. BEST ROW ENDED WITH ERROR."
    if tied_count > 1:
        return "MULTIPLE TOP SETTINGS. REVIEW BEFORE SETTING SWITCHES."
    if is_recommendable_result(result) and result.score >= 99.0 and result.repeatability >= 1.0:
        return "LIKELY CORRECT."
    if is_recommendable_result(result):
        return "STRONG MATCH. VERIFY BEFORE SETTING SWITCHES."
    if result.score >= 50.0:
        return "PARTIAL MATCH ONLY. NOT RELIABLE."
    return "NO CONFIDENT MATCH. CHECK CABLES, PORTS, FLOW CONTROL."


def print_result_details(result: CandidateResult) -> None:
    """Print switch-setting and score details for one scan result."""
    print(f"    BAUD RATE:          {result.settings.baud}")
    print(f"    DATA BITS:          {result.settings.data_bits}")
    print(
        f"    PARITY:             "
        f"{parity_name(result.settings.parity)} ({result.settings.parity_code()})"
    )
    print(f"    STOP BITS:          {result.settings.stop_bits}")
    print(f"    FLOW CONTROL:       {flow_control_name(result.settings.flow_control)}")
    print_wrapped_value("    SETTING:            ", result.settings.label())
    print(
        f"    RESULT:             "
        f"{result_indicator(result.score, result.status, result.error)}"
    )
    print(f"    SCORE:              {result.score:.2f}/100")
    print(f"    REPEAT:             {result.repeatability:.3f}")
    print(f"    SENT/READ:          {result.bytes_sent}/{result.bytes_received}")
    print(f"    OLD BYTES CLEAR:    {result.bytes_drained_before}")
    print(f"    EXACT RATIO:        {result.metrics.exact_byte_match_ratio:.3f}")
    print(f"    LINE RATIO:         {result.metrics.line_integrity_ratio:.3f}")
    print(f"    ASCII RATIO:        {result.metrics.printable_ascii_ratio:.3f}")
    print(f"    MISSING/EXTRA:      {result.metrics.missing_bytes}/{result.metrics.extra_bytes}")
    if result.status == "stale-output":
        print("    NOTE:               OUTPUT NEVER WENT QUIET.")
        if result.error:
            print_wrapped_value("    DETAIL:             ", result.error)
    elif result.error:
        print_wrapped_value("    ERROR:              ", result.error)
    elif result.metrics.extra_bytes > result.bytes_sent:
        print("    NOTE:               EXTRA OUTPUT/BACKLOG PRESENT.")


def print_tied_results(results: Sequence[CandidateResult]) -> None:
    """Print a compact table of tied top candidate settings."""
    print("    RK   SCORE   SETTING")
    for rank, result in enumerate(results, start=1):
        print(f"    {rank:>4}  {result.score:>5.1f}   {result.settings.label()}")


def ask_continue_after_top_match(result: CandidateResult) -> bool:
    """Ask the operator whether to continue after a top match."""
    print()
    print(border_line(REPORT_WIDTH))
    print(bordered_text("TOP MATCH FOUND", REPORT_WIDTH))
    print(border_line(REPORT_WIDTH))
    print_wrapped_value("    SETTING:            ", result.settings.label())
    print(f"    SCORE:              {result.score:.2f}/100")
    print("    CONTINUE TO LOOK FOR POSSIBLE TIES.")
    print("    ENTER N TO END NOW AND WRITE REPORT.")
    print(border_line(REPORT_WIDTH))
    return prompt_yes_no("CONTINUE SCAN", True)


def prompt_operator_break_action(target: str = "SCAN") -> str:
    """Ask what to do after Ctrl+C/BREAK during a running test."""
    target = target.upper()
    print()
    print(border_line(REPORT_WIDTH))
    print(bordered_text("OPERATOR BREAK", REPORT_WIDTH))
    print(border_line(REPORT_WIDTH))
    print(f"  {target} PAUSED.")
    print(f"  1 RESUME {target}")
    print(f"  2 END {target}, WRITE REPORT")
    print("  3 RETURN TO MAIN MENU AFTER REPORT")
    print("  0 QUIT AFTER REPORT")
    print(border_line(REPORT_WIDTH))
    while True:
        try:
            choice = read_operator_input("ENTER SELECTION [1]: ").lstrip("\ufeff").strip().lower()
        except EOFError:
            return "resume"
        if choice == "":
            return "resume"
        if choice in {"1", "c", "continue", "r", "resume"}:
            return "resume"
        if choice in {"2", "e", "end", "report", "write"}:
            return "report"
        if choice in {"3", "m", "menu", "main"}:
            return "menu"
        if choice in {"0", "q", "quit", "exit"}:
            return "quit"
        print("ENTER 1, 2, 3, OR 0.")


def parity_name(parity: str) -> str:
    """Return a clear parity label for reports."""
    return {
        "none": "NONE",
        "even": "EVEN",
        "odd": "ODD",
        "mark": "MARK",
        "space": "SPACE",
    }[parity]


def flow_control_name(flow_control: str) -> str:
    """Return a clear flow-control label for reports."""
    return {
        "none": "NONE",
        "xon/xoff": "XON/XOFF",
        "rts/cts": "RTS/CTS",
        "dsr/dtr": "DSR/DTR",
    }[flow_control]


def aggregate_result_timing(results: Sequence[CandidateResult]) -> TimingBreakdown:
    """Return total timing across candidate results."""
    return combine_timing_breakdowns([result.timing for result in results])


def slowest_stage_labels(timing: TimingBreakdown, count: int = 3) -> list[str]:
    """Return labels for the slowest timing stages."""
    stages = [
        ("OPEN/SETUP", timing.open_setup_sec),
        ("DRAIN", timing.drain_sec),
        ("WRITE", timing.write_sec),
        ("READ WAIT", timing.read_wait_sec),
        ("OTHER", timing.other_sec),
    ]
    stages.sort(key=lambda item: item[1], reverse=True)
    return [
        f"{name}={format_duration(seconds).upper()}"
        for name, seconds in stages[:count]
        if seconds > 0.0
    ]


def print_scan_summary(
    results: Sequence[CandidateResult],
    total_candidates: int,
    elapsed_sec: float,
    early_stopped: bool,
    top: int,
    early_stop_reason: str | None = None,
) -> None:
    """Print a concise human-readable scan summary."""
    ranked = sorted(results, key=result_sort_key, reverse=True)
    best = ranked[0] if ranked else None
    tied = top_tied_results(results)
    counts: dict[str, int] = {}
    for result in results:
        indicator = result_indicator(result.score, result.status, result.error)
        counts[indicator] = counts.get(indicator, 0) + 1

    print()
    print_report_title("SCAN SUMMARY")
    print(f"  RUN TIME:             {format_duration(elapsed_sec)}")
    print(f"  SETTINGS TESTED:      {len(results)}/{total_candidates}")
    if results:
        print(
            "  AVG SETTING TIME:     "
            f"{format_duration(elapsed_sec / max(len(results), 1))}"
        )
        slow_labels = slowest_stage_labels(aggregate_result_timing(results))
        if slow_labels:
            print("  SLOWEST STAGES:       " + ", ".join(slow_labels))
    print(f"  ENDED EARLY:          {'YES' if early_stopped else 'NO'}")
    print(
        "  RESULT COUNTS:        "
        + ", ".join(
            f"{name}={counts.get(name, 0)}"
            for name in ("PASS", "GOOD", "PARTIAL", "FAIL", "STALE", "ERROR")
        )
    )
    print(f"  TOP ROWS:             {len(ranked_top_results(results, top))}")
    print(f"  FINDING:              {confidence_summary(best, len(tied))}")
    if early_stopped:
        if early_stop_reason == "operator-ended-after-top-match":
            print("  NOTE:                 OPERATOR ENDED AFTER TOP MATCH.")
            print("                        LATER SETTINGS WERE NOT TESTED FOR TIES.")
        else:
            print("  NOTE:                 OPERATOR BREAK ENDED SCAN EARLY.")
            print("                        UNTESTED SETTINGS MAY STILL MATCH.")

    if best is None:
        return

    print()
    print(border_line(REPORT_WIDTH))
    if len(tied) > 1:
        print(bordered_text("MULTIPLE TOP SETTINGS FOUND", REPORT_WIDTH))
        print(border_line(REPORT_WIDTH))
        print(
            "    MORE THAN ONE SETTING MATCHED WITHIN "
            f"{TIE_SCORE_TOLERANCE:.1f} SCORE POINTS."
        )
        print("    DO NOT TREAT ROW 1 AS THE ONLY POSSIBLE SWITCH SETTING.")
        print("    USE A LARGER TEST MESSAGE OR REPEAT THESE SETTINGS.")
        print()
        print_tied_results(ranked_top_results(tied, top))
        print(border_line(REPORT_WIDTH))
        return

    if is_recommendable_result(best):
        print(bordered_text("RECOMMENDED SWITCH SETTING", REPORT_WIDTH))
        print(border_line(REPORT_WIDTH))
        print_result_details(best)
        if early_stopped:
            if early_stop_reason == "operator-ended-after-top-match":
                print("    NOTE:               OPERATOR ENDED AFTER THIS TOP MATCH.")
                print("                        POSSIBLE LATER TIES WERE NOT TESTED.")
            else:
                print("    NOTE:               OPERATOR BREAK ENDED SCAN EARLY.")
                print("                        UNTESTED SETTINGS MAY STILL MATCH.")
        print(border_line(REPORT_WIDTH))
        return

    if has_any_signal(results):
        print(bordered_text("NO RELIABLE SETTING FOUND", REPORT_WIDTH))
        print(border_line(REPORT_WIDTH))
        print("    BEST ROW WAS ONLY A PARTIAL RESULT.")
        print("    DO NOT USE IT AS THE BUFFER SWITCH SETTING YET.")
        print("    RESET/CLEAR BUFFER, CHECK CABLES AND FLOW CONTROL, RUN AGAIN.")
    else:
        print(bordered_text("NO WORKING SETTING FOUND", REPORT_WIDTH))
        print(border_line(REPORT_WIDTH))
        print("    CURRENT BUFFER SWITCH SETUP DID NOT PASS ANY TEST.")
        print("    DO NOT USE TOP ROW AS A SWITCH RECOMMENDATION.")
        print("    POSSIBLE CAUSES: UNUSED SWITCH POSITION, DISABLED PORT,")
        print("    HELD OUTPUT, WRONG CABLE, WRONG COM PORT, FLOW-CONTROL HOLD.")

    print()
    print(border_line(REPORT_WIDTH))
    print(bordered_text("BEST OBSERVED RESULT ONLY", REPORT_WIDTH))
    print(border_line(REPORT_WIDTH))
    print_result_details(best)
    print(border_line(REPORT_WIDTH))


def default_report_paths() -> tuple[Path, Path]:
    """Return append-only text report and debug log paths."""
    return Path("serial_probe_report.txt"), Path("serial_probe_debug.log")


def default_scan_options() -> ScanOptions:
    """Return practical defaults for the interactive scan."""
    default_text_report, default_log = default_report_paths()
    return ScanOptions(
        in_port="COM1",
        out_port="COM5",
        min_baud=75,
        max_baud=38400,
        payload_bytes=DEFAULT_PAYLOAD_BYTES,
        read_timeout=DEFAULT_READ_TIMEOUT,
        settle_ms=DEFAULT_SETTLE_MS,
        top=15,
        text_report=default_text_report,
        log_file=default_log,
        switch_note="",
        bursts=DEFAULT_BURSTS,
        progress_interval=DEFAULT_PROGRESS_INTERVAL,
        no_pre_drain=False,
        pre_drain_timeout=DEFAULT_PRE_DRAIN_TIMEOUT,
        pre_drain_quiet=DEFAULT_PRE_DRAIN_QUIET,
        max_drain_bytes=DEFAULT_MAX_DRAIN_BYTES,
        turbo_discovery_enabled=TURBO_DISCOVERY_ENABLED_DEFAULT,
        ask_on_top_match=False,
        auto_validate_top_matches=True,
        validate_size_1_bytes=8 * 1024,
        validate_size_2_tie_bytes=16 * 1024,
        auto_validate_flow_control=True,
        flow_validate_size_bytes=FLOW_VALIDATE_PAYLOAD_BYTES,
        baud_focus_enabled=BAUD_FOCUS_ENABLED_DEFAULT,
        baud_focus_strong_score_threshold=BAUD_FOCUS_STRONG_SCORE_THRESHOLD,
        baud_focus_lead_gap_threshold=BAUD_FOCUS_LEAD_GAP_THRESHOLD,
        baud_focus_min_strong_results=BAUD_FOCUS_MIN_STRONG_RESULTS,
        baud_focus_min_samples=BAUD_FOCUS_MIN_SAMPLES,
    )


def validate_options(options: ScanOptions) -> None:
    """Validate scan options before launching hardware I/O."""
    ensure_distinct_ports(options.in_port, options.out_port)
    if options.payload_bytes < minimum_payload_size():
        raise ValueError(f"TEST MESSAGE SIZE MUST BE AT LEAST {minimum_payload_size()} BYTES")
    if options.read_timeout <= 0:
        raise ValueError("READ WAIT MUST BE POSITIVE")
    if options.settle_ms < 0:
        raise ValueError("OPEN PAUSE CANNOT BE NEGATIVE")
    if options.top <= 0:
        raise ValueError("TOP ROW COUNT MUST BE POSITIVE")
    if options.bursts <= 0:
        raise ValueError("TEST COUNT MUST BE POSITIVE")
    if options.progress_interval <= 0:
        raise ValueError("SCREEN UPDATE INTERVAL MUST BE POSITIVE")
    if options.pre_drain_timeout < 0:
        raise ValueError("CLEAR OUTPUT TIME CANNOT BE NEGATIVE")
    if options.pre_drain_quiet <= 0:
        raise ValueError("QUIET TIME MUST BE POSITIVE")
    if options.max_drain_bytes <= 0:
        raise ValueError("MAX CLEAR BYTES MUST BE POSITIVE")
    if options.validate_size_1_bytes < minimum_payload_size():
        raise ValueError(f"VALIDATE SIZE 1 MUST BE AT LEAST {minimum_payload_size()} BYTES")
    if options.validate_size_2_tie_bytes < 0:
        raise ValueError("VALIDATE SIZE 2 ON TIE CANNOT BE NEGATIVE")
    if (
        options.validate_size_2_tie_bytes > 0
        and options.validate_size_2_tie_bytes < minimum_payload_size()
    ):
        raise ValueError(f"VALIDATE SIZE 2 MUST BE 0 OR AT LEAST {minimum_payload_size()} BYTES")
    if options.flow_validate_size_bytes < minimum_payload_size():
        raise ValueError(f"FLOW VALIDATE SIZE MUST BE AT LEAST {minimum_payload_size()} BYTES")
    if not 0.0 <= options.baud_focus_strong_score_threshold <= 100.0:
        raise ValueError("QUICK BAUD FOCUS SCORE MUST BE 0..100")
    if options.baud_focus_lead_gap_threshold < 0.0:
        raise ValueError("QUICK BAUD FOCUS GAP CANNOT BE NEGATIVE")
    if options.baud_focus_min_strong_results <= 0:
        raise ValueError("QUICK BAUD FOCUS GOOD COUNT MUST BE POSITIVE")
    if options.baud_focus_min_samples <= 0:
        raise ValueError("QUICK BAUD FOCUS SAMPLE COUNT MUST BE POSITIVE")
    available_bauds(options.min_baud, options.max_baud)


def estimate_scan_wire_seconds(options: ScanOptions) -> float:
    """Estimate total data-sending time for the selected scan."""
    candidates = generate_candidates(options.min_baud, options.max_baud)
    return sum(
        estimated_transmit_seconds(candidate, options.payload_bytes) * options.bursts
        for candidate in candidates
    )


def estimate_scan_overhead_seconds(options: ScanOptions) -> float:
    """Estimate non-wire wait time assuming quiet output."""
    candidates = generate_candidates(options.min_baud, options.max_baud)
    total = 0.0
    for candidate in candidates:
        timing = effective_discovery_timing(options, candidate, options.payload_bytes)
        per_burst = timing.read_timeout + (timing.settle_ms / 1000.0)
        if not options.no_pre_drain:
            per_burst += timing.pre_drain_quiet
        total += per_burst * options.bursts
    return total


def phase0_baseline_settings(baud: int) -> SerialSettings:
    """Return the fixed baseline settings used by the baud liveness sweep."""
    return SerialSettings(
        baud=baud,
        data_bits=PHASE0_BASELINE_DATA_BITS,
        parity=PHASE0_BASELINE_PARITY,
        stop_bits=PHASE0_BASELINE_STOP_BITS,
        flow_control=PHASE0_BASELINE_FLOW_CONTROL,
    )


def phase0_minimum_payload_size() -> int:
    """Return the smallest structural payload usable by Phase 0."""
    start = b"<<<SERIAL_PROBE_BEGIN PHASE0>>>\r\n"
    end = b"<<<SERIAL_PROBE_END PHASE0>>>\r\n"
    return len(start) + len(make_probe_line(1, "LIVE", "")) + len(end)


def phase0_payload_bytes() -> int:
    """Return the fixed internal Phase 0 liveness payload size."""
    return max(PHASE0_PAYLOAD_BYTES, phase0_minimum_payload_size())


def generate_phase0_payload(payload_bytes: int | None = None) -> ProbePayload:
    """Generate a compact structural probe payload for baud liveness."""
    byte_count = phase0_payload_bytes() if payload_bytes is None else payload_bytes
    byte_count = max(byte_count, phase0_minimum_payload_size())
    start = b"<<<SERIAL_PROBE_BEGIN PHASE0>>>\r\n"
    end = b"<<<SERIAL_PROBE_END PHASE0>>>\r\n"
    empty_line = make_probe_line(1, "LIVE", "")
    data_len = byte_count - len(start) - len(empty_line) - len(end)
    if data_len < 0:
        raise ValueError("PHASE 0 PAYLOAD IS TOO SMALL")
    line = make_probe_line(1, "LIVE", repeated_ascii_pattern(1, data_len))
    body = start + line
    payload = body + end
    if len(payload) != byte_count:
        raise AssertionError(
            f"phase 0 payload produced {len(payload)} bytes, expected {byte_count}"
        )
    body_hash = fnv1a32(body)
    return ProbePayload(
        data=payload,
        line_count=1,
        byte_count=len(payload),
        body_hash=f"{body_hash:08X}",
    )


def phase0_fixed_settings_label() -> str:
    """Return a compact description of fixed Phase 0 liveness settings."""
    return (
        f"{phase0_payload_bytes()} BYTES X {PHASE0_BURSTS}, "
        "8E1 FLOW=NONE, "
        f"READ={PHASE0_READ_TIMEOUT:.2f}S, "
        f"PAUSE={PHASE0_SETTLE_MS}MS, "
        f"CLEAR={PHASE0_PRE_DRAIN_TIMEOUT:.2f}S/"
        f"{PHASE0_PRE_DRAIN_QUIET:.2f}S"
    )


def phase0_scan_options(options: ScanOptions) -> ScanOptions:
    """Return fixed internal options for the Phase 0 baud liveness sweep."""
    return dataclasses.replace(
        options,
        payload_bytes=phase0_payload_bytes(),
        read_timeout=PHASE0_READ_TIMEOUT,
        settle_ms=PHASE0_SETTLE_MS,
        bursts=PHASE0_BURSTS,
        progress_interval=PHASE0_PROGRESS_INTERVAL,
        no_pre_drain=False,
        pre_drain_timeout=PHASE0_PRE_DRAIN_TIMEOUT,
        pre_drain_quiet=PHASE0_PRE_DRAIN_QUIET,
        max_drain_bytes=PHASE0_MAX_DRAIN_BYTES,
        turbo_discovery_enabled=False,
        ask_on_top_match=False,
    )


def buffer_purge_settings(options: ScanOptions) -> list[SerialSettings]:
    """Return output-side settings used to flush a stateful printer buffer."""
    return [
        phase0_baseline_settings(baud)
        for baud in scan_bauds(options.min_baud, options.max_baud)
    ]


def buffer_purge_banner(reason: str, settings_count: int) -> None:
    """Print a vintage-style buffer purge banner."""
    print()
    print_report_title("BUFFER PURGE")
    print_wrapped_value("  REASON:              ", reason)
    print("  DEVICE TYPE:          SERIAL FIFO/RAM PRINTER BUFFER")
    print(f"  ASSUMED CAPACITY:     {BUFFER_PURGE_CAPACITY_BYTES} BYTES")
    print(f"  OUTPUT BAUD TRIES:    {settings_count}")
    print(
        "  METHOD:               READ BUFFER OUTPUT UNTIL QUIET "
        "BEFORE SENDING NEW TEST DATA."
    )
    print(border_line(REPORT_WIDTH))


def purge_buffer_output(
    serial_module: Any,
    options: ScanOptions,
    logger: logging.Logger,
    reason: str,
    settings_list: Sequence[SerialSettings] | None = None,
) -> DrainResult:
    """Drain a stateful serial printer buffer before a test stage."""
    if not BUFFER_PURGE_ENABLED:
        return DrainResult(0, 0.0, True, "disabled", None)
    settings = list(settings_list) if settings_list is not None else buffer_purge_settings(options)
    if not settings:
        return DrainResult(0, 0.0, True, "no-settings", None)

    buffer_purge_banner(reason, len(settings))
    started = time.monotonic()
    total_drained = 0
    errors: list[str] = []
    for index, setting in enumerate(settings, start=1):
        prefix = f"[PURGE {index:02d}/{len(settings):02d} OUT {setting.label()}]"
        try:
            with open_serial_port(
                serial_module,
                options.out_port,
                setting,
                max(BUFFER_PURGE_QUIET_SECONDS, 0.5),
            ) as out_serial:
                drain = drain_output_until_quiet(
                    out_serial=out_serial,
                    quiet_seconds=BUFFER_PURGE_QUIET_SECONDS,
                    max_seconds=BUFFER_PURGE_PER_BAUD_MAX_SECONDS,
                    max_bytes=max(options.max_drain_bytes, BUFFER_PURGE_CAPACITY_BYTES * 2),
                    progress_interval=BUFFER_PURGE_PROGRESS_INTERVAL,
                    progress=console_progress,
                    prefix=prefix,
                    logger=logger,
                )
        except Exception as exc:
            error = str(exc)
            logger.info("buffer purge failed opening %s: %s", setting.label(), error)
            errors.append(error)
            continue

        total_drained += drain.bytes_drained
        logger.info(
            "buffer purge %s drained=%s quiet=%s reason=%s elapsed=%.3fs error=%s",
            setting.label(),
            drain.bytes_drained,
            drain.quiet,
            drain.reason,
            drain.elapsed_sec,
            drain.error,
        )
        if drain.error:
            errors.append(drain.error)
    elapsed = time.monotonic() - started
    quiet = not errors
    result = DrainResult(
        bytes_drained=total_drained,
        elapsed_sec=elapsed,
        quiet=quiet,
        reason="quiet" if quiet else "error",
        error="; ".join(errors) if errors else None,
    )
    print()
    print_report_title("BUFFER PURGE COMPLETE")
    print(f"  BYTES DRAINED:        {result.bytes_drained}")
    print(f"  RUN TIME:             {format_duration(result.elapsed_sec)}")
    print(f"  STATUS:               {'READY' if result.quiet else 'CHECK LOG'}")
    if result.error:
        print_wrapped_value("  DETAIL:               ", result.error)
    print(border_line(REPORT_WIDTH))
    return result


def phase0_extra_byte_limit(expected_byte_count: int) -> int:
    """Return the tolerated extra-byte limit for Phase 0 liveness."""
    ratio_limit = int(expected_byte_count * PHASE0_MAX_EXTRA_BYTES_RATIO)
    return max(PHASE0_MAX_EXTRA_BYTES, ratio_limit)


def classify_phase0_liveness(
    result: CandidateResult,
    expected_byte_count: int,
) -> Phase0LivenessDecision:
    """Return a conservative boolean liveness decision for one baud result."""
    if result.error or result.status == "error":
        return Phase0LivenessDecision(False, "SERIAL ERROR")
    if result.status == "partial-write":
        return Phase0LivenessDecision(False, "PARTIAL WRITE")
    if result.status == "stale-output":
        return Phase0LivenessDecision(False, "OUTPUT NOT QUIET")
    if result.status == "no-data" or result.bytes_received <= 0:
        return Phase0LivenessDecision(False, "NO DATA")
    if result.bytes_sent < expected_byte_count:
        return Phase0LivenessDecision(False, "INCOMPLETE SEND")
    if result.metrics.extra_bytes > phase0_extra_byte_limit(expected_byte_count):
        return Phase0LivenessDecision(False, "EXTRA OUTPUT")
    if result.metrics.line_integrity_ratio < PHASE0_MIN_LINE_INTEGRITY:
        return Phase0LivenessDecision(False, "NO VALID PROBE LINE")
    if not (
        result.metrics.start_marker_present
        or result.metrics.end_marker_present
    ):
        return Phase0LivenessDecision(False, "NO PROBE MARKER")
    if result.score < PHASE0_MIN_ALIVE_SCORE:
        return Phase0LivenessDecision(False, f"LOW SCORE {result.score:.1f}")
    return Phase0LivenessDecision(True, "VALID PROBE STRUCTURE")


def baud_liveness_result_from_candidate(
    result: CandidateResult,
    decision: Phase0LivenessDecision,
) -> BaudLivenessResult:
    """Return the compact Phase 0 report row for one candidate result."""
    return BaudLivenessResult(
        baud=result.settings.baud,
        alive=decision.alive,
        reason=decision.reason,
        settings=result.settings,
        score=result.score,
        status=result.status,
        error=result.error,
        bytes_sent=result.bytes_sent,
        bytes_received=result.bytes_received,
        bytes_drained_before=result.bytes_drained_before,
        elapsed_sec=result.elapsed_sec,
        metrics=result.metrics,
    )


def empty_baud_liveness_report(
    candidate_count: int,
    fallback_reason: str | None = None,
) -> BaudLivenessReport:
    """Return a not-run Phase 0 liveness report for metadata."""
    return BaudLivenessReport(
        ran=False,
        tested_bauds=[],
        alive_bauds=[],
        fallback_to_all_bauds=True,
        fallback_reason=fallback_reason,
        candidate_count_before=candidate_count,
        candidate_count_after=candidate_count,
        elapsed_sec=0.0,
        results=[],
    )


def candidates_after_phase0_liveness(
    candidates: Sequence[SerialSettings],
    report: BaudLivenessReport,
) -> list[SerialSettings]:
    """Return quick exploratory candidates after the Phase 0 baud gate."""
    if report.fallback_to_all_bauds:
        return list(candidates)
    alive = set(report.alive_bauds)
    return [candidate for candidate in candidates if candidate.baud in alive]


def exploratory_payload_bytes() -> int:
    """Return the fixed internal exploratory payload size."""
    return max(EXPLORATORY_PAYLOAD_BYTES, minimum_payload_size())


def exploratory_fixed_settings_label() -> str:
    """Return a compact description of fixed exploratory scan settings."""
    return (
        f"{exploratory_payload_bytes()} BYTES X {EXPLORATORY_BURSTS}, "
        f"READ={EXPLORATORY_READ_TIMEOUT:.1f}S, "
        f"PAUSE={EXPLORATORY_SETTLE_MS}MS, "
        f"CLEAR={EXPLORATORY_PRE_DRAIN_TIMEOUT:.1f}S/"
        f"{EXPLORATORY_PRE_DRAIN_QUIET:.1f}S"
    )


def baud_focus_settings_label(options: ScanOptions) -> str:
    """Return a compact operator label for baud focus settings."""
    if not options.baud_focus_enabled:
        return "OFF"
    return (
        "ON, "
        f"SCORE>={options.baud_focus_strong_score_threshold:.1f}, "
        f"GAP>={options.baud_focus_lead_gap_threshold:.1f}, "
        f"GOOD>={options.baud_focus_min_strong_results}, "
        f"SAMPLES>={options.baud_focus_min_samples}"
    )


def exploratory_scan_options(options: ScanOptions) -> ScanOptions:
    """Return fixed internal options for quick exploratory mode."""
    return dataclasses.replace(
        options,
        payload_bytes=exploratory_payload_bytes(),
        read_timeout=EXPLORATORY_READ_TIMEOUT,
        settle_ms=EXPLORATORY_SETTLE_MS,
        bursts=EXPLORATORY_BURSTS,
        progress_interval=EXPLORATORY_PROGRESS_INTERVAL,
        no_pre_drain=False,
        pre_drain_timeout=EXPLORATORY_PRE_DRAIN_TIMEOUT,
        pre_drain_quiet=EXPLORATORY_PRE_DRAIN_QUIET,
        max_drain_bytes=EXPLORATORY_MAX_DRAIN_BYTES,
        turbo_discovery_enabled=False,
        ask_on_top_match=False,
    )


def empty_baud_focus_report(
    enabled: bool,
    disabled_reason: str | None = None,
) -> BaudFocusReport:
    """Return a no-focus report with optional disabled reason."""
    return BaudFocusReport(
        enabled=enabled,
        engaged=False,
        focused_baud=None,
        deferred_other_bauds=False,
        deferred_candidate_count=0,
        deferred_bauds=[],
        engage_reason=None,
        release_reason=None,
        disabled_reason=disabled_reason,
        tested_before_engage=None,
    )


def group_candidates_by_baud(
    candidates: Sequence[SerialSettings],
) -> tuple[list[int], dict[int, list[SerialSettings]]]:
    """Group candidate settings by baud while preserving scan baud order."""
    baud_order: list[int] = []
    grouped: dict[int, list[SerialSettings]] = {}
    for candidate in candidates:
        if candidate.baud not in grouped:
            grouped[candidate.baud] = []
            baud_order.append(candidate.baud)
        grouped[candidate.baud].append(candidate)
    return baud_order, grouped


def next_unseen_candidate(
    candidates: Sequence[SerialSettings],
    tested: set[SerialSettings],
) -> SerialSettings | None:
    """Return the next candidate not already tested."""
    for candidate in candidates:
        if candidate not in tested:
            return candidate
    return None


def result_blocks_baud_focus(result: CandidateResult) -> str | None:
    """Return a reason when a result disables baud focus narrowing."""
    if result.status == "stale-output":
        return "STALE OUTPUT SEEN"
    if result.error or result.status in {"error", "partial-write"}:
        return "ERROR SEEN"
    return None


def baud_focus_stats(
    results: Sequence[CandidateResult],
    baud_order: Sequence[int],
    strong_score_threshold: float,
) -> list[BaudFocusStats]:
    """Return per-baud sample counts and strong-hit totals."""
    samples = {baud: 0 for baud in baud_order}
    strong = {baud: 0 for baud in baud_order}
    best = {baud: 0.0 for baud in baud_order}
    for result in results:
        baud = result.settings.baud
        if baud not in samples:
            continue
        samples[baud] += 1
        best[baud] = max(best[baud], result.score)
        if (
            not result.error
            and result.status not in {"error", "no-data", "stale-output", "partial-write"}
            and result.score >= strong_score_threshold
        ):
            strong[baud] += 1
    return [
        BaudFocusStats(
            baud=baud,
            samples=samples[baud],
            strong_results=strong[baud],
            best_score=best[baud],
        )
        for baud in baud_order
    ]


def select_baud_focus(
    results: Sequence[CandidateResult],
    baud_order: Sequence[int],
    grouped_candidates: dict[int, list[SerialSettings]],
    options: ScanOptions,
) -> BaudFocusDecision:
    """Return a focused baud only after all confidence gates pass."""
    if not options.baud_focus_enabled:
        return BaudFocusDecision(None, None, "QUICK BAUD FOCUS OFF")
    if len(baud_order) < 2:
        return BaudFocusDecision(None, None, "ONLY ONE BAUD SELECTED")

    stats = baud_focus_stats(
        results,
        baud_order,
        options.baud_focus_strong_score_threshold,
    )
    eligible = [
        stat
        for stat in stats
        if stat.samples >= min(
            options.baud_focus_min_samples,
            len(grouped_candidates.get(stat.baud, [])),
        )
        and stat.strong_results >= options.baud_focus_min_strong_results
        and stat.best_score >= options.baud_focus_strong_score_threshold
    ]
    if not eligible:
        return BaudFocusDecision(None, None, None)

    eligible.sort(
        key=lambda stat: (stat.best_score, stat.strong_results, stat.samples),
        reverse=True,
    )
    best = eligible[0]
    next_best_score = max(
        (stat.best_score for stat in stats if stat.baud != best.baud),
        default=0.0,
    )
    lead_gap = best.best_score - next_best_score
    if lead_gap < options.baud_focus_lead_gap_threshold:
        return BaudFocusDecision(None, None, None)

    reason = (
        f"SCORE={best.best_score:.1f} GAP={lead_gap:.1f} "
        f"GOOD={best.strong_results} SAMPLES={best.samples}"
    )
    return BaudFocusDecision(best.baud, reason, None)


def is_exploratory_signal(result: CandidateResult) -> bool:
    """Return True when an exploratory result is useful enough for narrowing."""
    if result.error:
        return False
    if result.status in {"error", "no-data", "stale-output", "partial-write"}:
        return False
    return result.score >= EXPLORATORY_MIN_NARROW_SCORE


def is_phase2_viable_signal(result: CandidateResult) -> bool:
    """Return True when an exploratory row has any usable life signal."""
    if result.error:
        return False
    return result.status in PHASE2_VIABLE_SIGNAL_STATUSES


def select_exploratory_candidates(
    results: Sequence[CandidateResult],
    all_candidates: Sequence[SerialSettings],
    elapsed_sec: float,
    baud_focus: BaudFocusReport,
    phase0_liveness: BaudLivenessReport,
) -> ExploratorySelection:
    """Select a conservative full-scan shortlist from exploratory results."""
    ranked_results = sorted(results, key=result_sort_key, reverse=True)
    ranked_nonzero = [result for result in ranked_results if result.score > 0.0]
    notes: list[str] = []
    fallback_reason: str | None = None
    cutoff_score: float | None = None
    shortlist_results: list[CandidateResult] = []
    narrowed_candidates: list[SerialSettings] = []
    truncated = False
    viable_candidates = [
        result.settings for result in ranked_results if is_phase2_viable_signal(result)
    ]

    stale_count = sum(1 for result in results if result.status == "stale-output")
    no_data_count = sum(1 for result in results if result.status == "no-data")
    error_count = sum(1 for result in results if result.error or result.status == "error")
    if stale_count:
        notes.append(f"STALE={stale_count}; OUTPUT WAS NOT QUIET FOR THOSE SETTINGS.")
    if no_data_count:
        notes.append(f"NO DATA={no_data_count}; NOTHING USEFUL WAS READ.")
    if error_count:
        notes.append(f"ERROR={error_count}; CHECK LOG FOR DRIVER DETAILS.")

    if not results:
        fallback_reason = "NO EXPLORATORY SETTINGS WERE TESTED."
    elif not ranked_nonzero:
        fallback_reason = "NO EXPLORATORY SIGNAL WAS OBSERVED."
    else:
        eligible = [result for result in ranked_results if is_exploratory_signal(result)]
        best = ranked_nonzero[0]
        if not eligible:
            fallback_reason = (
                f"BEST EXPLORATORY SCORE {best.score:.1f} "
                f"IS BELOW {EXPLORATORY_MIN_NARROW_SCORE:.1f}."
            )
        else:
            best_eligible = eligible[0]
            cutoff_score = max(
                EXPLORATORY_MIN_NARROW_SCORE,
                best_eligible.score - EXPLORATORY_SCORE_TOLERANCE,
            )
            within_cutoff = [
                result for result in eligible if result.score >= cutoff_score
            ]
            shortlist_results = within_cutoff[:EXPLORATORY_SHORTLIST_LIMIT]
            truncated = len(within_cutoff) > len(shortlist_results)
            seed_frames = {
                (
                    result.settings.baud,
                    result.settings.data_bits,
                    result.settings.parity,
                    result.settings.stop_bits,
                )
                for result in shortlist_results
            }
            expanded_settings = {
                SerialSettings(baud, data_bits, parity, stop_bits, flow)
                for baud, data_bits, parity, stop_bits in seed_frames
                for flow in FLOW_CONTROLS
            }
            narrowed_candidates = [
                candidate for candidate in all_candidates if candidate in expanded_settings
            ]
            tied_count = sum(
                1
                for result in within_cutoff
                if abs(result.score - best_eligible.score) <= TIE_SCORE_TOLERANCE
            )
            if tied_count > 1:
                notes.append(
                    f"AMBIGUOUS={tied_count}; TOP SCORES ARE CLOSE TOGETHER."
                )
            if best_eligible.score < RECOMMENDATION_MIN_SCORE:
                notes.append("CONFIDENCE IS LOW; USE ONLY AS A FULL-SCAN HINT.")
            if best_eligible.metrics.extra_bytes > best_eligible.bytes_sent:
                notes.append("BEST ROW HAD EXTRA OUTPUT; BACKLOG OR NOISE MAY EXIST.")
            if truncated:
                notes.append(
                    f"SHORTLIST LIMITED TO TOP {EXPLORATORY_SHORTLIST_LIMIT} "
                    "EXPLORATORY ROWS."
                )
            notes.append(
                "FLOW CONTROL WAS EXPANDED FOR EACH PROMISING FRAME SETTING."
            )

    if fallback_reason:
        if viable_candidates:
            notes.append(
                f"SIGNAL-ONLY PHASE 2 AVAILABLE: {len(viable_candidates)} SETTINGS."
            )
        else:
            notes.append("FULL SCAN WILL USE ALL SELECTED SETTINGS.")

    return ExploratorySelection(
        results=list(results),
        ranked_results=ranked_results,
        shortlist_results=shortlist_results,
        narrowed_candidates=narrowed_candidates,
        elapsed_sec=elapsed_sec,
        fallback_reason=fallback_reason,
        notes=notes,
        cutoff_score=cutoff_score,
        truncated=truncated,
        baud_focus=baud_focus,
        phase0_liveness=phase0_liveness,
        viable_candidates=viable_candidates,
    )


def phase2_candidates_after_exploratory(
    all_candidates: Sequence[SerialSettings],
    selection: ExploratorySelection | None,
    narrowing_accepted: bool,
) -> tuple[list[SerialSettings], str]:
    """Return phase-2 candidates and a metadata source label."""
    if selection is None:
        return list(all_candidates), PHASE2_CANDIDATE_SOURCE_FULL
    if narrowing_accepted and selection.narrowed_candidates:
        return (
            list(selection.narrowed_candidates),
            PHASE2_CANDIDATE_SOURCE_NARROWED,
        )
    if selection.viable_candidates:
        return (
            list(selection.viable_candidates),
            PHASE2_CANDIDATE_SOURCE_VIABLE,
        )
    return list(all_candidates), PHASE2_CANDIDATE_SOURCE_FULL


def print_phase0_start(
    options: ScanOptions,
    phase0_options: ScanOptions,
    bauds: Sequence[int],
    candidate_count: int,
    payload: ProbePayload,
) -> None:
    """Print the Phase 0 liveness sweep start banner."""
    print()
    print_report_title("PHASE 0 BAUD LIVENESS SWEEP")
    print("MODE: FIXED 8E1 FLOW=NONE; BAUD GATE FOR QUICK EXPLORATORY.")
    print(
        f"PORTS: {options.in_port} -> {options.out_port}; "
        f"TEST={payload.byte_count} BYTES X {phase0_options.bursts}"
    )
    print(
        f"TIMING: READ={phase0_options.read_timeout:.2f}S, "
        f"PAUSE={phase0_options.settle_ms}MS."
    )
    print(
        f"CLEAR OUTPUT: ON; QUIET={phase0_options.pre_drain_quiet:.2f}S "
        f"LIMIT={phase0_options.pre_drain_timeout:.2f}S "
        f"MAX={phase0_options.max_drain_bytes} BYTES."
    )
    print(f"BAUDS: {len(bauds)}; SETTINGS BEFORE GATE: {candidate_count}.")
    print("ALIVE REQUIRES VALID PROBE LINE AND MARKER; NOISE IS NOT ENOUGH.")
    print(border_line(REPORT_WIDTH))


def format_phase0_progress(
    result: BaudLivenessResult,
    index: int,
    total: int,
) -> str:
    """Return one Phase 0 console progress line."""
    state = "ALIVE" if result.alive else "NOT ALIVE"
    return (
        f"PHASE0 [{index:04d}/{total:04d}] {state:<9} "
        f"{result.settings.label():32s} "
        f"READ={result.bytes_received:6d} CLR={result.bytes_drained_before:6d} "
        f"SCORE={result.score:6.2f} {result.reason}"
    )


def print_phase0_summary(report: BaudLivenessReport) -> None:
    """Print a concise Phase 0 liveness summary."""
    print()
    print_report_title("PHASE 0 RESULTS")
    print(f"  RUN TIME:             {format_duration(report.elapsed_sec)}")
    print(f"  BAUDS TESTED:         {len(report.tested_bauds)}")
    print(f"  ALIVE BAUDS:          {len(report.alive_bauds)}")
    print(f"  FIXED SETTINGS:       {phase0_fixed_settings_label()}")
    if report.alive_bauds:
        print_wrapped_value(
            "  ALIVE LIST:           ",
            ", ".join(str(baud) for baud in report.alive_bauds),
        )
        print(
            "  QUICK CANDIDATES:     "
            f"{report.candidate_count_after}/{report.candidate_count_before}"
        )
    if report.fallback_to_all_bauds:
        print("  QUICK CANDIDATES:     ALL SELECTED SETTINGS")
        if report.fallback_reason:
            print_wrapped_value("  FALLBACK:             ", report.fallback_reason)
    print(border_line(REPORT_WIDTH))


def print_exploratory_start(
    options: ScanOptions,
    exploratory_options: ScanOptions,
    candidate_count: int,
    payload: ProbePayload,
) -> None:
    """Print the exploratory-mode start banner."""
    print()
    print_report_title("QUICK EXPLORATORY START")
    print("MODE: FIXED INTERNAL PRE-SCAN; FULL SCAN SETTINGS ARE NOT CHANGED.")
    print(
        f"PORTS: {options.in_port} -> {options.out_port}; "
        f"TEST={payload.byte_count} BYTES X {exploratory_options.bursts}"
    )
    print(
        f"TIMING: READ={exploratory_options.read_timeout:.1f}S, "
        f"PAUSE={exploratory_options.settle_ms}MS."
    )
    print(
        f"CLEAR OUTPUT: ON; QUIET={exploratory_options.pre_drain_quiet:.1f}S "
        f"LIMIT={exploratory_options.pre_drain_timeout:.1f}S "
        f"MAX={exploratory_options.max_drain_bytes} BYTES."
    )
    print(f"QUICK BAUD FOCUS: {baud_focus_settings_label(options)}.")
    print(f"SETTINGS: {candidate_count}; BROAD LIGHT PASS.")
    print(border_line(REPORT_WIDTH))


def print_baud_focus_report(report: BaudFocusReport) -> None:
    """Print baud focus summary lines in the report style."""
    if not report.enabled:
        print("  QUICK BAUD FOCUS:    OFF")
        return
    if report.engaged:
        print(f"  QUICK BAUD FOCUS:    ENGAGED {report.focused_baud}")
        print(
            "  OTHER BAUDS:         "
            + (
                f"DEFERRED {report.deferred_candidate_count} SETTINGS"
                if report.deferred_other_bauds
                else "NOT DEFERRED"
            )
        )
        if report.engage_reason:
            print_wrapped_value("  FOCUS REASON:        ", report.engage_reason)
        if report.release_reason:
            print_wrapped_value("  FOCUS RELEASE:       ", report.release_reason)
        return
    print("  QUICK BAUD FOCUS:    NOT ENGAGED")
    if report.disabled_reason:
        print_wrapped_value("  FOCUS DISABLED:      ", report.disabled_reason)


def print_exploratory_summary(selection: ExploratorySelection) -> None:
    """Print a concise exploratory finding summary."""
    print()
    print_report_title("QUICK EXPLORATORY RESULTS")
    print(f"  RUN TIME:             {format_duration(selection.elapsed_sec)}")
    print(f"  SETTINGS TESTED:      {len(selection.results)}")
    print(f"  FIXED SETTINGS:       {exploratory_fixed_settings_label()}")
    phase0 = selection.phase0_liveness
    print(
        "  PHASE 0 ALIVE:        "
        f"{len(phase0.alive_bauds)}/{len(phase0.tested_bauds)} BAUDS"
    )
    if phase0.fallback_to_all_bauds:
        print("  PHASE 0 GATE:         FALLBACK TO ALL BAUDS")
    else:
        print(
            "  PHASE 0 GATE:         "
            f"{phase0.candidate_count_after}/{phase0.candidate_count_before} SETTINGS"
        )
    print_baud_focus_report(selection.baud_focus)
    if selection.fallback_reason:
        print_wrapped_value("  FINDING:              ", selection.fallback_reason)
    elif selection.shortlist_results:
        print(
            "  FINDING:              "
            f"{len(selection.shortlist_results)} ROWS AT SCORE >= "
            f"{selection.cutoff_score:.1f}"
        )
        print(
            "  FULL-SCAN CANDIDATES: "
            f"{len(selection.narrowed_candidates)} AFTER FLOW EXPANSION"
        )
    print(
        "  SIGNAL CANDIDATES:    "
        f"{len(selection.viable_candidates)} FOR PHASE 2 FALLBACK"
    )

    top_rows = [
        result for result in selection.ranked_results if result.score > 0.0
    ][:EXPLORATORY_SUMMARY_ROWS]
    if top_rows:
        print()
        print(border_line(REPORT_WIDTH))
        print("RK SCORE   BAUD MODE FLOW       READ  CLR  EXCT LINE RESULT")
        print(border_line(REPORT_WIDTH))
        for rank, result in enumerate(top_rows, start=1):
            mode = (
                f"{result.settings.data_bits}"
                f"{result.settings.parity_code()}"
                f"{result.settings.stop_bits}"
            )
            indicator = result_indicator(result.score, result.status, result.error)
            print(
                f"{rank:>2} "
                f"{result.score:>5.1f} "
                f"{result.settings.baud:>6} "
                f"{mode:<4} "
                f"{result.settings.flow_control.upper():<8} "
                f"{result.bytes_received:>6} "
                f"{result.bytes_drained_before:>4} "
                f"{result.metrics.exact_byte_match_ratio:>4.2f} "
                f"{result.metrics.line_integrity_ratio:>4.2f} "
                f"{indicator}"
            )
        print(border_line(REPORT_WIDTH))
    else:
        print("  TOP ROWS:             NONE")

    if selection.notes:
        print()
        print("  NOTES:")
        for note in selection.notes:
            print_wrapped_value("    ", note)
    print(border_line(REPORT_WIDTH))


def exploratory_metadata(
    requested: bool,
    narrowing_accepted: bool,
    selection: ExploratorySelection | None,
    original_candidate_count: int,
    final_candidate_count: int,
    options: ScanOptions,
    phase2_candidate_source: str = PHASE2_CANDIDATE_SOURCE_FULL,
) -> dict[str, Any]:
    """Return structured metadata for exploratory mode and candidate narrowing."""
    if narrowing_accepted:
        phase2_candidate_source = PHASE2_CANDIDATE_SOURCE_NARROWED
    phase0_fixed_settings = {
        "payload_bytes": phase0_payload_bytes(),
        "read_timeout": PHASE0_READ_TIMEOUT,
        "settle_ms": PHASE0_SETTLE_MS,
        "bursts": PHASE0_BURSTS,
        "progress_interval": PHASE0_PROGRESS_INTERVAL,
        "pre_drain_enabled": True,
        "pre_drain_timeout": PHASE0_PRE_DRAIN_TIMEOUT,
        "pre_drain_quiet": PHASE0_PRE_DRAIN_QUIET,
        "max_drain_bytes": PHASE0_MAX_DRAIN_BYTES,
        "baseline_data_bits": PHASE0_BASELINE_DATA_BITS,
        "baseline_parity": PHASE0_BASELINE_PARITY,
        "baseline_stop_bits": PHASE0_BASELINE_STOP_BITS,
        "baseline_flow_control": PHASE0_BASELINE_FLOW_CONTROL,
        "min_alive_score": PHASE0_MIN_ALIVE_SCORE,
        "min_line_integrity": PHASE0_MIN_LINE_INTEGRITY,
        "max_extra_bytes": PHASE0_MAX_EXTRA_BYTES,
        "max_extra_bytes_ratio": PHASE0_MAX_EXTRA_BYTES_RATIO,
    }
    fixed_settings = {
        "payload_bytes": exploratory_payload_bytes(),
        "read_timeout": EXPLORATORY_READ_TIMEOUT,
        "settle_ms": EXPLORATORY_SETTLE_MS,
        "bursts": EXPLORATORY_BURSTS,
        "progress_interval": EXPLORATORY_PROGRESS_INTERVAL,
        "pre_drain_enabled": True,
        "pre_drain_timeout": EXPLORATORY_PRE_DRAIN_TIMEOUT,
        "pre_drain_quiet": EXPLORATORY_PRE_DRAIN_QUIET,
        "max_drain_bytes": EXPLORATORY_MAX_DRAIN_BYTES,
        "shortlist_limit": EXPLORATORY_SHORTLIST_LIMIT,
        "min_narrow_score": EXPLORATORY_MIN_NARROW_SCORE,
        "score_tolerance": EXPLORATORY_SCORE_TOLERANCE,
        "baud_focus_enabled": options.baud_focus_enabled,
        "baud_focus_strong_score_threshold": (
            options.baud_focus_strong_score_threshold
        ),
        "baud_focus_lead_gap_threshold": options.baud_focus_lead_gap_threshold,
        "baud_focus_min_strong_results": options.baud_focus_min_strong_results,
        "baud_focus_min_samples": options.baud_focus_min_samples,
    }
    metadata: dict[str, Any] = {
        "requested": requested,
        "phase0_fixed_internal_settings": phase0_fixed_settings,
        "fixed_internal_settings": fixed_settings,
        "narrowing_accepted": narrowing_accepted,
        "original_candidate_count": original_candidate_count,
        "final_candidate_count": final_candidate_count,
        "candidate_source": phase2_candidate_source,
        "phase2_candidate_source": phase2_candidate_source,
    }
    if selection is None:
        metadata["ran"] = False
        metadata["shortlist_count"] = 0
        metadata["narrowed_candidate_count"] = 0
        metadata["viable_signal_candidate_count"] = 0
        metadata["baud_focus"] = dataclass_to_jsonable(
            empty_baud_focus_report(
                options.baud_focus_enabled,
                "EXPLORATORY NOT RUN",
            )
        )
        metadata["phase0_baud_liveness"] = dataclass_to_jsonable(
            empty_baud_liveness_report(
                original_candidate_count,
                "EXPLORATORY NOT RUN",
            )
        )
        return metadata

    metadata.update(
        {
            "ran": True,
            "elapsed_sec": selection.elapsed_sec,
            "tested_candidates": len(selection.results),
            "fallback_reason": selection.fallback_reason,
            "cutoff_score": selection.cutoff_score,
            "truncated": selection.truncated,
            "notes": selection.notes,
            "phase0_baud_liveness": dataclass_to_jsonable(
                selection.phase0_liveness
            ),
            "baud_focus": dataclass_to_jsonable(selection.baud_focus),
            "shortlist_count": len(selection.shortlist_results),
            "narrowed_candidate_count": len(selection.narrowed_candidates),
            "viable_signal_candidate_count": len(selection.viable_candidates),
            "shortlist_settings": [
                dataclass_to_jsonable(result.settings)
                for result in selection.shortlist_results
            ],
            "narrowed_candidate_settings": [
                dataclass_to_jsonable(settings)
                for settings in selection.narrowed_candidates
            ],
            "viable_signal_candidate_settings": [
                dataclass_to_jsonable(settings)
                for settings in selection.viable_candidates
            ],
            "top_results": [
                dataclass_to_jsonable(result)
                for result in selection.ranked_results[:EXPLORATORY_SUMMARY_ROWS]
            ],
        }
    )
    return metadata


def run_phase0_baud_liveness(
    serial_module: Any,
    options: ScanOptions,
    candidates: Sequence[SerialSettings],
    logger: logging.Logger,
) -> BaudLivenessReport:
    """Run the fixed 8E1 baud liveness sweep used before quick mode."""
    purge_buffer_output(
        serial_module=serial_module,
        options=options,
        logger=logger,
        reason="START PHASE 0 WITH AN EMPTY BUFFER FIFO.",
    )
    phase0_options = phase0_scan_options(options)
    phase0_payload = generate_phase0_payload(phase0_options.payload_bytes)
    baud_order, _ = group_candidates_by_baud(candidates)
    if not baud_order:
        return empty_baud_liveness_report(
            len(candidates),
            "NO BAUDS SELECTED",
        )

    print_phase0_start(
        options=options,
        phase0_options=phase0_options,
        bauds=baud_order,
        candidate_count=len(candidates),
        payload=phase0_payload,
    )
    logger.info("phase 0 baud liveness sweep started")
    logger.info("phase 0 options: %s", phase0_options)
    logger.info(
        "phase 0 payload: %s bytes, %s lines",
        phase0_payload.byte_count,
        phase0_payload.line_count,
    )

    started = time.monotonic()
    results: list[BaudLivenessResult] = []
    expected_byte_count = phase0_payload.byte_count * phase0_options.bursts
    for index, baud in enumerate(baud_order, start=1):
        candidate_result = run_candidate(
            serial_module=serial_module,
            index=index,
            total=len(baud_order),
            settings=phase0_baseline_settings(baud),
            options=phase0_options,
            payload=phase0_payload,
            logger=logger,
            progress=None,
        )
        decision = classify_phase0_liveness(candidate_result, expected_byte_count)
        liveness_result = baud_liveness_result_from_candidate(
            candidate_result,
            decision,
        )
        results.append(liveness_result)
        print(
            format_phase0_progress(liveness_result, index, len(baud_order)),
            flush=True,
        )
        print(format_scan_eta(len(results), len(baud_order), started), flush=True)
        logger.info(
            "phase 0 baud %s: %s score=%.2f status=%s reason=%s read=%s cleared=%s error=%s",
            baud,
            "alive" if decision.alive else "not-alive",
            candidate_result.score,
            candidate_result.status,
            decision.reason,
            candidate_result.bytes_received,
            candidate_result.bytes_drained_before,
            candidate_result.error,
        )
        if decision.alive:
            if not prompt_yes_no_question(
                "Live Phase 0 baud found. Continue Phase 0 sweep?",
                False,
            ):
                logger.info("phase 0 stopped after live baud %s", baud)
                break

    elapsed_sec = time.monotonic() - started
    alive_bauds = [result.baud for result in results if result.alive]
    fallback_reason = None
    fallback_to_all = False
    if not alive_bauds:
        fallback_to_all = True
        fallback_reason = "NO ALIVE BAUDS; QUICK MODE WILL USE ALL SELECTED BAUDS."
    alive = set(alive_bauds)
    candidate_count_after = (
        len(candidates)
        if fallback_to_all
        else sum(1 for candidate in candidates if candidate.baud in alive)
    )
    report = BaudLivenessReport(
        ran=True,
        tested_bauds=list(baud_order),
        alive_bauds=alive_bauds,
        fallback_to_all_bauds=fallback_to_all,
        fallback_reason=fallback_reason,
        candidate_count_before=len(candidates),
        candidate_count_after=candidate_count_after,
        elapsed_sec=elapsed_sec,
        results=results,
    )
    print_phase0_summary(report)
    logger.info(
        "phase 0 completed; alive=%s/%s fallback=%s quick_candidates=%s/%s",
        len(alive_bauds),
        len(baud_order),
        fallback_to_all,
        candidate_count_after,
        len(candidates),
    )
    return report


def run_phase0_only_sweep(
    serial_module: Any,
    options: ScanOptions,
    logger: logging.Logger,
) -> int:
    """Run only the Phase 0 baud liveness sweep and stop before full scan."""
    candidates = prioritize_discovery_candidates(
        generate_candidates(options.min_baud, options.max_baud),
        options,
    )
    logger.info("phase 0 only sweep requested")
    logger.info("options: %s", options)
    logger.info("phase 0 only candidates before baud grouping: %s", len(candidates))
    run_phase0_baud_liveness(
        serial_module=serial_module,
        options=options,
        candidates=candidates,
        logger=logger,
    )
    print()
    print_report_title("PHASE 0 SWEEP COMPLETE")
    print("FULL SCAN WAS NOT RUN.")
    print_wrapped_value("  DEBUG LOG:   ", options.log_file)
    print(border_line(REPORT_WIDTH))
    return 0


def dual_phase0_settings(input_baud: int, output_baud: int) -> DualSerialSettings:
    """Return fixed baseline dual-bank settings for one baud pair."""
    return DualSerialSettings(
        input_settings=phase0_baseline_settings(input_baud),
        output_settings=phase0_baseline_settings(output_baud),
    )


def frame_candidates_for_baud(baud: int) -> list[SerialSettings]:
    """Return frame candidates for one baud in discovery order."""
    frames = [
        SerialSettings(baud, data_bits, parity, stop_bits, "none")
        for data_bits in DATA_BITS
        for parity in PARITIES
        for stop_bits in STOP_BITS
    ]
    frames.sort(key=discovery_frame_priority)
    return frames


def unique_dual_candidates(
    candidates: Sequence[DualSerialSettings],
) -> list[DualSerialSettings]:
    """Return dual settings with duplicates removed while preserving order."""
    unique: list[DualSerialSettings] = []
    seen: set[DualSerialSettings] = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        unique.append(candidate)
        seen.add(candidate)
    return unique


def dual_output_frame_sweep_for_pair(
    input_baud: int,
    output_baud: int,
) -> list[DualSerialSettings]:
    """Hold Phase 0 input framing and sweep output frames."""
    seed = dual_phase0_settings(input_baud, output_baud)
    return unique_dual_candidates(
        [
            DualSerialSettings(seed.input_settings, output_frame)
            for output_frame in frame_candidates_for_baud(output_baud)
        ]
    )


def dual_input_frame_sweep_for_pair(
    input_baud: int,
    output_settings: SerialSettings,
) -> list[DualSerialSettings]:
    """Hold the best observed output framing and sweep input frames."""
    return unique_dual_candidates(
        [
            DualSerialSettings(input_frame, output_settings)
            for input_frame in frame_candidates_for_baud(input_baud)
        ]
    )


def dual_flow_candidates_for_frame(
    settings: DualSerialSettings,
) -> list[DualSerialSettings]:
    """Return all input/output flow combinations for one dual-bank frame pair."""
    return unique_dual_candidates(
        [
            DualSerialSettings(
                dataclasses.replace(settings.input_settings, flow_control=input_flow),
                dataclasses.replace(settings.output_settings, flow_control=output_flow),
            )
            for input_flow in FLOW_CONTROLS
            for output_flow in FLOW_CONTROLS
        ]
    )


def best_dual_settings(
    results: Sequence[CandidateResult],
    fallback: DualSerialSettings,
) -> DualSerialSettings:
    """Return the best observed dual setting, or the fallback seed."""
    eligible = [
        result
        for result in results
        if isinstance(result.settings, DualSerialSettings)
        and result.score > 0.0
    ]
    if not eligible:
        return fallback
    return dual_result_settings(sorted(eligible, key=result_sort_key, reverse=True)[0])


def best_dual_output_settings(
    results: Sequence[CandidateResult],
    seed: DualSerialSettings,
) -> SerialSettings:
    """Return the best output frame seen while the input frame was fixed."""
    eligible = [
        result
        for result in results
        if isinstance(result.settings, DualSerialSettings)
        and result.settings.input_settings == seed.input_settings
        and result.settings.output_settings.baud == seed.output_settings.baud
        and result.score > 0.0
    ]
    if not eligible:
        return seed.output_settings
    best_result = sorted(eligible, key=result_sort_key, reverse=True)[0]
    return dual_result_settings(best_result).output_settings


def has_recommendable_dual_result(results: Sequence[CandidateResult]) -> bool:
    """Return True when staged dual discovery found a usable pair."""
    return any(is_recommendable_result(result) for result in results)


def dual_frame_candidates_for_pair(
    input_baud: int,
    output_baud: int,
) -> list[DualSerialSettings]:
    """Return independent input/output frame candidates for one baud pair."""
    input_frames = frame_candidates_for_baud(input_baud)
    output_frames = frame_candidates_for_baud(output_baud)
    return [
        DualSerialSettings(input_settings=input_frame, output_settings=output_frame)
        for input_frame in input_frames
        for output_frame in output_frames
    ]


def dual_baud_result_from_candidate(
    result: CandidateResult,
    decision: Phase0LivenessDecision,
) -> DualBaudLivenessResult:
    """Return a dual-bank Phase 0 baud result row."""
    settings = result.settings
    if not isinstance(settings, DualSerialSettings):
        raise TypeError("dual baud result requires dual serial settings")
    return DualBaudLivenessResult(
        input_baud=settings.input_settings.baud,
        output_baud=settings.output_settings.baud,
        alive=decision.alive,
        reason=decision.reason,
        settings=settings,
        score=result.score,
        status=result.status,
        error=result.error,
        bytes_sent=result.bytes_sent,
        bytes_received=result.bytes_received,
        bytes_drained_before=result.bytes_drained_before,
        elapsed_sec=result.elapsed_sec,
        metrics=result.metrics,
    )


def format_dual_phase0_progress(
    result: DualBaudLivenessResult,
    index: int,
    total: int,
) -> str:
    """Return one dual-bank Phase 0 console progress line."""
    state = "ALIVE" if result.alive else "NOT ALIVE"
    return (
        f"D-PHASE0 [{index:04d}/{total:04d}] {state:<9} "
        f"IN={result.input_baud:>6} OUT={result.output_baud:>6} "
        f"READ={result.bytes_received:6d} CLR={result.bytes_drained_before:6d} "
        f"SCORE={result.score:6.2f} {result.reason}"
    )


def select_dual_phase0_pairs(
    results: Sequence[DualBaudLivenessResult],
) -> tuple[list[tuple[int, int]], str | None]:
    """Return baud pairs to expand into dual-bank frame testing."""
    alive = [result for result in results if result.alive]
    if alive:
        alive.sort(
            key=lambda result: (
                result.score,
                result.metrics.line_integrity_ratio,
                result.metrics.exact_byte_match_ratio,
                result.bytes_received,
            ),
            reverse=True,
        )
        return (
            [
                (result.input_baud, result.output_baud)
                for result in alive[:DUAL_PHASE0_BAUD_PAIR_LIMIT]
            ],
            None,
        )
    signaled = [
        result
        for result in results
        if not result.error
        and result.status not in {"error", "no-data", "stale-output", "partial-write"}
        and result.score > 0.0
    ]
    signaled.sort(
        key=lambda result: (
            result.score,
            result.metrics.line_integrity_ratio,
            result.metrics.exact_byte_match_ratio,
            result.bytes_received,
        ),
        reverse=True,
    )
    if signaled:
        return (
            [
                (result.input_baud, result.output_baud)
                for result in signaled[:DUAL_PHASE0_FALLBACK_PAIR_LIMIT]
            ],
            "NO ALIVE BAUD PAIRS; USING BEST PARTIAL-SIGNAL PAIRS.",
        )
    return [], "NO DUAL-BANK BAUD PAIRS PRODUCED A USABLE SIGNAL."


def print_dual_phase0_summary(report: DualBaudLivenessReport) -> None:
    """Print a concise dual-bank Phase 0 summary."""
    input_bauds = sorted({input_baud for input_baud, _ in report.alive_pairs}, reverse=True)
    output_bauds = sorted({output_baud for _, output_baud in report.alive_pairs}, reverse=True)
    print()
    print_report_title("DUAL PHASE 0 RESULTS")
    print(f"  RUN TIME:             {format_duration(report.elapsed_sec)}")
    print(f"  BAUD PAIRS TESTED:    {len(report.tested_pairs)}/{report.total_pairs}")
    print(f"  ALIVE PAIRS:          {len(report.alive_pairs)}")
    if input_bauds:
        print_wrapped_value(
            "  INPUT BAUDS:          ",
            ", ".join(str(baud) for baud in input_bauds),
        )
    if output_bauds:
        print_wrapped_value(
            "  OUTPUT BAUDS:         ",
            ", ".join(str(baud) for baud in output_bauds),
        )
    if report.selected_pairs:
        print_wrapped_value(
            "  PAIRS FOR PHASE 1:    ",
            ", ".join(
                f"IN {input_baud}/OUT {output_baud}"
                for input_baud, output_baud in report.selected_pairs
            ),
        )
    if report.fallback_reason:
        print_wrapped_value("  NOTE:                 ", report.fallback_reason)
    print(border_line(REPORT_WIDTH))


def run_dual_phase0_baud_matrix(
    serial_module: Any,
    options: ScanOptions,
    logger: logging.Logger,
) -> DualBaudLivenessReport:
    """Run a fixed-frame input/output baud-pair liveness matrix."""
    purge_buffer_output(
        serial_module=serial_module,
        options=options,
        logger=logger,
        reason="START DUAL PHASE 0 WITH AN EMPTY BUFFER FIFO.",
    )
    phase0_options = phase0_scan_options(options)
    phase0_payload = generate_phase0_payload(phase0_options.payload_bytes)
    baud_order = scan_bauds(options.min_baud, options.max_baud)
    pairs = [
        (input_baud, output_baud)
        for input_baud in baud_order
        for output_baud in baud_order
    ]
    print()
    print_report_title("DUAL PHASE 0 BAUD MATRIX")
    print("MODE: TWO SWITCH BANKS; INPUT AND OUTPUT BAUDS MAY DIFFER.")
    print(
        f"PORTS: IN {options.in_port} -> BUFFER -> OUT {options.out_port}; "
        f"TEST={phase0_payload.byte_count} BYTES X {phase0_options.bursts}"
    )
    print("FIXED FRAME: 8E1 FLOW=NONE ON BOTH SIDES.")
    print(f"BAUD PAIRS: {len(pairs)}.")
    print(border_line(REPORT_WIDTH))
    logger.info("dual phase 0 baud matrix started pairs=%s", len(pairs))

    started = time.monotonic()
    results: list[DualBaudLivenessResult] = []
    expected_byte_count = phase0_payload.byte_count * phase0_options.bursts
    for index, (input_baud, output_baud) in enumerate(pairs, start=1):
        candidate_result = run_dual_candidate(
            serial_module=serial_module,
            index=index,
            total=len(pairs),
            settings=dual_phase0_settings(input_baud, output_baud),
            options=phase0_options,
            payload=phase0_payload,
            logger=logger,
            progress=None,
        )
        decision = classify_phase0_liveness(candidate_result, expected_byte_count)
        liveness_result = dual_baud_result_from_candidate(
            candidate_result,
            decision,
        )
        results.append(liveness_result)
        print(format_dual_phase0_progress(liveness_result, index, len(pairs)), flush=True)
        print(format_scan_eta(len(results), len(pairs), started), flush=True)
        logger.info(
            "dual phase 0 in=%s out=%s: %s score=%.2f status=%s reason=%s read=%s cleared=%s error=%s",
            input_baud,
            output_baud,
            "alive" if decision.alive else "not-alive",
            candidate_result.score,
            candidate_result.status,
            decision.reason,
            candidate_result.bytes_received,
            candidate_result.bytes_drained_before,
            candidate_result.error,
        )
        if decision.alive:
            if not prompt_yes_no_question(
                "Live dual baud pair found. Continue Phase 0 matrix?",
                False,
            ):
                logger.info(
                    "dual phase 0 stopped after live pair in=%s out=%s",
                    input_baud,
                    output_baud,
                )
                break

    elapsed_sec = sum(result.elapsed_sec for result in results)
    alive_pairs = [
        (result.input_baud, result.output_baud)
        for result in results
        if result.alive
    ]
    tested_pairs = [
        (result.input_baud, result.output_baud)
        for result in results
    ]
    selected_pairs, fallback_reason = select_dual_phase0_pairs(results)
    report = DualBaudLivenessReport(
        ran=True,
        tested_pairs=tested_pairs,
        total_pairs=len(pairs),
        alive_pairs=alive_pairs,
        selected_pairs=selected_pairs,
        fallback_reason=fallback_reason,
        elapsed_sec=elapsed_sec,
        results=results,
    )
    print_dual_phase0_summary(report)
    logger.info(
        "dual phase 0 completed alive=%s/%s selected=%s reason=%s",
        len(alive_pairs),
        len(results),
        selected_pairs,
        fallback_reason,
    )
    return report


def dual_result_settings(result: CandidateResult) -> DualSerialSettings:
    """Return dual settings from a candidate result or raise a clear error."""
    if not isinstance(result.settings, DualSerialSettings):
        raise TypeError("expected dual-bank candidate result")
    return result.settings


def ranked_dual_top_results(
    results: Sequence[CandidateResult],
    top: int,
) -> list[CandidateResult]:
    """Return top non-zero dual-bank results."""
    ranked = [
        result
        for result in sorted(results, key=result_sort_key, reverse=True)
        if result.score > 0.0
    ]
    perfect_count = sum(1 for result in ranked if result.score >= PERFECT_SCORE)
    return ranked[: max(top, perfect_count)]


def dual_side_table_label(settings: SerialSettings) -> str:
    """Return compact baud/frame/flow text for dual-bank tables."""
    frame = f"{settings.data_bits}{settings.parity_code()}{settings.stop_bits}"
    return f"{settings.baud} {frame} {flow_control_code(settings.flow_control)}"


def format_dual_progress(result: CandidateResult) -> str:
    """Return one console progress line for a dual-bank candidate."""
    settings = dual_result_settings(result)
    status = (
        result.status.upper()
        if not result.error
        else f"{result.status.upper()}: {result.error[:60]}"
    )
    indicator = result_indicator(result.score, result.status, result.error)
    return (
        f"[{result.index:04d}/{result.total:04d}] RESULT {indicator} "
        f"{settings.label():48s} "
        f"SENT={result.bytes_sent:7d} READ={result.bytes_received:7d} "
        f"CLR={result.bytes_drained_before:7d} "
        f"SCORE={result.score:6.2f} {status}"
    )


def print_dual_ranked_table(
    results: Sequence[CandidateResult],
    top: int,
    report_title: str = "DUAL BANK FINAL REPORT",
) -> None:
    """Print a ranked dual-bank result table."""
    ranked = ranked_dual_top_results(results, top)
    print()
    print_report_title(report_title)
    print("TOP OBSERVED INPUT/OUTPUT PAIRS (NON-ZERO SCORES)")
    print(border_line(REPORT_WIDTH))
    print("RK SCORE  IN SETTING      OUT SETTING      SENT   READ  CLR EXCT LINE RES")
    print(border_line(REPORT_WIDTH))
    for rank, result in enumerate(ranked, start=1):
        settings = dual_result_settings(result)
        indicator = result_indicator(result.score, result.status, result.error)
        print(
            f"{rank:>2} "
            f"{result.score:>5.1f} "
            f"{dual_side_table_label(settings.input_settings):<15} "
            f"{dual_side_table_label(settings.output_settings):<15} "
            f"{result.bytes_sent:>6} "
            f"{result.bytes_received:>6} "
            f"{result.bytes_drained_before:>4} "
            f"{result.metrics.exact_byte_match_ratio:>4.2f} "
            f"{result.metrics.line_integrity_ratio:>4.2f} "
            f"{indicator[:3]}"
        )
    print(border_line(REPORT_WIDTH))
    if not ranked:
        print("NO NON-ZERO RESULTS.")


def print_dual_result_details(result: CandidateResult) -> None:
    """Print switch-setting and score details for one dual-bank result."""
    settings = dual_result_settings(result)
    print(f"    INPUT BAUD:         {settings.input_settings.baud}")
    print(
        f"    INPUT MODE:         "
        f"{settings.input_settings.data_bits}"
        f"{settings.input_settings.parity_code()}"
        f"{settings.input_settings.stop_bits}"
    )
    print(
        f"    INPUT FLOW:         "
        f"{flow_control_name(settings.input_settings.flow_control)}"
    )
    print(f"    OUTPUT BAUD:        {settings.output_settings.baud}")
    print(
        f"    OUTPUT MODE:        "
        f"{settings.output_settings.data_bits}"
        f"{settings.output_settings.parity_code()}"
        f"{settings.output_settings.stop_bits}"
    )
    print(
        f"    OUTPUT FLOW:        "
        f"{flow_control_name(settings.output_settings.flow_control)}"
    )
    print_wrapped_value("    SETTING:            ", settings.label())
    print(
        f"    RESULT:             "
        f"{result_indicator(result.score, result.status, result.error)}"
    )
    print(f"    SCORE:              {result.score:.2f}/100")
    print(f"    REPEAT:             {result.repeatability:.3f}")
    print(f"    SENT/READ:          {result.bytes_sent}/{result.bytes_received}")
    print(f"    EXACT RATIO:        {result.metrics.exact_byte_match_ratio:.3f}")
    print(f"    LINE RATIO:         {result.metrics.line_integrity_ratio:.3f}")
    if result.error:
        print_wrapped_value("    ERROR:              ", result.error)


def print_dual_scan_summary(
    results: Sequence[CandidateResult],
    total_candidates: int,
    elapsed_sec: float,
    early_stopped: bool,
    top: int,
) -> None:
    """Print a concise dual-bank scan summary."""
    ranked = sorted(results, key=result_sort_key, reverse=True)
    best = ranked[0] if ranked else None
    tied = top_tied_results(results)
    print()
    print_report_title("DUAL BANK SUMMARY")
    print(f"  RUN TIME:             {format_duration(elapsed_sec)}")
    print(f"  SETTINGS TESTED:      {len(results)}/{total_candidates}")
    print(f"  ENDED EARLY:          {'YES' if early_stopped else 'NO'}")
    print(f"  FINDING:              {confidence_summary(best, len(tied))}")
    if best is None:
        return
    print()
    print(border_line(REPORT_WIDTH))
    if len(tied) > 1:
        print(bordered_text("MULTIPLE TOP INPUT/OUTPUT PAIRS", REPORT_WIDTH))
        print(border_line(REPORT_WIDTH))
        for rank, result in enumerate(ranked_dual_top_results(tied, top), start=1):
            print_wrapped_value(f"    {rank}. ", dual_result_settings(result).label())
        print(border_line(REPORT_WIDTH))
        return
    if is_recommendable_result(best):
        print(bordered_text("RECOMMENDED DUAL-BANK SETTING", REPORT_WIDTH))
    else:
        print(bordered_text("BEST OBSERVED DUAL-BANK RESULT", REPORT_WIDTH))
    print(border_line(REPORT_WIDTH))
    print_dual_result_details(best)
    print(border_line(REPORT_WIDTH))


def dual_candidate_table_lines(
    results: Sequence[CandidateResult],
    top: int,
    title: str,
) -> list[str]:
    """Return compact dual-bank ranked result table lines."""
    ranked = ranked_dual_top_results(results, top)
    lines = [
        title,
        border_line(REPORT_WIDTH),
        "RK SCORE  IN SETTING      OUT SETTING      SENT   READ  CLR EXCT LINE RES",
        border_line(REPORT_WIDTH),
    ]
    for rank, result in enumerate(ranked, start=1):
        settings = dual_result_settings(result)
        indicator = result_indicator(result.score, result.status, result.error)
        lines.append(
            f"{rank:>2} "
            f"{result.score:>5.1f} "
            f"{dual_side_table_label(settings.input_settings):<15} "
            f"{dual_side_table_label(settings.output_settings):<15} "
            f"{result.bytes_sent:>6} "
            f"{result.bytes_received:>6} "
            f"{result.bytes_drained_before:>4} "
            f"{result.metrics.exact_byte_match_ratio:>4.2f} "
            f"{result.metrics.line_integrity_ratio:>4.2f} "
            f"{indicator[:3]}"
        )
    lines.append(border_line(REPORT_WIDTH))
    if not ranked:
        lines.append("NO NON-ZERO RESULTS.")
    return lines


def dual_text_summary_lines(results: Sequence[CandidateResult]) -> list[str]:
    """Return summary and interpretation lines for a dual-bank report section."""
    ranked = sorted(results, key=result_sort_key, reverse=True)
    best = ranked[0] if ranked else None
    tied = top_tied_results(results)
    lines = [
        f"SETTINGS TESTED: {len(results)}",
        f"RESULT COUNTS:   {result_count_summary(results)}",
        f"FINDING:         {confidence_summary(best, len(tied))}",
    ]
    if best is not None:
        lines.append(f"BEST PAIR:       {dual_result_settings(best).label()}")
        lines.append(
            f"BEST SCORE:      {best.score:.2f} "
            f"SENT/READ={best.bytes_sent}/{best.bytes_received}"
        )
    if len(tied) > 1:
        lines.append(f"AMBIGUOUS:       {len(tied)} INPUT/OUTPUT PAIRS TIED.")
    elif is_recommendable_result(best):
        lines.append("ACTION:          RECORD THIS AS THE LIKELY BANK PAIR.")
    elif has_any_signal(results):
        lines.append("ACTION:          PARTIAL RESULT ONLY; REPEAT OR CHECK WIRING.")
    else:
        lines.append("ACTION:          NO WORKING INPUT/OUTPUT PAIR FOUND.")
    return lines


def write_dual_bank_text_report(
    path: Path,
    metadata: dict[str, Any],
    phase0: DualBaudLivenessReport,
    results: Sequence[CandidateResult],
    validation_results: Sequence[CandidateResult] | None = None,
) -> None:
    """Append a compact dual-bank scan report entry."""
    path.parent.mkdir(parents=True, exist_ok=True)
    validation_results = [] if validation_results is None else list(validation_results)
    created = dt.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")
    switch_note = str(metadata.get("switch_note") or "").strip()
    selected_pairs = ", ".join(
        f"IN {input_baud}/OUT {output_baud}"
        for input_baud, output_baud in phase0.selected_pairs
    )
    lines = [
        "",
        border_line(REPORT_WIDTH),
        bordered_text("SERIAL PROBE DUAL-BANK REPORT", REPORT_WIDTH),
        border_line(REPORT_WIDTH),
        f"APPENDED:        {created}",
        f"STARTED UTC:     {metadata.get('started_at', '')}",
        f"COM PATH:        {metadata.get('in_port')} -> BUFFER -> {metadata.get('out_port')}",
        f"SWITCH NOTE:     {switch_note if switch_note else '(NOT ENTERED)'}",
        "SCAN MODEL:      DUAL BANK INPUT/OUTPUT SETTINGS",
        (
            "PHASE 0 PAIRS:   "
            f"{len(phase0.alive_pairs)} ALIVE / "
            f"{len(phase0.tested_pairs)} TESTED / {phase0.total_pairs} POSSIBLE"
        ),
        f"PHASE 1 PAIRS:   {selected_pairs if selected_pairs else '(NONE)'}",
        f"STAGED TESTS:    {metadata.get('staged_candidate_count', 0)} PLANNED",
        (
            "FLOW DISCOVERY: "
            + ("ON" if metadata.get("dual_flow_discovery_enabled") else "OFF")
        ),
        (
            "FULL MATRIX:     "
            + (
                "RUN"
                if metadata.get("full_matrix_ran")
                else "SKIPPED"
                if metadata.get("full_matrix_skipped")
                else "NOT REACHED"
            )
        ),
        "",
        "DUAL-BANK SUMMARY:",
        *dual_text_summary_lines(results),
        "",
        *dual_candidate_table_lines(results, int(metadata.get("top", 15)), "DUAL-BANK TOP RESULTS"),
    ]
    if validation_results:
        lines.extend(
            [
                "",
                "DUAL-BANK VALIDATION:",
                *dual_text_summary_lines(validation_results),
                "",
                *dual_candidate_table_lines(
                    validation_results,
                    min(int(metadata.get("top", 15)), len(validation_results)),
                    "DUAL-BANK VALIDATION RESULTS",
                ),
            ]
        )
    lines.extend(
        [
            "",
            "INTERPRETATION NOTES:",
            "  DUAL-BANK MODE DOES NOT ASSUME SW1 AND SW2 USE THE SAME SETTING.",
            "  INPUT VALUES ARE FOR THE PC TRANSMIT PORT INTO THE BUFFER.",
            "  OUTPUT VALUES ARE FOR THE PC RECEIVE PORT FROM THE BUFFER.",
            "  INPUT FLOW AND OUTPUT FLOW ARE TESTED AS SEPARATE PORT SETTINGS.",
            "  FLOW TRANSFER MATCHES CAN STILL TIE UNLESS THE BUFFER CREATES BACKPRESSURE.",
            border_line(REPORT_WIDTH),
        ]
    )
    with path.open("a", encoding="utf-8") as report_file:
        report_file.write("\n".join(lines) + "\n")


def dual_scan_candidate_count(pairs: Sequence[tuple[int, int]]) -> int:
    """Return total dual-bank frame candidates for selected baud pairs."""
    frame_count = len(DATA_BITS) * len(PARITIES) * len(STOP_BITS)
    return len(pairs) * frame_count * frame_count


def run_exploratory_scan(
    serial_module: Any,
    options: ScanOptions,
    candidates: Sequence[SerialSettings],
    logger: logging.Logger,
) -> ExploratorySelection:
    """Run quick exploratory mode and return candidate narrowing findings."""
    phase0_liveness = run_phase0_baud_liveness(
        serial_module=serial_module,
        options=options,
        candidates=candidates,
        logger=logger,
    )
    quick_candidates = candidates_after_phase0_liveness(candidates, phase0_liveness)
    quick_options = exploratory_scan_options(options)
    quick_payload = generate_payload(quick_options.payload_bytes)
    print_exploratory_start(
        options,
        quick_options,
        len(quick_candidates),
        quick_payload,
    )
    logger.info("quick exploratory mode started")
    logger.info("quick options: %s", quick_options)
    logger.info(
        "quick candidates after phase 0: %s/%s",
        len(quick_candidates),
        len(candidates),
    )
    logger.info(
        "quick payload: %s bytes, %s lines",
        quick_payload.byte_count,
        quick_payload.line_count,
    )

    started = time.monotonic()
    results: list[CandidateResult] = []
    tested: set[SerialSettings] = set()
    baud_order, grouped = group_candidates_by_baud(quick_candidates)
    focus_enabled = options.baud_focus_enabled and len(baud_order) > 1
    focus_active = False
    focus_engaged = False
    focused_baud: int | None = None
    focus_engage_reason: str | None = None
    focus_release_reason: str | None = None
    focus_disabled_reason: str | None = (
        "ONLY ONE BAUD SELECTED"
        if options.baud_focus_enabled and len(baud_order) < 2
        else None
    )
    tested_before_engage: int | None = None
    deferred_candidate_count = 0
    deferred_bauds: list[int] = []

    while True:
        if focus_active and focused_baud is not None:
            settings = next_unseen_candidate(grouped[focused_baud], tested)
            if settings is None:
                deferred = [
                    candidate
                    for candidate in quick_candidates
                    if candidate not in tested and candidate.baud != focused_baud
                ]
                deferred_candidate_count = len(deferred)
                deferred_bauds = [
                    baud
                    for baud in baud_order
                    if baud != focused_baud
                    and any(candidate.baud == baud for candidate in deferred)
                ]
                if deferred_candidate_count:
                    print("OTHER BAUDS DEFERRED BY CONFIDENCE RULE")
                    logger.info(
                        "baud focus deferred %s settings at bauds %s",
                        deferred_candidate_count,
                        deferred_bauds,
                    )
                break
        else:
            settings = next_unseen_candidate(quick_candidates, tested)
            if settings is None:
                break

        index = len(results) + 1
        result = run_candidate(
            serial_module=serial_module,
            index=index,
            total=len(quick_candidates),
            settings=settings,
            options=quick_options,
            payload=quick_payload,
            logger=logger,
            progress=None,
        )
        results.append(result)
        tested.add(settings)
        print("QUICK " + format_progress(result), flush=True)
        print(format_scan_eta(len(results), len(quick_candidates), started), flush=True)

        if focus_enabled:
            block_reason = result_blocks_baud_focus(result)
            if focus_active:
                if block_reason:
                    focus_active = False
                    focus_release_reason = block_reason
                    print_wrapped_value(
                        "",
                        f"QUICK BAUD FOCUS CANCELED: {block_reason}",
                    )
                    print("RETURNING TO FULL BAUD SWEEP")
                    logger.info(
                        "baud focus canceled for %s: %s",
                        focused_baud,
                        block_reason,
                    )
                else:
                    decision = select_baud_focus(results, baud_order, grouped, options)
                    if decision.disabled_reason:
                        focus_active = False
                        focus_release_reason = decision.disabled_reason
                        print_wrapped_value(
                            "",
                            "QUICK BAUD FOCUS CANCELED: "
                            f"{decision.disabled_reason}",
                        )
                        print("RETURNING TO FULL BAUD SWEEP")
                        logger.info(
                            "baud focus canceled for %s: %s",
                            focused_baud,
                            decision.disabled_reason,
                        )
                    elif decision.baud != focused_baud:
                        focus_active = False
                        focus_release_reason = "CONFIDENCE DROPPED"
                        print("QUICK BAUD FOCUS CANCELED: CONFIDENCE DROPPED")
                        print("RETURNING TO FULL BAUD SWEEP")
                        logger.info(
                            "baud focus canceled for %s: confidence dropped",
                            focused_baud,
                        )
            elif block_reason:
                logger.info(
                    "baud focus waiting; %s before focus on %s",
                    block_reason,
                    result.settings.label(),
                )
            else:
                decision = select_baud_focus(results, baud_order, grouped, options)
                if decision.disabled_reason:
                    focus_disabled_reason = decision.disabled_reason
                    print_wrapped_value(
                        "",
                        "QUICK BAUD FOCUS DISABLED: "
                        f"{decision.disabled_reason}",
                    )
                    logger.info("baud focus disabled: %s", decision.disabled_reason)
                elif decision.baud is not None:
                    focus_active = True
                    focus_engaged = True
                    focused_baud = decision.baud
                    focus_engage_reason = decision.reason
                    tested_before_engage = len(results)
                    print_wrapped_value(
                        "",
                        (
                            f"QUICK BAUD FOCUS ENGAGED: {focused_baud} "
                            f"{focus_engage_reason or ''}"
                        ).rstrip(),
                    )
                    logger.info(
                        "baud focus engaged for %s: %s",
                        focused_baud,
                        focus_engage_reason,
                    )

    elapsed_sec = time.monotonic() - started
    baud_focus_report = BaudFocusReport(
        enabled=options.baud_focus_enabled,
        engaged=focus_engaged,
        focused_baud=focused_baud,
        deferred_other_bauds=deferred_candidate_count > 0,
        deferred_candidate_count=deferred_candidate_count,
        deferred_bauds=deferred_bauds,
        engage_reason=focus_engage_reason,
        release_reason=focus_release_reason,
        disabled_reason=focus_disabled_reason,
        tested_before_engage=tested_before_engage,
    )
    selection = select_exploratory_candidates(
        results,
        candidates,
        elapsed_sec,
        baud_focus_report,
        phase0_liveness,
    )
    print_exploratory_summary(selection)
    logger.info(
        "quick exploratory completed; candidates=%s fallback=%s shortlist=%s narrowed=%s viable=%s baud_focus=%s",
        len(results),
        selection.fallback_reason,
        len(selection.shortlist_results),
        len(selection.narrowed_candidates),
        len(selection.viable_candidates),
        dataclass_to_jsonable(baud_focus_report),
    )
    return selection


def prompt_text(label: str, current: str) -> str:
    """Prompt for a string value, preserving current on blank input."""
    try:
        value = input(f"{label.upper()} [{current}]: ").strip()
    except EOFError:
        return current
    return current if value == "" else value


def prompt_int(label: str, current: int, minimum: int | None = None) -> int:
    """Prompt for an integer value, preserving current on blank input."""
    while True:
        try:
            value = input(f"{label.upper()} [{current}]: ").strip()
        except EOFError:
            return current
        if value == "":
            return current
        try:
            parsed = int(value)
        except ValueError:
            print("ENTER A WHOLE NUMBER.")
            continue
        if minimum is not None and parsed < minimum:
            print(f"ENTER A VALUE >= {minimum}.")
            continue
        return parsed


def prompt_float(label: str, current: float, minimum: float | None = None) -> float:
    """Prompt for a float value, preserving current on blank input."""
    while True:
        try:
            value = input(f"{label.upper()} [{current}]: ").strip()
        except EOFError:
            return current
        if value == "":
            return current
        try:
            parsed = float(value)
        except ValueError:
            print("ENTER A NUMBER.")
            continue
        if minimum is not None and parsed < minimum:
            print(f"ENTER A VALUE >= {minimum}.")
            continue
        return parsed


def prompt_path(label: str, current: Path) -> Path:
    """Prompt for a filesystem path, preserving current on blank input."""
    try:
        value = input(f"{label.upper()} [{current}]: ").strip()
    except EOFError:
        return current
    return current if value == "" else Path(value)


def prompt_yes_no(label: str, current: bool) -> bool:
    """Prompt for a yes/no value, preserving current on blank input."""
    default = "Y" if current else "N"
    while True:
        try:
            value = read_operator_input(f"{label.upper()} [{default}]: ").strip().lower()
        except EOFError:
            return current
        if value == "":
            return current
        if value in {"y", "yes"}:
            return True
        if value in {"n", "no"}:
            return False
        print("ENTER Y OR N.")


def prompt_yes_no_question(question: str, current: bool) -> bool:
    """Prompt for a yes/no answer using a full question string."""
    default = "Y" if current else "N"
    while True:
        try:
            value = read_operator_input(f"{question.upper()} (Y/N) [{default}]: ").strip().lower()
        except EOFError:
            return current
        if value == "":
            return current
        if value in {"y", "yes"}:
            return True
        if value in {"n", "no"}:
            return False
        print("ENTER Y OR N.")


def prompt_scan_mode() -> str:
    """Prompt for scan type. Blank defaults to conservative full scan."""
    while True:
        try:
            value = (
                input("SCAN TYPE: FULL OR QUICK [FULL]: ")
                .lstrip("\ufeff")
                .strip()
                .lower()
            )
        except EOFError:
            return "full"
        if value == "":
            return "full"
        if value in {"q", "quick", "a", "auto"}:
            return "quick"
        if value in {"f", "full", "m", "manual"}:
            return "full"
        print("ENTER F OR Q.")


def prompt_phase0_only_sweep() -> bool:
    """Ask whether start scan should run only the Phase 0 baud sweep."""
    return prompt_yes_no_question(
        "Run only Phase 0 baud liveness sweep?",
        False,
    )


def prompt_same_bank_settings() -> bool:
    """Ask whether both buffer switch banks should be treated as identical."""
    return prompt_yes_no_question(
        "Assume both buffer switch banks use the same serial setting?",
        True,
    )


def print_menu_help() -> None:
    """Print short help for the interactive CLI."""
    print_paged_lines(
        [
            *banner_lines(),
            "START: PYTHON SERIAL_PROBE.PY",
            "",
            "HELP",
            "",
            "PURPOSE",
            "  FIND SERIAL SWITCH SETTINGS FOR A PRINTER BUFFER.",
            "",
            "METHOD",
            "  SEND KNOWN ASCII TEXT TO INPUT PORT.",
            "  READ OUTPUT PORT.",
            "  TEST EACH SELECTED SERIAL SETTING.",
            "  RANK BY MATCH QUALITY.",
            "  USE 11 MEMORY TEST AFTER A GOOD SETTING IS FOUND.",
            "",
            "OPERATOR NOTES:",
            "  START SCAN:      FIRST ASKS WHETHER TO RUN ONLY PHASE 0.",
            "  BANKS:           SAME BANK MODE IS OLD BEHAVIOR; DUAL BANK TESTS PAIRS.",
            "  SCAN TYPE:       FULL OR QUICK AT SCAN START; BLANK=FULL.",
            "  FULL MODE:       MOST RELIABLE FOR SWITCH MAPPING; QUICK MODE ASKS.",
            "  QUICK MODE:      RUNS DISCOVERY AND MAY NARROW PHASE 2.",
            "  DEVICE PATH:     COM1 -> BUFFER INPUT -> BUFFER OUTPUT -> COM5.",
            "  PORT SETTINGS:   PROGRAM SETS COM PORTS; DEVICE MANAGER IS IGNORED.",
            "  TEST SIZE:       BYTES SENT FOR EACH SETTING.",
            "  TEST COUNT:      NUMBER OF TRIES PER SETTING.",
            "  DISCOVERY:       QUICK=YES; FULL ASKS; FIXED INTERNAL SETTINGS.",
            "  PHASE 0:         QUICK TESTS EACH BAUD AT 8E1 FLOW=NONE.",
            "  TURBO:           FASTER DISCOVERY TIMING.",
            "  QUICK BAUD FOCUS: QUICK-ONLY SPEED-UP; FULL DOES NOT NEED IT.",
            "  ASK ON MATCH:    PAUSE AFTER PASS; ASK CONTINUE.",
            "  FLOW VALIDATE:   AFTER BEST FRAME, TEST NONE/XON/RTS/DSR BEHAVIOR.",
            "  CLEAR OUTPUT:    DISCARD OLD BUFFER DATA FIRST.",
            "  MAX CLEAR:       DEFAULT 32768 BYTES.",
            "  TOP ROWS:        BEST RESULTS SHOWN AT END.",
            "  MEMORY TEST:     COMMAND 11 AFTER SCAN.",
            "  BREAK:           CTRL+C ASKS RESUME, REPORT, MENU, OR QUIT.",
            "  AFTER SCAN:      RUN AGAIN, MAIN MENU, OR QUIT.",
        ]
    )


def setting_lines(label: str, value: object) -> list[str]:
    """Return aligned current-settings rows."""
    prefix = f"  {label.upper():<20} "
    return wrapped_value_lines(prefix, value)


def print_setting(label: str, value: object) -> None:
    """Print one aligned current-settings row."""
    for line in setting_lines(label, value):
        print(line)


def print_configuration(options: ScanOptions) -> None:
    """Print the current interactive menu configuration."""
    try:
        bauds = available_bauds(options.min_baud, options.max_baud)
        baud_order = scan_bauds(options.min_baud, options.max_baud)
        candidates = generate_candidates(options.min_baud, options.max_baud)
        wire = estimate_scan_wire_seconds(options)
        overhead = estimate_scan_overhead_seconds(options)
        range_error: str | None = None
    except ValueError as exc:
        bauds = []
        baud_order = []
        candidates = []
        wire = 0.0
        overhead = 0.0
        range_error = str(exc)
    lines: list[str] = ["", *banner_lines(), "CURRENT SETTINGS"]
    lines.extend(setting_lines("SCAN TYPE:", "ASK AT START; BLANK=FULL"))
    lines.extend(setting_lines("PORTS:", f"{options.in_port} -> {options.out_port}"))
    lines.extend(
        setting_lines("BAUD RANGE:", f"{options.min_baud}..{options.max_baud}")
    )
    lines.extend(setting_lines("SETTINGS:", len(candidates)))
    if baud_order:
        lines.extend(
            setting_lines("BAUD ORDER:", f"{baud_order[0]} DOWN TO {baud_order[-1]}")
        )
    if range_error:
        lines.extend(setting_lines("RANGE ERROR:", range_error))
    lines.extend(
        setting_lines(
            "COUNT FORMULA:",
            f"{len(bauds)} BAUD X "
            f"{len(DATA_BITS)} DATA X {len(PARITIES)} PARITY X "
            f"{len(STOP_BITS)} STOP X {len(FLOW_CONTROLS)} FLOW",
        )
    )
    lines.extend(setting_lines("TEST BYTES:", f"{options.payload_bytes} BYTES"))
    lines.extend(setting_lines("TEST COUNT:", options.bursts))
    lines.extend(
        setting_lines(
            "QUICK MODE:",
            f"ASK AT START; FIXED INTERNAL {exploratory_fixed_settings_label()}",
        )
    )
    lines.extend(setting_lines("PHASE 0:", phase0_fixed_settings_label()))
    lines.extend(
        setting_lines("QUICK BAUD FOCUS:", baud_focus_settings_label(options))
    )
    lines.extend(
        setting_lines(
            "TURBO DISCOVERY:",
            "ON" if options.turbo_discovery_enabled else "OFF",
        )
    )
    lines.extend(
        setting_lines(
            "EFFECTIVE TIMING:",
            effective_timing_range_label(options, candidates),
        )
    )
    lines.extend(
        setting_lines(
            "CANDIDATE ORDER:",
            (
                "TURBO PRIORITY; LOW-VALUE FRAMES DEFERRED"
                if options.turbo_discovery_enabled
                else "EXHAUSTIVE PROGRAM ORDER"
            ),
        )
    )
    lines.extend(
        setting_lines("ASK ON MATCH:", "YES" if options.ask_on_top_match else "NO")
    )
    lines.extend(
        setting_lines(
            "AUTO VALIDATE:",
            (
                "OFF"
                if not options.auto_validate_top_matches
                else (
                    f"ON, SIZE1={options.validate_size_1_bytes} BYTES, "
                    f"SIZE2={options.validate_size_2_tie_bytes} BYTES"
                    if options.validate_size_2_tie_bytes > 0
                    else f"ON, SIZE1={options.validate_size_1_bytes} BYTES, SIZE2=OFF"
                )
            ),
        )
    )
    lines.extend(
        setting_lines(
            "FLOW VALIDATE:",
            (
                "OFF"
                if not options.auto_validate_flow_control
                else f"ON, SIZE={options.flow_validate_size_bytes} BYTES"
            ),
        )
    )
    lines.extend(setting_lines("READ WAIT:", f"{options.read_timeout:.2f}S"))
    lines.extend(setting_lines("OPEN PAUSE:", f"{options.settle_ms} MS"))
    lines.extend(
        setting_lines(
            "CLEAR OUTPUT:",
            (
                "NO"
                if options.no_pre_drain
                else (
                    f"YES, QUIET={options.pre_drain_quiet:.2f}S, "
                    f"LIMIT={options.pre_drain_timeout:.2f}S, "
                    f"MAX={options.max_drain_bytes} BYTES"
                )
            ),
        )
    )
    lines.extend(setting_lines("TOP ROWS:", options.top))
    lines.extend(setting_lines("MEMORY TEST:", "USE 11 AFTER SCAN"))
    lines.extend(setting_lines("REPORT FILE:", options.text_report))
    lines.extend(
        setting_lines("SWITCH NOTE:", options.switch_note or "(ASK AT SCAN START)")
    )
    lines.extend(setting_lines("LOG FILE:", options.log_file))
    lines.extend(setting_lines("SEND TIME:", format_duration(wire).upper()))
    lines.extend(
        setting_lines("WAIT TIME:", f"{format_duration(overhead).upper()} IF QUIET")
    )
    lines.extend(setting_lines("TOTAL EST.:", format_duration(wire + overhead).upper()))
    print_paged_lines(lines)


def configure_baud_range(options: ScanOptions) -> ScanOptions:
    """Prompt for the baud range."""
    print("AVAILABLE BAUD RATES:")
    print(", ".join(str(baud) for baud in BAUD_RATES))
    print("SCAN ORDER: FASTEST SELECTED BAUD FIRST.")
    min_baud = prompt_int("MINIMUM BAUD", options.min_baud)
    max_baud = prompt_int("MAXIMUM BAUD", options.max_baud)
    return dataclasses.replace(options, min_baud=min_baud, max_baud=max_baud)


def configure_payload(options: ScanOptions) -> ScanOptions:
    """Prompt for payload and burst settings."""
    print(f"MINIMUM TEST MESSAGE IS {minimum_payload_size()} BYTES.")
    payload_bytes = prompt_int(
        "TEST MESSAGE SIZE IN BYTES",
        options.payload_bytes,
        minimum=minimum_payload_size(),
    )
    bursts = prompt_int("NUMBER OF TESTS PER SETTING", options.bursts, minimum=1)
    ask_on_top_match = prompt_yes_no(
        "ASK WHETHER TO CONTINUE AFTER TOP MATCH",
        options.ask_on_top_match,
    )
    auto_validate = prompt_yes_no(
        "AUTO VALIDATE TOP MATCHES AFTER SCAN",
        options.auto_validate_top_matches,
    )
    auto_flow_validate = prompt_yes_no(
        "AUTO VALIDATE FLOW CONTROL AFTER SCAN",
        options.auto_validate_flow_control,
    )
    validate_size_1 = options.validate_size_1_bytes
    validate_size_2 = options.validate_size_2_tie_bytes
    flow_validate_size = options.flow_validate_size_bytes
    if auto_validate:
        validate_size_1 = prompt_int(
            "VALIDATE SIZE 1 BYTES",
            options.validate_size_1_bytes,
            minimum=minimum_payload_size(),
        )
        validate_size_2 = prompt_int(
            "VALIDATE SIZE 2 ON TIE (0=OFF)",
            options.validate_size_2_tie_bytes,
            minimum=0,
        )
        if 0 < validate_size_2 < minimum_payload_size():
            print(f"VALUE {validate_size_2} TOO SMALL. USING {minimum_payload_size()}.")
            validate_size_2 = minimum_payload_size()
    if auto_flow_validate:
        flow_validate_size = prompt_int(
            "FLOW VALIDATE SIZE BYTES",
            options.flow_validate_size_bytes,
            minimum=minimum_payload_size(),
        )
    return dataclasses.replace(
        options,
        payload_bytes=payload_bytes,
        bursts=bursts,
        ask_on_top_match=ask_on_top_match,
        auto_validate_top_matches=auto_validate,
        validate_size_1_bytes=validate_size_1,
        validate_size_2_tie_bytes=validate_size_2,
        auto_validate_flow_control=auto_flow_validate,
        flow_validate_size_bytes=flow_validate_size,
    )


def configure_timing(options: ScanOptions) -> ScanOptions:
    """Prompt for timing settings."""
    print("DISCOVERY TIMING")
    print("TURBO APPLIES ONLY TO SCAN DISCOVERY.")
    print("VALIDATION AND MEMORY TESTS STAY CONSERVATIVE.")
    turbo_enabled = prompt_yes_no(
        "TURBO DISCOVERY MODE",
        options.turbo_discovery_enabled,
    )
    read_timeout = prompt_float(
        "OUTPUT WAIT AFTER SEND, SECONDS",
        options.read_timeout,
        0.1,
    )
    settle_ms = prompt_int("PAUSE AFTER OPENING PORTS, MS", options.settle_ms, 0)
    pre_drain_quiet = prompt_float(
        "QUIET TIME BEFORE SEND, SECONDS",
        options.pre_drain_quiet,
        0.05,
    )
    pre_drain_timeout = prompt_float(
        "MAX TIME CLEARING OLD OUTPUT, SECONDS",
        options.pre_drain_timeout,
        0.0,
    )
    max_drain_bytes = prompt_int(
        "MAX OLD BYTES TO CLEAR BEFORE STALE",
        options.max_drain_bytes,
        1,
    )
    progress_interval = prompt_float(
        "SCREEN UPDATE INTERVAL, SECONDS",
        options.progress_interval,
        0.1,
    )
    return dataclasses.replace(
        options,
        turbo_discovery_enabled=turbo_enabled,
        read_timeout=read_timeout,
        settle_ms=settle_ms,
        pre_drain_quiet=pre_drain_quiet,
        pre_drain_timeout=pre_drain_timeout,
        max_drain_bytes=max_drain_bytes,
        progress_interval=progress_interval,
    )


def configure_pre_drain(options: ScanOptions) -> ScanOptions:
    """Prompt for stale-output drain settings."""
    enabled = prompt_yes_no("CLEAR OLD OUTPUT BEFORE EACH TEST", not options.no_pre_drain)
    pre_drain_quiet = prompt_float("QUIET TIME BEFORE SEND, SECONDS", options.pre_drain_quiet, 0.05)
    pre_drain_timeout = prompt_float("TIME LIMIT FOR CLEARING OLD OUTPUT, SECONDS", options.pre_drain_timeout, 0.0)
    max_drain_bytes = prompt_int("MAX OLD BYTES TO CLEAR BEFORE STALE", options.max_drain_bytes, 1)
    return dataclasses.replace(
        options,
        no_pre_drain=not enabled,
        pre_drain_quiet=pre_drain_quiet,
        pre_drain_timeout=pre_drain_timeout,
        max_drain_bytes=max_drain_bytes,
    )


def configure_reports(options: ScanOptions) -> ScanOptions:
    """Prompt for result and report settings."""
    top = prompt_int("NUMBER OF TOP ROWS TO SHOW", options.top, 1)
    text_report = prompt_path("APPEND TEXT REPORT FILE", options.text_report)
    switch_note = prompt_text("DEFAULT SWITCH/JUMPER NOTE", options.switch_note)
    log_file = prompt_path("LOG FILE", options.log_file)
    return dataclasses.replace(
        options,
        top=top,
        text_report=text_report,
        switch_note=switch_note,
        log_file=log_file,
    )


def configure_baud_focus(options: ScanOptions) -> ScanOptions:
    """Prompt for exploratory baud focus settings."""
    print("QUICK BAUD FOCUS")
    print("USED ONLY BY QUICK SCAN.")
    print("IF ONE BAUD IS CLEARLY BEST, QUICK SCAN MAY DEFER OTHER BAUDS.")
    print("FULL SCAN DOES NOT USE THIS.")
    enabled = prompt_yes_no("QUICK BAUD FOCUS ENABLED", options.baud_focus_enabled)
    score_threshold = prompt_float(
        "FOCUS SCORE",
        options.baud_focus_strong_score_threshold,
        0.0,
    )
    lead_gap = prompt_float(
        "FOCUS LEAD GAP",
        options.baud_focus_lead_gap_threshold,
        0.0,
    )
    min_strong = prompt_int(
        "FOCUS GOOD COUNT",
        options.baud_focus_min_strong_results,
        1,
    )
    min_samples = prompt_int(
        "FOCUS SAMPLE COUNT",
        options.baud_focus_min_samples,
        1,
    )
    if score_threshold > 100.0:
        print("VALUE TOO HIGH. USING 100.0.")
        score_threshold = 100.0
    return dataclasses.replace(
        options,
        baud_focus_enabled=enabled,
        baud_focus_strong_score_threshold=score_threshold,
        baud_focus_lead_gap_threshold=lead_gap,
        baud_focus_min_strong_results=min_strong,
        baud_focus_min_samples=min_samples,
    )


def prompt_parity(current: str) -> str:
    """Prompt for a parity value."""
    choices = {
        "n": "none",
        "none": "none",
        "e": "even",
        "even": "even",
        "o": "odd",
        "odd": "odd",
        "m": "mark",
        "mark": "mark",
        "s": "space",
        "space": "space",
    }
    while True:
        try:
            value = input(f"PARITY N/E/O/M/S [{current[0].upper()}]: ").strip().lower()
        except EOFError:
            return current
        if value == "":
            return current
        if value in choices:
            return choices[value]
        print("ENTER N, E, O, M, OR S.")


def prompt_flow_control(current: str) -> str:
    """Prompt for a flow-control mode."""
    flow_options = ["none", "xon/xoff", "rts/cts", "dsr/dtr"]
    print("FLOW CONTROL")
    print("  1. NONE")
    print("  2. XON/XOFF")
    print("  3. RTS/CTS")
    print("  4. DSR/DTR")
    current_index = flow_options.index(current) + 1
    while True:
        try:
            choice = input(f"SELECT FLOW CONTROL [{current_index}]: ").strip()
        except EOFError:
            return current
        if choice == "":
            return current
        if choice in {"1", "2", "3", "4"}:
            return flow_options[int(choice) - 1]
        print("ENTER 1, 2, 3, OR 4.")


def prompt_serial_setting(default: SerialSettings) -> SerialSettings:
    """Prompt for the known-good serial setting used by the memory test."""
    print()
    print_banner()
    print("MEMORY TEST SERIAL SETTING")
    print("ENTER SETTING FROM SCAN REPORT.")
    baud = prompt_int("BAUD RATE", default.baud, 1)
    while baud not in BAUD_RATES:
        print("BAUD RATE NOT IN PROGRAM LIST.")
        baud = prompt_int("BAUD RATE", default.baud, 1)
    data_bits = prompt_int("DATA BITS, 7 OR 8", default.data_bits, 7)
    while data_bits not in DATA_BITS:
        print("ENTER 7 OR 8.")
        data_bits = prompt_int("DATA BITS, 7 OR 8", default.data_bits, 7)
    parity = prompt_parity(default.parity)
    stop_bits = prompt_int("STOP BITS, 1 OR 2", default.stop_bits, 1)
    while stop_bits not in STOP_BITS:
        print("ENTER 1 OR 2.")
        stop_bits = prompt_int("STOP BITS, 1 OR 2", default.stop_bits, 1)
    flow_control = prompt_flow_control(default.flow_control)
    return SerialSettings(baud, data_bits, parity, stop_bits, flow_control)


def prompt_memory_sizes() -> list[int]:
    """Prompt for memory test sizes."""
    print()
    print_banner()
    print("MEMORY TEST SIZE")
    print("  1. 16K QUICK TEST")
    print("  2. 16K, 32K, 64K")
    print("  3. 4K THROUGH 64K")
    print("  4. CUSTOM MAXIMUM, IN K")
    while True:
        try:
            choice = input("ENTER SELECTION [2]: ").strip()
        except EOFError:
            return [16 * 1024, 32 * 1024, 64 * 1024]
        if choice == "":
            choice = "2"
        if choice == "1":
            return [16 * 1024]
        if choice == "2":
            return [16 * 1024, 32 * 1024, 64 * 1024]
        if choice == "3":
            return [
                4 * 1024,
                8 * 1024,
                12 * 1024,
                16 * 1024,
                20 * 1024,
                24 * 1024,
                32 * 1024,
                48 * 1024,
                64 * 1024,
            ]
        if choice == "4":
            max_k = prompt_int("MAXIMUM SIZE IN K", 64, 1)
            return [size_k * 1024 for size_k in range(4, max_k + 1, 4)]
        print("ENTER 1, 2, 3, OR 4.")


def prompt_memory_method() -> str:
    """Prompt for the memory test method."""
    print()
    print_banner()
    print("MEMORY TEST METHOD")
    print("  1. HOLD OUTPUT, THEN RELEASE")
    print("  2. READ WHILE SENDING")
    print()
    print("USE 1 WITH OFF LINE, HOLD, OR PAUSE CONTROL.")
    print("USE 2 WHEN OUTPUT CANNOT BE HELD.")
    while True:
        try:
            choice = input("ENTER SELECTION [1]: ").strip()
        except EOFError:
            return "hold-release"
        if choice == "":
            choice = "1"
        if choice == "1":
            return "hold-release"
        if choice == "2":
            return "live-transfer"
        print("ENTER 1 OR 2.")


def wait_for_operator(message: str) -> None:
    """Pause for operator action in the interactive terminal."""
    try:
        input(message)
    except EOFError:
        return


def write_payload_only(
    in_serial: Any,
    settings: SerialSettings,
    payload: ProbePayload,
    progress_interval: float,
    prefix: str,
    logger: logging.Logger,
) -> tuple[int, str | None, float]:
    """Write a payload without reading the output port."""
    started = time.monotonic()
    chunk_size = write_chunk_size(settings)
    bytes_sent = 0
    error: str | None = None
    expected = payload.data
    estimated = estimated_transmit_seconds(settings, len(expected))
    console_progress(
        f"{prefix}: SEND {len(expected)} BYTES "
        f"(CHUNK={chunk_size}, ABOUT {format_duration(estimated)})"
    )
    try:
        next_progress_at = time.monotonic() + max(progress_interval, 0.1)
        while bytes_sent < len(expected):
            chunk = expected[bytes_sent : bytes_sent + chunk_size]
            written = in_serial.write(chunk)
            if written is None:
                written = len(chunk)
            if written <= 0:
                raise RuntimeError("serial write returned zero bytes")
            bytes_sent += int(written)
            now = time.monotonic()
            if now >= next_progress_at:
                percent = (bytes_sent / len(expected)) * 100.0
                console_progress(
                    f"{prefix}: WRITING {bytes_sent}/{len(expected)} BYTES "
                    f"({percent:5.1f}%)"
                )
                next_progress_at = now + max(progress_interval, 0.1)
    except Exception as exc:  # pyserial raises driver-specific subclasses.
        error = str(exc)
        logger.debug("%s write failed: %s", prefix, error)
    return bytes_sent, error, time.monotonic() - started


def read_until_quiet(
    out_serial: Any,
    settings: SerialSettings,
    expected_bytes: int,
    read_timeout: float,
    progress_interval: float,
    prefix: str,
    logger: logging.Logger,
) -> tuple[bytes, str | None, float]:
    """Read output bytes until the line goes quiet."""
    started = time.monotonic()
    received = bytearray()
    last_data_time = time.monotonic()
    read_timeout = max(read_timeout, 0.1)
    max_seconds = max(
        estimated_transmit_seconds(settings, expected_bytes) * 2.0 + read_timeout + 10.0,
        read_timeout + 10.0,
    )
    deadline = started + max_seconds
    next_progress_at = time.monotonic() + max(progress_interval, 0.1)
    error: str | None = None
    console_progress(f"{prefix}: READ OUTPUT UNTIL QUIET FOR {read_timeout:.1f}S")
    while True:
        try:
            waiting = getattr(out_serial, "in_waiting", 0)
            read_size = min(max(int(waiting), 1), 4096)
            chunk = out_serial.read(read_size)
        except Exception as exc:  # pyserial raises driver-specific subclasses.
            error = str(exc)
            logger.debug("%s read failed: %s", prefix, error)
            break

        now = time.monotonic()
        if chunk:
            received.extend(chunk)
            last_data_time = now
        elif (now - last_data_time) >= read_timeout:
            break

        if now >= next_progress_at:
            silence = max(0.0, now - last_data_time)
            console_progress(
                f"{prefix}: READING RECEIVED={len(received)} BYTES, "
                f"QUIET={silence:.1f}/{read_timeout:.1f}S"
            )
            next_progress_at = now + max(progress_interval, 0.1)

        if now >= deadline:
            error = (
                "READ STOPPED BEFORE OUTPUT QUIET "
                f"AFTER {format_duration(max_seconds)}"
            )
            logger.debug("%s", error)
            break

    return bytes(received), error, time.monotonic() - started


def read_for_fixed_window(
    out_serial: Any,
    seconds: float,
    progress_interval: float,
    prefix: str,
    logger: logging.Logger,
) -> tuple[bytes, str | None, float]:
    """Read any output that arrives during a fixed observation window."""
    started = time.monotonic()
    deadline = started + max(seconds, 0.1)
    next_progress_at = started + max(progress_interval, 0.1)
    received = bytearray()
    error: str | None = None
    console_progress(f"{prefix}: OBSERVE HELD OUTPUT FOR {seconds:.1f}S")
    while time.monotonic() < deadline:
        try:
            waiting = getattr(out_serial, "in_waiting", 0)
            read_size = min(max(int(waiting), 1), 4096)
            chunk = out_serial.read(read_size)
        except Exception as exc:  # pyserial raises driver-specific subclasses.
            error = str(exc)
            logger.debug("%s hold observation failed: %s", prefix, error)
            break

        if chunk:
            received.extend(chunk)

        now = time.monotonic()
        if now >= next_progress_at:
            console_progress(
                f"{prefix}: HELD OUTPUT SEEN={len(received)} BYTES"
            )
            next_progress_at = now + max(progress_interval, 0.1)

    return bytes(received), error, time.monotonic() - started


def run_memory_hold_release_test(
    serial_module: Any,
    index: int,
    total: int,
    size_bytes: int,
    settings: SerialSettings,
    options: ScanOptions,
    logger: logging.Logger,
) -> MemoryTestResult:
    """Run one memory test by holding output during the input transfer."""
    payload = generate_payload(size_bytes)
    started = time.monotonic()
    prefix = f"[MEM {index:02d}/{total:02d} {settings.label()} {byte_size_label(size_bytes)}]"
    console_progress(border_line(PROGRESS_WIDTH))
    console_progress(
        bordered_text(
            f"MEMORY TEST: {byte_size_label(size_bytes)} ({index}/{total})",
            PROGRESS_WIDTH,
        )
    )
    console_progress(border_line(PROGRESS_WIDTH))
    try:
        with open_serial_port(
            serial_module,
            options.out_port,
            settings,
            max(options.read_timeout, 2.0),
        ) as out_serial:
            with open_serial_port(
                serial_module,
                options.in_port,
                settings,
                max(options.read_timeout, 2.0),
            ) as in_serial:
                reset_serial_buffers(out_serial)
                reset_serial_buffers(in_serial)
                time.sleep(options.settle_ms / 1000.0)

                drain = DrainResult(0, 0.0, True, "disabled", None)
                if not options.no_pre_drain:
                    drain = drain_output_until_quiet(
                        out_serial=out_serial,
                        quiet_seconds=options.pre_drain_quiet,
                        max_seconds=max(options.pre_drain_timeout, 2.0),
                        max_bytes=options.max_drain_bytes,
                        progress_interval=options.progress_interval,
                        progress=console_progress,
                        prefix=prefix,
                        logger=logger,
                    )
                    if not drain.quiet:
                        empty_score = score_received(payload.data, b"")
                        error = (
                            "OUTPUT DID NOT GO QUIET BEFORE MEMORY TEST "
                            f"(REASON={drain.reason.upper()}, CLEARED={drain.bytes_drained})"
                        )
                        if drain.error:
                            error = f"{error}: {drain.error}"
                        return MemoryTestResult(
                            size_bytes=size_bytes,
                            size_label=byte_size_label(size_bytes),
                            method="hold-release",
                            settings=settings,
                            bytes_sent=0,
                            bytes_received=0,
                            bytes_cleared_before=drain.bytes_drained,
                            bytes_seen_before_release=0,
                            score=0.0,
                            indicator="STALE",
                            status="stale-output",
                            error=error,
                            elapsed_sec=time.monotonic() - started,
                            metrics=empty_score.metrics,
                        )

                wait_for_operator(
                    "PUT BUFFER OUTPUT ON HOLD OR OFF LINE, THEN PRESS ENTER: "
                )
                reset_serial_buffers(out_serial)
                bytes_sent, write_error, _write_elapsed = write_payload_only(
                    in_serial=in_serial,
                    settings=settings,
                    payload=payload,
                    progress_interval=options.progress_interval,
                    prefix=prefix,
                    logger=logger,
                )
                bytes_seen_before_release = int(getattr(out_serial, "in_waiting", 0))
                if bytes_seen_before_release:
                    console_progress(
                        f"{prefix}: {bytes_seen_before_release} BYTES ARRIVED BEFORE RELEASE"
                    )
                wait_for_operator(
                    "RELEASE BUFFER OUTPUT OR PUT IT ON LINE, THEN PRESS ENTER: "
                )
                received_bytes, read_error, _read_elapsed = read_until_quiet(
                    out_serial=out_serial,
                    settings=settings,
                    expected_bytes=size_bytes,
                    read_timeout=max(options.read_timeout, 2.0),
                    progress_interval=options.progress_interval,
                    prefix=prefix,
                    logger=logger,
                )
    except Exception as exc:
        empty_score = score_received(payload.data, b"")
        logger.exception("memory test %s failed", byte_size_label(size_bytes))
        return MemoryTestResult(
            size_bytes=size_bytes,
            size_label=byte_size_label(size_bytes),
            method="hold-release",
            settings=settings,
            bytes_sent=0,
            bytes_received=0,
            bytes_cleared_before=0,
            bytes_seen_before_release=0,
            score=0.0,
            indicator="ERROR",
            status="error",
            error=str(exc),
            elapsed_sec=time.monotonic() - started,
            metrics=empty_score.metrics,
        )

    score = score_received(payload.data, received_bytes)
    error = write_error or read_error
    if write_error and bytes_sent < size_bytes:
        status = "partial-write"
    elif error:
        status = "error"
    elif bytes_sent < size_bytes:
        status = "partial-write"
    elif not received_bytes:
        status = "no-data"
    elif score.score >= 99.0:
        status = "exact"
    elif score.score >= 90.0:
        status = "strong"
    elif score.score >= 50.0:
        status = "partial"
    else:
        status = "weak"
    indicator = result_indicator(score.score, status, error)
    console_progress(
        f"{prefix}: RESULT {indicator} SCORE={score.score:.2f} ({status.upper()}); "
        f"SENT={bytes_sent}, RECEIVED={len(received_bytes)}, "
        f"CLEARED={drain.bytes_drained}, EARLY={bytes_seen_before_release}, "
        f"EXACT={score.metrics.exact_byte_match_ratio:.3f}, "
        f"LINES={score.metrics.line_integrity_ratio:.3f}"
    )
    return MemoryTestResult(
        size_bytes=size_bytes,
        size_label=byte_size_label(size_bytes),
        method="hold-release",
        settings=settings,
        bytes_sent=bytes_sent,
        bytes_received=len(received_bytes),
        bytes_cleared_before=drain.bytes_drained,
        bytes_seen_before_release=bytes_seen_before_release,
        score=score.score,
        indicator=indicator,
        status=status,
        error=error,
        elapsed_sec=time.monotonic() - started,
        metrics=score.metrics,
    )


def run_memory_size_test(
    serial_module: Any,
    index: int,
    total: int,
    size_bytes: int,
    settings: SerialSettings,
    options: ScanOptions,
    logger: logging.Logger,
    method: str,
) -> MemoryTestResult:
    """Run one memory transfer size using a known serial setting."""
    if method == "hold-release":
        return run_memory_hold_release_test(
            serial_module=serial_module,
            index=index,
            total=total,
            size_bytes=size_bytes,
            settings=settings,
            options=options,
            logger=logger,
        )

    payload = generate_payload(size_bytes)
    started = time.monotonic()
    console_progress(border_line(PROGRESS_WIDTH))
    console_progress(
        bordered_text(
            f"MEMORY TEST: {byte_size_label(size_bytes)} ({index}/{total})",
            PROGRESS_WIDTH,
        )
    )
    console_progress(border_line(PROGRESS_WIDTH))
    try:
        with open_serial_port(
            serial_module,
            options.out_port,
            settings,
            max(options.read_timeout, 2.0),
        ) as out_serial:
            with open_serial_port(
                serial_module,
                options.in_port,
                settings,
                max(options.read_timeout, 2.0),
            ) as in_serial:
                trial = execute_burst(
                    in_serial=in_serial,
                    out_serial=out_serial,
                    settings=settings,
                    payload=payload,
                    burst_index=1,
                    burst_total=1,
                    candidate_index=index,
                    candidate_total=total,
                    read_timeout=max(options.read_timeout, 2.0),
                    completion_quiet=max(options.read_timeout, 2.0),
                    settle_ms=options.settle_ms,
                    progress_interval=options.progress_interval,
                    no_pre_drain=options.no_pre_drain,
                    pre_drain_timeout=max(options.pre_drain_timeout, 2.0),
                    pre_drain_quiet=options.pre_drain_quiet,
                    max_drain_bytes=options.max_drain_bytes,
                    logger=logger,
                    progress=console_progress,
                )
    except Exception as exc:
        empty_score = score_received(payload.data, b"")
        logger.exception("memory test %s failed", byte_size_label(size_bytes))
        return MemoryTestResult(
            size_bytes=size_bytes,
            size_label=byte_size_label(size_bytes),
            method="live-transfer",
            settings=settings,
            bytes_sent=0,
            bytes_received=0,
            bytes_cleared_before=0,
            bytes_seen_before_release=0,
            score=0.0,
            indicator="ERROR",
            status="error",
            error=str(exc),
            elapsed_sec=time.monotonic() - started,
            metrics=empty_score.metrics,
        )

    return MemoryTestResult(
        size_bytes=size_bytes,
        size_label=byte_size_label(size_bytes),
        method="live-transfer",
        settings=settings,
        bytes_sent=trial.bytes_sent,
        bytes_received=trial.bytes_received,
        bytes_cleared_before=trial.bytes_drained_before,
        bytes_seen_before_release=0,
        score=trial.score,
        indicator=result_indicator(trial.score, trial.status, trial.error),
        status=trial.status,
        error=trial.error,
        elapsed_sec=trial.elapsed_sec,
        metrics=trial.metrics,
    )


def memory_test_interpretation(results: Sequence[MemoryTestResult]) -> str:
    """Return a short memory-test conclusion."""
    if not results:
        return "No memory tests were run."
    clean = [
        result
        for result in results
        if result.score >= 99.0
        and result.metrics.missing_bytes == 0
        and result.metrics.extra_bytes == 0
        and result.bytes_sent == result.size_bytes
        and result.bytes_received == result.size_bytes
    ]
    if not clean:
        return "No clean memory transfer was confirmed."
    largest = max(clean, key=lambda result: result.size_bytes)
    method = largest.method
    early_output = any(result.bytes_seen_before_release > 0 for result in clean)
    max_tested = max(result.size_bytes for result in results)
    if method == "live-transfer" or early_output:
        return (
            f"Largest clean transfer is {largest.size_label}. "
            "This checks clean data flow, not stored memory."
        )
    if largest.size_bytes == max_tested:
        return f"Buffer held at least {largest.size_label} cleanly."
    failed_larger = any(
        result.size_bytes > largest.size_bytes and result.indicator not in {"PASS", "GOOD"}
        for result in results
    )
    if failed_larger:
        return f"Likely installed memory is near {largest.size_label}."
    return f"Largest clean transfer is {largest.size_label}."


def print_memory_report(
    settings: SerialSettings,
    results: Sequence[MemoryTestResult],
    text_report: Path,
    log_file: Path,
) -> None:
    """Print the final memory test report."""
    print()
    print_report_title("MEMORY TEST REPORT")
    print(border_line(REPORT_WIDTH))
    print(bordered_text("SERIAL SETTING USED", REPORT_WIDTH))
    print(border_line(REPORT_WIDTH))
    print(f"    BAUD RATE:         {settings.baud}")
    print(f"    DATA BITS:         {settings.data_bits}")
    print(f"    PARITY:            {parity_name(settings.parity)} ({settings.parity_code()})")
    print(f"    STOP BITS:         {settings.stop_bits}")
    print(f"    FLOW CONTROL:      {flow_control_name(settings.flow_control)}")
    print_wrapped_value("    SETTING:           ", settings.label())
    print()
    print(f"  RESULT:              {memory_test_interpretation(results).upper()}")
    print(border_line(REPORT_WIDTH))
    print("SIZE  METHOD RESULT SCORE   SENT   READ EARLY CLEAR  MISS EXTRA  TIME")
    print(border_line(REPORT_WIDTH))
    for result in results:
        print(
            f"{result.size_label:<5} "
            f"{result.method[:6].upper():<6} "
            f"{result.indicator:<6} "
            f"{result.score:>5.1f} "
            f"{result.bytes_sent:>6} "
            f"{result.bytes_received:>6} "
            f"{result.bytes_seen_before_release:>5} "
            f"{result.bytes_cleared_before:>5} "
            f"{result.metrics.missing_bytes:>5} "
            f"{result.metrics.extra_bytes:>5} "
            f"{format_duration(result.elapsed_sec):>6}"
        )
    print(border_line(REPORT_WIDTH))
    if any(result.method == "live-transfer" for result in results):
        print("  NOTE: READ-WHILE-SENDING DOES NOT PROVE RAM SIZE.")
    if any(result.bytes_seen_before_release > 0 for result in results):
        print("  NOTE: EARLY BYTES ARRIVED BEFORE RELEASE.")
    print()
    print_report_title("MEMORY TEST FILES")
    print_wrapped_value("  TEXT REPORT: ", f"{text_report} (APPENDED)")
    print_wrapped_value("  DEBUG LOG:   ", log_file)
    print(border_line(REPORT_WIDTH))


def write_memory_text_report(
    text_report: Path,
    settings: SerialSettings,
    results: Sequence[MemoryTestResult],
) -> None:
    """Append a compact memory test section to the text report."""
    text_report.parent.mkdir(parents=True, exist_ok=True)
    created = dt.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")
    lines = [
        "",
        border_line(REPORT_WIDTH),
        bordered_text("SERIAL PROBE MEMORY TEST", REPORT_WIDTH),
        border_line(REPORT_WIDTH),
        f"APPENDED:        {created}",
        f"SETTING:         {settings.label()}",
        f"RESULT:          {memory_test_interpretation(results).upper()}",
        "",
        "SIZE  METHOD RESULT SCORE   SENT   READ EARLY CLEAR  MISS EXTRA  TIME",
        border_line(REPORT_WIDTH),
    ]
    for result in results:
        lines.append(
            f"{result.size_label:<5} "
            f"{result.method[:6].upper():<6} "
            f"{result.indicator:<6} "
            f"{result.score:>5.1f} "
            f"{result.bytes_sent:>6} "
            f"{result.bytes_received:>6} "
            f"{result.bytes_seen_before_release:>5} "
            f"{result.bytes_cleared_before:>5} "
            f"{result.metrics.missing_bytes:>5} "
            f"{result.metrics.extra_bytes:>5} "
            f"{format_duration(result.elapsed_sec):>6}"
        )
    lines.append(border_line(REPORT_WIDTH))
    with text_report.open("a", encoding="utf-8") as report_file:
        report_file.write("\n".join(lines) + "\n")


def flow_control_hold_byte_limit(payload_bytes: int) -> int:
    """Return tolerated in-flight bytes while a flow-control hold is active."""
    return max(16, min(256, payload_bytes // 100))


def flow_validation_indicator(status: str, score: float, error: str | None) -> str:
    """Return a compact indicator for a flow-control validation status."""
    if error or status == "error":
        return "ERROR"
    if status == "validated":
        return "PASS"
    if status in {"transfer-good", "paused-partial-transfer"}:
        return "GOOD"
    if status == "transfer-partial":
        return "PARTIAL"
    if status == "no-pause":
        return "FAIL"
    if score >= 90.0:
        return "GOOD"
    if score >= 50.0:
        return "PARTIAL"
    return "FAIL"


def flow_validation_result_from_candidate(
    flow_control: str,
    method: str,
    result: CandidateResult,
    payload: ProbePayload,
) -> FlowControlValidationResult:
    """Convert a normal transfer candidate result into a flow validation row."""
    clean = (
        result.score >= 99.0
        and result.bytes_sent == payload.byte_count
        and result.bytes_received == payload.byte_count
        and result.metrics.missing_bytes == 0
        and result.metrics.extra_bytes == 0
    )
    if result.error:
        status = "error"
        reason = result.error
    elif result.bytes_sent < payload.byte_count:
        status = "partial-write"
        reason = "PAYLOAD WAS NOT FULLY SENT."
    elif clean:
        status = "validated"
        reason = "CLEAN TRANSFER WITH THIS FLOW SETTING."
    elif result.score >= 90.0:
        status = "transfer-good"
        reason = "TRANSFER WAS GOOD BUT NOT BYTE-PERFECT."
    elif result.bytes_received > 0:
        status = "transfer-partial"
        reason = "TRANSFER PRODUCED ONLY A PARTIAL MATCH."
    else:
        status = "no-data"
        reason = "NO OUTPUT WAS RECEIVED."
    return FlowControlValidationResult(
        flow_control=flow_control,
        method=method,
        settings=result.settings,
        bytes_sent=result.bytes_sent,
        bytes_received=result.bytes_received,
        bytes_seen_while_held=0,
        score=result.score,
        indicator=flow_validation_indicator(status, result.score, result.error),
        status=status,
        reason=reason,
        error=result.error,
        elapsed_sec=result.elapsed_sec,
        metrics=result.metrics,
    )


def apply_flow_control_hold(control_serial: Any, flow_control: str) -> None:
    """Assert a receive-side hold for one flow-control mode."""
    if flow_control == "xon/xoff":
        control_serial.write(b"\x13")
        control_serial.flush()
        return
    if flow_control == "rts/cts":
        control_serial.rts = False
        return
    if flow_control == "dsr/dtr":
        control_serial.dtr = False
        return
    raise ValueError(f"cannot hold flow control mode {flow_control}")


def release_flow_control_hold(control_serial: Any, flow_control: str) -> None:
    """Release a receive-side hold for one flow-control mode."""
    if flow_control == "xon/xoff":
        control_serial.write(b"\x11")
        control_serial.flush()
        return
    if flow_control == "rts/cts":
        control_serial.rts = True
        return
    if flow_control == "dsr/dtr":
        control_serial.dtr = True
        return
    raise ValueError(f"cannot release flow control mode {flow_control}")


def flow_validation_error_result(
    flow_control: str,
    method: str,
    settings: SerialSettings,
    payload: ProbePayload,
    error: str,
    elapsed_sec: float,
) -> FlowControlValidationResult:
    """Return an error row for flow-control validation."""
    empty_score = score_received(payload.data, b"")
    return FlowControlValidationResult(
        flow_control=flow_control,
        method=method,
        settings=settings,
        bytes_sent=0,
        bytes_received=0,
        bytes_seen_while_held=0,
        score=0.0,
        indicator="ERROR",
        status="error",
        reason=error,
        error=error,
        elapsed_sec=elapsed_sec,
        metrics=empty_score.metrics,
    )


def run_flow_control_transfer_validation(
    serial_module: Any,
    index: int,
    total: int,
    settings: SerialSettings,
    options: ScanOptions,
    payload: ProbePayload,
    logger: logging.Logger,
) -> FlowControlValidationResult:
    """Validate a flow mode with a clean large transfer."""
    validation_options = dataclasses.replace(
        options,
        payload_bytes=payload.byte_count,
        read_timeout=max(options.read_timeout, FLOW_VALIDATE_READ_TIMEOUT),
        bursts=1,
        turbo_discovery_enabled=False,
        ask_on_top_match=False,
        no_pre_drain=False,
        pre_drain_timeout=max(options.pre_drain_timeout, 2.0),
    )
    result = run_candidate(
        serial_module=serial_module,
        index=index,
        total=total,
        settings=settings,
        options=validation_options,
        payload=payload,
        logger=logger,
        progress=console_progress,
    )
    return flow_validation_result_from_candidate(
        settings.flow_control,
        "large-transfer",
        result,
        payload,
    )


def run_flow_control_pause_validation(
    serial_module: Any,
    index: int,
    total: int,
    settings: SerialSettings,
    options: ScanOptions,
    payload: ProbePayload,
    logger: logging.Logger,
) -> FlowControlValidationResult:
    """Validate an output-side pause/release handshake behavior."""
    started = time.monotonic()
    flow_control = settings.flow_control
    method = {
        "xon/xoff": "xoff-xon-pause",
        "rts/cts": "rts-hold-release",
        "dsr/dtr": "dtr-hold-release",
    }[flow_control]
    prefix = (
        f"[FLOW {index:02d}/{total:02d} {settings.baud} "
        f"{settings.data_bits}{settings.parity_code()}{settings.stop_bits} "
        f"{flow_control.upper()}]"
    )
    control_settings = dataclasses.replace(settings, flow_control="none")
    read_timeout = max(options.read_timeout, FLOW_VALIDATE_READ_TIMEOUT)
    hold_applied = False
    payload_bytes = payload.byte_count
    empty_score = score_received(payload.data, b"")
    try:
        console_progress(border_line(PROGRESS_WIDTH))
        console_progress(
            bordered_text(
                f"FLOW VALIDATION {index}/{total} {flow_control.upper()}",
                PROGRESS_WIDTH,
            )
        )
        console_progress(border_line(PROGRESS_WIDTH))
        with open_serial_port(
            serial_module,
            options.out_port,
            control_settings,
            read_timeout,
        ) as out_serial:
            with open_serial_port(
                serial_module,
                options.in_port,
                settings,
                read_timeout,
            ) as in_serial:
                reset_serial_buffers(out_serial)
                reset_serial_buffers(in_serial)
                time.sleep(options.settle_ms / 1000.0)

                drain = DrainResult(0, 0.0, True, "disabled", None)
                if not options.no_pre_drain:
                    drain = drain_output_until_quiet(
                        out_serial=out_serial,
                        quiet_seconds=options.pre_drain_quiet,
                        max_seconds=max(options.pre_drain_timeout, 2.0),
                        max_bytes=options.max_drain_bytes,
                        progress_interval=options.progress_interval,
                        progress=console_progress,
                        prefix=prefix,
                        logger=logger,
                    )
                    if not drain.quiet:
                        error = (
                            "OUTPUT DID NOT GO QUIET BEFORE FLOW VALIDATION "
                            f"(REASON={drain.reason.upper()}, "
                            f"CLEARED={drain.bytes_drained})"
                        )
                        if drain.error:
                            error = f"{error}: {drain.error}"
                        return FlowControlValidationResult(
                            flow_control=flow_control,
                            method=method,
                            settings=settings,
                            bytes_sent=0,
                            bytes_received=0,
                            bytes_seen_while_held=0,
                            score=0.0,
                            indicator="FAIL",
                            status="stale-output",
                            reason=error,
                            error=error,
                            elapsed_sec=time.monotonic() - started,
                            metrics=empty_score.metrics,
                        )

                console_progress(f"{prefix}: ASSERT {flow_control.upper()} HOLD")
                apply_flow_control_hold(out_serial, flow_control)
                hold_applied = True
                time.sleep(FLOW_VALIDATE_RELEASE_SETTLE_SECONDS)

                bytes_sent, write_error, _write_elapsed = write_payload_only(
                    in_serial=in_serial,
                    settings=settings,
                    payload=payload,
                    progress_interval=options.progress_interval,
                    prefix=prefix,
                    logger=logger,
                )
                held_bytes, hold_error, _hold_elapsed = read_for_fixed_window(
                    out_serial=out_serial,
                    seconds=FLOW_VALIDATE_HOLD_SECONDS,
                    progress_interval=options.progress_interval,
                    prefix=prefix,
                    logger=logger,
                )
                console_progress(f"{prefix}: RELEASE {flow_control.upper()} HOLD")
                release_flow_control_hold(out_serial, flow_control)
                hold_applied = False
                time.sleep(FLOW_VALIDATE_RELEASE_SETTLE_SECONDS)

                release_bytes, read_error, _read_elapsed = read_until_quiet(
                    out_serial=out_serial,
                    settings=settings,
                    expected_bytes=payload_bytes,
                    read_timeout=read_timeout,
                    progress_interval=options.progress_interval,
                    prefix=prefix,
                    logger=logger,
                )
    except Exception as exc:
        logger.exception("flow validation failed for %s", settings.label())
        return flow_validation_error_result(
            flow_control=flow_control,
            method=method,
            settings=settings,
            payload=payload,
            error=str(exc),
            elapsed_sec=time.monotonic() - started,
        )
    finally:
        if hold_applied:
            try:
                release_flow_control_hold(out_serial, flow_control)  # type: ignore[name-defined]
            except Exception:
                logger.debug("failed to release %s hold in cleanup", flow_control)

    received_bytes = held_bytes + release_bytes
    score = score_received(payload.data, received_bytes)
    error = write_error or hold_error or read_error
    hold_limit = flow_control_hold_byte_limit(payload_bytes)
    pause_ok = len(held_bytes) <= hold_limit
    clean = (
        score.score >= 99.0
        and bytes_sent == payload_bytes
        and len(received_bytes) == payload_bytes
        and score.metrics.missing_bytes == 0
        and score.metrics.extra_bytes == 0
    )
    if error:
        status = "error"
        reason = error
    elif bytes_sent < payload_bytes:
        status = "partial-write"
        reason = "PAYLOAD WAS NOT FULLY SENT."
    elif not pause_ok:
        status = "no-pause"
        reason = (
            f"OUTPUT DID NOT PAUSE; SAW {len(held_bytes)} BYTES "
            f"WHILE HELD (LIMIT {hold_limit})."
        )
    elif clean:
        status = "validated"
        reason = "OUTPUT PAUSED DURING HOLD AND CLEANLY RESUMED."
    elif not received_bytes:
        status = "no-data"
        reason = "NO OUTPUT WAS RECEIVED AFTER RELEASE."
    elif score.score >= 90.0:
        status = "paused-partial-transfer"
        reason = "OUTPUT PAUSED, BUT RESUMED TRANSFER WAS NOT BYTE-PERFECT."
    else:
        status = "transfer-partial"
        reason = "OUTPUT PAUSED, BUT RESUMED TRANSFER WAS A POOR MATCH."

    return FlowControlValidationResult(
        flow_control=flow_control,
        method=method,
        settings=settings,
        bytes_sent=bytes_sent,
        bytes_received=len(received_bytes),
        bytes_seen_while_held=len(held_bytes),
        score=score.score,
        indicator=flow_validation_indicator(status, score.score, error),
        status=status,
        reason=reason,
        error=error,
        elapsed_sec=time.monotonic() - started,
        metrics=score.metrics,
    )


def select_flow_validation_frame(
    results: Sequence[CandidateResult],
    validation_results: Sequence[CandidateResult],
) -> SerialSettings | None:
    """Return the best baud/data/parity/stop frame for flow validation."""
    sources = [list(validation_results), list(results)]
    for source in sources:
        for result in sorted(source, key=result_sort_key, reverse=True):
            if is_recommendable_result(result):
                return SerialSettings(
                    baud=result.settings.baud,
                    data_bits=result.settings.data_bits,
                    parity=result.settings.parity,
                    stop_bits=result.settings.stop_bits,
                    flow_control="none",
                )
    return None


def flow_validation_settings_for_frame(frame: SerialSettings) -> list[SerialSettings]:
    """Return all flow-control settings for one proven serial frame."""
    return [
        dataclasses.replace(frame, flow_control=flow_control)
        for flow_control in FLOW_CONTROLS
    ]


def format_flow_validation_progress(
    result: FlowControlValidationResult,
    index: int,
    total: int,
) -> str:
    """Return one console line for a flow-control validation result."""
    return (
        f"FLOW [{index:02d}/{total:02d}] {result.indicator:<7} "
        f"{result.settings.label():32s} "
        f"SENT={result.bytes_sent:6d} READ={result.bytes_received:6d} "
        f"HELD={result.bytes_seen_while_held:5d} "
        f"SCORE={result.score:6.2f} {result.reason}"
    )


def flow_control_validation_recommendation(
    results: Sequence[FlowControlValidationResult],
) -> str:
    """Return a concise flow-control validation conclusion."""
    proven = [result for result in results if result.status == "validated"]
    proven_handshake = [
        result for result in proven if result.flow_control != "none"
    ]
    if len(proven_handshake) == 1:
        return f"PROVEN HANDSHAKE: {flow_control_name(proven_handshake[0].flow_control)}."
    if len(proven_handshake) > 1:
        flows = ", ".join(flow_control_name(result.flow_control) for result in proven_handshake)
        return f"MULTIPLE HANDSHAKES VALIDATED: {flows}."
    if any(result.flow_control == "none" for result in proven):
        return "CLEAN TRANSFER WITHOUT HANDSHAKE; NO HANDSHAKE MODE PROVEN."
    if results:
        return "NO FLOW CONTROL MODE VALIDATED."
    return "FLOW CONTROL VALIDATION WAS NOT RUN."


def flow_validation_report_lines(
    results: Sequence[FlowControlValidationResult],
) -> list[str]:
    """Return compact flow-control validation lines for the text report."""
    lines = [
        "FLOW CONTROL VALIDATION:",
        f"FINDING:         {flow_control_validation_recommendation(results)}",
        "",
        "FLOW     RESULT  SCORE   SENT   READ  HELD METHOD             REASON",
        border_line(REPORT_WIDTH),
    ]
    for result in results:
        lines.append(
            f"{result.flow_control.upper():<8} "
            f"{result.indicator:<7} "
            f"{result.score:>5.1f} "
            f"{result.bytes_sent:>6} "
            f"{result.bytes_received:>6} "
            f"{result.bytes_seen_while_held:>5} "
            f"{result.method:<18} "
            f"{result.reason[:28]}"
        )
    lines.append(border_line(REPORT_WIDTH))
    return lines


def print_flow_control_validation_report(
    frame: SerialSettings,
    results: Sequence[FlowControlValidationResult],
) -> None:
    """Print the final flow-control validation report."""
    print()
    print_report_title("FLOW CONTROL VALIDATION RESULTS")
    print(f"  FRAME SETTING:        {frame.baud} {frame.data_bits}{frame.parity_code()}{frame.stop_bits}")
    print(f"  TEST BYTES:           {max((result.bytes_sent for result in results), default=0)}")
    print_wrapped_value(
        "  FINDING:              ",
        flow_control_validation_recommendation(results),
    )
    print(border_line(REPORT_WIDTH))
    print("FLOW     RESULT  SCORE   SENT   READ  HELD METHOD             REASON")
    print(border_line(REPORT_WIDTH))
    for result in results:
        print(
            f"{result.flow_control.upper():<8} "
            f"{result.indicator:<7} "
            f"{result.score:>5.1f} "
            f"{result.bytes_sent:>6} "
            f"{result.bytes_received:>6} "
            f"{result.bytes_seen_while_held:>5} "
            f"{result.method:<18} "
            f"{result.reason[:28]}"
        )
    print(border_line(REPORT_WIDTH))


def run_flow_control_validation(
    serial_module: Any,
    options: ScanOptions,
    results: Sequence[CandidateResult],
    validation_results: Sequence[CandidateResult],
    logger: logging.Logger,
) -> tuple[list[FlowControlValidationResult], str | None]:
    """Run post-scan validation for all flow-control modes on the best frame."""
    frame = select_flow_validation_frame(results, validation_results)
    if frame is None:
        print()
        print_report_title("FLOW CONTROL VALIDATION")
        print("SKIPPED: NO RECOMMENDABLE FRAME SETTING WAS FOUND.")
        print(border_line(REPORT_WIDTH))
        logger.info("flow control validation skipped: no recommendable frame")
        return [], None

    purge_buffer_output(
        serial_module=serial_module,
        options=options,
        logger=logger,
        reason="CLEAR VALIDATION DATA BEFORE FLOW-CONTROL TESTS.",
    )
    payload = generate_payload(options.flow_validate_size_bytes)
    settings_list = flow_validation_settings_for_frame(frame)
    print()
    print_report_title("FLOW CONTROL VALIDATION")
    print(f"FRAME: {frame.baud} {frame.data_bits}{frame.parity_code()}{frame.stop_bits}.")
    print(f"TEST: {payload.byte_count} BYTES; HANDSHAKE MODES USE HOLD/RELEASE.")
    print("DISCOVERY PAYLOAD STAYS NEUTRAL; THIS PHASE TESTS HANDSHAKE BEHAVIOR.")
    print(border_line(REPORT_WIDTH))
    logger.info(
        "flow control validation started frame=%s payload=%s",
        frame.label(),
        payload.byte_count,
    )

    flow_results: list[FlowControlValidationResult] = []
    for index, settings in enumerate(settings_list, start=1):
        while True:
            try:
                if settings.flow_control == "none":
                    result = run_flow_control_transfer_validation(
                        serial_module=serial_module,
                        index=index,
                        total=len(settings_list),
                        settings=settings,
                        options=options,
                        payload=payload,
                        logger=logger,
                    )
                else:
                    result = run_flow_control_pause_validation(
                        serial_module=serial_module,
                        index=index,
                        total=len(settings_list),
                        settings=settings,
                        options=options,
                        payload=payload,
                        logger=logger,
                    )
            except KeyboardInterrupt:
                action = prompt_operator_break_action("FLOW VALIDATION")
                if action == "resume":
                    logger.info(
                        "operator resumed flow validation at %s/%s %s",
                        index,
                        len(settings_list),
                        settings.label(),
                    )
                    continue
                logger.info(
                    "operator break during flow validation; action=%s completed=%s/%s",
                    action,
                    len(flow_results),
                    len(settings_list),
                )
                print_flow_control_validation_report(frame, flow_results)
                return flow_results, action
            flow_results.append(result)
            print(format_flow_validation_progress(result, index, len(settings_list)))
            logger.info(
                "flow validation %s: indicator=%s status=%s score=%.2f sent=%s read=%s held=%s reason=%s error=%s",
                settings.flow_control,
                result.indicator,
                result.status,
                result.score,
                result.bytes_sent,
                result.bytes_received,
                result.bytes_seen_while_held,
                result.reason,
                result.error,
            )
            break

    print_flow_control_validation_report(frame, flow_results)
    logger.info(
        "flow control validation completed: %s",
        flow_control_validation_recommendation(flow_results),
    )
    return flow_results, None


def run_memory_test(options: ScanOptions) -> None:
    """Prompt for and run a memory transfer test."""
    try:
        ensure_distinct_ports(options.in_port, options.out_port)
    except ValueError as exc:
        print(f"SETTINGS ERROR: {exc}")
        return
    settings = prompt_serial_setting(SerialSettings(9600, 8, "none", 1, "none"))
    sizes = prompt_memory_sizes()
    method = prompt_memory_method()
    text_report = options.text_report
    log_file = options.log_file
    logger = setup_logging(log_file)
    serial_module = import_or_install_pyserial()
    purge_buffer_output(
        serial_module=serial_module,
        options=options,
        logger=logger,
        reason="CLEAR BUFFER BEFORE MEMORY TEST.",
    )
    print()
    print_report_title("MEMORY TEST START")
    print("  USES SERIAL SETTING ENTERED FROM SCAN REPORT.")
    if method == "hold-release":
        print("  HOLD BUFFER OUTPUT WHILE DATA IS SENT; RELEASE WHEN ASKED.")
    else:
        print("  READS WHILE SENDING. CHECKS CLEAN TRANSFER SIZE.")
    print(f"  SETTING:    {settings.label()}")
    print(f"  SIZES:      {', '.join(byte_size_label(size) for size in sizes)}")
    print(f"  METHOD:     {method.upper()}")
    print(border_line(REPORT_WIDTH))
    results: list[MemoryTestResult] = []
    operator_break_action: str | None = None
    size_index = 0
    while size_index < len(sizes):
        size = sizes[size_index]
        display_index = size_index + 1
        try:
            result = run_memory_size_test(
                serial_module=serial_module,
                index=display_index,
                total=len(sizes),
                size_bytes=size,
                settings=settings,
                options=options,
                logger=logger,
                method=method,
            )
        except KeyboardInterrupt:
            action = prompt_operator_break_action("MEMORY TEST")
            if action == "resume":
                logger.info(
                    "operator resumed memory test at %s/%s %s",
                    display_index,
                    len(sizes),
                    byte_size_label(size),
                )
                continue
            operator_break_action = action
            logger.info(
                "operator break during memory test; action=%s completed=%s/%s",
                action,
                len(results),
                len(sizes),
            )
            break
        results.append(result)
        size_index += 1
    write_memory_text_report(text_report, settings, results)
    print_memory_report(settings, results, text_report, log_file)
    if operator_break_action == "quit":
        raise QuitProgramAfterReport()


def print_commands() -> None:
    """Print the main command menu."""
    print()
    print_banner()
    print("MAIN MENU")
    print("  1 START SCAN                 7 SET REPORT FILES")
    print("  2 SET COM PORTS              8 RESET REPORT FILES")
    print("  3 SET BAUD RANGE             9 CURRENT SETTINGS")
    print("  4 SET TEST SIZE/COUNT       10 QUICK BAUD FOCUS")
    print("  5 SET TIMING/TURBO          11 MEMORY TEST")
    print("  6 CLEAR OLD OUTPUT          12 HELP")
    print("  0 QUIT")


def interactive_menu(options: ScanOptions | None = None) -> ScanOptions | None:
    """Show the command-line style configuration menu."""
    if options is None:
        options = default_scan_options()
    while True:
        print_commands()
        try:
            choice = read_operator_input("ENTER SELECTION: ").lstrip("\ufeff").strip().lower()
        except EOFError:
            return None

        if choice == "1":
            try:
                validate_options(options)
            except ValueError as exc:
                print(f"SETTINGS ERROR: {exc}")
                continue
            return options
        if choice == "2":
            options = dataclasses.replace(
                options,
                in_port=prompt_text("Input/transmit port", options.in_port),
                out_port=prompt_text("Output/read port", options.out_port),
            )
        elif choice == "3":
            options = configure_baud_range(options)
        elif choice == "4":
            options = configure_payload(options)
        elif choice == "5":
            options = configure_timing(options)
        elif choice == "6":
            options = configure_pre_drain(options)
        elif choice == "7":
            options = configure_reports(options)
        elif choice == "8":
            default_text_report, default_log = default_report_paths()
            options = dataclasses.replace(
                options,
                text_report=default_text_report,
                log_file=default_log,
            )
        elif choice == "9":
            print_configuration(options)
        elif choice == "10":
            options = configure_baud_focus(options)
        elif choice == "11":
            run_memory_test(options)
        elif choice == "12":
            print_menu_help()
        elif choice in {"0", "q", "quit", "exit"}:
            return None
        else:
            print("ENTER A NUMBER FROM 0 TO 12.")


def prompt_after_scan_action(title: str = "RUN COMPLETE") -> str:
    """Ask what to do after a scan or sweep finishes or stops."""
    print()
    print(border_line(REPORT_WIDTH))
    print(bordered_text(title, REPORT_WIDTH))
    print(border_line(REPORT_WIDTH))
    print("  1 START SCAN AGAIN")
    print("  2 RETURN TO MAIN MENU")
    print("  0 QUIT")
    print(border_line(REPORT_WIDTH))
    while True:
        try:
            choice = read_operator_input("ENTER SELECTION [2]: ").lstrip("\ufeff").strip().lower()
        except EOFError:
            return "menu"
        if choice == "":
            return "menu"
        if choice in {"1", "r", "rerun", "run"}:
            return "rerun"
        if choice in {"2", "m", "menu", "main"}:
            return "menu"
        if choice in {"0", "q", "quit", "exit"}:
            return "quit"
        print("ENTER 1, 2, OR 0.")


def run_dual_bank_validation(
    serial_module: Any,
    options: ScanOptions,
    shortlist: Sequence[CandidateResult],
    logger: logging.Logger,
) -> tuple[list[CandidateResult], str | None]:
    """Run a larger validation payload for top dual-bank candidates."""
    if not shortlist:
        return [], None
    purge_buffer_output(
        serial_module=serial_module,
        options=options,
        logger=logger,
        reason="CLEAR SCAN DATA BEFORE DUAL VALIDATION.",
    )
    print()
    print_report_title("DUAL BANK VALIDATION")
    print(
        f"SHORTLIST: {len(shortlist)} TOP-SCORE PAIR(S) "
        f"AT {options.validate_size_1_bytes} BYTES."
    )
    print(border_line(REPORT_WIDTH))
    validation_options = dataclasses.replace(
        options,
        turbo_discovery_enabled=False,
        payload_bytes=options.validate_size_1_bytes,
        bursts=1,
        ask_on_top_match=False,
    )
    validation_payload = generate_payload(options.validate_size_1_bytes)
    validation_results: list[CandidateResult] = []
    index = 0
    while index < len(shortlist):
        candidate = shortlist[index]
        settings = dual_result_settings(candidate)
        display_index = index + 1
        print(
            f"DUAL VALIDATION: {validation_payload.byte_count} BYTES "
            f"{display_index}/{len(shortlist)} {settings.label()}"
        )
        try:
            result = run_dual_candidate(
                serial_module=serial_module,
                index=display_index,
                total=len(shortlist),
                settings=settings,
                options=validation_options,
                payload=validation_payload,
                logger=logger,
                progress=console_progress,
            )
        except KeyboardInterrupt:
            action = prompt_operator_break_action("DUAL VALIDATION")
            if action == "resume":
                logger.info(
                    "operator resumed dual validation at %s/%s %s",
                    display_index,
                    len(shortlist),
                    settings.label(),
                )
                continue
            logger.info(
                "operator break during dual validation; action=%s completed=%s/%s",
                action,
                len(validation_results),
                len(shortlist),
            )
            return validation_results, action
        validation_results.append(result)
        print(format_dual_progress(result), flush=True)
        index += 1
    print_dual_ranked_table(
        validation_results,
        min(options.top, len(validation_results)),
        report_title="DUAL BANK VALIDATION REPORT",
    )
    return validation_results, None


def run_dual_bank_scan(
    serial_module: Any,
    options: ScanOptions,
    logger: logging.Logger,
    phase0_only: bool = False,
) -> int:
    """Run a dual-bank scan where input and output settings may differ."""
    if not phase0_only:
        turbo_enabled = prompt_yes_no_question(
            "Turbo discovery mode?",
            options.turbo_discovery_enabled,
        )
        switch_note = prompt_text(
            "Switch/jumper note for report",
            options.switch_note,
        )
        options = dataclasses.replace(
            options,
            turbo_discovery_enabled=turbo_enabled,
            switch_note=switch_note,
        )
    logger.info("dual-bank scan started phase0_only=%s", phase0_only)
    logger.info("options: %s", options)

    phase0_report = run_dual_phase0_baud_matrix(
        serial_module=serial_module,
        options=options,
        logger=logger,
    )
    if phase0_only:
        print()
        print_report_title("DUAL PHASE 0 COMPLETE")
        print("FULL DUAL-BANK SCAN WAS NOT RUN.")
        print_wrapped_value("  DEBUG LOG:   ", options.log_file)
        print(border_line(REPORT_WIDTH))
        return 0

    selected_pairs = phase0_report.selected_pairs
    if not selected_pairs:
        print()
        print_report_title("DUAL BANK SCAN SKIPPED")
        print("NO BAUD PAIRS WERE GOOD ENOUGH TO EXPAND INTO FRAME TESTING.")
        print(border_line(REPORT_WIDTH))
        return 1

    dual_flow_discovery_enabled = options.auto_validate_flow_control
    full_candidates: list[DualSerialSettings] = []
    staged_preview_candidates: list[DualSerialSettings] = []
    staged_flow_candidate_count = 0
    for input_baud, output_baud in selected_pairs:
        full_candidates.extend(dual_frame_candidates_for_pair(input_baud, output_baud))
        seed = dual_phase0_settings(input_baud, output_baud)
        staged_preview_candidates.extend(dual_output_frame_sweep_for_pair(input_baud, output_baud))
        staged_preview_candidates.extend(
            dual_input_frame_sweep_for_pair(input_baud, seed.output_settings)
        )
        if dual_flow_discovery_enabled:
            staged_flow_candidate_count += max(
                0,
                len(dual_flow_candidates_for_frame(seed)) - 1,
            )
    full_candidates = unique_dual_candidates(full_candidates)
    staged_preview_candidates = unique_dual_candidates(staged_preview_candidates)
    candidate_total = len(full_candidates) + staged_flow_candidate_count
    staged_total = len(staged_preview_candidates) + staged_flow_candidate_count
    payload = generate_payload(options.payload_bytes)
    purge_buffer_output(
        serial_module=serial_module,
        options=options,
        logger=logger,
        reason="CLEAR PHASE 0 DATA BEFORE DUAL FRAME SCAN.",
    )
    scan_started = time.monotonic()
    started_at = dt.datetime.now(dt.timezone.utc).isoformat()

    print()
    print_report_title("DUAL BANK SCAN START")
    print("MODEL: SW1/SW2 MAY CONTROL DIFFERENT BUFFER PORTS.")
    print("INPUT SIDE:  PC TRANSMIT PORT INTO BUFFER.")
    print("OUTPUT SIDE: PC RECEIVE PORT FROM BUFFER.")
    print_wrapped_value(
        "BAUD PAIRS: ",
        ", ".join(
            f"IN {input_baud}/OUT {output_baud}"
            for input_baud, output_baud in selected_pairs
        ),
    )
    print(
        f"STAGED SETTINGS: {staged_total} "
        f"BEFORE FULL MATRIX FALLBACK."
    )
    print(f"FULL MATRIX: {len(full_candidates)} INPUT/OUTPUT FRAME PAIRS IF NEEDED.")
    print(
        f"PORTS: {options.in_port} -> BUFFER -> {options.out_port}; "
        f"TEST={payload.byte_count} BYTES X {options.bursts}"
    )
    print("DISCOVERY ORDER: PHASE 0 FRAME, OUTPUT SWEEP, INPUT SWEEP.")
    if dual_flow_discovery_enabled:
        print("FLOW DISCOVERY: INPUT/OUTPUT FLOW COMBINATIONS AFTER BEST FRAME.")
    else:
        print("FLOW DISCOVERY: OFF.")
    print("FULL MATRIX RUNS ONLY IF STAGED DISCOVERY DOES NOT FIND A GOOD PAIR.")
    print("FRAME SWEEPS USE FLOW=NONE; FLOW SWEEP TESTS HANDSHAKE SETTINGS.")
    if options.auto_validate_top_matches:
        print(f"VALIDATION: ON; SIZE1={options.validate_size_1_bytes} BYTES.")
    else:
        print("VALIDATION: OFF.")
    if options.auto_validate_flow_control:
        print("FLOW VALIDATION: DEFERRED; DUAL MODE FIRST FINDS THE FRAME PAIR.")
    print_progress_legend()
    print(f"REPORT: {options.text_report} (APPEND)")
    print(f"DEBUG LOG: {options.log_file}")
    staged_rough_total = 0.0
    staged_estimate_candidates = list(staged_preview_candidates)
    if dual_flow_discovery_enabled:
        for input_baud, output_baud in selected_pairs:
            staged_estimate_candidates.extend(
                dual_flow_candidates_for_frame(dual_phase0_settings(input_baud, output_baud))
            )
    staged_estimate_candidates = unique_dual_candidates(staged_estimate_candidates)
    for candidate in staged_estimate_candidates:
        timing = effective_discovery_timing(
            options,
            candidate.input_settings,
            options.payload_bytes,
        )
        staged_rough_total += estimated_transmit_seconds(
            candidate.input_settings,
            options.payload_bytes,
        ) * options.bursts
        per_burst_wait = timing.read_timeout + (timing.settle_ms / 1000.0)
        if not options.no_pre_drain:
            per_burst_wait += timing.pre_drain_quiet
        staged_rough_total += per_burst_wait * options.bursts
    full_rough_total = 0.0
    for candidate in full_candidates:
        timing = effective_discovery_timing(
            options,
            candidate.input_settings,
            options.payload_bytes,
        )
        full_rough_total += estimated_transmit_seconds(
            candidate.input_settings,
            options.payload_bytes,
        ) * options.bursts
        per_burst_wait = timing.read_timeout + (timing.settle_ms / 1000.0)
        if not options.no_pre_drain:
            per_burst_wait += timing.pre_drain_quiet
        full_rough_total += per_burst_wait * options.bursts
    print(
        f"STAGE EST.: {format_duration(staged_rough_total)}; "
        f"FINISH ABOUT {format_finish_clock(staged_rough_total)}"
    )
    print(
        f"FULL EST. IF NEEDED: {format_duration(full_rough_total)}; "
        f"FINISH ABOUT {format_finish_clock(full_rough_total)}"
    )

    results: list[CandidateResult] = []
    tested_settings: set[DualSerialSettings] = set()
    early_stopped = False
    operator_break_action: str | None = None
    operator_break_stage: str | None = None
    full_matrix_ran = False
    full_matrix_skipped = False

    def run_dual_sequence(
        stage_title: str,
        stage_name: str,
        sequence: Sequence[DualSerialSettings],
    ) -> list[CandidateResult]:
        nonlocal early_stopped, operator_break_action, operator_break_stage
        unique_sequence = [
            settings for settings in unique_dual_candidates(sequence)
            if settings not in tested_settings
        ]
        if (
            not unique_sequence
            or early_stopped
            or operator_break_action is not None
        ):
            return []

        print()
        print_report_title(stage_title)
        print(f"SETTINGS: {len(unique_sequence)}")
        print(border_line(REPORT_WIDTH))
        stage_results: list[CandidateResult] = []
        index = 0
        while index < len(unique_sequence):
            settings = unique_sequence[index]
            display_index = len(results) + 1
            try:
                result = run_dual_candidate(
                    serial_module=serial_module,
                    index=display_index,
                    total=candidate_total,
                    settings=settings,
                    options=options,
                    payload=payload,
                    logger=logger,
                    progress=console_progress,
                )
            except KeyboardInterrupt:
                action = prompt_operator_break_action(stage_name)
                if action == "resume":
                    logger.info(
                        "operator resumed %s at %s/%s %s",
                        stage_name.lower(),
                        display_index,
                        candidate_total,
                        settings.label(),
                    )
                    continue
                operator_break_action = action
                operator_break_stage = stage_name
                early_stopped = True
                logger.info(
                    "operator break during %s; action=%s completed=%s/%s",
                    stage_name.lower(),
                    action,
                    len(results),
                    candidate_total,
                )
                break
            results.append(result)
            stage_results.append(result)
            tested_settings.add(settings)
            print(format_dual_progress(result), flush=True)
            print(
                format_scan_eta(len(results), candidate_total, scan_started),
                flush=True,
            )
            if options.ask_on_top_match and is_top_match_result(result):
                print()
                print_report_title("DUAL TOP MATCH FOUND")
                print_dual_result_details(result)
                print("    CONTINUE TO LOOK FOR POSSIBLE TIES.")
                print("    ENTER N TO END NOW AND WRITE REPORT.")
                print(border_line(REPORT_WIDTH))
                if not prompt_yes_no("CONTINUE DUAL SCAN", True):
                    early_stopped = True
                    operator_break_stage = stage_name
                    logger.info(
                        "operator ended %s after top match: %s",
                        stage_name.lower(),
                        settings.label(),
                    )
                    break
            index += 1
        return stage_results

    for input_baud, output_baud in selected_pairs:
        if early_stopped or operator_break_action is not None:
            break
        seed = dual_phase0_settings(input_baud, output_baud)
        seed_results = run_dual_sequence(
            "DUAL SEED CONFIRM",
            "DUAL SEED CONFIRM",
            [seed],
        )
        output_results = run_dual_sequence(
            "DUAL OUTPUT FRAME SWEEP",
            "DUAL OUTPUT FRAME SWEEP",
            dual_output_frame_sweep_for_pair(input_baud, output_baud),
        )
        best_output = best_dual_output_settings(
            [*seed_results, *output_results],
            seed,
        )
        input_results = run_dual_sequence(
            "DUAL INPUT FRAME SWEEP",
            "DUAL INPUT FRAME SWEEP",
            dual_input_frame_sweep_for_pair(input_baud, best_output),
        )
        if dual_flow_discovery_enabled:
            best_frame = best_dual_settings(
                [*seed_results, *output_results, *input_results],
                seed,
            )
            run_dual_sequence(
                "DUAL FLOW DISCOVERY",
                "DUAL FLOW DISCOVERY",
                dual_flow_candidates_for_frame(best_frame),
            )

    if not early_stopped and operator_break_action is None:
        if has_recommendable_dual_result(results):
            full_matrix_skipped = True
            print()
            print_report_title("DUAL FULL MATRIX SKIPPED")
            print("STAGED DISCOVERY FOUND A GOOD INPUT/OUTPUT FRAME PAIR.")
            print(f"FULL {len(full_candidates)}-PAIR MATRIX WAS NOT NEEDED.")
            print(border_line(REPORT_WIDTH))
        else:
            fallback_candidates = [
                candidate
                for candidate in full_candidates
                if candidate not in tested_settings
            ]
            if fallback_candidates:
                full_matrix_ran = True
                fallback_results = run_dual_sequence(
                    "DUAL FULL MATRIX FALLBACK",
                    "DUAL FULL MATRIX FALLBACK",
                    fallback_candidates,
                )
                if (
                    dual_flow_discovery_enabled
                    and fallback_results
                    and has_recommendable_dual_result(results)
                    and not early_stopped
                    and operator_break_action is None
                ):
                    best_frame = best_dual_settings(results, fallback_candidates[0])
                    run_dual_sequence(
                        "DUAL FLOW DISCOVERY",
                        "DUAL FLOW DISCOVERY",
                        dual_flow_candidates_for_frame(best_frame),
                    )

    elapsed_sec = time.monotonic() - scan_started
    completed_at = dt.datetime.now(dt.timezone.utc).isoformat()
    metadata: dict[str, Any] = {
        "tool": "serial_probe",
        "mode": "dual-bank-scan",
        "started_at": started_at,
        "completed_at": completed_at,
        "elapsed_sec": elapsed_sec,
        "python": sys.version,
        "platform": platform.platform(),
        "in_port": options.in_port,
        "out_port": options.out_port,
        "switch_note": options.switch_note,
        "top": options.top,
        "options": dataclass_to_jsonable(options),
        "phase0_dual_baud_liveness": dataclass_to_jsonable(phase0_report),
        "completed_candidates": len(results),
        "candidate_count": candidate_total,
        "full_frame_candidate_count": len(full_candidates),
        "staged_candidate_count": staged_total,
        "dual_flow_discovery_enabled": dual_flow_discovery_enabled,
        "full_matrix_ran": full_matrix_ran,
        "full_matrix_skipped": full_matrix_skipped,
        "early_stopped": early_stopped,
        "operator_break_action": operator_break_action,
        "operator_break_stage": operator_break_stage,
        "recommendation_status": scan_recommendation_status(results),
    }
    ranked_results = sorted(results, key=result_sort_key, reverse=True)
    best_result = ranked_results[0] if ranked_results else None
    tied_results = top_tied_results(results)
    metadata["recommended_setting"] = (
        dataclass_to_jsonable(best_result.settings)
        if is_recommendable_result(best_result) and len(tied_results) <= 1
        else None
    )
    metadata["tied_top_settings"] = [
        dataclass_to_jsonable(result.settings) for result in tied_results
    ]

    print()
    print_report_title("DUAL BANK RESULTS")
    print("INPUT/OUTPUT FRAME PAIR RANKING AND SUMMARY.")
    print_dual_ranked_table(results, options.top)
    print_dual_scan_summary(
        results=results,
        total_candidates=candidate_total,
        elapsed_sec=elapsed_sec,
        early_stopped=early_stopped,
        top=options.top,
    )

    validation_results: list[CandidateResult] = []
    if (
        options.auto_validate_top_matches
        and results
        and operator_break_action is None
    ):
        ranked = ranked_dual_top_results(results, options.top)
        top_score = ranked[0].score if ranked else 0.0
        shortlist = [
            result for result in ranked if abs(result.score - top_score) <= 0.0001
        ]
        validation_results, validation_break_action = run_dual_bank_validation(
            serial_module=serial_module,
            options=options,
            shortlist=shortlist,
            logger=logger,
        )
        if validation_break_action is not None:
            operator_break_action = validation_break_action
            operator_break_stage = "DUAL VALIDATION"
            metadata["operator_break_action"] = operator_break_action
            metadata["operator_break_stage"] = operator_break_stage
    if options.auto_validate_flow_control:
        print()
        print_report_title("DUAL FLOW CONTROL NOTE")
        print("FLOW CONTROL WAS INCLUDED AFTER THE BEST DUAL FRAME PAIR.")
        print("INPUT FLOW IS THE PC TRANSMIT SIDE INTO THE BUFFER.")
        print("OUTPUT FLOW IS THE PC RECEIVE SIDE FROM THE BUFFER.")
        print(border_line(REPORT_WIDTH))

    write_dual_bank_text_report(
        options.text_report,
        metadata,
        phase0_report,
        results,
        validation_results=validation_results,
    )
    print()
    print_report_title("REPORT FILES")
    print_wrapped_value("  TEXT REPORT: ", f"{options.text_report} (APPENDED)")
    print_wrapped_value("  DEBUG LOG:   ", options.log_file)
    print(border_line(REPORT_WIDTH))
    if operator_break_action == "menu":
        raise ReturnToMainMenuAfterReport()
    if operator_break_action == "quit":
        raise QuitProgramAfterReport()
    if operator_break_action is not None:
        return OPERATOR_BREAK_EXIT_CODE
    return 0


def metadata_for_scan(
    options: ScanOptions,
    pyserial_version: str,
    payload: ProbePayload,
    candidate_count: int,
    scan_mode: str,
    started_at: str,
    completed_at: str | None = None,
    early_stopped: bool = False,
) -> dict[str, Any]:
    """Build structured metadata for reporting."""
    return {
        "tool": "serial_probe",
        "started_at": started_at,
        "completed_at": completed_at,
        "python": sys.version,
        "platform": platform.platform(),
        "pyserial_version": pyserial_version,
        "in_port": options.in_port,
        "out_port": options.out_port,
        "mode": "scan",
        "scan_mode": scan_mode,
        "candidate_count": candidate_count,
        "completed_candidates": None,
        "early_stopped": early_stopped,
        "top": options.top,
        "payload": dataclass_to_jsonable(payload),
        "options": dataclass_to_jsonable(options),
        "baud_list": BAUD_RATES,
        "baud_order": scan_bauds(options.min_baud, options.max_baud),
        "data_bits": DATA_BITS,
        "parities": PARITIES,
        "stop_bits": STOP_BITS,
        "flow_controls": FLOW_CONTROLS,
    }


def run_scan(options: ScanOptions) -> int:
    """Run the serial probe scan and write reports."""
    phase0_only = prompt_phase0_only_sweep()
    same_bank_settings = prompt_same_bank_settings()
    serial_module = import_or_install_pyserial()
    logger = setup_logging(options.log_file)
    pyserial_version = str(getattr(serial_module, "VERSION", "unknown"))
    logger.info("serial_probe started")

    if phase0_only:
        if same_bank_settings:
            return run_phase0_only_sweep(
                serial_module=serial_module,
                options=options,
                logger=logger,
            )
        return run_dual_bank_scan(
            serial_module=serial_module,
            options=options,
            logger=logger,
            phase0_only=True,
        )

    if not same_bank_settings:
        return run_dual_bank_scan(
            serial_module=serial_module,
            options=options,
            logger=logger,
        )

    scan_mode = prompt_scan_mode()
    turbo_enabled = prompt_yes_no_question(
        "Turbo discovery mode?",
        options.turbo_discovery_enabled,
    )
    switch_note = prompt_text(
        "Switch/jumper note for report",
        options.switch_note,
    )
    options = dataclasses.replace(
        options,
        turbo_discovery_enabled=turbo_enabled,
        switch_note=switch_note,
    )
    logger.info("scan-start turbo discovery=%s", turbo_enabled)

    all_candidates = prioritize_discovery_candidates(
        generate_candidates(options.min_baud, options.max_baud),
        options,
    )
    candidates = list(all_candidates)
    payload = generate_payload(options.payload_bytes)
    logger.info("options: %s", options)
    logger.info("payload: %s bytes, %s lines", payload.byte_count, payload.line_count)
    logger.info("candidates: %s", len(all_candidates))

    quick_mode = scan_mode == "quick"
    if quick_mode:
        exploratory_requested = True
        phase2_quick_accept = True
        print("QUICK SCAN: EXPLORATORY=YES PHASE2=YES")
        logger.info("scan type quick: exploratory=yes phase2=yes")
    else:
        print("FULL MODE: CONSERVATIVE OPERATOR PROMPTS ENABLED")
        exploratory_requested = prompt_yes_no_question(
            "Run quick exploratory mode first?",
            False,
        )
        phase2_quick_accept = False
        logger.info("scan type full: exploratory=%s", exploratory_requested)

    exploratory_selection: ExploratorySelection | None = None
    exploratory_narrowing_accepted = False
    phase2_candidate_source = PHASE2_CANDIDATE_SOURCE_FULL
    if exploratory_requested:
        exploratory_selection = run_exploratory_scan(
            serial_module=serial_module,
            options=options,
            candidates=all_candidates,
            logger=logger,
        )
        if exploratory_selection.narrowed_candidates:
            if phase2_quick_accept:
                exploratory_narrowing_accepted = True
                print("QUICK SCAN: PHASE2 QUICK LIST ACCEPTED")
            else:
                exploratory_narrowing_accepted = prompt_yes_no_question(
                    "Use these findings to narrow full analysis?",
                    False,
                )
            if exploratory_narrowing_accepted:
                candidates, phase2_candidate_source = phase2_candidates_after_exploratory(
                    all_candidates,
                    exploratory_selection,
                    exploratory_narrowing_accepted,
                )
                logger.info(
                    "exploratory narrowing accepted; full candidates=%s/%s",
                    len(candidates),
                    len(all_candidates),
                )
            else:
                logger.info("operator declined exploratory narrowing")
                candidates, phase2_candidate_source = phase2_candidates_after_exploratory(
                    all_candidates,
                    exploratory_selection,
                    exploratory_narrowing_accepted,
                )
                if phase2_candidate_source == PHASE2_CANDIDATE_SOURCE_VIABLE:
                    print(
                        "PHASE 2 SIGNAL-ONLY MODE: "
                        f"TESTING {len(candidates)}/{len(all_candidates)} "
                        "SETTINGS WITH QUICK LIFE SIGNAL."
                    )
                else:
                    print("FULL ANALYSIS WILL TEST ALL SELECTED SETTINGS.")
        else:
            logger.info(
                "exploratory narrowing unavailable: %s",
                exploratory_selection.fallback_reason,
            )
            candidates, phase2_candidate_source = phase2_candidates_after_exploratory(
                all_candidates,
                exploratory_selection,
                exploratory_narrowing_accepted,
            )
            if phase2_candidate_source == PHASE2_CANDIDATE_SOURCE_VIABLE:
                print(
                    "PHASE 2 SIGNAL-ONLY MODE: "
                    f"TESTING {len(candidates)}/{len(all_candidates)} "
                    "SETTINGS WITH QUICK LIFE SIGNAL."
                )
            else:
                print("FULL ANALYSIS WILL TEST ALL SELECTED SETTINGS.")
        logger.info(
            "phase 2 candidate source=%s candidates=%s/%s narrowed=%s viable=%s",
            phase2_candidate_source,
            len(candidates),
            len(all_candidates),
            len(exploratory_selection.narrowed_candidates),
            len(exploratory_selection.viable_candidates),
        )
    else:
        logger.info("operator skipped quick exploratory mode")
        logger.info(
            "phase 2 candidate source=%s candidates=%s/%s",
            phase2_candidate_source,
            len(candidates),
            len(all_candidates),
        )

    purge_buffer_output(
        serial_module=serial_module,
        options=options,
        logger=logger,
        reason="CLEAR DISCOVERY DATA BEFORE PHASE 1 SCAN.",
    )
    scan_started = time.monotonic()
    started_at = dt.datetime.now(dt.timezone.utc).isoformat()

    print_report_title("SCAN START")
    print(f"SCAN TYPE: {scan_type_label(scan_mode)}.")
    if quick_mode:
        print("QUICK SCAN: EXPLORATORY=YES PHASE2=YES.")
    else:
        print("FULL MODE: QUICK DISCOVERY RUNS ONLY IF YOU ANSWER YES.")
    print("DEVICE PATH: COM INPUT -> BUFFER -> COM OUTPUT.")
    print("ASSUMPTION: BOTH BUFFER SWITCH BANKS ARE SET THE SAME WAY.")
    print("NOTE: PROGRAM SETS COM PORTS; DEVICE MANAGER DEFAULTS ARE NOT USED.")
    if options.switch_note:
        print(f"SWITCH NOTE: {options.switch_note}")
    print(
        "TURBO DISCOVERY: "
        + ("ON; ADAPTIVE TIMING AND PRIORITY ORDER." if options.turbo_discovery_enabled else "OFF.")
    )
    if exploratory_narrowing_accepted:
        print("MODE: FULL ANALYSIS OF QUICK EXPLORATORY SHORTLIST.")
        print(f"SETTINGS: {len(candidates)}/{len(all_candidates)} SELECTED BY QUICK MODE.")
    elif phase2_candidate_source == PHASE2_CANDIDATE_SOURCE_VIABLE:
        print("MODE: FULL ANALYSIS OF QUICK EXPLORATORY SIGNAL SETTINGS.")
        print(
            f"SETTINGS: {len(candidates)}/{len(all_candidates)} "
            "SELECTED BY QUICK LIFE SIGNAL."
        )
    else:
        print("MODE: TEST ALL SELECTED SERIAL SETTINGS.")
        if exploratory_requested:
            print("QUICK MODE: RAN; FULL ANALYSIS NOT NARROWED.")
    print(
        f"PORTS: {options.in_port} -> {options.out_port}; "
        f"TEST={payload.byte_count} BYTES X {options.bursts}"
    )
    if options.no_pre_drain:
        print(
            f"CLEAR OUTPUT: OFF; OLD {options.out_port} DATA WILL BE SCORED."
        )
    else:
        print(
            f"CLEAR OUTPUT: ON; QUIET={options.pre_drain_quiet:.1f}S "
            f"LIMIT={options.pre_drain_timeout:.1f}S "
            f"MAX={options.max_drain_bytes} BYTES."
        )
    if options.ask_on_top_match:
        print("TOP MATCH PROMPT: ON; PASS ASKS WHETHER TO CONTINUE.")
    else:
        print("TOP MATCH PROMPT: OFF.")
    if options.auto_validate_top_matches:
        size2 = (
            f"{options.validate_size_2_tie_bytes} BYTES"
            if options.validate_size_2_tie_bytes > 0
            else "OFF"
        )
        print(
            "STAGE 2 VALIDATION: ON; "
            f"SIZE1={options.validate_size_1_bytes} BYTES SIZE2={size2}."
        )
    else:
        print("STAGE 2 VALIDATION: OFF.")
    if options.auto_validate_flow_control:
        print(
            "FLOW VALIDATION: ON; "
            f"SIZE={options.flow_validate_size_bytes} BYTES AFTER BEST FRAME."
        )
    else:
        print("FLOW VALIDATION: OFF.")
    print(f"EFFECTIVE TIMING: {effective_timing_range_label(options, candidates)}.")
    print(f"SCREEN UPDATE: {options.progress_interval:.1f}S WHILE SETTING RUNS.")
    print_progress_legend()
    print(f"REPORT: {options.text_report} (APPEND)")
    print(f"DEBUG LOG: {options.log_file}")
    rough_total = 0.0
    for candidate in candidates:
        timing = effective_discovery_timing(options, candidate, options.payload_bytes)
        rough_total += estimated_transmit_seconds(
            candidate,
            options.payload_bytes,
        ) * options.bursts
        per_burst_wait = timing.read_timeout + (timing.settle_ms / 1000.0)
        if not options.no_pre_drain:
            per_burst_wait += timing.pre_drain_quiet
        rough_total += per_burst_wait * options.bursts
    print(
        f"START EST.: {format_duration(rough_total)}; "
        f"FINISH ABOUT {format_finish_clock(rough_total)}"
    )

    results: list[CandidateResult] = []
    early_stopped = False
    early_stop_reason: str | None = None
    operator_break_action: str | None = None
    operator_break_stage: str | None = None
    candidate_index = 0
    while candidate_index < len(candidates):
        settings = candidates[candidate_index]
        display_index = candidate_index + 1
        try:
            result = run_candidate(
                serial_module=serial_module,
                index=display_index,
                total=len(candidates),
                settings=settings,
                options=options,
                payload=payload,
                logger=logger,
                progress=console_progress,
            )
        except KeyboardInterrupt:
            action = prompt_operator_break_action("SCAN")
            if action == "resume":
                logger.info(
                    "operator resumed scan at candidate %s/%s %s",
                    display_index,
                    len(candidates),
                    settings.label(),
                )
                continue
            operator_break_action = action
            operator_break_stage = "SCAN"
            early_stopped = True
            early_stop_reason = f"operator-break-{action}"
            logger.info(
                "operator break during scan; action=%s completed=%s/%s",
                action,
                len(results),
                len(candidates),
            )
            break
        results.append(result)
        print(format_progress(result), flush=True)
        print(format_scan_eta(len(results), len(candidates), scan_started), flush=True)
        if options.ask_on_top_match and is_top_match_result(result):
            if ask_continue_after_top_match(result):
                logger.info("operator chose to continue after top match: %s", result.settings.label())
            else:
                early_stopped = True
                early_stop_reason = "operator-ended-after-top-match"
                logger.info("operator ended scan after top match: %s", result.settings.label())
                break
        candidate_index += 1

    completed_at = dt.datetime.now(dt.timezone.utc).isoformat()
    elapsed_sec = time.monotonic() - scan_started
    metadata = metadata_for_scan(
        options=options,
        pyserial_version=pyserial_version,
        payload=payload,
        candidate_count=len(candidates),
        scan_mode=scan_mode,
        started_at=started_at,
        completed_at=completed_at,
        early_stopped=early_stopped,
    )
    metadata["completed_candidates"] = len(results)
    metadata["elapsed_sec"] = elapsed_sec
    metadata["switch_note"] = options.switch_note
    metadata["early_stop_reason"] = early_stop_reason
    metadata["operator_break_action"] = operator_break_action
    metadata["operator_break_stage"] = operator_break_stage
    metadata["recommendation_status"] = scan_recommendation_status(results)
    ranked_results = sorted(results, key=result_sort_key, reverse=True)
    best_result = ranked_results[0] if ranked_results else None
    tied_results = top_tied_results(results)
    metadata["recommended_setting"] = (
        dataclass_to_jsonable(best_result.settings)
        if is_recommendable_result(best_result) and len(tied_results) <= 1
        else None
    )
    metadata["tied_top_settings"] = [
        dataclass_to_jsonable(result.settings) for result in tied_results
    ]
    metadata["tie_score_tolerance"] = TIE_SCORE_TOLERANCE
    metadata["full_candidate_count_before_exploratory"] = len(all_candidates)
    metadata["full_candidate_count_after_exploratory"] = len(candidates)
    metadata["phase2_candidate_source"] = phase2_candidate_source
    metadata["phase2_candidate_count"] = len(candidates)
    metadata["exploratory_mode"] = exploratory_metadata(
        requested=exploratory_requested,
        narrowing_accepted=exploratory_narrowing_accepted,
        selection=exploratory_selection,
        original_candidate_count=len(all_candidates),
        final_candidate_count=len(candidates),
        options=options,
        phase2_candidate_source=phase2_candidate_source,
    )
    logger.info(
        "serial_probe completed; candidates=%s/%s early_stopped=%s exploratory_narrowed=%s phase2_source=%s",
        len(results),
        len(all_candidates),
        early_stopped,
        exploratory_narrowing_accepted,
        phase2_candidate_source,
    )
    logger.info("text_report=%s", options.text_report)

    print()
    print_report_title("PHASE 1 RESULTS")
    print("BASE SCAN RANKING AND SUMMARY.")
    print_ranked_table(results, options.top, report_title="PHASE 1 FINAL REPORT")
    print_scan_summary(
        results=results,
        total_candidates=len(candidates),
        elapsed_sec=elapsed_sec,
        early_stopped=early_stopped,
        top=options.top,
        early_stop_reason=early_stop_reason,
    )
    print_wrapped_value("  NOTE:                 ", f"SCAN TYPE {scan_type_label(scan_mode)}.")
    if exploratory_narrowing_accepted:
        print_wrapped_value(
            "  NOTE:                 ",
            "FULL ANALYSIS USED QUICK EXPLORATORY "
            f"CANDIDATES ({len(candidates)}/{len(all_candidates)}).",
        )
    elif phase2_candidate_source == PHASE2_CANDIDATE_SOURCE_VIABLE:
        print_wrapped_value(
            "  NOTE:                 ",
            "FULL ANALYSIS USED QUICK EXPLORATORY "
            f"SIGNAL CANDIDATES ({len(candidates)}/{len(all_candidates)}).",
        )
    elif exploratory_requested:
        print("  NOTE:                 QUICK EXPLORATORY DID NOT NARROW FULL ANALYSIS.")
    if exploratory_selection is not None:
        phase0 = exploratory_selection.phase0_liveness
        print_wrapped_value(
            "  NOTE:                 ",
            "PHASE 0 BAUD LIVENESS "
            f"{len(phase0.alive_bauds)}/{len(phase0.tested_bauds)}.",
        )
        if phase0.fallback_to_all_bauds and phase0.fallback_reason:
            print_wrapped_value(
                "  NOTE:                 ",
                f"PHASE 0 FALLBACK: {phase0.fallback_reason}",
            )
    if exploratory_selection is not None and exploratory_selection.baud_focus.engaged:
        report = exploratory_selection.baud_focus
        print_wrapped_value(
            "  NOTE:                 ",
            "QUICK BAUD FOCUS "
            f"{report.focused_baud}; "
            f"DEFERRED={report.deferred_candidate_count}.",
        )
        if report.release_reason:
            print_wrapped_value(
                "  NOTE:                 ",
                f"QUICK BAUD FOCUS RELEASED: {report.release_reason}.",
            )
    validation_results: list[CandidateResult] = []
    if options.auto_validate_top_matches and results and operator_break_action is None:
        ranked = ranked_top_results(results, options.top)
        top_score = ranked[0].score if ranked else 0.0
        shortlist = [result for result in ranked if abs(result.score - top_score) <= 0.0001]
        if shortlist:
            purge_buffer_output(
                serial_module=serial_module,
                options=options,
                logger=logger,
                reason="CLEAR PHASE 1 DATA BEFORE VALIDATION.",
            )
            print()
            print_report_title("PHASE 2 VALIDATION")
            print(
                f"SHORTLIST: {len(shortlist)} TOP-SCORE SETTING(S) "
                f"FROM SCORE={top_score:.2f}."
            )
            stage2_options = dataclasses.replace(
                options,
                turbo_discovery_enabled=False,
                payload_bytes=options.validate_size_1_bytes,
                bursts=1,
                ask_on_top_match=False,
            )
            stage2_payload = generate_payload(options.validate_size_1_bytes)
            stage2_results: list[CandidateResult] = []
            validation_index = 0
            while validation_index < len(shortlist):
                candidate = shortlist[validation_index]
                display_index = validation_index + 1
                print(
                    f"VALIDATION PASS 1: {stage2_payload.byte_count} BYTES "
                    f"{display_index}/{len(shortlist)} {candidate.settings.label()}"
                )
                try:
                    stage2_result = run_candidate(
                        serial_module=serial_module,
                        index=display_index,
                        total=len(shortlist),
                        settings=candidate.settings,
                        options=stage2_options,
                        payload=stage2_payload,
                        logger=logger,
                        progress=console_progress,
                    )
                except KeyboardInterrupt:
                    action = prompt_operator_break_action("VALIDATION")
                    if action == "resume":
                        logger.info(
                            "operator resumed validation at %s/%s %s",
                            display_index,
                            len(shortlist),
                            candidate.settings.label(),
                        )
                        continue
                    operator_break_action = action
                    operator_break_stage = "VALIDATION"
                    logger.info(
                        "operator break during validation; action=%s completed=%s/%s",
                        action,
                        len(stage2_results),
                        len(shortlist),
                    )
                    break
                stage2_results.append(stage2_result)
                print(format_progress(stage2_result), flush=True)
                validation_index += 1
            tied_after_1 = top_tied_results(stage2_results)
            final_stage2_results = stage2_results
            if (
                operator_break_action is None
                and len(tied_after_1) > 1
                and options.validate_size_2_tie_bytes > 0
            ):
                print(
                    f"TIE REMAINS ({len(tied_after_1)}). "
                    f"RUNNING PASS 2 AT {options.validate_size_2_tie_bytes} BYTES."
                )
                purge_buffer_output(
                    serial_module=serial_module,
                    options=options,
                    logger=logger,
                    reason="CLEAR VALIDATION PASS 1 DATA BEFORE PASS 2.",
                )
                stage2_options = dataclasses.replace(
                    stage2_options,
                    payload_bytes=options.validate_size_2_tie_bytes,
                )
                stage2_payload = generate_payload(options.validate_size_2_tie_bytes)
                tie_results: list[CandidateResult] = []
                tie_index = 0
                while tie_index < len(tied_after_1):
                    candidate = tied_after_1[tie_index]
                    display_index = tie_index + 1
                    print(
                        f"VALIDATION PASS 2: {stage2_payload.byte_count} BYTES "
                        f"{display_index}/{len(tied_after_1)} "
                        f"{candidate.settings.label()}"
                    )
                    try:
                        tie_result = run_candidate(
                            serial_module=serial_module,
                            index=display_index,
                            total=len(tied_after_1),
                            settings=candidate.settings,
                            options=stage2_options,
                            payload=stage2_payload,
                            logger=logger,
                            progress=console_progress,
                        )
                    except KeyboardInterrupt:
                        action = prompt_operator_break_action("VALIDATION")
                        if action == "resume":
                            logger.info(
                                "operator resumed validation pass 2 at %s/%s %s",
                                display_index,
                                len(tied_after_1),
                                candidate.settings.label(),
                            )
                            continue
                        operator_break_action = action
                        operator_break_stage = "VALIDATION"
                        logger.info(
                            "operator break during validation pass 2; "
                            "action=%s completed=%s/%s",
                            action,
                            len(tie_results),
                            len(tied_after_1),
                        )
                        break
                    tie_results.append(tie_result)
                    print(format_progress(tie_result), flush=True)
                    tie_index += 1
                final_stage2_results = tie_results
            elif operator_break_action is None and len(tied_after_1) > 1:
                print("TIE REMAINS AFTER PASS 1. PASS 2 IS OFF.")

            print("STAGE 2 FINAL RANKING:")
            print_ranked_table(
                final_stage2_results,
                min(options.top, len(final_stage2_results)),
                report_title="PHASE 2 FINAL REPORT",
            )
            validation_results = list(final_stage2_results)
    flow_validation_results: list[FlowControlValidationResult] = []
    if (
        options.auto_validate_flow_control
        and results
        and operator_break_action is None
    ):
        flow_validation_results, flow_break_action = run_flow_control_validation(
            serial_module=serial_module,
            options=options,
            results=results,
            validation_results=validation_results,
            logger=logger,
        )
        if flow_break_action is not None:
            operator_break_action = flow_break_action
            operator_break_stage = "FLOW VALIDATION"
    metadata["flow_control_validation"] = {
        "enabled": options.auto_validate_flow_control,
        "ran": bool(flow_validation_results),
        "payload_bytes": options.flow_validate_size_bytes,
        "recommendation": flow_control_validation_recommendation(
            flow_validation_results
        ),
        "results": dataclass_to_jsonable(flow_validation_results),
    }
    if operator_break_action is not None:
        metadata["operator_break_action"] = operator_break_action
        metadata["operator_break_stage"] = operator_break_stage
        if operator_break_stage in {"VALIDATION", "FLOW VALIDATION"}:
            metadata["early_stop_reason"] = f"operator-break-{operator_break_action}"
    write_text_report(
        options.text_report,
        metadata,
        results,
        validation_results=validation_results,
        flow_validation_results=flow_validation_results,
    )
    print()
    print_report_title("REPORT FILES")
    print_wrapped_value("  TEXT REPORT: ", f"{options.text_report} (APPENDED)")
    print_wrapped_value("  DEBUG LOG:   ", options.log_file)
    print(border_line(REPORT_WIDTH))
    if operator_break_action == "menu":
        raise ReturnToMainMenuAfterReport()
    if operator_break_action == "quit":
        raise QuitProgramAfterReport()
    if operator_break_action is not None:
        return OPERATOR_BREAK_EXIT_CODE
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Program entry point."""
    enable_terminal_style()
    if sys.version_info < (3, 10):
        print("PYTHON 3.10 OR NEWER IS REQUIRED.")
        return 2
    args = list(sys.argv[1:] if argv is None else argv)
    if any(arg in {"-h", "--help", "help"} for arg in args):
        print_menu_help()
        return 0
    if args:
        print("THIS PROGRAM IS SET FROM THE TERMINAL MENU.")
        print("RUN WITHOUT OPTIONS:")
        print("  PYTHON SERIAL_PROBE.PY")
        return 2
    options = default_scan_options()
    last_status = 0
    scan_started = False
    while True:
        try:
            selected_options = interactive_menu(options)
        except ReturnToMainMenuAfterReport:
            last_status = OPERATOR_BREAK_EXIT_CODE
            continue
        except QuitProgramAfterReport:
            return OPERATOR_BREAK_EXIT_CODE
        if selected_options is None:
            print("PROGRAM ENDED." if scan_started else "NO SCAN STARTED.")
            return last_status
        options = selected_options
        while True:
            scan_started = True
            try:
                last_status = run_scan(options)
                action = prompt_after_scan_action()
            except ReturnToMainMenuAfterReport:
                last_status = OPERATOR_BREAK_EXIT_CODE
                break
            except QuitProgramAfterReport:
                return OPERATOR_BREAK_EXIT_CODE
            except KeyboardInterrupt:
                print()
                print("INTERRUPTED BY OPERATOR.")
                last_status = OPERATOR_BREAK_EXIT_CODE
                action = prompt_after_scan_action("TEST INTERRUPTED")
            if action == "rerun":
                continue
            if action == "menu":
                break
            return last_status


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
        print("INTERRUPTED BY OPERATOR.")
        raise SystemExit(OPERATOR_BREAK_EXIT_CODE)
