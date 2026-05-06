#!/usr/bin/env python3
"""serial_probe.py

Interactive serial-port discovery and validation for printer-buffer devices.

The program treats the PC transmit port as the buffer input side and the PC
receive port as the buffer output side. It sends deterministic probe payloads,
scores the bytes returned by the device under test, and writes operator-facing
reports for baud, frame, flow-control, raw-byte, and ETX/ACK observations.

Terminology used throughout this module:
    * A candidate is one serial setting or one independent input/output pair.
    * Phase 0 is a fixed-frame baud-liveness gate, not a final ranking.
    * A nonce identifies one run/candidate/trial so stale buffer output can be
      separated from data produced by the current test.
"""

from __future__ import annotations

import atexit
import base64
import ctypes
import dataclasses
import datetime as dt
import logging
import math
import os
import platform
import re
import statistics
import string
import sys
import threading
import textwrap
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence, TextIO

BAUD_RATES: list[int] = [
    300,
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
FLOW_CONTROLS: list[str] = ["none", "xon/xoff", "dsr/dtr", "rts/cts"]
KIB_BYTES = 1024
DEFAULT_BURSTS = 1
DEFAULT_PAYLOAD_BYTES = 512
DEFAULT_READ_TIMEOUT = 2.0
DEFAULT_SETTLE_MS = 50
DEFAULT_PROGRESS_INTERVAL = 1.0
DEFAULT_PRE_DRAIN_TIMEOUT = 0.5
DEFAULT_PRE_DRAIN_QUIET = 0.1
DEFAULT_MAX_DRAIN_BYTES = 128 * KIB_BYTES
DEFAULT_EIGHT_BIT_PAYLOAD_BYTES = 512
TURBO_DISCOVERY_ENABLED_DEFAULT = False
BUFFER_PURGE_ENABLED = True
BUFFER_PURGE_CAPACITY_BYTES = 64 * KIB_BYTES
BUFFER_PURGE_QUIET_SECONDS = 1.5
BUFFER_PURGE_PER_BAUD_MAX_SECONDS = 2.5
BUFFER_PURGE_PROGRESS_INTERVAL = 1.0
FLOW_VALIDATE_PAYLOAD_BYTES = 1024
FLOW_VALIDATE_READ_TIMEOUT = 2.0
FLOW_VALIDATE_HOLD_SECONDS = 1.0
FLOW_VALIDATE_RELEASE_SETTLE_SECONDS = 0.15
INPUT_BACKPRESSURE_STRESS_ENABLED_DEFAULT = True
INPUT_BACKPRESSURE_STRESS_BYTES = 128 * KIB_BYTES
INPUT_BACKPRESSURE_FLOW_CONTROLS = ["xon/xoff", "dsr/dtr", "rts/cts"]
INPUT_BACKPRESSURE_METHOD = "buffer-full"
INPUT_BACKPRESSURE_MONITOR_SECONDS = 0.05
INPUT_BACKPRESSURE_STALL_SECONDS = 1.5
INPUT_BACKPRESSURE_SIGNAL_REASONS = (
    "DCD DROPPED",
    "DSR DROPPED",
    "CTS DROPPED",
)
INPUT_BACKPRESSURE_OBSERVED_REASONS = frozenset(
    (*INPUT_BACKPRESSURE_SIGNAL_REASONS, "WRITE STALLED")
)
RECEIVE_DEADLINE_SAFETY_FACTOR = 2.0
RECEIVE_DEADLINE_MARGIN_SECONDS = 2.0
DEFAULT_BANK2_JOB_TIMEOUT_OBSERVE_SECONDS = 10.0
DEFAULT_ETX_ACK_OBSERVE_SECONDS = 2.0
BANK2_BEHAVIOR_MAX_TIED_TARGETS = 3
TURBO_SETTLE_MS = 10
TURBO_PRE_DRAIN_TIMEOUT = 0.25
TURBO_PRE_DRAIN_QUIET = 0.05
TURBO_READ_TIMEOUT_HIGH_BAUD = 0.35
TURBO_READ_TIMEOUT_MID_BAUD = 0.45
TURBO_READ_TIMEOUT_LOW_BAUD = 0.70
TURBO_READ_TIMEOUT_VERY_LOW_BAUD = 1.00
TURBO_COMPLETION_QUIET = 0.05
DEFAULT_COMPLETION_QUIET = 0.10
LOW_BAUD_THRESHOLD = 4800
LOW_BAUD_PRE_DRAIN_QUIET_MAX = 0.75
LOW_BAUD_PRE_DRAIN_QUIET_RATIO = 0.25
LOW_BAUD_PRE_DRAIN_MULTIPLIER = 1.25
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
PHASE0_LOW_BAUD_THRESHOLD = LOW_BAUD_THRESHOLD
PHASE0_LOW_BAUD_READ_TIMEOUT_MAX = 6.0
PHASE0_LOW_BAUD_PRE_DRAIN_QUIET_MAX = 1.5
PHASE0_LOW_BAUD_READ_MULTIPLIER = 1.25
PHASE0_LOW_BAUD_PRE_DRAIN_MULTIPLIER = 2.5
PHASE0_BASELINE_DATA_BITS = 8
PHASE0_BASELINE_PARITY = "even"
PHASE0_BASELINE_STOP_BITS = 1
PHASE0_BASELINE_FLOW_CONTROL = "none"
DUAL_PHASE0_BAUD_PAIR_LIMIT = 6
DUAL_PHASE0_FALLBACK_PAIR_LIMIT = 4
PHASE0_NO_SIGNAL_AUTO_FALLBACK_BAUD_LIMIT = 2
FNV_OFFSET_32 = 0x811C9DC5
FNV_PRIME_32 = 0x01000193
ProgressCallback = Callable[[str], None]
ANSI_GREEN = "\033[92m"
ANSI_RESET = "\033[0m"
STD_OUTPUT_HANDLE = -11
ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
TERMINAL_COLUMNS = 80
SCREEN_WIDTH = TERMINAL_COLUMNS
REPORT_WIDTH = TERMINAL_COLUMNS
PROGRESS_TIMESTAMP_WIDTH = len("00:00:00 ")
PROGRESS_WIDTH = TERMINAL_COLUMNS - PROGRESS_TIMESTAMP_WIDTH
PAGE_BODY_LINES = 22
HELP_BODY_LINES = 20
SESSION_TEXT_REPORT_PATHS: set[Path] = set()
SESSION_LOG_FILE_PATHS: set[Path] = set()
RECOMMENDATION_MIN_SCORE = 90.0
TOP_MATCH_MIN_SCORE = 99.0
TIE_SCORE_TOLERANCE = 0.5
PERFECT_SCORE = 100.0
OPERATOR_BREAK_EXIT_CODE = 130
PAYLOAD_MODE_ASCII = "ascii"
PAYLOAD_MODE_EIGHT_BIT = "eight_bit"
PAYLOAD_MODE_PHASE0 = "phase0"
STALE_STATUSES = {"stale-output", "wrong-nonce", "mixed-nonce"}
SERIAL_IO_ERRORS: tuple[type[Exception], ...] = (
    OSError,
    RuntimeError,
)
BEST_EFFORT_PLATFORM_ERRORS: tuple[type[Exception], ...] = (
    AttributeError,
    OSError,
)


def flow_control_code(flow_control: str) -> str:
    """Return the short flow-control code used in compact result displays.

    Args:
        flow_control: Program-level flow-control name.

    Returns:
        The fixed-width-ish display code for known flow-control values.

    Raises:
        KeyError: If `flow_control` is outside the configured flow-control set.
    """
    return {
        "none": "NONE",
        "xon/xoff": "XON",
        "rts/cts": "RTS",
        "dsr/dtr": "DSR",
    }[flow_control]


class ReturnToMainMenuAfterReport(Exception):
    """Signal that the operator asked for the main menu after report writing."""


class SerialConfigurationError(RuntimeError):
    """Serial driver rejected one of the requested port settings."""


class ReturnToMainMenu(Exception):
    """Signal that the operator asked for the main menu without starting a scan."""


class QuitProgramAfterReport(Exception):
    """Signal that the operator asked to quit after report writing."""


@dataclass(frozen=True)
class SerialSettings:
    """One concrete baud/frame/flow profile for a serial side.

    Notes:
        Instances are frozen and hashable because candidate lists are de-duped
        by value. Values stay in program terminology until `serial_constants`
        maps them to pyserial constants at the I/O boundary.
    """

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
    """Independent input-side and output-side serial settings.

    Dual settings are used when a buffer may expose separate switch banks for
    the input path and output path. Compatibility properties intentionally
    return input-side values so shared timing and progress helpers can accept
    either `SerialSettings` or `DualSerialSettings`.
    """

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
        return f"IN {self.input_mode()} >> OUT {self.output_mode()}"


@dataclass(frozen=True)
class ProbeNonce:
    """Run/candidate/trial identity embedded in generated probe payloads.

    The nonce is part of the checksum-covered payload text. Scoring treats
    mismatched nonces as stale or wrong-candidate data rather than as a weak
    partial match.
    """

    run_id: str
    candidate_id: str
    trial_id: str
    switch_note_hash: str | None = None

    def fields(self) -> tuple[tuple[str, str], ...]:
        """Return compact nonce fields for payload text."""
        values = [
            ("RUN", self.run_id),
            ("CAND", self.candidate_id),
            ("TRIAL", self.trial_id),
        ]
        if self.switch_note_hash:
            values.append(("NOTE", self.switch_note_hash))
        return tuple(values)

    def compact(self) -> str:
        """Return a compact one-line nonce label."""
        return " ".join(f"{name}={value}" for name, value in self.fields())


@dataclass(frozen=True)
class ProbePayload:
    """Generated probe bytes plus metadata needed by scoring.

    Notes:
        `data` is the exact byte stream to send. `payload_mode` determines how
        `score_received` interprets structure, line checksums, and high-bit
        challenge bytes.
    """

    data: bytes
    line_count: int
    byte_count: int
    body_hash: str
    payload_mode: str = PAYLOAD_MODE_ASCII
    nonce: ProbeNonce | None = None
    high_bit_block: bytes = b""


@dataclass(frozen=True)
class ScoreMetrics:
    """Byte, structure, nonce, and high-bit metrics for one received burst.

    Ratios are computed against the expected payload where applicable. Counts
    such as `missing_bytes` and `extra_bytes` use byte-count deltas, while
    checksum line counts only include structurally valid probe lines.
    """

    exact_byte_match_ratio: float
    line_integrity_ratio: float
    missing_bytes: int
    extra_bytes: int
    printable_ascii_ratio: float
    length_ratio: float
    start_marker_present: bool
    end_marker_present: bool
    exact_prefix_bytes: int = 0
    valid_probe_line_count: int = 0
    current_nonce_line_count: int = 0
    wrong_nonce_line_count: int = 0
    high_bit_bytes_sent: int = 0
    high_bit_bytes_received: int = 0
    high_bit_exact_ratio: float = 0.0
    high_bit_stripped_count: int = 0
    seven_bit_masked_match_ratio: float = 0.0


@dataclass(frozen=True)
class ParsedProbeLine:
    """One checksum-valid probe line parsed from received bytes."""

    line_number: int
    checksum: int
    nonce: ProbeNonce | None
    raw_left: bytes


@dataclass(frozen=True)
class ScoreResult:
    """Score, metrics, classification, and evidence for one received burst.

    The score is a bounded 0..100 confidence value, not a probability.
    `classification` carries the operational reason used later for status
    precedence and reporting.
    """

    score: float
    metrics: ScoreMetrics
    classification: str = "no-data"
    evidence: tuple[str, ...] = ()


@dataclass(frozen=True)
class DrainResult:
    """Outcome from draining stale output before a probe or validation stage."""

    bytes_drained: int
    elapsed_sec: float
    quiet: bool
    reason: str
    error: str | None


@dataclass(frozen=True)
class TrialResult:
    """Result from one payload burst under one candidate setting.

    Notes:
        The object is immutable after construction. It records both transfer
        evidence and timing slices so candidate-level aggregation can preserve
        stale-output and serial-error precedence.
    """

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
    score_classification: str = "unknown"
    evidence: tuple[str, ...] = ()
    nonce_summary: str = ""
    payload_mode: str = PAYLOAD_MODE_ASCII
    bytes_received_at_write_done: int = 0


@dataclass(frozen=True)
class TimingBreakdown:
    """Wall-clock stage timing for one candidate, burst, or validation step."""

    open_setup_sec: float
    drain_sec: float
    write_sec: float
    read_wait_sec: float
    other_sec: float


@dataclass(frozen=True)
class CandidateResult:
    """Aggregated result for one serial-settings candidate.

    A candidate may be a same-setting scan row or an independent input/output
    pair. `trials` are preserved because report interpretation depends on
    repeatability, stale nonces, and per-burst errors.
    """

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
    evidence: tuple[str, ...] = ()


@dataclass(frozen=True)
class ScanOptions:
    """Runtime configuration shared by discovery, validation, and reports.

    The interactive menu mutates configuration by creating replacement
    instances. Functions that need a run id should call `ensure_run_id` or
    explicitly replace `run_id` before payload generation.
    """

    in_port: str
    out_port: str
    input_baud: int
    output_baud: int
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
    auto_validate_flow_control: bool
    flow_validate_size_bytes: int
    auto_stress_input_backpressure: bool
    input_backpressure_stress_bytes: int
    run_id: str = ""


@dataclass(frozen=True)
class MenuSelection:
    """Main-menu action, active options, and optional workflow key."""

    action: str
    options: ScanOptions
    workflow: str = ""


@dataclass(frozen=True)
class EffectiveTiming:
    """Timing values after discovery speed and low-baud policies are applied."""

    read_timeout: float
    settle_ms: int
    pre_drain_quiet: float
    pre_drain_timeout: float
    completion_quiet: float


@dataclass(frozen=True)
class Phase0LivenessDecision:
    """Boolean Phase 0 liveness decision plus the operator-facing reason."""

    alive: bool
    reason: str


@dataclass(frozen=True)
class DualBaudLivenessResult:
    """Phase 0 result for one independently tested input/output baud pair."""

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
    """Run summary for the dual-bank Phase 0 baud matrix.

    `selected_pairs` is the subset that later frame sweeps expand. It may come
    from alive pairs or from the same-baud fallback path when the operator
    chooses to continue after no Phase 0 signal.
    """

    ran: bool
    tested_pairs: list[tuple[int, int]]
    total_pairs: int
    alive_pairs: list[tuple[int, int]]
    selected_pairs: list[tuple[int, int]]
    fallback_reason: str | None
    elapsed_sec: float
    results: list[DualBaudLivenessResult]


@dataclass(frozen=True)
class FlowControlValidationResult:
    """Result from one flow-control validation method.

    The `method` field distinguishes transfer-only sweeps, output hold/release
    checks, and input-side buffer-full stress. Those methods prove different
    things and must not be collapsed into one generic "flow works" result.
    """

    flow_control: str
    method: str
    settings: SerialSettings | DualSerialSettings
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
    modem_line_observations: tuple[tuple[str, dict[str, bool]], ...] = ()


@dataclass(frozen=True)
class Bank2BehaviorProbeResult:
    """Byte-level raw printer-job observation for one known-baud probe."""

    name: str
    settings: SerialSettings | DualSerialSettings
    bytes_sent: int
    bytes_received: int
    sent_hash: str
    received_hash: str
    first_mismatch_offset: int | None
    missing_bytes: int
    extra_bytes: int
    exact_match: bool
    form_feed_inserted: bool
    cr_lf_changed: bool
    received_preview_ascii: str
    received_preview_hex: str
    status: str
    reason: str
    error: str | None
    elapsed_sec: float


@dataclass(frozen=True)
class Bank2EtxAckProbeResult:
    """Forward ETX and reverse ACK observation for one known-baud frame."""

    settings: SerialSettings | DualSerialSettings
    forward_bytes_sent: int
    forward_bytes_received: int
    reverse_bytes_sent: int
    reverse_bytes_received: int
    etx_forward_seen: bool
    etx_forward_exact: bool
    ack_reverse_seen: bool
    reverse_path_observed: bool
    forward_preview_ascii: str
    forward_preview_hex: str
    reverse_preview_ascii: str
    reverse_preview_hex: str
    status: str
    reason: str
    error: str | None
    elapsed_sec: float


@dataclass(frozen=True)
class Bank2CharacterizationResult:
    """Report block for one known-baud device or switch state.

    This groups evidence from targeted frame checks, high-bit tests, raw byte
    behavior, ETX/ACK probing, and flow-control validation. The conclusion is a
    compact operator summary, not a claim about DIP-switch semantics.
    """

    switch_note: str
    known_baud_text: str
    ascii_results: list[CandidateResult]
    eight_bit_results: list[CandidateResult]
    flow_results: list[FlowControlValidationResult]
    behavior_results: list[Bank2BehaviorProbeResult]
    etx_ack_results: list[Bank2EtxAckProbeResult]
    stale_data_seen: bool
    conclusion: str
    run_id: str
    flow_skip_reason: str | None = None


def fnv1a32(data: bytes) -> int:
    """Return the deterministic FNV-1a 32-bit hash for bytes.

    Args:
        data: Byte sequence to hash.

    Returns:
        Unsigned 32-bit FNV-1a hash value.
    """
    value = FNV_OFFSET_32
    for byte in data:
        value ^= byte
        value = (value * FNV_PRIME_32) & 0xFFFFFFFF
    return value


def sanitize_nonce_value(value: object, fallback: str = "NA") -> str:
    """Return a compact printable token safe for one probe-line field.

    Args:
        value: Value to convert into a marker token.
        fallback: Token used when `value` is blank or sanitizes to nothing.

    Returns:
        A token of at most 48 ASCII characters containing only marker-safe
        letters, digits, underscores, periods, colons, or hyphens.
    """
    text = str(value).strip()
    if not text:
        text = fallback
    cleaned = re.sub(r"[^A-Za-z0-9_.:-]+", "_", text)
    return cleaned[:48] or fallback


def make_run_id(prefix: str = "R") -> str:
    """Return a short process-local run id for nonce-bearing payloads.

    The value includes UTC time, `time_ns`, and process id hash input. It is
    intended to distinguish runs for stale-output detection, not to be a secure
    or reproducible identifier.
    """
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d%H%M%S")
    entropy = fnv1a32(f"{stamp}:{time.time_ns()}:{os.getpid()}".encode("ascii"))
    return sanitize_nonce_value(f"{prefix}{stamp}{entropy:08X}"[-24:])


def switch_note_hash(switch_note: str | None) -> str | None:
    """Return a stable compact hash for an operator switch-note string.

    Blank notes return `None`; non-blank notes return an eight-character
    uppercase FNV-1a hash used in payload nonces and reports.
    """
    note = (switch_note or "").strip()
    if not note:
        return None
    return f"{fnv1a32(note.encode('utf-8', 'replace')):08X}"


def candidate_nonce_id(
    _settings: SerialSettings | DualSerialSettings,
    candidate_index: int,
) -> str:
    """Return the candidate id embedded in nonce-bearing payloads.

    Notes:
        `_settings` is accepted for call-site symmetry with candidate execution
        helpers. The current payload format uses the candidate index to keep
        marker lines short and stable within one run.
    """
    return sanitize_nonce_value(f"C{candidate_index:04d}")


def trial_nonce_id(burst_index: int) -> str:
    """Return a compact trial id for one payload burst."""
    return sanitize_nonce_value(f"T{burst_index:02d}")


def representative_nonce() -> ProbeNonce:
    """Return a fixed-width nonce used for conservative size calculations."""
    return ProbeNonce(
        run_id="R00000000000000000000",
        candidate_id="C0000_00000000",
        trial_id="T00",
        switch_note_hash="00000000",
    )


def nonce_field_text(nonce: ProbeNonce | None) -> str:
    """Return probe marker field text for a nonce."""
    if nonce is None:
        return ""
    return " " + nonce.compact()


def make_probe_line(
    line_number: int,
    kind: str,
    data: str,
    nonce: ProbeNonce | None = None,
) -> bytes:
    """Build one checksum-protected ASCII line for a probe payload.

    The checksum covers the line text to the left of `HASH=`. Received lines
    are accepted only when that checksum verifies, which makes line-integrity
    scoring insensitive to unrelated serial noise before or after the line.
    """
    block_number = (line_number - 1) % 32
    nonce_text = nonce_field_text(nonce)
    left = (
        f"LINE {line_number:06d}{nonce_text} BLOCK={block_number:02d} "
        f"TYPE={kind} DATA={data}"
    ).encode("ascii")
    checksum = fnv1a32(left)
    return left + f" HASH={checksum:08X}\r\n".encode("ascii")


def repeated_ascii_pattern(line_number: int, length: int) -> str:
    """Return a deterministic printable ASCII pattern of exactly length chars."""
    alphabet = (
        string.ascii_uppercase
        + string.ascii_lowercase
        + string.digits
        + " .,:;!?/+*-_=#@[](){}<>"
    )
    offset = line_number % len(alphabet)
    rotated = alphabet[offset:] + alphabet[:offset]
    repeats = (length // len(rotated)) + 1
    return (rotated * repeats)[:length]


def ascii_start_marker(payload_bytes: int, nonce: ProbeNonce | None) -> bytes:
    """Return the ASCII probe begin marker."""
    if nonce is None:
        return (
            f"<<<SERIAL_PROBE_BEGIN VERSION=1 TARGET_BYTES={payload_bytes:08d}>>>\r\n"
        ).encode("ascii")
    return (
        f"<<<SERIAL_PROBE_BEGIN VERSION=2 MODE=ASCII "
        f"TARGET_BYTES={payload_bytes:08d}{nonce_field_text(nonce)}>>>\r\n"
    ).encode("ascii")


def ascii_end_marker(
    line_count: int,
    body_hash: int,
    nonce: ProbeNonce | None,
) -> bytes:
    """Return the ASCII probe end marker."""
    if nonce is None:
        return (
            f"<<<SERIAL_PROBE_END LINES={line_count:06d} HASH={body_hash:08X}>>>\r\n"
        ).encode("ascii")
    return (
        f"<<<SERIAL_PROBE_END LINES={line_count:06d} HASH={body_hash:08X}"
        f"{nonce_field_text(nonce)}>>>\r\n"
    ).encode("ascii")


def minimum_payload_size(nonce: ProbeNonce | None = None) -> int:
    """Return the smallest payload size this generator can represent."""
    size_nonce = representative_nonce() if nonce is None else nonce
    start = ascii_start_marker(0, size_nonce)
    end = ascii_end_marker(0, 0, size_nonce)
    smallest_line = make_probe_line(1, "PAD", "", size_nonce)
    return len(start) + len(end) + len(smallest_line)


def generate_payload(
    payload_bytes: int,
    nonce: ProbeNonce | None = None,
) -> ProbePayload:
    """Generate an ASCII-only probe payload of exactly `payload_bytes` bytes.

    The payload contains fixed-width start/end markers and checksum-bearing line
    records. Supplying a nonce makes the payload candidate/trial specific.

    Args:
        payload_bytes: Exact byte count to generate, including markers.
        nonce: Optional run/candidate/trial identity embedded in markers and
            every checksum-covered probe line.

    Returns:
        A `ProbePayload` whose `data` length equals `payload_bytes`.

    Raises:
        ValueError: If the requested size cannot hold the required structure.
        AssertionError: If the generator's internal sizing invariants fail.

    Notes:
        All payload bytes are printable ASCII plus CR/LF so normal serial text
        paths can be evaluated before the raw high-bit challenge is attempted.
    """
    min_size = minimum_payload_size(nonce)
    if payload_bytes < min_size:
        raise ValueError(f"payload-bytes must be at least {min_size}")

    start = ascii_start_marker(payload_bytes, nonce)
    end_len = len(ascii_end_marker(0, 0, nonce))
    lines: list[bytes] = []
    line_number = 1
    current_len = len(start)

    sample_full_line = make_probe_line(
        1,
        "DATA",
        repeated_ascii_pattern(1, 96),
        nonce,
    )
    min_pad_len = len(make_probe_line(1, "PAD", "", nonce))

    while True:
        candidate = make_probe_line(
            line_number,
            "DATA",
            repeated_ascii_pattern(line_number, 96),
            nonce,
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
        nonce_text = nonce_field_text(nonce)
        pad_prefix_len = len(
            f"LINE {line_number:06d}{nonce_text} BLOCK={block_number:02d} "
            "TYPE=PAD DATA=".encode("ascii")
        )
        pad_suffix_len = len(b" HASH=00000000\r\n")
        pad_data_len = remaining_for_pad - pad_prefix_len - pad_suffix_len
        if pad_data_len < 0:
            # WHY: Backing out one DATA line gives the final PAD line enough room
            # to preserve the exact requested byte count without shortening fixed
            # markers or weakening per-line checksums.
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
        lines.append(make_probe_line(line_number, "PAD", pad_data, nonce))
        current_len += len(lines[-1])
        line_number += 1
        break

    if not lines:
        # WATCHOUT: Normal control flow emits at least one line; this protects the
        # structural invariant if future size logic changes.
        lines.append(sample_full_line)
        current_len += len(sample_full_line)
        line_number += 1

    line_count = len(lines)
    body = start + b"".join(lines)
    body_hash = fnv1a32(body)
    end = ascii_end_marker(line_count, body_hash, nonce)
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
        payload_mode=PAYLOAD_MODE_ASCII,
        nonce=nonce,
    )


def expected_nonce_from_payload(expected: bytes) -> ProbeNonce | None:
    """Extract nonce fields from an expected payload marker if present."""
    head = expected[: min(len(expected), 512)].decode("ascii", "ignore")
    run = re.search(r"\bRUN=([^ >\r\n]+)", head)
    cand = re.search(r"\bCAND=([^ >\r\n]+)", head)
    trial = re.search(r"\bTRIAL=([^ >\r\n]+)", head)
    note = re.search(r"\bNOTE=([^ >\r\n]+)", head)
    if not (run and cand and trial):
        return None
    return ProbeNonce(
        run_id=run.group(1),
        candidate_id=cand.group(1),
        trial_id=trial.group(1),
        switch_note_hash=note.group(1) if note else None,
    )


def line_nonce_from_left(left: bytes) -> ProbeNonce | None:
    """Extract nonce fields from the checksum-covered part of a probe line."""
    text = left.decode("ascii", "ignore")
    run = re.search(r"\bRUN=([^ \r\n]+)", text)
    cand = re.search(r"\bCAND=([^ \r\n]+)", text)
    trial = re.search(r"\bTRIAL=([^ \r\n]+)", text)
    note = re.search(r"\bNOTE=([^ \r\n]+)", text)
    if not (run and cand and trial):
        return None
    return ProbeNonce(
        run_id=run.group(1),
        candidate_id=cand.group(1),
        trial_id=trial.group(1),
        switch_note_hash=note.group(1) if note else None,
    )


def nonce_matches(observed: ProbeNonce | None, expected: ProbeNonce | None) -> bool:
    """Return True if an observed line nonce belongs to the expected trial."""
    if expected is None:
        return observed is None
    if observed is None:
        return False
    return (
        observed.run_id == expected.run_id
        and observed.candidate_id == expected.candidate_id
        and observed.trial_id == expected.trial_id
        and observed.switch_note_hash == expected.switch_note_hash
    )


def parse_probe_lines(data: bytes) -> list[ParsedProbeLine]:
    """Return structurally valid checksum-protected probe lines found in bytes."""
    valid: list[ParsedProbeLine] = []
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
            valid.append(
                ParsedProbeLine(
                    line_number=line_number,
                    checksum=reported_hash,
                    nonce=line_nonce_from_left(left),
                    raw_left=left,
                )
            )
    return valid


def parse_valid_probe_lines(data: bytes) -> dict[int, int]:
    """Return valid line numbers and checksums found in probe-like bytes."""
    return {line.line_number: line.checksum for line in parse_probe_lines(data)}


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


def high_bit_challenge_pattern() -> bytes:
    """Return high-bit bytes that avoid XON/XOFF when masked to 7 bits."""
    return bytes(
        byte
        for byte in range(0x80, 0x100)
        if (byte & 0x7F) not in {0x11, 0x13}
    )


def eight_bit_header(payload_bytes: int, nonce: ProbeNonce | None) -> bytes:
    """Return the ASCII header for an eight-bit challenge payload."""
    return (
        f"<<<SERIAL_PROBE_8BIT_BEGIN VERSION=1 TARGET_BYTES={payload_bytes:08d}"
        f"{nonce_field_text(nonce)}>>>\r\n"
    ).encode("ascii")


def eight_bit_footer(
    high_bit_count: int,
    high_bit_hash: int,
    nonce: ProbeNonce | None,
) -> bytes:
    """Return the ASCII footer for an eight-bit challenge payload."""
    return (
        f"\r\n<<<SERIAL_PROBE_8BIT_END HIGH_BYTES={high_bit_count:06d} "
        f"HASH={high_bit_hash:08X}{nonce_field_text(nonce)}>>>\r\n"
    ).encode("ascii")


def seven_bit_masked_match_ratio(expected: bytes, received: bytes) -> float:
    """Return match ratio after clearing the received high bit."""
    compared = min(len(expected), len(received))
    if compared <= 0:
        return 0.0
    matches = sum(
        1
        for expected_byte, received_byte in zip(expected[:compared], received[:compared])
        if expected_byte == (received_byte & 0x7F)
    )
    return matches / compared


def minimum_eight_bit_payload_size(nonce: ProbeNonce | None = None) -> int:
    """Return the smallest useful eight-bit challenge payload size."""
    size_nonce = representative_nonce() if nonce is None else nonce
    return len(eight_bit_header(0, size_nonce)) + len(eight_bit_footer(1, 0, size_nonce)) + 1


def generate_eight_bit_payload(
    payload_bytes: int,
    nonce: ProbeNonce | None = None,
) -> ProbePayload:
    """Generate an eight-bit challenge payload with ASCII nonce metadata.

    Args:
        payload_bytes: Exact byte count to generate.
        nonce: Optional run/candidate/trial identity in ASCII header/footer text.

    Returns:
        A `ProbePayload` with an ASCII header, raw high-bit block, and ASCII
        footer describing the high-bit count and hash.

    Raises:
        ValueError: If the requested size cannot hold the header, one high-bit
            byte, and footer.
        AssertionError: If internal byte-count accounting fails.
    """
    min_size = minimum_eight_bit_payload_size(nonce)
    if payload_bytes < min_size:
        raise ValueError(f"eight-bit payload must be at least {min_size} bytes")

    header = eight_bit_header(payload_bytes, nonce)
    footer_len = len(eight_bit_footer(0, 0, nonce))
    high_len = payload_bytes - len(header) - footer_len
    if high_len <= 0:
        raise ValueError(f"eight-bit payload must be at least {min_size} bytes")
    pattern = high_bit_challenge_pattern()
    high_block = (pattern * ((high_len // len(pattern)) + 1))[:high_len]
    high_hash = fnv1a32(high_block)
    footer = eight_bit_footer(len(high_block), high_hash, nonce)
    payload = header + high_block + footer
    if len(payload) != payload_bytes:
        raise AssertionError(
            f"eight-bit payload produced {len(payload)} bytes, expected {payload_bytes}"
        )
    return ProbePayload(
        data=payload,
        line_count=0,
        byte_count=len(payload),
        body_hash=f"{high_hash:08X}",
        payload_mode=PAYLOAD_MODE_EIGHT_BIT,
        nonce=nonce,
        high_bit_block=high_block,
    )


def generate_probe_payload(
    payload_bytes: int,
    mode: str = PAYLOAD_MODE_ASCII,
    nonce: ProbeNonce | None = None,
) -> ProbePayload:
    """Generate one payload for the requested probe mode."""
    if mode == PAYLOAD_MODE_EIGHT_BIT:
        return generate_eight_bit_payload(payload_bytes, nonce)
    if mode == PAYLOAD_MODE_ASCII:
        return generate_payload(payload_bytes, nonce)
    if mode == PAYLOAD_MODE_PHASE0:
        return generate_phase0_payload(payload_bytes, nonce)
    raise ValueError(f"unknown payload mode {mode!r}")


def eight_bit_payload_slices(expected: bytes) -> tuple[int, int] | None:
    """Return the high-bit block slice boundaries for an expected challenge."""
    if b"<<<SERIAL_PROBE_8BIT_BEGIN" not in expected:
        return None
    header_end = expected.find(b"\r\n")
    footer_start = expected.rfind(b"\r\n<<<SERIAL_PROBE_8BIT_END")
    if header_end < 0 or footer_start < 0 or footer_start <= header_end:
        return None
    return header_end + 2, footer_start


def exact_prefix_byte_count(expected: bytes, received: bytes) -> int:
    """Return how many received bytes match the expected stream from byte zero."""
    count = 0
    for expected_byte, received_byte in zip(expected, received):
        if expected_byte != received_byte:
            break
        count += 1
    return count


def score_eight_bit_received(expected: bytes, received: bytes) -> ScoreResult:
    """Score received bytes for a raw eight-bit challenge payload.

    Args:
        expected: Exact payload bytes sent by the test.
        received: Bytes read back from the output side.

    Returns:
        Score and metrics for exact high-bit preservation, stripped high bits,
        header/footer presence, byte length, and stale-nonce detection.

    Notes:
        A path that strips high bits is capped below a recommendable score even
        when ASCII framing survives. That keeps printable transfer evidence
        separate from proof of an 8-bit-clean data path.
    """
    expected_len = len(expected)
    received_len = len(received)
    missing = max(expected_len - received_len, 0)
    extra = max(received_len - expected_len, 0)
    length_ratio = min(expected_len, received_len) / max(expected_len, received_len, 1)
    exact_ratio = exact_byte_match_ratio(expected, received)
    exact_prefix = exact_prefix_byte_count(expected, received)
    ascii_ratio = printable_ascii_ratio(received)
    start_marker = b"<<<SERIAL_PROBE_8BIT_BEGIN" in received
    end_marker = b"<<<SERIAL_PROBE_8BIT_END" in received
    expected_nonce = expected_nonce_from_payload(expected)
    wrong_nonce = False
    if expected_nonce is not None:
        received_nonce = expected_nonce_from_payload(received)
        wrong_nonce = received_nonce is not None and not nonce_matches(received_nonce, expected_nonce)

    high_sent = 0
    high_received = 0
    high_exact_ratio = 0.0
    high_stripped = 0
    masked_ratio = seven_bit_masked_match_ratio(expected, received)
    block_slice = eight_bit_payload_slices(expected)
    if block_slice is not None:
        start, stop = block_slice
        expected_high = expected[start:stop]
        received_high = received[start : min(start + len(expected_high), len(received))]
        high_sent = sum(1 for byte in expected_high if byte >= 0x80)
        high_received = sum(1 for byte in received_high if byte >= 0x80)
        high_matches = sum(
            1
            for expected_byte, received_byte in zip(expected_high, received_high)
            if expected_byte == received_byte
        )
        high_exact_ratio = high_matches / max(len(expected_high), 1)
        high_stripped = sum(
            1
            for expected_byte, received_byte in zip(expected_high, received_high)
            if expected_byte >= 0x80 and received_byte == (expected_byte & 0x7F)
        )

    metrics = ScoreMetrics(
        exact_byte_match_ratio=exact_ratio,
        line_integrity_ratio=1.0 if start_marker and end_marker else 0.0,
        missing_bytes=missing,
        extra_bytes=extra,
        printable_ascii_ratio=ascii_ratio,
        length_ratio=length_ratio,
        start_marker_present=start_marker,
        end_marker_present=end_marker,
        exact_prefix_bytes=exact_prefix,
        high_bit_bytes_sent=high_sent,
        high_bit_bytes_received=high_received,
        high_bit_exact_ratio=high_exact_ratio,
        high_bit_stripped_count=high_stripped,
        seven_bit_masked_match_ratio=masked_ratio,
    )

    if not received:
        return ScoreResult(0.0, metrics, "no-data", ("NO DATA",))
    if wrong_nonce:
        return ScoreResult(0.0, metrics, "wrong-nonce", ("STALE / WRONG NONCE",))
    if expected == received and high_sent > 0 and high_exact_ratio >= 1.0:
        return ScoreResult(100.0, metrics, "eight-bit-clean", ("8-BIT CLEAN",))
    if high_sent > 0 and high_stripped >= max(1, int(high_sent * 0.8)):
        confidence = 35.0 * length_ratio + 25.0 * float(start_marker and end_marker)
        return ScoreResult(
            max(0.0, min(60.0, confidence)),
            metrics,
            "eight-bit-masked",
            ("ASCII BYTE TRANSFER", "8-BIT NOT CLEAN"),
        )
    confidence = (
        40.0 * exact_ratio
        + 25.0 * high_exact_ratio
        + 20.0 * length_ratio
        + 15.0 * ((int(start_marker) + int(end_marker)) / 2)
    )
    classification = "eight-bit-not-clean" if high_exact_ratio < 1.0 else "partial"
    evidence = ("8-BIT NOT CLEAN",) if classification == "eight-bit-not-clean" else ()
    return ScoreResult(max(0.0, min(99.0, confidence)), metrics, classification, evidence)


def score_received(expected: bytes, received: bytes) -> ScoreResult:
    """Score received bytes against the expected probe payload.

    Args:
        expected: Exact payload bytes sent into the buffer input side.
        received: Bytes read from the buffer output side.

    Returns:
        Bounded score, metrics, classification, and evidence labels.

    Notes:
        Byte ratios are position-aligned from offset zero. Probe-line integrity
        is checksum-based and keyed by line number. Nonce mismatches are treated
        as stale output, not as partial credit for the current candidate.
    """
    if b"<<<SERIAL_PROBE_8BIT_BEGIN" in expected:
        return score_eight_bit_received(expected, received)

    expected_len = len(expected)
    received_len = len(received)
    missing = max(expected_len - received_len, 0)
    extra = max(received_len - expected_len, 0)
    length_ratio = min(expected_len, received_len) / max(expected_len, received_len, 1)
    exact_ratio = exact_byte_match_ratio(expected, received)
    exact_prefix = exact_prefix_byte_count(expected, received)
    expected_lines = parse_valid_probe_lines(expected)
    expected_nonce = expected_nonce_from_payload(expected)
    received_parsed_lines = parse_probe_lines(received)
    received_lines = {
        line.line_number: line.checksum for line in received_parsed_lines
    }
    current_nonce_lines = sum(
        1 for line in received_parsed_lines if nonce_matches(line.nonce, expected_nonce)
    )
    wrong_nonce_lines = sum(
        1
        for line in received_parsed_lines
        if expected_nonce is not None
        and not nonce_matches(line.nonce, expected_nonce)
    )
    matching_lines = sum(
        1
        for line_number, checksum in received_lines.items()
        if expected_lines.get(line_number) == checksum
    )
    line_ratio = matching_lines / max(len(expected_lines), 1)
    ascii_ratio = printable_ascii_ratio(received)
    high_sent = sum(1 for byte in expected if byte >= 0x80)
    high_received = sum(1 for byte in received if byte >= 0x80)
    masked_ratio = seven_bit_masked_match_ratio(expected, received)
    start_marker = b"<<<SERIAL_PROBE_BEGIN" in received
    end_marker = b"<<<SERIAL_PROBE_END" in received
    marker_ratio = (int(start_marker) + int(end_marker)) / 2

    if not received:
        confidence = 0.0
        classification = "no-data"
        evidence = ("NO DATA",)
    elif wrong_nonce_lines > 0 and current_nonce_lines == 0:
        confidence = 0.0
        classification = "wrong-nonce"
        evidence = ("STALE / WRONG NONCE",)
    else:
        confidence = (
            55.0 * exact_ratio
            + 25.0 * line_ratio
            + 10.0 * ascii_ratio
            + 5.0 * length_ratio
            + 5.0 * marker_ratio
        )
        classification = "ascii-transfer" if line_ratio > 0.0 else "partial"
        evidence = ("ASCII BYTE TRANSFER",) if line_ratio > 0.0 else ()
        if wrong_nonce_lines > 0:
            confidence = min(confidence, 49.0)
            classification = "mixed-nonce"
            evidence = ("STALE / WRONG NONCE",)
        if (
            expected_len == received_len
            and exact_ratio == 1.0
            and line_ratio == 1.0
            and start_marker
            and end_marker
            and wrong_nonce_lines == 0
        ):
            confidence = 100.0
            classification = "exact"
            evidence = ("BAUD ALIVE", "ASCII BYTE TRANSFER")

    metrics = ScoreMetrics(
        exact_byte_match_ratio=exact_ratio,
        line_integrity_ratio=line_ratio,
        missing_bytes=missing,
        extra_bytes=extra,
        printable_ascii_ratio=ascii_ratio,
        length_ratio=length_ratio,
        start_marker_present=start_marker,
        end_marker_present=end_marker,
        exact_prefix_bytes=exact_prefix,
        valid_probe_line_count=len(received_parsed_lines),
        current_nonce_line_count=current_nonce_lines,
        wrong_nonce_line_count=wrong_nonce_lines,
        high_bit_bytes_sent=high_sent,
        high_bit_bytes_received=high_received,
        seven_bit_masked_match_ratio=masked_ratio,
    )
    return ScoreResult(
        score=max(0.0, min(100.0, confidence)),
        metrics=metrics,
        classification=classification,
        evidence=evidence,
    )


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


def normalize_port_name(port: str) -> str:
    """Normalize a Windows COM port name for same-port checks."""
    normalized = port.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized


def validate_port_name(port: str) -> None:
    """Raise if a port name is not a plain Windows COM port."""
    normalized = normalize_port_name(port)
    if not normalized:
        raise ValueError("PORT NAME CANNOT BE BLANK")
    match = re.fullmatch(r"COM([1-9][0-9]*)", normalized)
    if not match:
        raise ValueError("ENTER A WINDOWS COM PORT SUCH AS COM1 OR COM5")
    port_number = int(match.group(1))
    if port_number > 256:
        raise ValueError("COM PORT NUMBER MUST BE 1 THROUGH 256")


def ensure_distinct_ports(in_port: str, out_port: str) -> None:
    """Raise if the input and output ports resolve to the same port name."""
    validate_port_name(in_port)
    validate_port_name(out_port)
    if normalize_port_name(in_port) == normalize_port_name(out_port):
        raise ValueError("INPUT AND OUTPUT PORTS MUST NOT BE THE SAME")


def import_pyserial() -> Any:
    """Import pyserial or exit with an explicit installation instruction."""
    try:
        import serial  # type: ignore[import-not-found]

        return serial
    except ImportError:
        print("INSTALL PYSERIAL WITH: PYTHON -M PIP INSTALL PYSERIAL")
        raise SystemExit(2)


# noinspection SpellCheckingInspection
def serial_constants(serial_module: Any, settings: SerialSettings) -> dict[str, Any]:
    """Map program-level settings to pyserial constructor arguments.

    Args:
        serial_module: Imported pyserial module or a test double with matching
            constants.
        settings: Concrete serial setting to convert.

    Returns:
        Keyword arguments accepted by `serial.Serial`.

    Raises:
        KeyError: If a setting contains a data-bit, parity, or stop-bit value
        outside this program's configured candidate space.
    """
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


def transmit_side_settings(
    settings: SerialSettings | DualSerialSettings,
) -> SerialSettings:
    """Return the settings used to write into the buffer input side."""
    if isinstance(settings, DualSerialSettings):
        return settings.input_settings
    return settings


def receive_side_settings(
    settings: SerialSettings | DualSerialSettings,
) -> SerialSettings:
    """Return the settings used to read from the buffer output side."""
    if isinstance(settings, DualSerialSettings):
        return settings.output_settings
    return settings


def serial_frame_bits(settings: SerialSettings) -> float:
    """Estimate serial frame width in bits per byte for one concrete setting."""
    parity_bits = 0 if settings.parity == "none" else 1
    return 1 + settings.data_bits + parity_bits + settings.stop_bits


def estimated_frame_bits(settings: SerialSettings | DualSerialSettings) -> float:
    """Estimate transmit-side serial frame width in bits per byte."""
    return serial_frame_bits(transmit_side_settings(settings))


def estimated_buffer_drain_seconds(
    settings: SerialSettings | DualSerialSettings,
    buffer_bytes: int = BUFFER_PURGE_CAPACITY_BYTES,
    safety_factor: float = 1.2,
) -> float:
    """Estimate full FIFO drain time for a byte count at the output frame.

    Args:
        settings: Same-side or dual-side settings. Dual settings use the output
            side because the drain is observed on the receive port.
        buffer_bytes: Number of buffered bytes to drain.
        safety_factor: Multiplicative margin applied after the physical line
            estimate.

    Returns:
        Estimated seconds needed to drain the requested byte count.
    """
    receive_settings = receive_side_settings(settings)
    seconds = (
        buffer_bytes * serial_frame_bits(receive_settings)
    ) / max(receive_settings.baud, 1)
    return seconds * max(safety_factor, 0.0)


def write_chunk_size(settings: SerialSettings | DualSerialSettings) -> int:
    """Choose a write chunk size that behaves well at very low baud rates."""
    transmit_settings = transmit_side_settings(settings)
    bytes_per_second = max(
        1.0,
        transmit_settings.baud / serial_frame_bits(transmit_settings),
    )
    quarter_second = int(bytes_per_second * 0.25)
    return max(8, min(2048, quarter_second))


def open_serial_port(
    serial_module: Any,
    port: str,
    settings: SerialSettings,
    read_timeout: float,
) -> Any:
    """Open a serial port for one candidate setting.

    Args:
        serial_module: Imported pyserial module or test double.
        port: Windows COM port name.
        settings: Concrete serial settings for the opened side.
        read_timeout: Candidate read timeout; clamped for the pyserial polling
            timeout and used as a lower bound for write timeout.

    Returns:
        An open pyserial-compatible serial object.

    Raises:
        SerialConfigurationError: If pyserial rejects the requested setting
        with `ValueError`.
        OSError: If the port cannot be opened by the serial driver.
    """
    constants = serial_constants(serial_module, settings)
    try:
        return serial_module.Serial(
            port=port,
            timeout=min(0.05, max(read_timeout, 0.001)),
            write_timeout=max(3.0, read_timeout),
            inter_byte_timeout=0.05,
            **constants,
        )
    except ValueError as exc:
        raise SerialConfigurationError(
            f"{port} {settings.label()}: {exc}"
        ) from exc


def reset_serial_buffers(serial_port: Any) -> None:
    """Reset pyserial input and output buffers on an open port.

    Raises:
        OSError: If the serial driver rejects either reset operation.
        RuntimeError: If a pyserial-compatible test double raises one.
    """
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()


def modem_line_snapshot(serial_port: Any) -> dict[str, bool]:
    """Return a best-effort modem-control and status snapshot.

    Missing attributes and platform I/O failures are recorded as `False` so
    flow-validation reports can still be produced on adapters that do not expose
    every modem line.
    """
    snapshot: dict[str, bool] = {}
    for name in ("cts", "dsr", "cd", "ri", "rts", "dtr"):
        try:
            snapshot[name] = bool(getattr(serial_port, name))
        except BEST_EFFORT_PLATFORM_ERRORS:
            snapshot[name] = False
    return snapshot


def modem_line_snapshot_label(snapshot: dict[str, bool]) -> str:
    """Return compact modem-line state text for logs and progress output."""
    return " ".join(
        f"{name.upper()}={1 if snapshot.get(name, False) else 0}"
        for name in ("cts", "dsr", "cd", "ri", "rts", "dtr")
    )


def modem_line_observation(
    label: str,
    snapshot: dict[str, bool],
) -> tuple[str, dict[str, bool]]:
    """Return a report-safe modem-line observation."""
    return label, dict(snapshot)


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
    """Enable green ANSI terminal styling when the current console supports it.

    The function has process-wide stdout side effects: it prints the ANSI start
    sequence and registers an `atexit` reset when color is enabled. `NO_COLOR`
    disables this path.
    """
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
            get_std_handle = getattr(kernel32, "GetStdHandle")
            get_console_mode = getattr(kernel32, "GetConsoleMode")
            set_console_mode = getattr(kernel32, "SetConsoleMode")
            handle = get_std_handle(STD_OUTPUT_HANDLE)
            mode = ctypes.c_uint32()
            if get_console_mode(handle, ctypes.byref(mode)):
                set_console_mode(
                    handle,
                    mode.value | ENABLE_VIRTUAL_TERMINAL_PROCESSING,
                )
        except BEST_EFFORT_PLATFORM_ERRORS:
            pass
    print(ANSI_GREEN, end="")
    atexit.register(lambda: print(ANSI_RESET, end=""))


def print_banner() -> None:
    """Print the terminal-style program banner."""
    for line in banner_lines():
        print(line)


def border_line(width: int = SCREEN_WIDTH) -> str:
    """Return a line made of asterisks."""
    return "*" * width


def bordered_text(text: str, width: int = SCREEN_WIDTH) -> str:
    """Return one centered text line inside asterisks."""
    inner_width = max(width - 4, 1)
    cleaned = text[:inner_width]
    return f"* {cleaned.center(inner_width)} *"


def fit_terminal_line(text: object, width: int = TERMINAL_COLUMNS) -> str:
    """Return one printable line that fits the fixed terminal width."""
    cleaned = str(text).replace("\t", " ")
    if len(cleaned) <= width:
        return cleaned
    if width <= 3:
        return cleaned[:width]
    return cleaned[: width - 3] + "..."


def terminal_text(value: object) -> str:
    """Return operator-facing text in the uppercase terminal style."""
    return str(value).replace("\t", " ").upper()


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
    pause_at_end: bool = True,
    return_label: str = "RETURN",
) -> None:
    """Print lines with a simple 80x25-friendly page pause.

    Args:
        lines: Lines to print.
        page_lines: Number of body lines before prompting. Non-positive values
            print the whole sequence without mid-page pauses.
        pause_at_end: Whether to prompt after the last printed page.
        return_label: Prompt text used by the final pause.

    Notes:
        `EOFError` is treated as non-interactive output and prints the remaining
        lines without asking for more input.
    """
    all_lines = list(lines)
    if page_lines <= 0:
        page_lines = len(all_lines)
    for index, line in enumerate(all_lines, start=1):
        print(line)
        if index >= len(all_lines) or index % page_lines != 0:
            continue
        while True:
            try:
                choice = read_operator_input("PRESS ENTER FOR MORE, Q TO STOP: ")
            except EOFError:
                print()
                for rest in all_lines[index:]:
                    print(rest)
                return
            choice = choice.lstrip("\ufeff").strip().lower()
            if choice == "":
                break
            if choice in {"q", "quit", "0"}:
                return
            print("PRESS ENTER OR Q.")
        print()
    if pause_at_end and all_lines:
        while True:
            try:
                choice = read_operator_input(f"PRESS ENTER TO {return_label}: ")
            except EOFError:
                return
            choice = choice.lstrip("\ufeff").strip().lower()
            if choice in {"", "q", "quit", "0", "m", "menu"}:
                return
            print("PRESS ENTER.")


def read_operator_input(prompt: str) -> str:
    """Read terminal input, tolerating occasional bad console bytes.

    A `UnicodeDecodeError` returns an empty string so prompts fall back to their
    documented defaults instead of aborting the interactive menu.
    """
    try:
        return input(prompt)
    except UnicodeDecodeError:
        return ""


def prompt_loop_active() -> bool:
    """Return whether an interactive prompt should keep accepting input."""
    return not sys.is_finalizing()


def wrapped_value_lines(
    prefix: str,
    value: object,
    width: int = TERMINAL_COLUMNS,
) -> list[str]:
    """Return prefixed lines wrapped to the terminal width."""
    available = max(20, width - len(prefix))
    wrapped = textwrap.wrap(
        terminal_text(value),
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


def byte_size_label(byte_count: int) -> str:
    """Return a compact byte-count label for purge and report screens."""
    if byte_count > 0 and byte_count % KIB_BYTES == 0:
        return f"{byte_count // KIB_BYTES}K"
    return f"{byte_count} BYTES"


def byte_size_detail(byte_count: int) -> str:
    """Return a byte-count label with exact bytes when a K label is used."""
    label = byte_size_label(byte_count)
    if label.endswith("K"):
        return f"{label} ({byte_count} BYTES)"
    return label


def format_finish_clock(remaining_seconds: float, now: dt.datetime | None = None) -> str:
    """Return an approximate local clock time for scan completion."""
    current = now if now is not None else dt.datetime.now().astimezone()
    finish = current + dt.timedelta(seconds=max(0.0, remaining_seconds))
    if finish.date() == current.date():
        return finish.strftime("%H:%M:%S")
    return finish.strftime("%Y-%m-%d %H:%M:%S")


def result_indicator(score: float, status: str, error: str | None = None) -> str:
    """Return a short human-readable success indicator for console output."""
    if status in STALE_STATUSES:
        return "STALE"
    if status == "eight-bit-not-clean":
        return "FAIL"
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


def estimated_transmit_seconds(
    settings: SerialSettings | DualSerialSettings,
    byte_count: int,
) -> float:
    """Estimate physical transmit time for byte_count at candidate settings."""
    transmit_settings = transmit_side_settings(settings)
    return (
        byte_count * serial_frame_bits(transmit_settings)
    ) / max(transmit_settings.baud, 1)


def estimated_receive_seconds(
    settings: SerialSettings | DualSerialSettings,
    byte_count: int,
) -> float:
    """Estimate output-side receive time for byte_count at candidate settings."""
    receive_settings = receive_side_settings(settings)
    return (
        byte_count * serial_frame_bits(receive_settings)
    ) / max(receive_settings.baud, 1)


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
    """Return timing with unassigned elapsed time folded into `other_sec`."""
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
    """Return phase-1 timing after turbo and low-baud policies.

    Args:
        options: Active scan options.
        settings: Candidate output-side timing context.
        payload_bytes: Payload size used to estimate physical transmit/receive
            duration for adaptive cleanup windows.

    Returns:
        Effective read, settle, pre-drain, and completion-quiet values.

    Notes:
        Low-baud cleanup can expand pre-drain windows even when turbo discovery
        is off. Phase 0 has a separate low-baud path because its compact payload
        otherwise looks fast enough to under-wait at 300/1200 baud.
    """
    if not options.turbo_discovery_enabled:
        timing = EffectiveTiming(
            read_timeout=options.read_timeout,
            settle_ms=options.settle_ms,
            pre_drain_quiet=options.pre_drain_quiet,
            pre_drain_timeout=options.pre_drain_timeout,
            completion_quiet=min(options.read_timeout, DEFAULT_COMPLETION_QUIET),
        )
        if phase0_low_baud_timing_applies(options, settings, payload_bytes):
            return phase0_low_baud_timing(timing, settings, payload_bytes)
        return low_baud_pre_drain_timing(timing, settings, payload_bytes)

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


def low_baud_pre_drain_values(
    settings: SerialSettings | DualSerialSettings,
    payload_bytes: int,
    pre_drain_timeout: float,
    pre_drain_quiet: float,
) -> tuple[float, float]:
    """Return pre-drain timeout and quiet values sized for slow output bauds.

    The receive side controls stale-output cleanup because old data is observed
    on the output port, even when the input side uses a different baud.
    """
    receive_settings = receive_side_settings(settings)
    if receive_settings.baud >= LOW_BAUD_THRESHOLD:
        return pre_drain_timeout, pre_drain_quiet

    receive_seconds = estimated_receive_seconds(settings, payload_bytes)
    quiet_floor = min(
        LOW_BAUD_PRE_DRAIN_QUIET_MAX,
        max(DEFAULT_PRE_DRAIN_QUIET, receive_seconds * LOW_BAUD_PRE_DRAIN_QUIET_RATIO),
    )
    pre_drain_quiet = max(pre_drain_quiet, quiet_floor)
    pre_drain_timeout = max(
        pre_drain_timeout,
        receive_seconds * LOW_BAUD_PRE_DRAIN_MULTIPLIER + pre_drain_quiet,
    )
    return pre_drain_timeout, pre_drain_quiet


def low_baud_pre_drain_timing(
    timing: EffectiveTiming,
    settings: SerialSettings | DualSerialSettings,
    payload_bytes: int,
) -> EffectiveTiming:
    """Return timing with slow-output cleanup room while keeping read policy."""
    pre_drain_timeout, pre_drain_quiet = low_baud_pre_drain_values(
        settings,
        payload_bytes,
        timing.pre_drain_timeout,
        timing.pre_drain_quiet,
    )
    if (
        pre_drain_timeout == timing.pre_drain_timeout
        and pre_drain_quiet == timing.pre_drain_quiet
    ):
        return timing
    return EffectiveTiming(
        read_timeout=timing.read_timeout,
        settle_ms=timing.settle_ms,
        pre_drain_quiet=pre_drain_quiet,
        pre_drain_timeout=pre_drain_timeout,
        completion_quiet=timing.completion_quiet,
    )


def phase0_low_baud_timing_applies(
    options: ScanOptions,
    settings: SerialSettings,
    payload_bytes: int,
) -> bool:
    """Return True when compact Phase 0 needs low-baud cleanup timing."""
    return (
        settings.baud < PHASE0_LOW_BAUD_THRESHOLD
        and payload_bytes == phase0_payload_bytes()
        and options.bursts == PHASE0_BURSTS
        and options.settle_ms == PHASE0_SETTLE_MS
        and abs(options.read_timeout - PHASE0_READ_TIMEOUT) < 0.001
        and abs(options.pre_drain_quiet - PHASE0_PRE_DRAIN_QUIET) < 0.001
    )


def phase0_low_baud_timing(
    timing: EffectiveTiming,
    settings: SerialSettings,
    payload_bytes: int,
) -> EffectiveTiming:
    """Return Phase 0 timing with enough room for slow output-side draining."""
    receive_seconds = estimated_receive_seconds(settings, payload_bytes)
    read_timeout = max(
        timing.read_timeout,
        min(
            PHASE0_LOW_BAUD_READ_TIMEOUT_MAX,
            receive_seconds * PHASE0_LOW_BAUD_READ_MULTIPLIER,
        ),
    )
    pre_drain_quiet = max(
        timing.pre_drain_quiet,
        min(
            PHASE0_LOW_BAUD_PRE_DRAIN_QUIET_MAX,
            receive_seconds / 2.0,
        ),
    )
    pre_drain_timeout = max(
        timing.pre_drain_timeout,
        receive_seconds * PHASE0_LOW_BAUD_PRE_DRAIN_MULTIPLIER + pre_drain_quiet,
    )
    return EffectiveTiming(
        read_timeout=read_timeout,
        settle_ms=timing.settle_ms,
        pre_drain_quiet=pre_drain_quiet,
        pre_drain_timeout=pre_drain_timeout,
        completion_quiet=timing.completion_quiet,
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
    """Return True when a received payload has enough structure to stop early.

    Notes:
        Completion is conservative: the received byte count must reach the
        expected count and include the proper end marker. Non-exact payloads are
        accepted only when checksum scoring still meets the top-match threshold.
    """
    end_marker = (
        b"<<<SERIAL_PROBE_8BIT_END"
        if b"<<<SERIAL_PROBE_8BIT_BEGIN" in expected
        else b"<<<SERIAL_PROBE_END"
    )
    if len(received) < len(expected):
        return False
    if end_marker not in received:
        return False
    if received == expected:
        return True
    score = score_received(expected, received)
    return (
        score.score >= TOP_MATCH_MIN_SCORE
        and score.metrics.end_marker_present
        and score.metrics.line_integrity_ratio >= 1.0
    )


def console_progress(message: str) -> None:
    """Print a timestamped live progress message."""
    prefix = f"{time.strftime('%H:%M:%S')} "
    width = max(20, TERMINAL_COLUMNS - len(prefix))
    lines = terminal_text(message).splitlines() or [""]
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
    print("  PASS GOOD PART FAIL STALE ERROR  RESULT INDICATOR.")
    print("  SCORE                 0-100 TRANSFER MATCH CONFIDENCE.")
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
    """Drain output bytes until the output side is quiet or a limit is reached.

    Args:
        out_serial: Open output-side serial object.
        quiet_seconds: Required silence window before the drain is considered
            successful.
        max_seconds: Hard elapsed-time limit. Non-positive values disable the
            drain and return a successful `disabled` result.
        max_bytes: Maximum bytes to consume before reporting stale output.
        progress_interval: Minimum interval between progress messages.
        progress: Optional progress callback.
        prefix: Prefix included in progress output.
        logger: Logger used for serial read errors.

    Returns:
        Drain result with byte count, elapsed time, quiet flag, reason, and
        optional error string.

    Notes:
        The function resets the output port input buffer before reading. That
        intentionally discards bytes already queued at the PC adapter so the
        quiet test reflects device output observed after the drain begins.
    """
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
    except SERIAL_IO_ERRORS as exc:
        logger.debug("pre-drain failed: %s", exc)
        elapsed = time.monotonic() - started
        return DrainResult(bytes_drained, elapsed, False, "error", str(exc))


def payload_for_trial(
    template: ProbePayload,
    settings: SerialSettings | DualSerialSettings,
    candidate_index: int,
    burst_index: int,
    run_id: str,
    switch_hash: str | None,
) -> ProbePayload:
    """Return a nonce-bearing payload for the current candidate and trial.

    Args:
        template: Payload shape and mode selected for the stage.
        settings: Candidate settings used only for candidate nonce construction.
        candidate_index: One-based candidate index within the current stage.
        burst_index: One-based burst index for repeated tests.
        run_id: Current run identifier.
        switch_hash: Optional hash of the operator switch note.

    Returns:
        `template` unchanged when it already has a nonce; otherwise a new
        payload with the same byte count and mode plus current nonce fields.
    """
    if template.nonce is not None:
        return template
    nonce = ProbeNonce(
        run_id=sanitize_nonce_value(run_id or make_run_id()),
        candidate_id=candidate_nonce_id(settings, candidate_index),
        trial_id=trial_nonce_id(burst_index),
        switch_note_hash=switch_hash,
    )
    return generate_probe_payload(template.byte_count, template.payload_mode, nonce)


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
    run_id: str = "",
    switch_hash: str | None = None,
) -> TrialResult:
    """Send one probe burst and score the received bytes.

    Args:
        in_serial: Open input-side serial object used for writes.
        out_serial: Open output-side serial object used for reads.
        settings: Same-side or dual-side candidate settings.
        payload: Probe payload template for this stage.
        burst_index: One-based burst number.
        burst_total: Total bursts planned for this candidate.
        candidate_index: One-based candidate number.
        candidate_total: Total candidates planned for this stage.
        read_timeout: Silence window used after writes complete.
        completion_quiet: Shorter silence window allowed after structural
            completion has been detected.
        settle_ms: Post-reset pause before optional stale-output drain.
        progress_interval: Minimum interval between progress messages.
        no_pre_drain: If true, skip stale-output cleanup before sending.
        pre_drain_timeout: Maximum stale-output drain time.
        pre_drain_quiet: Silence window required by stale-output drain.
        max_drain_bytes: Maximum stale bytes allowed before marking stale.
        logger: Logger for diagnostic details.
        progress: Optional progress callback.
        run_id: Current run id for nonce generation.
        switch_hash: Optional switch-note hash for nonce generation.

    Returns:
        A `TrialResult` containing transfer metrics, status, previews, timing,
        and any serial I/O error.

    Notes:
        The reader thread starts before the writer so data emitted immediately
        by a low-latency buffer is not missed. Stale output prevents the send
        entirely, because mixing old bytes with the current nonce would make the
        candidate score misleading.
    """
    started = time.monotonic()
    payload = payload_for_trial(
        template=payload,
        settings=settings,
        candidate_index=candidate_index,
        burst_index=burst_index,
        run_id=run_id,
        switch_hash=switch_hash,
    )
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
    transmit_settings = transmit_side_settings(settings)
    receive_settings = receive_side_settings(settings)
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
        # WHY: A noisy or still-draining buffer can produce checksum-valid data
        # from a previous trial. We fail the burst as stale before sending rather
        # than mix old output into the current candidate score.
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
                score_classification=empty_score.classification,
                evidence=empty_score.evidence,
                nonce_summary=payload.nonce.compact() if payload.nonce else "",
                payload_mode=payload.payload_mode,
            )

    def reader() -> None:
        """Read output bytes until the writer is done and the line is quiet."""
        nonlocal last_data_time
        while not stop_event.is_set():
            try:
                waiting = getattr(out_serial, "in_waiting", 0)
                read_size = min(max(int(waiting), 1), 4096)
                read_chunk = out_serial.read(read_size)
            except SERIAL_IO_ERRORS as reader_exc:
                reader_errors.append(str(reader_exc))
                stop_event.set()
                break

            reader_now = time.monotonic()
            if read_chunk:
                with received_lock:
                    received.extend(read_chunk)
                last_data_time = reader_now
                continue

            if writer_done.is_set():
                with received_lock:
                    received_snapshot = bytes(received)
                is_complete = receive_completion_detected(received_snapshot, expected)
                active_quiet_target = completion_quiet if is_complete else read_timeout
                if (reader_now - last_data_time) >= active_quiet_target:
                    stop_event.set()
                    break

    # WHY: Start the reader before writing so fast loopback-style hardware cannot
    # fill the adapter buffer before the read loop is active.
    reader_thread = threading.Thread(target=reader, name="serial-probe-reader", daemon=True)
    reader_thread.start()

    bytes_sent = 0
    write_error: str | None = None
    bytes_received_at_write_done: int
    write_started = time.monotonic()
    if progress:
        estimated = estimated_transmit_seconds(settings, len(expected))
        progress(
            f"{prefix}: SEND {len(expected)} BYTES ON {transmit_settings.label()} "
            f"(CHUNK={chunk_size}, ABOUT {format_duration(estimated)})"
        )
    try:
        next_progress_at = time.monotonic() + progress_interval
        while bytes_sent < len(expected):
            write_chunk = expected[bytes_sent : bytes_sent + chunk_size]
            written = in_serial.write(write_chunk)
            if written is None:
                written = len(write_chunk)
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
    except SERIAL_IO_ERRORS as exc:
        write_error = str(exc)
        logger.debug("burst %s write failed: %s", burst_index, write_error)
    finally:
        bytes_received_at_write_done = received_length(received, received_lock)
        last_data_time = time.monotonic()
        writer_done.set()
    write_elapsed = time.monotonic() - write_started

    if progress:
        progress(
            f"{prefix}: WRITE DONE, SENT={bytes_sent}; "
            f"WAIT {read_timeout:.2f}S QUIET ON {receive_settings.label()}"
        )

    receive_estimate = estimated_receive_seconds(settings, len(expected))
    wait_limit = max(
        read_timeout + (settle_ms / 1000.0) + RECEIVE_DEADLINE_MARGIN_SECONDS,
        (
            receive_estimate * RECEIVE_DEADLINE_SAFETY_FACTOR
            + read_timeout
            + (settle_ms / 1000.0)
            + RECEIVE_DEADLINE_MARGIN_SECONDS
        ),
        2.0,
    )
    # WATCHOUT: The join deadline is larger than the quiet timeout because the
    # physical line may still be draining at low baud after the write call returns.
    wait_deadline = time.monotonic() + wait_limit
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
    elif score.classification in {"wrong-nonce", "mixed-nonce"}:
        status = score.classification
    elif not received_bytes:
        status = "no-data"
    elif score.classification in {"eight-bit-masked", "eight-bit-not-clean"}:
        status = "eight-bit-not-clean"
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
        score_classification=score.classification,
        evidence=score.evidence,
        nonce_summary=payload.nonce.compact() if payload.nonce else "",
        payload_mode=payload.payload_mode,
        bytes_received_at_write_done=bytes_received_at_write_done,
    )


def aggregate_metrics(trials: Sequence[TrialResult]) -> ScoreMetrics:
    """Aggregate burst metrics into one candidate metric object.

    Ratios are arithmetic means across trials. Byte and line counts are summed
    so reports show the total missing/extra/stale evidence across repeats.
    """
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
        exact_prefix_bytes=sum(trial.metrics.exact_prefix_bytes for trial in trials),
        valid_probe_line_count=sum(trial.metrics.valid_probe_line_count for trial in trials),
        current_nonce_line_count=sum(
            trial.metrics.current_nonce_line_count for trial in trials
        ),
        wrong_nonce_line_count=sum(trial.metrics.wrong_nonce_line_count for trial in trials),
        high_bit_bytes_sent=sum(trial.metrics.high_bit_bytes_sent for trial in trials),
        high_bit_bytes_received=sum(
            trial.metrics.high_bit_bytes_received for trial in trials
        ),
        high_bit_exact_ratio=statistics.fmean(
            trial.metrics.high_bit_exact_ratio for trial in trials
        ),
        high_bit_stripped_count=sum(
            trial.metrics.high_bit_stripped_count for trial in trials
        ),
        seven_bit_masked_match_ratio=statistics.fmean(
            trial.metrics.seven_bit_masked_match_ratio for trial in trials
        ),
    )


def combined_evidence(trials: Sequence[TrialResult]) -> tuple[str, ...]:
    """Return ordered evidence labels from multiple trials."""
    labels: list[str] = []
    for trial in trials:
        for label in trial.evidence:
            if label not in labels:
                labels.append(label)
    return tuple(labels)


def aggregate_candidate_result(
    index: int,
    total: int,
    settings: SerialSettings | DualSerialSettings,
    trials: list[TrialResult],
    elapsed_sec: float,
    opening_error: str | None = None,
) -> CandidateResult:
    """Aggregate one candidate's burst trials into a candidate result.

    Notes:
        Status precedence is intentionally conservative: stale nonce/output
        beats serial errors, errors beat no-data, and high-bit failures remain
        failures even when ASCII markers were visible.
    """
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
            evidence=(),
        )

    scores = [trial.score for trial in trials]
    score = statistics.fmean(scores) if scores else 0.0
    high_quality_trials = sum(1 for trial in trials if trial.score >= 98.0)
    repeatability = high_quality_trials / max(len(trials), 1)
    errors = [trial.error for trial in trials if trial.error]
    stale_trials = [trial for trial in trials if trial.status in STALE_STATUSES]
    metrics = aggregate_metrics(trials)

    # WHY: Stale or wrong-nonce data invalidates the candidate score more than a
    # weak byte match does, so it wins status precedence during aggregation.
    if stale_trials:
        status = stale_trials[0].status
    elif errors:
        status = "error"
    elif not trials or sum(trial.bytes_received for trial in trials) == 0:
        status = "no-data"
    elif any(trial.status == "eight-bit-not-clean" for trial in trials):
        status = "eight-bit-not-clean"
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
        evidence=combined_evidence(trials),
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
                        run_id=options.run_id,
                        switch_hash=switch_note_hash(options.switch_note),
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
    except SERIAL_IO_ERRORS as exc:
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
        output_settings,
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
                        run_id=options.run_id,
                        switch_hash=switch_note_hash(options.switch_note),
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
    except SERIAL_IO_ERRORS as exc:
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
    """Return descending sort key fields for candidate ranking.

    Ranking prioritizes overall score, line integrity, exact byte ratio, then
    received byte count. Callers sort this key with `reverse=True`.
    """
    return (
        result.score,
        result.metrics.line_integrity_ratio,
        result.metrics.exact_byte_match_ratio,
        result.bytes_received,
    )


def session_file_key(path: Path) -> Path:
    """Return a stable key for tracking files initialized this program session."""
    return path.expanduser().resolve(strict=False)


def session_file_mode(path: Path, initialized_paths: set[Path]) -> str:
    """Return append mode for report/log files so history is preserved.

    Reports and logs are now always appended across program restarts. This
    preserves prior evidence and avoids accidental loss from truncation.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    key = session_file_key(path)
    # WHY: preserve prior sessions by default; callers can archive/rotate files
    # externally when they want fresh files.
    initialized_paths.add(key)
    return "a"


def open_session_text_report(path: Path) -> TextIO:
    """Open a text report block in append mode."""
    return path.open(
        session_file_mode(path, SESSION_TEXT_REPORT_PATHS),
        encoding="utf-8",
    )


def setup_logging(log_file: Path) -> logging.Logger:
    """Configure file logging and return the scan logger."""
    logger = logging.getLogger("serial_probe")
    logger.setLevel(logging.DEBUG)
    for existing_handler in list(logger.handlers):
        logger.removeHandler(existing_handler)
        existing_handler.close()
    handler = logging.FileHandler(
        log_file,
        mode=session_file_mode(log_file, SESSION_LOG_FILE_PATHS),
        encoding="utf-8",
    )
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    )
    logger.addHandler(handler)
    return logger


def bytes_to_jsonable(value: bytes | bytearray | memoryview) -> str | dict[str, Any]:
    """Return a JSON-safe representation of raw bytes.

    Printable ASCII bytes are emitted as text for readable diagnostics. Any
    non-printable payload is base64-wrapped with its byte count so raw probes can
    be serialized without data loss.
    """
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


def frame_label(settings: SerialSettings | DualSerialSettings) -> str:
    """Return data/parity/stop label for a result frame."""
    return f"{settings.data_bits}{settings.parity_code()}{settings.stop_bits}"


def frame_or_pair_label(settings: SerialSettings | DualSerialSettings) -> str:
    """Return one frame label or a compact input/output frame pair label."""
    if isinstance(settings, DualSerialSettings):
        input_frame = frame_label(settings.input_settings)
        output_frame = frame_label(settings.output_settings)
        return f"IN {input_frame} >> OUT {output_frame}"
    return frame_label(settings)


def progress_count_label(value: int) -> str:
    """Return a compact byte count for 80-column live result rows."""
    if value > 0 and value % KIB_BYTES == 0:
        return f"{value // KIB_BYTES}K"
    return str(value)


def progress_serial_label(settings: SerialSettings) -> str:
    """Return short baud/frame/flow text for 80-column progress rows."""
    return (
        f"{settings.baud} "
        f"{settings.data_bits}{settings.parity_code()}{settings.stop_bits} "
        f"{flow_control_code(settings.flow_control)}"
    )


def progress_settings_label(settings: SerialSettings | DualSerialSettings) -> str:
    """Return compact settings text for one live progress row."""
    if isinstance(settings, DualSerialSettings):
        return (
            f"I{progress_serial_label(settings.input_settings)}/"
            f"O{progress_serial_label(settings.output_settings)}"
        )
    return progress_serial_label(settings)


def progress_status_text(status: str, error: str | None) -> str:
    """Return compact status text for one live progress row."""
    text = status.upper().replace("-", " ")
    if error:
        text = f"{text}: {terminal_text(error)}"
    return " ".join(text.split())


def append_status_to_progress(base: str, status: str) -> str:
    """Append status text without exceeding the fixed terminal width."""
    if not status:
        return fit_terminal_line(base)
    status = terminal_text(status)
    available = TERMINAL_COLUMNS - len(base) - 1
    if available <= 0:
        return fit_terminal_line(base)
    return f"{base} {status[:available]}"


def clean_ascii_transfer(result: CandidateResult) -> bool:
    """Return True when a result proves useful ASCII byte transfer.

    This does not prove raw 8-bit cleanliness, parity uniqueness, stop-bit
    uniqueness, or flow-control behavior.
    """
    if result.error or result.status in STALE_STATUSES:
        return False
    if result.status in {"error", "no-data", "partial-write", "weak", "eight-bit-not-clean"}:
        return False
    return result.score >= 90.0 and result.metrics.line_integrity_ratio > 0.0


def frame_ambiguity_lines(results: Sequence[CandidateResult]) -> list[str]:
    """Return report lines explaining byte-level frame ambiguity.

    ASCII payloads can pass under more than one parity/stop interpretation. The
    report calls out those ties so clean byte transfer is not over-read as a
    unique hardware switch mapping.
    """
    clean = [result for result in results if clean_ascii_transfer(result)]
    if not clean:
        return []
    best_score = max(result.score for result in clean)
    near_best = [
        result for result in clean if (best_score - result.score) <= TIE_SCORE_TOLERANCE
    ]
    frames = sorted({frame_or_pair_label(result.settings) for result in near_best})
    flows = sorted({result.settings.flow_control for result in near_best})
    stop_bits = sorted({result.settings.stop_bits for result in near_best})
    parities = sorted({result.settings.parity for result in near_best})
    lines: list[str] = []
    if len(frames) > 1:
        lines.append("EVIDENCE:        BAUD ALIVE; ASCII BYTE TRANSFER.")
        lines.append(
            "FRAME:           AMBIGUOUS AMONG "
            + ", ".join(frames[:8])
            + (" ..." if len(frames) > 8 else "")
        )
    if len(stop_bits) > 1:
        lines.append("STOP BITS:       NOT BYTE-PROVABLE IN THIS TRANSFER.")
    if len(parities) > 1:
        lines.append("PARITY:          NOT DISTINGUISHED BY THIS PAYLOAD.")
    if len(flows) > 1 or any(flow != "none" for flow in flows):
        lines.append("FLOW:            NORMAL TRANSFER ONLY; FLOW NOT OBSERVED.")
    return lines


def stale_nonce_seen(results: Sequence[CandidateResult]) -> bool:
    """Return True if wrong-run or stale candidate data was seen."""
    return any(
        result.status in STALE_STATUSES or result.metrics.wrong_nonce_line_count > 0
        for result in results
    )


def format_progress(result: CandidateResult) -> str:
    """Return one console progress line for a candidate result."""
    indicator = result_indicator(result.score, result.status, result.error)
    base = (
        f"[{result.index:04d}/{result.total:04d}] {indicator} "
        f"{progress_settings_label(result.settings)} "
        f"S={progress_count_label(result.bytes_sent)} "
        f"R={progress_count_label(result.bytes_received)} "
        f"C={progress_count_label(result.bytes_drained_before)} "
        f"Q={result.score:03.0f}"
    )
    return append_status_to_progress(
        base,
        progress_status_text(result.status, result.error),
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
        return fit_terminal_line(
            f"SCAN {completed:04d}/{total:04d}: "
            f"EL={format_duration(elapsed)} LEFT=? FIN=?"
        )

    average = elapsed / completed
    remaining_seconds = average * remaining_count
    finish = format_finish_clock(remaining_seconds, clock_now)
    return fit_terminal_line(
        f"SCAN {completed:04d}/{total:04d}: "
        f"EL={format_duration(elapsed)} "
        f"AVG={format_duration(average)}/SET "
        f"LEFT={format_duration(remaining_seconds)} "
        f"FIN={finish}"
    )


def is_recommendable_result(result: CandidateResult | None) -> bool:
    """Return True when a result is strong enough to call a recommendation.

    Stale, wrong-nonce, error, no-data, weak, and high-bit-not-clean rows are
    never recommendable regardless of their numeric score.
    """
    if result is None or result.error:
        return False
    if result.status in {
        "error",
        "no-data",
        "stale-output",
        "wrong-nonce",
        "mixed-nonce",
        "partial-write",
        "weak",
        "eight-bit-not-clean",
    }:
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
        and result.status
        not in {"error", "no-data", "stale-output", "wrong-nonce", "mixed-nonce", "partial-write"}
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
    if result.status in {"wrong-nonce", "mixed-nonce"}:
        return "NO MATCH. STALE OR WRONG-NONCE DATA WAS RECEIVED."
    if result.error:
        return "NO MATCH. BEST ROW ENDED WITH ERROR."
    if tied_count > 1:
        return "MULTIPLE TOP SETTINGS. REVIEW BEFORE SETTING SWITCHES."
    if is_recommendable_result(result) and result.score >= 99.0 and result.repeatability >= 1.0:
        return "CLEAN BYTE TRANSFER OBSERVED."
    if is_recommendable_result(result):
        return "STRONG BYTE TRANSFER. REVIEW AMBIGUITIES."
    if result.score >= 50.0:
        return "PARTIAL MATCH ONLY. NOT RELIABLE."
    return "NO CONFIDENT MATCH. CHECK CABLES, PORTS, FLOW CONTROL."


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
    while prompt_loop_active():
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
    return "resume"


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


def default_report_paths() -> tuple[Path, Path]:
    """Return fixed text report and debug log paths."""
    return Path("serial_probe_report.txt"), Path("serial_probe_debug.log")


def default_scan_options() -> ScanOptions:
    """Return practical defaults for the interactive scan.

    Defaults are tuned for the documented COM1-to-COM5 printer-buffer setup:
    quick 512-byte discovery, one burst per candidate, stale-output clearing on,
    top-match validation on, and flow/backpressure validation enabled.
    """
    default_text_report, default_log = default_report_paths()
    return ScanOptions(
        in_port="COM1",
        out_port="COM5",
        input_baud=38400,
        output_baud=38400,
        min_baud=300,
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
        auto_validate_flow_control=True,
        flow_validate_size_bytes=FLOW_VALIDATE_PAYLOAD_BYTES,
        auto_stress_input_backpressure=INPUT_BACKPRESSURE_STRESS_ENABLED_DEFAULT,
        input_backpressure_stress_bytes=INPUT_BACKPRESSURE_STRESS_BYTES,
    )


def ensure_run_id(options: ScanOptions, prefix: str = "R") -> ScanOptions:
    """Return options with a run id assigned for nonce-bearing payloads."""
    if options.run_id:
        return options
    return dataclasses.replace(options, run_id=make_run_id(prefix))


def validate_options(options: ScanOptions) -> None:
    """Validate scan options before launching hardware I/O.

    Raises:
        ValueError: If ports, baud values, payload sizes, timing values, drain
        limits, or report paths are outside the supported operating envelope.
    """
    ensure_distinct_ports(options.in_port, options.out_port)
    validate_supported_baud(options.input_baud)
    validate_supported_baud(options.output_baud)
    validate_supported_baud(options.min_baud)
    validate_supported_baud(options.max_baud)
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
    if not options.no_pre_drain and options.pre_drain_timeout < options.pre_drain_quiet:
        raise ValueError("CLEAR OUTPUT TIME MUST BE >= QUIET TIME")
    if options.max_drain_bytes <= 0:
        raise ValueError("MAX CLEAR BYTES MUST BE POSITIVE")
    if options.validate_size_1_bytes < minimum_payload_size():
        raise ValueError(f"VALIDATE SIZE 1 MUST BE AT LEAST {minimum_payload_size()} BYTES")
    if options.flow_validate_size_bytes < minimum_payload_size():
        raise ValueError(f"FLOW VALIDATE SIZE MUST BE AT LEAST {minimum_payload_size()} BYTES")
    if options.input_backpressure_stress_bytes < minimum_payload_size():
        raise ValueError(
            f"BUFFER-FULL STRESS SIZE MUST BE AT LEAST {minimum_payload_size()} BYTES"
        )
    validate_report_path(options.text_report)
    validate_report_path(options.log_file)
    available_bauds(options.min_baud, options.max_baud)


def phase0_baseline_settings(baud: int) -> SerialSettings:
    """Return the fixed baseline settings used by the baud liveness sweep."""
    return SerialSettings(
        baud=baud,
        data_bits=PHASE0_BASELINE_DATA_BITS,
        parity=PHASE0_BASELINE_PARITY,
        stop_bits=PHASE0_BASELINE_STOP_BITS,
        flow_control=PHASE0_BASELINE_FLOW_CONTROL,
    )


def phase0_start_marker(nonce: ProbeNonce | None) -> bytes:
    """Return the compact Phase 0 begin marker."""
    return f"<<<SERIAL_PROBE_BEGIN PHASE0{nonce_field_text(nonce)}>>>\r\n".encode(
        "ascii"
    )


def phase0_end_marker(nonce: ProbeNonce | None) -> bytes:
    """Return the compact Phase 0 end marker."""
    return f"<<<SERIAL_PROBE_END PHASE0{nonce_field_text(nonce)}>>>\r\n".encode(
        "ascii"
    )


def phase0_minimum_payload_size(nonce: ProbeNonce | None = None) -> int:
    """Return the smallest structural payload usable by Phase 0."""
    start = phase0_start_marker(nonce)
    end = phase0_end_marker(nonce)
    return len(start) + len(make_probe_line(1, "LIVE", "", nonce)) + len(end)


def phase0_payload_bytes() -> int:
    """Return the fixed internal Phase 0 liveness payload size."""
    return max(
        PHASE0_PAYLOAD_BYTES,
        phase0_minimum_payload_size(representative_nonce()),
    )


def generate_phase0_payload(
    payload_bytes: int | None = None,
    nonce: ProbeNonce | None = None,
) -> ProbePayload:
    """Generate the compact structural payload used by Phase 0 liveness.

    Args:
        payload_bytes: Optional exact byte count. `None` uses the fixed internal
            Phase 0 size and undersized values are raised to the structural
            minimum.
        nonce: Optional run/candidate/trial identity.

    Returns:
        Phase 0 `ProbePayload` with one checksum-protected `LIVE` line.
    """
    byte_count = phase0_payload_bytes() if payload_bytes is None else payload_bytes
    byte_count = max(byte_count, phase0_minimum_payload_size(nonce))
    start = phase0_start_marker(nonce)
    end = phase0_end_marker(nonce)
    empty_line = make_probe_line(1, "LIVE", "", nonce)
    data_len = byte_count - len(start) - len(empty_line) - len(end)
    if data_len < 0:
        raise ValueError("PHASE 0 PAYLOAD IS TOO SMALL")
    line = make_probe_line(1, "LIVE", repeated_ascii_pattern(1, data_len), nonce)
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
        payload_mode=PAYLOAD_MODE_PHASE0,
        nonce=nonce,
    )


def phase0_fixed_settings_label() -> str:
    """Return a compact description of fixed Phase 0 liveness settings."""
    return (
        f"{phase0_payload_bytes()} BYTES X {PHASE0_BURSTS}, "
        "8E1 FLOW=NONE, "
        f"READ>={PHASE0_READ_TIMEOUT:.2f}S, "
        f"PAUSE={PHASE0_SETTLE_MS}MS, "
        f"CLEAR>={PHASE0_PRE_DRAIN_TIMEOUT:.2f}S/"
        f"{PHASE0_PRE_DRAIN_QUIET:.2f}S"
    )


def phase0_scan_options(options: ScanOptions) -> ScanOptions:
    """Return fixed internal options for the Phase 0 baud-liveness sweep.

    Phase 0 ignores the operator's normal payload size, burst count, turbo mode,
    and top-match prompt because it is a boolean gate over baud pairs.
    """
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
    print(f"  ASSUMED CAPACITY:     {byte_size_detail(BUFFER_PURGE_CAPACITY_BYTES)}")
    print(f"  OUTPUT BAUD TRIES:    {settings_count}")
    print_wrapped_value(
        "  METHOD:              ",
        "READ BUFFER OUTPUT UNTIL QUIET BEFORE SENDING NEW TEST DATA.",
    )
    print_wrapped_value(
        "  NOTE:                ",
        "SHORT QUIET TIME DOES NOT PROVE A FULL BUFFER IS EMPTY.",
    )
    print(border_line(REPORT_WIDTH))


def purge_buffer_output(
    serial_module: Any,
    options: ScanOptions,
    logger: logging.Logger,
    reason: str,
    settings_list: Sequence[SerialSettings] | None = None,
) -> DrainResult:
    """Drain a stateful serial printer buffer before a test stage.

    Args:
        serial_module: Imported pyserial module or test double.
        options: Active scan options.
        logger: Logger for purge diagnostics.
        reason: Operator-facing explanation printed before purge attempts.
        settings_list: Optional output-side settings to purge. When omitted,
            Phase 0 baseline settings across the scan baud range are used.

    Returns:
        Combined drain outcome across all attempted output settings.

    Notes:
        The purge is intentionally output-side only. It is meant to clear the
        device FIFO before scoring, not to validate input-side transmit settings.
    """
    if not BUFFER_PURGE_ENABLED:
        return DrainResult(0, 0.0, True, "disabled", None)
    settings = list(settings_list) if settings_list is not None else buffer_purge_settings(options)
    if not settings:
        return DrainResult(0, 0.0, True, "no-settings", None)

    buffer_purge_banner(reason, len(settings))
    started = time.monotonic()
    total_drained = 0
    all_quiet = True
    issue_reasons: list[str] = []
    issues: list[str] = []
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
        except SERIAL_IO_ERRORS as exc:
            error = str(exc)
            logger.info("buffer purge failed opening %s: %s", setting.label(), error)
            all_quiet = False
            issue_reasons.append("error")
            issues.append(f"{setting.label()}: {error}")
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
        if not drain.quiet or drain.error:
            all_quiet = False
            issue_reasons.append(drain.reason)
            issue = f"{setting.label()}: {drain.reason}"
            if drain.error:
                issue = f"{issue}: {drain.error}"
            issues.append(issue)
    elapsed = time.monotonic() - started
    quiet = all_quiet and not issues
    unique_issue_reasons = list(dict.fromkeys(issue_reasons))
    result_reason = "quiet"
    if not quiet:
        result_reason = (
            unique_issue_reasons[0]
            if len(unique_issue_reasons) == 1
            else "incomplete"
        )
    result = DrainResult(
        bytes_drained=total_drained,
        elapsed_sec=elapsed,
        quiet=quiet,
        reason=result_reason,
        error="; ".join(issues) if issues else None,
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


def run_known_baud_purge(
    serial_module: Any,
    options: ScanOptions,
    logger: logging.Logger,
    settings: SerialSettings,
    max_seconds: float,
    reason: str,
    capacity_bytes: int = BUFFER_PURGE_CAPACITY_BYTES,
) -> DrainResult:
    """Run an explicit long-limit purge using a known output-side frame.

    The time limit is based on output baud, frame width, and a safety factor so
    known-baud validation can wait long enough for a full buffer to drain.
    """
    base_estimate = estimated_buffer_drain_seconds(
        settings,
        capacity_bytes,
        safety_factor=1.0,
    )
    safe_estimate = estimated_buffer_drain_seconds(settings, capacity_bytes)
    print()
    print_report_title("KNOWN-BAUD PURGE")
    print_wrapped_value("  REASON:              ", reason)
    print(f"  OUTPUT PORT:          {options.out_port}")
    print(f"  SETTING:              {settings.label()}")
    print(
        f"  {byte_size_label(capacity_bytes)} DRAIN ESTIMATE:   "
        f"{format_duration(base_estimate)} RAW"
    )
    print(f"  WITH SAFETY FACTOR:   {format_duration(safe_estimate)}")
    print(f"  THIS RUN LIMIT:       {format_duration(max_seconds)}")
    print("  NOTE:                 ONLY THIS EXPLICIT PURGE USES THE LONG LIMIT.")
    print(border_line(REPORT_WIDTH))
    started = time.monotonic()
    try:
        with open_serial_port(
            serial_module,
            options.out_port,
            settings,
            max(options.read_timeout, 1.0),
        ) as out_serial:
            result = drain_output_until_quiet(
                out_serial=out_serial,
                quiet_seconds=max(options.pre_drain_quiet, BUFFER_PURGE_QUIET_SECONDS),
                max_seconds=max_seconds,
                max_bytes=max(options.max_drain_bytes, capacity_bytes * 2),
                progress_interval=options.progress_interval,
                progress=console_progress,
                prefix=f"[KNOWN PURGE {settings.label()}]",
                logger=logger,
            )
    except SERIAL_IO_ERRORS as exc:
        result = DrainResult(0, time.monotonic() - started, False, "error", str(exc))
    print()
    print_report_title("KNOWN-BAUD PURGE COMPLETE")
    print(f"  BYTES DRAINED:        {result.bytes_drained}")
    print(f"  RUN TIME:             {format_duration(result.elapsed_sec)}")
    print(f"  STATUS:               {'QUIET' if result.quiet else result.reason.upper()}")
    if result.error:
        print_wrapped_value("  DETAIL:               ", result.error)
    print(border_line(REPORT_WIDTH))
    return result


def known_output_purge_settings(
    settings_list: Sequence[SerialSettings | DualSerialSettings],
) -> list[SerialSettings]:
    """Return unique output-side settings for known-baud purge passes."""
    return unique_serial_settings(
        [receive_side_settings(settings) for settings in settings_list]
    )


def run_known_output_purges(
    serial_module: Any,
    options: ScanOptions,
    logger: logging.Logger,
    settings_list: Sequence[SerialSettings | DualSerialSettings],
    reason: str,
    capacity_bytes: int = BUFFER_PURGE_CAPACITY_BYTES,
) -> list[DrainResult]:
    """Run long-limit purges for known output-side serial settings."""
    output_settings = known_output_purge_settings(settings_list)
    results: list[DrainResult] = []
    for index, settings in enumerate(output_settings, start=1):
        indexed_reason = reason
        if len(output_settings) > 1:
            indexed_reason = (
                f"{reason} OUTPUT SETTING {index}/{len(output_settings)}."
            )
        results.append(
            run_known_baud_purge(
                serial_module=serial_module,
                options=options,
                logger=logger,
                settings=settings,
                max_seconds=estimated_buffer_drain_seconds(settings, capacity_bytes),
                reason=indexed_reason,
                capacity_bytes=capacity_bytes,
            )
        )
    return results


def phase0_extra_byte_limit(expected_byte_count: int) -> int:
    """Return the tolerated extra-byte limit for Phase 0 liveness."""
    ratio_limit = int(expected_byte_count * PHASE0_MAX_EXTRA_BYTES_RATIO)
    return max(PHASE0_MAX_EXTRA_BYTES, ratio_limit)


def classify_phase0_liveness(
    result: CandidateResult,
    expected_byte_count: int,
) -> Phase0LivenessDecision:
    """Return the conservative Phase 0 liveness decision for one baud result.

    A pair is alive only when it has current-run probe structure, complete
    markers, enough score, no serial error, no stale output, and only limited
    extra bytes.
    """
    if result.error or result.status == "error":
        return Phase0LivenessDecision(False, "SERIAL ERROR")
    if result.status == "partial-write":
        return Phase0LivenessDecision(False, "PARTIAL WRITE")
    if result.status == "stale-output":
        return Phase0LivenessDecision(False, "OUTPUT NOT QUIET")
    if result.status in {"wrong-nonce", "mixed-nonce"}:
        return Phase0LivenessDecision(False, "STALE/WRONG NONCE")
    if result.status == "no-data" or result.bytes_received <= 0:
        return Phase0LivenessDecision(False, "NO DATA")
    if result.bytes_sent < expected_byte_count:
        return Phase0LivenessDecision(False, "INCOMPLETE SEND")
    if result.metrics.extra_bytes > phase0_extra_byte_limit(expected_byte_count):
        return Phase0LivenessDecision(False, "EXTRA OUTPUT")
    if result.metrics.line_integrity_ratio < PHASE0_MIN_LINE_INTEGRITY:
        return Phase0LivenessDecision(False, "NO VALID PROBE LINE")
    if (
        not result.metrics.start_marker_present
        and not result.metrics.end_marker_present
    ):
        return Phase0LivenessDecision(False, "NO PROBE MARKER")
    if (
        not result.metrics.start_marker_present
        or not result.metrics.end_marker_present
    ):
        return Phase0LivenessDecision(False, "INCOMPLETE PROBE MARKER")
    if result.score < PHASE0_MIN_ALIVE_SCORE:
        return Phase0LivenessDecision(False, f"LOW SCORE {result.score:.1f}")
    return Phase0LivenessDecision(True, "VALID PROBE STRUCTURE")


def dual_phase0_settings(input_baud: int, output_baud: int) -> DualSerialSettings:
    """Return fixed baseline dual-bank settings for one baud pair."""
    return DualSerialSettings(
        input_settings=phase0_baseline_settings(input_baud),
        output_settings=phase0_baseline_settings(output_baud),
    )


def discovery_frame_priority(settings: SerialSettings) -> tuple[int, int, int, int]:
    """Return the frame-test priority used during staged discovery.

    Common byte-transfer frames are tried before unusual mark/space or two-stop
    variants, and flow remains off until a likely frame is known.
    """
    parity_rank = {
        "none": 0,
        "even": 1,
        "odd": 2,
        "mark": 3,
        "space": 4,
    }.get(settings.parity, 9)
    flow_rank = {
        "none": 0,
        "xon/xoff": 1,
        "dsr/dtr": 2,
        "rts/cts": 3,
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


def frame_candidates_for_baud(baud: int) -> list[SerialSettings]:
    """Return frame candidates for one baud in deterministic discovery order."""
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


def unique_serial_settings(
    settings_list: Sequence[SerialSettings],
) -> list[SerialSettings]:
    """Return serial settings with duplicates removed while preserving order."""
    unique: list[SerialSettings] = []
    seen: set[SerialSettings] = set()
    for settings in settings_list:
        if settings in seen:
            continue
        unique.append(settings)
        seen.add(settings)
    return unique


def dual_output_frame_sweep_for_pair(
    input_baud: int,
    output_baud: int,
) -> list[DualSerialSettings]:
    """Hold Phase 0 input framing and sweep output frames.

    The staged scan first asks whether the output side can be decoded while the
    input side stays at the Phase 0 baseline. This narrows the later input sweep.
    """
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
    """Return all input/output flow combinations for one dual-bank frame pair.

    Flow is expanded after frame evidence because the 16 flow combinations are
    meaningful only once byte transfer can already be decoded.
    """
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
    state = "YES" if result.alive else "NO"
    base = (
        f"PH0 [{index:04d}/{total:04d}] LIVE={state:<3} "
        f"I={result.input_baud:>5} O={result.output_baud:>5} "
        f"R={progress_count_label(result.bytes_received)} "
        f"C={progress_count_label(result.bytes_drained_before)} "
        f"Q={result.score:03.0f}"
    )
    return append_status_to_progress(base, result.reason)


def select_dual_phase0_pairs(
    results: Sequence[DualBaudLivenessResult],
) -> tuple[list[tuple[int, int]], str | None]:
    """Return baud pairs to expand into dual-bank frame testing.

    Alive pairs are selected in score order with a fixed cap. If no pair is
    alive, selection is deferred to the explicit no-signal fallback prompt.
    """
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
        and result.status
        not in {"error", "no-data", "stale-output", "wrong-nonce", "mixed-nonce", "partial-write"}
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


def same_baud_fallback_pairs(options: ScanOptions) -> list[tuple[int, int]]:
    """Return same-baud pairs in scan order for no-signal Phase 0 recovery."""
    return [(baud, baud) for baud in scan_bauds(options.min_baud, options.max_baud)]


def phase0_results_all_serial_errors(report: DualBaudLivenessReport) -> bool:
    """Return True when Phase 0 failed only because serial I/O could not run."""
    return bool(report.results) and all(result.error for result in report.results)


def first_phase0_error(report: DualBaudLivenessReport) -> str | None:
    """Return the first Phase 0 serial error detail, if any."""
    for result in report.results:
        if result.error:
            return result.error
    return None


def phase0_no_signal_fallback_default(pair_count: int) -> bool:
    """Return the operator default for no-signal same-baud fallback."""
    return 0 < pair_count <= PHASE0_NO_SIGNAL_AUTO_FALLBACK_BAUD_LIMIT


def prompt_phase0_no_signal_fallback(
    options: ScanOptions,
    report: DualBaudLivenessReport,
) -> tuple[list[tuple[int, int]], str | None]:
    """Ask whether to continue with a same-baud frame fallback after Phase 0."""
    fallback_pairs = same_baud_fallback_pairs(options)
    if not fallback_pairs:
        return [], "NO SELECTED BAUDS WERE AVAILABLE FOR FRAME FALLBACK."

    print()
    print_report_title("PHASE 0 NO-SIGNAL FALLBACK")
    print("PHASE 0 DID NOT SELECT A BAUD PAIR FOR FRAME TESTING.")
    if phase0_results_all_serial_errors(report):
        print("EVERY PHASE 0 ROW ENDED WITH A SERIAL PORT ERROR.")
        detail = first_phase0_error(report)
        if detail:
            print_wrapped_value("  DETAIL:               ", detail)
        print("CHECK COM PORT NUMBERS AND CLOSE OTHER PROGRAMS USING THE PORTS.")
        print(border_line(REPORT_WIDTH))
        return [], "NO PHASE 0 SIGNAL; SERIAL PORT ERRORS PREVENTED FRAME FALLBACK."

    candidate_count = dual_scan_candidate_count(fallback_pairs)
    print("A FRAME FALLBACK CAN STILL FIND SETTINGS WHEN THE FIXED 8E1 PROBE IS WRONG.")
    print(f"SAME-BAUD PAIRS:       {len(fallback_pairs)}")
    print(f"FRAME PAIRS TO TEST:   {candidate_count}")
    print("FOR A WIDE RANGE, NARROW THE BAUD RANGE FIRST IF THIS IS TOO MANY.")
    print(border_line(REPORT_WIDTH))
    default = phase0_no_signal_fallback_default(len(fallback_pairs))
    if not prompt_yes_no("RUN SAME-BAUD FRAME FALLBACK", default):
        return [], "NO PHASE 0 SIGNAL; SAME-BAUD FRAME FALLBACK DECLINED."
    return fallback_pairs, "NO PHASE 0 SIGNAL; RUNNING SAME-BAUD FRAME FALLBACK."


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


def phase0_pair_list_label(pairs: Sequence[tuple[int, int]]) -> str:
    """Return compact input/output baud-pair text."""
    if not pairs:
        return "(NONE)"
    return ", ".join(f"IN {input_baud}/OUT {output_baud}" for input_baud, output_baud in pairs)


def dual_phase0_result_row(result: DualBaudLivenessResult) -> str:
    """Return one compact Phase 0 result row for text reports."""
    live = "Y" if result.alive else "N"
    prefix = (
        f"{result.input_baud:>6} "
        f"{result.output_baud:>6} "
        f"{live:<4} "
        f"{result.score:>5.1f} "
        f"{result.bytes_sent:>5} "
        f"{result.bytes_received:>5} "
        f"{result.bytes_drained_before:>5} "
        f"{result.status.upper()[:10]:<10} "
    )
    reason_width = max(1, REPORT_WIDTH - len(prefix))
    return prefix + fit_terminal_line(terminal_text(result.reason), reason_width)


def dual_phase0_report_lines(report: DualBaudLivenessReport) -> list[str]:
    """Return detailed Phase 0 liveness report lines."""
    input_bauds = sorted({input_baud for input_baud, _ in report.alive_pairs}, reverse=True)
    output_bauds = sorted({output_baud for _, output_baud in report.alive_pairs}, reverse=True)
    lines = [
        "PHASE 0 BAUD LIVENESS:",
        f"RUN TIME:        {format_duration(report.elapsed_sec)}",
        f"BAUD PAIRS:      {len(report.tested_pairs)}/{report.total_pairs} TESTED",
        f"ALIVE PAIRS:     {len(report.alive_pairs)}",
    ]
    lines.extend(
        wrapped_value_lines(
            "INPUT BAUDS:     ",
            ", ".join(str(baud) for baud in input_bauds) if input_bauds else "(NONE)",
            REPORT_WIDTH,
        )
    )
    lines.extend(
        wrapped_value_lines(
            "OUTPUT BAUDS:    ",
            ", ".join(str(baud) for baud in output_bauds) if output_bauds else "(NONE)",
            REPORT_WIDTH,
        )
    )
    lines.extend(
        wrapped_value_lines(
            "SELECTED PAIRS:  ",
            phase0_pair_list_label(report.selected_pairs),
            REPORT_WIDTH,
        )
    )
    if report.fallback_reason:
        lines.extend(
            wrapped_value_lines(
                "NOTE:            ",
                report.fallback_reason,
                REPORT_WIDTH,
            )
        )
    lines.extend(
        [
            "",
            "INBAUD OUTBAUD LIVE SCORE  SENT  READ   CLR STATUS     REASON",
            border_line(REPORT_WIDTH),
        ]
    )
    if not report.results:
        lines.append("(NOT RUN)")
    for result in report.results:
        lines.append(dual_phase0_result_row(result))
    lines.append(border_line(REPORT_WIDTH))
    return lines


def write_phase0_text_report(
    path: Path,
    metadata: dict[str, Any],
    phase0: DualBaudLivenessReport,
) -> None:
    """Write a Phase 0 or early-exit report block to the text report."""
    path.parent.mkdir(parents=True, exist_ok=True)
    created = terminal_text(
        dt.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")
    )
    switch_note = terminal_text(str(metadata.get("switch_note") or "").strip())
    started_at = terminal_text(metadata.get("started_at", ""))
    completed_at = terminal_text(metadata.get("completed_at", ""))
    run_id = terminal_text(metadata.get("run_id", ""))
    workflow = terminal_text(metadata.get("workflow", ""))
    in_port = terminal_text(metadata.get("in_port", ""))
    out_port = terminal_text(metadata.get("out_port", ""))
    lines = [
        "",
        border_line(REPORT_WIDTH),
        bordered_text("SERIAL PROBE PHASE 0 REPORT", REPORT_WIDTH),
        border_line(REPORT_WIDTH),
        f"WRITTEN:         {created}",
        f"STARTED UTC:     {started_at}",
        f"COMPLETED UTC:   {completed_at}",
        f"RUN ID:          {run_id}",
        f"WORKFLOW:        {workflow}",
        f"COM PATH:        {in_port} >> BUFFER >> {out_port}",
        f"DEVICE NOTE:     {switch_note if switch_note else '(NOT ENTERED)'}",
    ]
    lines.extend(
        wrapped_value_lines(
            "OUTCOME:         ",
            metadata.get("outcome", ""),
            REPORT_WIDTH,
        )
    )
    lines.extend(["", *dual_phase0_report_lines(phase0)])
    lines.extend(
        [
            "",
            "INTERPRETATION NOTES:",
            "  PHASE 0 IS A BAUD-LIVENESS GATE USING FIXED 8E1 FLOW=NONE.",
            "  ALIVE PAIRS SHOW BASIC CHECKED PROBE STRUCTURE AT THAT BAUD PAIR.",
            "  A NON-ALIVE ROW DOES NOT PROVE THE BAUD IS IMPOSSIBLE WITH ANOTHER FRAME.",
            "  FRAME, FLOW, RAW BYTE, AND ETX/ACK NEED LATER WORKFLOWS.",
            border_line(REPORT_WIDTH),
        ]
    )
    with open_session_text_report(path) as report_file:
        report_file.write("\n".join(lines) + "\n")


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
    print("MODE: INPUT AND OUTPUT PORT BAUDS ARE TESTED INDEPENDENTLY.")
    print_wrapped_value(
        "PORTS: ",
        (
            f"IN {options.in_port} >> BUFFER >> OUT {options.out_port}; "
            f"TEST={phase0_payload.byte_count} BYTES X {phase0_options.bursts}"
        ),
    )
    print("FIXED FRAME: 8E1 FLOW=NONE ON BOTH SIDES.")
    print(f"BAUD PAIRS: {len(pairs)} INPUT X OUTPUT COMBINATIONS.")
    print("LIVE PAIRS ARE RECORDED; PHASE 0 CONTINUES AUTOMATICALLY.")
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
    indicator = result_indicator(result.score, result.status, result.error)
    base = (
        f"[{result.index:04d}/{result.total:04d}] {indicator} "
        f"{progress_settings_label(settings)} "
        f"S={progress_count_label(result.bytes_sent)} "
        f"R={progress_count_label(result.bytes_received)} "
        f"C={progress_count_label(result.bytes_drained_before)} "
        f"Q={result.score:03.0f}"
    )
    return append_status_to_progress(
        base,
        progress_status_text(result.status, result.error),
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
        print(bordered_text("BEST OBSERVED DUAL-BANK TRANSFER", REPORT_WIDTH))
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
        lines.append("ACTION:          CLEAN PAIR OBSERVED; DO NOT INFER DIP MEANING ALONE.")
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
    """Write a compact dual-bank scan report entry."""
    path.parent.mkdir(parents=True, exist_ok=True)
    validation_results = [] if validation_results is None else list(validation_results)
    created = terminal_text(
        dt.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")
    )
    switch_note = terminal_text(str(metadata.get("switch_note") or "").strip())
    started_at = terminal_text(metadata.get("started_at", ""))
    run_id = terminal_text(metadata.get("run_id", ""))
    in_port = terminal_text(metadata.get("in_port", ""))
    out_port = terminal_text(metadata.get("out_port", ""))
    selected_pairs = ", ".join(
        f"IN {input_baud}/OUT {output_baud}"
        for input_baud, output_baud in phase0.selected_pairs
    )
    lines = [
        "",
        border_line(REPORT_WIDTH),
        bordered_text("SERIAL PROBE DUAL-BANK REPORT", REPORT_WIDTH),
        border_line(REPORT_WIDTH),
        f"WRITTEN:         {created}",
        f"STARTED UTC:     {started_at}",
        f"RUN ID:          {run_id}",
        f"COM PATH:        {in_port} >> BUFFER >> {out_port}",
        f"DEVICE NOTE:     {switch_note if switch_note else '(NOT ENTERED)'}",
        "SCAN MODEL:      INPUT/OUTPUT PAIR TESTED; SWITCH MEANING NOT ASSUMED",
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
            "  ADVANCED MODE DOES NOT TREAT SW1/SW2 AS COMPLETE PORT PROFILES.",
            "  INPUT VALUES ARE FOR THE PC TRANSMIT PORT INTO THE BUFFER.",
            "  OUTPUT VALUES ARE FOR THE PC RECEIVE PORT FROM THE BUFFER.",
            "  THIS IS ONE POSSIBLE MODEL; SHARED FRAMING/FLOW IS ALSO POSSIBLE.",
            "  FLOW TRANSFER MATCHES DO NOT PROVE HANDSHAKE WITHOUT HOLD/BACKPRESSURE.",
            "  WRONG-NONCE DATA MEANS STALE BUFFER OUTPUT OR A WRONG CANDIDATE/TRIAL.",
            border_line(REPORT_WIDTH),
        ]
    )
    with open_session_text_report(path) as report_file:
        report_file.write("\n".join(lines) + "\n")


def dual_scan_candidate_count(pairs: Sequence[tuple[int, int]]) -> int:
    """Return total dual-bank frame candidates for selected baud pairs."""
    frame_count = len(DATA_BITS) * len(PARITIES) * len(STOP_BITS)
    return len(pairs) * frame_count * frame_count


def supported_baud_label() -> str:
    """Return the configured baud table as one operator-facing list."""
    return ", ".join(str(baud) for baud in BAUD_RATES)


def validate_supported_baud(baud: int) -> None:
    """Raise if baud is not one of the program baud rates."""
    if baud not in BAUD_RATES:
        raise ValueError("ENTER ONE OF: " + supported_baud_label())


def validate_report_path(path: Path) -> None:
    """Raise if a report/log path cannot be used as a write target."""
    text = str(path).strip()
    if not text:
        raise ValueError("PATH CANNOT BE BLANK")
    if any(ord(char) < 32 for char in text):
        raise ValueError("PATH CONTAINS A CONTROL CHARACTER")
    drive = path.drive
    remainder = text[len(drive) :] if drive else text
    if any(char in remainder for char in '<>"|?*') or ":" in remainder:
        raise ValueError("PATH CONTAINS A CHARACTER DOS/WINDOWS CANNOT USE")
    if path.exists() and path.is_dir():
        raise ValueError("PATH NAMES A DIRECTORY, NOT A FILE")
    parent = path.parent if str(path.parent) else Path(".")
    if not parent.exists():
        raise ValueError("PATH DIRECTORY DOES NOT EXIST")


def prompt_text(label: str, current: str) -> str:
    """Prompt for a string value, preserving current on blank input."""
    try:
        value = read_operator_input(
            f"{label.upper()} [{terminal_text(current)}]: "
        ).strip()
    except EOFError:
        return current
    return current if value == "" else value


def prompt_int(
    label: str,
    current: int,
    minimum: int | None = None,
    maximum: int | None = None,
) -> int:
    """Prompt for an integer value, preserving current on blank input."""
    while prompt_loop_active():
        try:
            value = read_operator_input(f"{label.upper()} [{current}]: ").strip()
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
        if maximum is not None and parsed > maximum:
            print(f"ENTER A VALUE <= {maximum}.")
            continue
        return parsed
    return current


def prompt_supported_baud(label: str, current: int) -> int:
    """Prompt for one baud value from the program baud table."""
    while prompt_loop_active():
        baud = prompt_int(label, current, minimum=1)
        try:
            validate_supported_baud(baud)
        except ValueError as exc:
            print(terminal_text(exc))
            continue
        return baud
    return current


def prompt_float(label: str, current: float, minimum: float | None = None) -> float:
    """Prompt for a float value, preserving current on blank input."""
    while prompt_loop_active():
        try:
            value = read_operator_input(f"{label.upper()} [{current}]: ").strip()
        except EOFError:
            return current
        if value == "":
            return current
        try:
            parsed = float(value)
        except ValueError:
            print("ENTER A NUMBER.")
            continue
        if not math.isfinite(parsed):
            print("ENTER A FINITE NUMBER.")
            continue
        if minimum is not None and parsed < minimum:
            print(f"ENTER A VALUE >= {minimum}.")
            continue
        return parsed
    return current


def prompt_port(label: str, current: str) -> str:
    """Prompt for a validated Windows COM port name."""
    while prompt_loop_active():
        port = prompt_text(label, current)
        try:
            validate_port_name(port)
        except ValueError as exc:
            print(f"PORT ERROR: {terminal_text(exc)}")
            continue
        return normalize_port_name(port)
    return normalize_port_name(current)


def prompt_yes_no(label: str, current: bool) -> bool:
    """Prompt for a yes/no value, preserving current on blank input."""
    default = "Y" if current else "N"
    while prompt_loop_active():
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
    return current


def prompt_yes_no_question(question: str, current: bool) -> bool:
    """Prompt for a yes/no answer using a full question string."""
    default = "Y" if current else "N"
    while prompt_loop_active():
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
    return current


def prompt_supported_baud_or_menu(label: str, current: int) -> int | None:
    """Prompt for a supported baud value, allowing return to main menu."""
    while prompt_loop_active():
        try:
            value = read_operator_input(
                f"{label.upper()} [{current}] (0=MENU): "
            ).strip()
        except EOFError:
            return None
        value = value.lstrip("\ufeff").strip().lower()
        if value in {"0", "m", "menu", "main", "return"}:
            return None
        if value == "":
            baud = current
        else:
            try:
                baud = int(value)
            except ValueError:
                print("ENTER A WHOLE NUMBER, OR 0 FOR MENU.")
                continue
        try:
            validate_supported_baud(baud)
        except ValueError as exc:
            print(terminal_text(exc))
            continue
        return baud
    return None


def parse_start_scan_workflow_choice(choice: str) -> str | None:
    """Return a start-scan workflow key for one operator choice."""
    choice = choice.lstrip("\ufeff").strip().lower()
    if choice == "":
        return "discovery"
    if choice in {"1", "d", "discovery", "auto", "automatic"}:
        return "discovery"
    if choice in {"2", "b", "bank", "bank2", "switch", "known", "known-baud", "device"}:
        return "bank2"
    if choice in {"3", "p", "phase0", "baud"}:
        return "phase0"
    if choice in {"4", "m", "menu", "main"}:
        return "menu"
    return None


def prompt_start_scan_workflow() -> str:
    """Ask which automated workflow option 1 should run."""
    print()
    print_report_title("START SCAN WORKFLOW")
    print("  1 AUTOMATED DISCOVERY")
    print("  2 KNOWN-BAUD DEVICE TEST")
    print("  3 PHASE 0 BAUD LIVENESS ONLY")
    print("  4 RETURN TO MAIN MENU")
    print(border_line(REPORT_WIDTH))
    while prompt_loop_active():
        try:
            choice = read_operator_input("ENTER SELECTION [1]: ")
        except EOFError:
            return "discovery"
        workflow = parse_start_scan_workflow_choice(choice)
        if workflow is not None:
            return workflow
        print("ENTER 1, 2, 3, OR 4.")
    return "discovery"


def start_scan_workflow_uses_baud_range(workflow: str) -> bool:
    """Return whether one start-scan workflow needs a baud range prompt."""
    return workflow in {"discovery", "phase0"}


def print_menu_help(paged: bool = True) -> None:
    """Print first-run operator help for the interactive CLI."""
    text_report, debug_log = default_report_paths()
    print_paged_lines(
        [
            *banner_lines(),
            "START: PYTHON SERIAL_PROBE.PY",
            "",
            "HELP - OPERATOR BRIEFING",
            "",
            "WHAT THIS PROGRAM DOES",
            "  SERIAL PROBE FINDS SERIAL SWITCH SETTINGS FOR A PRINTER BUFFER.",
            "  IT SENDS KNOWN TEST DATA INTO ONE COM PORT AND READS ANOTHER.",
            "  THE REPORT RANKS BAUD, DATA BITS, PARITY, STOP BITS, AND FLOW.",
            "  IT IS A TEST SET, NOT A TERMINAL EMULATOR OR MODEM PROGRAM.",
            "",
            "NORMAL HOOK-UP",
            "  PC COM1 TRANSMITS TO BUFFER INPUT.",
            "  BUFFER OUTPUT TRANSMITS TO PC COM5.",
            "  PATH: COM1 >> BUFFER INPUT >> BUFFER OUTPUT >> COM5.",
            "  SET BOTH BUFFER SWITCH BANKS THE SAME UNLESS THE TEST SAYS OTHERWISE.",
            "",
            "FIRST RUN CHECK LIST",
            "  1 CLEAR OR RESET THE BUFFER SO OLD PRINT DATA IS NOT POURING OUT.",
            "  2 USE OPTION 5 TO READ THE PRESENT PROGRAM SETUP.",
            "  3 USE OPTION 2 IF YOUR COM PORTS OR KNOWN BAUDS ARE NOT DEFAULT.",
            "  4 USE OPTION 1, THEN SELECT AUTOMATED DISCOVERY.",
            f"  5 SELECT BAUD RANGE. SUPPORTED: {supported_baud_label()}.",
            "  6 WAIT FOR THE REPORT. CTRL+C OPENS THE OPERATOR BREAK MENU.",
            "",
            "START SCAN WORKFLOW",
            "  1 AUTOMATED DISCOVERY",
            "    USE WHEN THE BUFFER SPEED OR FRAME IS UNKNOWN.",
            "    PHASE 0 FINDS LIVE INPUT/OUTPUT BAUD PAIRS.",
            "    THEN FRAME SWEEPS AND FLOW CHECKS LOOK FOR A CLEAN TRANSFER.",
            "    TOP MATCHES MAY BE VERIFIED BEFORE THE FINAL REPORT.",
            "",
            "  2 KNOWN-BAUD DEVICE TEST",
            "    USE WHEN INPUT AND OUTPUT BAUDS ARE ALREADY KNOWN.",
            "    BAUDS COME FROM OPTION 2 ON THE MAIN MENU.",
            "    TESTS ASCII, 8-BIT DATA, RAW CONTROL BYTES, ETX/ACK, AND FLOW.",
            "    BUFFER-FULL STRESS RUNS AFTER OUTPUT HOLD IS PROVEN.",
            "",
            "  3 PHASE 0 BAUD LIVENESS ONLY",
            "    QUICK BAUD-PAIR CHECK USING FIXED 8E1, FLOW NONE.",
            "    SHOWS WHICH BAUD PAIRS ARE ALIVE; IT IS NOT A FINAL SETTING.",
            "",
            "  4 RETURN TO MAIN MENU",
            "",
            "MAIN MENU",
            "  1 START SCAN:          SELECT ONE OF THE TESTS ABOVE.",
            "  2 SET COM PORTS/BAUD:  INPUT PORT, OUTPUT PORT, FIXED BAUDS.",
            "  3 SCAN/VALIDATE SETUP: MESSAGE SIZE, TEST COUNT, VERIFY, FLOW.",
            "  4 TIMING/STALE:        READ WAITS AND OLD-OUTPUT CLEARING.",
            "  5 CURRENT SETTINGS:    PRINT THE WHOLE ACTIVE SETUP.",
            "  6 HELP:                THIS OPERATOR BRIEFING.",
            "  0 QUIT:                END PROGRAM.",
            "",
            "OPERATOR NOTES",
            "  PROGRAM SETS BAUD, FRAME, AND FLOW WHEN IT OPENS THE PORTS.",
            "  DEVICE MANAGER DEFAULTS ARE NOT USED AS TEST SETTINGS.",
            "  INPUT AND OUTPUT BAUDS MAY DIFFER ON TWO-BANK BUFFERS.",
            "  AUTOMATED DISCOVERY AND PHASE 0 ASK FOR THE BAUD RANGE AT START.",
            "  PHASE 0 ALWAYS USES 8 DATA, EVEN PARITY, 1 STOP, FLOW NONE.",
            "  OPTION 3 DOES NOT CHANGE PHASE 0 ONLY.",
            "  QUICK STALE CLEARING REJECTS OLD OUTPUT BEFORE EACH TEST.",
            "  KNOWN-BAUD AND VERIFY RUNS USE LONGER CALCULATED PURGES.",
            "  EACH TEST PAYLOAD HAS RUN, CANDIDATE, AND TRIAL MARKS.",
            "  ASCII PASSING IS NOT ENOUGH; 8-BIT AND RAW BYTE TESTS MAY MATTER.",
            "",
            "READING THE SCREEN",
            "  PASS OR GOOD MEANS A USEFUL TRANSFER WAS SEEN.",
            "  PARTIAL MEANS SOME TEST DATA CAME THROUGH BUT NOT CLEANLY.",
            "  FAIL OR NO-DATA MEANS THAT SETTING DID NOT CARRY THE TEST.",
            "  STALE MEANS OLD BUFFER OUTPUT MADE THE TEST UNTRUSTWORTHY.",
            "  ERROR MEANS THE SERIAL DRIVER OR PORT REJECTED THE OPERATION.",
            "  SCORE IS 0 THROUGH 100; TRUST THE FINAL REPORT INTERPRETATION.",
            "",
            "REPORTS",
            f"  TEXT REPORT:       {terminal_text(text_report)} (NEW SESSION REPLACES OLD FILE).",
            f"  DEBUG LOG:         {terminal_text(debug_log)} (NEW SESSION REPLACES OLD FILE).",
            "  A RECOMMENDED SETTING NEEDS STRONG BYTE TRANSFER EVIDENCE.",
            "  MULTIPLE TOP SETTINGS MEANS REVIEW TIED ROWS BEFORE SETTING SWITCHES.",
            "  NO WORKING SETTING MEANS CHECK CABLES, PORTS, BAUDS, POWER, OR RESET.",
            "",
            "KEYS",
            "  ENTER ACCEPTS A DEFAULT WHEN A PROMPT SHOWS ONE IN BRACKETS.",
            "  0 USUALLY RETURNS TO THE MAIN MENU OR QUITS THE PRESENT MENU.",
            "  CTRL+C DURING A TEST ASKS RESUME, REPORT, MENU, OR QUIT.",
            "  AFTER A RUN THE PROGRAM ASKS RUN AGAIN, MAIN MENU, OR QUIT.",
        ],
        page_lines=HELP_BODY_LINES if paged else 0,
        pause_at_end=paged,
        return_label="RETURN TO MENU",
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
    fixed_baud_text = (
        f"IN {options.input_baud} / OUT {options.output_baud}"
        if options.input_baud != options.output_baud
        else str(options.input_baud)
    )
    try:
        bauds = available_bauds(options.min_baud, options.max_baud)
        baud_order = scan_bauds(options.min_baud, options.max_baud)
        timing_candidates = [phase0_baseline_settings(baud) for baud in baud_order]
        phase0_pair_count = len(bauds) * len(bauds)
        selected_pair_limit = min(DUAL_PHASE0_BAUD_PAIR_LIMIT, phase0_pair_count)
        fallback_pair_limit = min(DUAL_PHASE0_FALLBACK_PAIR_LIMIT, phase0_pair_count)
        frame_pairs_per_live_pair = dual_scan_candidate_count([(0, 0)])
        range_error: str | None = None
    except ValueError as exc:
        bauds = []
        baud_order = []
        timing_candidates = []
        phase0_pair_count = 0
        selected_pair_limit = 0
        fallback_pair_limit = 0
        frame_pairs_per_live_pair = 0
        range_error = str(exc)
    lines: list[str] = ["", *banner_lines(), "CURRENT SETTINGS"]
    lines.extend(setting_lines("START SCAN:", "DISCOVERY / KNOWN-BAUD DEVICE / PHASE 0"))
    lines.extend(
        setting_lines(
            "OPTION 2 PORTS:",
            f"{options.in_port} INPUT >> {options.out_port} OUTPUT",
        )
    )
    lines.extend(
        setting_lines(
            "OPTION 2 BAUDS:",
            f"INPUT {options.input_baud} / OUTPUT {options.output_baud}",
        )
    )
    lines.extend(
        setting_lines(
            "SCAN BAUD RANGE:",
            f"{options.min_baud}..{options.max_baud} (ASKED IN START SCAN 1 OR 3)",
        )
    )
    lines.extend(setting_lines("BAUDS:", len(bauds)))
    if baud_order:
        lines.extend(
            setting_lines("BAUD ORDER:", f"{baud_order[0]} DOWN TO {baud_order[-1]}")
        )
    if range_error:
        lines.extend(setting_lines("RANGE ERROR:", range_error))
    lines.extend(
        setting_lines(
            "PHASE 0 PAIRS:",
            f"{phase0_pair_count} INPUT/OUTPUT BAUD PAIRS AT 8E1 FLOW=NONE",
        )
    )
    lines.extend(
        setting_lines(
            "PHASE 0 SELECT:",
            f"UP TO {selected_pair_limit} LIVE PAIRS; FALLBACK UP TO {fallback_pair_limit}",
        )
    )
    lines.extend(
        setting_lines(
            "FRAME TESTS:",
            f"{frame_pairs_per_live_pair} INPUT/OUTPUT FRAME PAIRS PER LIVE BAUD PAIR",
        )
    )
    lines.extend(setting_lines("QUICK SCAN MSG:", f"{options.payload_bytes} BYTES"))
    lines.extend(setting_lines("SCAN COUNT:", options.bursts))
    lines.extend(setting_lines("PHASE 0:", phase0_fixed_settings_label()))
    lines.extend(
        setting_lines(
            "TURBO DISCOVERY:",
            "ON" if options.turbo_discovery_enabled else "OFF",
        )
    )
    lines.extend(
        setting_lines(
            "FRAME TIMING:",
            effective_timing_range_label(options, timing_candidates),
        )
    )
    lines.extend(
        setting_lines(
            "SCAN ORDER:",
            "PHASE 0 BAUD PAIRS; SAME-BAUD FALLBACK IF NEEDED; STAGED FRAME SWEEPS",
        )
    )
    lines.extend(
        setting_lines("ASK ON MATCH:", "YES" if options.ask_on_top_match else "NO")
    )
    lines.extend(
        setting_lines(
            "TOP-MATCH VERIFY:",
            (
                "OFF"
                if not options.auto_validate_top_matches
                else f"ON, SIZE={options.validate_size_1_bytes} BYTES"
            ),
        )
    )
    lines.extend(
        setting_lines(
            "FLOW TESTS:",
            (
                "OFF"
                if not options.auto_validate_flow_control
                else (
                    "ON; "
                    f"TRANSFER={options.flow_validate_size_bytes} BYTES; "
                    "BUFFER-FULL="
                    + (
                        f"{options.input_backpressure_stress_bytes} BYTES"
                        if options.auto_stress_input_backpressure
                        else "OFF"
                    )
                )
            ),
        )
    )
    lines.extend(setting_lines("READ WAIT:", f"{options.read_timeout:.2f}S"))
    lines.extend(setting_lines("OPEN PAUSE:", f"{options.settle_ms} MS"))
    lines.extend(
        setting_lines(
            "PER-TEST STALE:",
            (
                "NO"
                if options.no_pre_drain
                else (
                    f"YES, QUIET={options.pre_drain_quiet:.2f}S, "
                    f"QUICK LIMIT={options.pre_drain_timeout:.2f}S, "
                    f"MAX={options.max_drain_bytes} BYTES"
                )
            ),
        )
    )
    lines.extend(
        setting_lines(
            "KNOWN-BAUD PURGE:",
            "USES CALCULATED LONG LIMITS WHEN OUTPUT BAUD/FRAME IS KNOWN",
        )
    )
    lines.extend(setting_lines("TOP ROWS:", options.top))
    lines.extend(
        setting_lines(
            "KNOWN-BAUD TEST:",
            f"START SCAN 2; USES OPTION 2 PORTS AND BAUDS: {fixed_baud_text}",
        )
    )
    lines.extend(setting_lines("REPORT FILE:", options.text_report))
    lines.extend(
        setting_lines("DEVICE/SWITCH NOTE:", options.switch_note or "(ASK AT SCAN START)")
    )
    lines.extend(setting_lines("LOG FILE:", options.log_file))
    lines.extend(setting_lines("ESTIMATE:", "SHOWN AFTER PHASE 0 SELECTS LIVE BAUD PAIRS"))
    print_paged_lines(lines)


def configure_baud_range(
    options: ScanOptions,
    *,
    allow_menu: bool = False,
) -> ScanOptions | None:
    """Prompt for the baud range."""
    print("AVAILABLE BAUD RATES:")
    print(supported_baud_label())
    print("SCAN ORDER: FASTEST SELECTED BAUD FIRST.")
    if allow_menu:
        print("ENTER 0 AT EITHER BAUD PROMPT TO RETURN TO MAIN MENU.")
    while prompt_loop_active():
        if allow_menu:
            min_baud = prompt_supported_baud_or_menu("MINIMUM BAUD", options.min_baud)
            if min_baud is None:
                return None
            max_baud = prompt_supported_baud_or_menu("MAXIMUM BAUD", options.max_baud)
            if max_baud is None:
                return None
        else:
            min_baud = prompt_supported_baud("MINIMUM BAUD", options.min_baud)
            max_baud = prompt_supported_baud("MAXIMUM BAUD", options.max_baud)
        try:
            available_bauds(min_baud, max_baud)
        except ValueError as exc:
            print(f"BAUD RANGE ERROR: {terminal_text(exc)}")
            print("RE-ENTER BAUD RANGE.")
            continue
        return dataclasses.replace(options, min_baud=min_baud, max_baud=max_baud)
    return options


def configure_ports(options: ScanOptions) -> ScanOptions:
    """Prompt for validated input/output COM ports and fixed port bauds."""
    print("AVAILABLE BAUD RATES:")
    print(supported_baud_label())
    while prompt_loop_active():
        in_port = prompt_port("INPUT/TRANSMIT PORT", options.in_port)
        input_baud = prompt_supported_baud(
            "INPUT/TRANSMIT BAUD",
            options.input_baud,
        )
        out_port = prompt_port("OUTPUT/READ PORT", options.out_port)
        output_baud = prompt_supported_baud(
            "OUTPUT/READ BAUD",
            options.output_baud,
        )
        try:
            ensure_distinct_ports(in_port, out_port)
        except ValueError as exc:
            print(f"PORT ERROR: {terminal_text(exc)}")
            print("RE-ENTER COM PORTS.")
            continue
        return dataclasses.replace(
            options,
            in_port=in_port,
            out_port=out_port,
            input_baud=input_baud,
            output_baud=output_baud,
        )
    return options


def yes_no_text(value: bool) -> str:
    """Return a short operator-facing yes/no word."""
    return "YES" if value else "NO"


def print_scan_validate_setup(options: ScanOptions) -> None:
    """Print the scan/validate setup submenu with setting scope."""
    def setting_line(number: str, label: str, value: object, used_by: str) -> None:
        """Print one submenu setting with its workflow scope."""
        print(f"  {number} {label:<31} {value}")
        for line in wrapped_value_lines("    USED BY: ", used_by, REPORT_WIDTH):
            print(line)

    print()
    print_report_title("SCAN / VALIDATE SETUP")
    print("THESE SETTINGS DO NOT CHANGE PHASE 0.")
    print("PHASE 0 USES FIXED BAUD-LIVENESS PAYLOAD AND COUNT.")
    print(f"MINIMUM MESSAGE SIZE: {minimum_payload_size()} BYTES.")
    print(border_line(REPORT_WIDTH))
    setting_line(
        "1",
        "QUICK SCAN MESSAGE BYTES",
        options.payload_bytes,
        (
            "START 1 DISCOVERY; START 2 KNOWN-BAUD ASCII; "
            f"START 2 8-BIT BYTES=MAX({DEFAULT_EIGHT_BIT_PAYLOAD_BYTES}, THIS)."
        ),
    )
    setting_line(
        "2",
        "QUICK SCAN COUNT",
        options.bursts,
        "START 1 AUTOMATED DISCOVERY ONLY.",
    )
    setting_line(
        "3",
        "PAUSE ON CLEAN MATCH",
        yes_no_text(options.ask_on_top_match),
        "START 1 AUTOMATED DISCOVERY ONLY.",
    )
    setting_line(
        "4",
        "TOP-MATCH VERIFY",
        yes_no_text(options.auto_validate_top_matches),
        "START 1 AUTOMATED DISCOVERY AFTER SCAN.",
    )
    setting_line(
        "5",
        "TOP-MATCH VERIFY BYTES",
        options.validate_size_1_bytes,
        "ONLY WHEN 4=YES; START 1 TOP-MATCH VALIDATION.",
    )
    setting_line(
        "6",
        "FLOW TESTS",
        yes_no_text(options.auto_validate_flow_control),
        "START 1 FLOW SWEEP; START 2 FLOW TRANSFER AND BUFFER-FULL TESTS.",
    )
    setting_line(
        "7",
        "FLOW TRANSFER BYTES",
        options.flow_validate_size_bytes,
        "ONLY WHEN 6=YES; START 2 SHORT FLOW TRANSFER MATRIX.",
    )
    setting_line(
        "8",
        "BUFFER-FULL STRESS",
        yes_no_text(options.auto_stress_input_backpressure),
        "ONLY WHEN 6=YES; START 2 INPUT-SIDE BUFFER-FULL HANDSHAKE.",
    )
    setting_line(
        "9",
        "BUFFER-FULL BYTES",
        options.input_backpressure_stress_bytes,
        "ONLY WHEN 6=YES AND 8=YES; USE MORE THAN BUFFER CAPACITY.",
    )
    print("  0 RETURN TO MAIN MENU")
    print(border_line(REPORT_WIDTH))


def configure_payload(options: ScanOptions) -> ScanOptions:
    """Show the scan/validation setup submenu."""
    while prompt_loop_active():
        print_scan_validate_setup(options)
        try:
            choice = read_operator_input("ENTER SELECTION (1-9,0): ").lstrip("\ufeff").strip().lower()
        except EOFError:
            return options
        if choice in {"0", "m", "menu", "main", "return"}:
            return options
        if choice == "1":
            options = dataclasses.replace(
                options,
                payload_bytes=prompt_int(
                    "QUICK SCAN MESSAGE BYTES",
                    options.payload_bytes,
                    minimum=minimum_payload_size(),
                ),
            )
        elif choice == "2":
            options = dataclasses.replace(
                options,
                bursts=prompt_int(
                    "QUICK SCAN COUNT",
                    options.bursts,
                    minimum=1,
                ),
            )
        elif choice == "3":
            options = dataclasses.replace(
                options,
                ask_on_top_match=prompt_yes_no(
                    "PAUSE ON CLEAN MATCH",
                    options.ask_on_top_match,
                ),
            )
        elif choice == "4":
            options = dataclasses.replace(
                options,
                auto_validate_top_matches=prompt_yes_no(
                    "TOP-MATCH VERIFY",
                    options.auto_validate_top_matches,
                ),
            )
        elif choice == "5":
            options = dataclasses.replace(
                options,
                validate_size_1_bytes=prompt_int(
                    "TOP-MATCH VERIFY BYTES",
                    options.validate_size_1_bytes,
                    minimum=minimum_payload_size(),
                ),
            )
        elif choice == "6":
            options = dataclasses.replace(
                options,
                auto_validate_flow_control=prompt_yes_no(
                    "FLOW TESTS",
                    options.auto_validate_flow_control,
                ),
            )
        elif choice == "7":
            options = dataclasses.replace(
                options,
                flow_validate_size_bytes=prompt_int(
                    "FLOW TRANSFER BYTES",
                    options.flow_validate_size_bytes,
                    minimum=minimum_payload_size(),
                ),
            )
        elif choice == "8":
            options = dataclasses.replace(
                options,
                auto_stress_input_backpressure=prompt_yes_no(
                    "BUFFER-FULL STRESS",
                    options.auto_stress_input_backpressure,
                ),
            )
        elif choice == "9":
            options = dataclasses.replace(
                options,
                input_backpressure_stress_bytes=prompt_int(
                    "BUFFER-FULL BYTES",
                    options.input_backpressure_stress_bytes,
                    minimum=minimum_payload_size(),
                ),
            )
        else:
            print("ENTER 1-9 OR 0.")
    return options


def configure_timing(options: ScanOptions) -> ScanOptions:
    """Prompt for timing settings."""
    while prompt_loop_active():
        print("TIMING AND PER-TEST STALE DATA")
        print("TURBO APPLIES ONLY TO SCAN DISCOVERY.")
        print("VALIDATION USES CONSERVATIVE TIMING.")
        print("THE STALE-DATA LIMIT BELOW IS A QUICK PER-TEST CLEAR.")
        print("KNOWN-BAUD PURGES USE CALCULATED LONG LIMITS.")
        turbo_enabled = prompt_yes_no(
            "TURBO DISCOVERY MODE",
            options.turbo_discovery_enabled,
        )
        pre_drain_enabled = prompt_yes_no(
            "REJECT STALE OUTPUT BEFORE EACH TEST",
            not options.no_pre_drain,
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
            "MAX QUICK CLEAR TIME BEFORE TEST, SECONDS",
            options.pre_drain_timeout,
            0.0,
        )
        if pre_drain_enabled and pre_drain_timeout < pre_drain_quiet:
            print("TIMING ERROR: MAX QUICK CLEAR TIME MUST BE >= QUIET TIME.")
            print("RE-ENTER TIMING VALUES.")
            continue
        max_drain_bytes = prompt_int(
            "MAX QUICK OLD BYTES BEFORE STALE",
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
            no_pre_drain=not pre_drain_enabled,
            read_timeout=read_timeout,
            settle_ms=settle_ms,
            pre_drain_quiet=pre_drain_quiet,
            pre_drain_timeout=pre_drain_timeout,
            max_drain_bytes=max_drain_bytes,
            progress_interval=progress_interval,
        )
    return options


def write_payload_only(
    in_serial: Any,
    settings: SerialSettings,
    payload: ProbePayload,
    progress_interval: float,
    prefix: str,
    logger: logging.Logger,
) -> tuple[int, str | None, float]:
    """Write a payload without reading the output port.

    Args:
        in_serial: Open input-side serial object.
        settings: Transmit-side settings used for chunk sizing and estimates.
        payload: Payload to write.
        progress_interval: Minimum interval between progress messages.
        prefix: Progress/log prefix.
        logger: Logger for serial write errors.

    Returns:
        Tuple of bytes sent, optional serial I/O error string, and elapsed
        seconds. Programming errors such as `TypeError` are not swallowed.
    """
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
        in_serial.flush()
    except SERIAL_IO_ERRORS as exc:
        error = str(exc)
        logger.debug("%s write failed: %s", prefix, error)
    return bytes_sent, error, time.monotonic() - started


def read_until_quiet(
    out_serial: Any,
    settings: SerialSettings | DualSerialSettings,
    expected_bytes: int,
    read_timeout: float,
    progress_interval: float,
    prefix: str,
    logger: logging.Logger,
) -> tuple[bytes, str | None, float]:
    """Read output bytes until the line goes quiet.

    The absolute deadline scales with estimated output-side line time so low
    baud validation has room to drain after a large payload.
    """
    started = time.monotonic()
    received = bytearray()
    last_data_time = time.monotonic()
    read_timeout = max(read_timeout, 0.1)
    max_seconds = max(
        estimated_receive_seconds(settings, expected_bytes) * 2.0 + read_timeout + 10.0,
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
        except SERIAL_IO_ERRORS as exc:
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


def read_serial_until_quiet(
    serial_port: Any,
    settings: SerialSettings | DualSerialSettings,
    expected_bytes: int,
    read_timeout: float,
    progress_interval: float,
    prefix: str,
    logger: logging.Logger,
    direction_label: str,
) -> tuple[bytes, str | None, float]:
    """Read bytes from a named direction until the line goes quiet.

    This is the directional variant used by ETX/ACK probing, where the same
    physical serial objects are read in both forward and reverse-path tests.
    """
    started = time.monotonic()
    received = bytearray()
    last_data_time = time.monotonic()
    read_timeout = max(read_timeout, 0.1)
    max_seconds = max(
        estimated_receive_seconds(settings, expected_bytes) * 2.0 + read_timeout + 5.0,
        read_timeout + 5.0,
    )
    deadline = started + max_seconds
    next_progress_at = time.monotonic() + max(progress_interval, 0.1)
    error: str | None = None
    console_progress(
        f"{prefix}: READ {direction_label.upper()} UNTIL QUIET FOR {read_timeout:.1f}S"
    )
    while True:
        try:
            waiting = getattr(serial_port, "in_waiting", 0)
            read_size = min(max(int(waiting), 1), 4096)
            chunk = serial_port.read(read_size)
        except SERIAL_IO_ERRORS as exc:
            error = str(exc)
            logger.debug("%s %s read failed: %s", prefix, direction_label, error)
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
                f"{prefix}: READING {direction_label.upper()} "
                f"RECEIVED={len(received)} BYTES, "
                f"QUIET={silence:.1f}/{read_timeout:.1f}S"
            )
            next_progress_at = now + max(progress_interval, 0.1)

        if now >= deadline:
            error = (
                f"{direction_label.upper()} READ STOPPED BEFORE QUIET "
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
    """Read output that arrives during a fixed observation window."""
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
        except SERIAL_IO_ERRORS as exc:
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


def flow_control_hold_byte_limit(payload_bytes: int) -> int:
    """Return tolerated in-flight bytes while a flow-control hold is active."""
    return max(16, min(256, payload_bytes // 100))


def flow_validation_indicator(status: str, score: float, error: str | None) -> str:
    """Return a compact indicator for a flow-control validation status."""
    if error or status == "error":
        return "ERROR"
    if status in {"validated", "backpressure-proven"}:
        return "PASS"
    if status in {
        "transfer-good",
        "paused-partial-transfer",
        "backpressure-partial",
    }:
        return "GOOD"
    if status in {"stress-skipped", "stress-invalid"}:
        return "SKIP"
    if status == "no-backpressure":
        return "FAIL"
    if status == "transfer-partial":
        return "PARTIAL"
    if status == "no-pause":
        return "FAIL"
    if score >= 90.0:
        return "GOOD"
    if score >= 50.0:
        return "PARTIAL"
    return "FAIL"


def dual_flow_control_code(settings: DualSerialSettings) -> str:
    """Return a compact input/output flow-control code for validation tables."""
    def side_code(flow_control: str) -> str:
        """Return OFF for disabled flow, otherwise the compact flow code."""
        if flow_control == "none":
            return "OFF"
        return flow_control_code(flow_control)

    return (
        f"{side_code(settings.input_settings.flow_control)}/"
        f"{side_code(settings.output_settings.flow_control)}"
    )


def flow_control_setting_label(settings: SerialSettings | DualSerialSettings) -> str:
    """Return a report label for one same-frame or dual flow setting."""
    if isinstance(settings, DualSerialSettings):
        return (
            f"IN {flow_control_name(settings.input_settings.flow_control)} >> "
            f"OUT {flow_control_name(settings.output_settings.flow_control)}"
        )
    return flow_control_name(settings.flow_control)


def flow_validation_frame_setting_label(
    settings: SerialSettings | DualSerialSettings,
) -> str:
    """Return the tested frame/pair label for flow-validation summaries."""
    if isinstance(settings, DualSerialSettings):
        return settings.label()
    return f"{settings.baud} {frame_label(settings)}"


def flow_validation_result_from_candidate(
    flow_control: str,
    method: str,
    result: CandidateResult,
    payload: ProbePayload,
) -> FlowControlValidationResult:
    """Convert a normal transfer candidate result into a flow-validation row.

    Transfer-only rows can prove that bytes move under a flow setting, but they
    do not prove that a device honors pause/resume or input-side backpressure.
    """
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
        status = "transfer-good"
        if isinstance(result.settings, DualSerialSettings):
            reason = "CLEAN DUAL-FLOW TRANSFER; HANDSHAKE HOLD/RELEASE NOT PROVEN."
        elif flow_control == "none":
            reason = "CLEAN TRANSFER; NO FLOW HANDSHAKE WAS REQUESTED."
        else:
            reason = "CLEAN TRANSFER ONLY; FLOW NOT OBSERVED OR PROVEN."
    elif result.score >= 90.0:
        status = "transfer-good"
        if isinstance(result.settings, DualSerialSettings):
            reason = "DUAL-FLOW TRANSFER WAS GOOD BUT NOT BYTE-PERFECT."
        else:
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
    """Assert a receive-side hold for one flow-control mode.

    XON/XOFF sends XOFF, RTS/CTS lowers RTS, and DSR/DTR lowers DTR on the
    receive-side adapter.

    Raises:
        ValueError: If `flow_control` cannot express an output-side hold.
    """
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
    """Release a receive-side hold for one flow-control mode.

    Raises:
        ValueError: If `flow_control` cannot express an output-side hold.
    """
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
    settings: SerialSettings | DualSerialSettings,
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
    """Validate transfer compatibility for one same-frame flow mode.

    Notes:
        A clean transfer with flow enabled is useful evidence, but only
        `run_flow_control_pause_validation` observes an output-side hold and
        release.
    """
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


def run_dual_flow_control_transfer_validation(
    serial_module: Any,
    index: int,
    total: int,
    settings: DualSerialSettings,
    options: ScanOptions,
    payload: ProbePayload,
    logger: logging.Logger,
) -> FlowControlValidationResult:
    """Validate transfer compatibility for one dual input/output flow pair."""
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
    result = run_dual_candidate(
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
        dual_flow_control_code(settings),
        "dual-transfer",
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
    """Validate output-side pause/release handshake behavior.

    The receive-side adapter asserts a hold before the input-side payload is
    written. A passing row requires little output while held, a clean drain after
    release, and no serial error.
    """
    started = time.monotonic()
    flow_control = settings.flow_control
    payload = payload_for_trial(
        template=payload,
        settings=settings,
        candidate_index=index,
        burst_index=1,
        run_id=options.run_id or make_run_id("F"),
        switch_hash=switch_note_hash(options.switch_note),
    )
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
    # WHY: The output port is the control surface for the hold. Its own flow
    # mode is disabled so XOFF or modem-line changes are produced deliberately by
    # this test rather than by pyserial's automatic handshake handling.
    read_timeout = max(options.read_timeout, FLOW_VALIDATE_READ_TIMEOUT)
    hold_applied = False
    payload_bytes = payload.byte_count
    pre_drain_timeout, pre_drain_quiet = low_baud_pre_drain_values(
        settings,
        payload_bytes,
        max(options.pre_drain_timeout, 2.0),
        options.pre_drain_quiet,
    )
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

                if not options.no_pre_drain:
                    drain = drain_output_until_quiet(
                        out_serial=out_serial,
                        quiet_seconds=pre_drain_quiet,
                        max_seconds=pre_drain_timeout,
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

                before_hold_lines = modem_line_snapshot(out_serial)
                logger.debug(
                    "%s modem lines before hold: %s",
                    prefix,
                    before_hold_lines,
                )
                console_progress(
                    f"{prefix}: LINES BEFORE HOLD "
                    f"{modem_line_snapshot_label(before_hold_lines)}"
                )
                console_progress(f"{prefix}: ASSERT {flow_control.upper()} HOLD")
                # WHY: Holding before the write separates "can transfer with this
                # flow option" from "device honors an output-side pause."
                apply_flow_control_hold(out_serial, flow_control)
                hold_applied = True
                time.sleep(FLOW_VALIDATE_RELEASE_SETTLE_SECONDS)
                after_hold_lines = modem_line_snapshot(out_serial)
                logger.debug(
                    "%s modem lines after hold: %s",
                    prefix,
                    after_hold_lines,
                )
                console_progress(
                    f"{prefix}: LINES AFTER HOLD "
                    f"{modem_line_snapshot_label(after_hold_lines)}"
                )

                bytes_sent, write_error, _write_elapsed = write_payload_only(
                    in_serial=in_serial,
                    settings=settings,
                    payload=payload,
                    progress_interval=options.progress_interval,
                    prefix=prefix,
                    logger=logger,
                )
                after_write_lines = modem_line_snapshot(out_serial)
                logger.debug(
                    "%s modem lines after write: %s",
                    prefix,
                    after_write_lines,
                )
                console_progress(
                    f"{prefix}: LINES AFTER WRITE "
                    f"{modem_line_snapshot_label(after_write_lines)}"
                )
                held_bytes, hold_error, _hold_elapsed = read_for_fixed_window(
                    out_serial=out_serial,
                    seconds=FLOW_VALIDATE_HOLD_SECONDS,
                    progress_interval=options.progress_interval,
                    prefix=prefix,
                    logger=logger,
                )
                before_release_lines = modem_line_snapshot(out_serial)
                logger.debug(
                    "%s modem lines before release: %s",
                    prefix,
                    before_release_lines,
                )
                console_progress(
                    f"{prefix}: LINES BEFORE RELEASE "
                    f"{modem_line_snapshot_label(before_release_lines)}"
                )
                console_progress(f"{prefix}: RELEASE {flow_control.upper()} HOLD")
                release_flow_control_hold(out_serial, flow_control)
                hold_applied = False
                time.sleep(FLOW_VALIDATE_RELEASE_SETTLE_SECONDS)
                after_release_lines = modem_line_snapshot(out_serial)
                logger.debug(
                    "%s modem lines after release: %s",
                    prefix,
                    after_release_lines,
                )
                console_progress(
                    f"{prefix}: LINES AFTER RELEASE "
                    f"{modem_line_snapshot_label(after_release_lines)}"
                )

                release_bytes, read_error, _read_elapsed = read_until_quiet(
                    out_serial=out_serial,
                    settings=settings,
                    expected_bytes=payload_bytes,
                    read_timeout=read_timeout,
                    progress_interval=options.progress_interval,
                    prefix=prefix,
                    logger=logger,
                )
    except SERIAL_IO_ERRORS as exc:
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
            except SERIAL_IO_ERRORS:
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

    modem_observations = (
        modem_line_observation("BEFORE HOLD", before_hold_lines),
        modem_line_observation("AFTER HOLD", after_hold_lines),
        modem_line_observation("AFTER WRITE", after_write_lines),
        modem_line_observation("BEFORE RELEASE", before_release_lines),
        modem_line_observation("AFTER RELEASE", after_release_lines),
    )
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
        modem_line_observations=modem_observations,
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
    setting_label = fit_terminal_line(result.settings.label(), 32)
    return fit_terminal_line(
        f"FLOW [{index:02d}/{total:02d}] {result.indicator:<7} "
        f"{setting_label:32s} "
        f"SENT={result.bytes_sent:6d} READ={result.bytes_received:6d} "
        f"HELD={result.bytes_seen_while_held:5d} "
        f"SCORE={result.score:6.2f} {terminal_text(result.reason)}",
        TERMINAL_COLUMNS,
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
    stress_results = [
        result for result in results if result.method == INPUT_BACKPRESSURE_METHOD
    ]
    proven_backpressure = [
        result for result in stress_results if result.status == "backpressure-proven"
    ]
    if len(proven_backpressure) == 1:
        return (
            "INPUT BACKPRESSURE PROVEN: "
            f"{input_backpressure_result_label(proven_backpressure[0])}."
        )
    if len(proven_backpressure) > 1:
        flows = ", ".join(
            input_backpressure_result_label(result)
            for result in proven_backpressure
        )
        return f"MULTIPLE INPUT BACKPRESSURE MODES: {flows}."
    if stress_results and all(result.status == "stress-skipped" for result in stress_results):
        return "INPUT BACKPRESSURE STRESS SKIPPED."
    if stress_results and any(result.status == "no-backpressure" for result in stress_results):
        return "INPUT BACKPRESSURE NOT OBSERVED."
    if stress_results:
        return "INPUT BACKPRESSURE NOT PROVEN."
    if any(result.flow_control == "none" for result in proven):
        return "CLEAN TRANSFER WITHOUT HANDSHAKE; NO HANDSHAKE MODE PROVEN."
    clean_dual = [
        result
        for result in results
        if isinstance(result.settings, DualSerialSettings)
        and result.status == "transfer-good"
        and result.score >= 99.0
        and result.bytes_sent == result.bytes_received
        and result.metrics.missing_bytes == 0
        and result.metrics.extra_bytes == 0
    ]
    if len(clean_dual) == 1:
        return (
            "CLEAN DUAL FLOW TRANSFER: "
            f"{flow_control_setting_label(clean_dual[0].settings)}; "
            "HANDSHAKE NOT PROVEN."
        )
    if len(clean_dual) > 1:
        flows = compact_label_list(
            [flow_control_setting_label(result.settings) for result in clean_dual],
            limit=4,
        )
        return f"MULTIPLE CLEAN DUAL FLOW TRANSFERS: {flows}; HANDSHAKE NOT PROVEN."
    if any(
        result.flow_control == "none" and result.status == "transfer-good"
        for result in results
    ):
        return "CLEAN TRANSFER WITHOUT HANDSHAKE; NO HANDSHAKE MODE PROVEN."
    if results:
        return "NO FLOW CONTROL MODE VALIDATED."
    return "FLOW CONTROL VALIDATION WAS NOT RUN."


def input_backpressure_result_label(result: FlowControlValidationResult) -> str:
    """Return the observed input-backpressure label for summary text."""
    for reason in INPUT_BACKPRESSURE_SIGNAL_REASONS:
        if result.reason.startswith(reason):
            return reason
    return flow_control_name(result.flow_control)


def input_backpressure_observation_label(
    result: FlowControlValidationResult,
) -> str | None:
    """Return operator-facing observed input-flow evidence for one stress row."""
    if result.status not in {"backpressure-proven", "backpressure-partial"}:
        return None
    if result.reason.startswith("DCD DROPPED"):
        return "DTR-DCD OBSERVED"
    if result.reason.startswith("DSR DROPPED"):
        return "DTR-DSR OBSERVED"
    if result.reason.startswith("CTS DROPPED"):
        return "CTS OBSERVED"
    if result.reason.startswith("WRITE STALLED") and result.flow_control == "xon/xoff":
        return "XON/XOFF OBSERVED"
    if result.reason.startswith("WRITE STALLED"):
        return f"{flow_control_name(result.flow_control)} WRITE STALL OBSERVED"
    return None


def bank2_flow_control_observed_summary(
    results: Sequence[FlowControlValidationResult],
) -> str:
    """Return observed flow behavior without treating clean transfer as proof."""
    stress_labels: list[str] = []
    for result in flow_input_backpressure_results(results):
        label = input_backpressure_observation_label(result)
        if label and label not in stress_labels:
            stress_labels.append(label)
    if stress_labels:
        return compact_label_list(stress_labels, limit=4)

    hold_labels = [
        flow_control_name(result.flow_control)
        for result in flow_hold_release_results(results)
        if result.status == "validated" and result.flow_control != "none"
    ]
    if hold_labels:
        return f"OUTPUT HOLD ONLY: {compact_label_list(hold_labels, limit=4)}"

    if flow_transfer_matrix_results(results):
        return "FLOW TRANSFER MATRIX ONLY; HANDSHAKE NOT OBSERVED."
    if results:
        return "NOT OBSERVED."
    return "NOT RUN."


def bank2_byte_transfer_summary(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
) -> str:
    """Return best observed byte-transfer evidence separate from flow evidence."""
    target = best_bank2_followup_target(ascii_results, eight_bit_results)
    if target is None:
        return "(NONE)"
    width = (
        "8-BIT CLEAN"
        if bank2_eight_bit_clean_results(eight_bit_results)
        else "ASCII CLEAN ONLY"
    )
    return f"{width}; BEST OBSERVED {frame_or_pair_label(target.settings)}"


def flow_validation_modem_line_report_lines(
    results: Sequence[FlowControlValidationResult],
) -> list[str]:
    """Return modem-line snapshots captured during flow validation."""
    rows: list[str] = []
    for result in results:
        for event, snapshot in result.modem_line_observations:
            flow_label = fit_terminal_line(result.flow_control.upper(), 8)
            method = fit_terminal_line(terminal_text(result.method), 16)
            event_label = fit_terminal_line(event, 14)
            rows.append(
                f"{flow_label:<8} {method:<16} {event_label:<14} "
                f"{modem_line_snapshot_label(snapshot)}"
            )
    if not rows:
        return []
    return [
        "MODEM-LINE BEHAVIOR:",
        f"{'FLOW':<8} {'METHOD':<16} {'EVENT':<14} LINES",
        border_line(REPORT_WIDTH),
        *rows,
    ]


FLOW_VALIDATION_METHOD_WIDTH = 16
FLOW_VALIDATION_TABLE_HEADER = (
    f"{'FLOW':<8} {'RESULT':<7} {'SCORE':>5} {'SENT':>6} {'READ':>6} "
    f"{'HELD':>5} {'METHOD':<{FLOW_VALIDATION_METHOD_WIDTH}} REASON"
)


def flow_validation_table_row_lines(
    result: FlowControlValidationResult,
) -> list[str]:
    """Return one flow-validation table row, wrapping reason text at 80 columns."""
    method = fit_terminal_line(terminal_text(result.method), FLOW_VALIDATION_METHOD_WIDTH)
    flow_label = fit_terminal_line(result.flow_control.upper(), 8)
    prefix = (
        f"{flow_label:<8} "
        f"{result.indicator:<7} "
        f"{result.score:>5.1f} "
        f"{result.bytes_sent:>6} "
        f"{result.bytes_received:>6} "
        f"{result.bytes_seen_while_held:>5} "
        f"{method:<{FLOW_VALIDATION_METHOD_WIDTH}} "
    )
    reason_width = max(1, REPORT_WIDTH - len(prefix))
    reason_lines = textwrap.wrap(
        terminal_text(result.reason),
        width=reason_width,
        break_long_words=True,
    ) or [""]
    return [
        prefix + reason_lines[0],
        *(" " * len(prefix) + line for line in reason_lines[1:]),
    ]


def flow_validation_note_lines(
    results: Sequence[FlowControlValidationResult],
) -> list[str]:
    """Return explanatory report notes for the flow-validation method used."""
    if results and all(result.method == INPUT_BACKPRESSURE_METHOD for result in results):
        return [
            "NOTE: BUFFER-FULL STRESS HOLDS OUTPUT SO THE INPUT BUFFER CAN FILL.",
            "NOTE: PASS REQUIRES INPUT THROTTLE AND CLEAN DRAIN AFTER RELEASE.",
        ]
    if any(result.method == "dual-transfer" for result in results):
        return [
            "NOTE: DUAL-FLOW SWEEP PROVES TRANSFER COMPATIBILITY ONLY.",
            "NOTE: HOLD/RELEASE HANDSHAKE IS NOT PROVEN FOR DUAL FRAME PAIRS.",
        ]
    return [
        "NOTE: OUTPUT-SIDE HOLD/RELEASE PROVES ONLY OBSERVED PAUSE/RESUME.",
        "NOTE: INPUT-SIDE BACKPRESSURE IS NOT PROVEN BY THIS FLOW VALIDATION.",
    ]


def flow_validation_report_lines(
    results: Sequence[FlowControlValidationResult],
) -> list[str]:
    """Return compact flow-control validation lines for the text report."""
    lines = [
        "FLOW CONTROL VALIDATION:",
    ]
    lines.extend(
        wrapped_value_lines(
            "FINDING:         ",
            flow_control_validation_recommendation(results),
            REPORT_WIDTH,
        )
    )
    lines.extend(["", FLOW_VALIDATION_TABLE_HEADER, border_line(REPORT_WIDTH)])
    for result in results:
        lines.extend(flow_validation_table_row_lines(result))
    modem_lines = flow_validation_modem_line_report_lines(results)
    if modem_lines:
        lines.append("")
        lines.extend(modem_lines)
    lines.extend(flow_validation_note_lines(results))
    lines.append(border_line(REPORT_WIDTH))
    return lines


def print_flow_control_validation_report(
    frame: SerialSettings | DualSerialSettings,
    results: Sequence[FlowControlValidationResult],
) -> None:
    """Print the final flow-control validation report."""
    print()
    print_report_title("FLOW CONTROL VALIDATION RESULTS")
    print_wrapped_value(
        "  FRAME SETTING:        ",
        flow_validation_frame_setting_label(frame),
    )
    print(f"  TEST BYTES:           {max((result.bytes_sent for result in results), default=0)}")
    print_wrapped_value(
        "  FINDING:              ",
        flow_control_validation_recommendation(results),
    )
    print(border_line(REPORT_WIDTH))
    print(FLOW_VALIDATION_TABLE_HEADER)
    print(border_line(REPORT_WIDTH))
    for result in results:
        for line in flow_validation_table_row_lines(result):
            print(line)
    modem_lines = flow_validation_modem_line_report_lines(results)
    if modem_lines:
        print()
        for line in modem_lines:
            print(line)
    for note in flow_validation_note_lines(results):
        print(note)
    print(border_line(REPORT_WIDTH))


def run_flow_control_validation(
    serial_module: Any,
    options: ScanOptions,
    results: Sequence[CandidateResult],
    validation_results: Sequence[CandidateResult],
    logger: logging.Logger,
) -> tuple[list[FlowControlValidationResult], str | None]:
    """Run post-scan flow validation for all modes on the best same-side frame.

    Returns:
        A list of validation rows plus an optional operator-break action.
    """
    frame = select_flow_validation_frame(results, validation_results)
    if frame is None:
        print()
        print_report_title("FLOW CONTROL VALIDATION")
        print("SKIPPED: NO RECOMMENDABLE FRAME SETTING WAS FOUND.")
        print(border_line(REPORT_WIDTH))
        logger.info("flow control validation skipped: no recommendable frame")
        return [], None

    run_known_output_purges(
        serial_module=serial_module,
        options=options,
        logger=logger,
        settings_list=[frame],
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


def run_dual_flow_control_validation(
    serial_module: Any,
    options: ScanOptions,
    target: CandidateResult,
    logger: logging.Logger,
) -> tuple[list[FlowControlValidationResult], str | None]:
    """Run known-baud flow validation for an independent frame pair.

    The dual sweep tests the 16 input/output flow combinations as transfer
    checks only. Output hold/release is skipped for asymmetric pairs because the
    same serial frame is not available on both sides.
    """
    if not isinstance(target.settings, DualSerialSettings):
        raise ValueError("dual flow validation requires dual serial settings")

    frame = DualSerialSettings(
        input_settings=dataclasses.replace(
            target.settings.input_settings,
            flow_control="none",
        ),
        output_settings=dataclasses.replace(
            target.settings.output_settings,
            flow_control="none",
        ),
    )
    run_known_output_purges(
        serial_module=serial_module,
        options=options,
        logger=logger,
        settings_list=[frame.output_settings],
        reason="CLEAR VALIDATION DATA BEFORE DUAL FLOW-CONTROL TESTS.",
    )
    payload = generate_payload(options.flow_validate_size_bytes)
    settings_list = dual_flow_candidates_for_frame(frame)
    print()
    print_report_title("DUAL FLOW CONTROL VALIDATION")
    print_wrapped_value("FRAME PAIR: ", frame.label())
    print(f"TEST: {payload.byte_count} BYTES; INPUT/OUTPUT FLOW COMBINATIONS.")
    print("THIS SWEEP PROVES TRANSFER COMPATIBILITY, NOT HOLD/RELEASE HANDSHAKE.")
    print(border_line(REPORT_WIDTH))
    logger.info(
        "dual flow control validation started frame=%s payload=%s candidates=%s",
        frame.label(),
        payload.byte_count,
        len(settings_list),
    )

    flow_results: list[FlowControlValidationResult] = []
    for index, settings in enumerate(settings_list, start=1):
        while True:
            try:
                result = run_dual_flow_control_transfer_validation(
                    serial_module=serial_module,
                    index=index,
                    total=len(settings_list),
                    settings=settings,
                    options=options,
                    payload=payload,
                    logger=logger,
                )
            except KeyboardInterrupt:
                action = prompt_operator_break_action("DUAL FLOW VALIDATION")
                if action == "resume":
                    logger.info(
                        "operator resumed dual flow validation at %s/%s %s",
                        index,
                        len(settings_list),
                        settings.label(),
                    )
                    continue
                logger.info(
                    "operator break during dual flow validation; action=%s completed=%s/%s",
                    action,
                    len(flow_results),
                    len(settings_list),
                )
                print_flow_control_validation_report(frame, flow_results)
                return flow_results, action
            flow_results.append(result)
            print(format_flow_validation_progress(result, index, len(settings_list)))
            logger.info(
                "dual flow validation %s: indicator=%s status=%s score=%.2f sent=%s read=%s reason=%s error=%s",
                settings.label(),
                result.indicator,
                result.status,
                result.score,
                result.bytes_sent,
                result.bytes_received,
                result.reason,
                result.error,
            )
            break

    print_flow_control_validation_report(frame, flow_results)
    logger.info(
        "dual flow control validation completed: %s",
        flow_control_validation_recommendation(flow_results),
    )
    return flow_results, None


def flow_pause_holds_output(result: FlowControlValidationResult) -> bool:
    """Return True when a hold/release row is good enough to fill the buffer."""
    if result.method not in {"xoff-xon-pause", "rts-hold-release", "dtr-hold-release"}:
        return False
    if result.error or result.status in {
        "error",
        "no-pause",
        "no-data",
        "partial-write",
        "stale-output",
    }:
        return False
    if result.bytes_sent <= 0 or result.bytes_received <= 0:
        return False
    return result.bytes_seen_while_held <= flow_control_hold_byte_limit(
        max(result.bytes_sent, 1)
    )


def select_backpressure_output_hold(
    hold_results: Sequence[FlowControlValidationResult],
) -> FlowControlValidationResult | None:
    """Return the best proven output hold mode for buffer-fill stress.

    Validated rows are preferred, then rows with fewer bytes leaked while held.
    XON/XOFF is preferred over modem-line holds when evidence is otherwise tied.
    """
    candidates = [result for result in hold_results if flow_pause_holds_output(result)]
    if not candidates:
        return None
    flow_rank = {"xon/xoff": 0, "dsr/dtr": 1, "rts/cts": 2}
    return sorted(
        candidates,
        key=lambda result: (
            0 if result.status == "validated" else 1,
            result.bytes_seen_while_held,
            flow_rank.get(result.flow_control, 9),
        ),
    )[0]


def input_backpressure_settings_for_target(
    target: CandidateResult,
) -> DualSerialSettings | None:
    """Return concrete input/output frame settings for input stress."""
    if isinstance(target.settings, DualSerialSettings):
        return DualSerialSettings(
            dataclasses.replace(target.settings.input_settings, flow_control="none"),
            dataclasses.replace(target.settings.output_settings, flow_control="none"),
        )
    if isinstance(target.settings, SerialSettings):
        frame = dataclasses.replace(target.settings, flow_control="none")
        return DualSerialSettings(frame, frame)
    return None


def input_backpressure_skip_result(
    settings: SerialSettings | DualSerialSettings,
    reason: str,
) -> FlowControlValidationResult:
    """Return a report row for an input stress test that could not run."""
    empty_score = score_received(b"", b"")
    return FlowControlValidationResult(
        flow_control="stress",
        method=INPUT_BACKPRESSURE_METHOD,
        settings=settings,
        bytes_sent=0,
        bytes_received=0,
        bytes_seen_while_held=0,
        score=0.0,
        indicator="SKIP",
        status="stress-skipped",
        reason=reason,
        error=None,
        elapsed_sec=0.0,
        metrics=empty_score.metrics,
    )


def release_input_backpressure_hold(
    out_serial: Any,
    output_hold_flow: str,
    hold_released: threading.Event,
) -> None:
    """Release the output hold once input backpressure has been observed."""
    if hold_released.is_set():
        return
    release_flow_control_hold(out_serial, output_hold_flow)
    hold_released.set()


def modem_line_drop_reason(
    before_snapshot: dict[str, bool],
    line_snapshot: dict[str, bool],
    line_name: str,
    line_label: str,
) -> str | None:
    """Return a modem-status drop reason when a line changed from high to low."""
    if before_snapshot.get(line_name, True) and not line_snapshot.get(line_name, True):
        return f"{line_label} DROPPED"
    return None


def input_backpressure_release_reason(
    input_flow: str,
    before_snapshot: dict[str, bool],
    line_snapshot: dict[str, bool],
    stalled: bool,
) -> str | None:
    """Return the backpressure reason visible at the input adapter.

    For DSR/DTR-style printer pacing, DCD is checked before DSR/CTS because many
    adapters expose printer DTR as carrier-detect rather than as DSR.
    """
    if input_flow == "rts/cts":
        reason = modem_line_drop_reason(before_snapshot, line_snapshot, "cts", "CTS")
        if reason:
            return reason
    if input_flow == "dsr/dtr":
        for line_name, line_label in (
            ("cd", "DCD"),
            ("dsr", "DSR"),
            ("cts", "CTS"),
        ):
            reason = modem_line_drop_reason(
                before_snapshot,
                line_snapshot,
                line_name,
                line_label,
            )
            if reason:
                return reason
    if stalled:
        return "WRITE STALLED"
    return None


def input_backpressure_observed(reason: str | None) -> bool:
    """Return True when a release reason proves input-side throttle was seen."""
    return reason in INPUT_BACKPRESSURE_OBSERVED_REASONS


def run_input_backpressure_stress(
    serial_module: Any,
    index: int,
    total: int,
    frame: DualSerialSettings,
    input_flow: str,
    output_hold_flow: str,
    options: ScanOptions,
    payload: ProbePayload,
    logger: logging.Logger,
) -> FlowControlValidationResult:
    """Fill the buffer with output held and watch input-side flow control.

    The test writes a large payload while output is held, monitors input-side
    modem lines and write progress for throttle evidence, releases output, and
    then scores the drained bytes.
    """
    started = time.monotonic()
    input_settings = dataclasses.replace(
        frame.input_settings,
        flow_control=input_flow,
    )
    output_settings = dataclasses.replace(
        frame.output_settings,
        flow_control="none",
    )
    settings = DualSerialSettings(input_settings, output_settings)
    payload = payload_for_trial(
        template=payload,
        settings=settings,
        candidate_index=index,
        burst_index=1,
        run_id=options.run_id or make_run_id("B"),
        switch_hash=switch_note_hash(options.switch_note),
    )
    expected = payload.data
    payload_bytes = payload.byte_count
    read_timeout = max(options.read_timeout, FLOW_VALIDATE_READ_TIMEOUT)
    chunk_size = write_chunk_size(input_settings)
    hold_limit = flow_control_hold_byte_limit(payload_bytes)
    stall_seconds = max(
        INPUT_BACKPRESSURE_STALL_SECONDS,
        estimated_transmit_seconds(input_settings, chunk_size) * 3.0,
    )
    prefix = (
        f"[BFULL {index:02d}/{total:02d} IN={flow_control_code(input_flow)} "
        f"HOLD={flow_control_code(output_hold_flow)}]"
    )
    console_progress(border_line(PROGRESS_WIDTH))
    console_progress(
        bordered_text(
            f"INPUT BUFFER-FULL {index}/{total} {input_flow.upper()}",
            PROGRESS_WIDTH,
        )
    )
    console_progress(border_line(PROGRESS_WIDTH))
    console_progress(
        f"{prefix}: HOLD OUTPUT WITH {output_hold_flow.upper()}; "
        f"SEND {payload_bytes} BYTES"
    )

    received = bytearray()
    received_lock = threading.Lock()
    write_lock = threading.Lock()
    stop_event = threading.Event()
    writer_done = threading.Event()
    hold_released = threading.Event()
    reader_errors: list[str] = []
    bytes_sent = 0
    held_bytes_seen = 0
    write_error: str | None = None
    last_write_time = time.monotonic()
    last_data_time = time.monotonic()
    release_reason: str | None = None
    release_sent = 0
    before_input_lines: dict[str, bool] = {}
    after_input_lines: dict[str, bool] = {}
    hold_applied = False

    def reader() -> None:
        """Read output continuously so the PC-side receive buffer cannot fill."""
        nonlocal held_bytes_seen, last_data_time
        while not stop_event.is_set():
            try:
                waiting = getattr(out_serial, "in_waiting", 0)  # type: ignore[name-defined]
                read_size = min(max(int(waiting), 1), 4096)
                chunk = out_serial.read(read_size)  # type: ignore[name-defined]
            except SERIAL_IO_ERRORS as exc:
                reader_errors.append(str(exc))
                stop_event.set()
                break

            now = time.monotonic()
            if chunk:
                with received_lock:
                    received.extend(chunk)
                    if not hold_released.is_set():
                        held_bytes_seen += len(chunk)
                last_data_time = now
                continue

            if (
                writer_done.is_set()
                and hold_released.is_set()
                and (now - last_data_time) >= read_timeout
            ):
                stop_event.set()
                break

    def writer() -> None:
        """Write the stress payload and leave release decisions to the monitor."""
        nonlocal bytes_sent, write_error, last_write_time
        try:
            while bytes_sent < len(expected):
                chunk = expected[bytes_sent : bytes_sent + chunk_size]
                written = in_serial.write(chunk)  # type: ignore[name-defined]
                if written is None:
                    written = len(chunk)
                if written <= 0:
                    raise RuntimeError("serial write returned zero bytes")
                with write_lock:
                    bytes_sent += int(written)
                    last_write_time = time.monotonic()
            in_serial.flush()  # type: ignore[name-defined]
        except SERIAL_IO_ERRORS as exc:
            write_error = str(exc)
            logger.debug("%s write failed: %s", prefix, write_error)
        finally:
            writer_done.set()

    try:
        with open_serial_port(
            serial_module,
            options.out_port,
            output_settings,
            read_timeout,
        ) as out_serial:
            with open_serial_port(
                serial_module,
                options.in_port,
                input_settings,
                read_timeout,
            ) as in_serial:
                reset_serial_buffers(out_serial)
                reset_serial_buffers(in_serial)
                time.sleep(options.settle_ms / 1000.0)

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
                            "OUTPUT DID NOT GO QUIET BEFORE BUFFER-FULL STRESS "
                            f"(REASON={drain.reason.upper()}, "
                            f"CLEARED={drain.bytes_drained})"
                        )
                        if drain.error:
                            error = f"{error}: {drain.error}"
                        return input_backpressure_skip_result(settings, error)

                before_input_lines = modem_line_snapshot(in_serial)
                console_progress(
                    f"{prefix}: INPUT LINES BEFORE "
                    f"{modem_line_snapshot_label(before_input_lines)}"
                )
                apply_flow_control_hold(out_serial, output_hold_flow)
                hold_applied = True
                time.sleep(FLOW_VALIDATE_RELEASE_SETTLE_SECONDS)

                reader_thread = threading.Thread(
                    target=reader,
                    name="serial-probe-backpressure-reader",
                    daemon=True,
                )
                writer_thread = threading.Thread(
                    target=writer,
                    name="serial-probe-backpressure-writer",
                    daemon=True,
                )
                reader_thread.start()
                writer_thread.start()

                last_progress_bytes = 0
                last_progress_time = time.monotonic()
                next_progress_at = time.monotonic() + max(options.progress_interval, 0.1)
                max_hold_seconds = max(
                    estimated_transmit_seconds(input_settings, payload_bytes)
                    + read_timeout
                    + 10.0,
                    10.0,
                )
                hold_deadline = time.monotonic() + max_hold_seconds
                while writer_thread.is_alive() and not hold_released.is_set():
                    time.sleep(INPUT_BACKPRESSURE_MONITOR_SECONDS)
                    now = time.monotonic()
                    with write_lock:
                        sent_snapshot = bytes_sent
                        last_write_snapshot = last_write_time
                    if sent_snapshot != last_progress_bytes:
                        last_progress_bytes = sent_snapshot
                        last_progress_time = now
                    stalled = (
                        sent_snapshot > 0
                        and (now - max(last_progress_time, last_write_snapshot))
                        >= stall_seconds
                    )
                    line_snapshot = modem_line_snapshot(in_serial)
                    reason = input_backpressure_release_reason(
                        input_flow,
                        before_input_lines,
                        line_snapshot,
                        stalled,
                    )
                    if reason:
                        release_reason = reason
                        release_sent = sent_snapshot
                        after_input_lines = line_snapshot
                        console_progress(
                            f"{prefix}: {reason}; RELEASE OUTPUT HOLD "
                            f"AFTER {sent_snapshot} BYTES"
                        )
                        release_input_backpressure_hold(
                            out_serial,
                            output_hold_flow,
                            hold_released,
                        )
                        hold_applied = False
                        break
                    if now >= hold_deadline:
                        release_reason = "NO INPUT THROTTLE BEFORE HOLD LIMIT"
                        release_sent = sent_snapshot
                        after_input_lines = line_snapshot
                        console_progress(
                            f"{prefix}: NO INPUT THROTTLE; RELEASE OUTPUT HOLD"
                        )
                        release_input_backpressure_hold(
                            out_serial,
                            output_hold_flow,
                            hold_released,
                        )
                        hold_applied = False
                        break
                    if now >= next_progress_at:
                        with received_lock:
                            held_snapshot = held_bytes_seen
                        console_progress(
                            f"{prefix}: SENT={sent_snapshot}/{payload_bytes} "
                            f"HELD-OUT={held_snapshot} "
                            f"{modem_line_snapshot_label(line_snapshot)}"
                        )
                        next_progress_at = now + max(options.progress_interval, 0.1)

                if not hold_released.is_set():
                    with write_lock:
                        release_sent = bytes_sent
                    after_input_lines = modem_line_snapshot(in_serial)
                    release_reason = "STRESS WRITE COMPLETED WITHOUT INPUT THROTTLE"
                    console_progress(f"{prefix}: WRITE COMPLETE; RELEASE OUTPUT HOLD")
                    release_input_backpressure_hold(
                        out_serial,
                        output_hold_flow,
                        hold_released,
                    )
                    hold_applied = False

                writer_join_seconds = max(
                    estimated_transmit_seconds(input_settings, payload_bytes) * 2.0
                    + 10.0,
                    10.0,
                )
                writer_thread.join(timeout=writer_join_seconds)
                if writer_thread.is_alive():
                    write_error = write_error or "WRITE DID NOT FINISH AFTER OUTPUT RELEASE"
                    stop_event.set()

                reader_deadline = time.monotonic() + max(
                    estimated_receive_seconds(settings, payload_bytes) * 2.0
                    + read_timeout
                    + 10.0,
                    read_timeout + 10.0,
                )
                while reader_thread.is_alive() and time.monotonic() < reader_deadline:
                    reader_thread.join(timeout=0.2)
                if reader_thread.is_alive():
                    reader_errors.append("OUTPUT READ DID NOT GO QUIET AFTER STRESS")
                    stop_event.set()
                    reader_thread.join(timeout=1.0)
    except SERIAL_IO_ERRORS as exc:
        logger.exception("input backpressure stress failed for %s", settings.label())
        return flow_validation_error_result(
            flow_control=input_flow,
            method=INPUT_BACKPRESSURE_METHOD,
            settings=settings,
            payload=payload,
            error=str(exc),
            elapsed_sec=time.monotonic() - started,
        )
    finally:
        if hold_applied:
            try:
                release_flow_control_hold(out_serial, output_hold_flow)  # type: ignore[name-defined]
            except SERIAL_IO_ERRORS:
                logger.debug("failed to release %s hold in cleanup", output_hold_flow)

    received_bytes = bytes(received)
    score = score_received(expected, received_bytes)
    error = write_error or (reader_errors[0] if reader_errors else None)
    output_hold_ok = held_bytes_seen <= hold_limit
    backpressure_observed = input_backpressure_observed(release_reason)
    clean = (
        score.score >= 99.0
        and bytes_sent == payload_bytes
        and len(received_bytes) == payload_bytes
        and score.metrics.missing_bytes == 0
        and score.metrics.extra_bytes == 0
    )
    if not output_hold_ok:
        status = "stress-invalid"
        reason = (
            f"OUTPUT HOLD LEAKED {held_bytes_seen} BYTES BEFORE RELEASE "
            f"(LIMIT {hold_limit}); BUFFER MAY NOT HAVE FILLED."
        )
    elif error and backpressure_observed:
        status = "backpressure-partial"
        reason = f"{release_reason} AFTER {release_sent} BYTES; {error}"
    elif error:
        status = "error"
        reason = error
    elif bytes_sent < payload_bytes and not backpressure_observed:
        status = "partial-write"
        reason = "STRESS PAYLOAD WAS NOT FULLY SENT."
    elif backpressure_observed and clean:
        status = "backpressure-proven"
        reason = (
            f"{release_reason} AFTER {release_sent} BYTES; "
            "OUTPUT THEN DRAINED CLEANLY."
        )
    elif backpressure_observed:
        status = "backpressure-partial"
        reason = (
            f"{release_reason} AFTER {release_sent} BYTES; "
            "DRAINED OUTPUT WAS NOT BYTE-PERFECT."
        )
    elif clean:
        status = "no-backpressure"
        reason = (
            f"NO INPUT-SIDE THROTTLE OBSERVED WITHIN {bytes_sent} BYTES; "
            "STRESS PAYLOAD WAS ACCEPTED WHILE OUTPUT WAS HELD."
        )
    elif not received_bytes:
        status = "no-data"
        reason = "NO OUTPUT WAS RECEIVED AFTER STRESS RELEASE."
    else:
        status = "transfer-partial"
        reason = "STRESS DRAIN PRODUCED ONLY A PARTIAL MATCH."

    if before_input_lines or after_input_lines:
        logger.info(
            "%s input lines before=%s after=%s",
            prefix,
            before_input_lines,
            after_input_lines,
        )
    after_event = (
        "BUFFER FULL"
        if input_backpressure_observed(release_reason)
        else "AFTER STRESS"
    )
    modem_observations = tuple(
        modem_line_observation(label, snapshot)
        for label, snapshot in (
            ("IDLE", before_input_lines),
            (after_event, after_input_lines),
        )
        if snapshot
    )
    return FlowControlValidationResult(
        flow_control=input_flow,
        method=INPUT_BACKPRESSURE_METHOD,
        settings=settings,
        bytes_sent=bytes_sent,
        bytes_received=len(received_bytes),
        bytes_seen_while_held=held_bytes_seen,
        score=score.score,
        indicator=flow_validation_indicator(status, score.score, error),
        status=status,
        reason=reason,
        error=error,
        elapsed_sec=time.monotonic() - started,
        metrics=score.metrics,
        modem_line_observations=modem_observations,
    )


def run_input_backpressure_validation(
    serial_module: Any,
    options: ScanOptions,
    target: CandidateResult | None,
    hold_results: Sequence[FlowControlValidationResult],
    logger: logging.Logger,
) -> tuple[list[FlowControlValidationResult], str | None]:
    """Run the known-baud input-side buffer-full stress test."""
    print()
    print_report_title("INPUT BUFFER-FULL STRESS")
    if target is None:
        print("SKIPPED: NO CLEAN FOLLOW-UP FRAME WAS FOUND.")
        print(border_line(REPORT_WIDTH))
        return [], None
    frame = input_backpressure_settings_for_target(target)
    if frame is None:
        print("SKIPPED: NO CONCRETE INPUT/OUTPUT FRAME WAS FOUND.")
        print(border_line(REPORT_WIDTH))
        return [], None
    if not options.auto_stress_input_backpressure:
        reason = "BUFFER-FULL STRESS IS OFF IN SCAN / VALIDATE SETUP."
        print(f"SKIPPED: {reason}")
        print(border_line(REPORT_WIDTH))
        return [input_backpressure_skip_result(frame, reason)], None
    hold = select_backpressure_output_hold(hold_results)
    if hold is None:
        reason = "NO OUTPUT HOLD MODE PROVEN; BUFFER MAY NOT FILL AT EQUAL BAUD."
        print(f"SKIPPED: {reason}")
        print(border_line(REPORT_WIDTH))
        return [input_backpressure_skip_result(frame, reason)], None

    run_known_output_purges(
        serial_module=serial_module,
        options=options,
        logger=logger,
        settings_list=[frame.output_settings],
        reason="CLEAR VALIDATION DATA BEFORE INPUT BUFFER-FULL STRESS.",
    )
    payload = generate_payload(options.input_backpressure_stress_bytes)
    print_wrapped_value("FRAME PAIR: ", frame.label())
    print(
        f"TEST: {payload.byte_count} BYTES; OUTPUT HELD BY "
        f"{flow_control_name(hold.flow_control)}."
    )
    print("RESULT PROVES INPUT-SIDE BUFFER-FULL HANDSHAKE ONLY IF THROTTLE IS SEEN.")
    print(border_line(REPORT_WIDTH))
    logger.info(
        "input backpressure stress started frame=%s payload=%s output_hold=%s",
        frame.label(),
        payload.byte_count,
        hold.flow_control,
    )

    results: list[FlowControlValidationResult] = []
    total = len(INPUT_BACKPRESSURE_FLOW_CONTROLS)
    for index, input_flow in enumerate(INPUT_BACKPRESSURE_FLOW_CONTROLS, start=1):
        while True:
            try:
                result = run_input_backpressure_stress(
                    serial_module=serial_module,
                    index=index,
                    total=total,
                    frame=frame,
                    input_flow=input_flow,
                    output_hold_flow=hold.flow_control,
                    options=options,
                    payload=payload,
                    logger=logger,
                )
            except KeyboardInterrupt:
                action = prompt_operator_break_action("INPUT BUFFER-FULL STRESS")
                if action == "resume":
                    logger.info(
                        "operator resumed input backpressure stress at %s/%s %s",
                        index,
                        total,
                        input_flow,
                    )
                    continue
                logger.info(
                    "operator break during input backpressure stress; action=%s completed=%s/%s",
                    action,
                    len(results),
                    total,
                )
                print_flow_control_validation_report(frame, results)
                return results, action
            results.append(result)
            print(format_flow_validation_progress(result, index, total))
            logger.info(
                "input backpressure %s: indicator=%s status=%s sent=%s read=%s held=%s reason=%s error=%s",
                input_flow,
                result.indicator,
                result.status,
                result.bytes_sent,
                result.bytes_received,
                result.bytes_seen_while_held,
                result.reason,
                result.error,
            )
            break

    print_flow_control_validation_report(frame, results)
    logger.info(
        "input backpressure stress completed: %s",
        flow_control_validation_recommendation(results),
    )
    return results, None


def bank2_frame_serial_settings(baud: int) -> list[SerialSettings]:
    """Return the targeted frame list used for known-baud characterization.

    The order favors common printer-buffer frames first, then expands into
    parity and stop-bit variants. Flow is held at `none`; flow behavior is tested
    in a later validation phase.
    """
    frame_specs = [
        (8, "none", 1),
        (7, "even", 1),
        (7, "odd", 1),
        (7, "mark", 1),
        (7, "space", 1),
        (8, "even", 1),
        (8, "odd", 1),
        (8, "mark", 1),
        (8, "space", 1),
        (7, "none", 1),
        (7, "even", 2),
        (7, "odd", 2),
        (7, "mark", 2),
        (7, "space", 2),
        (7, "none", 2),
        (8, "none", 2),
        (8, "even", 2),
        (8, "odd", 2),
        (8, "mark", 2),
        (8, "space", 2),
    ]
    return [
        SerialSettings(
            baud=baud,
            data_bits=data_bits,
            parity=parity,
            stop_bits=stop_bits,
            flow_control="none",
        )
        for data_bits, parity, stop_bits in frame_specs
    ]


def bank2_frame_candidates(
    input_baud: int,
    output_baud: int,
    independent_frames: bool = True,
) -> list[SerialSettings | DualSerialSettings]:
    """Return targeted frame candidates for known same-baud or baud-pair tests.

    Same-baud tests can use single settings when independent frames are disabled.
    Different input/output bauds always force dual settings because each side
    must be opened at its own physical line speed.
    """
    input_frames = bank2_frame_serial_settings(input_baud)
    independent_frames = independent_frames or input_baud != output_baud
    if not independent_frames and input_baud == output_baud:
        return list(input_frames)
    output_frames = bank2_frame_serial_settings(output_baud)
    return [
        DualSerialSettings(input_settings=input_frame, output_settings=output_frame)
        for input_frame in input_frames
        for output_frame in output_frames
    ]


def run_bank2_candidate(
    serial_module: Any,
    index: int,
    total: int,
    settings: SerialSettings | DualSerialSettings,
    options: ScanOptions,
    payload: ProbePayload,
    logger: logging.Logger,
) -> CandidateResult:
    """Run one same-setting or input/output-pair candidate."""
    if isinstance(settings, DualSerialSettings):
        return run_dual_candidate(
            serial_module=serial_module,
            index=index,
            total=total,
            settings=settings,
            options=options,
            payload=payload,
            logger=logger,
            progress=console_progress,
        )
    return run_candidate(
        serial_module=serial_module,
        index=index,
        total=total,
        settings=settings,
        options=options,
        payload=payload,
        logger=logger,
        progress=console_progress,
    )


def ascii_pass_frame_labels(results: Sequence[CandidateResult]) -> list[str]:
    """Return compact labels for clean ASCII frame candidates."""
    labels: list[str] = []
    for result in results:
        if not clean_ascii_transfer(result):
            continue
        label = frame_or_pair_label(result.settings)
        if label not in labels:
            labels.append(label)
    return labels


def likely_bank2_ascii_results(results: Sequence[CandidateResult]) -> list[CandidateResult]:
    """Return clean ASCII frames close enough to retest with high-bit bytes."""
    clean = [result for result in results if clean_ascii_transfer(result)]
    if not clean:
        return []
    best_score = max(result.score for result in clean)
    return [
        result for result in clean if (best_score - result.score) <= TIE_SCORE_TOLERANCE
    ]


def eight_bit_result_summary(results: Sequence[CandidateResult]) -> str:
    """Return a concise high-bit challenge conclusion.

    A clean result requires high-bit bytes to arrive unchanged. Rows where high
    bits are stripped are reported as a 7-bit/masked path even if ASCII framing
    looks otherwise healthy.
    """
    if not results:
        return "NOT RUN"
    clean = [
        result
        for result in results
        if result.score >= 99.0
        and result.metrics.high_bit_bytes_sent > 0
        and result.metrics.high_bit_exact_ratio >= 1.0
    ]
    if clean:
        return "8-BIT CLEAN"
    masked = [
        result for result in results if result.metrics.high_bit_stripped_count > 0
    ]
    if masked:
        return "8-BIT NOT CLEAN; 7-BIT/MASKED PATH, HIGH-BIT BYTES STRIPPED"
    if any(result.bytes_received > 0 for result in results):
        return "8-BIT NOT CLEAN OR INCONCLUSIVE"
    return "NO 8-BIT DATA RECEIVED"


def bank2_flow_summary(results: Sequence[FlowControlValidationResult]) -> str:
    """Return a concise flow-validation summary for known-baud reports.

    Matrix, output-hold, and input-full results are kept separate because each
    method proves a different operational property.
    """
    if not results:
        return "NOT RUN"
    matrix_results = flow_transfer_matrix_results(results)
    hold_results = flow_hold_release_results(results)
    stress_results = flow_input_backpressure_results(results)
    summary_parts: list[str] = []
    if matrix_results:
        summary_parts.append(f"MATRIX: {bank2_flow_matrix_summary(results)}")
    if hold_results:
        summary_parts.append(f"OUTPUT HOLD: {bank2_flow_hold_summary(results)}")
    if stress_results:
        summary_parts.append(f"INPUT FULL: {bank2_flow_input_summary(results)}")
    if summary_parts:
        return "; ".join(summary_parts)
    return flow_control_validation_recommendation(results)


def flow_transfer_matrix_results(
    results: Sequence[FlowControlValidationResult],
) -> list[FlowControlValidationResult]:
    """Return dual input/output flow-transfer matrix rows."""
    return [result for result in results if result.method == "dual-transfer"]


def flow_hold_release_results(
    results: Sequence[FlowControlValidationResult],
) -> list[FlowControlValidationResult]:
    """Return same-frame output hold/release flow validation rows."""
    return [
        result
        for result in results
        if result.method not in {"dual-transfer", INPUT_BACKPRESSURE_METHOD}
    ]


def flow_input_backpressure_results(
    results: Sequence[FlowControlValidationResult],
) -> list[FlowControlValidationResult]:
    """Return input-side buffer-full stress rows."""
    return [result for result in results if result.method == INPUT_BACKPRESSURE_METHOD]


def bank2_flow_matrix_summary(
    results: Sequence[FlowControlValidationResult],
) -> str:
    """Return the known-baud input/output flow-transfer matrix summary."""
    matrix_results = flow_transfer_matrix_results(results)
    if not matrix_results:
        return "NOT RUN"
    return flow_control_validation_recommendation(matrix_results)


def bank2_flow_hold_summary(
    results: Sequence[FlowControlValidationResult],
) -> str:
    """Return the known-baud output-side hold/release summary."""
    hold_results = flow_hold_release_results(results)
    if not hold_results:
        return "NOT RUN"
    return flow_control_validation_recommendation(hold_results)


def bank2_flow_input_summary(
    results: Sequence[FlowControlValidationResult],
) -> str:
    """Return the known-baud input-side buffer-full stress summary."""
    stress_results = flow_input_backpressure_results(results)
    if not stress_results:
        return "NOT RUN"
    return flow_control_validation_recommendation(stress_results)


def titled_flow_validation_report_lines(
    title: str,
    results: Sequence[FlowControlValidationResult],
) -> list[str]:
    """Return flow validation report lines under a custom section title."""
    lines = flow_validation_report_lines(results)
    if lines:
        lines[0] = title
    return lines


def bank2_flow_report_lines(
    results: Sequence[FlowControlValidationResult],
    skip_reason: str | None = None,
) -> list[str]:
    """Return detailed known-baud flow evidence split by test method."""
    if skip_reason:
        lines = ["FLOW CONTROL VALIDATION:"]
        lines.extend(
            wrapped_value_lines(
                "FINDING:         ",
                f"SKIPPED: {skip_reason}",
                REPORT_WIDTH,
            )
        )
        lines.append(border_line(REPORT_WIDTH))
        return lines

    matrix_results = flow_transfer_matrix_results(results)
    hold_results = flow_hold_release_results(results)
    stress_results = flow_input_backpressure_results(results)
    if not matrix_results and not hold_results and not stress_results:
        return [
            "FLOW CONTROL VALIDATION:",
            "FINDING:         NOT RUN",
            border_line(REPORT_WIDTH),
        ]

    lines: list[str] = []
    if matrix_results:
        lines.extend(
            titled_flow_validation_report_lines(
                "FLOW TRANSFER MATRIX:",
                matrix_results,
            )
        )
    if matrix_results and hold_results:
        lines.append("")
    if hold_results:
        lines.extend(
            titled_flow_validation_report_lines(
                "OUTPUT HOLD/RELEASE:",
                hold_results,
            )
        )
    if (matrix_results or hold_results) and stress_results:
        lines.append("")
    if stress_results:
        lines.extend(
            titled_flow_validation_report_lines(
                "INPUT BUFFER-FULL STRESS:",
                stress_results,
            )
        )
    return lines


def same_baud_frame(left: SerialSettings, right: SerialSettings) -> bool:
    """Return True when two concrete settings use the same baud and frame."""
    return (
        left.baud == right.baud
        and left.data_bits == right.data_bits
        and left.parity == right.parity
        and left.stop_bits == right.stop_bits
    )


def compact_label_list(labels: Sequence[str], limit: int = 10) -> str:
    """Return a compact comma list with an old-terminal friendly overflow note."""
    unique: list[str] = []
    for label in labels:
        if label not in unique:
            unique.append(label)
    if not unique:
        return "(NONE)"
    shown = unique[: max(limit, 1)]
    suffix = f" +{len(unique) - len(shown)} MORE" if len(unique) > len(shown) else ""
    return ", ".join(shown) + suffix


def bank2_settings_same_frame(settings: SerialSettings | DualSerialSettings) -> bool:
    """Return True when a known-baud setting uses the same baud/frame on both sides."""
    if not isinstance(settings, DualSerialSettings):
        return True
    return same_baud_frame(settings.input_settings, settings.output_settings)


def is_8e1(settings: SerialSettings) -> bool:
    """Return True for the observed all-off baseline frame."""
    return (
        settings.data_bits == 8
        and settings.parity == "even"
        and settings.stop_bits == 1
    )


def is_eight_bit_clean_result(result: CandidateResult) -> bool:
    """Return True when the high-bit challenge was byte-clean."""
    if result.error or result.status in STALE_STATUSES:
        return False
    return (
        result.score >= 99.0
        and result.metrics.high_bit_bytes_sent > 0
        and result.metrics.high_bit_exact_ratio >= 1.0
    )


def bank2_eight_bit_clean_results(
    results: Sequence[CandidateResult],
) -> list[CandidateResult]:
    """Return known-baud high-bit-clean candidates."""
    return [result for result in results if is_eight_bit_clean_result(result)]


def bank2_side_frame_labels(
    results: Sequence[CandidateResult],
    side: str,
) -> list[str]:
    """Return ordered input or output frame labels from result settings."""
    labels: list[str] = []
    for result in results:
        if side == "input":
            settings = transmit_side_settings(result.settings)
        elif side == "output":
            settings = receive_side_settings(result.settings)
        else:
            raise ValueError(f"unknown side {side!r}")
        label = frame_label(settings)
        if label not in labels:
            labels.append(label)
    return labels


def bank2_compact_result_summary(
    results: Sequence[CandidateResult],
    label_limit: int = 10,
) -> str:
    """Return compact human-readable settings evidence for known-baud reports."""
    if not results:
        return "(NONE)"
    labels = [frame_or_pair_label(result.settings) for result in results]
    if any(isinstance(result.settings, DualSerialSettings) for result in results):
        input_frames = bank2_side_frame_labels(results, "input")
        output_frames = bank2_side_frame_labels(results, "output")
        return (
            f"{len(labels)} PAIR(S); "
            f"IN {compact_label_list(input_frames, label_limit)}; "
            f"OUT {compact_label_list(output_frames, label_limit)}"
        )
    return compact_label_list(labels, label_limit)


def bank2_ascii_pass_summary(results: Sequence[CandidateResult]) -> str:
    """Return a compact summary of clean ASCII known-baud transfer candidates."""
    clean = [result for result in results if clean_ascii_transfer(result)]
    return bank2_compact_result_summary(clean)


def bank2_masked_eight_bit_results(
    results: Sequence[CandidateResult],
) -> list[CandidateResult]:
    """Return high-bit challenge rows where high bits were stripped or masked."""
    return [
        result
        for result in results
        if result.metrics.high_bit_bytes_sent > 0
        and result.metrics.high_bit_stripped_count > 0
    ]


def bank2_eight_bit_detail_summary(results: Sequence[CandidateResult]) -> str:
    """Return a compact summary of high-bit known-baud transfer candidates."""
    clean = bank2_eight_bit_clean_results(results)
    if clean:
        return "8-BIT CLEAN; " + bank2_compact_result_summary(clean, label_limit=8)
    return eight_bit_result_summary(results)


def bank2_data_width_summary(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
) -> str:
    """Return the clearest report row for observed data width."""
    if bank2_eight_bit_clean_results(eight_bit_results):
        return "8-BIT CLEAN; HIGH-BIT BYTES SURVIVED."
    if bank2_masked_eight_bit_results(eight_bit_results):
        return "7-BIT/MASKED DATA PATH; HIGH-BIT BYTES WERE STRIPPED."
    if ascii_pass_frame_labels(ascii_results):
        if eight_bit_results:
            return "ASCII ONLY; RAW 8-BIT DATA WIDTH IS NOT PROVEN."
        return "ASCII ONLY; 8-BIT CHALLENGE WAS NOT RUN."
    if any(result.bytes_received > 0 for result in eight_bit_results):
        return "INCONCLUSIVE; BYTES RETURNED BUT NO CLEAN FRAME MATCHED."
    return "NOT PROVEN; NO CLEAN ASCII OR 8-BIT TRANSFER."


def bank2_usable_frame_summary(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
) -> str:
    """Return the selected setting plus its limitation in one actionable row."""
    target = best_bank2_followup_target(ascii_results, eight_bit_results)
    if target is None:
        return "(NONE)"
    target_label = frame_or_pair_label(target.settings)
    if bank2_eight_bit_clean_results(eight_bit_results):
        return f"{target_label} FOR BYTE TRANSFER."
    return f"{target_label} FOR PRINTABLE 7-BIT ASCII ONLY."


def bank2_target_output_variant_labels(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
    *,
    same_parity: bool = False,
    same_stop: bool = False,
) -> list[str]:
    """Return clean receive-side frame labels comparable to the selected target."""
    target = best_bank2_followup_target(ascii_results, eight_bit_results)
    if target is None:
        return []
    target_output = receive_side_settings(target.settings)
    labels: list[str] = []
    for result in ascii_results:
        if not clean_ascii_transfer(result):
            continue
        output = receive_side_settings(result.settings)
        if output.data_bits != target_output.data_bits:
            continue
        if same_parity and output.parity != target_output.parity:
            continue
        if same_stop and output.stop_bits != target_output.stop_bits:
            continue
        label = frame_label(output)
        if label not in labels:
            labels.append(label)
    return labels


def bank2_parity_proof_summary(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
) -> str:
    """Return whether the byte results uniquely proved receive-side parity."""
    labels = bank2_target_output_variant_labels(
        ascii_results,
        eight_bit_results,
        same_stop=True,
    )
    if not labels:
        return "NOT PROVEN; NO CLEAN ASCII FRAME."
    if len(labels) > 1:
        return f"NOT UNIQUE; OUTPUT {compact_label_list(labels, 8)} PASSED ASCII."
    return f"ONLY OUTPUT {labels[0]} PASSED ASCII; BYTE TEST ONLY."


def bank2_stop_bits_proof_summary(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
) -> str:
    """Return whether the byte results uniquely proved receive-side stop bits."""
    labels = bank2_target_output_variant_labels(
        ascii_results,
        eight_bit_results,
        same_parity=True,
    )
    if not labels:
        return "NOT PROVEN; NO CLEAN ASCII FRAME."
    if len(labels) > 1:
        return f"NOT UNIQUE; OUTPUT {compact_label_list(labels, 8)} PASSED ASCII."
    return f"ONLY OUTPUT {labels[0]} PASSED ASCII; BYTE TEST ONLY."


def bank2_target_sort_key(result: CandidateResult) -> tuple[bool, bool, bool, bool, bool, bool, bool, float, float, float, int]:
    """Return a stable preference key for known-baud follow-up targets."""
    transmit = transmit_side_settings(result.settings)
    receive = receive_side_settings(result.settings)
    same_frame = bank2_settings_same_frame(result.settings)
    baseline = same_frame and is_8e1(transmit) and is_8e1(receive)
    return (
        not result.error and result.status not in STALE_STATUSES,
        is_eight_bit_clean_result(result),
        is_recommendable_result(result),
        same_frame,
        baseline,
        is_8e1(transmit),
        is_8e1(receive),
        result.score,
        result.metrics.line_integrity_ratio,
        result.metrics.exact_byte_match_ratio,
        -result.index,
    )


def best_bank2_followup_target(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
) -> CandidateResult | None:
    """Return the best proven known-baud target for follow-up probes.

    Clean high-bit rows are preferred over ASCII-only rows. When evidence ties,
    same-frame and 8E1-compatible rows are preferred so later flow and raw-byte
    probes run against the most conservative target.
    """
    sources = [
        bank2_eight_bit_clean_results(eight_bit_results),
        likely_bank2_ascii_results(ascii_results),
    ]
    for source in sources:
        eligible = [
            result
            for result in source
            if not result.error and result.status not in STALE_STATUSES
        ]
        if eligible:
            return sorted(eligible, key=bank2_target_sort_key, reverse=True)[0]
    return None


def bank2_followup_target_label(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
) -> str:
    """Return report text for the selected known-baud follow-up frame."""
    target = best_bank2_followup_target(ascii_results, eight_bit_results)
    if target is None:
        return "(NONE - NO CLEAN ASCII/8-BIT TARGET)"
    return frame_or_pair_label(target.settings)


def best_bank2_ascii_target(
    results: Sequence[CandidateResult],
) -> CandidateResult | None:
    """Return the best likely known-baud ASCII target for follow-up probes."""
    likely = likely_bank2_ascii_results(results)
    source = likely or list(results)
    if not source:
        return None
    return sorted(source, key=result_sort_key, reverse=True)[0]


def bank2_behavior_targets(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
) -> list[CandidateResult]:
    """Return a small target set for optional raw behavior probes.

    The target set is capped so a known-baud run remains practical even when
    several frames tie under ASCII scoring.
    """
    clean_8bit = bank2_eight_bit_clean_results(eight_bit_results)
    if clean_8bit:
        ranked = sorted(clean_8bit, key=bank2_target_sort_key, reverse=True)
        if len(ranked) <= BANK2_BEHAVIOR_MAX_TIED_TARGETS:
            return ranked
        return ranked[:1]
    likely = likely_bank2_ascii_results(ascii_results)
    if not likely:
        return []
    ranked = sorted(likely, key=bank2_target_sort_key, reverse=True)
    if len(ranked) <= BANK2_BEHAVIOR_MAX_TIED_TARGETS:
        return ranked
    return ranked[:1]


def bank2_flow_skip_reason(
    target: CandidateResult | None,
    input_baud: int,
    output_baud: int,
) -> str | None:
    """Return a conservative known-baud flow-validation skip reason, if any."""
    if target is None:
        return "No clean follow-up frame was found"
    if input_baud != output_baud and not isinstance(target.settings, DualSerialSettings):
        return "Known baud pair needs an independent input/output follow-up frame"
    return None


def bank2_flow_validation_seed(
    target: CandidateResult | None,
) -> list[CandidateResult]:
    """Return a same-frame seed row for existing flow validation."""
    if target is None:
        return []
    if isinstance(target.settings, DualSerialSettings):
        return [dataclasses.replace(target, settings=target.settings.input_settings)]
    return [target]


def bank2_dual_flow_validation_target(
    target: CandidateResult | None,
) -> CandidateResult | None:
    """Return a dual-settings target for known-baud flow matrix testing."""
    if target is None:
        return None
    if isinstance(target.settings, DualSerialSettings):
        return target
    base_settings = dataclasses.replace(target.settings, flow_control="none")
    return dataclasses.replace(
        target,
        settings=DualSerialSettings(base_settings, base_settings),
    )


def bank2_behavior_marker(
    run_id: str,
    target_index: int,
    probe_name: str,
    switch_hash: str | None,
) -> bytes:
    """Return a unique marker embedded in raw known-baud behavior payloads."""
    note = f" NOTE={switch_hash}" if switch_hash else ""
    marker = (
        f"<<<KNOWN_RAW RUN={sanitize_nonce_value(run_id or make_run_id('KBR'))} "
        f"CAND=KBT{target_index:02d} PROBE={sanitize_nonce_value(probe_name)}"
        f"{note}>>>"
    )
    return marker.encode("ascii")


def bank2_behavior_probe_payloads(
    run_id: str,
    target_index: int,
    switch_hash: str | None,
) -> list[tuple[str, bytes]]:
    """Return raw byte payload classes for known-baud behavior probing.

    Payloads are intentionally grouped by byte class so reports can distinguish
    line-ending conversion, printable ASCII handling, printer controls, 7-bit
    controls, XON/XOFF-sensitive controls, and timeout/form-feed behavior.
    """
    payloads: list[tuple[str, bytes]] = []
    for name, separator in (
        ("CR_ONLY", b"\r"),
        ("LF_ONLY", b"\n"),
        ("CRLF", b"\r\n"),
    ):
        marker = bank2_behavior_marker(run_id, target_index, name, switch_hash)
        payloads.append(
            (
                name,
                separator.join(
                    [
                        marker,
                        b"LINE=ONE",
                        b"LINE=TWO",
                        b"END=RAW-BEHAVIOR",
                    ]
                )
                + separator,
            )
        )

    marker = bank2_behavior_marker(run_id, target_index, "PRINT_CTRL", switch_hash)
    payloads.append(
        (
            "PRINT_CTRL",
            b"\r\n".join(
                [
                    marker,
                    b"START",
                    b"Line 1",
                    b"Line 2\tTabbed text",
                    b"Line 3 before form feed",
                    b"\x0c",
                    b"AFTER FORM FEED",
                    b"\x1b@",
                    b"\x1bE",
                    b"\x1bF",
                    b"END",
                ]
            )
            + b"\r\n",
        )
    )

    marker = bank2_behavior_marker(run_id, target_index, "ASCII_SWEEP", switch_hash)
    payloads.append(("ASCII_SWEEP", marker + b"\r\n" + bytes(range(0x20, 0x7F)) + b"\r\n"))

    marker = bank2_behavior_marker(run_id, target_index, "CTL7_SAFE", switch_hash)
    # WHY: The safe control sweep excludes XON/XOFF so a later full sweep can
    # isolate whether software flow-control bytes are being interpreted.
    payloads.append(
        (
            "CTL7_SAFE",
            marker
            + b"\r\nCTL7_SAFE_BEGIN\r\n"
            + bytes(byte for byte in range(0x00, 0x80) if byte not in {0x11, 0x13})
            + b"\r\nCTL7_SAFE_END\r\n",
        )
    )

    marker = bank2_behavior_marker(run_id, target_index, "CTL7_FULL", switch_hash)
    payloads.append(
        (
            "CTL7_FULL",
            marker
            + b"\r\nCTL7_FULL_BEGIN\r\n"
            + bytes(range(0x00, 0x80))
            + b"\r\nCTL7_FULL_END\r\n",
        )
    )

    marker = bank2_behavior_marker(run_id, target_index, "TIMEOUT_FF", switch_hash)
    payloads.append(
        (
            "TIMEOUT_FF",
            marker + b"\r\nSHORT-JOB=TRUE\r\nWAITING-FOR-TIMEOUT\r\n",
        )
    )
    return payloads


def cr_lf_change_observed(sent: bytes, received: bytes) -> bool:
    """Return True when byte differences look like CR/LF conversion."""
    if sent == received or not received:
        return False
    sent_without = sent.replace(b"\r", b"").replace(b"\n", b"")
    received_without = received.replace(b"\r", b"").replace(b"\n", b"")
    if sent_without == received_without and (
        sent.count(b"\r") != received.count(b"\r")
        or sent.count(b"\n") != received.count(b"\n")
    ):
        return True
    normalize = lambda data: data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return normalize(sent) == normalize(received)


def first_mismatch_offset(sent: bytes, received: bytes) -> int | None:
    """Return the first differing byte offset, or None for exact byte equality."""
    prefix_bytes = exact_prefix_byte_count(sent, received)
    if prefix_bytes == len(sent) == len(received):
        return None
    return prefix_bytes


def classify_bank2_behavior_bytes(
    sent: bytes,
    received: bytes,
    error: str | None = None,
) -> tuple[bool, bool, bool, str, str]:
    """Classify one raw behavior probe without assigning switch meaning.

    Returns:
        Tuple of exact-match flag, form-feed-inserted flag, CR/LF-changed flag,
        status, and operator-facing reason.
    """
    exact_match = received == sent
    form_feed_inserted = b"\x0c" in received and b"\x0c" not in sent
    cr_lf_changed = cr_lf_change_observed(sent, received)
    if error:
        return exact_match, form_feed_inserted, cr_lf_changed, "error", error
    if not received:
        return exact_match, form_feed_inserted, cr_lf_changed, "no-data", "NO DATA RECEIVED."
    if exact_match:
        return exact_match, form_feed_inserted, cr_lf_changed, "exact", "EXACT BYTE MATCH."
    if form_feed_inserted and cr_lf_changed:
        return (
            exact_match,
            form_feed_inserted,
            cr_lf_changed,
            "transformed",
            "FORM FEED AND CR/LF BYTE CHANGES OBSERVED.",
        )
    if form_feed_inserted:
        return (
            exact_match,
            form_feed_inserted,
            cr_lf_changed,
            "transformed",
            "FORM FEED BYTE OBSERVED IN OUTPUT.",
        )
    if cr_lf_changed:
        return (
            exact_match,
            form_feed_inserted,
            cr_lf_changed,
            "transformed",
            "CR/LF BYTE CHANGE OBSERVED.",
        )
    if len(received) < len(sent):
        return exact_match, form_feed_inserted, cr_lf_changed, "partial", "PARTIAL OUTPUT."
    return exact_match, form_feed_inserted, cr_lf_changed, "transformed", "OUTPUT BYTES DIFFER."


def write_raw_bytes_only(
    in_serial: Any,
    settings: SerialSettings | DualSerialSettings,
    data: bytes,
    progress_interval: float,
    prefix: str,
    logger: logging.Logger,
) -> tuple[int, str | None, float]:
    """Write raw bytes without applying probe-payload scoring."""
    started = time.monotonic()
    chunk_size = write_chunk_size(settings)
    bytes_sent = 0
    error: str | None = None
    estimated = estimated_transmit_seconds(settings, len(data))
    console_progress(
        f"{prefix}: SEND RAW {len(data)} BYTES "
        f"(CHUNK={chunk_size}, ABOUT {format_duration(estimated)})"
    )
    try:
        next_progress_at = time.monotonic() + max(progress_interval, 0.1)
        while bytes_sent < len(data):
            chunk = data[bytes_sent : bytes_sent + chunk_size]
            written = in_serial.write(chunk)
            if written is None:
                written = len(chunk)
            if written <= 0:
                raise RuntimeError("serial write returned zero bytes")
            bytes_sent += int(written)
            now = time.monotonic()
            if now >= next_progress_at:
                percent = (bytes_sent / len(data)) * 100.0
                console_progress(
                    f"{prefix}: WRITING RAW {bytes_sent}/{len(data)} BYTES "
                    f"({percent:5.1f}%)"
                )
                next_progress_at = now + max(progress_interval, 0.1)
        in_serial.flush()
    except SERIAL_IO_ERRORS as exc:
        error = str(exc)
        logger.debug("%s raw write failed: %s", prefix, error)
    return bytes_sent, error, time.monotonic() - started


def run_bank2_behavior_probe(
    serial_module: Any,
    index: int,
    total: int,
    name: str,
    settings: SerialSettings | DualSerialSettings,
    options: ScanOptions,
    payload: bytes,
    observe_seconds: float,
    logger: logging.Logger,
) -> Bank2BehaviorProbeResult:
    """Run one raw known-baud printer-job behavior probe.

    Raw probes bypass structured payload scoring and compare sent/received bytes
    directly. They are meant to characterize transformations, not to discover a
    new frame.
    """
    started = time.monotonic()
    transmit_settings = transmit_side_settings(settings)
    receive_settings = receive_side_settings(settings)
    read_timeout = max(options.read_timeout, 0.5)
    quiet_seconds = observe_seconds if name == "TIMEOUT_FF" else read_timeout
    payload_receive_seconds = estimated_receive_seconds(settings, len(payload))
    pre_drain_timeout, pre_drain_quiet = low_baud_pre_drain_values(
        settings,
        len(payload),
        max(options.pre_drain_timeout, payload_receive_seconds + options.pre_drain_quiet),
        options.pre_drain_quiet,
    )
    prefix = f"[KNOWN RAW {index:02d}/{total:02d} {name} {settings.label()}]"
    received = b""
    bytes_sent = 0
    error: str | None = None
    try:
        console_progress(border_line(PROGRESS_WIDTH))
        console_progress(
            bordered_text(f"KNOWN-BAUD RAW {index}/{total} {name}", PROGRESS_WIDTH)
        )
        console_progress(border_line(PROGRESS_WIDTH))
        with open_serial_port(
            serial_module,
            options.out_port,
            receive_settings,
            quiet_seconds,
        ) as out_serial:
            with open_serial_port(
                serial_module,
                options.in_port,
                transmit_settings,
                read_timeout,
            ) as in_serial:
                reset_serial_buffers(out_serial)
                reset_serial_buffers(in_serial)
                time.sleep(options.settle_ms / 1000.0)

                if not options.no_pre_drain:
                    drain = drain_output_until_quiet(
                        out_serial=out_serial,
                        quiet_seconds=pre_drain_quiet,
                        max_seconds=pre_drain_timeout,
                        max_bytes=options.max_drain_bytes,
                        progress_interval=options.progress_interval,
                        progress=console_progress,
                        prefix=prefix,
                        logger=logger,
                    )
                    if not drain.quiet:
                        error = (
                            "OUTPUT DID NOT GO QUIET BEFORE RAW PROBE "
                            f"(REASON={drain.reason.upper()}, "
                            f"CLEARED={drain.bytes_drained})"
                        )

                if error is None:
                    bytes_sent, write_error, _write_elapsed = write_raw_bytes_only(
                        in_serial=in_serial,
                        settings=settings,
                        data=payload,
                        progress_interval=options.progress_interval,
                        prefix=prefix,
                        logger=logger,
                    )
                    received, read_error, _read_elapsed = read_until_quiet(
                        out_serial=out_serial,
                        settings=settings,
                        expected_bytes=len(payload),
                        read_timeout=quiet_seconds,
                        progress_interval=options.progress_interval,
                        prefix=prefix,
                        logger=logger,
                    )
                    error = write_error or read_error
    except SERIAL_IO_ERRORS as exc:
        logger.exception(
            "Known-baud behavior probe failed for %s %s",
            name,
            settings.label(),
        )
        error = str(exc)

    exact, form_feed, cr_lf, status, reason = classify_bank2_behavior_bytes(
        payload,
        received,
        error,
    )
    result = Bank2BehaviorProbeResult(
        name=name,
        settings=settings,
        bytes_sent=bytes_sent,
        bytes_received=len(received),
        sent_hash=f"{fnv1a32(payload):08X}",
        received_hash=f"{fnv1a32(received):08X}",
        first_mismatch_offset=first_mismatch_offset(payload, received),
        missing_bytes=max(len(payload) - len(received), 0),
        extra_bytes=max(len(received) - len(payload), 0),
        exact_match=exact,
        form_feed_inserted=form_feed,
        cr_lf_changed=cr_lf,
        received_preview_ascii=preview_ascii(received),
        received_preview_hex=preview_hex(received),
        status=status,
        reason=reason,
        error=error,
        elapsed_sec=time.monotonic() - started,
    )
    logger.info(
        "Known-baud behavior %s %s: status=%s sent=%s read=%s "
        "first_diff=%s exact=%s ff=%s crlf=%s reason=%s error=%s",
        name,
        settings.label(),
        result.status,
        result.bytes_sent,
        result.bytes_received,
        result.first_mismatch_offset,
        result.exact_match,
        result.form_feed_inserted,
        result.cr_lf_changed,
        result.reason,
        result.error,
    )
    logger.debug(
        "Known-baud behavior %s preview ascii=%r hex=%s",
        name,
        result.received_preview_ascii,
        result.received_preview_hex,
    )
    return result


def format_bank2_behavior_progress(
    result: Bank2BehaviorProbeResult,
    index: int,
    total: int,
) -> str:
    """Return one compact console line for a raw behavior probe."""
    frame_text = frame_or_pair_label(result.settings).replace(" >> ", ">>")[:17]
    flags = []
    if result.exact_match:
        flags.append("EXACT")
    if result.form_feed_inserted:
        flags.append("FF")
    if result.cr_lf_changed:
        flags.append("CRLF")
    flag_text = ",".join(flags) if flags else "-"
    diff_text = (
        "-"
        if result.first_mismatch_offset is None
        else str(result.first_mismatch_offset)
    )
    return fit_terminal_line(
        f"RAW [{index:02d}/{total:02d}] {result.status.upper()[:11]} "
        f"{frame_text} "
        f"S={progress_count_label(result.bytes_sent)} "
        f"R={progress_count_label(result.bytes_received)} "
        f"D={diff_text} F={flag_text} {terminal_text(result.reason)}",
        TERMINAL_COLUMNS,
    )


def run_bank2_behavior_probes(
    serial_module: Any,
    options: ScanOptions,
    targets: Sequence[CandidateResult],
    observe_seconds: float,
    logger: logging.Logger,
) -> list[Bank2BehaviorProbeResult]:
    """Run optional raw known-baud behavior probes against a small target set."""
    if not targets:
        return []
    print()
    print_report_title("KNOWN-BAUD RAW BYTE-BEHAVIOR PROBES")
    print(f"TARGETS: {len(targets)} PROBE FRAME(S).")
    print(f"TIMEOUT OBSERVE WINDOW: {observe_seconds:.1f}S.")
    print("BYTE-LEVEL OBSERVATIONS ONLY; COMPARE DEVICE/SWITCH REPORT BLOCKS.")
    print(border_line(REPORT_WIDTH))
    total = len(targets) * len(bank2_behavior_probe_payloads("COUNT", 1, None))
    results: list[Bank2BehaviorProbeResult] = []
    index = 0
    for target_index, target in enumerate(targets, start=1):
        payloads = bank2_behavior_probe_payloads(
            options.run_id,
            target_index,
            switch_note_hash(options.switch_note),
        )
        for name, payload in payloads:
            index += 1
            result = run_bank2_behavior_probe(
                serial_module=serial_module,
                index=index,
                total=total,
                name=name,
                settings=target.settings,
                options=options,
                payload=payload,
                observe_seconds=observe_seconds,
                logger=logger,
            )
            results.append(result)
            print(format_bank2_behavior_progress(result, index, total), flush=True)
    return results


def bank2_behavior_summary(
    results: Sequence[Bank2BehaviorProbeResult],
) -> str:
    """Return a concise summary of raw known-baud behavior observations."""
    if not results:
        return "NOT RUN"
    by_name = {result.name: result for result in results}
    safe = by_name.get("CTL7_SAFE")
    full = by_name.get("CTL7_FULL")
    if safe and full and safe.exact_match and not full.exact_match:
        return "XON/XOFF CONTROL BYTES AFFECT RAW PATH"
    if any(result.form_feed_inserted for result in results):
        return "FORM FEED OBSERVED"
    if any(result.cr_lf_changed for result in results):
        return "CR/LF TRANSFORMATION OBSERVED"
    if any(result.status == "transformed" for result in results):
        return "RAW BYTE TRANSFORMATION OBSERVED"
    if all(result.status == "exact" for result in results):
        return "RAW BYTE CLASSES EXACT"
    if any(result.status == "no-data" for result in results):
        return "RAW PROBE NO DATA OR PARTIAL"
    if any(result.status == "error" for result in results):
        return "RAW PROBE ERROR"
    return "RAW BYTE BEHAVIOR INCONCLUSIVE"


BANK2_SUMMARY_LABEL_WIDTH = 16


def bank2_value_lines(
    label: str,
    value: object,
    indent: str = "  ",
    width: int = REPORT_WIDTH,
) -> list[str]:
    """Return known-baud key/value lines with one fixed value column."""
    return wrapped_value_lines(
        f"{indent}{label:<{BANK2_SUMMARY_LABEL_WIDTH}} ",
        value,
        width,
    )


def print_bank2_value(label: str, value: object) -> None:
    """Print one aligned known-baud key/value row."""
    for line in bank2_value_lines(label, value):
        print(line)


def yn_flag(value: bool) -> str:
    """Return a one-character yes/no table flag."""
    return "Y" if value else "N"


def bank2_raw_diff_text(result: Bank2BehaviorProbeResult) -> str:
    """Return raw-probe first-difference text."""
    if result.first_mismatch_offset is None:
        return "-"
    return str(result.first_mismatch_offset)


def bank2_raw_sr_text(sent: int, received: int) -> str:
    """Return compact sent/received text for raw-probe tables."""
    return f"{sent}/{received}"


def bank2_raw_console_header() -> str:
    """Return the console raw-probe table header."""
    return (
        f"  {'RAW PROBE':<12} "
        f"{'STATUS':<11} "
        f"{'S/R':>9} "
        f"{'DIFF':>5} "
        f"{'EX':>2} "
        f"{'FF':>2} "
        f"{'CRLF':>4}"
    )


def bank2_raw_console_row(result: Bank2BehaviorProbeResult) -> str:
    """Return one console raw-probe table row."""
    return (
        f"  {result.name:<12} "
        f"{result.status.upper():<11} "
        f"{bank2_raw_sr_text(result.bytes_sent, result.bytes_received):>9} "
        f"{bank2_raw_diff_text(result):>5} "
        f"{yn_flag(result.exact_match):>2} "
        f"{yn_flag(result.form_feed_inserted):>2} "
        f"{yn_flag(result.cr_lf_changed):>4}"
    )


def bank2_raw_report_header() -> str:
    """Return the text-report raw-probe table header."""
    return (
        f"{'PROBE':<12} "
        f"{'STATUS':<11} "
        f"{'S/R':>9} "
        f"{'MISS':>4} "
        f"{'EXTRA':>5} "
        f"{'DIFF':>5} "
        f"{'EX':>2} "
        f"{'FF':>2} "
        f"{'CRLF':>4} "
        f"{'HASH-S':>8} "
        f"{'HASH-R':>8}"
    )


def bank2_raw_report_row(result: Bank2BehaviorProbeResult) -> str:
    """Return one text-report raw-probe table row."""
    return (
        f"{result.name:<12} "
        f"{result.status.upper():<11} "
        f"{bank2_raw_sr_text(result.bytes_sent, result.bytes_received):>9} "
        f"{result.missing_bytes:>4} "
        f"{result.extra_bytes:>5} "
        f"{bank2_raw_diff_text(result):>5} "
        f"{yn_flag(result.exact_match):>2} "
        f"{yn_flag(result.form_feed_inserted):>2} "
        f"{yn_flag(result.cr_lf_changed):>4} "
        f"{result.sent_hash:>8} "
        f"{result.received_hash:>8}"
    )


def bank2_behavior_report_lines(
    results: Sequence[Bank2BehaviorProbeResult],
) -> list[str]:
    """Return compact raw behavior lines for reports."""
    lines = [
        "BEHAVIOR RESULT:",
        *bank2_value_lines("SUMMARY:", bank2_behavior_summary(results), indent=""),
        "",
        bank2_raw_report_header(),
        border_line(REPORT_WIDTH),
    ]
    if not results:
        lines.append("(NOT RUN)")
    for result in results:
        lines.append(bank2_raw_report_row(result))
        lines.extend(wrapped_value_lines("  SETTING: ", result.settings.label(), REPORT_WIDTH))
        lines.extend(wrapped_value_lines("  WHY: ", result.reason, REPORT_WIDTH))
    lines.extend(
        [
            "NOTE: RAW BEHAVIOR PROBES ARE BYTE-LEVEL OBSERVATIONS ONLY.",
            "NOTE: HASH VALUES ARE FNV-1A 32-BIT OVER SENT AND RECEIVED BYTES.",
            "NOTE: DEVICE/SWITCH MEANING REQUIRES COMPARING MULTIPLE BLOCKS.",
            border_line(REPORT_WIDTH),
        ]
    )
    return lines


def bank2_etx_ack_payload(run_id: str, switch_hash: str | None) -> bytes:
    """Return a small ETX-bearing payload for forward path testing."""
    note = f" NOTE={switch_hash}" if switch_hash else ""
    marker = (
        f"<<<KNOWN_ETXACK RUN={sanitize_nonce_value(run_id or make_run_id('KBE'))}"
        f"{note}>>>"
    ).encode("ascii")
    return marker + b"\r\nETX-FOLLOWS:" + b"\x03" + b":END\r\n"


def classify_etx_ack_bytes(
    forward_payload: bytes,
    forward_received: bytes,
    reverse_received: bytes,
    error: str | None = None,
) -> tuple[bool, bool, bool, bool, str, str]:
    """Classify ETX forward and ACK reverse byte observations.

    Returns:
        Tuple of ETX-seen flag, exact-forward flag, ACK-seen flag,
        reverse-path-observed flag, status, and operator-facing reason.
    """
    etx_forward_seen = b"\x03" in forward_received
    etx_forward_exact = forward_received == forward_payload
    ack_reverse_seen = b"\x06" in reverse_received
    reverse_path_observed = ack_reverse_seen
    if error:
        return (
            etx_forward_seen,
            etx_forward_exact,
            ack_reverse_seen,
            reverse_path_observed,
            "error",
            error,
        )
    if ack_reverse_seen and etx_forward_seen:
        return (
            etx_forward_seen,
            etx_forward_exact,
            ack_reverse_seen,
            reverse_path_observed,
            "reverse-seen",
            "ETX FORWARD SEEN; ACK REVERSE SEEN.",
        )
    if ack_reverse_seen:
        return (
            etx_forward_seen,
            etx_forward_exact,
            ack_reverse_seen,
            reverse_path_observed,
            "reverse-seen",
            "ACK REVERSE SEEN; ETX FORWARD NOT CONFIRMED.",
        )
    if etx_forward_seen:
        return (
            etx_forward_seen,
            etx_forward_exact,
            ack_reverse_seen,
            reverse_path_observed,
            "forward-only",
            "ETX FORWARD SEEN; ACK REVERSE NOT SEEN.",
        )
    if forward_received:
        return (
            etx_forward_seen,
            etx_forward_exact,
            ack_reverse_seen,
            reverse_path_observed,
            "etx-missing",
            "FORWARD DATA SEEN BUT ETX BYTE WAS NOT OBSERVED.",
        )
    return (
        etx_forward_seen,
        etx_forward_exact,
        ack_reverse_seen,
        reverse_path_observed,
        "no-data",
        "NO FORWARD OR REVERSE PROTOCOL BYTES OBSERVED.",
    )


def run_bank2_etx_ack_probe(
    serial_module: Any,
    options: ScanOptions,
    settings: SerialSettings | DualSerialSettings,
    observe_seconds: float,
    logger: logging.Logger,
) -> Bank2EtxAckProbeResult:
    """Run a selected-frame ETX forward and simulated ACK reverse-path probe.

    The reverse check injects ACK from the printer side and reads the computer
    side. It observes byte path directionality; it does not emulate printer
    protocol state.
    """
    started = time.monotonic()
    transmit_settings = transmit_side_settings(settings)
    receive_settings = receive_side_settings(settings)
    read_timeout = max(options.read_timeout, observe_seconds, 0.5)
    payload = bank2_etx_ack_payload(
        options.run_id,
        switch_note_hash(options.switch_note),
    )
    ack_payload = b"\x06"
    pre_drain_bytes = max(options.payload_bytes, len(payload), len(ack_payload))
    pre_drain_timeout, pre_drain_quiet = low_baud_pre_drain_values(
        settings,
        pre_drain_bytes,
        max(options.pre_drain_timeout, 0.5),
        options.pre_drain_quiet,
    )
    prefix = f"[KNOWN ETX/ACK {settings.label()}]"
    forward_received = b""
    reverse_received = b""
    forward_sent = 0
    reverse_sent = 0
    error: str | None = None
    try:
        print()
        print_report_title("KNOWN-BAUD ETX/ACK PATH PROBE")
        print(f"PROBE FRAME: {frame_or_pair_label(settings)}")
        print("FORWARD TEST SENDS ETX (03) TOWARD PRINTER SIDE.")
        print("REVERSE TEST INJECTS ACK (06) FROM PRINTER SIDE.")
        print("BYTE-LEVEL PATH TEST ONLY; NO PRINTER MEANING IS ASSIGNED.")
        print(border_line(REPORT_WIDTH))
        console_progress(border_line(PROGRESS_WIDTH))
        console_progress(bordered_text("KNOWN-BAUD ETX/ACK PATH", PROGRESS_WIDTH))
        console_progress(border_line(PROGRESS_WIDTH))
        with open_serial_port(
            serial_module,
            options.out_port,
            receive_settings,
            read_timeout,
        ) as out_serial:
            with open_serial_port(
                serial_module,
                options.in_port,
                transmit_settings,
                read_timeout,
            ) as in_serial:
                reset_serial_buffers(out_serial)
                reset_serial_buffers(in_serial)
                time.sleep(options.settle_ms / 1000.0)

                if not options.no_pre_drain:
                    drain = drain_output_until_quiet(
                        out_serial=out_serial,
                        quiet_seconds=pre_drain_quiet,
                        max_seconds=pre_drain_timeout,
                        max_bytes=options.max_drain_bytes,
                        progress_interval=options.progress_interval,
                        progress=console_progress,
                        prefix=prefix,
                        logger=logger,
                    )
                    if not drain.quiet:
                        error = (
                            "OUTPUT DID NOT GO QUIET BEFORE ETX/ACK PROBE "
                            f"(REASON={drain.reason.upper()}, "
                            f"CLEARED={drain.bytes_drained})"
                        )

                if error is None:
                    console_progress(f"{prefix}: FORWARD ETX TEST")
                    forward_sent, write_error, _write_elapsed = write_raw_bytes_only(
                        in_serial=in_serial,
                        settings=transmit_settings,
                        data=payload,
                        progress_interval=options.progress_interval,
                        prefix=prefix,
                        logger=logger,
                    )
                    forward_received, forward_error, _forward_elapsed = read_serial_until_quiet(
                        serial_port=out_serial,
                        settings=receive_settings,
                        expected_bytes=len(payload),
                        read_timeout=read_timeout,
                        progress_interval=options.progress_interval,
                        prefix=prefix,
                        logger=logger,
                        direction_label="printer-side forward",
                    )
                    error = write_error or forward_error

                if error is None:
                    console_progress(f"{prefix}: REVERSE ACK TEST")
                    in_serial.reset_input_buffer()
                    reverse_sent, reverse_write_error, _reverse_write_elapsed = (
                        write_raw_bytes_only(
                            in_serial=out_serial,
                            settings=receive_settings,
                            data=ack_payload,
                            progress_interval=options.progress_interval,
                            prefix=prefix,
                            logger=logger,
                        )
                    )
                    reverse_received, reverse_read_error, _reverse_read_elapsed = (
                        read_serial_until_quiet(
                            serial_port=in_serial,
                            settings=transmit_settings,
                            expected_bytes=len(ack_payload),
                            read_timeout=observe_seconds,
                            progress_interval=options.progress_interval,
                            prefix=prefix,
                            logger=logger,
                            direction_label="computer-side reverse",
                        )
                    )
                    error = reverse_write_error or reverse_read_error
    except SERIAL_IO_ERRORS as exc:
        logger.exception("Known-baud ETX/ACK probe failed for %s", settings.label())
        error = str(exc)

    etx_seen, etx_exact, ack_seen, reverse_seen, status, reason = classify_etx_ack_bytes(
        payload,
        forward_received,
        reverse_received,
        error,
    )
    result = Bank2EtxAckProbeResult(
        settings=settings,
        forward_bytes_sent=forward_sent,
        forward_bytes_received=len(forward_received),
        reverse_bytes_sent=reverse_sent,
        reverse_bytes_received=len(reverse_received),
        etx_forward_seen=etx_seen,
        etx_forward_exact=etx_exact,
        ack_reverse_seen=ack_seen,
        reverse_path_observed=reverse_seen,
        forward_preview_ascii=preview_ascii(forward_received),
        forward_preview_hex=preview_hex(forward_received),
        reverse_preview_ascii=preview_ascii(reverse_received),
        reverse_preview_hex=preview_hex(reverse_received),
        status=status,
        reason=reason,
        error=error,
        elapsed_sec=time.monotonic() - started,
    )
    logger.info(
        "Known-baud ETX/ACK %s: status=%s fwd_sent=%s fwd_read=%s rev_sent=%s rev_read=%s etx=%s ack=%s reason=%s error=%s",
        settings.label(),
        result.status,
        result.forward_bytes_sent,
        result.forward_bytes_received,
        result.reverse_bytes_sent,
        result.reverse_bytes_received,
        result.etx_forward_seen,
        result.ack_reverse_seen,
        result.reason,
        result.error,
    )
    print(
        f"ETX/ACK {result.status.upper():<12} "
        f"ETX={'Y' if result.etx_forward_seen else 'N'} "
        f"ACK={'Y' if result.ack_reverse_seen else 'N'} "
        f"FWD={result.forward_bytes_sent}/{result.forward_bytes_received} "
        f"REV={result.reverse_bytes_sent}/{result.reverse_bytes_received} "
        f"{terminal_text(result.reason)}"
    )
    return result


def run_bank2_etx_ack_probes(
    serial_module: Any,
    options: ScanOptions,
    target: CandidateResult | None,
    observe_seconds: float,
    logger: logging.Logger,
) -> list[Bank2EtxAckProbeResult]:
    """Run optional ETX/ACK probing on the selected known-baud probe frame."""
    if target is None:
        print()
        print_report_title("KNOWN-BAUD ETX/ACK PATH PROBE")
        print("SKIPPED: NO CLEAN FOLLOW-UP FRAME WAS FOUND.")
        print("ASCII FRAME MUST PASS BEFORE ETX/ACK PATH RESULTS ARE USEFUL.")
        print(border_line(REPORT_WIDTH))
        return []
    return [
        run_bank2_etx_ack_probe(
            serial_module=serial_module,
            options=options,
            settings=target.settings,
            observe_seconds=observe_seconds,
            logger=logger,
        )
    ]


def bank2_etx_ack_summary(results: Sequence[Bank2EtxAckProbeResult]) -> str:
    """Return a concise ETX/ACK path summary for known-baud reports."""
    if not results:
        return "NOT RUN"
    if any(result.ack_reverse_seen for result in results):
        return "ACK REVERSE SEEN"
    if any(result.etx_forward_seen for result in results):
        return "ETX FORWARD ONLY; ACK REVERSE NOT SEEN"
    if any(result.error for result in results):
        return "ETX/ACK PROBE ERROR"
    return "ETX/ACK PATH NOT OBSERVED"


def bank2_etx_ack_report_lines(
    results: Sequence[Bank2EtxAckProbeResult],
) -> list[str]:
    """Return compact ETX/ACK report lines."""
    lines = [
        "ETX/ACK PATH RESULT:",
        f"SUMMARY:         {bank2_etx_ack_summary(results)}",
        "",
        "STATUS       ETX ACK  FWD-S/R  REV-S/R  FRAME/PAIR              REASON",
        border_line(REPORT_WIDTH),
    ]
    if not results:
        lines.append("(NOT RUN)")
    for result in results:
        lines.append(
            f"{result.status.upper():<12} "
            f"{'Y' if result.etx_forward_seen else 'N':<3} "
            f"{'Y' if result.ack_reverse_seen else 'N':<3} "
            f"{result.forward_bytes_sent:>3}/{result.forward_bytes_received:<4} "
            f"{result.reverse_bytes_sent:>3}/{result.reverse_bytes_received:<4} "
            f"{frame_or_pair_label(result.settings)[:23]:<23} "
            f"{terminal_text(result.reason)[:24]}"
        )
    lines.extend(
        [
            "NOTE: ETX/ACK REQUIRES ACK (06) TO TRAVEL FROM PRINTER SIDE BACK TO COMPUTER SIDE.",
            "NOTE: THIS PROBE INJECTS ACK FROM THE PRINTER SIDE; IT DOES NOT EMULATE A PRINTER.",
            border_line(REPORT_WIDTH),
        ]
    )
    return lines


def bank2_conclusion(
    ascii_results: Sequence[CandidateResult],
    eight_bit_results: Sequence[CandidateResult],
    flow_results: Sequence[FlowControlValidationResult],
    behavior_results: Sequence[Bank2BehaviorProbeResult],
    etx_ack_results: Sequence[Bank2EtxAckProbeResult],
) -> str:
    """Return compact conclusion text for a known-baud switch state.

    The conclusion summarizes observed evidence in priority order. It does not
    assign semantic meaning to a switch bank, because the code only observes byte
    transfer, transformations, flow behavior, and ETX/ACK path evidence.
    """
    stale_seen = stale_nonce_seen(ascii_results) or stale_nonce_seen(eight_bit_results)

    def with_stale_warning(message: str) -> str:
        """Append stale-output context to a conclusion when needed."""
        if stale_seen:
            return f"{message}; stale row seen"
        return message

    pass_frames = ascii_pass_frame_labels(ascii_results)
    if not pass_frames:
        return with_stale_warning("No working frame found for selected known bauds")
    eight_summary = eight_bit_result_summary(eight_bit_results)
    if "7-BIT/MASKED" in eight_summary:
        return with_stale_warning(
            "7-bit ASCII transfer passes; high-bit data masked; parity/stop not proven"
        )
    if "NOT CLEAN" in eight_summary or "MASKED" in eight_summary:
        return with_stale_warning(
            "ASCII transfer passes, but 8-bit path is not clean; parity/stop not proven"
        )
    if any(result.form_feed_inserted for result in behavior_results):
        return with_stale_warning(
            "Form feed observed after timeout window; compare prior blocks"
        )
    if any(result.cr_lf_changed for result in behavior_results):
        return with_stale_warning("CR/LF transformation observed; compare prior blocks")
    if any(result.status == "transformed" for result in behavior_results):
        return with_stale_warning(
            "Raw byte behavior changed or transformed; compare prior blocks"
        )
    raw_exact = bool(behavior_results) and all(
        result.status == "exact" for result in behavior_results
    )
    if etx_ack_results and any(result.ack_reverse_seen for result in etx_ack_results):
        text = "8-bit clean; ACK reverse seen"
        if raw_exact:
            text = "8-bit clean; raw exact; ACK reverse seen"
        return with_stale_warning(text)
    if etx_ack_results and any(result.etx_forward_seen for result in etx_ack_results):
        text = "8-bit clean; ACK reverse not seen"
        if raw_exact:
            text = "8-bit clean; raw exact; ACK reverse not seen"
        return with_stale_warning(text)
    if bank2_eight_bit_clean_results(eight_bit_results):
        if raw_exact:
            return with_stale_warning("8-bit clean; raw bytes exact")
        return with_stale_warning("8-bit path clean; frame still ambiguous")
    if len(pass_frames) > 1:
        return with_stale_warning(
            "Ambiguous; byte transfer does not distinguish these settings"
        )
    if any(
        result.status in {"validated", "backpressure-proven"}
        for result in flow_results
    ):
        return with_stale_warning(
            "Flow behavior changed or was validated; compare prior blocks"
        )
    if not flow_results and not eight_bit_results:
        return with_stale_warning("ASCII frame unchanged; no 8-bit or flow phase run")
    return with_stale_warning("No observable change except listed byte-transfer evidence")


def bank2_setup_verdict(result: Bank2CharacterizationResult) -> str:
    """Return the human setup verdict for one known-baud characterization."""
    pass_frames = ascii_pass_frame_labels(result.ascii_results)
    if not pass_frames:
        return (
            "NO WORKING SERIAL SETTING FOUND FOR THIS DEVICE/SWITCH STATE AT "
            f"{result.known_baud_text}."
        )
    if not bank2_eight_bit_clean_results(result.eight_bit_results):
        return (
            "ASCII TRANSFER WAS FOUND; RAW 8-BIT TRANSFER IS NOT PROVEN CLEAN. "
            "DATA WIDTH/PARITY/STOP ROWS SHOW WHAT IS AND IS NOT PROVEN."
        )
    return "WORKING BYTE TRANSFER FOUND FOR THIS DEVICE/SWITCH STATE."


def bank2_next_action(result: Bank2CharacterizationResult) -> str:
    """Return the next operator action for one known-baud characterization."""
    pass_frames = ascii_pass_frame_labels(result.ascii_results)
    if not pass_frames:
        return (
            "DO NOT USE A FALLBACK FRAME AS THE SETTING. CHECK OPTION 2 BAUDS, "
            "COM1>>BUFFER INPUT AND BUFFER OUTPUT>>COM5 CABLING, SWITCH STATE, "
            "AND CLEAR/RESET THE BUFFER; THEN RUN AUTOMATED DISCOVERY OR RETRY "
            "KNOWN-BAUD DEVICE TEST WITH DIFFERENT KNOWN BAUDS."
        )
    if not bank2_eight_bit_clean_results(result.eight_bit_results):
        return (
            "USE THE USABLE FRAME ABOVE ONLY FOR PRINTABLE 7-BIT TRAFFIC. "
            "PARITY/STOP ARE NOT PROVEN WHEN THE PROOF ROWS SAY NOT UNIQUE. "
            "FOR RAW DATA OR PRINTER CONTROL BYTES, FIND AN 8-BIT CLEAN SETTING."
        )
    if result.flow_skip_reason:
        return "COMPARE THIS BLOCK TO OTHER DEVICE/SWITCH STATES; FLOW WAS NOT PROVEN HERE."
    return "COMPARE THIS BLOCK TO OTHER DEVICE/SWITCH STATES AND KEEP THE CLEANEST RESULT."


def write_bank2_text_report(
    path: Path,
    result: Bank2CharacterizationResult,
) -> None:
    """Write a concise known-baud device-test block."""
    path.parent.mkdir(parents=True, exist_ok=True)
    created = dt.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")
    ascii_summary = bank2_ascii_pass_summary(result.ascii_results)
    eight_detail = bank2_eight_bit_detail_summary(result.eight_bit_results)
    flow_summary = bank2_flow_summary(result.flow_results)
    flow_matrix_summary = bank2_flow_matrix_summary(result.flow_results)
    flow_hold_summary = bank2_flow_hold_summary(result.flow_results)
    flow_input_summary = bank2_flow_input_summary(result.flow_results)
    flow_observed = bank2_flow_control_observed_summary(result.flow_results)
    if result.flow_skip_reason:
        flow_summary = f"SKIPPED: {result.flow_skip_reason}"
        flow_matrix_summary = flow_summary
        flow_hold_summary = flow_summary
        flow_input_summary = flow_summary
        flow_observed = flow_summary
    behavior_summary = bank2_behavior_summary(result.behavior_results)
    etx_ack_summary = bank2_etx_ack_summary(result.etx_ack_results)
    target_label = bank2_followup_target_label(
        result.ascii_results,
        result.eight_bit_results,
    )
    byte_transfer = bank2_byte_transfer_summary(
        result.ascii_results,
        result.eight_bit_results,
    )
    data_width = bank2_data_width_summary(
        result.ascii_results,
        result.eight_bit_results,
    )
    usable_frame = bank2_usable_frame_summary(
        result.ascii_results,
        result.eight_bit_results,
    )
    parity_proof = bank2_parity_proof_summary(
        result.ascii_results,
        result.eight_bit_results,
    )
    stop_proof = bank2_stop_bits_proof_summary(
        result.ascii_results,
        result.eight_bit_results,
    )
    verdict = bank2_setup_verdict(result)
    next_action = bank2_next_action(result)
    lines = [
        "",
        border_line(REPORT_WIDTH),
        bordered_text("KNOWN-BAUD DEVICE TEST", REPORT_WIDTH),
        border_line(REPORT_WIDTH),
        *bank2_value_lines("WRITTEN:", created, indent=""),
        *bank2_value_lines("RUN ID:", result.run_id, indent=""),
        *bank2_value_lines(
            "DEVICE NOTE:",
            result.switch_note or "(NOT ENTERED)",
            indent="",
        ),
        *bank2_value_lines("KNOWN BAUD/PAIR:", result.known_baud_text, indent=""),
        *bank2_value_lines("BYTE TRANSFER:", byte_transfer, indent=""),
        *bank2_value_lines("DATA WIDTH:", data_width, indent=""),
        *bank2_value_lines("USABLE FRAME:", usable_frame, indent=""),
        *bank2_value_lines("PARITY PROOF:", parity_proof, indent=""),
        *bank2_value_lines("STOP PROOF:", stop_proof, indent=""),
        *bank2_value_lines("ASCII PASS:", ascii_summary, indent=""),
        *bank2_value_lines("8-BIT RESULT:", eight_detail, indent=""),
        *bank2_value_lines("FOLLOW-UP FRAME:", target_label, indent=""),
        *bank2_value_lines("FLOW CONTROL OBSERVED:", flow_observed, indent=""),
        *bank2_value_lines("FLOW RESULT:", flow_summary, indent=""),
        *bank2_value_lines("FLOW MATRIX:", flow_matrix_summary, indent=""),
        *bank2_value_lines("OUTPUT HOLD:", flow_hold_summary, indent=""),
        *bank2_value_lines("INPUT FULL:", flow_input_summary, indent=""),
        *bank2_value_lines("BEHAVIOR RESULT:", behavior_summary, indent=""),
        *bank2_value_lines("ETX/ACK RESULT:", etx_ack_summary, indent=""),
        *bank2_value_lines(
            "STALE DATA SEEN:",
            "YES" if result.stale_data_seen else "NO",
            indent="",
        ),
        *bank2_value_lines("CONCLUSION:", result.conclusion, indent=""),
        *bank2_value_lines("VERDICT:", verdict, indent=""),
        *bank2_value_lines("NEXT:", next_action, indent=""),
        "",
        "SUMMARY TABLE:",
        border_line(REPORT_WIDTH),
        *bank2_value_lines(
            "DEVICE NOTE:",
            result.switch_note or "(NOT ENTERED)",
            indent="  ",
        ),
        *bank2_value_lines("KNOWN:", result.known_baud_text, indent="  "),
        *bank2_value_lines("BYTE TRANSFER:", byte_transfer, indent="  "),
        *bank2_value_lines("DATA WIDTH:", data_width, indent="  "),
        *bank2_value_lines("USABLE:", usable_frame, indent="  "),
        *bank2_value_lines("PARITY:", parity_proof, indent="  "),
        *bank2_value_lines("STOP:", stop_proof, indent="  "),
        *bank2_value_lines("ASCII:", ascii_summary, indent="  "),
        *bank2_value_lines("8-BIT:", eight_detail, indent="  "),
        *bank2_value_lines("RAW:", behavior_summary, indent="  "),
        *bank2_value_lines("ETX/ACK:", etx_ack_summary, indent="  "),
        *bank2_value_lines("FLOW OBSERVED:", flow_observed, indent="  "),
        *bank2_value_lines("FLOW:", flow_summary, indent="  "),
        *bank2_value_lines("FLOW MATRIX:", flow_matrix_summary, indent="  "),
        *bank2_value_lines("OUTPUT HOLD:", flow_hold_summary, indent="  "),
        *bank2_value_lines("INPUT FULL:", flow_input_summary, indent="  "),
        *bank2_value_lines(
            "STALE:",
            "YES" if result.stale_data_seen else "NO",
            indent="  ",
        ),
        *bank2_value_lines("CONCLUSION:", result.conclusion, indent="  "),
        *bank2_value_lines("VERDICT:", verdict, indent="  "),
        border_line(REPORT_WIDTH),
        "",
        "EVIDENCE:",
        *bank2_value_lines("BYTE TRANSFER:", byte_transfer),
        *bank2_value_lines("DATA WIDTH:", data_width),
        *bank2_value_lines("USABLE FRAME:", usable_frame),
        *bank2_value_lines("PARITY PROOF:", parity_proof),
        *bank2_value_lines("STOP PROOF:", stop_proof),
        *bank2_value_lines("ASCII:", ascii_summary),
        *bank2_value_lines("8-BIT:", eight_detail),
        *bank2_value_lines("FOLLOW-UP FRAME:", target_label),
        *bank2_value_lines("RAW BYTES:", behavior_summary),
        *bank2_value_lines("ETX/ACK:", etx_ack_summary),
        *bank2_value_lines("FLOW OBSERVED:", flow_observed),
        *bank2_value_lines("FLOW:", flow_summary),
        *bank2_value_lines("FLOW MATRIX:", flow_matrix_summary),
        *bank2_value_lines("OUTPUT HOLD:", flow_hold_summary),
        *bank2_value_lines("INPUT FULL:", flow_input_summary),
        *bank2_value_lines(
            "STALE WARNING:",
            "YES" if result.stale_data_seen else "NO",
        ),
        "",
        *bank2_behavior_report_lines(result.behavior_results),
        "",
        *bank2_etx_ack_report_lines(result.etx_ack_results),
        "",
        *bank2_flow_report_lines(result.flow_results, result.flow_skip_reason),
        "",
        "INTERPRETATION:",
        "  FRAME PASS HERE MEANS CLEAN ASCII BYTE TRANSFER, NOT UNIQUE PARITY/STOP PROOF.",
        "  THE DATA WIDTH, PARITY PROOF, AND STOP PROOF ROWS ARE THE SHORT ANSWER.",
        "  FOLLOW-UP FRAME IS THE TEST FRAME CHOSEN FOR MORE PROBES, NOT A UNIQUE PROOF.",
        "  8-BIT CLEAN IS STRONGER EVIDENCE FOR AN 8-BIT DATA PATH.",
        "  ETX/ACK REQUIRES A REVERSE ACK PATH; XON/XOFF DOES NOT PROVE THAT PATH.",
        "  FLOW OBSERVED IS SEPARATE FROM BYTE TRANSFER UNDER A FLOW SETTING.",
        "  FLOW MATRIX SHOWS WHICH IN/OUT FLOW PAIRS MOVE BYTES CLEANLY.",
        "  OUTPUT HOLD/RELEASE PROVES ONLY OBSERVED OUTPUT-SIDE PAUSE/RESUME.",
        "  INPUT BUFFER-FULL STRESS IS THE INPUT-SIDE BACKPRESSURE PROOF.",
        "  BEHAVIOR PROBES OBSERVE BYTES ONLY; THEY DO NOT ASSIGN DEVICE MEANING.",
        "  STALE WARNING MEANS ONE ROW HAD WRONG-RUN DATA; IT IS NOT DEVICE MEANING.",
        "  COMPARE THIS REPORT BLOCK TO OTHER DEVICE-NOTE BLOCKS MANUALLY.",
        border_line(REPORT_WIDTH),
    ]
    with open_session_text_report(path) as report_file:
        report_file.write("\n".join(lines) + "\n")


def print_bank2_report(
    result: Bank2CharacterizationResult,
    report_path: Path,
) -> None:
    """Print the known-baud device-test conclusion."""
    print()
    print_report_title("KNOWN-BAUD DEVICE TEST")
    print_bank2_value("DEVICE NOTE:", result.switch_note or "(NOT ENTERED)")
    print_bank2_value("KNOWN BAUD/PAIR:", result.known_baud_text)
    target_label = bank2_followup_target_label(
        result.ascii_results,
        result.eight_bit_results,
    )
    print_bank2_value(
        "BYTE TRANSFER:",
        bank2_byte_transfer_summary(result.ascii_results, result.eight_bit_results),
    )
    print_bank2_value(
        "DATA WIDTH:",
        bank2_data_width_summary(result.ascii_results, result.eight_bit_results),
    )
    print_bank2_value(
        "USABLE FRAME:",
        bank2_usable_frame_summary(result.ascii_results, result.eight_bit_results),
    )
    print_bank2_value(
        "PARITY PROOF:",
        bank2_parity_proof_summary(result.ascii_results, result.eight_bit_results),
    )
    print_bank2_value(
        "STOP PROOF:",
        bank2_stop_bits_proof_summary(result.ascii_results, result.eight_bit_results),
    )
    print_bank2_value("ASCII PASS:", bank2_ascii_pass_summary(result.ascii_results))
    print_bank2_value(
        "8-BIT RESULT:",
        bank2_eight_bit_detail_summary(result.eight_bit_results),
    )
    print_bank2_value("FOLLOW-UP FRAME:", target_label)
    flow_summary = bank2_flow_summary(result.flow_results)
    flow_matrix_summary = bank2_flow_matrix_summary(result.flow_results)
    flow_hold_summary = bank2_flow_hold_summary(result.flow_results)
    flow_input_summary = bank2_flow_input_summary(result.flow_results)
    flow_observed = bank2_flow_control_observed_summary(result.flow_results)
    if result.flow_skip_reason:
        flow_summary = f"SKIPPED: {result.flow_skip_reason}"
        flow_matrix_summary = flow_summary
        flow_hold_summary = flow_summary
        flow_input_summary = flow_summary
        flow_observed = flow_summary
    print_bank2_value("FLOW OBSERVED:", flow_observed)
    print_bank2_value("FLOW RESULT:", flow_summary)
    print_bank2_value("FLOW MATRIX:", flow_matrix_summary)
    print_bank2_value("OUTPUT HOLD:", flow_hold_summary)
    print_bank2_value("INPUT FULL:", flow_input_summary)
    print_bank2_value(
        "BEHAVIOR RESULT:",
        bank2_behavior_summary(result.behavior_results),
    )
    print_bank2_value(
        "ETX/ACK RESULT:",
        bank2_etx_ack_summary(result.etx_ack_results),
    )
    if result.behavior_results:
        print(border_line(REPORT_WIDTH))
        print(bank2_raw_console_header())
        for probe in result.behavior_results:
            print(bank2_raw_console_row(probe))
    if result.etx_ack_results:
        print(border_line(REPORT_WIDTH))
        print("  ETX/ACK      STATUS       ETX ACK  FWD-S/R  REV-S/R")
        for probe in result.etx_ack_results:
            print(
                f"  PATH         "
                f"{probe.status.upper():<12} "
                f"{'Y' if probe.etx_forward_seen else 'N':<3} "
                f"{'Y' if probe.ack_reverse_seen else 'N':<3} "
                f"{probe.forward_bytes_sent:>3}/{probe.forward_bytes_received:<4} "
                f"{probe.reverse_bytes_sent:>3}/{probe.reverse_bytes_received:<4}"
            )
    print_bank2_value("STALE DATA SEEN:", "YES" if result.stale_data_seen else "NO")
    print_bank2_value("CONCLUSION:", result.conclusion)
    print_bank2_value("VERDICT:", bank2_setup_verdict(result))
    print_bank2_value("NEXT:", bank2_next_action(result))
    print_bank2_value("TEXT REPORT:", f"{report_path} (CURRENT SESSION)")
    print(border_line(REPORT_WIDTH))


def run_second_bank_characterization(options: ScanOptions) -> None:
    """Run a targeted known-baud device characterization.

    The workflow uses the operator-configured input/output bauds, purges the
    known output frames, runs targeted ASCII frame checks, retests likely frames
    with a high-bit payload, optionally probes raw behavior and ETX/ACK, and
    runs flow validation only when a clean follow-up frame exists.
    """
    print()
    print_report_title("KNOWN-BAUD DEVICE TEST")
    print("USES OPTION 2 BAUDS TO TEST FRAME, BYTES, AND DEVICE BEHAVIOR.")
    print_wrapped_value(
        "PORTS: ",
        f"{options.in_port} >> BUFFER >> {options.out_port}",
    )
    print_wrapped_value(
        "SEQUENCE: ",
        (
            "PURGE, ASCII FRAME TEST, 8-BIT CHALLENGE, RAW BEHAVIOR, "
            "ETX/ACK PATH, FLOW MATRIX, OUTPUT HOLD, INPUT BUFFER-FULL."
        ),
    )
    print(border_line(REPORT_WIDTH))
    input_baud = options.input_baud
    output_baud = options.output_baud
    print_wrapped_value(
        "KNOWN BAUDS: ",
        f"INPUT {input_baud}, OUTPUT {output_baud} (MAIN MENU OPTION 2)",
    )
    switch_note = prompt_text("DEVICE/SWITCH NOTE FOR REPORT", options.switch_note)
    independent_frames = prompt_yes_no("TEST INPUT/OUTPUT FRAMES INDEPENDENTLY", True)
    if input_baud != output_baud:
        print("INPUT/OUTPUT BAUD DIFFER; INDEPENDENT FRAME TESTING FORCED ON.")
        independent_frames = True
    run_behavior = prompt_yes_no("RUN RAW BYTE-BEHAVIOR PROBES", True)
    job_timeout_observe = DEFAULT_BANK2_JOB_TIMEOUT_OBSERVE_SECONDS
    if run_behavior:
        job_timeout_observe = prompt_float(
            "JOB TIMEOUT OBSERVE SECONDS",
            DEFAULT_BANK2_JOB_TIMEOUT_OBSERVE_SECONDS,
            0.5,
        )
    run_etx_ack = prompt_yes_no("RUN ETX/ACK REVERSE-PATH PROBE", True)
    etx_ack_observe = DEFAULT_ETX_ACK_OBSERVE_SECONDS
    if run_etx_ack:
        etx_ack_observe = prompt_float(
            "ETX/ACK OBSERVE SECONDS",
            DEFAULT_ETX_ACK_OBSERVE_SECONDS,
            0.5,
        )
    payload_bytes = max(options.payload_bytes, minimum_payload_size())

    bank_options = dataclasses.replace(
        options,
        min_baud=min(input_baud, output_baud),
        max_baud=max(input_baud, output_baud),
        payload_bytes=payload_bytes,
        bursts=1,
        switch_note=switch_note,
        turbo_discovery_enabled=False,
        ask_on_top_match=False,
        no_pre_drain=False,
        run_id=make_run_id("KB"),
    )
    try:
        ensure_distinct_ports(bank_options.in_port, bank_options.out_port)
    except ValueError as exc:
        print(f"SETTINGS ERROR: {terminal_text(exc)}")
        return

    logger = setup_logging(bank_options.log_file)
    serial_module = import_pyserial()
    known_text = (
        f"IN {input_baud} / OUT {output_baud}"
        if input_baud != output_baud
        else str(input_baud)
    )
    frames = bank2_frame_candidates(
        input_baud,
        output_baud,
        independent_frames=independent_frames,
    )
    # WHY: Known-baud mode limits the search to frame behavior at the selected
    # baud or baud pair. A weak fallback row is not promoted into behavior or
    # flow probing unless the ASCII/high-bit evidence produces a clean target.
    run_known_output_purges(
        serial_module=serial_module,
        options=bank_options,
        logger=logger,
        settings_list=known_output_purge_settings(frames),
        reason="KNOWN-BAUD DEVICE PRE-TEST PURGE.",
    )

    ascii_results: list[CandidateResult] = []
    ascii_payload = generate_payload(payload_bytes)
    print()
    print_report_title("KNOWN-BAUD ASCII FRAME TEST")
    print(f"KNOWN BAUD/PAIR: {known_text}")
    print(f"FRAME CANDIDATES: {len(frames)}")
    print("TESTING TARGETED FRAME LIST.")
    print(border_line(REPORT_WIDTH))
    for index, settings in enumerate(frames, start=1):
        try:
            result = run_bank2_candidate(
                serial_module=serial_module,
                index=index,
                total=len(frames),
                settings=settings,
                options=bank_options,
                payload=ascii_payload,
                logger=logger,
            )
        except KeyboardInterrupt:
            action = prompt_operator_break_action("KNOWN-BAUD ASCII TEST")
            if action == "resume":
                continue
            break
        ascii_results.append(result)
        if isinstance(result.settings, DualSerialSettings):
            print(format_dual_progress(result), flush=True)
        else:
            print(format_progress(result), flush=True)

    eight_bit_results: list[CandidateResult] = []
    if ascii_results:
        likely_results = likely_bank2_ascii_results(ascii_results)
        eight_targets = likely_results or sorted(ascii_results, key=result_sort_key, reverse=True)[:1]
        eight_targets_clean = bool(likely_results)
        # TRADEOFF: Running the high-bit challenge against the best non-clean row
        # keeps diagnostics useful, but follow-up probes still require a clean
        # target so reports do not recommend a fallback frame.
        eight_payload_bytes = max(
            DEFAULT_EIGHT_BIT_PAYLOAD_BYTES,
            payload_bytes,
            minimum_eight_bit_payload_size(),
        )
        eight_payload = generate_eight_bit_payload(eight_payload_bytes)
        print()
        print_report_title("KNOWN-BAUD 8-BIT CHALLENGE")
        if eight_targets_clean:
            print(f"TARGETS: {len(eight_targets)} CLEAN ASCII FRAME(S).")
            print("ASCII PASS ALONE DOES NOT PROVE THIS PHASE.")
        else:
            print(f"TARGETS: {len(eight_targets)} BEST NON-CLEAN FRAME(S).")
            print("NO CLEAN ASCII FRAME FOUND; 8-BIT RESULT IS DIAGNOSTIC ONLY.")
        print(border_line(REPORT_WIDTH))
        for index, prior in enumerate(eight_targets, start=1):
            result = run_bank2_candidate(
                serial_module=serial_module,
                index=index,
                total=len(eight_targets),
                settings=prior.settings,
                options=dataclasses.replace(
                    bank_options,
                    payload_bytes=eight_payload_bytes,
                    read_timeout=max(bank_options.read_timeout, 2.0),
                ),
                payload=eight_payload,
                logger=logger,
            )
            eight_bit_results.append(result)
            if isinstance(result.settings, DualSerialSettings):
                print(format_dual_progress(result), flush=True)
            else:
                print(format_progress(result), flush=True)
            if result.status == "eight-bit-not-clean":
                if eight_targets_clean:
                    print("ASCII TRANSFER PASSES, BUT 8-BIT PATH IS NOT CLEAN.")
                else:
                    print("NO CLEAN ASCII PASS; 8-BIT CHECK IS DIAGNOSTIC ONLY.")

    behavior_results: list[Bank2BehaviorProbeResult] = []
    followup_target = best_bank2_followup_target(ascii_results, eight_bit_results)
    if run_behavior:
        behavior_targets = bank2_behavior_targets(ascii_results, eight_bit_results)
        if behavior_targets:
            behavior_results = run_bank2_behavior_probes(
                serial_module=serial_module,
                options=bank_options,
                targets=behavior_targets,
                observe_seconds=job_timeout_observe,
                logger=logger,
            )
        else:
            print()
            print_report_title("KNOWN-BAUD RAW BYTE-BEHAVIOR PROBES")
            print("SKIPPED: NO CLEAN ASCII TARGET WAS FOUND.")
            print(border_line(REPORT_WIDTH))

    etx_ack_results: list[Bank2EtxAckProbeResult] = []
    if run_etx_ack:
        etx_ack_results = run_bank2_etx_ack_probes(
            serial_module=serial_module,
            options=bank_options,
            target=followup_target,
            observe_seconds=etx_ack_observe,
            logger=logger,
        )

    flow_results: list[FlowControlValidationResult] = []
    flow_skip_reason = bank2_flow_skip_reason(followup_target, input_baud, output_baud)
    if not bank_options.auto_validate_flow_control:
        flow_skip_reason = "FLOW TESTS DISABLED IN SCAN / VALIDATE SETUP"
        print()
        print_report_title("KNOWN-BAUD FLOW VALIDATION")
        print(f"SKIPPED: {terminal_text(flow_skip_reason)}.")
        print(border_line(REPORT_WIDTH))
        logger.info("Known-baud flow validation skipped: disabled by operator")
    elif flow_skip_reason:
        print()
        print_report_title("KNOWN-BAUD FLOW VALIDATION")
        print(f"SKIPPED: {terminal_text(flow_skip_reason)}.")
        print("RESULT: NOT OBSERVABLE IN THIS KNOWN-BAUD RUN.")
        print(border_line(REPORT_WIDTH))
        logger.info("Known-baud flow validation skipped: %s", flow_skip_reason)
    else:
        dual_flow_target = bank2_dual_flow_validation_target(followup_target)
        if dual_flow_target is not None:
            matrix_results, _break_action = run_dual_flow_control_validation(
                serial_module=serial_module,
                options=bank_options,
                target=dual_flow_target,
                logger=logger,
            )
            flow_results.extend(matrix_results)

        if followup_target is not None and bank2_settings_same_frame(followup_target.settings):
            hold_results, _break_action = run_flow_control_validation(
                serial_module=serial_module,
                options=bank_options,
                results=ascii_results,
                validation_results=bank2_flow_validation_seed(followup_target),
                logger=logger,
            )
            flow_results.extend(hold_results)
        elif followup_target is not None:
            print()
            print_report_title("KNOWN-BAUD OUTPUT HOLD/RELEASE")
            print("SKIPPED: FOLLOW-UP FRAME IS ASYMMETRIC.")
            print("RESULT: FLOW TRANSFER MATRIX WAS RUN; HOLD/RELEASE NOT PROVEN.")
            print(border_line(REPORT_WIDTH))
            logger.info(
                "Known-baud output hold/release skipped: asymmetric follow-up frame"
            )

        stress_results, _break_action = run_input_backpressure_validation(
            serial_module=serial_module,
            options=bank_options,
            target=followup_target,
            hold_results=flow_hold_release_results(flow_results),
            logger=logger,
        )
        flow_results.extend(stress_results)

    stale_seen = stale_nonce_seen(ascii_results) or stale_nonce_seen(eight_bit_results)
    conclusion = bank2_conclusion(
        ascii_results,
        eight_bit_results,
        flow_results,
        behavior_results,
        etx_ack_results,
    )
    result = Bank2CharacterizationResult(
        switch_note=switch_note,
        known_baud_text=known_text,
        ascii_results=ascii_results,
        eight_bit_results=eight_bit_results,
        flow_results=flow_results,
        behavior_results=behavior_results,
        etx_ack_results=etx_ack_results,
        stale_data_seen=stale_seen,
        conclusion=conclusion,
        run_id=bank_options.run_id,
        flow_skip_reason=flow_skip_reason,
    )
    write_bank2_text_report(bank_options.text_report, result)
    print_bank2_report(result, bank_options.text_report)



def print_commands() -> None:
    """Print the main command menu."""
    def menu_line(left: str = "", right: str = "") -> None:
        """Print one padded command-menu row."""
        inner_width = SCREEN_WIDTH - 4
        if right:
            text = f"{left:<32}  {right:<34}"
        else:
            text = left
        print(f"* {text[:inner_width].ljust(inner_width)} *")

    print()
    print_banner()
    print(bordered_text("MAIN MENU", SCREEN_WIDTH))
    print(border_line(SCREEN_WIDTH))
    menu_line("  1  START SCAN", "  2  SET COM PORTS / BAUD")
    menu_line("  3  SCAN / VALIDATE SETUP", "  4  TIMING / PER-TEST STALE")
    menu_line("  5  CURRENT SETTINGS", "  6  HELP")
    menu_line("  0  QUIT")
    print(border_line(SCREEN_WIDTH))


def interactive_menu(options: ScanOptions | None = None) -> MenuSelection | None:
    """Show the command-line style configuration menu."""
    if options is None:
        options = default_scan_options()
    while prompt_loop_active():
        print_commands()
        try:
            choice = read_operator_input("COMMAND (0-6): ").lstrip("\ufeff").strip()
        except EOFError:
            return None

        if choice == "1":
            workflow = prompt_start_scan_workflow()
            if workflow == "menu":
                continue
            scan_options = options
            if start_scan_workflow_uses_baud_range(workflow):
                print()
                print_report_title("START SCAN BAUD RANGE")
                print("SELECT THE BAUD RANGE FOR THIS SCAN.")
                configured_options = configure_baud_range(options, allow_menu=True)
                if configured_options is None:
                    continue
                scan_options = configured_options
            try:
                validate_options(scan_options)
            except ValueError as exc:
                print(f"SETTINGS ERROR: {terminal_text(exc)}")
                continue
            options = scan_options
            return MenuSelection("scan", options, workflow)
        if choice == "2":
            options = configure_ports(options)
        elif choice == "3":
            options = configure_payload(options)
        elif choice == "4":
            options = configure_timing(options)
        elif choice == "5":
            print_configuration(options)
        elif choice == "6":
            print_menu_help()
        elif choice == "0":
            return None
        else:
            print("ENTER A NUMBER FROM 0 THROUGH 6.")
    return None


def prompt_after_scan_action(
    title: str = "RUN COMPLETE",
    run_again_label: str = "START SCAN AGAIN",
) -> str:
    """Ask what to do after a scan or sweep finishes or stops."""
    print()
    print(border_line(REPORT_WIDTH))
    print(bordered_text(title, REPORT_WIDTH))
    print(border_line(REPORT_WIDTH))
    print(f"  1 {run_again_label}")
    print("  2 RETURN TO MAIN MENU")
    print("  0 QUIT")
    print(border_line(REPORT_WIDTH))
    while prompt_loop_active():
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
    return "menu"


def run_dual_bank_validation(
    serial_module: Any,
    options: ScanOptions,
    shortlist: Sequence[CandidateResult],
    logger: logging.Logger,
) -> tuple[list[CandidateResult], str | None]:
    """Run a larger validation payload for top dual-bank candidates."""
    if not shortlist:
        return [], None
    run_known_output_purges(
        serial_module=serial_module,
        options=options,
        logger=logger,
        settings_list=[result.settings for result in shortlist],
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
    """Run automated dual-bank discovery where input and output may differ.

    Args:
        serial_module: Imported pyserial module or test double.
        options: Active scan options.
        logger: Logger for scan diagnostics.
        phase0_only: When true, run only the fixed-frame baud-liveness matrix
            and write the Phase 0 report.

    Returns:
        Process-style status code: zero for completed workflow, one for no
        selected baud pairs, or `OPERATOR_BREAK_EXIT_CODE` when interrupted and
        reported.

    Notes:
        Full frame-pair expansion is a fallback. The normal path is Phase 0
        baud-pair liveness, output-frame sweep, input-frame sweep, and optional
        flow expansion around the best observed frame pair.
    """
    workflow_started_at = dt.datetime.now(dt.timezone.utc).isoformat()
    if not phase0_only:
        turbo_enabled = prompt_yes_no_question(
            "Turbo discovery mode?",
            options.turbo_discovery_enabled,
        )
        switch_note = prompt_text(
            "Device/switch note for report",
            options.switch_note,
        )
        options = dataclasses.replace(
            options,
            turbo_discovery_enabled=turbo_enabled,
            switch_note=switch_note,
            run_id=make_run_id("D"),
        )
    else:
        options = dataclasses.replace(options, run_id=make_run_id("D"))
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
        write_phase0_text_report(
            options.text_report,
            {
                "workflow": "PHASE 0 BAUD LIVENESS ONLY",
                "started_at": workflow_started_at,
                "completed_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                "run_id": options.run_id,
                "in_port": options.in_port,
                "out_port": options.out_port,
                "switch_note": options.switch_note,
                "outcome": "FULL DUAL-BANK SCAN WAS NOT RUN.",
            },
            phase0_report,
        )
        print_wrapped_value("  TEXT REPORT: ", f"{options.text_report} (CURRENT SESSION)")
        print_wrapped_value("  DEBUG LOG:   ", options.log_file)
        print(border_line(REPORT_WIDTH))
        return 0

    selected_pairs = phase0_report.selected_pairs
    if not selected_pairs:
        # WHY: Phase 0 is deliberately conservative. For a narrow baud range, the
        # operator may still choose a same-baud frame fallback rather than lose a
        # useful manual diagnostic path.
        selected_pairs, fallback_reason = prompt_phase0_no_signal_fallback(
            options,
            phase0_report,
        )
        phase0_report = dataclasses.replace(
            phase0_report,
            selected_pairs=selected_pairs,
            fallback_reason=fallback_reason or phase0_report.fallback_reason,
        )
    if not selected_pairs:
        print()
        print_report_title("DUAL BANK SCAN SKIPPED")
        print("NO BAUD PAIRS WERE SELECTED FOR FRAME TESTING.")
        if phase0_report.fallback_reason:
            print_wrapped_value("  DETAIL:               ", phase0_report.fallback_reason)
        write_phase0_text_report(
            options.text_report,
            {
                "workflow": "AUTOMATED DISCOVERY",
                "started_at": workflow_started_at,
                "completed_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                "run_id": options.run_id,
                "in_port": options.in_port,
                "out_port": options.out_port,
                "switch_note": options.switch_note,
                "outcome": "NO BAUD PAIRS WERE SELECTED FOR FRAME TESTING.",
            },
            phase0_report,
        )
        print_wrapped_value("  TEXT REPORT:          ", f"{options.text_report} (CURRENT SESSION)")
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
    # TRADEOFF: The full matrix remains the correctness fallback, but staged
    # candidates are estimated and run first because the Cartesian frame-pair
    # space is expensive on real low-baud hardware.
    full_candidates = unique_dual_candidates(full_candidates)
    staged_preview_candidates = unique_dual_candidates(staged_preview_candidates)
    candidate_total = len(full_candidates) + staged_flow_candidate_count
    staged_total = len(staged_preview_candidates) + staged_flow_candidate_count
    payload = generate_payload(options.payload_bytes)
    phase0_purge_settings = [
        dual_phase0_settings(input_baud, output_baud).output_settings
        for input_baud, output_baud in selected_pairs
    ]
    run_known_output_purges(
        serial_module=serial_module,
        options=options,
        logger=logger,
        settings_list=phase0_purge_settings,
        reason="CLEAR PHASE 0 DATA BEFORE DUAL FRAME SCAN.",
    )
    scan_started = time.monotonic()
    started_at = dt.datetime.now(dt.timezone.utc).isoformat()

    print()
    print_report_title("DUAL BANK SCAN START")
    print("MODEL: ADVANCED PC INPUT/OUTPUT PORT SETTINGS MAY DIFFER.")
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
    print_wrapped_value(
        "PORTS: ",
        (
            f"{options.in_port} >> BUFFER >> {options.out_port}; "
            f"TEST={payload.byte_count} BYTES X {options.bursts}"
        ),
    )
    print("DISCOVERY ORDER: PHASE 0 FRAME, OUTPUT SWEEP, INPUT SWEEP.")
    if dual_flow_discovery_enabled:
        print("FLOW DISCOVERY: INPUT/OUTPUT FLOW COMBINATIONS AFTER BEST FRAME.")
    else:
        print("FLOW DISCOVERY: OFF.")
    print("FULL MATRIX RUNS ONLY IF STAGED DISCOVERY DOES NOT FIND A GOOD PAIR.")
    print("FRAME SWEEPS USE FLOW=NONE; FLOW SWEEP TESTS HANDSHAKE SETTINGS.")
    if options.auto_validate_top_matches:
        print(f"VALIDATION: ON; SIZE={options.validate_size_1_bytes} BYTES.")
    else:
        print("VALIDATION: OFF.")
    print_progress_legend()
    print_wrapped_value("REPORT: ", f"{options.text_report} (CURRENT SESSION)")
    print_wrapped_value("DEBUG LOG: ", options.log_file)
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
            candidate.output_settings,
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
            candidate.output_settings,
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
        """Run one staged dual-bank candidate sequence."""
        nonlocal early_stopped, operator_break_action, operator_break_stage
        unique_sequence = [
            dual_settings for dual_settings in unique_dual_candidates(sequence)
            if dual_settings not in tested_settings
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
            dual_settings = unique_sequence[index]
            display_index = len(results) + 1
            try:
                candidate_result = run_dual_candidate(
                    serial_module=serial_module,
                    index=display_index,
                    total=candidate_total,
                    settings=dual_settings,
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
                        dual_settings.label(),
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
            results.append(candidate_result)
            stage_results.append(candidate_result)
            tested_settings.add(dual_settings)
            print(format_dual_progress(candidate_result), flush=True)
            print(
                format_scan_eta(len(results), candidate_total, scan_started),
                flush=True,
            )
            if options.ask_on_top_match and is_top_match_result(candidate_result):
                print()
                print_report_title("DUAL TOP MATCH FOUND")
                print_dual_result_details(candidate_result)
                print("    CONTINUE TO LOOK FOR POSSIBLE TIES.")
                print("    ENTER N TO END NOW AND WRITE REPORT.")
                print(border_line(REPORT_WIDTH))
                if not prompt_yes_no("CONTINUE DUAL SCAN", True):
                    early_stopped = True
                    operator_break_stage = stage_name
                    logger.info(
                        "operator ended %s after top match: %s",
                        stage_name.lower(),
                        dual_settings.label(),
                    )
                    break
            index += 1
        return stage_results

    for input_baud, output_baud in selected_pairs:
        if early_stopped or operator_break_action is not None:
            break
        seed = dual_phase0_settings(input_baud, output_baud)
        # WHY: Decode output first while the input side stays at Phase 0 framing,
        # then sweep input against the best output frame. This cuts most obvious
        # misses before falling back to the full matrix.
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
        "run_id": options.run_id,
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
    print_wrapped_value("  TEXT REPORT: ", f"{options.text_report} (CURRENT SESSION)")
    print_wrapped_value("  DEBUG LOG:   ", options.log_file)
    print(border_line(REPORT_WIDTH))
    if operator_break_action == "menu":
        raise ReturnToMainMenuAfterReport()
    if operator_break_action == "quit":
        raise QuitProgramAfterReport()
    if operator_break_action is not None:
        return OPERATOR_BREAK_EXIT_CODE
    return 0


def run_scan(options: ScanOptions, workflow: str | None = None) -> int:
    """Prompt for and run the selected start-scan workflow."""
    if workflow is None:
        workflow = prompt_start_scan_workflow()
        if start_scan_workflow_uses_baud_range(workflow):
            print()
            print_report_title("START SCAN BAUD RANGE")
            print("SELECT THE BAUD RANGE FOR THIS SCAN.")
            configured_options = configure_baud_range(options, allow_menu=True)
            if configured_options is None:
                raise ReturnToMainMenu()
            options = configured_options
    if workflow == "menu":
        raise ReturnToMainMenu()
    validate_options(options)
    if workflow == "bank2":
        run_second_bank_characterization(options)
        return 0

    logger = setup_logging(options.log_file)
    serial_module = import_pyserial()
    return run_dual_bank_scan(
        serial_module=serial_module,
        options=options,
        logger=logger,
        phase0_only=(workflow == "phase0"),
    )


def assert_approx(value: float, expected: float, tolerance: float, label: str) -> None:
    """Raise AssertionError if a numeric self-test value is out of range."""
    if abs(value - expected) > tolerance:
        raise AssertionError(f"{label}: got {value:.3f}, expected {expected:.3f}")


def fake_clean_candidate_result(
    settings: SerialSettings | DualSerialSettings,
    payload: ProbePayload,
) -> CandidateResult:
    """Return a clean in-memory candidate result for self-tests."""
    score = score_received(payload.data, payload.data)
    return CandidateResult(
        index=1,
        total=1,
        settings=settings,
        bytes_sent=payload.byte_count,
        bytes_received=payload.byte_count,
        bytes_drained_before=0,
        score=score.score,
        repeatability=1.0,
        status="exact",
        error=None,
        elapsed_sec=0.0,
        timing=zero_timing_breakdown(),
        metrics=score.metrics,
        trials=[],
        evidence=score.evidence,
    )


def run_self_tests() -> int:
    """Run pure-Python self-tests that do not require serial hardware."""
    print("RUNNING SERIAL PROBE SELF-TESTS")

    def expect_value_error(func: Callable[[], object], label: str) -> None:
        """Assert that a callable rejects invalid input with ValueError."""
        try:
            func()
        except ValueError:
            return
        raise AssertionError(label)

    nonce_a = ProbeNonce("RUNTEST", "CAND_A", "TRIAL_1", "NOTE0001")
    nonce_b = ProbeNonce("RUNTEST", "CAND_B", "TRIAL_1", "NOTE0001")
    payload_a = generate_payload(512, nonce_a)
    payload_b = generate_payload(512, nonce_b)

    score_a = score_received(payload_a.data, payload_a.data)
    assert score_a.score == 100.0, "nonced exact payload did not score 100"
    assert score_a.classification == "exact", "exact payload was not classified exact"

    wrong = score_received(payload_b.data, payload_a.data)
    assert wrong.score == 0.0, "wrong nonce must not score above zero"
    assert wrong.classification == "wrong-nonce", "wrong nonce not classified stale"

    eight = generate_eight_bit_payload(512, nonce_a)
    eight_score = score_received(eight.data, eight.data)
    assert eight_score.score == 100.0, "eight-bit exact payload did not score 100"
    assert (
        eight_score.classification == "eight-bit-clean"
    ), "eight-bit exact payload was not clean"

    masked = bytes(byte & 0x7F for byte in eight.data)
    masked_score = score_received(eight.data, masked)
    assert masked_score.score < 100.0, "masked eight-bit payload scored perfect"
    assert (
        masked_score.classification == "eight-bit-masked"
    ), "masked high-bit payload was not classified masked"
    assert (
        masked_score.metrics.high_bit_stripped_count > 0
    ), "masked high-bit stripped bytes were not counted"

    phase0_template = generate_phase0_payload()
    assert (
        phase0_payload_bytes() >= phase0_minimum_payload_size(representative_nonce())
    ), "Phase 0 liveness payload must fit nonce fields"
    assert (
        phase0_payload_bytes() > PHASE0_PAYLOAD_BYTES
    ), "Phase 0 liveness payload must include nonce capacity"
    assert (
        phase0_template.byte_count == phase0_payload_bytes()
    ), "Phase 0 template size changed"
    phase0_trial = payload_for_trial(
        template=phase0_template,
        settings=dual_phase0_settings(1200, 1200),
        candidate_index=1,
        burst_index=1,
        run_id="DTEST",
        switch_hash=None,
    )
    assert phase0_trial is not phase0_template, "Phase 0 trial should be nonced"
    assert phase0_trial.nonce is not None, "Phase 0 trial nonce missing"
    assert b"RUN=DTEST" in phase0_trial.data, "Phase 0 payload missing run nonce"
    assert (
        phase0_trial.byte_count == phase0_template.byte_count
    ), "Phase 0 trial should keep fixed payload size"
    phase0_score = score_received(phase0_trial.data, phase0_trial.data)
    assert phase0_score.score == 100.0, "Phase 0 exact payload did not score 100"
    stale_phase0_trial = payload_for_trial(
        template=phase0_template,
        settings=dual_phase0_settings(1200, 1200),
        candidate_index=1,
        burst_index=1,
        run_id="DOLD",
        switch_hash=None,
    )
    stale_phase0_score = score_received(phase0_trial.data, stale_phase0_trial.data)
    assert stale_phase0_score.score == 0.0, "stale Phase 0 nonce must score zero"
    assert (
        stale_phase0_score.classification == "wrong-nonce"
    ), "stale Phase 0 nonce not classified stale"
    phase0_options = phase0_scan_options(default_scan_options())
    phase0_1200_timing = effective_discovery_timing(
        phase0_options,
        phase0_baseline_settings(1200),
        phase0_payload_bytes(),
    )
    assert (
        phase0_1200_timing.read_timeout > PHASE0_READ_TIMEOUT
    ), "Phase 0 1200 baud read timeout was not expanded"
    assert (
        phase0_1200_timing.pre_drain_timeout > PHASE0_PRE_DRAIN_TIMEOUT
    ), "Phase 0 1200 baud pre-drain timeout was not expanded"
    phase0_2400_timing = effective_discovery_timing(
        phase0_options,
        phase0_baseline_settings(2400),
        phase0_payload_bytes(),
    )
    assert (
        phase0_2400_timing.pre_drain_timeout > PHASE0_PRE_DRAIN_TIMEOUT
    ), "Phase 0 2400 baud pre-drain timeout was not expanded"
    phase0_4800_timing = effective_discovery_timing(
        phase0_options,
        phase0_baseline_settings(4800),
        phase0_payload_bytes(),
    )
    assert (
        phase0_4800_timing.read_timeout == PHASE0_READ_TIMEOUT
    ), "Phase 0 4800 baud timing should stay at the fast baseline"

    frame_1200 = SerialSettings(1200, 8, "none", 1, "none")
    frame_300 = SerialSettings(300, 8, "none", 1, "none")
    default_options = default_scan_options()
    default_1200_timing = effective_discovery_timing(
        default_options,
        frame_1200,
        DEFAULT_PAYLOAD_BYTES,
    )
    assert (
        default_1200_timing.read_timeout == DEFAULT_READ_TIMEOUT
    ), "general low-baud timing should not change normal read quiet"
    assert (
        default_1200_timing.pre_drain_quiet > DEFAULT_PRE_DRAIN_QUIET
    ), "general 1200 baud pre-drain quiet was not expanded"
    assert (
        default_1200_timing.pre_drain_timeout > DEFAULT_PRE_DRAIN_TIMEOUT
    ), "general 1200 baud pre-drain timeout was not expanded"
    default_2400_timing = effective_discovery_timing(
        default_options,
        SerialSettings(2400, 8, "none", 1, "none"),
        DEFAULT_PAYLOAD_BYTES,
    )
    assert (
        default_2400_timing.read_timeout == DEFAULT_READ_TIMEOUT
    ), "general 2400 baud timing should not change normal read quiet"
    assert (
        default_2400_timing.pre_drain_quiet > DEFAULT_PRE_DRAIN_QUIET
    ), "general 2400 baud pre-drain quiet was not expanded"
    assert (
        default_2400_timing.pre_drain_timeout > DEFAULT_PRE_DRAIN_TIMEOUT
    ), "general 2400 baud pre-drain timeout was not expanded"
    default_4800_timing = effective_discovery_timing(
        default_options,
        SerialSettings(4800, 8, "none", 1, "none"),
        DEFAULT_PAYLOAD_BYTES,
    )
    assert (
        default_4800_timing.pre_drain_timeout == DEFAULT_PRE_DRAIN_TIMEOUT
    ), "4800 baud should stay at the normal pre-drain baseline"
    assert_approx(
        estimated_buffer_drain_seconds(frame_1200, safety_factor=1.0),
        (BUFFER_PURGE_CAPACITY_BYTES * 10) / 1200,
        0.5,
        "1200 baud drain estimate",
    )
    assert_approx(
        estimated_buffer_drain_seconds(frame_300, safety_factor=1.0),
        (BUFFER_PURGE_CAPACITY_BYTES * 10) / 300,
        0.5,
        "300 baud drain estimate",
    )

    expected_bank2_frames = [
        "8N1",
        "7E1",
        "7O1",
        "7M1",
        "7S1",
        "8E1",
        "8O1",
        "8M1",
        "8S1",
        "7N1",
        "7E2",
        "7O2",
        "7M2",
        "7S2",
        "7N2",
        "8N2",
        "8E2",
        "8O2",
        "8M2",
        "8S2",
    ]
    actual_bank2_frames = [
        frame_label(settings) for settings in bank2_frame_serial_settings(1200)
    ]
    assert actual_bank2_frames == expected_bank2_frames, "known-baud frame order changed"

    dual_bank2 = bank2_frame_candidates(1200, 4800, independent_frames=True)
    assert len(dual_bank2) == 400, "known-baud independent frame product must be 400"
    assert all(
        isinstance(settings, DualSerialSettings) for settings in dual_bank2
    ), "known-baud pair candidates must be dual settings"

    def has_dual_frame_pair(input_frame: str, output_frame: str) -> bool:
        """Return whether the generated bank contains one frame pair."""
        return any(
            isinstance(candidate_settings, DualSerialSettings)
            and frame_label(candidate_settings.input_settings) == input_frame
            and frame_label(candidate_settings.output_settings) == output_frame
            for candidate_settings in dual_bank2
        )

    assert has_dual_frame_pair("7E1", "8N1"), "missing mixed 7E1 >> 8N1 pair"
    assert has_dual_frame_pair("8N1", "7O2"), "missing mixed 8N1 >> 7O2 pair"
    assert has_dual_frame_pair("7M1", "8S2"), "missing mixed 7M1 >> 8S2 pair"

    legacy_bank2 = bank2_frame_candidates(1200, 1200, independent_frames=False)
    assert len(legacy_bank2) == 20, "legacy same-frame known-baud list must be 20"
    assert all(
        isinstance(settings, SerialSettings) for settings in legacy_bank2
    ), "legacy same-frame known-baud list must use SerialSettings"
    assert [
        frame_label(settings) for settings in legacy_bank2
    ] == expected_bank2_frames, "legacy same-frame known-baud order changed"

    dual_settings = DualSerialSettings(
        input_settings=SerialSettings(9600, 8, "none", 1, "none"),
        output_settings=SerialSettings(1200, 7, "even", 2, "none"),
    )
    assert (
        transmit_side_settings(dual_settings) == dual_settings.input_settings
    ), "transmit side did not return dual input settings"
    assert (
        receive_side_settings(dual_settings) == dual_settings.output_settings
    ), "receive side did not return dual output settings"
    assert_approx(
        estimated_receive_seconds(dual_settings, 120),
        (120 * 11) / 1200,
        0.0001,
        "dual receive estimate",
    )
    assert (
        frame_or_pair_label(dual_settings) == "IN 8N1 >> OUT 7E2"
    ), "dual frame label did not include both sides"

    validate_port_name("COM1")
    validate_port_name(r"\\.\COM10")
    expect_value_error(lambda: validate_port_name(""), "blank COM port accepted")
    expect_value_error(lambda: validate_port_name("COM0"), "COM0 accepted")
    expect_value_error(lambda: validate_port_name("LPT1"), "non-COM port accepted")
    expect_value_error(lambda: validate_port_name("COM999"), "out-of-range COM port accepted")
    ensure_distinct_ports("COM1", "COM5")
    expect_value_error(
        lambda: ensure_distinct_ports("COM1", r"\\.\COM1"),
        "same COM port accepted",
    )
    validate_supported_baud(9600)
    validate_supported_baud(2400)
    expect_value_error(lambda: validate_supported_baud(12345), "unsupported baud accepted")
    validate_report_path(Path("serial_probe_report.txt"))
    expect_value_error(lambda: validate_report_path(Path(".")), "directory report path accepted")
    expect_value_error(
        lambda: validate_report_path(Path("BAD:NAME.TXT")),
        "invalid report path character accepted",
    )
    expect_value_error(
        lambda: validate_report_path(Path("__missing_report_dir__") / "report.txt"),
        "missing report directory accepted",
    )

    assert parse_start_scan_workflow_choice("") == "discovery", "blank workflow must default to discovery"
    assert parse_start_scan_workflow_choice("1") == "discovery", "workflow 1 parse failed"
    assert parse_start_scan_workflow_choice("2") == "bank2", "workflow 2 parse failed"
    assert parse_start_scan_workflow_choice("known-baud") == "bank2", "known-baud workflow parse failed"
    assert parse_start_scan_workflow_choice("3") == "phase0", "workflow 3 parse failed"
    assert parse_start_scan_workflow_choice("4") == "menu", "workflow 4 must return to menu"
    assert parse_start_scan_workflow_choice("main") == "menu", "workflow menu alias parse failed"
    assert parse_start_scan_workflow_choice("bad") is None, "invalid workflow choice should not parse"

    narrow_options = dataclasses.replace(
        default_scan_options(),
        min_baud=38400,
        max_baud=38400,
    )
    midrange_options = dataclasses.replace(
        default_scan_options(),
        min_baud=1200,
        max_baud=9600,
    )
    assert same_baud_fallback_pairs(narrow_options) == [
        (38400, 38400)
    ], "single-baud fallback pair missing"
    assert same_baud_fallback_pairs(midrange_options) == [
        (9600, 9600),
        (4800, 4800),
        (2400, 2400),
        (1200, 1200),
    ], "same-baud fallback pairs must follow scan order"
    assert phase0_no_signal_fallback_default(1), "single-baud fallback should default on"
    assert phase0_no_signal_fallback_default(2), "two-baud fallback should default on"
    assert not phase0_no_signal_fallback_default(3), "wide fallback should require consent"

    def fake_phase0_row(
        input_baud: int,
        output_baud: int,
        *,
        error: str | None = None,
        row_status: str = "no-data",
    ) -> DualBaudLivenessResult:
        """Build a compact Phase 0 row for fallback-path assertions."""
        return DualBaudLivenessResult(
            input_baud=input_baud,
            output_baud=output_baud,
            alive=False,
            reason="SERIAL ERROR" if error else "NO DATA",
            settings=dual_phase0_settings(input_baud, output_baud),
            score=0.0,
            status=row_status,
            error=error,
            bytes_sent=0,
            bytes_received=0,
            bytes_drained_before=0,
            elapsed_sec=0.0,
            metrics=aggregate_metrics([]),
        )

    error_phase0_report = DualBaudLivenessReport(
        ran=True,
        tested_pairs=[(38400, 38400)],
        total_pairs=1,
        alive_pairs=[],
        selected_pairs=[],
        fallback_reason=None,
        elapsed_sec=0.0,
        results=[fake_phase0_row(38400, 38400, error="COM5 BUSY", row_status="error")],
    )
    assert phase0_results_all_serial_errors(
        error_phase0_report
    ), "all-error Phase 0 report was not recognized"
    assert (
        first_phase0_error(error_phase0_report) == "COM5 BUSY"
    ), "first Phase 0 error detail was not preserved"
    mixed_phase0_report = dataclasses.replace(
        error_phase0_report,
        results=[
            fake_phase0_row(38400, 38400, error="COM5 BUSY", row_status="error"),
            fake_phase0_row(19200, 19200),
        ],
    )
    assert not phase0_results_all_serial_errors(
        mixed_phase0_report
    ), "mixed Phase 0 outcomes should not be classified as all serial errors"

    asym_followup = DualSerialSettings(
        SerialSettings(38400, 8, "even", 1, "none"),
        SerialSettings(38400, 8, "none", 1, "none"),
    )
    same_followup = DualSerialSettings(
        SerialSettings(38400, 8, "even", 1, "none"),
        SerialSettings(38400, 8, "even", 1, "none"),
    )
    asym_eight = dataclasses.replace(
        fake_clean_candidate_result(asym_followup, eight),
        index=1,
    )
    same_eight = dataclasses.replace(
        fake_clean_candidate_result(same_followup, eight),
        index=2,
    )
    preferred = best_bank2_followup_target([], [asym_eight, same_eight])
    assert preferred is not None, "known-baud follow-up target was not selected"
    assert (
        preferred.settings == same_followup
    ), "known-baud target did not prefer same-frame 8E1 over asymmetric 8-bit clean"
    assert (
        bank2_flow_skip_reason(preferred, 38400, 38400) is None
    ), "same-frame known-baud target incorrectly skipped flow validation"
    assert (
        bank2_flow_skip_reason(asym_eight, 38400, 38400) is None
    ), "asymmetric known-baud target should use dual flow validation"
    dual_flow_settings = DualSerialSettings(
        dataclasses.replace(asym_followup.input_settings, flow_control="dsr/dtr"),
        dataclasses.replace(asym_followup.output_settings, flow_control="none"),
    )
    dual_flow_row = flow_validation_result_from_candidate(
        dual_flow_control_code(dual_flow_settings),
        "dual-transfer",
        fake_clean_candidate_result(dual_flow_settings, payload_a),
        payload_a,
    )
    assert (
        dual_flow_row.flow_control == "DSR/OFF"
    ), "dual flow table code should show input/output flow controls"
    assert (
        "CLEAN DUAL FLOW TRANSFER"
        in flow_control_validation_recommendation([dual_flow_row])
    ), "dual flow transfer summary should not be reported as skipped"
    weak_followup = dataclasses.replace(
        fake_clean_candidate_result(same_followup, payload_a),
        status="weak",
        score=10.0,
    )
    assert (
        best_bank2_followup_target([weak_followup], []) is None
    ), "weak known-baud rows must not become follow-up frames"
    assert (
        "NO CLEAN" in bank2_followup_target_label([weak_followup], [])
    ), "unclean known-baud follow-up label should explain why no frame is shown"
    no_clean_bank2 = Bank2CharacterizationResult(
        switch_note="",
        known_baud_text="IN 38400 / OUT 19200",
        ascii_results=[weak_followup],
        eight_bit_results=[],
        flow_results=[],
        behavior_results=[],
        etx_ack_results=[],
        stale_data_seen=False,
        conclusion=bank2_conclusion([weak_followup], [], [], [], []),
        run_id="KBTEST",
        flow_skip_reason=None,
    )
    assert (
        no_clean_bank2.conclusion == "No working frame found for selected known bauds"
    ), "no-clean known-baud conclusion should say no working frame was found"
    assert (
        "NO WORKING SERIAL SETTING FOUND" in bank2_setup_verdict(no_clean_bank2)
    ), "no-clean known-baud verdict should be explicit"
    assert (
        "DO NOT USE A FALLBACK FRAME" in bank2_next_action(no_clean_bank2)
    ), "no-clean known-baud next action should reject fallback frames"
    assert (
        bank2_flow_skip_reason(None, 38400, 19200)
        == "No clean follow-up frame was found"
    ), "known-baud flow skip reason should prefer missing clean target"
    assert "PAIR(S)" in bank2_ascii_pass_summary(
        [fake_clean_candidate_result(same_followup, payload_a)]
    ), "known-baud ASCII summary did not compact dual pairs"

    exact, form_feed, cr_lf, status, _reason = classify_bank2_behavior_bytes(
        b"RAW\r\n",
        b"RAW\r\n",
    )
    assert exact and not form_feed and not cr_lf and status == "exact", "raw exact classification failed"
    behavior_payloads = dict(bank2_behavior_probe_payloads("KBRUN", 1, None))
    assert b"\x1b@" in behavior_payloads["PRINT_CTRL"], "printer ESC reset missing"
    assert b"\x1bE" in behavior_payloads["PRINT_CTRL"], "printer ESC bold missing"
    assert b"\x1bF" in behavior_payloads["PRINT_CTRL"], "printer ESC cancel missing"
    assert (
        bytes(range(0x20, 0x7F)) in behavior_payloads["ASCII_SWEEP"]
    ), "printable ASCII sweep missing"
    assert b"\x11" not in behavior_payloads["CTL7_SAFE"], "safe control sweep included XON"
    assert b"\x13" not in behavior_payloads["CTL7_SAFE"], "safe control sweep included XOFF"
    assert b"\x11" in behavior_payloads["CTL7_FULL"], "full control sweep omitted XON"
    assert b"\x13" in behavior_payloads["CTL7_FULL"], "full control sweep omitted XOFF"
    assert first_mismatch_offset(b"ABCD", b"ABX") == 2, "first mismatch offset was wrong"
    exact, form_feed, cr_lf, status, _reason = classify_bank2_behavior_bytes(
        b"RAW\r\n",
        b"RAW\r\n\x0c",
    )
    assert (
        not exact and form_feed and status == "transformed"
    ), "raw form-feed insertion classification failed"
    exact, form_feed, cr_lf, status, _reason = classify_bank2_behavior_bytes(
        b"A\rB\r",
        b"A\r\nB\r\n",
    )
    assert (
        not exact and not form_feed and cr_lf and status == "transformed"
    ), "raw CR/LF transformation classification failed"

    etx_payload = b"HELLO\x03\r\n"
    etx_seen, etx_exact, ack_seen, reverse_seen, status, _reason = (
        classify_etx_ack_bytes(etx_payload, etx_payload, b"\x06")
    )
    assert (
        etx_seen
        and etx_exact
        and ack_seen
        and reverse_seen
        and status == "reverse-seen"
    ), "ETX/ACK reverse-seen classification failed"
    etx_seen, etx_exact, ack_seen, reverse_seen, status, _reason = (
        classify_etx_ack_bytes(etx_payload, etx_payload, b"")
    )
    assert (
        etx_seen and etx_exact and not ack_seen and not reverse_seen and status == "forward-only"
    ), "ETX/ACK forward-only classification failed"
    etx_seen, etx_exact, ack_seen, reverse_seen, status, _reason = (
        classify_etx_ack_bytes(etx_payload, b"HELLO\r\n", b"")
    )
    assert (
        not etx_seen and not ack_seen and not reverse_seen and status == "etx-missing"
    ), "ETX/ACK missing-ETX classification failed"

    exact_behavior = Bank2BehaviorProbeResult(
        name="CR_ONLY",
        settings=same_followup,
        bytes_sent=5,
        bytes_received=5,
        sent_hash=f"{fnv1a32(b'ABCDE'):08X}",
        received_hash=f"{fnv1a32(b'ABCDE'):08X}",
        first_mismatch_offset=None,
        missing_bytes=0,
        extra_bytes=0,
        exact_match=True,
        form_feed_inserted=False,
        cr_lf_changed=False,
        received_preview_ascii="ABCDE",
        received_preview_hex="41 42 43 44 45",
        status="exact",
        reason="EXACT BYTE MATCH.",
        error=None,
        elapsed_sec=0.0,
    )
    stale_eight = dataclasses.replace(asym_eight, status="wrong-nonce", score=0.0)
    conclusion = bank2_conclusion(
        ascii_results=[fake_clean_candidate_result(same_followup, payload_a)],
        eight_bit_results=[same_eight, stale_eight],
        flow_results=[],
        behavior_results=[exact_behavior],
        etx_ack_results=[],
    )
    assert (
        conclusion == "8-bit clean; raw bytes exact; stale row seen"
    ), "known-baud stale warning incorrectly dominated useful evidence"

    flow_settings = SerialSettings(1200, 8, "none", 1, "xon/xoff")
    clean_flow = fake_clean_candidate_result(flow_settings, payload_a)
    flow_row = flow_validation_result_from_candidate(
        "xon/xoff",
        "large-transfer",
        clean_flow,
        payload_a,
    )
    assert flow_row.status != "validated", "normal transfer incorrectly proved flow"
    assert "NOT OBSERVED" in flow_row.reason, "flow reason did not mark not observed"
    before_lines = {"cts": True, "dsr": True, "cd": True}
    after_dcd_drop = {"cts": True, "dsr": True, "cd": False}
    assert (
        input_backpressure_release_reason(
            "dsr/dtr",
            before_lines,
            after_dcd_drop,
            False,
        )
        == "DCD DROPPED"
    ), "DCD drop should be reported as printer-DTR backpressure evidence"
    assert input_backpressure_observed(
        "DCD DROPPED"
    ), "DCD drop should count as observed input backpressure"

    print("SELF-TESTS PASSED")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Program entry point."""
    enable_terminal_style()
    if sys.version_info < (3, 10):
        print("PYTHON 3.10 OR NEWER IS REQUIRED.")
        return 2
    args = list(sys.argv[1:] if argv is None else argv)
    if any(arg in {"--self-test", "self-test"} for arg in args):
        return run_self_tests()
    if any(arg in {"-h", "--help", "help"} for arg in args):
        print_menu_help(paged=False)
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
            selection = interactive_menu(options)
        except ReturnToMainMenuAfterReport:
            last_status = OPERATOR_BREAK_EXIT_CODE
            continue
        except QuitProgramAfterReport:
            return OPERATOR_BREAK_EXIT_CODE
        if selection is None:
            print("PROGRAM ENDED." if scan_started else "NO SCAN STARTED.")
            return last_status
        options = selection.options
        run_again_label = "START SCAN AGAIN"
        while True:
            try:
                last_status = run_scan(options, selection.workflow)
                scan_started = True
                action = prompt_after_scan_action(
                    run_again_label=run_again_label,
                )
            except ReturnToMainMenu:
                break
            except ReturnToMainMenuAfterReport:
                scan_started = True
                last_status = OPERATOR_BREAK_EXIT_CODE
                break
            except QuitProgramAfterReport:
                return OPERATOR_BREAK_EXIT_CODE
            except KeyboardInterrupt:
                print()
                print("INTERRUPTED BY OPERATOR.")
                scan_started = True
                last_status = OPERATOR_BREAK_EXIT_CODE
                action = prompt_after_scan_action(
                    "TEST INTERRUPTED",
                    run_again_label=run_again_label,
                )
            if action == "rerun":
                continue
            if action == "menu":
                break
            return last_status
    return last_status


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
        print("INTERRUPTED BY OPERATOR.")
        raise SystemExit(OPERATOR_BREAK_EXIT_CODE)
