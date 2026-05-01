#!/usr/bin/env python3
"""Discover likely serial settings for a serial-to-serial printer buffer.

The script opens an input serial port and an output serial port with matching
candidate settings, sends deterministic ASCII probe payloads, scores what is
received, and reports ranked candidates.
"""

from __future__ import annotations

import atexit
import base64
import csv
import ctypes
import dataclasses
import datetime as dt
import json
import logging
import os
import platform
import statistics
import subprocess
import sys
import threading
import textwrap
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence

BAUD_RATES: list[int] = [
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
EXPLORATORY_PAYLOAD_BYTES = 160
EXPLORATORY_READ_TIMEOUT = 0.4
EXPLORATORY_SETTLE_MS = 20
EXPLORATORY_BURSTS = 1
EXPLORATORY_PROGRESS_INTERVAL = 1.0
EXPLORATORY_PRE_DRAIN_TIMEOUT = 0.5
EXPLORATORY_PRE_DRAIN_QUIET = 0.1
EXPLORATORY_MAX_DRAIN_BYTES = DEFAULT_MAX_DRAIN_BYTES
EXPLORATORY_SHORTLIST_LIMIT = 24
EXPLORATORY_MIN_NARROW_SCORE = 90.0
EXPLORATORY_SCORE_TOLERANCE = 10.0
EXPLORATORY_SUMMARY_ROWS = 8
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
RECOMMENDATION_MIN_SCORE = 90.0
TOP_MATCH_MIN_SCORE = 99.0
TIE_SCORE_TOLERANCE = 0.5
PERFECT_SCORE = 100.0


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
    received_preview_ascii: str
    received_preview_hex: str


@dataclass(frozen=True)
class CandidateResult:
    """Aggregated result for one serial settings candidate."""

    index: int
    total: int
    settings: SerialSettings
    bytes_sent: int
    bytes_received: int
    bytes_drained_before: int
    score: float
    repeatability: float
    status: str
    error: str | None
    elapsed_sec: float
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
    json_report: Path
    csv_report: Path
    log_file: Path
    bursts: int
    progress_interval: float
    no_pre_drain: bool
    pre_drain_timeout: float
    pre_drain_quiet: float
    max_drain_bytes: int
    ask_on_top_match: bool
    auto_validate_top_matches: bool
    validate_size_1_bytes: int
    validate_size_2_tie_bytes: int


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


def estimated_frame_bits(settings: SerialSettings) -> float:
    """Estimate serial frame width in bits per transmitted byte."""
    parity_bits = 0 if settings.parity == "none" else 1
    return 1 + settings.data_bits + parity_bits + settings.stop_bits


def write_chunk_size(settings: SerialSettings) -> int:
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
    print(border_line(SCREEN_WIDTH))
    print(bordered_text("SERIAL PROBE 1.0  -  PRINTER BUFFER SETUP", SCREEN_WIDTH))
    print(border_line(SCREEN_WIDTH))


def border_line(width: int = SCREEN_WIDTH) -> str:
    """Return an asterisk border line."""
    return "*" * width


def bordered_text(text: str, width: int = SCREEN_WIDTH) -> str:
    """Return one centered text line inside an asterisk border."""
    inner_width = max(width - 4, 1)
    cleaned = text[:inner_width]
    return f"* {cleaned.center(inner_width)} *"


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


def console_progress(message: str) -> None:
    """Print a timestamped live progress message."""
    prefix = f"{time.strftime('%H:%M:%S')} "
    width = max(20, 80 - len(prefix))
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
    settings: SerialSettings,
    payload: ProbePayload,
    burst_index: int,
    burst_total: int,
    candidate_index: int,
    candidate_total: int,
    read_timeout: float,
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
    prefix = (
        f"[{candidate_index:04d}/{candidate_total:04d} {settings.label()}] "
        f"TEST {burst_index}/{burst_total}"
    )
    chunk_size = write_chunk_size(settings)

    if progress:
        progress(f"{prefix}: RESET BUFFERS; PAUSE {settle_ms} MS")
    reset_serial_buffers(in_serial)
    reset_serial_buffers(out_serial)
    time.sleep(settle_ms / 1000.0)

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
                elapsed_sec=time.monotonic() - started,
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

            if writer_done.is_set() and (now - last_data_time) >= read_timeout:
                stop_event.set()
                break

    reader_thread = threading.Thread(target=reader, name="serial-probe-reader", daemon=True)
    reader_thread.start()

    bytes_sent = 0
    write_error: str | None = None
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

    if progress:
        progress(
            f"{prefix}: WRITE DONE, SENT={bytes_sent}; "
            f"WAIT {read_timeout:.1f}S QUIET ON {settings.label()}"
        )

    wait_deadline = time.monotonic() + max(read_timeout + (settle_ms / 1000.0) + 2.0, 2.0)
    next_wait_progress_at = time.monotonic() + progress_interval
    while reader_thread.is_alive():
        reader_thread.join(timeout=0.2)
        now = time.monotonic()
        if progress and now >= next_wait_progress_at:
            silence = max(0.0, now - last_data_time)
            progress(
                f"{prefix}: READING RECEIVED={received_length(received, received_lock)} "
                f"BYTES, QUIET={silence:.1f}/{read_timeout:.1f}S"
            )
            next_wait_progress_at = now + progress_interval
        if now >= wait_deadline:
            stop_event.set()
            break

    if reader_thread.is_alive():
        stop_event.set()
        reader_thread.join(timeout=1.0)

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
    settings: SerialSettings,
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
            f"({format_duration(total_estimate)} TOTAL)"
        )
        progress(
            f"[{index:04d}/{total:04d} {settings.label()}] OPEN OUT {options.out_port} "
            f"AND IN {options.in_port}"
        )

    try:
        with open_serial_port(
            serial_module, options.out_port, settings, options.read_timeout
        ) as out_serial:
            with open_serial_port(
                serial_module, options.in_port, settings, options.read_timeout
            ) as in_serial:
                if progress:
                    progress(
                        f"[{index:04d}/{total:04d} {settings.label()}] PORTS OPEN; "
                        f"RESET BUFFERS; PAUSE {options.settle_ms} MS"
                    )
                reset_serial_buffers(out_serial)
                reset_serial_buffers(in_serial)
                time.sleep(options.settle_ms / 1000.0)
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
                        read_timeout=options.read_timeout,
                        settle_ms=options.settle_ms,
                        progress_interval=options.progress_interval,
                        no_pre_drain=options.no_pre_drain,
                        pre_drain_timeout=options.pre_drain_timeout,
                        pre_drain_quiet=options.pre_drain_quiet,
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
        return aggregate_candidate_result(
            index=index,
            total=total,
            settings=settings,
            trials=[],
            elapsed_sec=elapsed,
            opening_error=str(exc),
        )

    elapsed = time.monotonic() - started
    return aggregate_candidate_result(
        index=index,
        total=total,
        settings=settings,
        trials=trials,
        elapsed_sec=elapsed,
    )


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


def write_json_report(
    path: Path,
    metadata: dict[str, Any],
    results: Sequence[CandidateResult],
) -> None:
    """Write the full JSON scan report."""
    path.parent.mkdir(parents=True, exist_ok=True)
    top_results = ranked_top_results(results, metadata["top"])
    payload = {
        "metadata": dataclass_to_jsonable(metadata),
        "top_results": [dataclass_to_jsonable(result) for result in top_results],
        "candidates": [dataclass_to_jsonable(result) for result in results],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def write_csv_report(path: Path, results: Sequence[CandidateResult]) -> None:
    """Write a sortable CSV summary for all candidate results."""
    path.parent.mkdir(parents=True, exist_ok=True)
    ranked = sorted(results, key=result_sort_key, reverse=True)
    fieldnames = [
        "rank",
        "score",
        "repeatability",
        "status",
        "baud",
        "data_bits",
        "parity",
        "stop_bits",
        "flow_control",
        "bytes_sent",
        "bytes_received",
        "old_bytes_cleared",
        "exact_byte_match_ratio",
        "line_integrity_ratio",
        "missing_bytes",
        "extra_bytes",
        "printable_ascii_ratio",
        "length_ratio",
        "elapsed_sec",
        "error",
    ]
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for rank, result in enumerate(ranked, start=1):
            writer.writerow(
                {
                    "rank": rank,
                    "score": f"{result.score:.3f}",
                    "repeatability": f"{result.repeatability:.3f}",
                    "status": result.status,
                    "baud": result.settings.baud,
                    "data_bits": result.settings.data_bits,
                    "parity": result.settings.parity,
                    "stop_bits": result.settings.stop_bits,
                    "flow_control": result.settings.flow_control,
                    "bytes_sent": result.bytes_sent,
                    "bytes_received": result.bytes_received,
                    "old_bytes_cleared": result.bytes_drained_before,
                    "exact_byte_match_ratio": f"{result.metrics.exact_byte_match_ratio:.6f}",
                    "line_integrity_ratio": f"{result.metrics.line_integrity_ratio:.6f}",
                    "missing_bytes": result.metrics.missing_bytes,
                    "extra_bytes": result.metrics.extra_bytes,
                    "printable_ascii_ratio": f"{result.metrics.printable_ascii_ratio:.6f}",
                    "length_ratio": f"{result.metrics.length_ratio:.6f}",
                    "elapsed_sec": f"{result.elapsed_sec:.3f}",
                    "error": result.error or "",
                }
            )


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
    print(f"    SETTING:            {result.settings.label()}")
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
            print(f"    DETAIL:             {result.error}")
    elif result.error:
        print(f"    ERROR:              {result.error}")
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
    print(f"    SETTING:            {result.settings.label()}")
    print(f"    SCORE:              {result.score:.2f}/100")
    print("    CONTINUE TO LOOK FOR POSSIBLE TIES.")
    print("    ENTER N TO END NOW AND WRITE REPORT.")
    print(border_line(REPORT_WIDTH))
    return prompt_yes_no("CONTINUE SCAN", True)


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


def print_scan_summary(
    results: Sequence[CandidateResult],
    total_candidates: int,
    elapsed_sec: float,
    early_stopped: bool,
    top: int,
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
        print("  NOTE:                 OPERATOR ENDED AFTER TOP MATCH.")
        print("                        LATER SETTINGS WERE NOT TESTED FOR TIES.")

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
            print("    NOTE:               OPERATOR ENDED AFTER THIS TOP MATCH.")
            print("                        POSSIBLE LATER TIES WERE NOT TESTED.")
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


def default_report_paths() -> tuple[Path, Path, Path]:
    """Return timestamped default JSON, CSV, and log paths."""
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return (
        Path(f"serial_probe_report_{stamp}.json"),
        Path(f"serial_probe_summary_{stamp}.csv"),
        Path(f"serial_probe_{stamp}.log"),
    )


def default_scan_options() -> ScanOptions:
    """Return practical defaults for the interactive scan."""
    default_json, default_csv, default_log = default_report_paths()
    return ScanOptions(
        in_port="COM1",
        out_port="COM5",
        min_baud=110,
        max_baud=38400,
        payload_bytes=DEFAULT_PAYLOAD_BYTES,
        read_timeout=DEFAULT_READ_TIMEOUT,
        settle_ms=DEFAULT_SETTLE_MS,
        top=15,
        json_report=default_json,
        csv_report=default_csv,
        log_file=default_log,
        bursts=DEFAULT_BURSTS,
        progress_interval=DEFAULT_PROGRESS_INTERVAL,
        no_pre_drain=False,
        pre_drain_timeout=DEFAULT_PRE_DRAIN_TIMEOUT,
        pre_drain_quiet=DEFAULT_PRE_DRAIN_QUIET,
        max_drain_bytes=DEFAULT_MAX_DRAIN_BYTES,
        ask_on_top_match=False,
        auto_validate_top_matches=True,
        validate_size_1_bytes=8 * 1024,
        validate_size_2_tie_bytes=16 * 1024,
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
    per_burst = (
        options.read_timeout
        + (options.settle_ms / 1000.0)
        + (0.0 if options.no_pre_drain else options.pre_drain_quiet)
    )
    return len(candidates) * options.bursts * per_burst


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
        ask_on_top_match=False,
    )


def is_exploratory_signal(result: CandidateResult) -> bool:
    """Return True when an exploratory result is useful enough for narrowing."""
    if result.error:
        return False
    if result.status in {"error", "no-data", "stale-output", "partial-write"}:
        return False
    return result.score >= EXPLORATORY_MIN_NARROW_SCORE


def select_exploratory_candidates(
    results: Sequence[CandidateResult],
    all_candidates: Sequence[SerialSettings],
    elapsed_sec: float,
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
    )


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
    print(f"SETTINGS: {candidate_count}; BROAD LIGHT PASS.")
    print(border_line(REPORT_WIDTH))


def print_exploratory_summary(selection: ExploratorySelection) -> None:
    """Print a concise exploratory finding summary."""
    print()
    print_report_title("QUICK EXPLORATORY RESULTS")
    print(f"  RUN TIME:             {format_duration(selection.elapsed_sec)}")
    print(f"  SETTINGS TESTED:      {len(selection.results)}")
    print(f"  FIXED SETTINGS:       {exploratory_fixed_settings_label()}")
    if selection.fallback_reason:
        print(f"  FINDING:              {selection.fallback_reason}")
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
            print(f"    {note}")
    print(border_line(REPORT_WIDTH))


def exploratory_metadata(
    requested: bool,
    narrowing_accepted: bool,
    selection: ExploratorySelection | None,
    original_candidate_count: int,
    final_candidate_count: int,
) -> dict[str, Any]:
    """Return JSON metadata for exploratory mode and candidate narrowing."""
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
    }
    metadata: dict[str, Any] = {
        "requested": requested,
        "fixed_internal_settings": fixed_settings,
        "narrowing_accepted": narrowing_accepted,
        "original_candidate_count": original_candidate_count,
        "final_candidate_count": final_candidate_count,
        "candidate_source": (
            "exploratory-narrowed"
            if narrowing_accepted
            else "all-selected-settings"
        ),
    }
    if selection is None:
        metadata["ran"] = False
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
            "shortlist_count": len(selection.shortlist_results),
            "narrowed_candidate_count": len(selection.narrowed_candidates),
            "shortlist_settings": [
                dataclass_to_jsonable(result.settings)
                for result in selection.shortlist_results
            ],
            "narrowed_candidate_settings": [
                dataclass_to_jsonable(settings)
                for settings in selection.narrowed_candidates
            ],
            "top_results": [
                dataclass_to_jsonable(result)
                for result in selection.ranked_results[:EXPLORATORY_SUMMARY_ROWS]
            ],
        }
    )
    return metadata


def run_exploratory_scan(
    serial_module: Any,
    options: ScanOptions,
    candidates: Sequence[SerialSettings],
    logger: logging.Logger,
) -> ExploratorySelection:
    """Run quick exploratory mode and return candidate narrowing findings."""
    quick_options = exploratory_scan_options(options)
    quick_payload = generate_payload(quick_options.payload_bytes)
    print_exploratory_start(options, quick_options, len(candidates), quick_payload)
    logger.info("quick exploratory mode started")
    logger.info("quick options: %s", quick_options)
    logger.info(
        "quick payload: %s bytes, %s lines",
        quick_payload.byte_count,
        quick_payload.line_count,
    )

    started = time.monotonic()
    results: list[CandidateResult] = []
    for index, settings in enumerate(candidates, start=1):
        result = run_candidate(
            serial_module=serial_module,
            index=index,
            total=len(candidates),
            settings=settings,
            options=quick_options,
            payload=quick_payload,
            logger=logger,
            progress=None,
        )
        results.append(result)
        print("QUICK " + format_progress(result), flush=True)
        print(format_scan_eta(len(results), len(candidates), started), flush=True)

    elapsed_sec = time.monotonic() - started
    selection = select_exploratory_candidates(results, candidates, elapsed_sec)
    print_exploratory_summary(selection)
    logger.info(
        "quick exploratory completed; candidates=%s fallback=%s shortlist=%s narrowed=%s",
        len(results),
        selection.fallback_reason,
        len(selection.shortlist_results),
        len(selection.narrowed_candidates),
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
            value = input(f"{label.upper()} [{default}]: ").strip().lower()
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
            value = input(f"{question.upper()} (Y/N) [{default}]: ").strip().lower()
        except EOFError:
            return current
        if value == "":
            return current
        if value in {"y", "yes"}:
            return True
        if value in {"n", "no"}:
            return False
        print("ENTER Y OR N.")


def print_menu_help() -> None:
    """Print short help for the interactive CLI."""
    print_banner()
    print("START: PYTHON SERIAL_PROBE.PY")
    print()
    print("HELP")
    print()
    print("PURPOSE")
    print("  FIND SERIAL SWITCH SETTINGS FOR A PRINTER BUFFER.")
    print()
    print("METHOD")
    print("  SEND KNOWN ASCII TEXT TO INPUT PORT.")
    print("  READ OUTPUT PORT.")
    print("  TEST EACH SELECTED SERIAL SETTING.")
    print("  RANK BY MATCH QUALITY.")
    print("  USE 11 MEMORY TEST AFTER A GOOD SETTING IS FOUND.")
    print()
    print("OPERATOR NOTES:")
    print("  TEST SIZE:       BYTES SENT FOR EACH SETTING.")
    print("  TEST COUNT:      NUMBER OF TRIES PER SETTING.")
    print("  QUICK MODE:      ASKED AT SCAN START; FIXED INTERNAL SETTINGS.")
    print("  ASK ON MATCH:    PAUSE AFTER PASS; ASK CONTINUE.")
    print("  CLEAR OUTPUT:    DISCARD OLD BUFFER DATA FIRST.")
    print("  MAX CLEAR:       DEFAULT 32768 BYTES.")
    print("  TOP ROWS:        BEST RESULTS SHOWN AT END.")
    print("  MEMORY TEST:     COMMAND 11 AFTER SCAN.")


def print_setting(label: str, value: object) -> None:
    """Print one aligned current-settings row."""
    prefix = f"  {label.upper():<20} "
    width = max(20, 80 - len(prefix))
    wrapped = textwrap.wrap(str(value), width=width, break_long_words=True) or [""]
    print(prefix + wrapped[0])
    for line in wrapped[1:]:
        print(" " * len(prefix) + line)


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
    print()
    print_banner()
    print("CURRENT SETTINGS")
    print_setting("PORTS:", f"{options.in_port} -> {options.out_port}")
    print_setting("BAUD RANGE:", f"{options.min_baud}..{options.max_baud}")
    print_setting("SETTINGS:", len(candidates))
    if baud_order:
        print_setting("BAUD ORDER:", f"{baud_order[0]} DOWN TO {baud_order[-1]}")
    if range_error:
        print_setting("RANGE ERROR:", range_error)
    print_setting(
        "COUNT FORMULA:",
        f"{len(bauds)} BAUD X "
        f"{len(DATA_BITS)} DATA X {len(PARITIES)} PARITY X "
        f"{len(STOP_BITS)} STOP X {len(FLOW_CONTROLS)} FLOW",
    )
    print_setting("TEST BYTES:", f"{options.payload_bytes} BYTES")
    print_setting("TEST COUNT:", options.bursts)
    print_setting(
        "QUICK MODE:",
        f"ASK AT START; FIXED INTERNAL {exploratory_fixed_settings_label()}",
    )
    print_setting("ASK ON MATCH:", "YES" if options.ask_on_top_match else "NO")
    print_setting(
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
    print_setting("READ WAIT:", f"{options.read_timeout:.2f}S")
    print_setting("OPEN PAUSE:", f"{options.settle_ms} MS")
    print_setting(
        "CLEAR OUTPUT:",
        (
            "NO"
            if options.no_pre_drain
            else (
                f"YES, QUIET={options.pre_drain_quiet:.2f}S, "
                f"LIMIT={options.pre_drain_timeout:.2f}S, "
                f"MAX={options.max_drain_bytes} BYTES"
            )
        )
    )
    print_setting("TOP ROWS:", options.top)
    print_setting("MEMORY TEST:", "USE 11 AFTER SCAN")
    print_setting("JSON FILE:", options.json_report)
    print_setting("CSV FILE:", options.csv_report)
    print_setting("LOG FILE:", options.log_file)
    print_setting("SEND TIME:", format_duration(wire).upper())
    print_setting("WAIT TIME:", f"{format_duration(overhead).upper()} IF QUIET")
    print_setting("TOTAL EST.:", format_duration(wire + overhead).upper())


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
    validate_size_1 = options.validate_size_1_bytes
    validate_size_2 = options.validate_size_2_tie_bytes
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
    return dataclasses.replace(
        options,
        payload_bytes=payload_bytes,
        bursts=bursts,
        ask_on_top_match=ask_on_top_match,
        auto_validate_top_matches=auto_validate,
        validate_size_1_bytes=validate_size_1,
        validate_size_2_tie_bytes=validate_size_2,
    )


def configure_timing(options: ScanOptions) -> ScanOptions:
    """Prompt for timing settings."""
    read_timeout = prompt_float("OUTPUT WAIT AFTER SEND, SECONDS", options.read_timeout, 0.1)
    settle_ms = prompt_int("PAUSE AFTER OPENING PORTS, MS", options.settle_ms, 0)
    progress_interval = prompt_float("SCREEN UPDATE INTERVAL, SECONDS", options.progress_interval, 0.1)
    return dataclasses.replace(
        options,
        read_timeout=read_timeout,
        settle_ms=settle_ms,
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
    json_report = prompt_path("JSON REPORT FILE", options.json_report)
    csv_report = prompt_path("CSV REPORT FILE", options.csv_report)
    log_file = prompt_path("LOG FILE", options.log_file)
    return dataclasses.replace(
        options,
        top=top,
        json_report=json_report,
        csv_report=csv_report,
        log_file=log_file,
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


def memory_report_paths() -> tuple[Path, Path, Path]:
    """Return timestamped memory-test JSON, CSV, and log paths."""
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return (
        Path(f"serial_probe_memory_report_{stamp}.json"),
        Path(f"serial_probe_memory_summary_{stamp}.csv"),
        Path(f"serial_probe_memory_{stamp}.log"),
    )


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
    json_report: Path,
    csv_report: Path,
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
    print(f"    SETTING:           {settings.label()}")
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
    print(f"  JSON FILE: {json_report}")
    print(f"  CSV FILE:  {csv_report}")
    print(f"  LOG FILE:  {log_file}")
    print(border_line(REPORT_WIDTH))


def write_memory_reports(
    json_report: Path,
    csv_report: Path,
    settings: SerialSettings,
    results: Sequence[MemoryTestResult],
) -> None:
    """Write JSON and CSV memory test reports."""
    json_report.parent.mkdir(parents=True, exist_ok=True)
    csv_report.parent.mkdir(parents=True, exist_ok=True)
    json_report.write_text(
        json.dumps(
            {
                "tool": "serial_probe",
                "report": "memory_test",
                "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                "serial_setting": dataclass_to_jsonable(settings),
                "interpretation": memory_test_interpretation(results),
                "results": [dataclass_to_jsonable(result) for result in results],
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    with csv_report.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "size_label",
                "size_bytes",
                "method",
                "indicator",
                "score",
                "bytes_sent",
                "bytes_received",
                "bytes_cleared_before",
                "bytes_seen_before_release",
                "missing_bytes",
                "extra_bytes",
                "elapsed_sec",
                "status",
                "error",
            ],
        )
        writer.writeheader()
        for result in results:
            writer.writerow(
                {
                    "size_label": result.size_label,
                    "size_bytes": result.size_bytes,
                    "method": result.method,
                    "indicator": result.indicator,
                    "score": f"{result.score:.3f}",
                    "bytes_sent": result.bytes_sent,
                    "bytes_received": result.bytes_received,
                    "bytes_cleared_before": result.bytes_cleared_before,
                    "bytes_seen_before_release": result.bytes_seen_before_release,
                    "missing_bytes": result.metrics.missing_bytes,
                    "extra_bytes": result.metrics.extra_bytes,
                    "elapsed_sec": f"{result.elapsed_sec:.3f}",
                    "status": result.status,
                    "error": result.error or "",
                }
            )


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
    json_report, csv_report, log_file = memory_report_paths()
    logger = setup_logging(log_file)
    serial_module = import_or_install_pyserial()
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
    results = [
        run_memory_size_test(
            serial_module=serial_module,
            index=index,
            total=len(sizes),
            size_bytes=size,
            settings=settings,
            options=options,
            logger=logger,
            method=method,
        )
        for index, size in enumerate(sizes, start=1)
    ]
    write_memory_reports(json_report, csv_report, settings, results)
    print_memory_report(settings, results, json_report, csv_report, log_file)


def print_commands() -> None:
    """Print the main command menu."""
    print()
    print_banner()
    print("MAIN MENU")
    print("  1 START SCAN                 7 SET REPORT FILES")
    print("  2 SET COM PORTS              8 MAKE REPORT NAMES")
    print("  3 SET BAUD RANGE             9 CURRENT SETTINGS")
    print("  4 SET TEST SIZE/COUNT       10 HELP")
    print("  5 SET TIMING                11 MEMORY TEST")
    print("  6 CLEAR OLD OUTPUT           0 QUIT")


def interactive_menu() -> ScanOptions | None:
    """Show the command-line style configuration menu."""
    options = default_scan_options()
    while True:
        print_commands()
        try:
            choice = input("ENTER SELECTION: ").strip().lower()
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
            default_json, default_csv, default_log = default_report_paths()
            options = dataclasses.replace(
                options,
                json_report=default_json,
                csv_report=default_csv,
                log_file=default_log,
            )
        elif choice == "9":
            print_configuration(options)
        elif choice == "10":
            print_menu_help()
        elif choice == "11":
            run_memory_test(options)
        elif choice in {"0", "q", "quit", "exit"}:
            return None
        else:
            print("ENTER A NUMBER FROM 0 TO 11.")


def metadata_for_scan(
    options: ScanOptions,
    pyserial_version: str,
    payload: ProbePayload,
    candidate_count: int,
    started_at: str,
    completed_at: str | None = None,
    early_stopped: bool = False,
) -> dict[str, Any]:
    """Build metadata for JSON reporting."""
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
    serial_module = import_or_install_pyserial()
    logger = setup_logging(options.log_file)
    all_candidates = generate_candidates(options.min_baud, options.max_baud)
    candidates = list(all_candidates)
    payload = generate_payload(options.payload_bytes)
    pyserial_version = str(getattr(serial_module, "VERSION", "unknown"))
    logger.info("serial_probe started")
    logger.info("options: %s", options)
    logger.info("payload: %s bytes, %s lines", payload.byte_count, payload.line_count)
    logger.info("candidates: %s", len(all_candidates))

    exploratory_requested = prompt_yes_no_question(
        "Run quick exploratory mode first?",
        False,
    )
    exploratory_selection: ExploratorySelection | None = None
    exploratory_narrowing_accepted = False
    if exploratory_requested:
        exploratory_selection = run_exploratory_scan(
            serial_module=serial_module,
            options=options,
            candidates=all_candidates,
            logger=logger,
        )
        if exploratory_selection.narrowed_candidates:
            exploratory_narrowing_accepted = prompt_yes_no_question(
                "Use these findings to narrow full analysis?",
                False,
            )
            if exploratory_narrowing_accepted:
                candidates = list(exploratory_selection.narrowed_candidates)
                logger.info(
                    "operator accepted exploratory narrowing; full candidates=%s/%s",
                    len(candidates),
                    len(all_candidates),
                )
            else:
                logger.info("operator declined exploratory narrowing")
                print("FULL ANALYSIS WILL TEST ALL SELECTED SETTINGS.")
        else:
            logger.info(
                "exploratory narrowing unavailable: %s",
                exploratory_selection.fallback_reason,
            )
            print("FULL ANALYSIS WILL TEST ALL SELECTED SETTINGS.")
    else:
        logger.info("operator skipped quick exploratory mode")

    scan_started = time.monotonic()
    started_at = dt.datetime.now(dt.timezone.utc).isoformat()

    print_report_title("SCAN START")
    if exploratory_narrowing_accepted:
        print("MODE: FULL ANALYSIS OF QUICK EXPLORATORY SHORTLIST.")
        print(f"SETTINGS: {len(candidates)}/{len(all_candidates)} SELECTED BY QUICK MODE.")
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
    print(f"SCREEN UPDATE: {options.progress_interval:.1f}S WHILE SETTING RUNS.")
    print_progress_legend()
    print(f"REPORTS: {options.json_report}, {options.csv_report}, {options.log_file}")
    rough_total = sum(
        estimated_transmit_seconds(candidate, options.payload_bytes) * options.bursts
        for candidate in candidates
    )
    rough_total += len(candidates) * options.bursts * (
        options.read_timeout
        + (options.settle_ms / 1000.0)
        + (0.0 if options.no_pre_drain else options.pre_drain_quiet)
    )
    print(
        f"START EST.: {format_duration(rough_total)}; "
        f"FINISH ABOUT {format_finish_clock(rough_total)}"
    )

    results: list[CandidateResult] = []
    early_stopped = False
    for index, settings in enumerate(candidates, start=1):
        result = run_candidate(
            serial_module=serial_module,
            index=index,
            total=len(candidates),
            settings=settings,
            options=options,
            payload=payload,
            logger=logger,
            progress=console_progress,
        )
        results.append(result)
        print(format_progress(result), flush=True)
        print(format_scan_eta(len(results), len(candidates), scan_started), flush=True)
        if options.ask_on_top_match and is_top_match_result(result):
            if ask_continue_after_top_match(result):
                logger.info("operator chose to continue after top match: %s", result.settings.label())
            else:
                early_stopped = True
                logger.info("operator ended scan after top match: %s", result.settings.label())
                break

    completed_at = dt.datetime.now(dt.timezone.utc).isoformat()
    elapsed_sec = time.monotonic() - scan_started
    metadata = metadata_for_scan(
        options=options,
        pyserial_version=pyserial_version,
        payload=payload,
        candidate_count=len(candidates),
        started_at=started_at,
        completed_at=completed_at,
        early_stopped=early_stopped,
    )
    metadata["completed_candidates"] = len(results)
    metadata["elapsed_sec"] = elapsed_sec
    metadata["early_stop_reason"] = (
        "operator-ended-after-top-match" if early_stopped else None
    )
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
    metadata["exploratory_mode"] = exploratory_metadata(
        requested=exploratory_requested,
        narrowing_accepted=exploratory_narrowing_accepted,
        selection=exploratory_selection,
        original_candidate_count=len(all_candidates),
        final_candidate_count=len(candidates),
    )
    write_json_report(options.json_report, metadata, results)
    write_csv_report(options.csv_report, results)
    logger.info(
        "serial_probe completed; candidates=%s/%s early_stopped=%s exploratory_narrowed=%s",
        len(results),
        len(all_candidates),
        early_stopped,
        exploratory_narrowing_accepted,
    )
    logger.info("json_report=%s", options.json_report)
    logger.info("csv_report=%s", options.csv_report)

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
    )
    if exploratory_narrowing_accepted:
        print(
            "  NOTE:                 FULL ANALYSIS USED QUICK EXPLORATORY "
            f"CANDIDATES ({len(candidates)}/{len(all_candidates)})."
        )
    elif exploratory_requested:
        print("  NOTE:                 QUICK EXPLORATORY DID NOT NARROW FULL ANALYSIS.")
    if options.auto_validate_top_matches and results:
        ranked = ranked_top_results(results, options.top)
        top_score = ranked[0].score if ranked else 0.0
        shortlist = [result for result in ranked if abs(result.score - top_score) <= 0.0001]
        if shortlist:
            print()
            print_report_title("PHASE 2 VALIDATION")
            print(
                f"SHORTLIST: {len(shortlist)} TOP-SCORE SETTING(S) "
                f"FROM SCORE={top_score:.2f}."
            )
            stage2_options = dataclasses.replace(
                options,
                payload_bytes=options.validate_size_1_bytes,
                bursts=1,
                ask_on_top_match=False,
            )
            stage2_payload = generate_payload(options.validate_size_1_bytes)
            stage2_results: list[CandidateResult] = []
            for index, candidate in enumerate(shortlist, start=1):
                print(
                    f"VALIDATION PASS 1: {stage2_payload.byte_count} BYTES "
                    f"{index}/{len(shortlist)} {candidate.settings.label()}"
                )
                stage2_result = run_candidate(
                    serial_module=serial_module,
                    index=index,
                    total=len(shortlist),
                    settings=candidate.settings,
                    options=stage2_options,
                    payload=stage2_payload,
                    logger=logger,
                    progress=console_progress,
                )
                stage2_results.append(stage2_result)
                print(format_progress(stage2_result), flush=True)
            tied_after_1 = top_tied_results(stage2_results)
            final_stage2_results = stage2_results
            if len(tied_after_1) > 1 and options.validate_size_2_tie_bytes > 0:
                print(
                    f"TIE REMAINS ({len(tied_after_1)}). "
                    f"RUNNING PASS 2 AT {options.validate_size_2_tie_bytes} BYTES."
                )
                stage2_options = dataclasses.replace(
                    stage2_options,
                    payload_bytes=options.validate_size_2_tie_bytes,
                )
                stage2_payload = generate_payload(options.validate_size_2_tie_bytes)
                tie_results: list[CandidateResult] = []
                for index, candidate in enumerate(tied_after_1, start=1):
                    print(
                        f"VALIDATION PASS 2: {stage2_payload.byte_count} BYTES "
                        f"{index}/{len(tied_after_1)} {candidate.settings.label()}"
                    )
                    tie_result = run_candidate(
                        serial_module=serial_module,
                        index=index,
                        total=len(tied_after_1),
                        settings=candidate.settings,
                        options=stage2_options,
                        payload=stage2_payload,
                        logger=logger,
                        progress=console_progress,
                    )
                    tie_results.append(tie_result)
                    print(format_progress(tie_result), flush=True)
                final_stage2_results = tie_results
            elif len(tied_after_1) > 1:
                print("TIE REMAINS AFTER PASS 1. PASS 2 IS OFF.")

            print("STAGE 2 FINAL RANKING:")
            print_ranked_table(
                final_stage2_results,
                min(options.top, len(final_stage2_results)),
                report_title="PHASE 2 FINAL REPORT",
            )
    print()
    print_report_title("REPORT FILES")
    print(f"  JSON FILE: {options.json_report}")
    print(f"  CSV FILE:  {options.csv_report}")
    print(f"  LOG FILE:  {options.log_file}")
    print(border_line(REPORT_WIDTH))
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
    options = interactive_menu()
    if options is None:
        print("NO SCAN STARTED.")
        return 0
    return run_scan(options)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
        print("INTERRUPTED BY OPERATOR.")
        raise SystemExit(130)
