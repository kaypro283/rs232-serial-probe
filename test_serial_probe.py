import builtins
import logging

import pytest
from pytest import CaptureFixture, MonkeyPatch

import serial_probe


class TypeErrorWriter:
    def write(self, data: bytes) -> int:
        raise TypeError("programming error")


class OSErrorWriter:
    def write(self, data: bytes) -> int:
        raise OSError("serial device unavailable")


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
