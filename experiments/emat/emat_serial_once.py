#!/usr/bin/env python3
"""Start an EMAT frame or acquire its data in one independent invocation.

This utility is intentionally independent from ROS 2 so a scheduler (systemd,
cron, or another process) can invoke it once per acquisition.  It implements
only the protocol facts currently confirmed:

    start:             0x81 0x01 -> 0x01, then 0x01 -> 0x02
    acquire (x8192):   0x00 -> expected_data_bytes arbitrary bytes
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

try:
    import serial
except ImportError:  # pragma: no cover - depends on the target system
    serial = None


START_COMMAND = b"\x81\x01"
READ_COMMAND = b"\x00"
CLEAR_COMMAND = b"\x01"
START_ACK = b"\x01"
CLEAR_ACK = b"\x02"
ACQUISITION_COUNT = 4000
ACQUISITION_INTERVAL_NS = 10_000_000


class ProtocolError(RuntimeError):
    """A received reply did not match the reply required by the current state."""


class ReadTimeout(TimeoutError):
    """The requested number of bytes did not arrive before the deadline."""


@dataclass(frozen=True)
class TransactionResult:
    data: bytes | None
    latency_us: int
    cleared: bool


def read_exactly(port: BinaryIO, size: int, timeout_s: float) -> bytes:
    """Read exactly *size* bytes, tolerating short and fragmented serial reads."""
    if size < 1:
        raise ValueError("expected_data_bytes must be at least 1")

    deadline = time.monotonic() + timeout_s
    received = bytearray()
    while len(received) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ReadTimeout(f"timed out after receiving {len(received)}/{size} bytes")

        # pyserial honors this timeout on the next read.  Keeping it bounded
        # makes the overall monotonic deadline authoritative even on short reads.
        port.timeout = min(remaining, 0.05)
        chunk = port.read(size - len(received))
        if chunk:
            received.extend(chunk)
    return bytes(received)


def write_exactly(port: BinaryIO, payload: bytes, timeout_s: float) -> None:
    port.write_timeout = timeout_s
    written = port.write(payload)
    if written != len(payload):
        raise OSError(f"short serial write: wrote {written}/{len(payload)} bytes")
    port.flush()


def pause(delay_ms: float) -> None:
    if delay_ms > 0:
        time.sleep(delay_ms / 1000.0)


def trace(enabled: bool, label: str, payload: bytes) -> None:
    """Write protocol trace to stderr so stdout remains usable as data output."""
    if enabled:
        print(f"{label}: {payload.hex()}", file=sys.stderr)


def expect_ack(port: BinaryIO, expected: bytes, timeout_s: float, operation: str, trace_enabled: bool) -> None:
    actual = read_exactly(port, 1, timeout_s)
    trace(trace_enabled, f"RX {operation} ACK", actual)
    if actual != expected:
        raise ProtocolError(
            f"{operation} ACK mismatch: expected {expected.hex()}, got {actual.hex()}"
        )


def run_start(port: BinaryIO, args: argparse.Namespace) -> TransactionResult:
    started_at = time.monotonic_ns()

    # STARTING: only a strict 0x01 is an ACK in this state.
    trace(args.raw_hex_trace, "TX start", START_COMMAND)
    write_exactly(port, START_COMMAND, args.write_timeout_ms / 1000.0)
    expect_ack(port, START_ACK, args.ack_timeout_ms / 1000.0, "start", args.raw_hex_trace)
    pause(args.inter_command_delay_ms)

    cleared = False
    # Every start clears the read address.  Only a strict 0x02 is accepted in
    # CLEARING state; the subsequent acquire starts reading at address zero.
    trace(args.raw_hex_trace, "TX clear", CLEAR_COMMAND)
    write_exactly(port, CLEAR_COMMAND, args.write_timeout_ms / 1000.0)
    expect_ack(port, CLEAR_ACK, args.ack_timeout_ms / 1000.0, "clear", args.raw_hex_trace)
    cleared = True
    pause(args.inter_command_delay_ms)

    latency_us = (time.monotonic_ns() - started_at) // 1_000
    return TransactionResult(data=None, latency_us=latency_us, cleared=cleared)


def run_acquire(port: BinaryIO, args: argparse.Namespace) -> TransactionResult:
    started_at = time.monotonic_ns()
    frames = bytearray()
    previous_acquisition_ns: int | None = None
    # READING: all byte values, including 0x01 and 0x02, are measurement data.
    # One --acquire invocation performs the complete, fixed-size acquisition.
    # Command starts are kept at least 10 ms apart.  The transaction time is
    # included in that interval instead of adding another 10 ms after each read.
    for index in range(ACQUISITION_COUNT):
        if previous_acquisition_ns is not None:
            deadline_ns = previous_acquisition_ns + ACQUISITION_INTERVAL_NS
            remaining_ns = deadline_ns - time.monotonic_ns()
            if remaining_ns > 0:
                time.sleep(remaining_ns / 1_000_000_000)
        trace(args.raw_hex_trace, f"TX acquire {index + 1}/{ACQUISITION_COUNT}", READ_COMMAND)
        previous_acquisition_ns = time.monotonic_ns()
        write_exactly(port, READ_COMMAND, args.write_timeout_ms / 1000.0)
        frame = read_exactly(port, args.expected_data_bytes, args.read_timeout_ms / 1000.0)
        trace(args.raw_hex_trace, f"RX data {index + 1}/{ACQUISITION_COUNT}", frame)
        frames.extend(frame)
    latency_us = (time.monotonic_ns() - started_at) // 1_000
    return TransactionResult(data=bytes(frames), latency_us=latency_us, cleared=False)


def write_hex_data(output_file: Path, data: bytes, frame_size: int) -> None:
    """Write lowercase hexadecimal frames separated by one space."""
    frames = (data[offset:offset + frame_size] for offset in range(0, len(data), frame_size))
    with output_file.open("w", encoding="ascii", newline="") as stream:
        stream.write(" ".join(frame.hex() for frame in frames))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial-port", required=True, help="e.g. /dev/serial/by-id/... (or COMx on Windows)")
    parser.add_argument("--baudrate", type=int, required=True, help="device baud rate (must be confirmed)")
    parser.add_argument("--data-bits", choices=(5, 6, 7, 8), type=int, default=8)
    parser.add_argument("--parity", choices=("none", "even", "odd", "mark", "space"), default="none")
    parser.add_argument("--stop-bits", choices=(1, 1.5, 2), type=float, default=1)
    parser.add_argument("--flow-control", choices=("none", "rtscts", "xonxoff"), default="none")
    parser.add_argument("--read-timeout-ms", type=float, default=500)
    parser.add_argument("--write-timeout-ms", type=float, default=500)
    parser.add_argument("--ack-timeout-ms", type=float, default=500)
    parser.add_argument(
        "--inter-command-delay-ms", type=float, default=800,
        help="delay between protocol commands in milliseconds (default: 800)",
    )
    parser.add_argument("--expected-data-bytes", type=int, default=1)
    parser.add_argument("--raw-hex-trace", action="store_true", help="print TX/RX protocol bytes to stderr for debugging")
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument(
        "--start", dest="mode", action="store_const", const="start",
        help="send start then clear read address",
    )
    mode_group.add_argument(
        "--acquire", dest="mode", action="store_const", const="acquire",
        help=f"send the read command and receive data {ACQUISITION_COUNT} times",
    )
    parser.add_argument(
        "--output-format", choices=("hex", "raw", "json"), default="hex",
        help="hex: one lowercase hex line (default); raw: unmodified bytes; json: data plus metadata",
    )
    parser.add_argument(
        "--output-file", type=Path, default=Path("emat_data.txt"),
        help=(
            "text file to which the complete acquisition is written as space-separated "
            "hexadecimal (default: ./emat_data.txt; --start does not write it)"
        ),
    )
    return parser


def serial_settings(args: argparse.Namespace) -> dict[str, object]:
    if serial is None:  # Defensive guard for callers importing this module.
        raise RuntimeError("Missing dependency: install pyserial (python3-serial).")
    parity = {
        "none": serial.PARITY_NONE, "even": serial.PARITY_EVEN,
        "odd": serial.PARITY_ODD, "mark": serial.PARITY_MARK, "space": serial.PARITY_SPACE,
    }[args.parity]
    return {
        "port": args.serial_port,
        "baudrate": args.baudrate,
        "bytesize": args.data_bits,
        "parity": parity,
        "stopbits": args.stop_bits,
        "timeout": 0.05,
        "write_timeout": args.write_timeout_ms / 1000.0,
        "rtscts": args.flow_control == "rtscts",
        "xonxoff": args.flow_control == "xonxoff",
    }


def print_result(result: TransactionResult, output_format: str, mode: str) -> None:
    if mode == "start":
        if output_format == "json":
            print(json.dumps({
                "valid": True, "mode": "start",
                "read_address_cleared": result.cleared,
                "transaction_latency_us": result.latency_us,
            }, separators=(",", ":")))
        elif output_format == "hex":
            print("01")
        else:
            sys.stdout.buffer.write(b"\x01")
            sys.stdout.buffer.flush()
        return

    assert result.data is not None
    if output_format == "raw":
        sys.stdout.buffer.write(result.data)
        sys.stdout.buffer.flush()
    elif output_format == "json":
        print(json.dumps({
            "valid": True,
            "data_hex": result.data.hex(),
            "data_length": len(result.data),
            "transaction_latency_us": result.latency_us,
            "mode": "acquire",
        }, separators=(",", ":")))
    else:
        print(result.data.hex())


def print_start_failure(output_format: str) -> None:
    """Return the caller-visible failure code for a failed start transaction."""
    if output_format == "json":
        print(json.dumps({"valid": False, "mode": "start", "status": "start failed"}, separators=(",", ":")))
    elif output_format == "raw":
        sys.stdout.buffer.write(b"\x00")
        sys.stdout.buffer.flush()
    else:
        print("00")


def main() -> int:
    args = build_parser().parse_args()
    if serial is None:
        print("error: missing dependency: install pyserial (python3-serial)", file=sys.stderr)
        return 2
    if min(args.read_timeout_ms, args.write_timeout_ms, args.ack_timeout_ms) <= 0:
        print("error: all timeout values must be greater than zero", file=sys.stderr)
        return 2
    if args.inter_command_delay_ms < 0:
        print("error: inter-command-delay-ms cannot be negative", file=sys.stderr)
        return 2
    try:
        with serial.Serial(**serial_settings(args)) as port:
            if args.mode == "start":
                result = run_start(port, args)
            else:
                result = run_acquire(port, args)
        if args.mode == "acquire":
            assert result.data is not None
            write_hex_data(args.output_file, result.data, args.expected_data_bytes)
        # print_result(result, args.output_format, args.mode)
        return 0
    except (serial.SerialException, OSError, TimeoutError, ProtocolError, ValueError) as exc:
        if args.mode == "start":
            print_start_failure(args.output_format)
        print(f"EMAT transaction failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
