#!/usr/bin/env python3

"""
Capture raw PDM bitstream blocks from the Pico CDC serial port.

This helper understands the interactive debug CLI that the firmware now exposes.
It will automatically:
    * Query the CLI for the current debug state.
    * Disable the periodic ASCII status ticker (unless --keep-stats is given).
    * Disable the per-block hex dump (unless --keep-dumps is given).
    * Enable binary streaming so that subsequent reads are pure PDM data.
    * Optionally request a different PDM clock before capturing (via --clock).
    * Restore the previous CLI state when finished (unless --no-restore).

The capture loop then writes fixed-size blocks to the requested output file.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Set, Tuple

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover - dependency check only
    raise SystemExit("pyserial is required. Install with `pip install pyserial`.") from exc


DEFAULT_BLOCK_BITS = 4096
DEFAULT_BLOCK_BYTES = DEFAULT_BLOCK_BITS // 8
DEFAULT_TIMEOUT = 1.0
CLI_READ_WINDOW = 1.0

COMMAND_HELP = b"?"
COMMAND_TOGGLE_STREAM = b"b"
COMMAND_TOGGLE_STATS = b"s"
COMMAND_TOGGLE_DUMP = b"d"
COMMAND_REINIT = b"r"
COMMAND_CLOCK = {
    512_000: b"1",
    1_024_000: b"2",
    2_048_000: b"3",
}

STREAM_LINE = re.compile(r"toggle binary streaming \(currently (ON|OFF)\)", re.IGNORECASE)
STATS_LINE = re.compile(r"toggle periodic stats \(currently (ON|OFF)\)", re.IGNORECASE)
DUMP_LINE = re.compile(r"toggle per-block sample dump \(currently (ON|OFF)\)", re.IGNORECASE)


@dataclass
class CliState:
    stream: Optional[bool] = None
    stats: Optional[bool] = None
    dump: Optional[bool] = None

    def fully_known(self) -> bool:
        return self.stream is not None and self.stats is not None and self.dump is not None


def debug(msg: str, *, verbose: bool) -> None:
    if verbose:
        print(msg, file=sys.stderr)


def read_available_lines(device: serial.Serial, *, window: float) -> List[str]:
    """
    Drain any ASCII output currently queued by the device for up to `window` seconds.
    We intentionally decode with 'ignore' to avoid UnicodeErrors if stray binary data
    sneaks into the buffer.
    """
    end_time = time.monotonic() + window
    lines: List[str] = []
    buffer = bytearray()

    while time.monotonic() < end_time:
        waiting = device.in_waiting
        if waiting == 0:
            time.sleep(0.01)
            continue

        chunk = device.read(waiting)
        if not chunk:
            continue

        buffer.extend(chunk)

        while True:
            try:
                newline_index = buffer.index(0x0A)  # '\n'
            except ValueError:
                break

            line_bytes = buffer[:newline_index + 1]
            del buffer[:newline_index + 1]
            line = line_bytes.decode("utf-8", errors="ignore").rstrip("\r\n")
            if line:
                lines.append(line)

    if buffer:
        line = buffer.decode("utf-8", errors="ignore").rstrip("\r\n")
        if line:
            lines.append(line)

    return lines


def parse_cli_state(lines: Iterable[str]) -> CliState:
    state = CliState()
    for line in lines:
        if state.stream is None:
            match = STREAM_LINE.search(line)
            if match:
                state.stream = match.group(1).upper() == "ON"
                continue

        if state.stats is None:
            match = STATS_LINE.search(line)
            if match:
                state.stats = match.group(1).upper() == "ON"
                continue

        if state.dump is None:
            match = DUMP_LINE.search(line)
            if match:
                state.dump = match.group(1).upper() == "ON"

    return state


def query_cli_state(device: serial.Serial, *, verbose: bool) -> CliState:
    """Request a help dump and parse the reported state of the debug toggles."""
    for attempt in range(3):
        device.reset_input_buffer()
        device.write(COMMAND_HELP)
        device.flush()
        time.sleep(0.05)

        lines = read_available_lines(device, window=CLI_READ_WINDOW)
        debug(f"[cli] help dump ({attempt + 1}):\n" + "\n".join(lines), verbose=verbose)

        state = parse_cli_state(lines)
        if state.fully_known():
            return state

    # Fall back to safe defaults if we never managed to parse the help text.
    debug("[cli] failed to parse help output, assuming defaults.", verbose=verbose)
    return CliState(stream=False, stats=True, dump=False)


def send_command(device: serial.Serial, command: bytes, *, verbose: bool, read_response: bool = True) -> List[str]:
    device.write(command)
    device.flush()
    time.sleep(0.05)

    if not read_response:
        return []

    lines = read_available_lines(device, window=0.5)
    if lines:
        debug(f"[cli] response to {command!r}:\n" + "\n".join(lines), verbose=verbose)
    return lines


def configure_device(
    device: serial.Serial,
    *,
    desired_stream: bool,
    desired_stats: bool,
    desired_dump: bool,
    clock: Optional[int],
    verbose: bool,
) -> Tuple[CliState, Set[str]]:
    """
    Ensure the Pico is in the requested CLI mode prior to capturing.

    Returns the starting state (for later restoration) and the set of toggles
    that were flipped during configuration.
    """
    start_state = query_cli_state(device, verbose=verbose)
    toggled: Set[str] = set()

    if clock is not None:
        command = COMMAND_CLOCK.get(clock)
        if command is None:
            raise ValueError(f"Unsupported clock value: {clock}")
        debug(f"[cli] requesting PDM clock {clock} Hz", verbose=verbose)
        send_command(device, command, verbose=verbose)
        time.sleep(0.1)
        # After a restart the capture counters reset, but the toggle states persist.

    # Disable stats if needed.
    if start_state.stats is True and not desired_stats:
        debug("[cli] disabling periodic stats", verbose=verbose)
        send_command(device, COMMAND_TOGGLE_STATS, verbose=verbose)
        toggled.add("stats")

    # Disable sample dump if needed.
    if start_state.dump is True and not desired_dump:
        debug("[cli] disabling sample dump", verbose=verbose)
        send_command(device, COMMAND_TOGGLE_DUMP, verbose=verbose)
        toggled.add("dump")

    # Enable streaming last so no ASCII is interleaved with binary data.
    if start_state.stream is False and desired_stream:
        debug("[cli] enabling binary streaming", verbose=verbose)
        send_command(device, COMMAND_TOGGLE_STREAM, verbose=verbose, read_response=False)
        toggled.add("stream")
        # Allow the acknowledgement line to be produced, then drop it.
        time.sleep(0.1)
        device.reset_input_buffer()

    return start_state, toggled


def restore_device(
    device: serial.Serial,
    *,
    start_state: CliState,
    toggled: Set[str],
    verbose: bool,
) -> None:
    """Return the CLI to its previous state, undoing any toggles we applied."""
    if "stream" in toggled:
        device.reset_input_buffer()
        debug("[cli] disabling binary streaming (restore)", verbose=verbose)
        send_command(device, COMMAND_TOGGLE_STREAM, verbose=verbose)

    if "stats" in toggled and start_state.stats:
        debug("[cli] re-enabling periodic stats (restore)", verbose=verbose)
        send_command(device, COMMAND_TOGGLE_STATS, verbose=verbose)

    if "dump" in toggled and start_state.dump:
        debug("[cli] re-enabling sample dump (restore)", verbose=verbose)
        send_command(device, COMMAND_TOGGLE_DUMP, verbose=verbose)


def read_exact(device: serial.Serial, length: int) -> bytes:
    """Read exactly `length` bytes from the serial device."""
    buffer = bytearray()
    while len(buffer) < length:
        chunk = device.read(length - len(buffer))
        if not chunk:
            raise TimeoutError("Timed out waiting for data from the device")
        buffer.extend(chunk)
    return bytes(buffer)


def capture_stream(
    port: str,
    output_path: Path,
    block_bytes: int,
    blocks: Optional[int],
    timeout: float,
    *,
    disable_stats: bool,
    disable_dump: bool,
    enable_stream: bool,
    restore: bool,
    clock: Optional[int],
    verbose: bool,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with serial.Serial(port=port, baudrate=115200, timeout=timeout) as device, output_path.open("wb") as sink:
        debug(f"[cli] connected to {port}", verbose=verbose)

        start_state, toggled = configure_device(
            device,
            desired_stream=enable_stream,
            desired_stats=not disable_stats,
            desired_dump=not disable_dump,
            clock=clock,
            verbose=verbose,
        )

        if not enable_stream:
            raise RuntimeError("Binary streaming must be enabled to capture data.")

        # Drop any pending output before we begin consuming binary blocks.
        device.reset_input_buffer()

        block_count = 0

        try:
            while blocks is None or block_count < blocks:
                data = read_exact(device, block_bytes)
                sink.write(data)
                block_count += 1

        except KeyboardInterrupt:
            debug("[capture] interrupted by user", verbose=True)
        finally:
            sink.flush()
            if restore:
                restore_device(device, start_state=start_state, toggled=toggled, verbose=verbose)

    print(f"Captured {block_count} block(s) ({block_count * block_bytes} bytes) to {output_path}", file=sys.stderr)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture raw PDM bitstream blocks from the Pico CDC serial port."
    )
    parser.add_argument("port", help="Serial port presented by the Pico (e.g. /dev/ttyACM0 or COM5)")
    parser.add_argument("output", help="Destination file path for the captured binary data")
    parser.add_argument(
        "--block-bytes",
        type=int,
        default=DEFAULT_BLOCK_BYTES,
        help=f"Number of bytes per block (default: {DEFAULT_BLOCK_BYTES}, matching {DEFAULT_BLOCK_BITS} PDM bits)",
    )
    parser.add_argument(
        "--blocks",
        type=int,
        default=None,
        help="Optional limit for the number of blocks to capture (default: capture until interrupted)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help=f"Serial read timeout in seconds (default: {DEFAULT_TIMEOUT})",
    )
    parser.add_argument(
        "--clock",
        type=int,
        choices=sorted(COMMAND_CLOCK.keys()),
        help="Request the firmware to restart with the given PDM clock frequency (Hz) before capturing.",
    )
    parser.add_argument(
        "--keep-stats",
        action="store_true",
        help="Leave the periodic stats ticker enabled (default: disable to avoid ASCII in the stream).",
    )
    parser.add_argument(
        "--keep-dumps",
        action="store_true",
        help="Leave the per-block hex dump enabled if it is currently active.",
    )
    parser.add_argument(
        "--no-restore",
        action="store_true",
        help="Do not restore the previous CLI state after capture (binary streaming will remain in its new state).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print diagnostic information about CLI interactions to stderr.",
    )

    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    if args.block_bytes <= 0:
        print("Block size must be positive", file=sys.stderr)
        return 2

    try:
        capture_stream(
            args.port,
            Path(args.output),
            args.block_bytes,
            args.blocks,
            args.timeout,
            disable_stats=not args.keep_stats,
            disable_dump=not args.keep_dumps,
            enable_stream=True,
            restore=not args.no_restore,
            clock=args.clock,
            verbose=args.verbose,
        )
    except TimeoutError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
