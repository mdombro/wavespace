#!/usr/bin/env python3

"""
Capture raw PDM bitstream blocks (plus metadata) from the Pico over USB CDC
or an SPI link driven by a Raspberry Pi master.

When talking to the device over USB this helper understands the interactive
debug CLI that the firmware now exposes. It will automatically:
    * Query the CLI for the current debug state.
    * Disable the periodic ASCII status ticker (unless --keep-stats is given).
    * Disable the per-block hex dump (unless --keep-dumps is given).
    * Enable binary streaming so that subsequent reads are pure PDM data.
    * Optionally request a different PDM clock before capturing (via --clock).
    * Restore the previous CLI state when finished (unless --no-restore).

Each block's metadata (block index, byte offset, end-of-block timestamp_us) is
recorded in a CSV sidecar.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
import csv
import struct
from dataclasses import dataclass
from pathlib import Path

from typing import Callable, Iterable, List, Optional, Sequence, Set, Tuple

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - dependency check only
    serial = None  # type: ignore

try:
    import spidev  # type: ignore
except ImportError:  # pragma: no cover - dependency check only
    spidev = None  # type: ignore


DEFAULT_BLOCK_BITS = 16384  # Two channels @ 8192 bits each by default
DEFAULT_BLOCK_BYTES = DEFAULT_BLOCK_BITS // 8  # total payload bytes across all channels
DEFAULT_TIMEOUT = 1.0
SPI_DEFAULT_DEVICE = "0.0"
SPI_DEFAULT_SPEED_HZ = 3_000_000
SPI_MAX_SPEED_HZ = 62_000_000
SPI_TRANSACTION_HEADER_STRUCT = struct.Struct("<III")
SPI_TRANSACTION_HEADER_BYTES = SPI_TRANSACTION_HEADER_STRUCT.size
CLI_READ_WINDOW = 1.0
MAX_CHANNELS_PLAUSIBLE = 8
SPI_STREAM_MAGIC = b"PDM1"
SPI_STREAM_MAGIC_LE = SPI_STREAM_MAGIC[::-1]
SPI_STREAM_MAGIC_LEN = len(SPI_STREAM_MAGIC)
SPI_STREAM_MAGIC_LEN = len(SPI_STREAM_MAGIC)

METADATA_STRUCT = struct.Struct(
    "<QQQIHhHHI"
)  # block_index, first_sample_byte_index, capture_end_time_us, bytes_per_ch, channels, trigger_index, trigger_tail, reserved, padding
METADATA_BYTES = METADATA_STRUCT.size
METADATA_HEADER = [
    "block_index",
    "byte_offset",
    "block_end_time_us",
    "payload_bytes_per_channel",
    "channel_count",
    "trigger_first_index",
    "trigger_tail_high",
    "dropped",
]

COMMAND_HELP = b"?"
COMMAND_TOGGLE_STREAM = b"b"
COMMAND_TOGGLE_STATS = b"s"
COMMAND_TOGGLE_DUMP = b"d"
COMMAND_REINIT = b"r"
COMMAND_TOGGLE_SPI_STREAM = b"p"
COMMAND_STATUS = b"n"
COMMAND_CLOCK = {
    512_000: b"1",
    1_024_000: b"2",
    2_048_000: b"3",
    3_000_000: b"4",
    4_000_000: b"5",
    4_800_000: b"6",
}

STREAM_LINE = re.compile(r"toggle binary streaming \(currently (ON|OFF)\)", re.IGNORECASE)
STATS_LINE = re.compile(r"toggle periodic stats \(currently (ON|OFF)\)", re.IGNORECASE)
DUMP_LINE = re.compile(r"toggle per-block sample dump \(currently (ON|OFF)\)", re.IGNORECASE)
SPI_LINE = re.compile(r"toggle SPI streaming \(currently (ON|OFF)\)", re.IGNORECASE)


@dataclass
class CliState:
    stream: Optional[bool] = None
    stats: Optional[bool] = None
    dump: Optional[bool] = None
    spi_stream: Optional[bool] = None

    def fully_known(self) -> bool:
        return self.stream is not None and self.stats is not None and self.dump is not None


@dataclass
class BlockMetadata:
    """Metadata parsed from the device; timestamp_us reflects the block end time (microseconds)."""
    block_index: int
    byte_offset: int
    timestamp_us: int
    payload_bytes_per_channel: int
    channel_count: int
    trigger_first_index: int
    trigger_tail_high: bool

    @property
    def total_payload_bytes(self) -> int:
        return self.payload_bytes_per_channel * max(1, self.channel_count)


def metadata_path_for(data_path: Path) -> Path:
    return data_path.with_name(data_path.name + ".metadata.csv")


def parse_spi_device_identifier(identifier: str) -> Tuple[int, int]:
    spec = identifier.strip() or SPI_DEFAULT_DEVICE
    if spec.startswith("/dev/spidev"):
        spec = spec.split("/dev/spidev", 1)[1]
    if "." not in spec:
        raise ValueError(f"Invalid SPI device '{identifier}'. Expected <bus>.<device> or /dev/spidevX.Y.")
    bus_str, dev_str = spec.split(".", 1)
    try:
        bus = int(bus_str, 0)
        dev = int(dev_str, 0)
    except ValueError as exc:
        raise ValueError(f"Invalid SPI bus/device numbers in '{identifier}'.") from exc
    if bus < 0 or dev < 0:
        raise ValueError("SPI bus and device numbers must be non-negative.")
    return bus, dev


def emit_dropped_blocks(
    writer: csv.writer,
    expected_index: Optional[int],
    current_index: int,
    *,
    label: str,
    verbose: bool,
) -> int:
    if expected_index is None:
        return current_index
    if current_index <= expected_index:
        return expected_index

    for missing in range(expected_index, current_index):
        writer.writerow([missing, -1, -1, 0, 0, -1, 0, 1])
        if verbose:
            debug(f"[{label}] block {missing} dropped", verbose=True)

    return current_index


def metadata_is_plausible(meta: BlockMetadata) -> bool:
    if meta.payload_bytes_per_channel <= 0 or meta.channel_count <= 0:
        return False
    if meta.byte_offset % meta.payload_bytes_per_channel != 0:
        return False
    expected_index = meta.byte_offset // meta.payload_bytes_per_channel
    return expected_index == meta.block_index


class PacketAligner:
    """
    Maintains framing when reading fixed-size packets from a byte stream by sliding one byte at a time
    until the metadata header looks coherent (block index matches byte offset).
    """

    def __init__(
        self,
        *,
        read_chunk: Callable[[int], bytes],
        packet_bytes: int,
        payload_bytes: int,
        label: str,
        verbose: bool,
    ) -> None:
        self._read_chunk = read_chunk
        self._packet_bytes = packet_bytes
        self._payload_bytes = payload_bytes
        self._label = label
        self._verbose = verbose
        self._last_block_index: Optional[int] = None

    def next_packet(self) -> Tuple[memoryview, BlockMetadata]:
        raw_packet = bytearray(self._read_chunk(self._packet_bytes))
        metadata = self._ensure_aligned(raw_packet)
        packet_view = memoryview(bytes(raw_packet))
        payload = packet_view[METADATA_BYTES:]
        return payload, metadata

    def _ensure_aligned(self, buffer: bytearray) -> BlockMetadata:
        shifts = 0
        while True:
            try:
                metadata, _ = parse_block_packet(bytes(buffer), self._payload_bytes)
            except ValueError:
                metadata = None

            if metadata and self._metadata_valid(metadata):
                if shifts and self._verbose:
                    debug(
                        f"[{self._label}] resynchronized after {shifts} byte shift(s) (block {metadata.block_index})",
                        verbose=True,
                    )
                if (
                    self._last_block_index is not None
                    and metadata.block_index > self._last_block_index + 1
                    and self._verbose
                ):
                    debug(
                        f"[{self._label}] skipped {metadata.block_index - self._last_block_index - 1} block(s)",
                        verbose=True,
                    )
                self._last_block_index = metadata.block_index
                actual_payload = metadata.total_payload_bytes
                self._payload_bytes = actual_payload
                packet_bytes = METADATA_BYTES + actual_payload
                if packet_bytes != self._packet_bytes:
                    self._packet_bytes = packet_bytes
                return metadata

            shifts += 1
            if shifts > self._packet_bytes * 4:
                raise RuntimeError(
                    f"[{self._label}] unable to resynchronize to packet boundary after {shifts} byte shifts"
                )

            next_byte = self._read_chunk(1)
            if not next_byte:
                raise TimeoutError(f"[{self._label}] timed out while searching for packet boundary")

            del buffer[0]
            buffer.extend(next_byte)

    def _metadata_valid(self, metadata: BlockMetadata) -> bool:
        if not metadata_is_plausible(metadata):
            return False
        if self._last_block_index is not None and metadata.block_index < self._last_block_index:
            return False
        return True


class ProgressReporter:
    """Periodically print capture progress without overwhelming stdout."""

    def __init__(self, label: str, interval: float = 1.0) -> None:
        self.label = label
        self.interval = max(0.1, interval)
        self._last = time.monotonic()

    def update(self, blocks: int, bytes_total: int) -> None:
        now = time.monotonic()
        if now - self._last < self.interval:
            return
        self._last = now
        print(
            f"[{self.label}] captured {blocks} block(s) ({bytes_total} payload bytes)...",
            file=sys.stderr,
        )


def parse_block_packet(packet: bytes, payload_bytes_hint: int) -> Tuple[BlockMetadata, memoryview]:
    if len(packet) < METADATA_BYTES:
        raise ValueError("Packet shorter than metadata header")

    (
        block_index,
        byte_offset,
        timestamp_us,
        bytes_per_channel,
        channel_count,
        trigger_index,
        trigger_tail_high,
        _,
        _reserved2,
    ) = METADATA_STRUCT.unpack_from(packet, 0)
    metadata = BlockMetadata(
        block_index=block_index,
        byte_offset=byte_offset,
        timestamp_us=timestamp_us,
        payload_bytes_per_channel=bytes_per_channel,
        channel_count=channel_count,
        trigger_first_index=int(trigger_index),
        trigger_tail_high=bool(trigger_tail_high),
    )

    available_payload = max(0, len(packet) - METADATA_BYTES)
    raw_channel_count = metadata.channel_count
    sanitized_channel_count = raw_channel_count
    if sanitized_channel_count <= 0 or sanitized_channel_count > MAX_CHANNELS_PLAUSIBLE:
        sanitized_channel_count = max(1, min(sanitized_channel_count, MAX_CHANNELS_PLAUSIBLE))
    metadata.channel_count = sanitized_channel_count

    total_payload = metadata.payload_bytes_per_channel * max(1, raw_channel_count)
    if total_payload <= 0 or total_payload > available_payload:
        total_payload = available_payload
        if payload_bytes_hint > 0:
            total_payload = min(payload_bytes_hint, total_payload)
        channels = max(1, metadata.channel_count)
        metadata.payload_bytes_per_channel = max(1, total_payload // channels)
        total_payload = metadata.payload_bytes_per_channel * channels
    else:
        metadata.payload_bytes_per_channel = max(1, metadata.payload_bytes_per_channel)
        total_payload = metadata.payload_bytes_per_channel * metadata.channel_count

    expected = METADATA_BYTES + total_payload
    if len(packet) < expected:
        raise ValueError(
            f"Malformed block packet: expected {expected} bytes, received {len(packet)} bytes "
            f"(payload hint {payload_bytes_hint})"
        )

    return metadata, memoryview(packet)[METADATA_BYTES:expected]


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
                continue

        if state.spi_stream is None:
            match = SPI_LINE.search(line)
            if match:
                state.spi_stream = match.group(1).upper() == "ON"
                continue

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
    return CliState(stream=False, stats=True, dump=False, spi_stream=None)


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


def dump_cli_status(device: serial.Serial, *, label: str) -> None:
    """Print the Pico transport stats banner (command 'n')."""
    lines = send_command(device, COMMAND_STATUS, verbose=False)
    if not lines:
        print(f"[cli] status dump ({label}) produced no output", file=sys.stderr)
        return
    print(f"[cli] status dump ({label}):", file=sys.stderr)
    for line in lines:
        print(f"    {line}", file=sys.stderr)


def dump_cli_status_via_port(
    port: str,
    *,
    timeout: Optional[float],
    label: str,
) -> None:
    if serial is None:
        print("[cli] pyserial not available; cannot read Pico status", file=sys.stderr)
        return
    try:
        with serial.Serial(port=port, baudrate=115200, timeout=timeout) as device:
            dump_cli_status(device, label=label)
    except Exception as exc:  # pragma: no cover - diagnostic helper
        print(f"[cli] failed to read status from {port}: {exc}", file=sys.stderr)


def configure_spi_streaming(
    device: serial.Serial,
    *,
    enable: bool,
    verbose: bool,
) -> Tuple[bool, bool]:
    """
    Ensure the Pico's SPI streaming toggle matches `enable`.

    Returns the starting state and whether we flipped it.
    """
    state = query_cli_state(device, verbose=verbose)
    if state.spi_stream is None:
        raise RuntimeError("Unable to determine SPI streaming state from the CLI output.")

    if state.spi_stream == enable:
        debug(f"[cli] SPI streaming already {'ENABLED' if enable else 'DISABLED'}", verbose=verbose)
        return state.spi_stream, False

    debug(f"[cli] {'enabling' if enable else 'disabling'} SPI streaming", verbose=verbose)
    send_command(device, COMMAND_TOGGLE_SPI_STREAM, verbose=verbose)
    time.sleep(0.1)
    device.reset_input_buffer()
    return state.spi_stream, True


def restore_spi_streaming(
    device: serial.Serial,
    *,
    start_state: bool,
    verbose: bool,
) -> None:
    """Restore the SPI streaming toggle to its starting state."""
    debug(
        f"[cli] restoring SPI streaming to {'ENABLED' if start_state else 'DISABLED'}",
        verbose=verbose,
    )
    send_command(device, COMMAND_TOGGLE_SPI_STREAM, verbose=verbose)


def read_exact(device: serial.Serial, length: int) -> bytes:
    """Read exactly `length` bytes from the serial device."""
    buffer = bytearray()
    while len(buffer) < length:
        chunk = device.read(length - len(buffer))
        if not chunk:
            raise TimeoutError("Timed out waiting for data from the device")
        buffer.extend(chunk)
    return bytes(buffer)




class SpiBlockReader:
    """Read one full header+payload packet per transfer while reusing buffers for speed."""

    def __init__(self, device: "spidev.SpiDev", packet_bytes: int, timeout: float) -> None:
        self._device = device
        self._packet_bytes = packet_bytes
        self._timeout = timeout
        self._supports_read = hasattr(device, "readbytes")
        self._tx_template = [0] * packet_bytes if not self._supports_read else None

    def read_packet(self) -> bytes:
        deadline = None if self._timeout <= 0 else time.monotonic() + self._timeout
        while True:
            try:
                if self._supports_read:
                    data = self._device.readbytes(self._packet_bytes)
                else:
                    data = self._device.xfer2(self._tx_template)
            except OSError as exc:
                raise RuntimeError(f"SPI transfer failed: {exc}") from exc

            if len(data) == self._packet_bytes:
                return bytes(data)

            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for a complete SPI packet from the Pico")

            time.sleep(0.0002)

def capture_stream_usb(
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
    metadata_path = metadata_path_for(output_path)

    serial_timeout = None if timeout <= 0 else timeout
    with (
        serial.Serial(port=port, baudrate=115200, timeout=serial_timeout) as device,
        output_path.open("wb") as sink,
        metadata_path.open("w", newline="") as metadata_file,
    ):
        metadata_writer = csv.writer(metadata_file)
        metadata_writer.writerow(METADATA_HEADER)
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

        packet_source = PacketAligner(
            read_chunk=lambda size: read_exact(device, size),
            packet_bytes=packet_bytes,
            payload_bytes=block_bytes,
            label="usb",
            verbose=verbose,
        )

        block_count = 0
        dropped_blocks = 0
        bytes_captured = 0
        progress = ProgressReporter("usb")
        expected_block_index: Optional[int] = None

        try:
            while blocks is None or block_count < blocks:
                payload_view, metadata = packet_source.next_packet()

                if expected_block_index is not None:
                    if metadata.block_index < expected_block_index:
                        debug(
                            f"[usb] received out-of-order block {metadata.block_index} (expected {expected_block_index}), skipping",
                            verbose=verbose,
                        )
                        continue
                    if metadata.block_index > expected_block_index:
                        dropped_blocks = emit_dropped_blocks(
                            metadata_writer,
                            expected_block_index,
                            metadata.block_index,
                            label="usb",
                            verbose=verbose,
                        )
                        progress.update(
                            block_count,
                            bytes_captured,
                        )
                expected_block_index = metadata.block_index + 1

                sink.write(payload_view)
                metadata_writer.writerow(
                    [
                        metadata.block_index,
                        metadata.byte_offset,
                        metadata.timestamp_us,
                        metadata.payload_bytes_per_channel,
                        metadata.channel_count,
                        metadata.trigger_first_index,
                        1 if metadata.trigger_tail_high else 0,
                        0,
                    ]
                )
                block_count += 1
                bytes_captured += metadata.total_payload_bytes
                progress.update(block_count, bytes_captured)

        except KeyboardInterrupt:
            debug("[capture] interrupted by user", verbose=True)
        finally:
            sink.flush()
            metadata_file.flush()
            if restore:
                restore_device(device, start_state=start_state, toggled=toggled, verbose=verbose)

    print(
        f"Captured {block_count} block(s) ({bytes_captured} payload bytes) to {output_path} "
        f"(metadata -> {metadata_path})",
        f"Dropped {dropped_blocks} block(s).",
        file=sys.stderr,
    )



def capture_stream_spi(
    device_identifier: str,
    output_path: Path,
    block_bytes: int,
    blocks: Optional[int],
    timeout: float,
    *,
    speed_hz: int,
    mode: int,
    cli_port: Optional[str],
    cli_show_stats: bool,
    restore_cli_spi: bool,
    verbose: bool,
) -> None:
    if spidev is None:
        raise RuntimeError("spidev module not available. Install with `pip install spidev` on the Raspberry Pi host.")

    bus, dev = parse_spi_device_identifier(device_identifier or SPI_DEFAULT_DEVICE)
    if speed_hz <= 0:
        raise ValueError("SPI speed must be positive.")
    spi_speed = min(speed_hz, SPI_MAX_SPEED_HZ)
    if speed_hz > SPI_MAX_SPEED_HZ:
        debug(f"[spi] requested {speed_hz} Hz exceeds max {SPI_MAX_SPEED_HZ} Hz; clamped.", verbose=verbose)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = metadata_path_for(output_path)
    cli_start_state: Optional[bool] = None
    spi_cli_toggled = False
    cli_timeout = None if timeout <= 0 else timeout

    if cli_port and cli_show_stats:
        dump_cli_status_via_port(
            cli_port,
            timeout=cli_timeout,
            label="pre-capture",
        )

    if cli_port:
        if serial is None:
            raise RuntimeError("pyserial is required to control SPI streaming via the CLI. Install with `pip install pyserial`.")
        debug(f"[spi] opening CLI control port {cli_port} to manage SPI streaming", verbose=verbose)
        with serial.Serial(port=cli_port, baudrate=115200, timeout=cli_timeout) as cli_device:
            cli_start_state, spi_cli_toggled = configure_spi_streaming(
                cli_device,
                enable=True,
                verbose=verbose,
            )

    spi = spidev.SpiDev()
    try:
        spi.open(bus, dev)
        spi.max_speed_hz = spi_speed
        spi.mode = mode
        try:
            spi.lsbfirst = False  # Ensure MSB-first transfers when supported.
        except AttributeError:
            pass
        spi.bits_per_word = 8
        debug(f"[spi] opened /dev/spidev{bus}.{dev} @ {spi.max_speed_hz} Hz (mode {mode})", verbose=verbose)

        reader = SpiBlockReader(
            spi,
            block_bytes + SPI_TRANSACTION_HEADER_BYTES,
            timeout,
        )

        block_count = 0
        bytes_captured = 0
        progress = ProgressReporter("spi")

        with (
            output_path.open("wb") as sink,
            metadata_path.open("w", newline="") as metadata_file,
        ):
            metadata_writer = csv.writer(metadata_file)
            metadata_writer.writerow(["transaction", "expected_bytes", "actual_bytes", "file_offset"])
            file_offset = 0

            try:
                while blocks is None or block_count < blocks:
                    raw_packet = reader.read_packet()
                    if len(raw_packet) < SPI_TRANSACTION_HEADER_BYTES:
                        debug("[spi] packet shorter than header; skipping", verbose=verbose)
                        continue

                    tx_index, expected_bytes, actual_bytes = SPI_TRANSACTION_HEADER_STRUCT.unpack(
                        raw_packet[:SPI_TRANSACTION_HEADER_BYTES]
                    )
                    payload = raw_packet[SPI_TRANSACTION_HEADER_BYTES:]

                    expected = min(expected_bytes, len(payload))
                    actual = min(actual_bytes, expected)

                    current_offset = file_offset
                    if actual > 0:
                        sink.write(payload[:actual])
                        file_offset += actual
                        bytes_captured += actual

                    metadata_writer.writerow([tx_index, expected_bytes, actual_bytes, current_offset])
                    block_count += 1
                    progress.update(block_count, bytes_captured)

            except KeyboardInterrupt:
                debug("[spi] capture interrupted by user", verbose=True)
            finally:
                sink.flush()
                metadata_file.flush()
    finally:
        spi.close()
        if cli_port and cli_show_stats:
            dump_cli_status_via_port(
                cli_port,
                timeout=cli_timeout,
                label="post-capture",
            )
        if cli_port and spi_cli_toggled and restore_cli_spi and cli_start_state is not None:
            try:
                debug(f"[spi] restoring CLI SPI streaming state via {cli_port}", verbose=verbose)
                with serial.Serial(port=cli_port, baudrate=115200, timeout=cli_timeout) as cli_device:
                    restore_spi_streaming(cli_device, start_state=cli_start_state, verbose=verbose)
            except Exception as exc:  # pragma: no cover - best effort cleanup
                print(f"Warning: failed to restore SPI streaming state via CLI: {exc}", file=sys.stderr)

    print(
        f"Captured {block_count} transaction(s) ({bytes_captured} payload bytes) to {output_path} "
        f"(metadata -> {metadata_path}).",
        file=sys.stderr,
    )
def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture raw PDM bitstream blocks (and their metadata) from the Pico over USB (CDC) or SPI."
    )
    parser.add_argument(
        "source",
        nargs="?",
        help=(
            "Serial port exposed by the Pico's CLI. Required for --transport usb, optional for SPI captures "
            "to auto-toggle the SPI streaming control."
        ),
    )
    parser.add_argument(
        "output",
        help="Destination file path for the captured binary payload; metadata is written to <output>.metadata.csv",
    )
    parser.add_argument(
        "--transport",
        choices=("usb", "spi"),
        default="usb",
        help="Select the transport used by the Pico (default: usb).",
    )

    parser.add_argument(
        "--block-bytes",
        type=int,
        default=DEFAULT_BLOCK_BYTES,
        help=(
            "Number of payload bytes per block before metadata (default: "
            f"{DEFAULT_BLOCK_BYTES}, matching {DEFAULT_BLOCK_BITS} PDM bits)."
        ),
    )
    parser.add_argument(
        "--blocks",
        type=int,
        default=None,
        help="Optional limit for the number of blocks to capture (default: capture until interrupted).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help=(
            "I/O timeout in seconds (default: 1.0). Applies to serial reads for USB"
            " and to spidev transfers for SPI captures."
        ),
    )
    parser.add_argument(
        "--clock",
        type=int,
        choices=sorted(COMMAND_CLOCK.keys()),
        help=(
            "(USB transports) Request the firmware to restart with the given PDM clock frequency (Hz) before capturing."
        ),
    )
    parser.add_argument(
        "--keep-stats",
        action="store_true",
        help="(USB transports) Leave the periodic stats ticker enabled (default: disable to avoid ASCII in the stream).",
    )
    parser.add_argument(
        "--keep-dumps",
        action="store_true",
        help="(USB transports) Leave the per-block hex dump enabled if it is currently active.",
    )
    parser.add_argument(
        "--no-restore",
        action="store_true",
        help="(USB transports) Do not restore the previous CLI state after capture (binary streaming will remain enabled).",
    )
    parser.add_argument(
        "--spi-device",
        default=SPI_DEFAULT_DEVICE,
        help="(SPI transport) Bus and device in <bus>.<device> form or /dev/spidevX.Y (default: 0.0).",
    )
    parser.add_argument(
        "--spi-speed-hz",
        type=int,
        default=SPI_DEFAULT_SPEED_HZ,
        help=(
            "(SPI transport) SPI clock frequency in Hz (max "
            f"{SPI_MAX_SPEED_HZ}); default {SPI_DEFAULT_SPEED_HZ} (matches firmware's recommended clock)."
        ),
    )
    parser.add_argument(
        "--spi-mode",
        type=int,
        choices=(0, 1, 2, 3),
        default=3,
        help="(SPI transport) SPI mode (CPOL/CPHA). Default 3 for compatibility with Pico SPI streaming.",
    )
    parser.add_argument(
        "--spi-cli-port",
        help="(SPI transport) Serial port to talk to the Pico CLI; defaults to SOURCE when provided.",
    )
    parser.add_argument(
        "--spi-keep-streaming",
        action="store_true",
        help="(SPI transport) Leave SPI streaming enabled after capture (requires --spi-cli-port or SOURCE).",
    )
    parser.add_argument(
        "--spi-show-stats",
        action="store_true",
        help="(SPI transport) Issue the CLI 'n' command before/after capture to show SPI packet counters.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print diagnostic information about CLI or network interactions to stderr.",
    )

    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    if args.block_bytes <= 0:
        print("Block size must be positive", file=sys.stderr)
        return 2

    try:
        if args.transport == "usb":
            if not args.source:
                print("USB transport requires a serial port argument", file=sys.stderr)
                return 2

            if serial is None:
                print("pyserial is required for USB captures. Install with `pip install pyserial`.", file=sys.stderr)
                return 2

            try:
                capture_stream_usb(
                    args.source,
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
        elif args.transport == "spi":
            if args.clock is not None or args.keep_stats or args.keep_dumps or args.no_restore:
                print(
                    "Warning: --clock/--keep-stats/--keep-dumps/--no-restore are ignored for SPI captures.",
                    file=sys.stderr,
                )
            spi_cli_port = args.spi_cli_port or args.source
            try:
                capture_stream_spi(
                    args.spi_device,
                    Path(args.output),
                    args.block_bytes,
                    args.blocks,
                    args.timeout,
                    speed_hz=args.spi_speed_hz,
                    mode=args.spi_mode,
                    cli_port=spi_cli_port,
                    cli_show_stats=args.spi_show_stats,
                    restore_cli_spi=not args.spi_keep_streaming,
                    verbose=args.verbose,
                )
            except TimeoutError as exc:
                print(f"Error: {exc}", file=sys.stderr)
                return 1
            except (OSError, ValueError, RuntimeError) as exc:
                print(f"SPI error: {exc}", file=sys.stderr)
                return 1
    except KeyboardInterrupt:
        print("Capture cancelled by user", file=sys.stderr)
        return 130

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
