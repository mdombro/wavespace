#!/usr/bin/env python3

"""
Capture raw PDM bitstream blocks from the Pico over USB CDC, a vendor-specific USB
bulk endpoint, or Wi-Fi UDP.

When talking to the device over USB this helper understands the interactive
debug CLI that the firmware now exposes. It will automatically:
    * Query the CLI for the current debug state.
    * Disable the periodic ASCII status ticker (unless --keep-stats is given).
    * Disable the per-block hex dump (unless --keep-dumps is given).
    * Enable binary streaming so that subsequent reads are pure PDM data.
    * Optionally request a different PDM clock before capturing (via --clock).
    * Optionally enable the dedicated vendor USB stream (when requested).
    * Restore the previous CLI state when finished (unless --no-restore).

In Wi-Fi mode the script simply listens for fixed-size UDP payloads from the
Pico and writes them directly to the requested output file.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Set, Tuple

import socket

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - dependency check only
    serial = None  # type: ignore

try:
    import usb.core  # type: ignore
    import usb.util  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    usb = None  # type: ignore
else:
    import usb  # type: ignore


DEFAULT_BLOCK_BITS = 8192
DEFAULT_BLOCK_BYTES = DEFAULT_BLOCK_BITS // 8
DEFAULT_TIMEOUT = 1.0
DEFAULT_WIFI_PORT = 5000
CLI_READ_WINDOW = 1.0
DEFAULT_USB_VENDOR_VID = 0x2E8A
DEFAULT_USB_VENDOR_PID = 0x4001
DEFAULT_USB_VENDOR_INTERFACE = 2
DEFAULT_USB_VENDOR_ENDPOINT = 0x83

COMMAND_HELP = b"?"
COMMAND_TOGGLE_STREAM = b"b"
COMMAND_TOGGLE_STATS = b"s"
COMMAND_TOGGLE_DUMP = b"d"
COMMAND_TOGGLE_VENDOR = b"u"
COMMAND_REINIT = b"r"
COMMAND_CLOCK = {
    512_000: b"1",
    1_024_000: b"2",
    2_048_000: b"3",
}

STREAM_LINE = re.compile(r"toggle binary streaming \(currently (ON|OFF)\)", re.IGNORECASE)
STATS_LINE = re.compile(r"toggle periodic stats \(currently (ON|OFF)\)", re.IGNORECASE)
DUMP_LINE = re.compile(r"toggle per-block sample dump \(currently (ON|OFF)\)", re.IGNORECASE)
VENDOR_LINE = re.compile(r"toggle USB vendor streaming \(currently (ON|OFF)\)", re.IGNORECASE)


@dataclass
class CliState:
    stream: Optional[bool] = None
    stats: Optional[bool] = None
    dump: Optional[bool] = None
    vendor: Optional[bool] = None

    def fully_known(self) -> bool:
        # Vendor streaming was added later; allow older firmware to omit it.
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
                continue

        if state.vendor is None:
            match = VENDOR_LINE.search(line)
            if match:
                state.vendor = match.group(1).upper() == "ON"

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
    desired_vendor: Optional[bool],
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

    if desired_vendor is not None:
        if start_state.vendor is None:
            debug("[cli] firmware did not report vendor streaming state; skipping toggle", verbose=verbose)
        elif start_state.vendor is True and not desired_vendor:
            debug("[cli] disabling USB vendor streaming", verbose=verbose)
            send_command(device, COMMAND_TOGGLE_VENDOR, verbose=verbose)
            toggled.add("vendor")
        elif start_state.vendor is False and desired_vendor:
            debug("[cli] enabling USB vendor streaming", verbose=verbose)
            send_command(device, COMMAND_TOGGLE_VENDOR, verbose=verbose, read_response=False)
            toggled.add("vendor")
            time.sleep(0.1)
            device.reset_input_buffer()

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

    if "vendor" in toggled and start_state.vendor is not None:
        device.reset_input_buffer()
        if start_state.vendor:
            debug("[cli] re-enabling USB vendor streaming (restore)", verbose=verbose)
        else:
            debug("[cli] disabling USB vendor streaming (restore)", verbose=verbose)
        send_command(device, COMMAND_TOGGLE_VENDOR, verbose=verbose)

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

    serial_timeout = None if timeout <= 0 else timeout
    with serial.Serial(port=port, baudrate=115200, timeout=serial_timeout) as device, output_path.open("wb") as sink:
        debug(f"[cli] connected to {port}", verbose=verbose)

        start_state, toggled = configure_device(
            device,
            desired_stream=enable_stream,
            desired_stats=not disable_stats,
            desired_dump=not disable_dump,
            desired_vendor=None,
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


def capture_stream_wifi(
    bind_host: str,
    bind_port: int,
    output_path: Path,
    block_bytes: int,
    blocks: Optional[int],
    timeout: float,
    *,
    allow_host: Optional[str],
    verbose: bool,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, output_path.open("wb") as sink:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((bind_host, bind_port))
        sock.settimeout(None if timeout <= 0 else timeout)

        debug(f"[wifi] listening on {bind_host}:{bind_port}", verbose=verbose)
        block_count = 0
        source_addr: Optional[Tuple[str, int]] = None

        try:
            while blocks is None or block_count < blocks:
                try:
                    data, addr = sock.recvfrom(65535)
                except socket.timeout as exc:
                    raise TimeoutError("Timed out waiting for UDP packets from the Pico") from exc

                if allow_host and addr[0] != allow_host:
                    debug(f"[wifi] ignoring packet from {addr[0]}:{addr[1]}", verbose=verbose)
                    continue

                if source_addr is None:
                    source_addr = addr
                    debug(f"[wifi] receiving from {addr[0]}:{addr[1]}", verbose=verbose)

                if len(data) != block_bytes:
                    debug(
                        f"[wifi] unexpected payload size {len(data)} (expected {block_bytes}), "
                        "packet skipped",
                        verbose=verbose,
                    )
                    continue

                sink.write(data)
                block_count += 1

        except KeyboardInterrupt:
            debug("[wifi] capture interrupted by user", verbose=True)
        finally:
            sink.flush()

    print(f"Captured {block_count} block(s) ({block_count * block_bytes} bytes) to {output_path}", file=sys.stderr)


def capture_stream_usb_vendor(
    cli_port: Optional[str],
    output_path: Path,
    block_bytes: int,
    blocks: Optional[int],
    timeout: float,
    *,
    disable_stats: bool,
    disable_dump: bool,
    restore: bool,
    clock: Optional[int],
    verbose: bool,
    vendor_vid: int,
    vendor_pid: int,
    vendor_interface: int,
    vendor_endpoint: int,
) -> None:
    if usb is None:
        raise RuntimeError("pyusb is required for USB vendor captures. Install with `pip install pyusb`.")

    if clock is not None and not cli_port:
        raise RuntimeError("--clock requires a CLI serial port when using the usb-vendor transport.")

    if (disable_stats or disable_dump or restore) and not cli_port:
        debug("[cli] note: CLI options requested but no serial port provided; skipping device reconfiguration.", verbose=verbose)

    output_path.parent.mkdir(parents=True, exist_ok=True)

    cli_device: Optional[serial.Serial] = None
    start_state = CliState()
    toggled: Set[str] = set()

    if cli_port:
        if serial is None:
            raise RuntimeError("pyserial is required when specifying a CLI serial port.")

        serial_timeout = None if timeout <= 0 else timeout
        cli_device = serial.Serial(port=cli_port, baudrate=115200, timeout=serial_timeout)
        debug(f"[cli] connected to {cli_port}", verbose=verbose)

        start_state, toggled = configure_device(
            cli_device,
            desired_stream=False,
            desired_stats=not disable_stats,
            desired_dump=not disable_dump,
            desired_vendor=True,
            clock=clock,
            verbose=verbose,
        )
        cli_device.reset_input_buffer()

    dev = usb.core.find(idVendor=vendor_vid, idProduct=vendor_pid)
    if dev is None:
        if cli_device:
            cli_device.close()
        raise RuntimeError(
            f"Unable to locate USB device (VID=0x{vendor_vid:04X}, PID=0x{vendor_pid:04X}). "
            "Ensure the Pico is connected and the new firmware is running."
        )

    detach_kernel = False
    claimed_interface = False
    try:
        if dev.is_kernel_driver_active(vendor_interface):
            try:
                dev.detach_kernel_driver(vendor_interface)
                detach_kernel = True
            except usb.core.USBError as exc:  # type: ignore[attr-defined]
                raise RuntimeError(f"Failed to detach kernel driver from interface {vendor_interface}: {exc}") from exc

        cfg = dev.get_active_configuration()
        interface = usb.util.find_descriptor(cfg, bInterfaceNumber=vendor_interface)
        if interface is None:
            raise RuntimeError(f"USB interface {vendor_interface} not found on the device")

        endpoint = usb.util.find_descriptor(interface, bEndpointAddress=vendor_endpoint)
        if endpoint is None:
            raise RuntimeError(f"USB endpoint 0x{vendor_endpoint:02X} not found on interface {vendor_interface}")

        usb.util.claim_interface(dev, vendor_interface)
        claimed_interface = True

        usb_timeout = 0 if timeout <= 0 else int(timeout * 1000)
        block_count = 0

        with output_path.open("wb") as sink:
            try:
                while blocks is None or block_count < blocks:
                    try:
                        data = endpoint.read(block_bytes, timeout=usb_timeout)
                    except usb.core.USBError as exc:  # type: ignore[attr-defined]
                        errno = getattr(exc, "errno", None)
                        message = str(exc).lower()
                        if timeout > 0 and (errno in (110, "TIMED_OUT") or "timed out" in message):
                            raise TimeoutError("Timed out waiting for data from the USB vendor endpoint") from exc
                        raise

                    sink.write(bytes(data))
                    block_count += 1

            except KeyboardInterrupt:
                debug("[usb] capture interrupted by user", verbose=True)
            finally:
                sink.flush()

        print(f"Captured {block_count} block(s) ({block_count * block_bytes} bytes) to {output_path}", file=sys.stderr)

    finally:
        if claimed_interface:
            usb.util.release_interface(dev, vendor_interface)
        if detach_kernel:
            try:
                dev.attach_kernel_driver(vendor_interface)
            except usb.core.USBError:  # type: ignore[attr-defined]
                pass
        usb.util.dispose_resources(dev)

        if cli_device:
            try:
                if restore:
                    restore_device(cli_device, start_state=start_state, toggled=toggled, verbose=verbose)
            finally:
                cli_device.close()


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture raw PDM bitstream blocks from the Pico over USB (CDC or vendor) or Wi-Fi (UDP)."
    )
    parser.add_argument(
        "source",
        nargs="?",
        help=(
            "Serial port exposed by the Pico's CLI. Required for --transport usb, optional for usb-vendor,"
            " ignored for Wi-Fi captures."
        ),
    )
    parser.add_argument("output", help="Destination file path for the captured binary data")
    parser.add_argument(
        "--transport",
        choices=("usb", "usb-vendor", "wifi"),
        default="usb",
        help="Select the transport used by the Pico (default: usb).",
    )
    parser.add_argument(
        "--usb-vendor-vid",
        type=lambda value: int(value, 0),
        default=DEFAULT_USB_VENDOR_VID,
        help=f"(usb-vendor) USB vendor ID to match (default: 0x{DEFAULT_USB_VENDOR_VID:04X}).",
    )
    parser.add_argument(
        "--usb-vendor-pid",
        type=lambda value: int(value, 0),
        default=DEFAULT_USB_VENDOR_PID,
        help=f"(usb-vendor) USB product ID to match (default: 0x{DEFAULT_USB_VENDOR_PID:04X}).",
    )
    parser.add_argument(
        "--usb-vendor-interface",
        type=int,
        default=DEFAULT_USB_VENDOR_INTERFACE,
        help=f"(usb-vendor) Interface number that carries the data stream (default: {DEFAULT_USB_VENDOR_INTERFACE}).",
    )
    parser.add_argument(
        "--usb-vendor-endpoint",
        type=lambda value: int(value, 0),
        default=DEFAULT_USB_VENDOR_ENDPOINT,
        help=f"(usb-vendor) IN endpoint address used for the stream (default: 0x{DEFAULT_USB_VENDOR_ENDPOINT:02X}).",
    )
    parser.add_argument(
        "--block-bytes",
        type=int,
        default=DEFAULT_BLOCK_BYTES,
        help=f"Number of bytes per block (default: {DEFAULT_BLOCK_BYTES}, matching {DEFAULT_BLOCK_BITS} PDM bits).",
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
            " and to UDP receive calls for Wi-Fi."
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
        "--wifi-bind",
        default="0.0.0.0",
        help="Address to bind for Wi-Fi/UDP capture (default: 0.0.0.0).",
    )
    parser.add_argument(
        "--wifi-port",
        type=int,
        default=DEFAULT_WIFI_PORT,
        help=f"UDP port to listen on for Wi-Fi captures (default: {DEFAULT_WIFI_PORT}).",
    )
    parser.add_argument(
        "--wifi-allow",
        help="Restrict Wi-Fi capture to packets from a specific source IPv4 address.",
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
    elif args.transport == "usb-vendor":
        if usb is None:
            print("pyusb is required for usb-vendor captures. Install with `pip install pyusb`.", file=sys.stderr)
            return 2

        if args.source and serial is None:
            print(
                "pyserial is required when supplying a CLI serial port for usb-vendor captures."
                " Install with `pip install pyserial`.",
                file=sys.stderr,
            )
            return 2

        try:
            capture_stream_usb_vendor(
                args.source,
                Path(args.output),
                args.block_bytes,
                args.blocks,
                args.timeout,
                disable_stats=not args.keep_stats,
                disable_dump=not args.keep_dumps,
                restore=not args.no_restore,
                clock=args.clock,
                verbose=args.verbose,
                vendor_vid=args.usb_vendor_vid,
                vendor_pid=args.usb_vendor_pid,
                vendor_interface=args.usb_vendor_interface,
                vendor_endpoint=args.usb_vendor_endpoint,
            )
        except TimeoutError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        except usb.core.USBError as exc:  # type: ignore[attr-defined]
            print(f"USB error: {exc}", file=sys.stderr)
            return 1
        except RuntimeError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
    else:
        if args.clock is not None or args.keep_stats or args.keep_dumps or args.no_restore:
            print(
                "Warning: --clock/--keep-stats/--keep-dumps/--no-restore are ignored for Wi-Fi captures.",
                file=sys.stderr,
            )

        try:
            capture_stream_wifi(
                args.wifi_bind,
                args.wifi_port,
                Path(args.output),
                args.block_bytes,
                args.blocks,
                args.timeout,
                allow_host=args.wifi_allow,
                verbose=args.verbose,
            )
        except TimeoutError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        except OSError as exc:
            print(f"Socket error: {exc}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
