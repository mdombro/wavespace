#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover - dependency check only
    raise SystemExit(
        "pyserial is required. Install with `pip install pyserial`."
    ) from exc


DEFAULT_BLOCK_BITS = 4096
DEFAULT_BLOCK_BYTES = DEFAULT_BLOCK_BITS // 8
DEFAULT_TIMEOUT = 1.0


def read_exact(device: serial.Serial, length: int) -> bytes:
    """Read exactly `length` bytes from the serial device."""
    buffer = bytearray()
    while len(buffer) < length:
        chunk = device.read(length - len(buffer))
        if not chunk:
            raise TimeoutError("Timed out waiting for data from the device")
        buffer.extend(chunk)
    return bytes(buffer)


def capture_stream(port: str, output_path: Path, block_bytes: int, blocks: Optional[int], timeout: float) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with serial.Serial(port=port, baudrate=115200, timeout=timeout) as device, output_path.open("wb") as sink:
        device.reset_input_buffer()
        block_count = 0

        try:
            while blocks is None or block_count < blocks:
                data = read_exact(device, block_bytes)
                sink.write(data)
                sink.flush()
                block_count += 1

        except KeyboardInterrupt:
            pass

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

    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    if args.block_bytes <= 0:
        print("Block size must be positive", file=sys.stderr)
        return 2

    try:
        capture_stream(args.port, Path(args.output), args.block_bytes, args.blocks, args.timeout)
    except TimeoutError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
