#!/usr/bin/env python3
"""Scan a raw SPI capture for sync words and validate candidate headers."""

import argparse
import struct
from pathlib import Path

SPI_SYNC_WORD = b"\xA5\x5A"
SPI_HEADER_STRUCT = struct.Struct("<2B B B Q B B H")
SPI_HEADER_BYTES = SPI_HEADER_STRUCT.size
SPI_PAYLOAD_BYTES = 512
SPI_FRAME_BYTES = SPI_HEADER_BYTES + SPI_PAYLOAD_BYTES + 2  # CRC


def scan_file(path: Path):
    data = path.read_bytes()
    total_occurrences = 0
    valid_headers = 0
    offsets = []

    pos = 0
    data_len = len(data)
    while True:
        idx = data.find(SPI_SYNC_WORD, pos)
        if idx < 0:
            break
        total_occurrences += 1
        offsets.append(idx)
        if idx + SPI_FRAME_BYTES <= data_len:
            _, _, version, header_len, _, _, _, payload_len = SPI_HEADER_STRUCT.unpack_from(data, idx)
            if version == 0 and header_len == SPI_HEADER_BYTES - 2 and payload_len == SPI_PAYLOAD_BYTES:
                valid_headers += 1
        pos = idx + 1

    return total_occurrences, valid_headers, offsets


def main():
    parser = argparse.ArgumentParser(description="Count SPI sync headers in a raw capture.")
    parser.add_argument("raw_file", type=Path, help="binary dump from spi_capture_helper")
    parser.add_argument(
        "--print-offsets",
        action="store_true",
        help="print every offset where the sync word was detected",
    )
    parser.add_argument(
        "--print-invalid",
        action="store_true",
        help="also print offsets whose headers fail basic sanity checks",
    )
    args = parser.parse_args()

    total, valid, offsets = scan_file(args.raw_file)

    if args.print_offsets:
        for offset in offsets:
            print(offset)
    elif args.print_invalid:
        data = args.raw_file.read_bytes()
        for offset in offsets:
            if offset + SPI_FRAME_BYTES > len(data):
                print(f"{offset} (truncated)")
                continue
            _, _, version, header_len, _, _, _, payload_len = SPI_HEADER_STRUCT.unpack_from(data, offset)
            if not (version == 0 and header_len == SPI_HEADER_BYTES - 2 and payload_len == SPI_PAYLOAD_BYTES):
                print(f"{offset} (candidate header mismatch: ver={version} hdr_len={header_len} payload={payload_len})")

    print(f"Found {total} sync-word occurrences; {valid} have valid header metadata.")


if __name__ == "__main__":
    main()
