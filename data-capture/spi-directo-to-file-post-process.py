#!/usr/bin/env python3
"""
process_spi_dump.py: parse raw SPI frames from spi_capture_helper
and emit the same output/metadata files as capture_raw_pdm.py.

Usage:
    process_spi_dump.py raw_frames.bin output.bin
"""

import argparse
import csv
import struct
import sys
from pathlib import Path

SPI_SYNC_WORD = b"\xA5\x5A"
SPI_HEADER_STRUCT = struct.Struct("<2B B B Q B B H")
SPI_HEADER_BYTES = SPI_HEADER_STRUCT.size
SPI_PAYLOAD_BYTES = 512
SPI_FRAME_BYTES = SPI_HEADER_BYTES + SPI_PAYLOAD_BYTES + 2  # CRC-16

SPI_FLAGS_OVERFLOW = 0x01
SPI_FLAGS_UNDERRUN = 0x02

def crc16_ccitt(data, initial=0xFFFF):
    crc = initial & 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def parse_frame(frame_bytes):
    if len(frame_bytes) != SPI_FRAME_BYTES:
        raise ValueError(f"expected {SPI_FRAME_BYTES} bytes, got {len(frame_bytes)}")

    sync0, sync1, version, header_len, start_bit_index, flags, channel_mask, payload_len = \
        SPI_HEADER_STRUCT.unpack_from(frame_bytes, 0)

    if sync0 != SPI_SYNC_WORD[0] or sync1 != SPI_SYNC_WORD[1]:
        raise ValueError("sync mismatch")
    if payload_len != SPI_PAYLOAD_BYTES:
        raise ValueError(f"unexpected payload_len {payload_len}")
    payload = frame_bytes[SPI_HEADER_BYTES:SPI_HEADER_BYTES + payload_len]
    crc_expected = struct.unpack_from("<H", frame_bytes, SPI_HEADER_BYTES + payload_len)[0]
    crc_region = frame_bytes[2:2 + header_len + payload_len]
    crc_ok = (crc16_ccitt(crc_region) == crc_expected)
    return {
        "version": version,
        "header_len": header_len,
        "start_bit_index": start_bit_index,
        "flags": flags,
        "channel_mask": channel_mask,
        "payload": payload,
        "crc_ok": crc_ok,
    }

def process_dump(raw_path, output_path):
    metadata_path = output_path.with_suffix(output_path.suffix + ".metadata.csv")
    bytes_captured = 0
    block_index = 0
    expected_bit_index = None
    gap_bits_total = 0

    with raw_path.open("rb") as raw, output_path.open("wb") as sink, metadata_path.open("w", newline="") as meta_file:
        writer = csv.writer(meta_file)
        writer.writerow([
            "record_type","sequence","start_bit_index","payload_bits","payload_bytes",
            "channel_mask","channel_count","flags","overflow","underrun","crc_ok",
            "file_offset","host_timestamp_us","gap_bits","notes"
        ])

        while True:
            frame_bytes = raw.read(SPI_FRAME_BYTES)
            if not frame_bytes:
                break
            if len(frame_bytes) < SPI_FRAME_BYTES:
                print("Warning: trailing partial frame ignored", file=sys.stderr)
                break

            try:
                frame = parse_frame(frame_bytes)
            except ValueError as exc:
                writer.writerow(["gap", block_index, expected_bit_index or -1,
                                 SPI_PAYLOAD_BYTES * 8, SPI_PAYLOAD_BYTES, "0x00", 0,
                                 "0x00",0,0,0,-1,0,SPI_PAYLOAD_BYTES*8,f"parse_error:{exc}"])
                continue

            if not frame["crc_ok"]:
                writer.writerow(["gap", block_index, expected_bit_index or -1,
                                 SPI_PAYLOAD_BYTES * 8, SPI_PAYLOAD_BYTES,
                                 f"0x{frame['channel_mask']:02X}", bin(frame["channel_mask"]).count("1"),
                                 f"0x{frame['flags']:02X}",
                                 1 if frame["flags"] & SPI_FLAGS_OVERFLOW else 0,
                                 1 if frame["flags"] & SPI_FLAGS_UNDERRUN else 0,
                                 0, -1, 0, SPI_PAYLOAD_BYTES * 8, "crc_fail"])
                block_index += 1
                continue

            if expected_bit_index is not None and frame["start_bit_index"] > expected_bit_index:
                gap_bits = frame["start_bit_index"] - expected_bit_index
                gap_bits_total += gap_bits
                writer.writerow(["gap", block_index, expected_bit_index, gap_bits, gap_bits // 8,
                                 "0x00",0,"0x00",0,0,0,-1,0,gap_bits,"missing_bits"])

            expected_bit_index = frame["start_bit_index"] + SPI_PAYLOAD_BYTES * 8
            sink.write(frame["payload"])
            bytes_captured += SPI_PAYLOAD_BYTES

            channel_count = bin(frame["channel_mask"]).count("1")
            writer.writerow([
                "frame", block_index, frame["start_bit_index"], SPI_PAYLOAD_BYTES * 8,
                SPI_PAYLOAD_BYTES, f"0x{frame['channel_mask']:02X}", channel_count,
                f"0x{frame['flags']:02X}",
                1 if frame["flags"] & SPI_FLAGS_OVERFLOW else 0,
                1 if frame["flags"] & SPI_FLAGS_UNDERRUN else 0,
                1, bytes_captured - SPI_PAYLOAD_BYTES, 0, 0, ""
            ])
            block_index += 1

    print(f"Processed {block_index} frames ({bytes_captured} bytes). Gaps: {gap_bits_total} bits.")

def main():
    parser = argparse.ArgumentParser(description="Parse raw SPI dump into output.bin + metadata.")
    parser.add_argument("raw_dump", type=Path, help="binary dump from spi_capture_helper")
    parser.add_argument("output", type=Path, help="output file path for payload bytes")
    args = parser.parse_args()
    process_dump(args.raw_dump, args.output)

if __name__ == "__main__":
    main()
