#!/usr/bin/env python3
"""
Stream a raw 1-bit PDM capture (as produced by capture_raw_pdm.py) while it is
being written, apply the same low-pass + decimation pipeline as the original
parse_bit_stream.m helper, and persist the filtered PCM plus metadata into a
.npz archive.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Iterable, Iterator, List, Optional

import numpy as np
from collections import deque


@dataclass
class BlockMeta:
    block_index: int
    byte_offset: int
    timestamp_us: int  # End-of-block timestamp from metadata (microseconds)
    bytes_per_channel: int
    channel_count: int
    dropped: bool = False

    @property
    def payload_bytes(self) -> int:
        return self.bytes_per_channel * max(1, self.channel_count)


@dataclass
class ProcessedBlock:
    meta: BlockMeta
    sample_index: int
    sample_count: int


def design_fir(fs_in: float, fc: float) -> np.ndarray:
    trans = 0.15 * fc
    dfn = trans / (fs_in / 2.0)
    if dfn <= 0:
        raise ValueError("Transition width too small; increase cutoff or decrease sample rate.")
    n_est = int(np.ceil(3.3 / dfn))
    taps = max(512, 2 * int(np.ceil(n_est / 2)))
    wc = fc / (fs_in / 2.0)

    n = np.arange(taps + 1)
    m = n - taps / 2.0
    h_ideal = wc * np.sinc(wc * m)
    window = 0.54 - 0.46 * np.cos(2 * np.pi * n / taps)
    h = h_ideal * window
    h /= np.sum(h)
    return h.astype(np.float64)


class StreamingFilter:
    def __init__(self, taps: np.ndarray, decimation: int) -> None:
        self.taps = taps
        self.decimation = max(1, decimation)
        self._state = np.zeros(len(self.taps) - 1, dtype=np.float64)
        self._delay_remaining = len(self.taps) // 2

    def process(self, chunk: np.ndarray) -> np.ndarray:
        if chunk.size == 0:
            return np.empty(0, dtype=np.float64)

        data = np.concatenate((self._state, chunk))
        filtered = np.convolve(data, self.taps, mode="valid")
        self._state = data[-(len(self.taps) - 1):]

        if self._delay_remaining > 0:
            if filtered.size <= self._delay_remaining:
                self._delay_remaining -= filtered.size
                return np.empty(0, dtype=np.float64)
            filtered = filtered[self._delay_remaining:]
            self._delay_remaining = 0

        if self.decimation > 1:
            filtered = filtered[:: self.decimation]
        return filtered


class MetadataTailer:
    def __init__(self, path: Path, poll_interval: float) -> None:
        self.path = path
        self.poll_interval = poll_interval
        self.file = self._wait_for_file(path, mode="r", text=True)
        self._consume_header()

    def _wait_for_file(self, path: Path, mode: str, text: bool = False):
        while not path.exists():
            time.sleep(self.poll_interval)
        encoding = "utf-8" if text else None
        return open(path, mode, encoding=encoding, newline="")  # pylint: disable=consider-using-with

    def _consume_header(self) -> None:
        pos = self.file.tell()
        line = self.file.readline()
        if not line.lower().startswith("block_index"):
            self.file.seek(pos)

    def read_new(self) -> List[BlockMeta]:
        records: List[BlockMeta] = []
        while True:
            pos = self.file.tell()
            line = self.file.readline()
            if not line:
                self.file.seek(pos)
                break
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 5:
                self.file.seek(pos)
                break
            try:
                bytes_per_channel = int(parts[3])
                channel_count = int(parts[4])
                dropped = bool(int(parts[5])) if len(parts) > 5 else False
                records.append(
                    BlockMeta(
                        block_index=int(parts[0]),
                        byte_offset=int(parts[1]),
                        timestamp_us=int(parts[2]),
                        bytes_per_channel=bytes_per_channel,
                        channel_count=channel_count,
                        dropped=dropped,
                    )
                )
            except ValueError:
                self.file.seek(pos)
                break
        return records


class BinaryTailer:
    def __init__(self, path: Path, poll_interval: float) -> None:
        self.path = path
        self.poll_interval = poll_interval
        self.file = self._wait_for_file(path)
        self.offset = 0

    def _wait_for_file(self, path: Path):
        while not path.exists():
            time.sleep(self.poll_interval)
        return open(path, "rb")  # pylint: disable=consider-using-with

    def read_block(self, length: int) -> Optional[bytes]:
        if length <= 0:
            return b""
        self.file.seek(self.offset)
        chunk = self.file.read(length)
        if len(chunk) < length:
            self.file.seek(self.offset)
            return None
        self.offset += length
        return chunk


class StreamProcessor:
    def __init__(
        self,
        data_path: Path,
        metadata_path: Path,
        poll_interval: float,
        default_block_bytes: int,
        filter_taps: np.ndarray,
        decimation: int,
        bit_msbfirst: bool,
        map_pm1: bool,
    ) -> None:
        self.data_tailer = BinaryTailer(data_path, poll_interval)
        self.meta_tailer = MetadataTailer(metadata_path, poll_interval)
        self.filters: List[StreamingFilter] = []
        self.default_block_bytes = max(0, default_block_bytes)
        self.filter_taps = filter_taps
        self.decimation = decimation
        self.poll_interval = poll_interval
        self.bitorder = "big" if bit_msbfirst else "little"
        self.map_pm1 = map_pm1
        self.pending_meta: Deque[BlockMeta] = deque()
        self.blocks_processed = 0
        self.sample_cursor = 0
        self.block_records: List[ProcessedBlock] = []
        self.samples: List[np.ndarray] = []

    def run(self, max_blocks: Optional[int], idle_timeout: float) -> None:
        idle_elapsed = 0.0

        while max_blocks is None or self.blocks_processed < max_blocks:
            new_meta = self.meta_tailer.read_new()
            if new_meta:
                self.pending_meta.extend(new_meta)

            made_progress = False
            while self.pending_meta:
                meta = self.pending_meta.popleft()
                if meta.dropped:
                    self.block_records.append(
                        ProcessedBlock(
                            meta=meta,
                            sample_index=self.sample_cursor,
                            sample_count=0,
                        )
                    )
                    self.blocks_processed += 1
                    made_progress = True
                    continue

                bytes_to_read = meta.payload_bytes if meta.payload_bytes > 0 else self.default_block_bytes
                if bytes_to_read <= 0:
                    raise RuntimeError("Unable to determine payload size for incoming block.")
                chunk = self.data_tailer.read_block(bytes_to_read)
                if chunk is None:
                    self.pending_meta.appendleft(meta)
                    break
                samples = self._process_chunk(chunk, meta.channel_count)
                sample_count = samples.size
                if sample_count:
                    self.samples.append(samples)
                self.block_records.append(
                    ProcessedBlock(
                        meta=meta,
                        sample_index=self.sample_cursor,
                        sample_count=sample_count,
                    )
                )
                self.sample_cursor += sample_count
                self.blocks_processed += 1
                made_progress = True
                if max_blocks is not None and self.blocks_processed >= max_blocks:
                    return

            if made_progress:
                idle_elapsed = 0.0
                continue

            time.sleep(self.poll_interval)
            idle_elapsed += self.poll_interval
            if idle_timeout > 0 and idle_elapsed >= idle_timeout:
                break

    def _process_chunk(self, chunk: bytes, channel_count: int) -> np.ndarray:
        self._ensure_filters(channel_count)
        byte_view = np.frombuffer(chunk, dtype=np.uint8)
        bits = np.unpackbits(byte_view, bitorder=self.bitorder)
        if channel_count <= 1:
            signal = bits.astype(np.float64)
            if self.map_pm1:
                signal = 2.0 * signal - 1.0
            return self.filters[0].process(signal)

        if bits.size % channel_count != 0:
            raise ValueError(
                f"Bitstream length {bits.size} is not divisible by channel count {channel_count}"
            )

        samples_per_channel = bits.size // channel_count
        combined: Optional[np.ndarray] = None
        for ch in range(channel_count):
            ch_bits = bits[ch::channel_count][:samples_per_channel]
            signal = ch_bits.astype(np.float64)
            if self.map_pm1:
                signal = 2.0 * signal - 1.0
            filtered = self.filters[ch].process(signal)
            if combined is None:
                combined = filtered.copy()
            else:
                if filtered.size != combined.size:
                    raise ValueError("Channel outputs have mismatched sizes; cannot combine.")
                # Combination hook: replace this summation if future processing needs a different strategy.
                combined += filtered

        return combined if combined is not None else np.empty(0, dtype=np.float64)

    def _ensure_filters(self, channel_count: int) -> None:
        required = max(1, channel_count)
        while len(self.filters) < required:
            self.filters.append(StreamingFilter(self.filter_taps, self.decimation))

    def export(self) -> dict:
        if self.samples:
            samples = np.concatenate(self.samples).astype(np.float32)
        else:
            samples = np.empty(0, dtype=np.float32)

        block_index = np.array([rec.meta.block_index for rec in self.block_records], dtype=np.uint64)
        byte_offset = np.array([rec.meta.byte_offset for rec in self.block_records], dtype=np.uint64)
        timestamp_us = np.array([rec.meta.timestamp_us for rec in self.block_records], dtype=np.uint64)
        channel_count = np.array([rec.meta.channel_count for rec in self.block_records], dtype=np.uint16)
        bytes_per_channel = np.array(
            [rec.meta.bytes_per_channel for rec in self.block_records], dtype=np.uint32
        )
        sample_index = np.array([rec.sample_index for rec in self.block_records], dtype=np.int64)
        sample_count = np.array([rec.sample_count for rec in self.block_records], dtype=np.int64)

        return {
            "samples": samples,
            "block_index": block_index,
            "byte_offset": byte_offset,
            "timestamp_us": timestamp_us,
             "channel_count": channel_count,
             "bytes_per_channel": bytes_per_channel,
            "sample_index": sample_index,
            "sample_count": sample_count,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stream-filter PDM capture data into an NPZ archive.")
    parser.add_argument("data", type=Path, help="Path to the raw binary payload file (e.g., vendor.bin).")
    parser.add_argument(
        "metadata",
        type=Path,
        help="Path to the metadata CSV written by capture_raw_pdm.py (e.g., vendor.bin.metadata.csv).",
    )
    parser.add_argument("output", type=Path, help="Destination .npz file.")
    parser.add_argument(
        "--block-bytes",
        type=int,
        default=2048,
        help="Fallback payload bytes per block if metadata is incomplete (default: %(default)s).",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=2_048_000,
        help="Input PDM bit rate in Hz (default: %(default)s).",
    )
    parser.add_argument(
        "--cutoff",
        type=float,
        default=80_000,
        help="Low-pass cutoff frequency in Hz (default: %(default)s).",
    )
    parser.add_argument(
        "--lsb-first",
        dest="bit_msbfirst",
        action="store_false",
        help="Interpret each byte as LSB-first (default: MSB-first).",
    )
    parser.add_argument(
        "--keep-bits",
        dest="map_pm1",
        action="store_false",
        help="Keep bits in {0,1} (default maps to {-1,+1}).",
    )
    parser.set_defaults(bit_msbfirst=True, map_pm1=True)
    parser.add_argument(
        "--decimation",
        type=int,
        default=0,
        help="Optional decimation factor; 0 chooses automatically based on cutoff (default: %(default)s).",
    )
    parser.add_argument(
        "--target-mult",
        type=float,
        default=2.5,
        help="Multiplier for auto decimation (fs_out ≈ target_mult * cutoff). Default: %(default)s.",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.1,
        help="Seconds between polling the input files for new data (default: %(default)s).",
    )
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=2.0,
        help="Stop after this many seconds of inactivity (0 disables auto-stop). Default: %(default)s.",
    )
    parser.add_argument(
        "--max-blocks",
        type=int,
        help="Optional limit on blocks to process before exiting.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    fs_in = args.sample_rate
    cutoff = args.cutoff
    taps = design_fir(fs_in, cutoff)

    if args.decimation and args.decimation > 0:
        decimation = args.decimation
    else:
        fs_target = args.target_mult * cutoff
        decimation = max(1, int(np.floor(fs_in / fs_target)))

    processor = StreamProcessor(
        data_path=args.data,
        metadata_path=args.metadata,
        default_block_bytes=args.block_bytes,
        poll_interval=args.poll_interval,
        filter_taps=taps,
        decimation=decimation,
        bit_msbfirst=args.bit_msbfirst,
        map_pm1=args.map_pm1,
    )

    try:
        processor.run(max_blocks=args.max_blocks, idle_timeout=args.idle_timeout)
    except KeyboardInterrupt:
        pass

    payload = processor.export()
    fs_out = fs_in / decimation

    np.savez(
        args.output,
        samples=payload["samples"],
        fs_in=np.float64(fs_in),
        fs_out=np.float64(fs_out),
        cutoff=np.float64(cutoff),
        decimation=np.int32(decimation),
        taps=taps.astype(np.float64),
        block_index=payload["block_index"],
        byte_offset=payload["byte_offset"],
        timestamp_us=payload["timestamp_us"],
        channel_count=payload["channel_count"],
        bytes_per_channel=payload["bytes_per_channel"],
        sample_index=payload["sample_index"],
        sample_count=payload["sample_count"],
    )

    print(
        f"Processed {processor.blocks_processed} block(s), "
        f"{payload['samples'].size} filtered sample(s) @ {fs_out / 1e3:.1f} kHz."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
