#!/usr/bin/env python3
"""
Offline post-processing for PDM capture sets.

Given a directory of capture JSON files (capture_XXXX.json or capture_XXXX_chY.json)
and the referenced .bin payloads, run the same FIR + decimation pipeline as
stream_filter_pdm.py, combine channels, and write one NPZ per capture with:
  - xyz: float32 [3] from a provided CSV (columns: index, X, Y, Z)
  - val: float32 [T] filtered/combined samples for that capture

Example:
  python post-process-pdm.py --captures pdm_captures --xyz positions.csv --out points
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np


# ----------------- Filter helpers (mirrors stream_filter_pdm.py) -----------------

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


def design_highpass(fs_in: float, fc: float) -> np.ndarray:
    if fc <= 0:
        raise ValueError("High-pass cutoff must be positive.")
    lowpass = design_fir(fs_in, fc)
    highpass = -lowpass
    mid = len(highpass) // 2
    highpass[mid] += 1.0
    return highpass.astype(np.float64)


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


# ----------------------------- Capture utilities -----------------------------

@dataclass
class ChannelSpec:
    channel_index: int
    path: Path
    byte_count: Optional[int] = None
    bit_count: Optional[int] = None


@dataclass
class CaptureSpec:
    index: int
    json_path: Path
    channels: List[ChannelSpec]
    raw_meta: Dict


def _parse_index_from_name(path: Path) -> Optional[int]:
    m = re.search(r"capture_(\d+)", path.stem)
    return int(m.group(1)) if m else None


def _channel_from_json(json_path: Path, payload_dir: Path, data: Dict) -> ChannelSpec:
    ch_idx = int(data.get("channel_index", 0))
    byte_count = data.get("byte_count") or data.get("byte_count_per_channel")
    bit_count = data.get("bit_count")
    data_file = data.get("data_file") or data.get("file") or data.get("path")
    if data_file:
        payload_path = payload_dir / data_file
    else:
        payload_path = json_path.with_name(f"{json_path.stem}.bin")
    return ChannelSpec(channel_index=ch_idx, path=payload_path, byte_count=byte_count, bit_count=bit_count)


def parse_capture_json(json_path: Path) -> CaptureSpec:
    payload_dir = json_path.parent
    with open(json_path, "r", encoding="utf-8") as f:
        meta = json.load(f)

    cap_idx = meta.get("index")
    if cap_idx is None:
        cap_idx = _parse_index_from_name(json_path)
    if cap_idx is None:
        raise ValueError(f"Unable to infer capture index for {json_path}")
    cap_idx = int(cap_idx)

    channels: List[ChannelSpec] = []
    if isinstance(meta.get("channels"), list) and meta["channels"]:
        for ch in meta["channels"]:
            channels.append(_channel_from_json(json_path, payload_dir, ch))
    else:
        channels.append(_channel_from_json(json_path, payload_dir, meta))

    # Fill in missing per-channel bit count if the capture-level metadata provides it.
    if meta.get("bit_count") and all(c.bit_count is None for c in channels):
        for c in channels:
            c.bit_count = int(meta["bit_count"])
    if meta.get("byte_count_per_channel"):
        for c in channels:
            if c.byte_count is None:
                c.byte_count = int(meta["byte_count_per_channel"])

    return CaptureSpec(index=cap_idx, json_path=json_path, channels=channels, raw_meta=meta)


def discover_captures(captures_dir: Path) -> List[CaptureSpec]:
    captures: Dict[int, CaptureSpec] = {}
    for json_path in sorted(captures_dir.glob("capture_*.json")):
        spec = parse_capture_json(json_path)
        existing = captures.get(spec.index)
        if existing:
            existing.channels.extend(spec.channels)
        else:
            captures[spec.index] = spec
    return [captures[k] for k in sorted(captures.keys())]


def load_xyz_table(csv_path: Path) -> Dict[int, np.ndarray]:
    xyz_map: Dict[int, np.ndarray] = {}
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} has no header row.")
        fields = {name.lower(): name for name in reader.fieldnames}
        for required in ("index", "x", "y", "z"):
            if required not in fields:
                raise ValueError(f"Missing '{required}' column in {csv_path}")
        for row in reader:
            idx = int(row[fields["index"]])
            xyz = np.array(
                [
                    float(row[fields["x"]]),
                    float(row[fields["y"]]),
                    float(row[fields["z"]]),
                ],
                dtype=np.float32,
            )
            xyz_map[idx] = xyz
    return xyz_map


# ----------------------------- Processing helpers -----------------------------

def read_channel_bits(channel: ChannelSpec, bitorder: str) -> np.ndarray:
    payload = channel.path.read_bytes()
    if channel.byte_count:
        payload = payload[: int(channel.byte_count)]
    byte_view = np.frombuffer(payload, dtype=np.uint8)
    bits = np.unpackbits(byte_view, bitorder=bitorder)
    target_bits = channel.bit_count or (channel.byte_count * 8 if channel.byte_count else None)
    if target_bits:
        bits = bits[: int(target_bits)]
    return bits.astype(np.float64)


def filter_channel(bits: np.ndarray, taps: np.ndarray, decimation: int, map_pm1: bool) -> np.ndarray:
    signal = bits
    if map_pm1:
        signal = 2.0 * signal - 1.0
    filt = StreamingFilter(taps, decimation)
    return filt.process(signal)


def combine_channels(channels: Sequence[np.ndarray], method: str) -> np.ndarray:
    if not channels:
        return np.empty(0, dtype=np.float64)
    lengths = {c.size for c in channels}
    if len(lengths) != 1:
        raise ValueError("Channel outputs have mismatched lengths; cannot combine.")
    stack = np.stack(channels, axis=0)
    if method == "sum":
        return np.sum(stack, axis=0)
    if method == "mean":
        return np.mean(stack, axis=0)
    if method == "first":
        return stack[0]
    raise ValueError(f"Unknown combine method '{method}'")


def process_capture(
    spec: CaptureSpec,
    taps: np.ndarray,
    decimation: int,
    bitorder: str,
    map_pm1: bool,
    combine_method: str,
    highpass_taps: Optional[np.ndarray] = None,
) -> np.ndarray:
    filtered: List[np.ndarray] = []
    for ch in sorted(spec.channels, key=lambda c: c.channel_index):
        bits = read_channel_bits(ch, bitorder=bitorder)
        filtered.append(filter_channel(bits, taps=taps, decimation=decimation, map_pm1=map_pm1))
    combined = combine_channels(filtered, method=combine_method)
    if highpass_taps is not None and combined.size:
        combined = StreamingFilter(highpass_taps, decimation=1).process(combined)
    return combined


# ----------------------------------- CLI -----------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Batch-filter PDM capture files into per-point NPZ archives."
    )
    parser.add_argument("--captures", type=Path, default=Path("pdm_captures"),
                        help="Directory containing capture_XXXX.json and corresponding .bin files.")
    parser.add_argument("--xyz", type=Path, required=True,
                        help="CSV with columns index,X,Y,Z linking capture index to coordinates.")
    parser.add_argument("--out", "--output-dir", dest="output_dir", type=Path, required=True,
                        help="Destination directory for per-capture NPZ files.")
    parser.add_argument("--pattern", type=str, default="point_{index:05d}.npz",
                        help="Filename pattern (Python format with {index}) for output files.")
    parser.add_argument("--sample-rate", type=float, default=2_048_000,
                        help="Input PDM bit rate in Hz (default: %(default)s).")
    parser.add_argument("--cutoff", type=float, default=80_000,
                        help="Low-pass cutoff frequency in Hz (default: %(default)s).")
    parser.add_argument("--decimation", type=int, default=0,
                        help="Decimation factor; 0 chooses automatically based on cutoff.")
    parser.add_argument("--target-mult", type=float, default=2.5,
                        help="Multiplier for auto decimation (fs_out ≈ target_mult * cutoff). Default: %(default)s.")
    parser.add_argument("--lsb-first", dest="bit_msbfirst", action="store_false",
                        help="Interpret each byte as LSB-first (default: MSB-first).")
    parser.add_argument("--keep-bits", dest="map_pm1", action="store_false",
                        help="Keep bits in {0,1} (default maps to {-1,+1}).")
    parser.add_argument("--highpass-cutoff", type=float, default=20_000,
                        help="High-pass cutoff in Hz applied after decimation. Set ≤ 0 to disable.")
    parser.set_defaults(bit_msbfirst=True, map_pm1=True)
    parser.add_argument(
        "--combine",
        choices=["sum", "mean", "first"],
        default="sum",
        help="How to merge multi-channel outputs. Default matches stream_filter_pdm (sum).",
    )
    parser.add_argument(
        "--indices",
        type=str,
        help="Optional comma-separated list of capture indices to process (others are skipped).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    captures_dir = args.captures
    if not captures_dir.exists():
        raise SystemExit(f"Capture directory not found: {captures_dir}")

    xyz_table = load_xyz_table(args.xyz)
    taps = design_fir(args.sample_rate, args.cutoff)
    if args.decimation and args.decimation > 0:
        decimation = args.decimation
    else:
        fs_target = args.target_mult * args.cutoff
        decimation = max(1, int(np.floor(args.sample_rate / fs_target)))
    fs_out = args.sample_rate / decimation
    bitorder = "big" if args.bit_msbfirst else "little"
    highpass_cutoff = args.highpass_cutoff if args.highpass_cutoff and args.highpass_cutoff > 0 else None
    highpass_taps: Optional[np.ndarray] = None
    if highpass_cutoff is not None:
        nyquist = fs_out / 2.0
        if highpass_cutoff >= nyquist:
            raise SystemExit(f"High-pass cutoff ({highpass_cutoff} Hz) must be below Nyquist ({nyquist} Hz).")
        highpass_taps = design_highpass(fs_out, highpass_cutoff)

    requested_indices: Optional[Iterable[int]] = None
    if args.indices:
        requested_indices = {int(idx.strip()) for idx in args.indices.split(",") if idx.strip()}

    captures = discover_captures(captures_dir)
    if not captures:
        raise SystemExit(f"No capture_*.json files found under {captures_dir}")

    args.output_dir.mkdir(parents=True, exist_ok=True)

    processed = 0
    for spec in captures:
        if requested_indices is not None and spec.index not in requested_indices:
            continue
        if spec.index not in xyz_table:
            print(f"Skipping capture {spec.index}: missing XYZ entry in {args.xyz}")
            continue

        combined = process_capture(
            spec=spec,
            taps=taps,
            decimation=decimation,
            bitorder=bitorder,
            map_pm1=args.map_pm1,
            combine_method=args.combine,
            highpass_taps=highpass_taps,
        )
        if combined.size == 0:
            print(f"Skipping capture {spec.index}: empty filtered output.")
            continue

        out_name = args.pattern.format(index=spec.index)
        out_path = args.output_dir / out_name
        np.savez_compressed(
            out_path,
            xyz=xyz_table[spec.index].astype(np.float32),
            val=combined.astype(np.float32),
            capture_index=np.int32(spec.index),
            channel_count=np.int16(len(spec.channels)),
            fs_in=np.float64(args.sample_rate),
            fs_out=np.float64(fs_out),
            cutoff=np.float64(args.cutoff),
            decimation=np.int32(decimation),
            taps=taps.astype(np.float64),
            highpass_cutoff=np.float64(highpass_cutoff or 0.0),
            highpass_taps=(highpass_taps.astype(np.float64) if highpass_taps is not None else np.array([], dtype=np.float64)),
        )
        processed += 1

    print(
        f"Processed {processed} capture(s) to {args.output_dir} "
        f"({len(captures)} discovered; fs_out ≈ {fs_out/1e3:.1f} kHz)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
