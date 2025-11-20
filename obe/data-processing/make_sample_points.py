#!/usr/bin/env python3
"""
Generate per-point NPZ files compatible with visualization/pyvista-viz.py by
correlating filtered PDM samples (from stream_filter_pdm.py) with timestamped
XYZ positions.

Each row in the events CSV must contain a timestamp (microseconds) and a 3D
coordinate. For every event we extract the filtered samples spanning
[timestamp, timestamp + window] and store them in point_{index}.npz with:
    - xyz: float32[3]
    - val: float32[window_samples] scaled to [0, 1] (optional)
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np


@dataclass
class EventPoint:
    timestamp_us: float
    xyz: np.ndarray


def load_filtered_npz(path: Path) -> dict:
    data = np.load(path, allow_pickle=False)
    required = ("samples", "fs_out", "timestamp_us", "sample_index", "sample_count")
    for key in required:
        if key not in data:
            raise KeyError(f"{path} missing required dataset '{key}'")
    return data


def load_events(csv_path: Path, timestamp_col: str, x_col: str, y_col: str, z_col: str) -> List[EventPoint]:
    events: List[EventPoint] = []
    with csv_path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        missing = [col for col in (timestamp_col, x_col, y_col, z_col) if col not in reader.fieldnames]
        if missing:
            raise ValueError(f"{csv_path} missing column(s): {', '.join(missing)}")
        for row in reader:
            try:
                timestamp_us = float(row[timestamp_col])
                xyz = np.array(
                    [float(row[x_col]), float(row[y_col]), float(row[z_col])],
                    dtype=np.float32,
                )
            except ValueError as exc:
                print(f"[events] skipping row due to parse error: {exc}", file=sys.stderr)
                continue
            events.append(EventPoint(timestamp_us=timestamp_us, xyz=xyz))
    return events


class SampleMapper:
    def __init__(
        self,
        samples: np.ndarray,
        timestamp_us: np.ndarray,
        sample_index: np.ndarray,
        sample_count: np.ndarray,
        fs_out: float,
        fs_in: float,
        taps: Optional[np.ndarray],
    ):
        self.samples = samples.astype(np.float32, copy=False)
        self.fs_out = fs_out
        self.samples_per_us = fs_out / 1e6 if fs_out > 0 else 0.0
        self.total_samples = len(self.samples)

        sample_index = sample_index.astype(np.int64)
        sample_count = sample_count.astype(np.int64)
        timestamp_us = timestamp_us.astype(np.float64)

        valid = sample_count > 0
        if not np.any(valid):
            raise ValueError("Filtered NPZ contains no samples.")

        first = int(np.argmax(valid))
        delay_us = 0.0
        if taps is not None and fs_in > 0:
            delay_us = (len(taps) // 2) / fs_in * 1e6

        self.start_sample_index = int(sample_index[first])

        sample_period_us = 1e6 / fs_out if fs_out > 0 else 0.0
        block_duration_us = sample_count[first] * sample_period_us
        block_start_us = float(timestamp_us[first]) - block_duration_us
        self.start_time_us = block_start_us + delay_us

    def extract(self, start_us: float, window_us: float) -> Optional[np.ndarray]:
        offset_us = start_us - self.start_time_us
        if offset_us < 0:
            return None

        sample_offset = int(round(offset_us * self.samples_per_us)) + self.start_sample_index
        if sample_offset >= self.total_samples:
            return None

        window_samples = max(1, int(round(window_us * self.samples_per_us)))
        global_end = min(self.total_samples, sample_offset + window_samples)
        if sample_offset >= global_end:
            return None
        return self.samples[sample_offset:global_end]


def postprocess_series(series: np.ndarray, *, take_abs: bool) -> np.ndarray:
    data = series.astype(np.float32, copy=False)
    if take_abs:
        data = np.abs(data)
    # Map to [0,1] by symmetric scaling around zero.
    peak = float(np.max(np.abs(data)))
    if peak > 0:
        data = (data / peak + 1.0) * 0.5
    else:
        data = np.full_like(data, 0.5)
    return np.clip(data, 0.0, 1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate sample point NPZ files for PyVista visualization.")
    parser.add_argument("filtered_npz", type=Path, help="NPZ produced by stream_filter_pdm.py.")
    parser.add_argument("events_csv", type=Path, help="CSV with timestamp_us,x,y,z columns.")
    parser.add_argument("output_dir", type=Path, help="Directory where point_*.npz files will be written.")
    parser.add_argument(
        "--timestamp-col",
        default="timestamp_us",
        help="Column name for timestamps (microseconds). Default: %(default)s",
    )
    parser.add_argument("--x-col", default="x", help="Column for X coordinates. Default: %(default)s")
    parser.add_argument("--y-col", default="y", help="Column for Y coordinates. Default: %(default)s")
    parser.add_argument("--z-col", default="z", help="Column for Z coordinates. Default: %(default)s")
    parser.add_argument(
        "--window-ms",
        type=float,
        default=20.0,
        help="Duration of each sample window in milliseconds. Default: %(default)s",
    )
    parser.add_argument(
        "--abs",
        dest="take_abs",
        action="store_true",
        help="Use absolute value before scaling.",
    )
    parser.set_defaults(take_abs=True)
    parser.add_argument(
        "--start-index",
        type=int,
        default=0,
        help="Index to start numbering the output point files (default: %(default)s).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    data = load_filtered_npz(args.filtered_npz)
    samples = data["samples"]
    fs_out = float(data["fs_out"])
    fs_in = float(data.get("fs_in", 0.0))
    mapper = SampleMapper(
        samples=samples,
        timestamp_us=data["timestamp_us"],
        sample_index=data["sample_index"],
        sample_count=data["sample_count"],
        fs_out=fs_out,
        fs_in=fs_in,
        taps=data.get("taps"),
    )
    events = load_events(args.events_csv, args.timestamp_col, args.x_col, args.y_col, args.z_col)
    if not events:
        print("No events found in CSV; nothing to do.", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    window_us = args.window_ms * 1_000.0
    processed = 0
    skipped = 0

    for idx, event in enumerate(events, start=args.start_index):
        # print(event.timestamp_us, window_us)
        series = mapper.extract(event.timestamp_us, window_us)
        if series is None:
            skipped += 1
            continue
        values = postprocess_series(series, take_abs=args.take_abs)
        #values = series # do not want to clip to 0,1, leave as full analog range
        fn = args.output_dir / f"point_{idx:05d}.npz"
        np.savez(fn, xyz=event.xyz.astype(np.float32), val=values.astype(np.float32))
        processed += 1

    print(
        f"Created {processed} point file(s) in {args.output_dir}. "
        f"Skipped {skipped} event(s) outside available sample range."
    )
    return 0 if processed > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
