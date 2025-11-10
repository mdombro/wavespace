#!/usr/bin/env python3
"""
Build a synthetic timestamp + XYZ CSV for exercising stream_filter_pdm.py
and make_sample_points.py. Timestamps span the valid range of the filtered
NPZ capture, while coordinates are laid out on a configurable 3D grid.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import numpy as np


@dataclass
class BlockSlice:
    start_us: float
    end_us: float


def build_block_slices(
    timestamp_us: np.ndarray,
    sample_count: np.ndarray,
    fs_out: float,
    fs_in: float,
    taps: np.ndarray | None,
) -> List[BlockSlice]:
    mask = sample_count > 0
    if not np.any(mask):
        raise ValueError("Filtered NPZ contains no usable samples.")

    timestamp_us = timestamp_us.astype(np.float64)
    sample_count = sample_count.astype(np.int64)
    first = int(np.argmax(mask))
    delay_us = 0.0
    if taps is not None and fs_in > 0:
        delay_us = (len(taps) // 2) / fs_in * 1e6

    sample_period_us = 1e6 / fs_out
    current = float(timestamp_us[first]) + delay_us
    slices: List[BlockSlice] = []

    for idx in range(first, len(timestamp_us)):
        count = int(sample_count[idx])
        if count <= 0:
            continue
        start = max(current, float(timestamp_us[idx]))
        duration = count * sample_period_us
        end = start + duration
        slices.append(BlockSlice(start_us=start, end_us=end))
        current = end

    return slices


def generate_grid(
    counts: Sequence[int],
    spacing: Sequence[float],
    origin: Sequence[float],
) -> List[Tuple[float, float, float]]:
    nx, ny, nz = counts
    sx, sy, sz = spacing
    ox, oy, oz = origin
    xs = [ox + sx * i for i in range(nx)]
    ys = [oy + sy * j for j in range(ny)]
    zs = [oz + sz * k for k in range(nz)]
    return [(x, y, z) for x, y, z in product(xs, ys, zs)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate synthetic timestamp/XYZ CSV for testing.")
    parser.add_argument("filtered_npz", type=Path, help="NPZ produced by stream_filter_pdm.py")
    parser.add_argument("output_csv", type=Path, help="Destination CSV path")
    parser.add_argument(
        "--grid",
        nargs=3,
        type=int,
        metavar=("NX", "NY", "NZ"),
        default=(4, 4, 4),
        help="Number of samples along X,Y,Z (default: %(default)s)",
    )
    parser.add_argument(
        "--spacing",
        nargs=3,
        type=float,
        metavar=("SX", "SY", "SZ"),
        default=(0.1, 0.1, 0.1),
        help="Spacing between grid points (default: %(default)s)",
    )
    parser.add_argument(
        "--origin",
        nargs=3,
        type=float,
        metavar=("OX", "OY", "OZ"),
        default=(0.0, 0.0, 0.0),
        help="Origin for the grid (default: %(default)s)",
    )
    parser.add_argument(
        "--window-ms",
        type=float,
        default=20.0,
        help="Spacing between successive timestamps in milliseconds (default: %(default)s)",
    )
    parser.add_argument(
        "--timestamp-column",
        default="timestamp_us",
        help="Name of the timestamp column in the CSV (default: %(default)s)",
    )
    parser.add_argument("--x-column", default="x", help="Name of the X column (default: %(default)s)")
    parser.add_argument("--y-column", default="y", help="Name of the Y column (default: %(default)s)")
    parser.add_argument("--z-column", default="z", help="Name of the Z column (default: %(default)s)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    data = np.load(args.filtered_npz, allow_pickle=False)
    for key in ("timestamp_us", "sample_count", "fs_out"):
        if key not in data:
            raise KeyError(f"{args.filtered_npz} missing required dataset '{key}'")

    slices = build_block_slices(
        timestamp_us=data["timestamp_us"],
        sample_count=data["sample_count"],
        fs_out=float(data["fs_out"]),
        fs_in=float(data.get("fs_in", 0.0)),
        taps=data.get("taps"),
    )
    if not slices:
        raise RuntimeError("No valid sample coverage computed.")

    grid = generate_grid(args.grid, args.spacing, args.origin)
    total_points = len(grid)
    if total_points == 0:
        raise ValueError("Grid dimensions must all be positive.")

    spacing_us = args.window_ms * 1_000.0
    time_start = max(slices[0].start_us, spacing_us)
    duration = max(0, (total_points - 1)) * spacing_us
    time_end = slices[-1].end_us - spacing_us
    if duration > time_end - time_start:
        raise RuntimeError("Not enough capture duration to place evenly spaced events at the requested window-ms spacing.")
                           
    if total_points == 1:
        timestamps = np.array([time_start], dtype=np.float64)
    else:
        timestamps = np.linspace(time_start, time_start + duration, num=total_points, endpoint=True)

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([args.timestamp_column, args.x_column, args.y_column, args.z_column])
        for ts, (x, y, z) in zip(timestamps, grid):
            writer.writerow([f"{ts:.3f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])

    print(f"Wrote {total_points} events to {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
