#!/usr/bin/env python3
"""
Create a CSV of XYZ coordinates, one row per capture index discovered in a folder.

The script scans for capture_*.json files to collect indices, lays those points out
on a regular 3D grid, and writes:
    index, X, Y, Z

Grid spacing is configurable (uniform or per-axis), and the grid size is chosen
automatically to fit all capture indices.

Example:
  python generate-random-events.py pdm_captures events.csv --spacing 0.1
  python generate-random-events.py pdm_captures events.csv --spacing 0.05 0.1 0.2 --origin 0 0 0.5
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from itertools import product
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


def find_capture_indices(captures_dir: Path) -> List[int]:
    """Return sorted unique indices parsed from capture_*.json filenames."""
    indices: List[int] = []
    seen = set()
    pattern = re.compile(r"capture_(\d+)")
    for json_path in sorted(captures_dir.glob("capture_*.json")):
        match = pattern.search(json_path.stem)
        if not match:
            continue
        idx = int(match.group(1))
        if idx not in seen:
            seen.add(idx)
            indices.append(idx)
    return sorted(indices)


def parse_spacing(spacing_vals: Sequence[float]) -> Tuple[float, float, float]:
    if len(spacing_vals) == 1:
        s = spacing_vals[0]
        if s <= 0:
            raise ValueError("Spacing must be positive.")
        return (s, s, s)
    if len(spacing_vals) == 3:
        sx, sy, sz = spacing_vals
        if sx <= 0 or sy <= 0 or sz <= 0:
            raise ValueError("Spacing must be positive.")
        return (sx, sy, sz)
    raise ValueError("Spacing must be one value (uniform) or three values (sx sy sz).")


def parse_origin(origin_vals: Sequence[float]) -> Tuple[float, float, float]:
    if len(origin_vals) != 3:
        raise ValueError("Origin must have exactly three values.")
    return (origin_vals[0], origin_vals[1], origin_vals[2])


def minimal_grid_dims(count: int) -> Tuple[int, int, int]:
    """Compute a small (nx, ny, nz) grid that fits count points."""
    if count <= 0:
        raise ValueError("Point count must be positive.")
    approx = int(math.ceil(count ** (1 / 3)))
    limit = max(1, approx + 3)
    best_dims = (count, 1, 1)  # start over-sized but valid
    best_volume = count * 1 * 1
    best_balance = max(best_dims)

    for nx in range(1, limit + 1):
        for ny in range(1, limit + 1):
            nz = int(math.ceil(count / (nx * ny)))
            volume = nx * ny * nz
            balance = max(nx, ny, nz)
            if volume < best_volume or (volume == best_volume and balance < best_balance):
                best_volume = volume
                best_balance = balance
                best_dims = (nx, ny, nz)
    return best_dims


def build_grid_points(
    count: int,
    spacing: Tuple[float, float, float],
    origin: Tuple[float, float, float],
) -> Tuple[List[Tuple[float, float, float]], Tuple[int, int, int]]:
    nx, ny, nz = minimal_grid_dims(count)
    sx, sy, sz = spacing
    ox, oy, oz = origin
    xs = [ox + sx * i for i in range(nx)]
    ys = [oy + sy * j for j in range(ny)]
    zs = [oz + sz * k for k in range(nz)]
    coords: List[Tuple[float, float, float]] = []
    for x, y, z in product(xs, ys, zs):
        coords.append((x, y, z))
        if len(coords) >= count:
            break
    return coords, (nx, ny, nz)


def write_csv(
    output_csv: Path,
    indices: Iterable[int],
    points: Iterable[Tuple[float, float, float]],
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "X", "Y", "Z"])
        for idx, (x, y, z) in zip(indices, points):
            writer.writerow([idx, f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate XYZ CSV rows from capture_*.json indices laid out on a grid."
    )
    parser.add_argument("captures_dir", type=Path, help="Directory containing capture_*.json files.")
    parser.add_argument("output_csv", type=Path, help="Destination CSV path.")
    parser.add_argument(
        "--spacing",
        type=float,
        nargs="+",
        default=[0.1],
        help="Grid spacing; provide 1 value (uniform) or 3 values (sx sy sz). Default: %(default)s",
    )
    parser.add_argument(
        "--origin",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("OX", "OY", "OZ"),
        help="Grid origin (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    indices = find_capture_indices(args.captures_dir)
    if not indices:
        raise SystemExit(f"No capture_*.json files found under {args.captures_dir}")

    try:
        spacing = parse_spacing(args.spacing)
        origin = parse_origin(args.origin)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    points, dims = build_grid_points(count=len(indices), spacing=spacing, origin=origin)
    write_csv(args.output_csv, indices=indices, points=points)

    print(
        f"Wrote {len(indices)} rows to {args.output_csv} "
        f"(grid {dims[0]}x{dims[1]}x{dims[2]}, spacing={spacing})."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
