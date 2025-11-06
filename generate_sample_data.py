#!/usr/bin/env python3
"""
Utility to generate synthetic sample data for plotly-viz.py.

Creates:
  1) A directory of per-point NPZ files (`point_00000.npz`, ...) with:
         xyz: float32 array shape [3]
         val: float32 array shape [T]
     Matches the current plotly-viz.py default loader (--path/--pattern).

  2) Optionally, a directory of per-frame NPZ files (`frame_XXXX.npz`) for legacy usage.

  3) Optionally, a single NPZ series file with:
         xyz: float32 array shape [N, 3]
         val: float32 array shape [T, N]

Usage examples:
  python generate_sample_data.py --out-dir sample_points --frames 160 --points 6000
  python generate_sample_data.py --frame-dir sample_frames --frames 160 --points 6000
  python generate_sample_data.py --series-file sample_series.npz --frames 160 --points 6000
  python generate_sample_data.py --out-dir points --frame-dir frames --series-file series.npz
"""

import argparse
import os
from pathlib import Path

import numpy as np


def make_static_point_cloud(points: int, radius: float, seed: int = 7):
    """Return xyz[N,3] distributed roughly inside a sphere."""
    rng = np.random.default_rng(seed)
    u = rng.random(points)
    cost = rng.uniform(-1.0, 1.0, points)
    phi = rng.uniform(0.0, 2.0 * np.pi, points)
    r = radius * np.power(u, 1.0 / 3.0)
    s = np.sqrt(1.0 - cost**2)
    xyz = np.stack(
        [r * s * np.cos(phi), r * s * np.sin(phi), r * cost],
        axis=-1,
    ).astype(np.float32)
    return xyz


def make_time_series_values(xyz: np.ndarray, frames: int, pulse_speed: float = 0.8):
    """
    Generate a (T,N) array of scalar values by sweeping a Gaussian shell
    through the point cloud.
    """
    T, N = frames, xyz.shape[0]
    val = np.empty((T, N), dtype=np.float32)
    rr = np.linalg.norm(xyz, axis=1)
    sigma = 0.16 * np.max(rr) if np.max(rr) > 0 else 1.0
    denom = 2.0 * sigma * sigma
    for t in range(T):
        tau = t / max(1, T - 1)
        r0 = pulse_speed * tau * np.max(rr)
        slice_val = np.exp(-((rr - r0) ** 2) / denom)
        vmax = slice_val.max()
        if vmax > 0:
            slice_val = slice_val / vmax
        val[t] = slice_val.astype(np.float32)
    return val


def write_point_files(out_dir: Path, xyz: np.ndarray, val_series: np.ndarray):
    """Write per-point NPZ files with shape conventions expected by plotly-viz.py."""
    out_dir.mkdir(parents=True, exist_ok=True)
    T = val_series.shape[0]
    for idx in range(xyz.shape[0]):
        fn = out_dir / f"point_{idx:05d}.npz"
        point_xyz = xyz[idx].astype(np.float32)
        point_val = val_series[:, idx].astype(np.float32)
        np.savez_compressed(fn, xyz=point_xyz, val=point_val)


def write_frame_series(out_dir: Path, frames: int, xyz: np.ndarray, val: np.ndarray):
    """
    Write per-frame NPZ files.
    Each file `frame_XXXX.npz` contains xyz and val arrays (copied to keep them independent).
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    frame_template = "frame_{:04d}.npz"
    T = min(frames, val.shape[0])
    for t in range(T):
        fn = out_dir / frame_template.format(t)
        np.savez_compressed(fn, xyz=xyz.astype(np.float32), val=val[t].astype(np.float32))


def write_static_series(series_path: Path, xyz: np.ndarray, val: np.ndarray):
    """Write a single NPZ file with xyz[N,3] and val[T,N]."""
    series_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(series_path, xyz=xyz.astype(np.float32), val=val.astype(np.float32))


def parse_args():
    ap = argparse.ArgumentParser(description="Generate sample data for plotly-viz.py")
    ap.add_argument("--out-dir", "--points-dir", dest="points_dir", type=str,
                    help="Destination directory for per-point NPZ files (default output).")
    ap.add_argument("--frame-dir", type=str,
                    help="Destination directory for per-frame NPZ files (optional).")
    ap.add_argument("--series-file", type=str,
                    help="Output path for single NPZ time series (optional).")
    ap.add_argument("--frames", type=int, default=160, help="Number of frames (T)")
    ap.add_argument("--points", type=int, default=6000, help="Number of points (N)")
    ap.add_argument("--radius", type=float, default=1.0, help="Cloud radius for synthetic geometry")
    ap.add_argument("--seed", type=int, default=7, help="Random seed for point placement")
    return ap.parse_args()


def main():
    args = parse_args()
    if not any([args.points_dir, args.frame_dir, args.series_file]):
        raise SystemExit("Specify at least --out-dir/--points-dir, --frame-dir, or --series-file.")

    xyz = make_static_point_cloud(points=args.points, radius=args.radius, seed=args.seed)
    val = make_time_series_values(xyz, frames=args.frames)

    if args.points_dir:
        write_point_files(Path(args.points_dir), xyz=xyz, val_series=val)
        print(f"Wrote {xyz.shape[0]} per-point NPZ files under {args.points_dir}")

    if args.frame_dir:
        write_frame_series(Path(args.frame_dir), frames=args.frames, xyz=xyz, val=val)
        print(f"Wrote per-frame NPZ files under {args.frame_dir}")

    if args.series_file:
        write_static_series(Path(args.series_file), xyz=xyz, val=val)
        print(f"Wrote NPZ time-series file to {args.series_file}")


if __name__ == "__main__":
    main()
