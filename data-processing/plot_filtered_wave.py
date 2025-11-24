#!/usr/bin/env python3
"""
Quick visualizer for filtered PDM NPZ outputs produced by post-process-pdm.py.

Example:
  python data-processing/plot_filtered_wave.py filtered-data/point_00004.npz --time-axis --length 5000
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_wave(path: Path) -> tuple[np.ndarray, float | None]:
    data = np.load(path)
    val = np.asarray(data["val"], dtype=np.float64)
    fs_out = float(data["fs_out"]) if "fs_out" in data else None
    return val, fs_out


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot filtered audio waveform from a post-processed NPZ.")
    parser.add_argument("npz", type=Path, help="NPZ file produced by post-process-pdm.py")
    parser.add_argument("--start", type=int, default=0, help="Start sample index (default: 0).")
    parser.add_argument("--length", type=int, help="Number of samples to plot (default: to end).")
    parser.add_argument("--time-axis", action="store_true",
                        help="Use seconds on the x-axis (requires fs_out in NPZ).")
    parser.add_argument("--save", type=Path, help="Optional path to save the plot instead of showing it.")
    args = parser.parse_args()

    if not args.npz.exists():
        raise SystemExit(f"NPZ not found: {args.npz}")

    signal, fs_out = load_wave(args.npz)
    if signal.size == 0:
        raise SystemExit("Signal is empty.")

    start = max(0, args.start)
    end = signal.size if args.length is None else min(signal.size, max(start, start + args.length))
    if end <= start:
        raise SystemExit(f"Empty slice: start={start}, end={end}.")

    segment = signal[start:end]
    if args.time_axis and fs_out:
        x = np.arange(segment.size) / fs_out + start / fs_out
        xlabel = "Time (s)"
    else:
        x = np.arange(start, start + segment.size)
        xlabel = "Sample"

    plt.figure(figsize=(10, 4))
    plt.plot(x, segment, lw=0.8)
    plt.xlabel(xlabel)
    plt.ylabel("Amplitude")
    title_fs = f", fs_out={fs_out/1e3:.1f} kHz" if fs_out else ""
    plt.title(f"{args.npz.name}: {segment.size} samples{title_fs}")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Wrote plot to {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
