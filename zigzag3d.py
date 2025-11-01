#!/usr/bin/env python3
"""
Strict XY-then-Z volume path generator + G-code exporter + 3D plot

Invariants:
  - Within each layer: Z is constant; moves are XY-only.
  - Between layers: moves are Z-only at the SAME (x,y).
  - The next layer's raster starts exactly at the previous layer's end (x,y).

Examples:
  python strict_xy_then_z.py --W 120 --H 80 --Z 40 --step 5 --layer 2 --export fill.nc
  python strict_xy_then_z.py --W 100 --H 100 --Z 50 --step 4 --layer 1.5 --plot --save plot.png
"""

import math
import argparse
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import matplotlib
# Try interactive backends; fall back silently if unavailable
for _bk in ("QtAgg", "TkAgg", "MacOSX"):
    try:
        matplotlib.use(_bk)
        break
    except Exception:
        pass
import matplotlib.pyplot as plt

Pt2 = Tuple[float, float]
Pt3 = Tuple[float, float, float]

# -------------------- Data --------------------

@dataclass
class Params:
    W: float = 100.0       # X size (mm)
    H: float = 80.0        # Y size (mm)
    Z_top: float = 40.0    # Z size (mm)
    stepover: float = 5.0  # spacing between scanlines (mm)
    layer_h: float = 2.0   # Z increment per layer (mm)
    start_left: bool = True
    start_bottom: bool = True

@dataclass
class Feeds:
    feed_xy: float = 1200.0  # mm/min for XY contours
    feed_z: float = 600.0    # mm/min for vertical transitions

# -------------------- Path generation --------------------

def raster_xy(W: float, H: float, step: float,
              start_left: bool, start_bottom: bool) -> List[Pt2]:
    """Return a boustrophedon path across [0..W]x[0..H] with lines along X, stepping in Y."""
    n = max(1, int(math.floor(H / step)) + 1)
    y_levels = [0.0 + i * step for i in range(n)]
    if not y_levels or y_levels[-1] < H - 1e-9:
        y_levels.append(H)
    if not start_bottom:
        y_levels.reverse()

    pts: List[Pt2] = []
    left_to_right = start_left
    for y in y_levels:
        if left_to_right:
            pts += [(0.0, y), (W, y)]
        else:
            pts += [(W, y), (0.0, y)]
        left_to_right = not left_to_right

    # Deduplicate consecutive points
    out = [pts[0]]
    for p in pts[1:]:
        if p != out[-1]:
            out.append(p)
    return out

def build_strict_xy_then_z(p: Params) -> List[Pt3]:
    """
    Build a 3D path with strict separation:
      - Layer k: trace XY raster at z_k (Z constant).
      - Then: one vertical segment to z_{k+1} at SAME (x,y).
      - Next layer's raster is reversed as needed so it starts at that (x,y).
    """
    base = raster_xy(p.W, p.H, p.stepover, p.start_left, p.start_bottom)
    base_rev = list(reversed(base))

    layers = max(1, int(math.floor(p.Z_top / p.layer_h)) + 1)
    z_levels = [min(p.Z_top, k * p.layer_h) for k in range(layers)]

    path: List[Pt3] = []

    # Layer 0
    z = z_levels[0]
    path.append((base[0][0], base[0][1], z))
    for (x, y) in base[1:]:
        path.append((x, y, z))
    end_xy = base[-1]

    # Subsequent layers
    for z in z_levels[1:]:
        # Pure vertical to next Z at SAME (x,y)
        path.append((end_xy[0], end_xy[1], z))

        # Choose orientation so the next layer starts at current (x,y)
        if end_xy == base[0]:
            layer_pts = base
        elif end_xy == base[-1]:
            layer_pts = base_rev
        else:
            # Numerical tolerance fallback: choose whichever endpoint is closer
            if _dist2(end_xy, base[0]) <= _dist2(end_xy, base[-1]):
                layer_pts = base
            else:
                layer_pts = base_rev

        # Trace the layer at constant Z (skip first point; already at it)
        for (x, y) in layer_pts[1:]:
            path.append((x, y, z))
        end_xy = layer_pts[-1]

    # Optionally cap to exact top (if last z < Z_top due to rounding)
    if path and abs(path[-1][2] - p.Z_top) > 1e-9:
        x, y, _ = path[-1]
        path.append((x, y, p.Z_top))
    return path

def _dist2(a: Pt2, b: Pt2) -> float:
    dx, dy = a[0]-b[0], a[1]-b[1]
    return dx*dx + dy*dy

# -------------------- Safety check --------------------

def assert_no_slant(path: List[Pt3], eps: float = 1e-12):
    """Every segment must be (dz==0) OR (dx==0 AND dy==0)."""
    P = np.array(path, float)
    d = np.diff(P, axis=0)
    dx, dy, dz = d[:, 0], d[:, 1], d[:, 2]
    ok = (np.abs(dz) < eps) | ((np.abs(dx) < eps) & (np.abs(dy) < eps))
    if not np.all(ok):
        bad_idx = np.where(~ok)[0][:10].tolist()
        raise AssertionError(f"Slanted segments at indices {bad_idx}. Path must be XY-only or Z-only per move.")

# -------------------- G-code --------------------

def to_gcode(path: List[Pt3], feeds: Feeds,
             units: str = "G21", absolute: str = "G90") -> str:
    if not path:
        return ""
    lines = []
    lines += ["( strict XY-then-Z path )", "G17", units, absolute, "G94"]

    x0, y0, z0 = path[0]
    lines.append(f"G0 X{x0:.4f} Y{y0:.4f} Z{z0:.4f}")

    last = (x0, y0, z0)
    for (x, y, z) in path[1:]:
        dx, dy, dz = x - last[0], y - last[1], z - last[2]
        if abs(dz) > 0 and abs(dx) < 1e-12 and abs(dy) < 1e-12:
            lines.append(f"F{feeds.feed_z:.3f}")
        else:
            lines.append(f"F{feeds.feed_xy:.3f}")
        lines.append(f"G1 X{x:.4f} Y{y:.4f} Z{z:.4f}")
        last = (x, y, z)

    lines += ["M5", "M30"]
    return "\n".join(lines)

# -------------------- Plot --------------------

def plot_path(path: List[Pt3], W: float, H: float, Z_top: float, save_png: str = ""):
    P = np.array(path, float)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(P[:, 0], P[:, 1], P[:, 2], linewidth=1.0)
    ax.scatter([P[0, 0]], [P[0, 1]], [P[0, 2]], s=30)
    # box frame
    def edge(p, q): ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]], linewidth=1.0)
    c = [(0, 0, 0), (W, 0, 0), (W, H, 0), (0, H, 0),
         (0, 0, Z_top), (W, 0, Z_top), (W, H, Z_top), (0, H, Z_top)]
    for i in range(4): edge(c[i], c[(i+1) % 4])
    for i in range(4, 8): edge(c[i], c[4 + (i - 3) % 4])
    for i in range(4): edge(c[i], c[i + 4])
    ax.set_box_aspect((W, H, Z_top))
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.set_zlim(0, Z_top)
    ax.set_xlabel("X (mm)"); ax.set_ylabel("Y (mm)"); ax.set_zlabel("Z (mm)")
    ax.set_title("Strict XY-then-Z Path (no slant)")
    plt.tight_layout()
    if save_png:
        plt.savefig(save_png, dpi=200)
        print(f"Saved plot: {save_png}")
    else:
        try:
            plt.show()
        except Exception:
            print("Headless backend; re-run with --save plot.png or install a GUI backend (Tk/Qt).")

# -------------------- CLI --------------------

def parse_args():
    p = argparse.ArgumentParser(description="Strict XY-then-Z volume path generator & G-code exporter")
    p.add_argument("--W", type=float, default=100.0, help="X size (mm)")
    p.add_argument("--H", type=float, default=80.0, help="Y size (mm)")
    p.add_argument("--Z", type=float, default=40.0, help="Z size (mm)")
    p.add_argument("--step", type=float, default=5.0, help="XY stepover (mm)")
    p.add_argument("--layer", type=float, default=2.0, help="Layer height (mm)")
    p.add_argument("--start-left", action="store_true", default=True, help="First pass goes left->right")
    p.add_argument("--start-right", dest="start_left", action="store_false", help="First pass goes right->left")
    p.add_argument("--start-bottom", action="store_true", default=True, help="First pass near Y=0")
    p.add_argument("--start-top", dest="start_bottom", action="store_false", help="First pass near Y=H")
    p.add_argument("--feed-xy", type=float, default=1200.0, help="Feed for XY moves (mm/min)")
    p.add_argument("--feed-z", type=float, default=600.0, help="Feed for Z moves (mm/min)")
    p.add_argument("--export", type=str, default="", help="Write G-code to this file")
    p.add_argument("--plot", action="store_true", help="Show a 3D plot")
    p.add_argument("--save", type=str, default="", help="Save plot PNG to this path")
    p.add_argument("--no-assert", action="store_true", help="Skip slant-safety assertion")
    return p.parse_args()

def main():
    a = parse_args()
    p = Params(W=a.W, H=a.H, Z_top=a.Z, stepover=a.step, layer_h=a.layer,
               start_left=a.start_left, start_bottom=a.start_bottom)
    f = Feeds(feed_xy=a.feed_xy, feed_z=a.feed_z)

    path = build_strict_xy_then_z(p)

    if not a.no_assert:
        assert_no_slant(path)

    if a.export:
        gcode = to_gcode(path, f)
        with open(a.export, "w") as fh:
            fh.write(gcode)
        print(f"Wrote G-code: {a.export}")

    if a.plot or a.save:
        plot_path(path, p.W, p.H, p.Z_top, save_png=a.save)

if __name__ == "__main__":
    main()

