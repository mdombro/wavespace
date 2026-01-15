#!/usr/bin/env python3
"""
PyVista-based animated point-cloud viewer.

- Visualizes time-varying 3D point clouds where per-point scalar values map to color and size
- Playback controls: Play/Pause, time slider scrubbing, FPS-configurable timer
- Display controls: colormap selector, marker size scaling slider, configurable downsampling
- Data sources:
    * Directory of per-frame NPZ files (xyz[N,3], val[N])
    * Single NPZ series (xyz[T,N,3] or xyz[N,3], val[T,N])
    * Synthetic demo (expanding wavefront)

Dependencies:
    pip install numpy pyvista pyvistaqt PyQt5
"""

import argparse
import os
import sys
import re
from pathlib import Path
from dataclasses import dataclass

# Detect early whether the current environment has a display server. This lets us
# fail fast with a useful message instead of PyQt attempting to load the xcb
# platform plugin and aborting the process when no GUI is available.
DISPLAY_ENV_VARS = ("DISPLAY", "WAYLAND_DISPLAY", "WAYLAND_DISPLAY_1", "MIR_SOCKET")
HEADLESS_ENV = sys.platform.startswith("linux") and not any(
    os.environ.get(var) for var in DISPLAY_ENV_VARS
)

# Recognise --offscreen early so we can configure Qt before importing PyQt5.
OFFSCREEN_REQUESTED = "--offscreen" in sys.argv[1:]
if OFFSCREEN_REQUESTED:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from PyQt5 import QtCore, QtWidgets
import pyvista as pv
from pyvistaqt import QtInteractor


# ---------------- Data providers ----------------


class ProviderBase:
    def __len__(self):
        raise NotImplementedError

    def get(self, t):
        """Return xyz[N,3], val[N] for frame t."""
        raise NotImplementedError

    def static_xyz(self) -> bool:
        return False

    def max_points(self) -> int:
        return getattr(self, "_N", None)

    def amplitude_range(self):
        return (-1.0, 1.0)


class NPZFramesProvider(ProviderBase):
    def __init__(self, path, pattern, frames):
        self.path = path
        self.pattern = pattern
        self.T = int(frames)
        if self.T <= 0:
            raise ValueError("Frame count must be positive.")
        self._amp_min = float("inf")
        self._amp_max = float("-inf")
        self._prefetch = {}
        self._first_xyz = None
        self._static = True
        self._scan_amplitude_range()
        if 0 in self._prefetch:
            xyz0, _ = self._prefetch[0]
        else:
            xyz0, _ = self._load_frame(0)
        self._N = xyz0.shape[0]
        if self._first_xyz is None:
            self._first_xyz = xyz0.copy()

    def __len__(self):
        return self.T

    def _filename(self, t):
        return os.path.join(self.path, self.pattern.format(t))

    def _load_frame(self, t, use_prefetch=True):
        if use_prefetch and t in self._prefetch:
            xyz, val = self._prefetch.pop(t)
            return xyz.copy(), val.copy()
        fn = self._filename(t)
        if not os.path.exists(fn):
            raise FileNotFoundError(fn)
        data = np.load(fn, allow_pickle=False)
        xyz = np.asarray(data["xyz"], dtype=np.float32)
        val = np.asarray(data["val"], dtype=np.float32).reshape(-1)
        if val.size:
            vmin = float(np.min(val))
            vmax = float(np.max(val))
            if vmin < self._amp_min:
                self._amp_min = vmin
            if vmax > self._amp_max:
                self._amp_max = vmax
        return xyz, val

    def _scan_amplitude_range(self):
        for t in range(self.T):
            xyz, val = self._load_frame(t, use_prefetch=False)
            if t == 0:
                self._prefetch[0] = (xyz.copy(), val.copy())
                self._first_xyz = xyz.copy()
                self._N = xyz.shape[0]
        if not np.isfinite(self._amp_min):
            self._amp_min = 0.0
        if not np.isfinite(self._amp_max):
            self._amp_max = 0.0

    def get(self, t):
        xyz, val = self._load_frame(int(t))
        if self._static and not np.array_equal(self._first_xyz, xyz):
            self._static = False
        return xyz, val

    def static_xyz(self) -> bool:
        return bool(self._static)

    def amplitude_range(self):
        return (self._amp_min, self._amp_max)


class NPZSeriesProvider(ProviderBase):
    def __init__(self, series_path):
        data = np.load(series_path, allow_pickle=False)
        self._val = np.asarray(data["val"], dtype=np.float32)
        self.T, self.N = self._val.shape
        self._N = self.N
        self._xyz = data["xyz"]
        self._amp_min = float(np.min(self._val)) if self._val.size else 0.0
        self._amp_max = float(np.max(self._val)) if self._val.size else 0.0
        if self._xyz.ndim == 2 and self._xyz.shape == (self.N, 3):
            self._mode = "static"
        elif self._xyz.ndim == 3 and self._xyz.shape == (self.T, self.N, 3):
            self._mode = "moving"
        else:
            raise ValueError("xyz must be (N,3) or (T,N,3)")

    def __len__(self):
        return self.T

    def get(self, t):
        tt = int(t)
        if self._mode == "static":
            xyz = self._xyz.astype(np.float32)
        else:
            xyz = self._xyz[tt].astype(np.float32)
        return xyz, self._val[tt]

    def static_xyz(self):
        return self._mode == "static"

    def amplitude_range(self):
        return (self._amp_min, self._amp_max)


class PointFilesProvider(ProviderBase):
    """
    Loads a directory of per-point NPZ files (point_{index}.npz) where each file contains:
      - xyz: float32 array with shape [3] (or [1,3])
      - val: float32 array with shape [T]
    """

    def __init__(self, path, pattern="point_{:05d}.npz", points=None):
        self.path = path
        self.pattern = pattern
        self._xyz = None
        self._val_series = None
        self._T = None

        files = list_point_files(path, pattern)
        if points:
            files = files[: int(points)]
        self._files = files
        self._N = len(self._files)
        if self._N == 0:
            raise ValueError("No point files found. Check --path/--pattern or use --demo.")

        self._load_all()

    def __len__(self):
        return self._T

    def _load_all(self):
        xyz_list = []
        series_list = []
        min_frames = None
        for fn in self._files:
            data = np.load(fn, allow_pickle=False)
            if "xyz" not in data or "val" not in data:
                raise KeyError(f"Point file {fn} must contain 'xyz' and 'val'")
            xyz = np.asarray(data["xyz"], dtype=np.float32).reshape(-1)
            if xyz.size != 3:
                raise ValueError(f"'xyz' in {fn} must have 3 elements, got {xyz.shape}")
            series = np.asarray(data["val"], dtype=np.float32).reshape(-1)
            frame_count = series.shape[0]
            if min_frames is None or frame_count < min_frames:
                min_frames = frame_count
            xyz_list.append(xyz)
            series_list.append(series)

        if min_frames is None or min_frames == 0:
            raise ValueError("Point files did not contain any frames.")

        trimmed = []
        kept_xyz = []
        kept_files = []
        for series, xyz, fn in zip(series_list, xyz_list, self._files):
            if series.shape[0] < min_frames:
                print(
                    f"[point-loader] dropping {fn}: only {series.shape[0]} frame(s) available (need {min_frames})",
                    file=sys.stderr,
                )
                continue
            if series.shape[0] > min_frames:
                print(
                    f"[point-loader] trimming {fn} from {series.shape[0]} to {min_frames} frame(s)",
                    file=sys.stderr,
                )
            trimmed.append(series[:min_frames])
            kept_xyz.append(xyz)
            kept_files.append(fn)

        if not trimmed:
            raise ValueError("All point files were dropped due to insufficient frame counts.")

        self._files = kept_files
        self._xyz = np.vstack(kept_xyz)
        self._ragged_series = [np.asarray(series, dtype=np.float32) for series in trimmed]
        self._val_series = None
        self._T = max(series.shape[0] for series in self._ragged_series)
        mins = [float(np.min(series)) for series in self._ragged_series if series.size]
        maxs = [float(np.max(series)) for series in self._ragged_series if series.size]
        self._amp_min = min(mins) if mins else 0.0
        self._amp_max = max(maxs) if maxs else 0.0

    def get(self, t):
        if self._val_series is None:
            self._materialize_series()
        tt = int(t)
        if tt < 0 or tt >= self._T:
            raise IndexError(f"Frame {tt} out of range for T={self._T}")
        return self._xyz, self._val_series[tt]

    def static_xyz(self):
        return True

    def _materialize_series(self):
        max_len = self._T
        padded = []
        for series, fn in zip(self._ragged_series, self._files):
            length = series.shape[0]
            if length < max_len:
                pad = np.zeros(max_len - length, dtype=np.float32)
                series = np.concatenate([series, pad])
            elif length > max_len:
                series = series[:max_len]
            padded.append(series)
        self._val_series = np.stack(padded, axis=1)
        if self._val_series.size:
            self._amp_min = float(np.min(self._val_series))
            self._amp_max = float(np.max(self._val_series))

    def amplitude_range(self):
        return (self._amp_min, self._amp_max)


def make_demo_provider(T=160, N=1000, radius=1.0):
    rng = np.random.default_rng(7)
    u = rng.random(N)
    cost = rng.uniform(-1, 1, N)
    phi = rng.uniform(0, 2 * np.pi, N)
    r = radius * np.power(u, 1.0 / 3.0)
    s = np.sqrt(1 - cost**2)
    xyz = np.stack(
        [r * s * np.cos(phi), r * s * np.sin(phi), r * cost], axis=-1
    ).astype(np.float32)

    val_series = np.empty((T, N), dtype=np.float32)
    rr = np.linalg.norm(xyz, axis=1)
    sigma = 0.16 * radius
    denom = 2 * sigma * sigma
    for t in range(T):
        tau = t / max(1, T - 1)
        r0 = 0.8 * radius * tau
        val = np.exp(-((rr - r0) ** 2) / denom)
        m = val.max()
        if m > 0:
            val = val / m
        val_series[t] = val.astype(np.float32)

    class Demo(ProviderBase):
        def __init__(self, xyz_data, val_data):
            self._xyz = xyz_data
            self._val = val_data
            if self._val.size:
                self._amp_min = float(self._val.min())
                self._amp_max = float(self._val.max())
            else:
                self._amp_min = 0.0
                self._amp_max = 0.0

        def __len__(self):
            return self._val.shape[0]

        def get(self, t):
            return self._xyz, self._val[int(t)]

        def static_xyz(self):
            return True

        def max_points(self):
            return self._xyz.shape[0]

        def amplitude_range(self):
            return (self._amp_min, self._amp_max)

    return Demo(xyz, val_series)


# ---------------- Visualization helpers ----------------


COLORSCALES = ["RdBu", "PuOr", "BrBG", "PiYG", "Spectral", "coolwarm", "seismic", "RdYlGn"]
VIZ_MODES = ["dots", "cloud", "splat"]


@dataclass
class VizParams:
    cmap: str = "RdBu"
    size_scale: float = 1.0
    downsample: int = 0  # 0 = no downsample
    amp_threshold: float = 0.0
    viz_mode: str = "dots"  # "dots", "cloud", or "splat"
    grid_resolution: int = 15  # voxel grid resolution for cloud mode
    splat_radius: float = 1.0  # radius multiplier for Gaussian splats


def _normalize_cmap_name(name: str) -> str:
    """Return a Matplotlib-friendly colormap identifier."""
    if not name:
        return COLORSCALES[0]
    cleaned = (name or "").strip()
    if cleaned in COLORSCALES:
        return cleaned
    lower = cleaned.lower()
    return lower if lower else COLORSCALES[0]


def _downsample(xyz, val, step):
    step = int(step or 0)
    if step <= 1:
        return xyz, val
    idx = np.arange(0, xyz.shape[0], step, dtype=np.int64)
    return xyz[idx], val[idx]


def _safe_diag_extent(xyz):
    if xyz.size == 0:
        return 1.0
    bounds = np.ptp(xyz, axis=0)
    diag = float(np.linalg.norm(bounds))
    return diag if diag > 0 else 1.0


def _interpolate_to_volume(xyz, val, resolution=40, padding=0.1):
    """
    Interpolate sparse point measurements to a regular 3D volume grid.

    Uses inverse distance weighting for fast interpolation.
    Returns a pv.ImageData with 'val' scalar field.
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    val = np.asarray(val, dtype=np.float32).reshape(-1)

    if xyz.shape[0] == 0:
        grid = pv.ImageData()
        grid.dimensions = (2, 2, 2)
        grid.origin = (0, 0, 0)
        grid.spacing = (1, 1, 1)
        grid.point_data["val"] = np.zeros(8, dtype=np.float32)
        return grid

    # Compute bounds with padding
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    ranges = maxs - mins
    ranges = np.where(ranges < 1e-6, 1.0, ranges)  # avoid zero range
    pad = ranges * padding
    mins = mins - pad
    maxs = maxs + pad

    # Create grid coordinates
    gx = np.linspace(mins[0], maxs[0], resolution)
    gy = np.linspace(mins[1], maxs[1], resolution)
    gz = np.linspace(mins[2], maxs[2], resolution)
    GX, GY, GZ = np.meshgrid(gx, gy, gz, indexing='ij')
    grid_points = np.stack([GX.ravel(), GY.ravel(), GZ.ravel()], axis=1)

    # Inverse distance weighting interpolation
    # For each grid point, compute weighted average of nearby measurement values
    n_grid = grid_points.shape[0]
    n_pts = xyz.shape[0]

    # Compute distances from each grid point to each measurement point
    # Use chunking to avoid memory issues with large grids
    chunk_size = 5000
    grid_vals = np.zeros(n_grid, dtype=np.float32)

    power = 2.0  # IDW power parameter

    for i in range(0, n_grid, chunk_size):
        end = min(i + chunk_size, n_grid)
        chunk_pts = grid_points[i:end]

        # Distance matrix: (chunk_size, n_pts)
        diff = chunk_pts[:, np.newaxis, :] - xyz[np.newaxis, :, :]
        dist = np.sqrt(np.sum(diff ** 2, axis=2))

        # Avoid division by zero
        dist = np.maximum(dist, 1e-10)

        # Inverse distance weights
        weights = 1.0 / (dist ** power)
        weights_sum = weights.sum(axis=1, keepdims=True)
        weights_norm = weights / weights_sum

        # Weighted average
        grid_vals[i:end] = (weights_norm * val[np.newaxis, :]).sum(axis=1)

    # Create PyVista ImageData
    grid = pv.ImageData()
    grid.dimensions = (resolution, resolution, resolution)
    grid.origin = tuple(mins)
    spacing = (maxs - mins) / (resolution - 1)
    grid.spacing = tuple(spacing)
    grid.point_data["val"] = grid_vals.astype(np.float32)

    return grid


def _compute_sizes(val, size_scale, scale_factor, amp_abs_max):
    if val.size == 0:
        return np.empty((0,), dtype=np.float32)
    denom = amp_abs_max if amp_abs_max > 1e-12 else 1.0
    mag = np.clip(np.abs(val) / denom, 0.0, 1.0)
    base = 2.0 + 8.0 * mag  # baseline range roughly matching previous defaults
    return scale_factor * size_scale * base


# ---------------- PyVista Qt viewer ----------------


class PointCloudViewer(QtWidgets.QMainWindow):
    def __init__(self, provider: ProviderBase, fps=24, default_params=None):
        super().__init__()
        self.setWindowTitle("PyVista Point Cloud Viewer")
        self.provider = provider
        self.T = len(provider)
        self.params = default_params or VizParams()

        self.current_frame = 0
        self.is_playing = False
        self.downsample_step = max(1, int(self.params.downsample) or 1)
        self.size_scale = float(self.params.size_scale) if self.params.size_scale > 0 else 1.0
        self.size_scale = min(4.0, max(0.1, self.size_scale))
        self.cmap = self.params.cmap if self.params.cmap in COLORSCALES else COLORSCALES[0]
        self.static_xyz = provider.static_xyz()
        amp_min, amp_max = provider.amplitude_range()
        self.amp_min = float(amp_min)
        self.amp_max = float(amp_max)
        base_amp = max(abs(self.amp_min), abs(self.amp_max))
        self._amp_slider_enabled = base_amp > 0
        self.amp_abs_max = base_amp if base_amp > 0 else 1.0
        self.amp_threshold = min(max(0.0, self.params.amp_threshold), self.amp_abs_max)
        self._current_scale_abs = self.amp_abs_max
        self._clim = (-self.amp_abs_max, self.amp_abs_max)
        self._amp_slider_resolution = 1000

        xyz0, val0 = provider.get(0)
        xyz0, val0 = _downsample(xyz0, val0, self.params.downsample)
        xyz0, val0 = self._apply_amp_threshold(xyz0, val0)
        self._base_extent = _safe_diag_extent(xyz0)
        self._size_scale = self._base_extent / 150.0
        self._glyph_geom = pv.Sphere(radius=1.0, theta_resolution=12, phi_resolution=12)

        self._mesh = None
        self._glyph = None
        self._actor = None
        self.label_frame = None
        self.label_downsample = None
        self.label_amp_threshold = None

        self._build_ui(fps)
        self._initialize_scene(xyz0, val0)
        self._update_status_labels()

    # ----- UI construction -----

    def _build_ui(self, fps):
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)

        # Playback controls row
        top_row = QtWidgets.QHBoxLayout()
        self.btn_play = QtWidgets.QPushButton("⏵ Play")
        self.btn_pause = QtWidgets.QPushButton("⏸ Pause")
        self.slider_frame = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_frame.setRange(0, max(0, self.T - 1))
        self.slider_frame.setSingleStep(1)
        self.slider_frame.setPageStep(1)
        self.label_frame = QtWidgets.QLabel()
        self.label_downsample = QtWidgets.QLabel()
        self.label_size_scale = None
        self.slider_amp = None

        top_row.addWidget(self.btn_play)
        top_row.addWidget(self.btn_pause)
        top_row.addWidget(self.slider_frame)
        top_row.addWidget(self.label_frame)
        main_layout.addLayout(top_row)

        # Settings row
        settings_row = QtWidgets.QHBoxLayout()

        # Colormap
        cmap_layout = QtWidgets.QVBoxLayout()
        cmap_layout.addWidget(QtWidgets.QLabel("Colormap"))
        self.combo_cmap = QtWidgets.QComboBox()
        self.combo_cmap.addItems(COLORSCALES)
        self.combo_cmap.setCurrentText(self.cmap)
        cmap_layout.addWidget(self.combo_cmap)
        settings_row.addLayout(cmap_layout)

        # Size scaling slider
        self.slider_size_scale = self._make_slider(10, 400, int(round(self.size_scale * 100)))
        size_layout = QtWidgets.QVBoxLayout()
        size_layout.addWidget(QtWidgets.QLabel("Size scale"))
        size_layout.addWidget(self.slider_size_scale)
        self.label_size_scale = QtWidgets.QLabel(self._format_size_scale_label(self.size_scale))
        size_layout.addWidget(self.label_size_scale)
        settings_row.addLayout(size_layout)

        # Downsample slider + note
        self.slider_downsample = self._make_slider(0, 20, self.params.downsample)
        down_layout = QtWidgets.QVBoxLayout()
        label_row = QtWidgets.QHBoxLayout()
        label_row.addWidget(QtWidgets.QLabel("Downsample (every Nth point)"))
        label_row.addWidget(self.label_downsample)
        label_row.addStretch(1)
        down_layout.addLayout(label_row)
        down_layout.addWidget(self.slider_downsample)
        settings_row.addLayout(down_layout)

        # Amplitude threshold slider
        amp_layout = QtWidgets.QVBoxLayout()
        amp_layout.addWidget(QtWidgets.QLabel("Amplitude threshold (|value|)"))
        self.slider_amp = self._make_slider(
            0,
            self._amp_slider_resolution,
            self._amp_value_to_slider(self.amp_threshold),
        )
        if not self._amp_slider_enabled:
            self.slider_amp.setEnabled(False)
        amp_layout.addWidget(self.slider_amp)
        self.label_amp_threshold = QtWidgets.QLabel(self._format_amp_label(self.amp_threshold))
        amp_layout.addWidget(self.label_amp_threshold)
        settings_row.addLayout(amp_layout)

        main_layout.addLayout(settings_row)

        # PyVista view
        self.plotter = QtInteractor(self)
        self.plotter.set_background("black")
        main_layout.addWidget(self.plotter)

        # Timer for playback
        self.timer = QtCore.QTimer(self)
        interval_ms = int(1000 / max(1, fps))
        self.timer.setInterval(interval_ms)
        self.timer.timeout.connect(self._advance_frame)

        # Signal wiring
        self.btn_play.clicked.connect(self.start_playback)
        self.btn_pause.clicked.connect(self.stop_playback)
        self.slider_frame.valueChanged.connect(self._on_frame_slider)
        self.combo_cmap.currentTextChanged.connect(self._on_cmap_changed)
        self.slider_size_scale.valueChanged.connect(self._on_size_scale_changed)
        self.slider_downsample.valueChanged.connect(self._on_downsample_changed)
        self.slider_amp.valueChanged.connect(self._on_amp_threshold_changed)

    def _frame_abs_value(self, val) -> float:
        arr = np.asarray(val, dtype=np.float32)
        if arr.size == 0:
            return 0.0
        return float(np.max(np.abs(arr)))

    def _select_scale_abs(self, frame_abs: float) -> float:
        if frame_abs and frame_abs > 0:
            return frame_abs
        if self.amp_abs_max > 0:
            return self.amp_abs_max
        return 1.0

    def _update_scale_and_clim(self, frame_abs: float, update_mapper: bool = False) -> float:
        scale_abs = self._select_scale_abs(frame_abs)
        if scale_abs <= 0:
            scale_abs = 1.0
        self._current_scale_abs = scale_abs
        self._clim = (-scale_abs, scale_abs)
        if update_mapper:
            self._apply_color_limits_to_mapper()
        return scale_abs

    def _apply_color_limits_to_mapper(self):
        vtk_mapper = self._get_actor_mapper()
        if vtk_mapper is None:
            return
        self._ensure_mapper_scalar_settings(vtk_mapper)
        if hasattr(vtk_mapper, "SetScalarRange"):
            vtk_mapper.SetScalarRange(*self._clim)
        lut = None
        if hasattr(vtk_mapper, "GetLookupTable"):
            lut = vtk_mapper.GetLookupTable()
        if lut is not None:
            if hasattr(lut, "SetRange"):
                lut.SetRange(*self._clim)
            elif hasattr(lut, "scalar_range"):
                try:
                    lut.scalar_range = self._clim
                except Exception:
                    pass
        if hasattr(vtk_mapper, "Modified"):
            vtk_mapper.Modified()

    def _format_size_scale_label(self, value):
        return f"{value:.2f}×"

    def _make_slider(self, minimum, maximum, value):
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(int(minimum), int(maximum))
        slider.setSingleStep(1)
        slider.setPageStep(1)
        slider.setValue(int(round(value)))
        return slider

    def _amp_value_to_slider(self, value):
        if self.amp_abs_max <= 0:
            return 0
        frac = float(np.clip(value / self.amp_abs_max, 0.0, 1.0))
        return int(round(frac * self._amp_slider_resolution))

    def _slider_to_amp_value(self, slider_value):
        if self.amp_abs_max <= 0:
            return 0.0
        return (float(slider_value) / float(self._amp_slider_resolution)) * self.amp_abs_max

    def _format_amp_label(self, value):
        return f"|val| ≥ {value:.4g}" if value > 0 else "No amplitude cutoff"

    def _update_amp_label(self):
        if self.label_amp_threshold is not None:
            self.label_amp_threshold.setText(self._format_amp_label(self.amp_threshold))

    def _wrap_slider(self, label, slider):
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(QtWidgets.QLabel(label))
        layout.addWidget(slider)
        return layout

    # ----- Plot initialization -----

    def _initialize_scene(self, xyz, val):
        frame_abs = self._frame_abs_value(val)
        scale_abs = self._update_scale_and_clim(frame_abs, update_mapper=False)
        mesh = self._build_polydata(xyz, val, scale_abs)
        if mesh.n_points:
            glyph = mesh.glyph(scale="scale", geom=self._glyph_geom, orient=False)
        else:
            glyph = pv.PolyData()
        self._apply_scalars_to_glyph(glyph, mesh)
        self._mesh = mesh
        self._glyph = glyph
        self._actor = self.plotter.add_mesh(
            glyph,
            scalars="val",
            cmap=_normalize_cmap_name(self.cmap),
            clim=self._clim,
            lighting=False,
            show_scalar_bar=False,
            opacity=0.5,
        )
        if hasattr(self._actor, "prop"):
            prop = self._actor.prop
            if hasattr(prop, "opacity"):
                try:
                    prop.opacity = 0.5
                except Exception:
                    pass
            if hasattr(prop, "SetOpacity"):
                prop.SetOpacity(0.5)
        elif hasattr(self._actor, "GetProperty"):
            self._actor.GetProperty().SetOpacity(0.5)
        vtk_mapper = self._get_actor_mapper()
        if vtk_mapper is not None:
            self._ensure_mapper_scalar_settings(vtk_mapper)
            if hasattr(vtk_mapper, "SetScalarRange"):
                vtk_mapper.SetScalarRange(*self._clim)
        self.plotter.show_axes()
        self.plotter.render()

    def _build_polydata(self, xyz, val, scale_abs: float):
        xyz = np.asarray(xyz, dtype=np.float32)
        val = np.asarray(val, dtype=np.float32).reshape(-1)
        mesh = pv.PolyData(xyz)
        mesh.point_data["val"] = val
        mesh.point_data.active_scalars_name = "val"
        scale = _compute_sizes(val, self.size_scale, self._size_scale, scale_abs)
        mesh.point_data["scale"] = scale
        return mesh

    def _apply_amp_threshold(self, xyz, val):
        thr = float(self.amp_threshold)
        xyz_arr = np.asarray(xyz, dtype=np.float32)
        val_arr = np.asarray(val, dtype=np.float32).reshape(-1)
        if thr <= 0 or val_arr.size == 0:
            return xyz_arr, val_arr
        mask = np.abs(val_arr) >= thr
        if not np.any(mask):
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0,), dtype=np.float32),
            )
        return xyz_arr[mask], val_arr[mask]

    # ----- UI callbacks -----

    def start_playback(self):
        if not self.is_playing and self.T > 1:
            self.timer.start()
            self.is_playing = True

    def stop_playback(self):
        if self.is_playing:
            self.timer.stop()
            self.is_playing = False

    def _on_frame_slider(self, value):
        self.set_frame(int(value))

    def _on_cmap_changed(self, cmap):
        self.cmap = cmap
        self._update_cmap()

    def _on_size_scale_changed(self, value):
        self.size_scale = max(0.1, float(value) / 100.0)
        if self.label_size_scale:
            self.label_size_scale.setText(self._format_size_scale_label(self.size_scale))
        self._refresh_current_mesh()

    def _on_downsample_changed(self, value):
        step = int(value)
        self.downsample_step = max(1, step if step != 0 else 1)
        self._update_status_labels()
        self._refresh_current_mesh(rebuild_geometry=True)

    def _on_amp_threshold_changed(self, value):
        new_thr = self._slider_to_amp_value(value)
        if abs(new_thr - self.amp_threshold) < 1e-6:
            return
        self.amp_threshold = new_thr
        self._update_amp_label()
        self._refresh_current_mesh(rebuild_geometry=True)

    # ----- Scene updates -----

    def set_frame(self, frame_index, rebuild_geometry=False):
        frame_index = int(frame_index) % max(1, self.T)
        if frame_index == self.current_frame and not rebuild_geometry:
            return
        self.current_frame = frame_index
        xyz, val = self.provider.get(frame_index)
        xyz, val = _downsample(xyz, val, self.downsample_step)
        xyz, val = self._apply_amp_threshold(xyz, val)
        frame_abs = self._frame_abs_value(val)
        scale_abs = self._update_scale_and_clim(frame_abs, update_mapper=True)

        reuse_points = (
            self.static_xyz
            and not rebuild_geometry
            and self._mesh is not None
            and self.amp_threshold <= 0
        )

        if reuse_points:
            vals = np.asarray(val, dtype=np.float32)
            self._mesh.point_data["val"] = vals
            scale = _compute_sizes(vals, self.size_scale, self._size_scale, scale_abs)
            self._mesh.point_data["scale"] = scale
            self._update_glyph(self._mesh, reuse_points=True)
        else:
            self._mesh = self._build_polydata(xyz, val, scale_abs)
            self._update_glyph(self._mesh, reuse_points=False)

        if self.slider_frame.value() != frame_index:
            self.slider_frame.blockSignals(True)
            self.slider_frame.setValue(frame_index)
            self.slider_frame.blockSignals(False)
        self._update_status_labels()

    def _refresh_current_mesh(self, rebuild_geometry=False):
        self.set_frame(self.current_frame, rebuild_geometry=rebuild_geometry)

    def _advance_frame(self):
        if self.T <= 1:
            return
        next_frame = (self.current_frame + 1) % self.T
        self.set_frame(next_frame)

    def _update_cmap(self):
        if not self._actor:
            return
        lut = pv.LookupTable(cmap=_normalize_cmap_name(self.cmap))
        lut.scalar_range = self._clim
        vtk_mapper = self._get_actor_mapper()
        if vtk_mapper is None:
            raise RuntimeError("Could not access mapper on actor for colormap update.")
        self._ensure_mapper_scalar_settings(vtk_mapper)
        if hasattr(vtk_mapper, "SetLookupTable"):
            vtk_mapper.SetLookupTable(lut)
        if hasattr(vtk_mapper, "SetScalarRange"):
            vtk_mapper.SetScalarRange(*self._clim)
        if hasattr(vtk_mapper, "Modified"):
            vtk_mapper.Modified()
        self.plotter.render()

    def _get_actor_mapper(self):
        if not self._actor:
            return None
        mapper = getattr(self._actor, "mapper", None)
        if mapper is None and hasattr(self._actor, "GetMapper"):
            mapper = self._actor.GetMapper()
        if mapper is None:
            return None
        return getattr(mapper, "_vtk_obj", mapper)

    def _ensure_mapper_scalar_settings(self, vtk_mapper):
        if hasattr(vtk_mapper, "SetScalarModeToUsePointFieldData"):
            vtk_mapper.SetScalarModeToUsePointFieldData()
        if hasattr(vtk_mapper, "SelectColorArray"):
            vtk_mapper.SelectColorArray("val")
        if hasattr(vtk_mapper, "ScalarVisibilityOn"):
            vtk_mapper.ScalarVisibilityOn()
        if hasattr(vtk_mapper, "SetColorModeToMapScalars"):
            vtk_mapper.SetColorModeToMapScalars()

    def _frame_label_text(self):
        total = max(0, int(self.T))
        if total <= 0:
            return "Frame: 0/0"
        return f"Frame: {self.current_frame + 1}/{total}"

    def _downsample_label_text(self):
        step = max(1, int(self.downsample_step))
        if step <= 1:
            return "Downsample: off"
        return f"Downsample: every {step}"

    def _update_status_labels(self):
        if self.label_frame is not None:
            self.label_frame.setText(self._frame_label_text())
        if self.label_downsample is not None:
            self.label_downsample.setText(self._downsample_label_text())
        self._update_amp_label()

    def _apply_scalars_to_glyph(self, glyph, mesh):
        if glyph is None or mesh is None:
            return
        if "val" not in mesh.point_data:
            return
        vals = mesh.point_data["val"]
        vals = np.asarray(vals, dtype=np.float32)
        total_points = int(getattr(glyph, "n_points", 0) or 0)
        if total_points and vals.size == 0:
            vals = np.zeros(total_points, dtype=np.float32)
        elif total_points and vals.size and total_points != vals.size:
            reps = int(np.ceil(total_points / float(vals.size)))
            vals = np.repeat(vals, reps)[:total_points]
        glyph.point_data["val"] = vals
        if hasattr(glyph.point_data, "active_scalars_name"):
            glyph.point_data.active_scalars_name = "val"

    def _update_glyph(self, mesh, reuse_points):
        camera_pos = self.plotter.camera_position
        if mesh.n_points:
            glyph = mesh.glyph(scale="scale", geom=self._glyph_geom, orient=False)
        else:
            glyph = pv.PolyData()

        self._apply_scalars_to_glyph(glyph, mesh)
        self._glyph = glyph
        vtk_mapper = self._get_actor_mapper()
        if vtk_mapper is None:
            raise RuntimeError("Could not access mapper on actor for glyph update.")
        self._ensure_mapper_scalar_settings(vtk_mapper)
        if hasattr(vtk_mapper, "SetInputData"):
            vtk_mapper.SetInputData(glyph)
        else:
            vtk_mapper.SetInputDataObject(glyph)
        self._ensure_mapper_scalar_settings(vtk_mapper)
        if hasattr(vtk_mapper, "SetScalarRange"):
            vtk_mapper.SetScalarRange(*self._clim)
        if hasattr(vtk_mapper, "Modified"):
            vtk_mapper.Modified()
        self.plotter.camera_position = camera_pos
        self.plotter.render()

    def closeEvent(self, event):
        self.stop_playback()
        super().closeEvent(event)


# ---------------- Cloud (Volume) Viewer ----------------


class CloudViewer(QtWidgets.QMainWindow):
    """Volumetric cloud visualization of acoustic pressure fields."""

    def __init__(self, provider: ProviderBase, fps=24, default_params=None):
        super().__init__()
        self.setWindowTitle("PyVista Cloud Viewer - Volumetric Pressure Field")
        self.provider = provider
        self.T = len(provider)
        self.params = default_params or VizParams()

        self.current_frame = 0
        self.is_playing = False
        self.grid_resolution = max(10, min(100, self.params.grid_resolution))
        self.cmap = self.params.cmap if self.params.cmap in COLORSCALES else COLORSCALES[0]
        self.opacity_level = 0.5  # 0-1 scale for opacity intensity
        self.show_points = False  # toggle to show measurement points (default off)

        amp_min, amp_max = provider.amplitude_range()
        self.amp_min = float(amp_min)
        self.amp_max = float(amp_max)
        self.amp_abs_max = max(abs(self.amp_min), abs(self.amp_max))
        if self.amp_abs_max <= 0:
            self.amp_abs_max = 1.0
        self._clim = (-self.amp_abs_max, self.amp_abs_max)

        # Volume and point cloud actors
        self._volume_actor = None
        self._points_actor = None
        self._volume_grid = None
        self._needs_full_rebuild = False

        self._build_ui(fps)
        xyz0, val0 = provider.get(0)
        self._initialize_scene(xyz0, val0)
        self._update_status_labels()

    def _build_ui(self, fps):
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)

        # Playback controls row
        top_row = QtWidgets.QHBoxLayout()
        self.btn_play = QtWidgets.QPushButton("⏵ Play")
        self.btn_pause = QtWidgets.QPushButton("⏸ Pause")
        self.slider_frame = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_frame.setRange(0, max(0, self.T - 1))
        self.slider_frame.setSingleStep(1)
        self.label_frame = QtWidgets.QLabel()

        top_row.addWidget(self.btn_play)
        top_row.addWidget(self.btn_pause)
        top_row.addWidget(self.slider_frame)
        top_row.addWidget(self.label_frame)
        main_layout.addLayout(top_row)

        # Settings row
        settings_row = QtWidgets.QHBoxLayout()

        # Colormap
        cmap_layout = QtWidgets.QVBoxLayout()
        cmap_layout.addWidget(QtWidgets.QLabel("Colormap"))
        self.combo_cmap = QtWidgets.QComboBox()
        self.combo_cmap.addItems(COLORSCALES)
        self.combo_cmap.setCurrentText(self.cmap)
        cmap_layout.addWidget(self.combo_cmap)
        settings_row.addLayout(cmap_layout)

        # Grid resolution slider
        res_layout = QtWidgets.QVBoxLayout()
        res_layout.addWidget(QtWidgets.QLabel("Grid Resolution"))
        self.slider_resolution = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_resolution.setRange(15, 80)
        self.slider_resolution.setValue(self.grid_resolution)
        self.label_resolution = QtWidgets.QLabel(f"{self.grid_resolution}³")
        res_layout.addWidget(self.slider_resolution)
        res_layout.addWidget(self.label_resolution)
        settings_row.addLayout(res_layout)

        # Opacity slider
        opacity_layout = QtWidgets.QVBoxLayout()
        opacity_layout.addWidget(QtWidgets.QLabel("Cloud Opacity"))
        self.slider_opacity = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_opacity.setRange(10, 100)
        self.slider_opacity.setValue(int(self.opacity_level * 100))
        self.label_opacity = QtWidgets.QLabel(f"{self.opacity_level:.0%}")
        opacity_layout.addWidget(self.slider_opacity)
        opacity_layout.addWidget(self.label_opacity)
        settings_row.addLayout(opacity_layout)

        # Show points checkbox
        self.checkbox_points = QtWidgets.QCheckBox("Show measurement points")
        self.checkbox_points.setChecked(self.show_points)
        settings_row.addWidget(self.checkbox_points)
        settings_row.addStretch(1)

        main_layout.addLayout(settings_row)

        # PyVista view
        self.plotter = QtInteractor(self)
        self.plotter.set_background([0.05, 0.05, 0.1])
        main_layout.addWidget(self.plotter)

        # Timer for playback
        self.timer = QtCore.QTimer(self)
        interval_ms = int(1000 / max(1, fps))
        self.timer.setInterval(interval_ms)
        self.timer.timeout.connect(self._advance_frame)

        # Signal wiring
        self.btn_play.clicked.connect(self.start_playback)
        self.btn_pause.clicked.connect(self.stop_playback)
        self.slider_frame.valueChanged.connect(self._on_frame_slider)
        self.combo_cmap.currentTextChanged.connect(self._on_cmap_changed)
        self.slider_resolution.valueChanged.connect(self._on_resolution_changed)
        self.slider_opacity.valueChanged.connect(self._on_opacity_changed)
        self.checkbox_points.stateChanged.connect(self._on_show_points_changed)

    def _initialize_scene(self, xyz, val):
        # Create initial volume
        self._volume_grid = _interpolate_to_volume(xyz, val, self.grid_resolution)
        self._add_volume_actor()

        # Add measurement points as small spheres
        self._xyz_points = xyz
        self._add_points_actor(xyz)

        self.plotter.show_axes()
        self.plotter.render()

    def _add_volume_actor(self, force_rebuild=False):
        if self._volume_actor is not None and not force_rebuild:
            # Try to update scalars in place to avoid flickering
            try:
                from vtkmodules.util.numpy_support import numpy_to_vtk
                mapper = self._volume_actor.GetMapper()
                if mapper is not None:
                    input_data = mapper.GetInput()
                    if input_data is not None:
                        new_scalars = self._volume_grid.point_data["val"].astype(np.float32)
                        vtk_arr = numpy_to_vtk(new_scalars, deep=True)
                        vtk_arr.SetName("val")
                        input_data.GetPointData().SetScalars(vtk_arr)
                        input_data.Modified()
                        mapper.Modified()
                        return
            except Exception:
                pass  # Fall through to full rebuild

        if self._volume_actor is not None:
            self.plotter.remove_actor(self._volume_actor)
            self._volume_actor = None

        # Build opacity transfer function - V-shaped with transparent center
        opacity_tf = self._build_opacity_transfer()

        self._volume_actor = self.plotter.add_volume(
            self._volume_grid,
            scalars="val",
            cmap=_normalize_cmap_name(self.cmap),
            opacity=opacity_tf,
            opacity_unit_distance=self._compute_opacity_unit_distance(),
            shade=False,
            clim=self._clim,
            show_scalar_bar=True,
            blending="composite",
        )

    def _build_opacity_transfer(self):
        # Build a V-shaped opacity transfer function:
        # Near-zero values = fully transparent
        # High positive/negative values = very opaque
        # PyVista's opacity list maps linearly across the scalar range
        # clim goes from -max to +max, so middle should be transparent
        # Using 7 points for better control: extremes opaque, center transparent
        base = min(1.0, self.opacity_level * 2.0)  # Boost opacity significantly

        # V-shape: high at edges, zero in middle
        return [base, base * 0.6, base * 0.15, 0.0, base * 0.15, base * 0.6, base]

    def _compute_opacity_unit_distance(self):
        # Compute appropriate opacity unit distance based on grid spacing
        # Smaller values = more opaque/dense appearance
        if self._volume_grid is None:
            return 1.0
        spacing = self._volume_grid.spacing
        avg_spacing = sum(spacing) / 3.0
        # Use small unit distance for more saturated colors
        return avg_spacing * 0.5 / max(0.1, self.opacity_level)

    def _add_points_actor(self, xyz):
        if self._points_actor is not None:
            self.plotter.remove_actor(self._points_actor)
            self._points_actor = None

        if not self.show_points or xyz.shape[0] == 0:
            return

        point_cloud = pv.PolyData(xyz)
        self._points_actor = self.plotter.add_mesh(
            point_cloud,
            color="yellow",
            point_size=5,
            render_points_as_spheres=True,
            opacity=0.6,
        )

    def start_playback(self):
        if not self.is_playing and self.T > 1:
            self.timer.start()
            self.is_playing = True

    def stop_playback(self):
        if self.is_playing:
            self.timer.stop()
            self.is_playing = False

    def _on_frame_slider(self, value):
        self.set_frame(int(value))

    def _on_cmap_changed(self, cmap):
        self.cmap = cmap
        self._rebuild_volume(force=True)

    def _on_resolution_changed(self, value):
        new_res = int(value)
        if new_res != self.grid_resolution:
            self.grid_resolution = new_res
            self.label_resolution.setText(f"{new_res}³")
            self._rebuild_volume(force=True)

    def _on_opacity_changed(self, value):
        self.opacity_level = value / 100.0
        self.label_opacity.setText(f"{self.opacity_level:.0%}")
        self._rebuild_volume(force=True)

    def _on_show_points_changed(self, state):
        self.show_points = state == QtCore.Qt.Checked
        self._add_points_actor(self._xyz_points)
        self.plotter.render()

    def set_frame(self, frame_index):
        frame_index = int(frame_index) % max(1, self.T)
        if frame_index == self.current_frame:
            return
        self.current_frame = frame_index

        xyz, val = self.provider.get(frame_index)
        self._xyz_points = xyz
        self._volume_grid = _interpolate_to_volume(xyz, val, self.grid_resolution)
        self._update_volume_data()

        if self.slider_frame.value() != frame_index:
            self.slider_frame.blockSignals(True)
            self.slider_frame.setValue(frame_index)
            self.slider_frame.blockSignals(False)
        self._update_status_labels()

    def _rebuild_volume(self, force=False):
        xyz, val = self.provider.get(self.current_frame)
        self._volume_grid = _interpolate_to_volume(xyz, val, self.grid_resolution)
        camera_pos = self.plotter.camera_position
        self._add_volume_actor(force_rebuild=force)
        self.plotter.camera_position = camera_pos
        self.plotter.render()

    def _update_volume_data(self):
        # Update scalars in place when possible to avoid flickering
        camera_pos = self.plotter.camera_position
        self._add_volume_actor(force_rebuild=False)
        if self.show_points:
            self._add_points_actor(self._xyz_points)
        self.plotter.camera_position = camera_pos
        self.plotter.render()

    def _advance_frame(self):
        if self.T <= 1:
            return
        next_frame = (self.current_frame + 1) % self.T
        self.set_frame(next_frame)

    def _update_status_labels(self):
        total = max(0, int(self.T))
        if total <= 0:
            self.label_frame.setText("Frame: 0/0")
        else:
            self.label_frame.setText(f"Frame: {self.current_frame + 1}/{total}")

    def closeEvent(self, event):
        self.stop_playback()
        super().closeEvent(event)


# ---------------- Gaussian Splat Viewer ----------------


class SplatViewer(QtWidgets.QMainWindow):
    """Gaussian splatting visualization - renders each point as a transparent Gaussian blob."""

    def __init__(self, provider: ProviderBase, fps=24, default_params=None):
        super().__init__()
        self.setWindowTitle("PyVista Splat Viewer - Gaussian Splatting")
        self.provider = provider
        self.T = len(provider)
        self.params = default_params or VizParams()

        self.current_frame = 0
        self.is_playing = False
        # Default to 'coolwarm' for splat mode as it has darker middle (less white)
        default_cmap = "coolwarm"
        self.cmap = self.params.cmap if self.params.cmap in COLORSCALES else default_cmap
        if self.cmap == "RdBu":  # RdBu has white middle, switch to coolwarm
            self.cmap = default_cmap
        self.splat_radius = max(0.1, self.params.splat_radius)
        self.splat_opacity = 0.6  # base opacity for splats
        self.downsample_step = max(1, self.params.downsample or 1)

        amp_min, amp_max = provider.amplitude_range()
        self.amp_min = float(amp_min)
        self.amp_max = float(amp_max)
        self.amp_abs_max = max(abs(self.amp_min), abs(self.amp_max))
        if self.amp_abs_max <= 0:
            self.amp_abs_max = 1.0
        self._clim = (-self.amp_abs_max, self.amp_abs_max)

        # Get initial data to compute base radius
        xyz0, val0 = provider.get(0)
        self._base_extent = _safe_diag_extent(xyz0)
        self._auto_radius = self._base_extent / 8.0  # larger splats for more overlap/blending

        # Actors
        self._splat_actor = None
        self._mesh = None

        self._build_ui(fps)
        self._initialize_scene(xyz0, val0)
        self._update_status_labels()

    def _build_ui(self, fps):
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)

        # Playback controls row
        top_row = QtWidgets.QHBoxLayout()
        self.btn_play = QtWidgets.QPushButton("⏵ Play")
        self.btn_pause = QtWidgets.QPushButton("⏸ Pause")
        self.slider_frame = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_frame.setRange(0, max(0, self.T - 1))
        self.slider_frame.setSingleStep(1)
        self.label_frame = QtWidgets.QLabel()

        top_row.addWidget(self.btn_play)
        top_row.addWidget(self.btn_pause)
        top_row.addWidget(self.slider_frame)
        top_row.addWidget(self.label_frame)
        main_layout.addLayout(top_row)

        # Settings row
        settings_row = QtWidgets.QHBoxLayout()

        # Colormap
        cmap_layout = QtWidgets.QVBoxLayout()
        cmap_layout.addWidget(QtWidgets.QLabel("Colormap"))
        self.combo_cmap = QtWidgets.QComboBox()
        self.combo_cmap.addItems(COLORSCALES)
        self.combo_cmap.setCurrentText(self.cmap)
        cmap_layout.addWidget(self.combo_cmap)
        settings_row.addLayout(cmap_layout)

        # Splat radius slider
        radius_layout = QtWidgets.QVBoxLayout()
        radius_layout.addWidget(QtWidgets.QLabel("Splat Size"))
        self.slider_radius = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_radius.setRange(10, 300)  # 0.1x to 3.0x
        self.slider_radius.setValue(int(self.splat_radius * 100))
        self.label_radius = QtWidgets.QLabel(f"{self.splat_radius:.1f}x")
        radius_layout.addWidget(self.slider_radius)
        radius_layout.addWidget(self.label_radius)
        settings_row.addLayout(radius_layout)

        # Opacity slider
        opacity_layout = QtWidgets.QVBoxLayout()
        opacity_layout.addWidget(QtWidgets.QLabel("Splat Opacity"))
        self.slider_opacity = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_opacity.setRange(10, 100)
        self.slider_opacity.setValue(int(self.splat_opacity * 100))
        self.label_opacity = QtWidgets.QLabel(f"{self.splat_opacity:.0%}")
        opacity_layout.addWidget(self.slider_opacity)
        opacity_layout.addWidget(self.label_opacity)
        settings_row.addLayout(opacity_layout)

        # Downsample slider
        down_layout = QtWidgets.QVBoxLayout()
        down_layout.addWidget(QtWidgets.QLabel("Downsample"))
        self.slider_downsample = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_downsample.setRange(1, 20)
        self.slider_downsample.setValue(self.downsample_step)
        self.label_downsample = QtWidgets.QLabel(
            f"every {self.downsample_step}" if self.downsample_step > 1 else "off"
        )
        down_layout.addWidget(self.slider_downsample)
        down_layout.addWidget(self.label_downsample)
        settings_row.addLayout(down_layout)

        settings_row.addStretch(1)
        main_layout.addLayout(settings_row)

        # PyVista view
        self.plotter = QtInteractor(self)
        self.plotter.set_background("black")
        main_layout.addWidget(self.plotter)

        # Timer for playback
        self.timer = QtCore.QTimer(self)
        interval_ms = int(1000 / max(1, fps))
        self.timer.setInterval(interval_ms)
        self.timer.timeout.connect(self._advance_frame)

        # Signal wiring
        self.btn_play.clicked.connect(self.start_playback)
        self.btn_pause.clicked.connect(self.stop_playback)
        self.slider_frame.valueChanged.connect(self._on_frame_slider)
        self.combo_cmap.currentTextChanged.connect(self._on_cmap_changed)
        self.slider_radius.valueChanged.connect(self._on_radius_changed)
        self.slider_opacity.valueChanged.connect(self._on_opacity_changed)
        self.slider_downsample.valueChanged.connect(self._on_downsample_changed)

    def _initialize_scene(self, xyz, val):
        # Enable depth peeling for better transparency blending
        self.plotter.enable_depth_peeling(number_of_peels=8, occlusion_ratio=0.0)
        xyz, val = _downsample(xyz, val, self.downsample_step)
        self._create_splat_mesh(xyz, val)
        self.plotter.show_axes()
        self.plotter.render()

    def _create_splat_mesh(self, xyz, val):
        """Create Gaussian splat visualization using transparent spheres."""
        if self._splat_actor is not None:
            self.plotter.remove_actor(self._splat_actor)
            self._splat_actor = None

        xyz = np.asarray(xyz, dtype=np.float32)
        val = np.asarray(val, dtype=np.float32).reshape(-1)

        if xyz.shape[0] == 0:
            return

        # Create point cloud with scalar values
        points = pv.PolyData(xyz)
        points.point_data["val"] = val
        points.point_data.active_scalars_name = "val"

        # Compute sphere radius based on data extent and user setting
        sphere_radius = self._auto_radius * self.splat_radius

        # Create a smoother sphere for better blending appearance
        sphere = pv.Sphere(radius=sphere_radius, theta_resolution=20, phi_resolution=20)

        # Glyph at each point
        glyphs = points.glyph(geom=sphere, orient=False, scale=False)

        # The glyphs need the scalar values propagated
        n_sphere_pts = sphere.n_points
        n_data_pts = xyz.shape[0]
        # Each data point generates n_sphere_pts vertices in the glyph output
        expanded_vals = np.repeat(val, n_sphere_pts)
        glyphs.point_data["val"] = expanded_vals

        # Use full amplitude range
        clim = self._clim

        # Use 'viridis' or a sequential colormap that doesn't have white
        # Or use the selected cmap - user can switch to coolwarm/seismic which have darker middles
        cmap_name = _normalize_cmap_name(self.cmap)

        # Add with transparency for blending
        self._splat_actor = self.plotter.add_mesh(
            glyphs,
            scalars="val",
            cmap=cmap_name,
            clim=clim,
            opacity=self.splat_opacity * 0.5,
            smooth_shading=True,
            show_scalar_bar=True,
            lighting=False,
        )

        self._mesh = points
        self._n_sphere_pts = n_sphere_pts

    def _update_splat_scalars(self, val):
        """Try to update scalars in place for smooth animation."""
        if self._splat_actor is None or self._mesh is None:
            return False

        try:
            val = np.asarray(val, dtype=np.float32).reshape(-1)
            n_sphere_pts = getattr(self, '_n_sphere_pts', None)
            if n_sphere_pts is None:
                return False

            mapper = self._splat_actor.GetMapper()
            if mapper is None:
                return False

            input_data = mapper.GetInput()
            if input_data is None:
                return False

            n_pts = input_data.GetNumberOfPoints()
            expected_pts = len(val) * n_sphere_pts
            if n_pts != expected_pts:
                return False

            expanded_vals = np.repeat(val, n_sphere_pts).astype(np.float32)

            from vtkmodules.util.numpy_support import numpy_to_vtk
            vtk_arr = numpy_to_vtk(expanded_vals, deep=True)
            vtk_arr.SetName("val")
            input_data.GetPointData().SetScalars(vtk_arr)
            input_data.Modified()
            mapper.Modified()
            return True

        except Exception:
            return False

    def start_playback(self):
        if not self.is_playing and self.T > 1:
            self.timer.start()
            self.is_playing = True

    def stop_playback(self):
        if self.is_playing:
            self.timer.stop()
            self.is_playing = False

    def _on_frame_slider(self, value):
        self.set_frame(int(value))

    def _on_cmap_changed(self, cmap):
        self.cmap = cmap
        self._rebuild_splats()

    def _on_radius_changed(self, value):
        self.splat_radius = value / 100.0
        self.label_radius.setText(f"{self.splat_radius:.1f}x")
        self._rebuild_splats()

    def _on_opacity_changed(self, value):
        self.splat_opacity = value / 100.0
        self.label_opacity.setText(f"{self.splat_opacity:.0%}")
        self._rebuild_splats()

    def _on_downsample_changed(self, value):
        self.downsample_step = max(1, int(value))
        self.label_downsample.setText(
            f"every {self.downsample_step}" if self.downsample_step > 1 else "off"
        )
        self._rebuild_splats()

    def set_frame(self, frame_index):
        frame_index = int(frame_index) % max(1, self.T)
        if frame_index == self.current_frame:
            return
        self.current_frame = frame_index

        xyz, val = self.provider.get(frame_index)
        xyz, val = _downsample(xyz, val, self.downsample_step)

        # Try in-place scalar update first for smooth animation
        camera_pos = self.plotter.camera_position
        if not self._update_splat_scalars(val):
            self._create_splat_mesh(xyz, val)
        self.plotter.camera_position = camera_pos
        self.plotter.render()

        if self.slider_frame.value() != frame_index:
            self.slider_frame.blockSignals(True)
            self.slider_frame.setValue(frame_index)
            self.slider_frame.blockSignals(False)
        self._update_status_labels()

    def _rebuild_splats(self):
        xyz, val = self.provider.get(self.current_frame)
        xyz, val = _downsample(xyz, val, self.downsample_step)
        camera_pos = self.plotter.camera_position
        self._create_splat_mesh(xyz, val)
        self.plotter.camera_position = camera_pos
        self.plotter.render()

    def _advance_frame(self):
        if self.T <= 1:
            return
        next_frame = (self.current_frame + 1) % self.T
        self.set_frame(next_frame)

    def _update_status_labels(self):
        total = max(0, int(self.T))
        if total <= 0:
            self.label_frame.setText("Frame: 0/0")
        else:
            self.label_frame.setText(f"Frame: {self.current_frame + 1}/{total}")

    def closeEvent(self, event):
        self.stop_playback()
        super().closeEvent(event)


# ---------------- CLI helpers ----------------


def infer_frames(path, pattern):
    t = 0
    while os.path.exists(os.path.join(path, pattern.format(t))):
        t += 1
    return t


def infer_point_files(path, pattern):
    return len(list_point_files(path, pattern))


def list_point_files(path, pattern):
    placeholder = None
    if "{" in pattern and "}" in pattern:
        start = pattern.index("{")
        end = pattern.index("}", start)
        placeholder = pattern[start : end + 1]
        glob_pattern = pattern[:start] + "*" + pattern[end + 1 :]
        regex = re.compile(
            "^"
            + re.escape(pattern[:start])
            + r"(\d+)"
            + re.escape(pattern[end + 1 :])
            + "$"
        )
    else:
        glob_pattern = pattern
        regex = re.compile("^" + re.escape(pattern) + "$")

    base = Path(path)
    files = []
    for candidate in sorted(base.glob(glob_pattern)):
        name = candidate.name
        match = regex.match(name)
        if not match:
            continue
        idx = int(match.group(1)) if match.groups() else 0
        files.append((idx, candidate))

    files.sort(key=lambda item: item[0])
    return [str(candidate) for _, candidate in files]


def main(argv=None):
    argv = argv or sys.argv[1:]
    ap = argparse.ArgumentParser(description="PyVista animated point-cloud viewer.")
    ap.add_argument("--demo", action="store_true", help="Use synthetic expanding wavefront")
    ap.add_argument(
        "--path",
        type=str,
        default=".",
        help="Folder with NPZ files (per-frame or per-point depending on --mode)",
    )
    ap.add_argument(
        "--pattern",
        type=str,
        help="Filename pattern with {:d} placeholder. Defaults to frame_{:04d}.npz (frame mode) or point_{:05d}.npz (point mode).",
    )
    ap.add_argument("--frames", type=int, help="Frame count for per-frame mode (if not inferable)")
    ap.add_argument(
        "--points",
        type=int,
        help="Point count for per-point mode (if not inferable).",
    )
    ap.add_argument(
        "--mode",
        type=str,
        choices=["frame", "point"],
        default="frame",
        help="Interpretation of files under --path/--pattern (per-frame NPZs or per-point NPZs).",
    )
    ap.add_argument("--series", type=str, help="Single NPZ with xyz/val time series")
    ap.add_argument("--fps", type=int, default=24, help="Playback FPS")
    ap.add_argument(
        "--size-scale",
        type=float,
        default=1.0,
        help="Marker size multiplier (1.0 = default radius; increase for larger points).",
    )
    ap.add_argument("--downsample", type=int, default=0, help="Keep every Nth point (0=no downsample)")
    ap.add_argument(
        "--amp-threshold",
        type=float,
        default=0.0,
        help="Initial |value| cutoff; points below this magnitude are hidden.",
    )
    ap.add_argument("--host", type=str, default="127.0.0.1", help="Unused (compatibility placeholder)")
    ap.add_argument("--port", type=int, default=8050, help="Unused (compatibility placeholder)")
    ap.add_argument(
        "--offscreen",
        action="store_true",
        help="Attempt to run with Qt offscreen platform (no interactive window).",
    )
    ap.add_argument(
        "--viz-mode",
        type=str,
        choices=VIZ_MODES,
        default="dots",
        help="Visualization mode: 'dots' for discrete point markers, 'cloud' for volumetric field, 'splat' for Gaussian splatting.",
    )
    ap.add_argument(
        "--grid-resolution",
        type=int,
        default=15,
        help="Voxel grid resolution for cloud mode (default 15, range 15-80).",
    )
    ap.add_argument(
        "--splat-radius",
        type=float,
        default=1.0,
        help="Splat size multiplier for splat mode (default 1.0).",
    )
    args = ap.parse_args(argv)

    if args.offscreen and not OFFSCREEN_REQUESTED:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    if HEADLESS_ENV and not (args.offscreen or os.environ.get("QT_QPA_PLATFORM") == "offscreen"):
        raise SystemExit(
            "No display server detected (DISPLAY/WAYLAND variables are empty). "
            "Run on a machine with a GUI or re-run with --offscreen to try headless mode."
        )

    if args.demo:
        provider = make_demo_provider()
    elif args.series:
        provider = NPZSeriesProvider(args.series)
    else:
        pattern = args.pattern or ("point_{:05d}.npz" if args.mode == "point" else "frame_{:04d}.npz")
        if args.mode == "point":
            count = args.points or infer_point_files(args.path, pattern)
            if count == 0:
                raise SystemExit("No point files found. Check --path/--pattern or use --demo.")
            provider = PointFilesProvider(args.path, pattern, count)
        else:
            frames = args.frames or infer_frames(args.path, pattern)
            if frames == 0:
                raise SystemExit("No frames found. Check --path/--pattern or use --demo.")
            provider = NPZFramesProvider(args.path, pattern, frames)

    params = VizParams(
        cmap=COLORSCALES[0],
        size_scale=max(0.1, args.size_scale),
        downsample=max(0, args.downsample),
        amp_threshold=max(0.0, args.amp_threshold),
        viz_mode=args.viz_mode,
        grid_resolution=max(15, min(80, args.grid_resolution)),
        splat_radius=max(0.1, args.splat_radius),
    )

    if args.offscreen:
        print(
            "Running in Qt offscreen mode; no interactive window will be displayed.",
            file=sys.stderr,
        )

    try:
        qt_app = QtWidgets.QApplication(sys.argv)
    except Exception as exc:
        raise SystemExit(
            f"Failed to initialise the Qt application. Details: {exc}\n"
            "If you are running in a headless environment, pass --offscreen "
            "or set QT_QPA_PLATFORM=offscreen."
        )

    if args.viz_mode == "cloud":
        viewer = CloudViewer(provider, fps=args.fps, default_params=params)
    elif args.viz_mode == "splat":
        viewer = SplatViewer(provider, fps=args.fps, default_params=params)
    else:
        viewer = PointCloudViewer(provider, fps=args.fps, default_params=params)
    viewer.resize(1200, 800)
    viewer.show()
    sys.exit(qt_app.exec_())


if __name__ == "__main__":
    main()
