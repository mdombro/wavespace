#!/usr/bin/env python3
"""
PyVista-based animated point-cloud and marching cubes viewer.

- Visualizes time-varying 3D point clouds where per-point scalar values map to color and size
- Marching cubes mode: extracts isosurfaces from gridded data with adjustable threshold
- Playback controls: Play/Pause, time slider scrubbing, FPS-configurable timer
- Display controls: colormap selector, marker size scaling slider, configurable downsampling
- Data sources:
    * Directory of per-frame NPZ files (xyz[N,3], val[N])
    * Single NPZ series (xyz[T,N,3] or xyz[N,3], val[T,N])
    * Synthetic demo (expanding wavefront)

Rendering modes:
    * point: Traditional point cloud visualization with spherical glyphs
    * marching_cubes: Isosurface extraction using marching cubes algorithm
      - Automatically infers grid structure from XYZ coordinates
      - Normalizes scattered data to regular grid via interpolation
      - Adjustable threshold slider for isosurface level

Dependencies:
    pip install numpy pyvista pyvistaqt PyQt5 scipy
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


def _normalize_val(val: np.ndarray) -> np.ndarray:
    """Normalize values to [0, 1] range using min-max scaling."""
    val = np.asarray(val, dtype=np.float32)
    vmin, vmax = val.min(), val.max()
    if vmax - vmin < 1e-12:
        return np.zeros_like(val)
    return (val - vmin) / (vmax - vmin)


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


class NPZFramesProvider(ProviderBase):
    def __init__(self, path, pattern, frames):
        self.path = path
        self.pattern = pattern
        self.T = int(frames)
        if self.T <= 0:
            raise ValueError("Frame count must be positive.")
        xyz0, _ = self._load(0)
        self._N = xyz0.shape[0]
        self._first_xyz = xyz0.copy()
        self._static = True

    def __len__(self):
        return self.T

    def _filename(self, t):
        return os.path.join(self.path, self.pattern.format(t))

    def _load(self, t):
        fn = self._filename(t)
        if not os.path.exists(fn):
            raise FileNotFoundError(fn)
        data = np.load(fn, allow_pickle=False)
        xyz = data["xyz"].astype(np.float32)
        val = data["val"].astype(np.float32)
        val = _normalize_val(val)
        return xyz, val

    def get(self, t):
        xyz, val = self._load(int(t))
        if self._static and not np.array_equal(self._first_xyz, xyz):
            self._static = False
        return xyz, val

    def static_xyz(self) -> bool:
        return bool(self._static)


class NPZSeriesProvider(ProviderBase):
    def __init__(self, series_path):
        data = np.load(series_path, allow_pickle=False)
        self._val = _normalize_val(data["val"].astype(np.float32))
        self.T, self.N = self._val.shape
        self._N = self.N
        self._xyz = data["xyz"]
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
            series = _normalize_val(np.asarray(data["val"], dtype=np.float32).reshape(-1))
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

    class Demo(ProviderBase):
        def __len__(self):
            return T

        def get(self, t):
            tt = int(t)
            tau = tt / max(1, T - 1)
            c = 0.8 * radius
            r0 = c * tau
            rr = np.linalg.norm(xyz, axis=1)
            sigma = 0.16 * radius
            val = np.exp(-((rr - r0) ** 2) / (2 * sigma * sigma)).astype(np.float32)
            m = val.max()
            if m > 0:
                val /= m
            return xyz, val

        def static_xyz(self):
            return True

        def max_points(self):
            return xyz.shape[0]

    return Demo()


# ---------------- Visualization helpers ----------------


COLORSCALES = ["Viridis", "Plasma", "Cividis", "Magma", "Turbo", "Inferno", "Ice", "Aggrnyl"]


@dataclass
class VizParams:
    cmap: str = "Viridis"
    size_scale: float = 1.0
    downsample: int = 0  # 0 = no downsample
    threshold: float = 0.5  # for marching_cubes mode


def _normalize_cmap_name(name: str) -> str:
    """Translate UI colormap labels into the lowercase keys expected by Matplotlib."""
    return (name or "").strip().lower()


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


def _compute_sizes(val, size_scale, scale_factor):
    base = 2.0 + 8.0 * val  # baseline range roughly matching previous defaults
    return scale_factor * size_scale * base


def _infer_grid(xyz):
    """
    Infer regular grid structure from scattered XYZ points.
    Returns (origin, spacing, dimensions) where:
      - origin: [x0, y0, z0] coordinates of grid origin
      - spacing: [dx, dy, dz] spacing between grid points
      - dimensions: [nx, ny, nz] number of points in each direction
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    if xyz.shape[0] == 0:
        raise ValueError("Cannot infer grid from empty point set")

    # Find unique coordinates along each axis
    x_unique = np.unique(np.round(xyz[:, 0], decimals=6))
    y_unique = np.unique(np.round(xyz[:, 1], decimals=6))
    z_unique = np.unique(np.round(xyz[:, 2], decimals=6))

    # Calculate spacing as the minimum non-zero difference
    def calc_spacing(vals):
        if len(vals) < 2:
            return 1.0
        diffs = np.diff(vals)
        diffs = diffs[diffs > 1e-9]  # filter out numerical noise
        return float(np.min(diffs)) if len(diffs) > 0 else 1.0

    dx = calc_spacing(x_unique)
    dy = calc_spacing(y_unique)
    dz = calc_spacing(z_unique)

    # Grid dimensions
    nx = len(x_unique)
    ny = len(y_unique)
    nz = len(z_unique)

    # Origin is the minimum corner
    origin = np.array([x_unique[0], y_unique[0], z_unique[0]], dtype=np.float32)
    spacing = np.array([dx, dy, dz], dtype=np.float32)
    dimensions = np.array([nx, ny, nz], dtype=np.int32)

    return origin, spacing, dimensions


def _normalize_to_grid(xyz, val, origin, spacing, dimensions):
    """
    Normalize scattered XYZ points with values to a regular 3D grid using interpolation.

    Args:
        xyz: [N, 3] array of point coordinates
        val: [N] array of values at each point
        origin: [3] grid origin coordinates
        spacing: [3] grid spacing in each dimension
        dimensions: [3] grid dimensions (nx, ny, nz)

    Returns:
        grid_data: [nx, ny, nz] array of interpolated values
    """
    from scipy.interpolate import griddata

    xyz = np.asarray(xyz, dtype=np.float64)
    val = np.asarray(val, dtype=np.float64)

    nx, ny, nz = dimensions

    # Create regular grid
    x = origin[0] + np.arange(nx) * spacing[0]
    y = origin[1] + np.arange(ny) * spacing[1]
    z = origin[2] + np.arange(nz) * spacing[2]

    xg, yg, zg = np.meshgrid(x, y, z, indexing='ij')
    grid_points = np.stack([xg.ravel(), yg.ravel(), zg.ravel()], axis=-1)

    # Interpolate scattered data to grid using linear interpolation
    # Fill missing values with 0
    grid_vals = griddata(xyz, val, grid_points, method='linear', fill_value=0.0)
    grid_data = grid_vals.reshape(nx, ny, nz).astype(np.float32)

    return grid_data


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

        xyz0, val0 = provider.get(0)
        xyz0, val0 = _downsample(xyz0, val0, self.params.downsample)
        self._base_extent = _safe_diag_extent(xyz0)
        self._size_scale = self._base_extent / 150.0
        self._glyph_geom = pv.Sphere(radius=1.0, theta_resolution=12, phi_resolution=12)

        self._mesh = None
        self._glyph = None
        self._actor = None
        self.label_frame = None
        self.label_downsample = None

        self._build_ui(fps)
        self._initialize_scene(xyz0, val0)

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

    def _format_size_scale_label(self, value):
        return f"{value:.2f}×"
        self._update_status_labels()

    def _make_slider(self, minimum, maximum, value):
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(int(minimum), int(maximum))
        slider.setSingleStep(1)
        slider.setPageStep(1)
        slider.setValue(int(round(value)))
        return slider

    def _wrap_slider(self, label, slider):
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(QtWidgets.QLabel(label))
        layout.addWidget(slider)
        return layout

    # ----- Plot initialization -----

    def _initialize_scene(self, xyz, val):
        mesh = self._build_polydata(xyz, val)
        glyph = mesh.glyph(scale="scale", geom=self._glyph_geom, orient=False)
        self._apply_scalars_to_glyph(glyph, mesh)
        self._mesh = mesh
        self._glyph = glyph
        self._actor = self.plotter.add_mesh(
            glyph,
            scalars="val",
            cmap=_normalize_cmap_name(self.cmap),
            clim=[0.0, 1.0],
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
                vtk_mapper.SetScalarRange(0.0, 1.0)
        self.plotter.show_axes()
        self.plotter.render()

    def _build_polydata(self, xyz, val):
        xyz = np.asarray(xyz, dtype=np.float32)
        val = np.clip(np.asarray(val, dtype=np.float32), 0.0, 1.0)
        mesh = pv.PolyData(xyz)
        mesh.point_data["val"] = val
        mesh.point_data.active_scalars_name = "val"
        scale = _compute_sizes(val, self.size_scale, self._size_scale)
        mesh.point_data["scale"] = scale
        return mesh

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

    # ----- Scene updates -----

    def set_frame(self, frame_index, rebuild_geometry=False):
        frame_index = int(frame_index) % max(1, self.T)
        if frame_index == self.current_frame and not rebuild_geometry:
            return
        self.current_frame = frame_index
        xyz, val = self.provider.get(frame_index)
        xyz, val = _downsample(xyz, val, self.downsample_step)

        if self.static_xyz and not rebuild_geometry and self._mesh is not None:
            self._mesh.point_data["val"] = np.clip(val, 0.0, 1.0)
            scale = _compute_sizes(val, self.size_scale, self._size_scale)
            self._mesh.point_data["scale"] = scale
            self._update_glyph(self._mesh, reuse_points=True)
        else:
            self._mesh = self._build_polydata(xyz, val)
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
        lut.scalar_range = (0.0, 1.0)
        vtk_mapper = self._get_actor_mapper()
        if vtk_mapper is None:
            raise RuntimeError("Could not access mapper on actor for colormap update.")
        self._ensure_mapper_scalar_settings(vtk_mapper)
        if hasattr(vtk_mapper, "SetLookupTable"):
            vtk_mapper.SetLookupTable(lut)
        if hasattr(vtk_mapper, "SetScalarRange"):
            vtk_mapper.SetScalarRange(0.0, 1.0)
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
        if reuse_points and self._glyph is not None:
            # Regenerate glyph with updated scalars/scale but same points
            glyph = mesh.glyph(scale="scale", geom=self._glyph_geom, orient=False)
        else:
            glyph = mesh.glyph(scale="scale", geom=self._glyph_geom, orient=False)

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
            vtk_mapper.SetScalarRange(0.0, 1.0)
        if hasattr(vtk_mapper, "Modified"):
            vtk_mapper.Modified()
        self.plotter.camera_position = camera_pos
        self.plotter.render()

    def closeEvent(self, event):
        self.stop_playback()
        super().closeEvent(event)


class MarchingCubesViewer(QtWidgets.QMainWindow):
    """
    Marching cubes isosurface viewer with adjustable threshold.
    Infers grid structure from XYZ points, normalizes each frame to the grid,
    and extracts isosurfaces using marching cubes algorithm.
    """
    def __init__(self, provider: ProviderBase, fps=24, default_params=None):
        super().__init__()
        self.setWindowTitle("PyVista Marching Cubes Viewer")
        self.provider = provider
        self.T = len(provider)
        self.params = default_params or VizParams()

        self.current_frame = 0
        self.is_playing = False
        self.threshold = float(self.params.threshold) if hasattr(self.params, 'threshold') else 0.5
        self.threshold = max(0.0, min(1.0, self.threshold))  # Clamp to [0, 1]
        self.cmap = self.params.cmap if self.params.cmap in COLORSCALES else COLORSCALES[0]

        # Infer grid structure from first frame
        xyz0, val0 = provider.get(0)
        self.origin, self.spacing, self.dimensions = _infer_grid(xyz0)
        print(f"[marching-cubes] Inferred grid: origin={self.origin}, spacing={self.spacing}, dimensions={self.dimensions}")

        self._mesh = None
        self._actor = None
        self.label_frame = None
        self.label_threshold = None

        self._build_ui(fps)
        self._initialize_scene()

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

        # Threshold slider
        self.slider_threshold = self._make_slider(0, 100, int(self.threshold * 100))
        threshold_layout = QtWidgets.QVBoxLayout()
        threshold_layout.addWidget(QtWidgets.QLabel("Isosurface Threshold"))
        threshold_layout.addWidget(self.slider_threshold)
        self.label_threshold = QtWidgets.QLabel(self._format_threshold_label(self.threshold))
        threshold_layout.addWidget(self.label_threshold)
        settings_row.addLayout(threshold_layout)

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
        self.slider_threshold.valueChanged.connect(self._on_threshold_changed)

    def _make_slider(self, minimum, maximum, value):
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(int(minimum), int(maximum))
        slider.setSingleStep(1)
        slider.setPageStep(1)
        slider.setValue(int(round(value)))
        return slider

    def _format_threshold_label(self, value):
        return f"{value:.3f}"

    def _initialize_scene(self):
        """Initialize the scene with the first frame."""
        self.set_frame(0)
        self.plotter.show_axes()
        self.plotter.reset_camera()

    def _create_grid_mesh(self, frame_index):
        """Create a PyVista ImageData mesh from the current frame."""
        xyz, val = self.provider.get(frame_index)
        grid_data = _normalize_to_grid(xyz, val, self.origin, self.spacing, self.dimensions)

        # Create PyVista ImageData (uniform grid)
        mesh = pv.ImageData()
        mesh.dimensions = self.dimensions
        mesh.origin = self.origin
        mesh.spacing = self.spacing
        mesh.point_data["values"] = grid_data.ravel(order='F')  # Fortran order for VTK

        return mesh

    def _update_isosurface(self, mesh):
        """Extract and display isosurface at current threshold."""
        # Remove previous mesh
        if self._actor is not None:
            self.plotter.remove_actor(self._actor)
            self._actor = None

        # Extract isosurface using marching cubes
        try:
            contour = mesh.contour([self.threshold], scalars="values")
            if contour.n_points > 0:
                self._actor = self.plotter.add_mesh(
                    contour,
                    scalars="values",
                    cmap=_normalize_cmap_name(self.cmap),
                    show_scalar_bar=True,
                    lighting=True,
                    opacity=0.8,
                )
            else:
                # No surface at this threshold, show empty scene
                pass
        except Exception as e:
            print(f"[marching-cubes] Warning: Failed to create contour at threshold {self.threshold}: {e}")

        self.plotter.render()

    def set_frame(self, frame_index):
        """Update the view to show the specified frame."""
        frame_index = int(frame_index) % max(1, self.T)
        if frame_index == self.current_frame and self._mesh is not None:
            # Same frame, just update the isosurface with current threshold
            self._update_isosurface(self._mesh)
            return

        self.current_frame = frame_index
        self._mesh = self._create_grid_mesh(frame_index)
        self._update_isosurface(self._mesh)

        if self.slider_frame.value() != frame_index:
            self.slider_frame.blockSignals(True)
            self.slider_frame.setValue(frame_index)
            self.slider_frame.blockSignals(False)
        self._update_status_labels()

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
        # Recreate the isosurface with new colormap
        if self._mesh is not None:
            self._update_isosurface(self._mesh)

    def _on_threshold_changed(self, value):
        self.threshold = float(value) / 100.0
        if self.label_threshold:
            self.label_threshold.setText(self._format_threshold_label(self.threshold))
        # Update isosurface with new threshold
        if self._mesh is not None:
            self._update_isosurface(self._mesh)

    def _advance_frame(self):
        if self.T <= 1:
            return
        next_frame = (self.current_frame + 1) % self.T
        self.set_frame(next_frame)

    def _frame_label_text(self):
        total = max(0, int(self.T))
        if total <= 0:
            return "Frame: 0/0"
        return f"Frame: {self.current_frame + 1}/{total}"

    def _update_status_labels(self):
        if self.label_frame is not None:
            self.label_frame.setText(self._frame_label_text())

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
    ap.add_argument(
        "--render-mode",
        type=str,
        choices=["point", "marching_cubes"],
        default="point",
        help="Rendering mode: 'point' for point cloud visualization, 'marching_cubes' for isosurface extraction.",
    )
    ap.add_argument(
        "--threshold",
        type=float,
        default=0.5,
        help="Initial isosurface threshold for marching_cubes mode (0.0-1.0).",
    )
    ap.add_argument("--fps", type=int, default=24, help="Playback FPS")
    ap.add_argument(
        "--size-scale",
        type=float,
        default=1.0,
        help="Marker size multiplier (1.0 = default radius; increase for larger points).",
    )
    ap.add_argument("--downsample", type=int, default=0, help="Keep every Nth point (0=no downsample)")
    ap.add_argument("--host", type=str, default="127.0.0.1", help="Unused (compatibility placeholder)")
    ap.add_argument("--port", type=int, default=8050, help="Unused (compatibility placeholder)")
    ap.add_argument(
        "--offscreen",
        action="store_true",
        help="Attempt to run with Qt offscreen platform (no interactive window).",
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
        cmap="Viridis",
        size_scale=max(0.1, args.size_scale),
        downsample=max(0, args.downsample),
        threshold=max(0.0, min(1.0, args.threshold)),
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

    # Select viewer based on render mode
    if args.render_mode == "marching_cubes":
        viewer = MarchingCubesViewer(provider, fps=args.fps, default_params=params)
    else:  # default to point mode
        viewer = PointCloudViewer(provider, fps=args.fps, default_params=params)

    viewer.resize(1200, 800)
    viewer.show()
    sys.exit(qt_app.exec_())


if __name__ == "__main__":
    main()
