#!/usr/bin/env python3
"""
PyVista-based animated point-cloud viewer.

- Visualizes time-varying 3D point clouds where per-point scalar values map to color and size
- Playback controls: Play/Pause, time slider scrubbing, FPS-configurable timer
- Display controls: colormap selector, marker size min/max sliders, configurable downsampling
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
from dataclasses import dataclass

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
        val = np.clip(data["val"].astype(np.float32), 0.0, 1.0)
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
        self._val = np.clip(data["val"].astype(np.float32), 0.0, 1.0)
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


def make_demo_provider(T=160, N=60000, radius=1.0):
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
    size_min: float = 2.0
    size_max: float = 10.0
    downsample: int = 0  # 0 = no downsample


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


def _compute_sizes(val, size_min, size_max, scale_factor):
    sizes = size_min + (size_max - size_min) * val
    return scale_factor * sizes


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
        self.size_min = float(self.params.size_min)
        self.size_max = float(self.params.size_max)
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

        top_row.addWidget(self.btn_play)
        top_row.addWidget(self.btn_pause)
        top_row.addWidget(self.slider_frame)
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

        # Size sliders
        self.slider_size_min = self._make_slider(1, 20, self.size_min)
        self.slider_size_max = self._make_slider(4, 40, self.size_max)
        settings_row.addLayout(self._wrap_slider("Size min (px)", self.slider_size_min))
        settings_row.addLayout(self._wrap_slider("Size max (px)", self.slider_size_max))

        # Downsample slider + note
        self.slider_downsample = self._make_slider(0, 20, self.params.downsample)
        down_layout = self._wrap_slider("Downsample (every Nth point)", self.slider_downsample)
        down_layout.addWidget(QtWidgets.QLabel("0 = no downsample"))
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
        self.slider_size_min.valueChanged.connect(self._on_size_min_changed)
        self.slider_size_max.valueChanged.connect(self._on_size_max_changed)
        self.slider_downsample.valueChanged.connect(self._on_downsample_changed)

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
        self._mesh = mesh
        self._glyph = glyph
        self._actor = self.plotter.add_mesh(
            glyph,
            scalars="val",
            cmap=self.cmap,
            clim=[0.0, 1.0],
            lighting=False,
            show_scalar_bar=True,
        )
        self.plotter.show_axes()
        self.plotter.render()

    def _build_polydata(self, xyz, val):
        xyz = np.asarray(xyz, dtype=np.float32)
        val = np.clip(np.asarray(val, dtype=np.float32), 0.0, 1.0)
        mesh = pv.PolyData(xyz)
        mesh.point_data["val"] = val
        mesh.point_data.set_active_scalars("val")
        scale = _compute_sizes(val, self.size_min, self.size_max, self._size_scale)
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

    def _on_size_min_changed(self, value):
        v = float(value)
        if v > self.size_max:
            self.size_max = v
            self.slider_size_max.blockSignals(True)
            self.slider_size_max.setValue(int(round(v)))
            self.slider_size_max.blockSignals(False)
        self.size_min = v
        self._refresh_current_mesh()

    def _on_size_max_changed(self, value):
        v = float(value)
        if v < self.size_min:
            self.size_min = v
            self.slider_size_min.blockSignals(True)
            self.slider_size_min.setValue(int(round(v)))
            self.slider_size_min.blockSignals(False)
        self.size_max = v
        self._refresh_current_mesh()

    def _on_downsample_changed(self, value):
        step = int(value)
        self.downsample_step = max(1, step if step != 0 else 1)
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
            scale = _compute_sizes(val, self.size_min, self.size_max, self._size_scale)
            self._mesh.point_data["scale"] = scale
            self._update_glyph(self._mesh, reuse_points=True)
        else:
            self._mesh = self._build_polydata(xyz, val)
            self._update_glyph(self._mesh, reuse_points=False)

        if self.slider_frame.value() != frame_index:
            self.slider_frame.blockSignals(True)
            self.slider_frame.setValue(frame_index)
            self.slider_frame.blockSignals(False)

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
        lut = pv.LookupTable(cmap=self.cmap)
        lut.scalar_range = (0.0, 1.0)
        vtk_mapper = self._actor.actor.GetMapper()
        vtk_mapper.SetLookupTable(lut)
        vtk_mapper.SetScalarRange(0.0, 1.0)
        self.plotter.render()

    def _update_glyph(self, mesh, reuse_points):
        camera_pos = self.plotter.camera_position
        if reuse_points and self._glyph is not None:
            # Regenerate glyph with updated scalars/scale but same points
            glyph = mesh.glyph(scale="scale", geom=self._glyph_geom, orient=False)
        else:
            glyph = mesh.glyph(scale="scale", geom=self._glyph_geom, orient=False)

        self._glyph = glyph
        vtk_mapper = self._actor.actor.GetMapper()
        vtk_mapper.SetInputData(glyph)
        vtk_mapper.SetScalarRange(0.0, 1.0)
        vtk_mapper.Modified()
        self.plotter.camera_position = camera_pos
        self.plotter.render()

    def closeEvent(self, event):
        self.stop_playback()
        super().closeEvent(event)


# ---------------- CLI helpers ----------------


def infer_frames(path, pattern):
    t = 0
    while os.path.exists(os.path.join(path, pattern.format(t))):
        t += 1
    return t


def main(argv=None):
    argv = argv or sys.argv[1:]
    ap = argparse.ArgumentParser(description="PyVista animated point-cloud viewer.")
    ap.add_argument("--demo", action="store_true", help="Use synthetic expanding wavefront")
    ap.add_argument("--path", type=str, default=".", help="Folder with per-frame NPZs")
    ap.add_argument("--pattern", type=str, default="frame_{:04d}.npz", help="Filename pattern with {:d}")
    ap.add_argument("--frames", type=int, help="Frame count (if not inferable)")
    ap.add_argument("--series", type=str, help="Single NPZ with xyz/val time series")
    ap.add_argument("--fps", type=int, default=24, help="Playback FPS")
    ap.add_argument("--size-min", type=float, default=2.0)
    ap.add_argument("--size-max", type=float, default=10.0)
    ap.add_argument("--downsample", type=int, default=0, help="Keep every Nth point (0=no downsample)")
    ap.add_argument("--host", type=str, default="127.0.0.1", help="Unused (compatibility placeholder)")
    ap.add_argument("--port", type=int, default=8050, help="Unused (compatibility placeholder)")
    args = ap.parse_args(argv)

    if args.demo:
        provider = make_demo_provider()
    elif args.series:
        provider = NPZSeriesProvider(args.series)
    else:
        frames = args.frames or infer_frames(args.path, args.pattern)
        if frames == 0:
            raise SystemExit("No frames found. Check --path/--pattern or use --demo.")
        provider = NPZFramesProvider(args.path, args.pattern, frames)

    params = VizParams(
        cmap="Viridis",
        size_min=args.size_min,
        size_max=args.size_max,
        downsample=max(0, args.downsample),
    )

    qt_app = QtWidgets.QApplication(sys.argv)
    viewer = PointCloudViewer(provider, fps=args.fps, default_params=params)
    viewer.resize(1200, 800)
    viewer.show()
    sys.exit(qt_app.exec_())


if __name__ == "__main__":
    main()
