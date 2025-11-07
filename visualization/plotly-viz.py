#!/usr/bin/env python3
# Data format consumed by this viewer:
#   - Directory of NPZ files, one per spatial point. Each file must contain:
#       * `xyz`: float32 coordinate (shape [3] or [1,3]).
#       * `val`: float32 time-series values (shape [T]).
#     File names follow --pattern (format placeholder for the point index, default "point_{:05d}.npz").
#     There are as many files as points. All `val` arrays share the same length T (number of frames).
#   - Optionally, --series may point to a single NPZ with:
#       * `xyz`: [N,3] static coordinates or [T,N,3] for moving points.
#       * `val`: [T,N] scalar values. Values are clipped to [0,1].

"""
wave_points_dash.py — Browser-based animated 3D point-cloud viewer (Plotly Dash).

- Color & size both map to per-point scalar value in [0,1]
- Play/Pause + time slider
- Colormap select, size-min/max sliders
- Optional downsampling for very large point clouds
- No VTK/OpenGL/NVIDIA dependencies (uses your browser’s WebGL)

Run:
  pip install dash plotly numpy
  python wave_points_dash.py --demo
  # or your data:
  # python wave_points_dash.py --path /data --pattern "point_{:05d}.npz" --points 60000
  # python wave_points_dash.py --series /data/series.npz
"""

import argparse, os
import numpy as np
from dataclasses import dataclass

import dash
from dash import Dash, dcc, html, Input, Output, State, Patch
import plotly.graph_objects as go

# ------------- Data providers -------------

class ProviderBase:
    def __len__(self): raise NotImplementedError
    def get(self, t):  # -> (xyz[N,3], val[N])
        raise NotImplementedError
    def static_xyz(self) -> bool: return False
    def max_points(self) -> int: return self._N if hasattr(self, "_N") else None

class PointFilesProvider(ProviderBase):
    def __init__(self, path, pattern, points=None):
        self.path = path
        self.pattern = pattern
        self._N = int(points) if points is not None else None
        self._T = None
        self._xyz = None
        self._val = None
        self._load_all()

    def __len__(self): return self._T

    def _filename(self, idx: int) -> str:
        return os.path.join(self.path, self.pattern.format(idx))

    def _load_all(self):
        if self._N is None:
            self._N = infer_points(self.path, self.pattern)
        if self._N <= 0:
            raise ValueError("No point files found. Check --path/--pattern.")

        xyz_list = []
        val_list = []

        for idx in range(self._N):
            fn = self._filename(idx)
            if not os.path.exists(fn):
                raise FileNotFoundError(f"Expected point file missing: {fn}")
            data = np.load(fn, allow_pickle=False)

            if "xyz" not in data or "val" not in data:
                raise KeyError(f"Point file {fn} must contain 'xyz' and 'val'")

            xyz = np.asarray(data["xyz"], dtype=np.float32)
            if xyz.ndim == 2 and xyz.shape[0] == 1:
                xyz = xyz[0]
            xyz = xyz.reshape(-1)
            if xyz.shape[0] != 3:
                raise ValueError(f"'xyz' in {fn} must have 3 elements, got shape {xyz.shape}")

            val = np.clip(np.asarray(data["val"], dtype=np.float32), 0.0, 1.0)
            val = val.reshape(-1)
            if self._T is None:
                self._T = val.shape[0]
            elif val.shape[0] != self._T:
                raise ValueError(f"All 'val' arrays must have same length; {fn} has {val.shape[0]}, expected {self._T}")

            xyz_list.append(xyz)
            val_list.append(val)

        self._xyz = np.vstack(xyz_list).astype(np.float32)
        self._val = np.stack(val_list, axis=1).astype(np.float32)  # shape [T, N]
        if self._T is None:
            raise ValueError("Could not determine time dimension from point files.")
        self._N = self._xyz.shape[0]

    def get(self, t):
        tt = int(t)
        if tt < 0 or tt >= self._T:
            raise IndexError(f"Frame index {tt} out of bounds for T={self._T}")
        return self._xyz, self._val[tt]

    def static_xyz(self) -> bool: return True
    def max_points(self) -> int: return self._xyz.shape[0]
    
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

    def __len__(self): return self.T
    def get(self, t):
        if self._mode == "static":
            xyz = self._xyz.astype(np.float32)
        else:
            xyz = self._xyz[t].astype(np.float32)
        return xyz, self._val[t]
    def static_xyz(self): return self._mode == "static"

def make_demo_provider(T=160, N=10000, radius=1.0):
    rng = np.random.default_rng(7)
    # uniform in sphere
    u = rng.random(N)
    cost = rng.uniform(-1,1,N); phi = rng.uniform(0,2*np.pi,N)
    r = radius * u**(1/3); s = np.sqrt(1-cost**2)
    xyz = np.stack([r*s*np.cos(phi), r*s*np.sin(phi), r*cost], axis=-1).astype(np.float32)

    # precompute time-series values per point
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
        def __len__(self): return T
        def get(self, t):
            return xyz, val_series[int(t)]
        def static_xyz(self): return True
        def max_points(self): return xyz.shape[0]

    return Demo()

# ------------- Figure helpers -------------

COLORSCALES = ["Viridis","Plasma","Cividis","Magma","Turbo","Inferno","Ice","Aggrnyl"]

@dataclass
class VizParams:
    cmap: str = "Viridis"
    size_min: float = 2.0
    size_max: float = 10.0
    downsample: int = 0  # 0 = no downsample; otherwise keep N//downsample points

def _figure_for_frame(xyz, val, vp: VizParams):
    N = xyz.shape[0]
    if vp.downsample and vp.downsample > 1:
        step = int(max(1, vp.downsample))
        xyz = xyz[::step]; val = val[::step]; N = xyz.shape[0]

    sizes = vp.size_min + (vp.size_max - vp.size_min)*val
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(
        x=xyz[:,0], y=xyz[:,1], z=xyz[:,2],
        mode="markers",
        marker=dict(
            size=np.clip(sizes, 1, 100),
            color=val, colorscale=vp.cmap, cmin=0, cmax=1,
            opacity=0.95,
        )
    ))
    fig.update_layout(
        scene=dict(aspectmode="data", uirevision="scene", dragmode="orbit"),
        margin=dict(l=0,r=0,t=30,b=0),
        template="plotly_dark",
        title="Animated Point Cloud (value → color & size)",
        uirevision="scene"
    )
    return fig

# ------------- Dash app factory -------------

def create_app(provider: ProviderBase, fps=24, default_params=None):
    app = Dash(__name__)
    T = len(provider)
    vp = default_params or VizParams()

    xyz0, val0 = provider.get(0)
    initial_fig = _figure_for_frame(xyz0, val0, vp)

    app.layout = html.Div([
        html.Div([
            html.Div([
                html.Button("⏵ Play", id="btn-play", n_clicks=0, style={"marginRight":"6px"}),
                html.Button("⏸ Pause", id="btn-pause", n_clicks=0, style={"marginRight":"12px"}),
                dcc.Slider(0, T-1, 1, value=0, id="slider-t", updatemode="drag",
                           tooltip={"placement":"bottom", "always_visible":False},
                           dots=False, marks=None),
            ], style={"flex":"1 1 auto"}),
        ], style={"display":"flex","gap":"12px","alignItems":"center","margin":"8px 12px"}),

        html.Div([
            html.Div([
                html.Label("Colormap"),
                dcc.Dropdown(options=[{"label":c, "value":c} for c in COLORSCALES],
                             value=vp.cmap, id="dd-cmap", clearable=False)
            ], style={"width":"220px"}),
            html.Div([
                html.Label("Size min (px)"),
                dcc.Slider(1, 20, 1, value=vp.size_min, id="sl-szmin",
                           marks=None, dots=False,
                           tooltip={"placement":"bottom", "always_visible":False}),
                html.Div(id="lbl-szmin", className="slider-value")
            ], style={"flex":"1","marginLeft":"16px"}),
            html.Div([
                html.Label("Size max (px)"),
                dcc.Slider(4, 40, 1, value=vp.size_max, id="sl-szmax",
                           marks=None, dots=False,
                           tooltip={"placement":"bottom", "always_visible":False}),
                html.Div(id="lbl-szmax", className="slider-value")
            ], style={"flex":"1","marginLeft":"16px"}),
            html.Div([
                html.Label("Downsample (every Nth point)"),
                dcc.Slider(0, 20, 1, value=vp.downsample, id="sl-down",
                           marks=None, dots=False,
                           tooltip={"placement":"bottom", "always_visible":False}),
                html.Div(id="lbl-down", className="slider-value")
            ], style={"flex":"1","marginLeft":"16px"}),
        ], style={"display":"flex","gap":"12px","alignItems":"center","margin":"8px 12px"}),

        dcc.Graph(
            id="graph",
            figure=initial_fig,
            style={"height":"80vh"},
            config={"scrollZoom": True, "doubleClick": "reset", "displaylogo": False}
        ),
        dcc.Interval(id="timer", interval=int(1000/max(1,fps)), n_intervals=0, disabled=True),
        dcc.Store(id="store-playing", data=False),
        dcc.Store(id="store-indices", data=None),
        dcc.Store(id="store-frame", data=None),
        dcc.Store(id="store-frame-ack", data=None),
    ])

    # Initial figure
    app._provider = provider  # stash
    app._vp = vp

    @app.callback(
        Output("store-playing","data"),
        Input("btn-play","n_clicks"),
        Input("btn-pause","n_clicks"),
        State("store-playing","data"),
        prevent_initial_call=True
    )
    def play_pause(n_play, n_pause, playing):
        ctx = dash.callback_context
        if not ctx.triggered:
            return playing
        trig = ctx.triggered[0]["prop_id"].split(".")[0]
        return (trig == "btn-play")

    @app.callback(
        Output("timer","disabled"),
        Input("store-playing","data")
    )
    def toggle_timer(playing):
        return not bool(playing)

    @app.callback(
        Output("slider-t","value"),
        Input("timer","n_intervals"),
        State("store-playing","data"),
        State("slider-t","value")
    )
    def tick(_, playing, t):
        if not playing:
            return t
        return (t + 1) % T

    @app.callback(
        Output("store-indices","data"),
        Input("sl-down","value"),
        prevent_initial_call=False
    )
    def compute_indices(down):
        ds = int(down or 0)
        if ds <= 1:
            return None
        N = provider.max_points()
        if not N:
            return None
        return np.arange(0, N, ds, dtype=np.int32).tolist()

    @app.callback(
        Output("graph","figure"),
        Output("store-frame","data"),
        Output("lbl-szmin","children"),
        Output("lbl-szmax","children"),
        Output("lbl-down","children"),
        Input("slider-t","value"),
        Input("dd-cmap","value"),
        Input("sl-szmin","value"),
        Input("sl-szmax","value"),
        Input("sl-down","value"),
        Input("store-indices","data"),
        prevent_initial_call=False
    )
    def update_fig(t, cmap, szmin, szmax, down, indices_data):
        # Pull frame and render
        xyz, val = provider.get(int(t))
        ds = int(down or 0)
        if ds > 1:
            if indices_data is not None:
                idx = np.asarray(indices_data, dtype=np.int64)
            else:
                idx = np.arange(0, xyz.shape[0], ds, dtype=np.int64)
            xyz_sel = xyz[idx]
            val_sel = val[idx]
        else:
            xyz_sel = xyz
            val_sel = val
        sizes = float(szmin) + (float(szmax) - float(szmin)) * val_sel
        triggered = {entry["prop_id"].split(".")[0] for entry in dash.callback_context.triggered}
        update_positions = not triggered  # initial call
        if "sl-down" in triggered or "store-indices" in triggered:
            update_positions = True
        color_list = val_sel.tolist()
        size_list = np.clip(sizes, 1, 100).tolist()
        restyle_payload = {
            "color": color_list,
            "size": size_list,
            "colorscale": cmap,
            "cmin": 0,
            "cmax": 1,
        }
        if update_positions:
            p = Patch()
            # Update coordinates only when downsample changes to minimize payload
            p["data"][0]["x"] = xyz_sel[:, 0].tolist()
            p["data"][0]["y"] = xyz_sel[:, 1].tolist()
            p["data"][0]["z"] = xyz_sel[:, 2].tolist()
            p["data"][0]["marker"]["color"] = color_list
            p["data"][0]["marker"]["size"] = size_list
            p["data"][0]["marker"]["colorscale"] = cmap
            p["data"][0]["marker"]["cmin"] = 0
            p["data"][0]["marker"]["cmax"] = 1
            figure_update = p
        else:
            figure_update = dash.no_update
        lbl_szmin = f"Current: {float(szmin):.0f} px"
        lbl_szmax = f"Current: {float(szmax):.0f} px"
        if ds <= 1:
            lbl_down = "Current: no downsampling"
        else:
            lbl_down = f"Current: keep every {ds}th point"
        return figure_update, restyle_payload, lbl_szmin, lbl_szmax, lbl_down

    app.clientside_callback(
        """
        function(frameData){
            if(!frameData){
                return null;
            }
            const container = document.getElementById('graph');
            if(!container){
                return frameData;
            }
            const plot = container.querySelector('.js-plotly-plot');
            if(!plot || !plot.data || !window.Plotly){
                return frameData;
            }
            const restyle = {};
            if(frameData.color){
                restyle['marker.color'] = [frameData.color];
            }
            if(frameData.size){
                restyle['marker.size'] = [frameData.size];
            }
            if(frameData.colorscale){
                restyle['marker.colorscale'] = [frameData.colorscale];
            }
            if(frameData.cmin !== undefined){
                restyle['marker.cmin'] = [frameData.cmin];
            }
            if(frameData.cmax !== undefined){
                restyle['marker.cmax'] = [frameData.cmax];
            }
            if(Object.keys(restyle).length){
                window.Plotly.restyle(plot, restyle, [0]);
            }
            return frameData;
        }
        """,
        Output("store-frame-ack","data"),
        Input("store-frame","data"),
        prevent_initial_call=True
    )

    # seed initial figure
    app._initial_fig = initial_fig
    return app

# ------------- CLI -------------

def infer_points(path, pattern):
    idx = 0
    # Count consecutive point files starting at index 0
    while os.path.exists(os.path.join(path, pattern.format(idx))):
        idx += 1
    return idx

def main():
    ap = argparse.ArgumentParser(description="Dash animated point-cloud viewer.")
    ap.add_argument("--demo", action="store_true", help="Use synthetic expanding wavefront")
    ap.add_argument("--path", type=str, default=".", help="Folder with per-point NPZ files")
    ap.add_argument("--pattern", type=str, default="point_{:05d}.npz", help="Filename pattern with {:d} placeholder for point index")
    ap.add_argument("--points", type=int, help="Number of point files (if not inferable)")
    ap.add_argument("--series", type=str, help="Single NPZ with xyz/val time series")
    ap.add_argument("--fps", type=int, default=24, help="Playback FPS")
    ap.add_argument("--size-min", type=float, default=2.0)
    ap.add_argument("--size-max", type=float, default=10.0)
    ap.add_argument("--downsample", type=int, default=0, help="Keep every Nth point (0=no downsample)")
    ap.add_argument("--host", type=str, default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8050)
    args = ap.parse_args()

    if args.demo:
        provider = make_demo_provider()
    elif args.series:
        provider = NPZSeriesProvider(args.series)
    else:
        points = args.points or infer_points(args.path, args.pattern)
        if points == 0:
            raise SystemExit("No point files found. Check --path/--pattern or use --demo.")
        provider = PointFilesProvider(args.path, args.pattern, points)

    vp = VizParams(
        cmap="Viridis", size_min=args.size_min, size_max=args.size_max, downsample=max(0,args.downsample)
    )
    app = create_app(provider, fps=args.fps, default_params=vp)
    # Serve with initial figure to avoid first-render lag
    # @app.server._got_first_request #before_first_request
    # def _seed():
    #     pass
    app.run(host=args.host, port=args.port, debug=False)

if __name__ == "__main__":
    main()
