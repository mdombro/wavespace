from __future__ import annotations

import numpy as np
from pathlib import Path
from manim import (
    BLUE,
    BLUE_A,
    DOWN,
    LEFT,
    RIGHT,
    UP,
    Axes,
    Create,
    Cube,
    DEGREES,
    FadeIn,
    ImageMobject,
    LaggedStart,
    Line,
    Scene,
    Square,
    Text,
    ThreeDScene,
    VGroup,
    always_redraw,
)


class GridToPlots(Scene):
    def construct(self):
        screenshot_path = Path(__file__).with_name("Screenshot_20260202_221921-2.png")
        if screenshot_path.exists():
            overlay = ImageMobject(str(screenshot_path))
            overlay.set_opacity(0.25)
            overlay.scale_to_fit_width(self.camera.frame_width)
            overlay.scale_to_fit_height(self.camera.frame_height)
            self.add(overlay)

        rows, cols = 4, 5
        square_size = 0.4
        grid = VGroup(*[Square(side_length=square_size) for _ in range(rows * cols)])
        for sq in grid:
            sq.set_fill(opacity=0)
            sq.set_stroke(width=2)

        margin_x = 0.6
        row_gap = 0.26
        available_width = self.camera.frame_width - 2 * margin_x
        col_gap = max((available_width - cols * square_size) / (cols - 1), 0.1)
        col_step = square_size + col_gap
        row_step = square_size + row_gap
        start_x = -self.camera.frame_width / 2 + margin_x + square_size / 2
        start_y = row_step * (rows - 1) / 2
        for row in range(rows):
            for col in range(cols):
                idx = row * cols + col
                x = start_x + col * col_step
                y = start_y - row * row_step
                grid[idx].move_to([x, y, 0])

        self.play(
            LaggedStart(
                *[Create(sq) for sq in grid],
                lag_ratio=0.08,
                run_time=2.5,
            )
        )
        self.wait(0.2)

        def make_small_plot() -> VGroup:
            axes = Axes(
                x_range=(0, 5, 1),
                y_range=(-1.2, 1.2, 0.5),
                x_length=1.6,
                y_length=0.7,
                tips=False,
                axis_config={"include_ticks": False, "stroke_width": 2},
            )
            x_vals = np.linspace(0, 5, 60)
            y_vals = np.sin(2 * np.pi * x_vals / 5)
            line = axes.plot_line_graph(
                x_vals,
                y_vals,
                add_vertex_dots=False,
                line_color=BLUE,
                stroke_width=2,
            )
            return VGroup(axes, line)

        plots = VGroup(*[make_small_plot() for _ in range(5)])
        plots.arrange(RIGHT, buff=0.4, aligned_edge=DOWN)
        plots.next_to(grid, UP, buff=0.6).align_to(grid, LEFT)

        lines = VGroup()
        for idx, plot in enumerate(plots):
            start = grid[idx].get_center()
            end = plot.get_bottom()
            lines.add(Line(start, end, stroke_width=2))

        self.play(
            LaggedStart(
                *[Create(line) for line in lines],
                *[FadeIn(plot, shift=UP * 0.2) for plot in plots],
                lag_ratio=0.1,
                run_time=2.5,
            )
        )

        ellipsis = Text("...").scale(0.8)
        ellipsis.next_to(plots, RIGHT, buff=0.3).align_to(plots, DOWN)
        self.play(FadeIn(ellipsis, shift=RIGHT * 0.1))
        self.wait(1.0)


class GridToPlots3D(ThreeDScene):
    def construct(self):
        self.set_camera_orientation(phi=65 * DEGREES, theta=-45 * DEGREES, zoom=0.95)

        grid_size = 4
        cube_size = 0.35
        gap = 0.25
        step = cube_size + gap
        start = -0.5 * (grid_size - 1) * step

        cubes = VGroup()
        for z in range(grid_size):
            for y in range(grid_size):
                for x in range(grid_size):
                    cube = Cube(side_length=cube_size)
                    cube.set_fill(BLUE_A, opacity=1.0)
                    cube.set_stroke(width=1.0, color=BLUE)
                    cube.move_to([start + x * step, start + y * step, start + z * step])
                    cubes.add(cube)

        self.play(
            LaggedStart(
                *[Create(cube) for cube in cubes],
                lag_ratio=0.02,
                run_time=3.0,
            )
        )
        self.wait(0.2)

        def make_small_plot() -> VGroup:
            axes = Axes(
                x_range=(0, 5, 1),
                y_range=(-1.2, 1.2, 0.5),
                x_length=1.6,
                y_length=0.7,
                tips=False,
                axis_config={"include_ticks": False, "stroke_width": 2},
            )
            x_vals = np.linspace(0, 5, 60)
            y_vals = np.sin(2 * np.pi * x_vals / 5)
            line = axes.plot_line_graph(
                x_vals,
                y_vals,
                add_vertex_dots=False,
                line_color=BLUE,
                stroke_width=2,
            )
            return VGroup(axes, line)

        plots = VGroup(*[make_small_plot() for _ in range(5)])

        frame_left = -self.camera.frame_width / 2
        plot_x = frame_left + 1.4
        plot_step = 1.0
        start_y = plot_step * (len(plots) - 1) / 2
        for idx, plot in enumerate(plots):
            plot.move_to([plot_x, start_y - idx * plot_step, 0.0])
        self.add_fixed_in_frame_mobjects(plots)

        lines = VGroup(
            *[
                always_redraw(
                    lambda idx=idx: Line(
                        self.camera.project_point(cubes[idx].get_center()),
                        plots[idx].get_critical_point(RIGHT),
                        stroke_width=2,
                    )
                )
                for idx in range(len(plots))
            ]
        )
        self.add_fixed_in_frame_mobjects(lines)

        self.play(
            LaggedStart(
                *[Create(line) for line in lines],
                *[FadeIn(plot, shift=LEFT * 0.2) for plot in plots],
                lag_ratio=0.1,
                run_time=2.5,
            )
        )
        self.wait(1.0)
