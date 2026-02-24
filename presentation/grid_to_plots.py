from __future__ import annotations

import numpy as np
from pathlib import Path
from manim import (
    BLUE,
    DOWN,
    IN,
    LEFT,
    OUT,
    RIGHT,
    UP,
    Arrow,
    Axes,
    Cone,
    Create,
    Cube,
    DEGREES,
    FadeIn,
    FadeOut,
    ImageMobject,
    LaggedStart,
    Line,
    Scene,
    Surface,
    Square,
    Text,
    ThreeDScene,
    ValueTracker,
    VGroup,
)
from manim.utils.color import ManimColor, interpolate_color


class GridToPlots(Scene):
    def construct(self):
        screenshot_path = Path(__file__).parent / "images" / "Screenshot_20260202_221921-2.png"
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

        def make_small_plot(phase: float) -> VGroup:
            axes = Axes(
                x_range=(0, 5, 1),
                y_range=(-1.2, 1.2, 0.5),
                x_length=1.6,
                y_length=0.7,
                tips=False,
                axis_config={"include_ticks": False, "stroke_width": 2},
            )
            x_vals = np.linspace(0, 5, 60)
            y_vals = np.sin(2 * np.pi * x_vals / 5 + phase)
            line = axes.plot_line_graph(
                x_vals,
                y_vals,
                add_vertex_dots=False,
                line_color=BLUE,
                stroke_width=2,
            )
            return VGroup(axes, line)

        phases = np.linspace(0, 2 * np.pi, 5, endpoint=False)
        plots = VGroup(*[make_small_plot(phase) for phase in phases])
        plots.arrange(RIGHT, buff=0.4, aligned_edge=DOWN)
        plots.next_to(grid, UP, buff=0.6).align_to(grid, LEFT)

        lines = VGroup()
        for idx, plot in enumerate(plots):
            start = grid[idx].get_center()
            end = plot.get_bottom()
            lines.add(Arrow(start, end, buff=0, stroke_width=2))

        for idx in range(len(plots)):
            self.play(
                Create(lines[idx]),
                FadeIn(plots[idx], shift=UP * 0.2),
                run_time=0.6,
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
        cube_color = "#3B0A64"
        cube_edge_color = "#C9B2FF"

        cubes = VGroup()
        for z in range(grid_size):
            for y in range(grid_size):
                for x in range(grid_size):
                    cube = Cube(side_length=cube_size)
                    cube.set_fill(cube_color, opacity=1.0)
                    cube.set_stroke(width=1.6, color=cube_edge_color)
                    cube.move_to([start + x * step, start + y * step, start + z * step])
                    cubes.add(cube)

        self.play(
            LaggedStart(
                *[Create(cube) for cube in cubes],
                lag_ratio=0.02,
                run_time=1.0,
            )
        )
        self.wait(0.2)

        rot_matrix = self.camera.get_rotation_matrix()
        frame_center = self.camera.frame_center
        for cube in cubes:
            depth = np.dot(cube.get_center() - frame_center, rot_matrix.T)[2]
            cube.set_z_index(depth, family=True)

        def make_small_plot(phase: float) -> VGroup:
            axes = Axes(
                x_range=(0, 5, 1),
                y_range=(-1.2, 1.2, 0.5),
                x_length=1.6,
                y_length=0.7,
                tips=False,
                axis_config={"include_ticks": False, "stroke_width": 2},
            )
            x_vals = np.linspace(0, 5, 60)
            y_vals = np.sin(2 * np.pi * x_vals / 5 + phase)
            line = axes.plot_line_graph(
                x_vals,
                y_vals,
                add_vertex_dots=False,
                line_color=BLUE,
                stroke_width=2,
            )
            return VGroup(axes, line)

        phases = np.linspace(0, 2 * np.pi, 5, endpoint=False)
        plots = VGroup(*[make_small_plot(phase) for phase in phases])

        plot_step = 1.0
        start_y = plot_step * (len(plots) - 1) / 2
        plot_x = -self.camera.frame_width / 2 + 1.4
        for idx, plot in enumerate(plots):
            plot.move_to([plot_x, start_y - idx * plot_step, 0.0])
        self.camera.add_fixed_in_frame_mobjects(*plots)

        rng = np.random.default_rng(64)
        start_indices = rng.choice(len(cubes), size=len(plots), replace=False)
        start_cubes = [cubes[idx] for idx in start_indices]
        lines = VGroup(
            *[
                Arrow(
                    self.camera.project_point(start_cubes[idx].get_corner(LEFT + DOWN + IN)),
                    plots[idx].get_right(),
                    buff=0,
                    stroke_width=2,
                )
                for idx in range(len(plots))
            ]
        )
        self.camera.add_fixed_in_frame_mobjects(*lines)

        animations = []
        for idx in range(len(plots)):
            animations.append(Create(lines[idx]))
            animations.append(FadeIn(plots[idx], shift=LEFT * 0.2))

        self.play(
            LaggedStart(
                *animations,
                lag_ratio=0.1,
                run_time=2.6,
            )
        )
        self.wait(1.0)



class GridToPlots3DEffector(ThreeDScene):
    def construct(self):
        self.set_camera_orientation(phi=65 * DEGREES, theta=-45 * DEGREES, zoom=0.95)

        grid_size = 4
        cube_size = 0.35
        gap = 0.25
        step = cube_size + gap
        start = -0.5 * (grid_size - 1) * step
        cube_color = "#3B0A64"
        cube_edge_color = "#000000"
        highlight_fill_color = "#F4B183"

        cubes = VGroup()
        for z in range(grid_size):
            for y in range(grid_size):
                for x in range(grid_size):
                    cube = Cube(side_length=cube_size)
                    cube.set_fill(cube_color, opacity=1.0)
                    cube.set_stroke(width=1.6, color=cube_edge_color)
                    cube.move_to([start + x * step, start + y * step, start + z * step])
                    cubes.add(cube)

        self.play(
            LaggedStart(
                *[Create(cube) for cube in cubes],
                lag_ratio=0.02,
                run_time=1.0,
            )
        )
        self.wait(0.2)

        rot_matrix = self.camera.get_rotation_matrix()
        frame_center = self.camera.frame_center
        for cube in cubes:
            depth = np.dot(cube.get_center() - frame_center, rot_matrix.T)[2]
            cube.set_z_index(depth, family=True)

        def make_small_plot(phase: float) -> VGroup:
            axes = Axes(
                x_range=(0, 5, 1),
                y_range=(-1.2, 1.2, 0.5),
                x_length=1.6,
                y_length=0.7,
                tips=False,
                axis_config={"include_ticks": False, "stroke_width": 2},
            )
            x_vals = np.linspace(0, 5, 60)
            y_vals = np.sin(2 * np.pi * x_vals / 5 + phase)
            line = axes.plot_line_graph(
                x_vals,
                y_vals,
                add_vertex_dots=False,
                line_color=BLUE,
                stroke_width=2,
            )
            return VGroup(axes, line)

        phases = np.linspace(0, 2 * np.pi, 5, endpoint=False)
        plots = VGroup(*[make_small_plot(phase) for phase in phases])

        plot_step = 1.0
        start_y = plot_step * (len(plots) - 1) / 2
        plot_x = -self.camera.frame_width / 2 + 1.4
        for idx, plot in enumerate(plots):
            plot.move_to([plot_x, start_y - idx * plot_step, 0.0])

        def fix_in_frame(mobjects: VGroup) -> None:
            self.camera.add_fixed_in_frame_mobjects(*mobjects)

        # rng = np.random.default_rng(64)
        # start_indices = rng.choice(len(cubes), size=len(plots), replace=False)
        start_indices = [0,1,2,3]
        start_cubes = [cubes[idx] for idx in start_indices]

        min_z = start
        max_z = start + (grid_size - 1) * step
        effector_top_z = max_z + 0.9

        def add_plot_with_line(idx: int) -> Arrow:
            line = Arrow(
                self.camera.project_point(start_cubes[idx].get_corner(LEFT + DOWN + IN)),
                plots[idx].get_right(),
                buff=0,
                stroke_width=2,
            )
            fix_in_frame(VGroup(line, plots[idx]))
            return line

        selected_cubes = start_cubes
        segment_start = [selected_cubes[0].get_center()]
        segment_end = [segment_start[0]]
        move_tracker = ValueTracker(0.0)

        def update_effector(mob: Line) -> None:
            alpha = move_tracker.get_value()
            current = segment_start[0] + alpha * (segment_end[0] - segment_start[0])
            start_point = np.array([current[0], current[1], effector_top_z])
            end_point = np.array([current[0], current[1], current[2]])
            start_2d = self.camera.project_point(start_point)
            end_2d = self.camera.project_point(end_point)
            start_2d[2] = 0.0
            end_2d[2] = 0.0
            mob.put_start_and_end_on(start_2d, end_2d)

        start_point = np.array(
            [segment_start[0][0], segment_start[0][1], effector_top_z]
        )
        end_point = np.array(
            [segment_start[0][0], segment_start[0][1], segment_start[0][2]]
        )
        start_2d = self.camera.project_point(start_point)
        end_2d = self.camera.project_point(end_point)
        start_2d[2] = 0.0
        end_2d[2] = 0.0
        effector = Line(start_2d, end_2d, stroke_width=6, color=highlight_fill_color)
        effector.add_updater(update_effector)
        fix_in_frame(VGroup(effector))

        first_line = add_plot_with_line(0)
        self.play(
            Create(effector),
            Create(first_line),
            FadeIn(plots[0], shift=LEFT * 0.2),
            selected_cubes[0].animate.set_fill(highlight_fill_color),
            run_time=0.8,
        )

        for idx, (prev_cube, next_cube) in enumerate(
            zip(selected_cubes, selected_cubes[1:]),
            start=1,
        ):
            segment_start[0] = prev_cube.get_center()
            segment_end[0] = next_cube.get_center()
            move_tracker.set_value(0.0)
            next_line = add_plot_with_line(idx)
            self.play(
                move_tracker.animate.set_value(1.0),
                Create(next_line),
                FadeIn(plots[idx], shift=LEFT * 0.2),
                prev_cube.animate.set_fill(cube_color),
                next_cube.animate.set_fill(highlight_fill_color),
                run_time=1.4,
            )
            self.wait(0.2)

        self.wait(1.0)



class GridToPlots3DEffectorWithWave(ThreeDScene):
    def construct(self):
        self.set_camera_orientation(phi=65 * DEGREES, theta=-45 * DEGREES, zoom=0.95)

        grid_size = 4
        cube_size = 0.35
        gap = 0.25
        step = cube_size + gap
        start = -0.5 * (grid_size - 1) * step
        cube_color = "#3B0A64"
        cube_edge_color = "#000000"
        highlight_fill_color = "#F4B183"

        cubes = VGroup()
        for z in range(grid_size):
            for y in range(grid_size):
                for x in range(grid_size):
                    cube = Cube(side_length=cube_size)
                    cube.set_fill(cube_color, opacity=1.0)
                    cube.set_stroke(width=1.6, color=cube_edge_color)
                    cube.move_to([start + x * step, start + y * step, start + z * step])
                    cubes.add(cube)

        self.play(
            LaggedStart(
                *[Create(cube) for cube in cubes],
                lag_ratio=0.02,
                run_time=1.0,
            )
        )
        self.wait(0.2)

        rot_matrix = self.camera.get_rotation_matrix()
        frame_center = self.camera.frame_center
        for cube in cubes:
            depth = np.dot(cube.get_center() - frame_center, rot_matrix.T)[2]
            cube.set_z_index(depth, family=True)

        grid_min = start
        grid_max = start + (grid_size - 1) * step
        grid_center = np.array([grid_min + (grid_max - grid_min) / 2] * 3)
        speaker_pos = np.array([grid_max + 0.8, grid_min - 0.4, grid_min - 0.8])
        speaker_dir = grid_center - speaker_pos
        speaker_dir /= np.linalg.norm(speaker_dir)

        speaker_cone = Cone(base_radius=0.28, height=0.55, direction=-speaker_dir)
        speaker_cone.set_fill("#3A3A3A", opacity=1.0)
        speaker_cone.set_stroke(width=1.0, color="#666666")
        speaker_cone.move_to(speaker_pos)
        self.play(FadeIn(speaker_cone, shift=speaker_dir * 0.2), run_time=0.6)

        wave_color = "#8ED6FF"
        cube_wave_color = "#8ED6FF"
        base_color = ManimColor(cube_color)
        wave_fill_color = ManimColor(cube_wave_color)
        ordered_cubes = sorted(
            cubes,
            key=lambda cube: np.dot(cube.get_center() - speaker_pos, speaker_dir),
        )
        grid_span = (grid_size - 1) * step + cube_size
        wave_half_span = 0.8 * grid_span

        def make_wave_surface() -> Surface:
            surface = Surface(
                lambda u, v: np.array([-0.16 * (u**2 + v**2), u, v]),
                u_range=(-wave_half_span, wave_half_span),
                v_range=(-wave_half_span, wave_half_span),
                resolution=(14, 28),
                fill_color=wave_color,
                fill_opacity=0.2,
                stroke_color=wave_color,
                stroke_width=1.2,
            )
            local_dir = np.array([1.0, 0.0, 0.0])
            axis = np.cross(local_dir, speaker_dir)
            axis_norm = np.linalg.norm(axis)
            if axis_norm > 1e-6:
                angle = np.arccos(
                    np.clip(np.dot(local_dir, speaker_dir), -1.0, 1.0)
                )
                surface.rotate(angle, axis=axis / axis_norm)
            return surface

        wave_start = speaker_pos + speaker_dir * 0.6
        wave_end = grid_center + speaker_dir * (grid_size * step * 0.6)
        travel_dist = np.linalg.norm(wave_end - wave_start)
        cube_projections = [
            np.dot(cube.get_center() - wave_start, speaker_dir) / travel_dist
            for cube in ordered_cubes
        ]
        pulse_width = 0.18

        def add_cube_updaters(tracker: ValueTracker) -> list[tuple[Cube, callable]]:
            updaters = []
            for cube, proj in zip(ordered_cubes, cube_projections):
                def updater(mob, proj=proj):
                    delta = tracker.get_value() - proj
                    alpha = max(0.0, 1.0 - abs(delta) / pulse_width)
                    mob.set_fill(interpolate_color(base_color, wave_fill_color, alpha))
                cube.add_updater(updater)
                updaters.append((cube, updater))
            return updaters

        for _ in range(3):
            wave = make_wave_surface().move_to(wave_start)
            wave_tracker = ValueTracker(0.0)

            def update_wave(mob: Surface) -> None:
                t = wave_tracker.get_value()
                mob.move_to(wave_start + t * (wave_end - wave_start))

            wave.add_updater(update_wave)
            cube_updaters = add_cube_updaters(wave_tracker)
            self.add(wave)
            self.play(wave_tracker.animate.set_value(1.0), run_time=2.0)
            for cube, updater in cube_updaters:
                cube.remove_updater(updater)
                cube.set_fill(cube_color)
            wave.remove_updater(update_wave)
            self.remove(wave)

        def make_small_plot(phase: float) -> VGroup:
            axes = Axes(
                x_range=(0, 5, 1),
                y_range=(-1.2, 1.2, 0.5),
                x_length=1.6,
                y_length=0.7,
                tips=False,
                axis_config={"include_ticks": False, "stroke_width": 2},
            )
            x_vals = np.linspace(0, 5, 60)
            y_vals = np.sin(2 * np.pi * x_vals / 5 + phase)
            line = axes.plot_line_graph(
                x_vals,
                y_vals,
                add_vertex_dots=False,
                line_color=BLUE,
                stroke_width=2,
            )
            return VGroup(axes, line)

        phases = np.linspace(0, 2 * np.pi, 5, endpoint=False)
        plots = VGroup(*[make_small_plot(phase) for phase in phases])

        plot_step = 1.0
        start_y = plot_step * (len(plots) - 1) / 2
        plot_x = -self.camera.frame_width / 2 + 1.4
        for idx, plot in enumerate(plots):
            plot.move_to([plot_x, start_y - idx * plot_step, 0.0])

        def fix_in_frame(mobjects: VGroup) -> None:
            self.camera.add_fixed_in_frame_mobjects(*mobjects)

        # rng = np.random.default_rng(64)
        # start_indices = rng.choice(len(cubes), size=len(plots), replace=False)
        start_indices = [0,1,2,3]
        start_cubes = [cubes[idx] for idx in start_indices]

        min_z = start
        max_z = start + (grid_size - 1) * step
        effector_top_z = max_z + 0.9

        def add_plot_with_line(idx: int) -> Arrow:
            line = Arrow(
                self.camera.project_point(start_cubes[idx].get_corner(LEFT + DOWN + IN)),
                plots[idx].get_right(),
                buff=0,
                stroke_width=2,
            )
            fix_in_frame(VGroup(line, plots[idx]))
            return line

        selected_cubes = start_cubes
        segment_start = [selected_cubes[0].get_center()]
        segment_end = [segment_start[0]]
        move_tracker = ValueTracker(0.0)

        def update_effector(mob: Line) -> None:
            alpha = move_tracker.get_value()
            current = segment_start[0] + alpha * (segment_end[0] - segment_start[0])
            start_point = np.array([current[0], current[1], effector_top_z])
            end_point = np.array([current[0], current[1], current[2]])
            start_2d = self.camera.project_point(start_point)
            end_2d = self.camera.project_point(end_point)
            start_2d[2] = 0.0
            end_2d[2] = 0.0
            mob.put_start_and_end_on(start_2d, end_2d)

        start_point = np.array(
            [segment_start[0][0], segment_start[0][1], effector_top_z]
        )
        end_point = np.array(
            [segment_start[0][0], segment_start[0][1], segment_start[0][2]]
        )
        start_2d = self.camera.project_point(start_point)
        end_2d = self.camera.project_point(end_point)
        start_2d[2] = 0.0
        end_2d[2] = 0.0
        effector = Line(start_2d, end_2d, stroke_width=6, color=highlight_fill_color)
        effector.add_updater(update_effector)
        fix_in_frame(VGroup(effector))

        first_line = add_plot_with_line(0)
        self.play(
            Create(effector),
            Create(first_line),
            FadeIn(plots[0], shift=LEFT * 0.2),
            selected_cubes[0].animate.set_fill(highlight_fill_color),
            run_time=0.8,
        )

        for idx, (prev_cube, next_cube) in enumerate(
            zip(selected_cubes, selected_cubes[1:]),
            start=1,
        ):
            segment_start[0] = prev_cube.get_center()
            segment_end[0] = next_cube.get_center()
            move_tracker.set_value(0.0)
            next_line = add_plot_with_line(idx)
            self.play(
                move_tracker.animate.set_value(1.0),
                Create(next_line),
                FadeIn(plots[idx], shift=LEFT * 0.2),
                prev_cube.animate.set_fill(cube_color),
                next_cube.animate.set_fill(highlight_fill_color),
                run_time=1.4,
            )
            self.wait(0.2)

        self.wait(1.0)
