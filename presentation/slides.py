from manim import *
from manim_slides import Slide, ThreeDSlide
from manim.utils.color import ManimColor, interpolate_color
from slide_template import IntroTextTemplate, TextSlideSpec, TextBlockSpec, SlideImageSpec

import numpy as np
from pathlib import Path
from typing import Literal

YOUTUBE_CLIP_PATH = (
    Path(__file__).parent / "clips" / "o4TdHrMi6do_01m12s_01m24s.mp4"
)

class IntroPipeline(IntroTextTemplate):
    SPEC = TextSlideSpec(
        text_blocks=[
            TextBlockSpec(
                markdown=(
                    "# WAVESPACE\n"
                    "## Wide-area Acoustic Vector-field Estimation for Spatial Probing & Array Calibration Environment\n"
                    "<br>\n"
                    "(This is mostly a silly, ChatGPT assisted, made up name...)\n"
                    "<br>\n"
                    "### Wavespace is a project that can directly measure and then visualize the form and dynamics of a sound\n"
                    "wave traveling through air. A fun experiment tinkering with the Pi Pico and viewing the structures\n"
                    "of waves in full three dimensions as opposed to the typical two."
                    # "A reusable intro slide now renders from markdown and supports inline **bold** and *italics*.\n\n"
                    # "- Collect synchronized mic data\n"
                    # "- Process and transform signals\n"
                    # "  - Band-pass + normalization\n"
                    # "  - Feature extraction\n"
                    # "- Render 2D and 3D visual summaries"
                ),
                animation="stagger",     # "none" | "fade" | "stagger"
                horizontal_placement="left",  # "left" | "center" | "right"
                vertical_placement="top",     # "top" | "center"
                text_alignment="left",        # "left" | "center" | "right"
            )
        ],
        images=[
            [
                SlideImageSpec(Path("wavespace.png"), animation="fade"),
                SlideImageSpec(
                    Path("465766bf-dfa6-4b8e-93d9-3e45d605a487.jpg"),
                    animation="fade",
                    rotation_degrees=180
                ),
                SlideImageSpec(
                    Path("Screenshot_20260209_210554.png"),
                    animation="fade",
                ),
            ],  
        ],
        image_group_layouts=["right"],
        image_group_directions=["horizontal"],
        image_group_spacings=[0.18],
        image_group_offsets=[(0.0, 0.0)],
        image_group_positions=["bottom"],  # anchor group to slide edge/corner
        image_group_position_buffs=[0.45],
        uniform_image_size=[True],
        image_size_ratio=[0.30],       # per-group width proportions
        image_max_height_ratio=[0.40], # per-group max height proportions
        image_rotation_degrees=[0.0],   # per-group defaults unless per-image override is set
    )


class PersonalIntro(IntroTextTemplate):
    SPEC = TextSlideSpec(
        text_blocks=[
            TextBlockSpec(
                markdown=(
                    "# Where have I appeared from?<br>"
                    "Shout out to Will Morrison of Marble run fame for suggesting I come here tonight!"
                    "We are both part of the local Cambridge Hackspace orginziation - come check us out!"
                    "Will is very humble, but he did all of the work on the physical gantry system for this project"
                    "Shout out to Will for saving me a ton of time mucking about with stepper motors and drive systems!"
                    "<br><br>"
                    "5 second background - BS in Electrical and Computer Engineering and have worked adjacent to RF and radio"
                    " communications, so have a good number of years working close with the magic of waves and their behaviors."
                )
            )
        ]
    )

class IdeaOrigination(IntroTextTemplate):
    SPEC = TextSlideSpec(
        text_blocks=[
            TextBlockSpec(
                markdown=(
                    """
                    # Where did this idea come from?
                    <br>The original idea was inspired by the Alpha Phoenix video about the 2 billion frames per second camera to 
                    capture light traveling across the room.
                    <br>
                    https://www.youtube.com/watch?v=o4TdHrMi6do
                    <br>
                    Go watch the full video when you have time!
                    <br>
                    However, this required some specialized equipment and extrememly tight timing requirements. Plus, he already did it!
                     SOUND, though is MUCH easier to sample and work with!
                    <br>
                    So we went with that -- plus with sound, we can capture a THREE dimensional structure of sound.
                    This has always fascinated me because real waves are 3 dimensional and really, really complex because of that
                    It is something you never quite get a good appreciation of through a textbook
                    """
                )
            )
        ],
        images=[
            [SlideImageSpec(Path("g4.png"), animation="fade")]
        ],
        image_group_positions=["bottom_right"],
        image_group_layouts=["right"],
        image_size_ratio=[0.20],
        image_max_height_ratio=0.25
    )



class Methodology(IntroTextTemplate):
    SPEC = TextSlideSpec(
        text_blocks=[TextBlockSpec(markdown="")]
    )


class YouTubeClip(Slide):
    # External clip slides do not need reverse generation for this deck flow.
    skip_reversing = True

    def construct(self):
        if not YOUTUBE_CLIP_PATH.exists():
            raise FileNotFoundError(
                "Missing YouTube clip slide source at "
                f"{YOUTUBE_CLIP_PATH}. Put a local MP4 clipped to 1:12-1:24 there."
            )
        self.next_slide(src=YOUTUBE_CLIP_PATH)

class TwoDAnimations(Slide):
    def construct(self): 
        screenshot_path = Path(__file__).with_name("Screenshot_20260202_221921-2.png")
        if screenshot_path.exists():
            overlay = ImageMobject(str(screenshot_path))
            overlay.set_opacity(0.25)
            overlay.scale_to_fit_width(self.camera.frame_width)
            overlay.scale_to_fit_height(self.camera.frame_height)
            self.add(overlay)

        rows, cols = 20, 40
        square_size = 0.16
        grid = VGroup(*[Square(side_length=square_size) for _ in range(rows * cols)])
        for sq in grid:
            sq.set_fill(opacity=0)
            sq.set_stroke(width=2)

        margin_x = 0.6
        row_gap = 0.08
        available_width = self.camera.frame_width - 2 * margin_x
        col_gap = max((available_width - cols * square_size) / (cols - 1), 0.1)
        col_step = square_size + col_gap
        row_step = square_size + row_gap
        start_x = -self.camera.frame_width / 2 + margin_x + square_size / 2
        plot_buff = 0.6
        estimated_plot_height = 0.9
        top_clear_band = 0.20 * self.camera.frame_height
        max_content_top_y = self.camera.frame_height / 2 - top_clear_band
        max_grid_top_y = max_content_top_y - plot_buff - estimated_plot_height
        default_start_y = row_step * (rows - 1) / 2
        start_y = min(default_start_y, max_grid_top_y - square_size / 2) + 0.5
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
        plots.next_to(grid, UP, buff=plot_buff).align_to(grid, LEFT)

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


        self.next_slide()


class TwoDToThreeDWave(ThreeDSlide):
    def construct(self):
        x_span = 4.2
        y_span = 2.6

        self.set_camera_orientation(phi=0 * DEGREES, theta=-90 * DEGREES, zoom=1.0)

        plane = Surface(
            lambda u, v: np.array([u, v, 0.0]),
            u_range=(-x_span, x_span),
            v_range=(-y_span, y_span),
            resolution=(24, 14),
            fill_color=BLUE_E,
            fill_opacity=0.35,
            stroke_color=BLUE_B,
            stroke_width=1.0,
        )
        plane.set_shade_in_3d(True)

        disturbance = Dot3D(point=ORIGIN, radius=0.08, color=YELLOW)
        disturbance.set_shade_in_3d(False)

        ring_tracker = ValueTracker(0.0)
        max_radius = 4.0
        ring_spacing = 0.55
        ring_count = 8
        ring_cycle_end = max_radius + ring_spacing * (ring_count - 1)

        rings = VGroup()
        for i in range(ring_count):
            phase = i * ring_spacing
            ring = Circle(radius=1e-3, color=TEAL_A, stroke_width=3.0)
            ring.move_to(ORIGIN)

            def ring_updater(mob: Circle, phase=phase) -> None:
                radius = ring_tracker.get_value() - phase
                if radius <= 0.0 or radius >= max_radius:
                    mob.set_stroke(opacity=0.0)
                    return
                mob.become(
                    Circle(
                        radius=radius,
                        color=TEAL_A,
                        stroke_width=3.0,
                        stroke_opacity=max(0.0, 1.0 - radius / max_radius),
                    )
                )
                mob.move_to(ORIGIN)

            ring.add_updater(ring_updater)
            rings.add(ring)

        self.add(plane, rings, disturbance)
        self.next_slide(loop=True)
        self.play(
            ring_tracker.animate.set_value(ring_cycle_end),
            run_time=12.0,
            rate_func=linear,
        )
        self.next_slide()

        ripple_phase = ValueTracker(0.0)
        ripple_amplitude = ValueTracker(0.0)
        ripple_wavenumber = 3.3
        decay = 0.32

        rippled_plane = always_redraw(
            lambda: Surface(
                lambda u, v: np.array(
                    [
                        u,
                        v,
                        ripple_amplitude.get_value()
                        * np.sin(ripple_wavenumber * np.sqrt(u**2 + v**2) - ripple_phase.get_value())
                        * np.exp(-decay * np.sqrt(u**2 + v**2)),
                    ]
                ),
                u_range=(-x_span, x_span),
                v_range=(-y_span, y_span),
                resolution=(24, 14),
                fill_color=BLUE_D,
                fill_opacity=0.5,
                stroke_color=BLUE_B,
                stroke_width=1.0,
            )
        )

        self.remove(plane)
        self.add(rippled_plane)
        self.move_camera(
            phi=45 * DEGREES,
            theta=-45 * DEGREES,
            added_anims=[
                ring_tracker.animate.set_value(ring_cycle_end + 1.2),
                ripple_amplitude.animate.set_value(0.5),
                ripple_phase.animate.set_value(7.0),
            ],
            run_time=3.0,
        )
        self.next_slide(loop=True)
        self.play(
            ripple_phase.animate.set_value(19.0),
            run_time=10.0,
            rate_func=linear,
        )
        self.next_slide()

        arrow_x = x_span + 0.35
        arrow_up = Arrow3D(
            start=np.array([arrow_x, 0.0, 0.05]),
            end=np.array([arrow_x, 0.0, 1.8]),
            color=YELLOW,
            thickness=0.03,
        )
        arrow_down = Arrow3D(
            start=np.array([arrow_x, 0.0, -0.05]),
            end=np.array([arrow_x, 0.0, -1.8]),
            color=YELLOW,
            thickness=0.03,
        )
        text = Text(
            "But even this representation just shows\n" \
            "a 2D cross section, or a wave on a liquid surface",
            font_size=30,
        )
        text.to_corner(DL, buff=0.35)
        self.add_fixed_in_frame_mobjects(text)

        self.play(
            # Create(arrow_up),
            # Create(arrow_down),
            FadeIn(text, shift=RIGHT * 0.2),
            # Keep the same 3D ripple speed used in the previous loop (1.2 phase units/sec).
            ripple_phase.animate.increment_value(1.68),
            run_time=1.4,
            rate_func=linear,
        )
        self.next_slide(loop=True)
        self.play(
            ripple_phase.animate.increment_value(12.0),
            run_time=10.0,
            rate_func=linear,
        )
        self.next_slide()


class ThreeDRippleCloud(ThreeDSlide):
    def construct(self):
        self.set_camera_orientation(phi=68 * DEGREES, theta=-38 * DEGREES, zoom=0.9)
        self.begin_ambient_camera_rotation(rate=0.08)

        center = Dot3D(point=ORIGIN, radius=0.05, color=YELLOW_E)
        self.add(center)

        shell_tracker = ValueTracker(0.0)
        max_radius = 3.4
        shell_spacing = 0.52
        shell_count = 7
        fade_start_radius = max_radius * 0.72
        fade_end_radius = max_radius + 1.15
        cycle_span = fade_end_radius + shell_spacing * (shell_count - 1)
        shell_colors = [TEAL_A, BLUE_D]

        shells = VGroup()
        for idx in range(shell_count):
            phase = idx * shell_spacing
            color = shell_colors[idx % 2]
            shell = Sphere(center=ORIGIN, radius=1e-3, resolution=(10, 20))

            def shell_updater(mob: Sphere, phase=phase, color=color) -> None:
                radius = shell_tracker.get_value() - phase
                if radius <= 0.0 or radius >= fade_end_radius:
                    mob.set_fill(opacity=0.0)
                    mob.set_stroke(opacity=0.0)
                    return

                base_opacity = 0.42 * max(0.18, 1.0 - 0.45 * (radius / max_radius))
                if radius <= fade_start_radius:
                    distance_fade = 1.0
                else:
                    distance_fade = max(
                        0.0,
                        1.0 - (radius - fade_start_radius) / (fade_end_radius - fade_start_radius),
                    )
                opacity = base_opacity * distance_fade
                mob.become(Sphere(center=ORIGIN, radius=radius, resolution=(10, 20)))
                mob.set_fill(color, opacity=opacity)
                mob.set_stroke(color, width=1.2, opacity=min(0.9, opacity * 1.8))

            shell.add_updater(shell_updater)
            shells.add(shell)

        self.add(shells)
        self.next_slide(loop=True)
        self.play(
            shell_tracker.animate.set_value(cycle_span),
            run_time=16.0,
            rate_func=linear,
        )
        self.stop_ambient_camera_rotation()
        self.next_slide()


class ThreeDAnimations(ThreeDSlide): 
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

        self.next_slide()



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




        self.next_slide(loop=True)




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



        

        self.next_slide()





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

        # Bottom perimeter path: front row (4) then right edge (3).
        left_indices = [0, 1, 2, 3]
        right_indices = [7, 11, 15]
        selected_indices = left_indices + right_indices
        selected_cubes = [cubes[idx] for idx in selected_indices]

        phases = np.linspace(0, 2 * np.pi, len(selected_cubes), endpoint=False)
        left_plots = [make_small_plot(phase) for phase in phases[: len(left_indices)]]
        right_plots = [make_small_plot(phase) for phase in phases[len(left_indices) :]]

        plot_step = 1.0

        left_start_y = plot_step * (len(left_plots) - 1) / 2
        left_plot_x = -self.camera.frame_width / 2 + 1.4
        for idx, plot in enumerate(left_plots):
            plot.move_to([left_plot_x, left_start_y - idx * plot_step, 0.0])

        right_start_y = plot_step * (len(right_plots) - 1) / 2
        right_plot_x = self.camera.frame_width / 2 - 1.4
        for idx, plot in enumerate(right_plots):
            plot.move_to([right_plot_x, right_start_y - idx * plot_step, 0.0])

        def fix_in_frame(mobjects: VGroup) -> None:
            self.camera.add_fixed_in_frame_mobjects(*mobjects)

        min_z = start
        max_z = start + (grid_size - 1) * step
        effector_top_z = max_z + 0.9

        plot_entries: list[tuple[Cube, VGroup, Literal["left", "right"]]] = (
            [(cube, plot, "left") for cube, plot in zip(selected_cubes[: len(left_indices)], left_plots)]
            + [(cube, plot, "right") for cube, plot in zip(selected_cubes[len(left_indices) :], right_plots)]
        )

        def add_plot_with_line(cube: Cube, plot: VGroup, side: Literal["left", "right"]) -> Arrow:
            start_corner = LEFT + DOWN + IN if side == "left" else RIGHT + DOWN + IN
            plot_anchor = plot.get_right() if side == "left" else plot.get_left()
            line = Arrow(
                self.camera.project_point(cube.get_corner(start_corner)),
                plot_anchor,
                buff=0,
                stroke_width=2,
            )
            fix_in_frame(VGroup(line, plot))
            return line

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

        first_cube, first_plot, first_side = plot_entries[0]
        first_line = add_plot_with_line(first_cube, first_plot, first_side)
        first_shift = LEFT * 0.2 if first_side == "left" else RIGHT * 0.2
        self.play(
            Create(effector),
            Create(first_line),
            FadeIn(first_plot, shift=first_shift),
            first_cube.animate.set_fill(highlight_fill_color),
            run_time=0.8,
        )

        for prev_entry, next_entry in zip(plot_entries, plot_entries[1:]):
            prev_cube, _, _ = prev_entry
            next_cube, next_plot, next_side = next_entry
            segment_start[0] = prev_cube.get_center()
            segment_end[0] = next_cube.get_center()
            move_tracker.set_value(0.0)
            next_line = add_plot_with_line(next_cube, next_plot, next_side)
            next_shift = LEFT * 0.2 if next_side == "left" else RIGHT * 0.2
            self.play(
                move_tracker.animate.set_value(1.0),
                Create(next_line),
                FadeIn(next_plot, shift=next_shift),
                prev_cube.animate.set_fill(cube_color),
                next_cube.animate.set_fill(highlight_fill_color),
                run_time=1.4,
            )
            self.wait(0.2)
