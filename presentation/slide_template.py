from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal
import html
import re

from manim import *
from manim_slides import Slide


@dataclass
class SlideImageSpec:
    """Single image settings. Combine multiple instances into one image group."""
    path: Path
    layout: Literal["left", "right", "center", "background"] = "right"
    # Used only when group-level uniform sizing is disabled.
    width_ratio: float = 0.35
    # Rotation in degrees, applied after scaling.
    rotation_degrees: float | None = None
    opacity: float = 1.0
    animation: Literal["none", "fade", "grow"] = "fade"


@dataclass
class TextBlockSpec:
    """One independently positioned and styled markdown text block."""
    markdown: str
    horizontal_placement: Literal["left", "center", "right"] = "left"
    vertical_placement: Literal["top", "center"] = "top"
    text_alignment: Literal["left", "center", "right"] = "left"
    h1_font_size: int = 58
    h2_font_size: int = 40
    h3_font_size: int = 34
    body_font_size: int = 30
    bullet_font_size: int = 30
    sub_bullet_opacity: float = 0.9
    bullet_indent: float = 0.45
    line_spacing: float = 0.22
    block_spacing: float = 0.35
    animation: Literal["none", "fade", "stagger"] = "fade"
    fade_run_time: float = 1.0


@dataclass
class TextSlideSpec:
    """Slide spec with one or more text blocks plus grouped image controls."""
    text_blocks: list[TextBlockSpec] = field(default_factory=list)
    # Image groups. Each inner list is one independently controlled group.
    images: list[list[SlideImageSpec]] = field(default_factory=list)
    # Per-group layout. Defaults to first image layout in each group when omitted.
    image_group_layouts: list[Literal["left", "right", "center", "background"]] = field(
        default_factory=list
    )
    # Per-group arrangement direction for images inside the group.
    image_group_directions: list[Literal["horizontal", "vertical"]] = field(
        default_factory=list
    )
    # Per-group spacing between images inside a group.
    image_group_spacings: list[float] = field(default_factory=list)
    # Per-group x,y offset in scene units, applied after base placement.
    image_group_offsets: list[tuple[float, float]] = field(default_factory=list)
    # Per-group absolute slide placement anchor. If omitted, group participates in text flow.
    image_group_positions: list[
        Literal[
            "none",
            "center",
            "top",
            "bottom",
            "left",
            "right",
            "top_left",
            "top_right",
            "bottom_left",
            "bottom_right",
        ]
    ] = field(default_factory=list)
    # Per-group edge/corner buffer for anchored placement.
    image_group_position_buffs: list[float] = field(default_factory=list)
    # Group-level sizing options. Can be a single value or a per-group list.
    uniform_image_size: bool | list[bool] = True
    image_size_ratio: float | list[float] = 0.35
    image_max_height_ratio: float | list[float] = 0.75
    # Group-level default rotation (degrees), overridden by per-image rotation.
    image_rotation_degrees: float | list[float] = 0.0
    image_run_time: float = 0.5


@dataclass
class _MarkdownBlock:
    kind: Literal["h1", "h2", "h3", "paragraph", "bullet", "spacer"]
    text: str
    depth: int = 0


@dataclass
class _RenderedImageGroup:
    layout: Literal["left", "right", "center", "background"]
    position: Literal[
        "none",
        "center",
        "top",
        "bottom",
        "left",
        "right",
        "top_left",
        "top_right",
        "bottom_left",
        "bottom_right",
    ]
    position_buff: float
    mob: Mobject
    images: list[tuple[Mobject, SlideImageSpec]]


@dataclass
class _RenderedTextBlock:
    spec: TextBlockSpec
    group: VGroup
    lines: list[Mobject]


class IntroTextTemplate(Slide):
    SPEC = TextSlideSpec(
        text_blocks=[
            TextBlockSpec(
                markdown=(
                    "# Project Overview\n"
                    "Template slide for introducing a section or talk.\n\n"
                    "- Summarize the context in one sentence.\n"
                    "- State the objective and expected outcome.\n"
                    "  - Define success criteria.\n"
                    "  - Highlight constraints and risks.\n"
                    "- Outline the key points for this section."
                ),
                animation="fade",
            )
        ],
    )

    _BR_TOKEN = "%%BR%%"

    def _inject_br_tokens(self, text: str) -> str:
        return re.sub(r"<br\s*/?>", self._BR_TOKEN, text, flags=re.IGNORECASE)

    def _markdown_to_markup(self, text: str) -> str:
        text = self._inject_br_tokens(text)
        escaped = html.escape(text)
        escaped = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", escaped)
        escaped = re.sub(r"__(.+?)__", r"<b>\1</b>", escaped)
        escaped = re.sub(r"(?<!\*)\*(?!\*)(.+?)(?<!\*)\*(?!\*)", r"<i>\1</i>", escaped)
        escaped = re.sub(r"(?<!_)_(?!_)(.+?)(?<!_)_(?!_)", r"<i>\1</i>", escaped)
        escaped = escaped.replace(self._BR_TOKEN, "\n")
        return escaped

    def _parse_markdown(self, markdown: str) -> list[_MarkdownBlock]:
        markdown = self._inject_br_tokens(markdown)
        blocks: list[_MarkdownBlock] = []
        paragraph_lines: list[str] = []

        def flush_paragraph() -> None:
            if paragraph_lines:
                blocks.append(_MarkdownBlock("paragraph", " ".join(paragraph_lines)))
                paragraph_lines.clear()

        def process_logical_line(line: str) -> None:
            stripped = line.strip()
            if not stripped:
                flush_paragraph()
                return

            heading_match = re.match(r"^(#{1,3})\s+(.*)$", stripped)
            if heading_match:
                flush_paragraph()
                level = len(heading_match.group(1))
                text = heading_match.group(2).strip()
                kind: Literal["h1", "h2", "h3"] = "h1" if level == 1 else "h2" if level == 2 else "h3"
                blocks.append(_MarkdownBlock(kind, text))
                return

            bullet_match = re.match(r"^(\s*)[-*+]\s+(.*)$", line)
            if bullet_match:
                flush_paragraph()
                indent = len(bullet_match.group(1).replace("\t", "  "))
                depth = indent // 2
                text = bullet_match.group(2).strip()
                blocks.append(_MarkdownBlock("bullet", text, depth=depth))
                return

            paragraph_lines.append(stripped)

        for raw_line in markdown.splitlines():
            line = raw_line.rstrip()
            segments = line.split(self._BR_TOKEN)
            for idx, segment in enumerate(segments):
                process_logical_line(segment)
                if idx < len(segments) - 1:
                    flush_paragraph()
                    blocks.append(_MarkdownBlock("spacer", "", depth=1))

        flush_paragraph()
        return blocks

    def _resolve_image(self, image_path: Path) -> Path:
        path = image_path if image_path.is_absolute() else Path(__file__).parent / image_path
        if not path.exists():
            raise FileNotFoundError(f"Image for IntroTextTemplate does not exist: {path}")
        return path

    def _aligned_edge(self, alignment: Literal["left", "center", "right"]):
        if alignment == "left":
            return LEFT
        if alignment == "right":
            return RIGHT
        return ORIGIN

    def _position_content(self, content: Mobject, spec: TextBlockSpec) -> None:
        if spec.horizontal_placement == "left":
            target_x = -self.camera.frame_width / 2 + content.width / 2 + 0.8
        elif spec.horizontal_placement == "right":
            target_x = self.camera.frame_width / 2 - content.width / 2 - 0.8
        else:
            target_x = 0.0

        if spec.vertical_placement == "top":
            target_y = self.camera.frame_height / 2 - content.height / 2 - 0.7
        else:
            target_y = 0.0

        content.move_to([target_x, target_y, 0.0])

    def _place_group_on_slide(
        self,
        group: Mobject,
        position: Literal[
            "none",
            "center",
            "top",
            "bottom",
            "left",
            "right",
            "top_left",
            "top_right",
            "bottom_left",
            "bottom_right",
        ],
        buff: float,
    ) -> None:
        if position == "none":
            return
        if position == "center":
            group.move_to(ORIGIN)
        elif position == "top":
            group.to_edge(UP, buff=buff)
        elif position == "bottom":
            group.to_edge(DOWN, buff=buff)
        elif position == "left":
            group.to_edge(LEFT, buff=buff)
        elif position == "right":
            group.to_edge(RIGHT, buff=buff)
        elif position == "top_left":
            group.to_corner(UL, buff=buff)
        elif position == "top_right":
            group.to_corner(UR, buff=buff)
        elif position == "bottom_left":
            group.to_corner(DL, buff=buff)
        elif position == "bottom_right":
            group.to_corner(DR, buff=buff)

    def _build_text_group(
        self, spec: TextBlockSpec
    ) -> tuple[VGroup, list[Mobject]]:
        blocks = self._parse_markdown(spec.markdown)
        lines: list[Mobject] = []
        bullet_depths: list[int] = []

        for block in blocks:
            markup = self._markdown_to_markup(block.text)
            if block.kind == "h1":
                mob = MarkupText(f"<b>{markup}</b>", font_size=spec.h1_font_size)
            elif block.kind == "h2":
                mob = MarkupText(f"<b>{markup}</b>", font_size=spec.h2_font_size)
                mob.set_opacity(0.8)
            elif block.kind == "h3":
                mob = MarkupText(markup, font_size=spec.h3_font_size)
                mob.set_opacity(0.8)
            elif block.kind == "bullet":
                marker = "•" if block.depth == 0 else "◦"
                mob = MarkupText(
                    f"{marker} {markup}",
                    font_size=max(spec.bullet_font_size - 2 * block.depth, 18),
                )
                if block.depth > 0:
                    mob.set_opacity(spec.sub_bullet_opacity)
                bullet_depths.append(block.depth)
            elif block.kind == "spacer":
                base_line_height = MarkupText("Ag", font_size=spec.body_font_size).height
                mob = Rectangle(
                    width=1e-3,
                    height=base_line_height * max(block.depth, 1),
                    stroke_opacity=0.0,
                    fill_opacity=0.0,
                )
            else:
                mob = MarkupText(markup, font_size=spec.body_font_size)
            lines.append(mob)

        if not lines:
            placeholder = Text("")
            return VGroup(placeholder), [placeholder]

        text_group = VGroup(*lines).arrange(
            DOWN,
            aligned_edge=self._aligned_edge(spec.text_alignment),
            buff=spec.line_spacing,
        )

        bullet_idx = 0
        for mob, block in zip(lines, blocks):
            if block.kind == "bullet":
                depth = bullet_depths[bullet_idx]
                mob.shift(RIGHT * (spec.bullet_indent * depth))
                bullet_idx += 1

        return text_group, lines

    def _build_image_groups(
        self, spec: TextSlideSpec
    ) -> list[_RenderedImageGroup]:
        def group_value(value, group_idx: int):
            if isinstance(value, list):
                if not value:
                    raise ValueError("Per-group image option list cannot be empty.")
                return value[group_idx] if group_idx < len(value) else value[-1]
            return value

        rendered_groups: list[_RenderedImageGroup] = []
        for group_idx, image_specs in enumerate(spec.images):
            if not image_specs:
                continue

            group_layout = (
                spec.image_group_layouts[group_idx]
                if group_idx < len(spec.image_group_layouts)
                else image_specs[0].layout
            )
            group_direction = (
                spec.image_group_directions[group_idx]
                if group_idx < len(spec.image_group_directions)
                else ("vertical" if group_layout in ("left", "right") else "horizontal")
            )
            group_spacing = (
                spec.image_group_spacings[group_idx]
                if group_idx < len(spec.image_group_spacings)
                else 0.25
            )
            group_offset = (
                spec.image_group_offsets[group_idx]
                if group_idx < len(spec.image_group_offsets)
                else (0.0, 0.0)
            )
            group_position = (
                spec.image_group_positions[group_idx]
                if group_idx < len(spec.image_group_positions)
                else "none"
            )
            group_position_buff = (
                spec.image_group_position_buffs[group_idx]
                if group_idx < len(spec.image_group_position_buffs)
                else 0.5
            )
            uniform_size = bool(group_value(spec.uniform_image_size, group_idx))
            size_ratio = float(group_value(spec.image_size_ratio, group_idx))
            max_height_ratio = float(group_value(spec.image_max_height_ratio, group_idx))
            default_rotation = float(group_value(spec.image_rotation_degrees, group_idx))

            rendered_images: list[tuple[Mobject, SlideImageSpec]] = []
            for image_spec in image_specs:
                resolved = self._resolve_image(image_spec.path)
                image = ImageMobject(str(resolved))
                image.set_opacity(image_spec.opacity)

                rotation_degrees = (
                    default_rotation
                    if image_spec.rotation_degrees is None
                    else image_spec.rotation_degrees
                )

                if group_layout == "background":
                    image.scale_to_fit_width(self.camera.frame_width)
                    image.scale_to_fit_height(self.camera.frame_height)
                else:
                    width_ratio = size_ratio if uniform_size else image_spec.width_ratio
                    image.scale_to_fit_width(self.camera.frame_width * width_ratio)
                    image.scale_to_fit_height(
                        self.camera.frame_height * max_height_ratio
                    )

                if rotation_degrees:
                    image.rotate(rotation_degrees * DEGREES)

                rendered_images.append((image, image_spec))

            group_mob = Group(*[mob for mob, _ in rendered_images])
            if group_layout != "background" and len(rendered_images) > 1:
                arrange_direction = RIGHT if group_direction == "horizontal" else DOWN
                group_mob.arrange(arrange_direction, buff=group_spacing)

            if group_offset != (0.0, 0.0):
                group_mob.shift(RIGHT * group_offset[0] + UP * group_offset[1])

            rendered_groups.append(
                _RenderedImageGroup(
                    layout=group_layout,
                    position=group_position,
                    position_buff=group_position_buff,
                    mob=group_mob,
                    images=rendered_images,
                )
            )

        return rendered_groups

    def _image_animation(self, mob: Mobject, image_spec: SlideImageSpec):
        if image_spec.animation == "none":
            return None
        if image_spec.animation == "grow":
            return GrowFromCenter(mob)
        return FadeIn(mob)

    def construct(self):
        spec = self.SPEC
        rendered_text_blocks: list[_RenderedTextBlock] = []
        for text_block_spec in spec.text_blocks:
            text_group, text_lines = self._build_text_group(text_block_spec)
            rendered_text_blocks.append(
                _RenderedTextBlock(
                    spec=text_block_spec,
                    group=text_group,
                    lines=text_lines,
                )
            )

        primary_text_block = rendered_text_blocks[0] if rendered_text_blocks else None
        secondary_text_blocks = rendered_text_blocks[1:] if len(rendered_text_blocks) > 1 else []
        image_groups = self._build_image_groups(spec)

        anchored_groups = [group for group in image_groups if group.position != "none"]
        flow_groups = [group for group in image_groups if group.position == "none"]

        left_groups = [group for group in flow_groups if group.layout == "left"]
        right_groups = [group for group in flow_groups if group.layout == "right"]
        center_groups = [group for group in flow_groups if group.layout == "center"]
        bg_groups = [group for group in image_groups if group.layout == "background"]

        left_group = Group(*[group.mob for group in left_groups]) if left_groups else None
        right_group = Group(*[group.mob for group in right_groups]) if right_groups else None
        center_group = Group(*[group.mob for group in center_groups]) if center_groups else None
        flow_spacing = (
            primary_text_block.spec.block_spacing if primary_text_block is not None else 0.35
        )
        flow_alignment = (
            primary_text_block.spec.text_alignment if primary_text_block is not None else "left"
        )

        if left_group is not None and len(left_groups) > 1:
            left_group.arrange(DOWN, buff=flow_spacing)
        if right_group is not None and len(right_groups) > 1:
            right_group.arrange(DOWN, buff=flow_spacing)
        if center_group is not None and len(center_groups) > 1:
            center_group.arrange(DOWN, buff=flow_spacing)

        row_items: list[Mobject] = []
        if left_group is not None:
            row_items.append(left_group)
        if primary_text_block is not None:
            row_items.append(primary_text_block.group)
        if right_group is not None:
            row_items.append(right_group)

        if not row_items:
            main_row = None
        elif len(row_items) > 1:
            main_row = Group(*row_items).arrange(RIGHT, aligned_edge=UP, buff=0.8)
        else:
            main_row = row_items[0]

        content = main_row
        if center_group is not None:
            if content is None:
                content = center_group
            else:
                content = Group(content, center_group).arrange(
                    DOWN,
                    aligned_edge=self._aligned_edge(flow_alignment),
                    buff=flow_spacing,
                )

        max_width = self.camera.frame_width - 1.2
        max_height = self.camera.frame_height - 1.2
        if content is not None:
            if content.width > max_width:
                content.scale_to_fit_width(max_width)
            if content.height > max_height:
                content.scale_to_fit_height(max_height)

            if primary_text_block is not None:
                self._position_content(content, primary_text_block.spec)
            else:
                content.move_to(ORIGIN)

        for rendered in secondary_text_blocks:
            if rendered.group.width > max_width:
                rendered.group.scale_to_fit_width(max_width)
            if rendered.group.height > max_height:
                rendered.group.scale_to_fit_height(max_height)
            self._position_content(rendered.group, rendered.spec)

        for anchored_group in anchored_groups:
            self._place_group_on_slide(
                anchored_group.mob,
                anchored_group.position,
                anchored_group.position_buff,
            )

        for bg_group in bg_groups:
            for bg_mob, bg_spec in bg_group.images:
                anim = self._image_animation(bg_mob, bg_spec)
                if anim is None:
                    self.add(bg_mob)
                else:
                    self.play(anim, run_time=spec.image_run_time)

        foreground_images = [
            image
            for group in (left_groups + right_groups + center_groups + anchored_groups)
            for image in group.images
        ]

        for rendered in rendered_text_blocks:
            if rendered.spec.animation == "none":
                self.add(rendered.group)
            elif rendered.spec.animation == "stagger":
                self.play(
                    LaggedStart(
                        *[FadeIn(line, shift=DOWN * 0.05) for line in rendered.lines],
                        lag_ratio=0.12,
                    ),
                    run_time=rendered.spec.fade_run_time,
                )
            else:
                self.play(FadeIn(rendered.group), run_time=rendered.spec.fade_run_time)

        image_anims = []
        for mob, image_spec in foreground_images:
            anim = self._image_animation(mob, image_spec)
            if anim is None:
                self.add(mob)
            else:
                image_anims.append(anim)
        if image_anims:
            self.play(LaggedStart(*image_anims, lag_ratio=0.1), run_time=spec.image_run_time)

        self.next_slide()
