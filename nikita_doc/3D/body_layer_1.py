#!/usr/bin/env python3

import math
from pathlib import Path
from typing import NamedTuple

from build123d import (
    BuildPart,
    BuildLine,
    BuildSketch,
    Circle,
    Edge,
    ExportDXF,
    Line,
    Locations,
    Mode,
    Part,
    Polygon,
    Pos,
    Rectangle,
    Sketch,
    Sphere,
    add,
    export_step,
    extrude,
)

from ocp_utils import show
from servo_cutouts import SERVO_BACK_CUTOUT, SERVO_BRACKET_HOLES 
from geometry_utils import (
    mirror_circle_holes_horizontal_axis as mirror_holes_horizontal_axis,
    mirror_circle_holes_vertical_axis as mirror_holes_x_axis,
    mirror_points_horizontal_axis as mirror_points_x_axis,
    mirror_slots_vertical_axis as mirror_slots_x_axis,
)

THICKNESS = 1.0
AXIS_X_TOP_BOTTOM = 0.221
AXIS_X_CENTER = -2.779


class ServoBackCutoutOutlineConfig(NamedTuple):
    start_index: int
    rotation_deg_clockwise: float
    offset_x: float
    offset_y: float


SERVO_BACK_CUTOUT_OUTLINE_CONFIGS = [
    # left cutout
    ServoBackCutoutOutlineConfig(
        start_index=7,
        rotation_deg_clockwise=-45.0,
        offset_x=-94.194,
        offset_y=54.473,
    ),
    # center cutout
    ServoBackCutoutOutlineConfig(
        start_index=7,
        rotation_deg_clockwise=0.0,
        offset_x=0.0,
        offset_y=67.750,
    ),
    # right cutout
    ServoBackCutoutOutlineConfig(
        start_index=7,
        rotation_deg_clockwise=45.0,
        offset_x=94.949,
        offset_y=54.161,
    ),
]

# Set an explicit point index, or leave None and use DEBUG_POINT_MARKER.
DEBUG_POINT_INDEX: int | None = None
DEBUG_POINT_MARKER: str | None = "Marker"

def project_point_to_x_axis(point: tuple[float, float]) -> tuple[float, float]:
    """Project a 2D point onto the x-axis while preserving its x coordinate."""
    x, _ = point
    return (x, 0.0)


def shift_points(
    points: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    """Shift 2D points by a constant offset."""
    return [(x + dx, y + dy) for x, y in points]


def rotate_point_sequence(
    points: list[tuple[float, float]], start_index: int
) -> list[tuple[float, float]]:
    """Rotate a point sequence so it starts at start_index."""
    return points[start_index:] + points[:start_index]


def rotate_points_clockwise(
    points: list[tuple[float, float]], degrees: float
) -> list[tuple[float, float]]:
    """Rotate 2D points around the origin by degrees clockwise."""
    radians = math.radians(-degrees)
    cosine = math.cos(radians)
    sine = math.sin(radians)
    return [
        (x * cosine - y * sine, x * sine + y * cosine)
        for x, y in points
    ]


def transform_servo_back_cutout(
    config: ServoBackCutoutOutlineConfig,
) -> list[tuple[float, float]]:
    points = rotate_point_sequence(SERVO_BACK_CUTOUT, config.start_index)
    points = rotate_points_clockwise(points, config.rotation_deg_clockwise)
    return shift_points(points, config.offset_x, config.offset_y)


def transform_servo_bracket_holes(
    config: ServoBackCutoutOutlineConfig,
) -> list[tuple[float, float, float]]:
    hole_centers = [(x, y) for x, y, _ in SERVO_BRACKET_HOLES]
    hole_centers = rotate_points_clockwise(hole_centers, config.rotation_deg_clockwise)
    hole_centers = shift_points(hole_centers, config.offset_x, config.offset_y)
    radii = [radius for _, _, radius in SERVO_BRACKET_HOLES]
    return [
        (x, y, radius)
        for (x, y), radius in zip(hole_centers, radii, strict=True)
    ]


(
    SERVO_BACK_CUTOUT_LEFT_FOR_OUTLINE,
    SERVO_BACK_CUTOUT_FOR_OUTLINE,
    SERVO_BACK_CUTOUT_RIGHT_FOR_OUTLINE,
) = [
    transform_servo_back_cutout(config)
    for config in SERVO_BACK_CUTOUT_OUTLINE_CONFIGS
]

SERVO_BRACKET_HOLES_BASE_FOR_OUTLINE = [
    hole
    for config in SERVO_BACK_CUTOUT_OUTLINE_CONFIGS
    for hole in transform_servo_bracket_holes(config)
]

SERVO_BRACKET_HOLES_FOR_OUTLINE = (
    SERVO_BRACKET_HOLES_BASE_FOR_OUTLINE
    + mirror_holes_horizontal_axis(
        SERVO_BRACKET_HOLES_BASE_FOR_OUTLINE, AXIS_X_TOP_BOTTOM
    )
)


# Coordinates are centered at the drawing midpoint.
OUTER_UPPER_RAW_POINTS = [
    (None, (-107.962, 22.674)),
    (None, (-115.033, 29.745)),
    (None, (-115.033, 55.201)),
    *[(None, point) for point in SERVO_BACK_CUTOUT_LEFT_FOR_OUTLINE],
    (None, (-95.235, 75.000)),
    (None, (-69.779, 75.000)),
    (None, (-59.779, 65.000)),
    (None, (-31.779, 65.000)),
    (None, (-13.779, 83.000)),
    *[(None, point) for point in SERVO_BACK_CUTOUT_FOR_OUTLINE],
    (None, (14.221, 83.000)),
    (None, (32.221, 65.000)),
    (None, (60.221, 65.000)),
    (None, (70.221, 75.000)),
    (None, (95.677, 75.000)),
    *[(None, point) for point in SERVO_BACK_CUTOUT_RIGHT_FOR_OUTLINE],
    ("Marker", (115.221, 55.000)),
    (None, (115.221, 31.000)),
    (None, (132.221, 14.000)),
]

OUTER_UPPER: dict[int, tuple[str | None, tuple[float, float]]] = {}
for index, point_data in enumerate(OUTER_UPPER_RAW_POINTS):
    OUTER_UPPER[index] = point_data

OUTER_UPPER_POINTS = [OUTER_UPPER[index][1] for index in sorted(OUTER_UPPER)]

OUTER_POINTS = OUTER_UPPER_POINTS + mirror_points_x_axis(
    list(reversed(OUTER_UPPER_POINTS))
)

INNER_UPPER = [
    (90.221, 22.000),
    (59.221, 53.000),
    (24.221, 53.000),
    (18.221, 47.000),
    (-17.779, 47.000),
    (-23.779, 53.000),
    (-59.779, 53.000),
    (-95.779, 17.000),
]

INNER_START = project_point_to_x_axis(INNER_UPPER[0])
INNER_END = project_point_to_x_axis(INNER_UPPER[-1])

INNER_POINTS = (
    [INNER_START]
    + INNER_UPPER
    + [INNER_END]
    + mirror_points_x_axis(list(reversed(INNER_UPPER)))
)

SMALL_HOLES = SERVO_BRACKET_HOLES_FOR_OUTLINE

LARGE_HOLES_RIGHT_TOP_BOTTOM = [
    (46.221, -60.000, 1.500),
    (46.221, 60.000, 1.500),
]

LARGE_HOLES_RIGHT_CENTER = [
    (97.221, -15.500, 1.500),
    (97.221, 15.500, 1.500),
]

LARGE_HOLES = (
    mirror_holes_x_axis(LARGE_HOLES_RIGHT_TOP_BOTTOM, AXIS_X_TOP_BOTTOM)
    + LARGE_HOLES_RIGHT_TOP_BOTTOM
    + mirror_holes_x_axis(LARGE_HOLES_RIGHT_CENTER, AXIS_X_CENTER)
    + LARGE_HOLES_RIGHT_CENTER
)

SLOT_HOLES_ROW_RIGHT = [
    (22.221, -56.750, 8.000, 1.500),
    (38.221, -56.750, 8.000, 1.500),
    (54.221, -56.750, 8.000, 1.500),
    (22.221, 56.750, 8.000, 1.500),
    (38.221, 56.750, 8.000, 1.500),
    (54.221, 56.750, 8.000, 1.500),
]

SLOT_HOLES_CENTER_RIGHT = [
    (93.971, -16.000, 1.500, 8.000),
    (93.971, 0.000, 1.500, 8.000),
    (93.971, 16.000, 1.500, 8.000),
]

SLOT_HOLES = (
    mirror_slots_x_axis(SLOT_HOLES_ROW_RIGHT, AXIS_X_TOP_BOTTOM)
    + SLOT_HOLES_ROW_RIGHT
    + mirror_slots_x_axis(SLOT_HOLES_CENTER_RIGHT, AXIS_X_CENTER)
    + SLOT_HOLES_CENTER_RIGHT
)

def build_debug_edge_from_point(point_index: int) -> Edge:
    point_indices = list(OUTER_UPPER)
    if point_index not in OUTER_UPPER:
        raise ValueError(
            f"Point index must be one of {point_indices[0]}..{point_indices[-1]}"
        )

    next_pos = (point_indices.index(point_index) + 1) % len(point_indices)
    next_index = point_indices[next_pos]

    x1, y1 = OUTER_UPPER[point_index][1]
    x2, y2 = OUTER_UPPER[next_index][1]

    with BuildLine() as edge:
        Line((x1, y1, 0), (x2, y2, 0))

    return edge.edges()[0]


def resolve_debug_point_index() -> int | None:
    if DEBUG_POINT_INDEX is not None:
        return DEBUG_POINT_INDEX

    if DEBUG_POINT_MARKER is None:
        return None

    for index in sorted(OUTER_UPPER):
        marker, _ = OUTER_UPPER[index]
        if marker == DEBUG_POINT_MARKER:
            return index

    raise ValueError(f"No OUTER_UPPER point found for marker '{DEBUG_POINT_MARKER}'")

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Polygon(*OUTER_POINTS)
        Polygon(*INNER_POINTS, mode=Mode.SUBTRACT)

        for x, y, radius in SMALL_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)
                
        for x, y, radius in LARGE_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for x, y, width, height in SLOT_HOLES:
            with Locations((x, y)):
                Rectangle(width, height, mode=Mode.SUBTRACT)


    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/body_layer_1.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/body_layer_1.dxf")

    show(result, name="body_layer_1", clear=True)

    debug_point_index = resolve_debug_point_index()
    if debug_point_index is not None:
        edge = build_debug_edge_from_point(debug_point_index)
        x, y = OUTER_UPPER[debug_point_index][1]
        marker = Pos(x, y, THICKNESS + 0.4) * Sphere(0.5)
        show(
            edge,
            name=f"outline_edge_{debug_point_index}",
            clear=False,
            options={"color": "red"},
        )
        show(
            marker,
            name=f"outline_point_{debug_point_index}",
            clear=False,
            options={"color": "red"},
        )
        
if __name__ == "__main__":
    main()
