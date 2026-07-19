#!/usr/bin/env python3

import math
from pathlib import Path
from typing import NamedTuple

from build123d import *
from ocp_utils import show
from servo_cutouts import SERVO_BACK_CUTOUT, SERVO_FRONT_CUTOUT, SERVO_BRACKET_HOLES
from geometry_utils import (
    mirror_circle_holes_vertical_axis as mirror_holes_x_axis,
    mirror_points_horizontal_axis as mirror_points_x_axis,
    mirror_slots_vertical_axis as mirror_slots_x_axis,
)

THICKNESS = 1.0
AXIS_X_TOP_BOTTOM = 0.221
AXIS_X_CENTER = -2.779


class ServoCutoutConfig(NamedTuple):
    start_index: int
    rotation_deg_clockwise: float
    offset_x: float
    offset_y: float



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
    config: ServoCutoutConfig,
) -> list[tuple[float, float]]:
    points = rotate_point_sequence(SERVO_BACK_CUTOUT, config.start_index)
    points = rotate_points_clockwise(points, config.rotation_deg_clockwise)
    return shift_points(points, config.offset_x, config.offset_y)

def transform_servo_front_cutout(
    config: ServoCutoutConfig,
) -> list[tuple[float, float]]:
    points = rotate_point_sequence(SERVO_FRONT_CUTOUT, config.start_index)
    points = rotate_points_clockwise(points, config.rotation_deg_clockwise)
    return shift_points(points, config.offset_x, config.offset_y)


def transform_servo_bracket_holes(
    config: ServoCutoutConfig,
) -> list[tuple[float, float, float]]:
    hole_centers = [(x, y) for x, y, _ in SERVO_BRACKET_HOLES]
    hole_centers = rotate_points_clockwise(hole_centers, config.rotation_deg_clockwise)
    hole_centers = shift_points(hole_centers, config.offset_x, config.offset_y)
    radii = [radius for _, _, radius in SERVO_BRACKET_HOLES]
    return [
        (x, y, radius)
        for (x, y), radius in zip(hole_centers, radii, strict=True)
    ]


def octagon_points(center, radius, rotation_deg=0, n=8):
    cx, cy = center
    angle_offset = math.radians(rotation_deg)
    pts = []
    for i in range(n):
        theta = 2 * math.pi * i / n + angle_offset
        pts.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta)))
    return pts


octagon_size = 40

# Rechteck-Maße
rect_w = 220
rect_h = 130
half_w = (rect_w / 2) - (octagon_size/2)
half_h = rect_h / 2 - (octagon_size/2)

diag_offset = 6.0
linear_offset = 4.5

octagon_position_left_top = (-half_w + diag_offset, half_h - diag_offset)
octagon_position_right_top = (half_w - diag_offset, half_h - diag_offset)
octagon_position_right_bottom = (half_w - diag_offset, -half_h + diag_offset)
octagon_position_left_bottom = (-half_w + diag_offset, -half_h + diag_offset)
octagon_position_top_middle = (0, half_h + linear_offset)
octagon_position_right_middle = (half_w + linear_offset, 0)
octagon_position_bottom_middle = (0, -half_h - linear_offset)

servo_cutout_diag_offset = 15.5
servo_cutout_linear_offset = 22.0

servo_cutout_config_left_top = ServoCutoutConfig(
    start_index=7,
    rotation_deg_clockwise=-45.0,
    offset_x=octagon_position_left_top[0] - servo_cutout_diag_offset,
    offset_y=octagon_position_left_top[1] + servo_cutout_diag_offset,
)
servo_cutout_config_right_top = ServoCutoutConfig(
    start_index=7,
    rotation_deg_clockwise=45.0,
    offset_x=octagon_position_right_top[0] + servo_cutout_diag_offset,
    offset_y=octagon_position_right_top[1] + servo_cutout_diag_offset,
)
servo_cutout_config_right_bottom = ServoCutoutConfig(
    start_index=7,
    rotation_deg_clockwise=135.0,
    offset_x=octagon_position_right_bottom[0] + servo_cutout_diag_offset,
    offset_y=octagon_position_right_bottom[1] - servo_cutout_diag_offset,
)
servo_cutout_config_left_bottom = ServoCutoutConfig(
    start_index=7,
    rotation_deg_clockwise=-135.0,
    offset_x=octagon_position_left_bottom[0] - servo_cutout_diag_offset,
    offset_y=octagon_position_left_bottom[1] - servo_cutout_diag_offset,
)
servo_cutout_config_top_middle = ServoCutoutConfig(
    start_index=7,
    rotation_deg_clockwise=0.0,
    offset_x=octagon_position_top_middle[0],
    offset_y=octagon_position_top_middle[1] + servo_cutout_linear_offset,
)
servo_cutout_config_right_middle = ServoCutoutConfig(
    start_index=7,
    rotation_deg_clockwise=90.0,
    offset_x=octagon_position_right_middle[0] + servo_cutout_linear_offset,
    offset_y=octagon_position_right_middle[1],
)
servo_cutout_config_bottom_middle = ServoCutoutConfig(
    start_index=7,
    rotation_deg_clockwise=180.0,
    offset_x=octagon_position_bottom_middle[0],
    offset_y=octagon_position_bottom_middle[1] - servo_cutout_linear_offset,
)

SERVO_BACK_CUTOUT_LEFT_UP = transform_servo_back_cutout(servo_cutout_config_left_top)
SERVO_BACK_CUTOUT_CENTER_UP = transform_servo_back_cutout(servo_cutout_config_top_middle)
SERVO_BACK_CUTOUT_RIGHT_UP = transform_servo_back_cutout(servo_cutout_config_right_top)
SERVO_BACK_CUTOUT_LEFT_DOWN = transform_servo_back_cutout(servo_cutout_config_left_bottom)
SERVO_BACK_CUTOUT_CENTER_DOWN = transform_servo_back_cutout(servo_cutout_config_bottom_middle)
SERVO_BACK_CUTOUT_RIGHT_DOWN = transform_servo_back_cutout(servo_cutout_config_right_bottom)
SERVO_BACK_CUTOUT_HEAD = transform_servo_back_cutout(servo_cutout_config_right_middle)

SERVO_FRONT_CUTOUT_LEFT_UP = transform_servo_front_cutout(servo_cutout_config_left_top)
SERVO_FRONT_CUTOUT_CENTER_UP = transform_servo_front_cutout(servo_cutout_config_top_middle)
SERVO_FRONT_CUTOUT_RIGHT_UP = transform_servo_front_cutout(servo_cutout_config_right_top)   
SERVO_FRONT_CUTOUT_LEFT_DOWN = transform_servo_front_cutout(servo_cutout_config_left_bottom)
SERVO_FRONT_CUTOUT_CENTER_DOWN = transform_servo_front_cutout(servo_cutout_config_bottom_middle)
SERVO_FRONT_CUTOUT_RIGHT_DOWN = transform_servo_front_cutout(servo_cutout_config_right_bottom)
SERVO_FRONT_CUTOUT_HEAD = transform_servo_front_cutout(servo_cutout_config_right_middle)

SERVO_BRACKET_HOLES_LEFT_UP = transform_servo_bracket_holes(servo_cutout_config_left_top)
SERVO_BRACKET_HOLES_CENTER_UP = transform_servo_bracket_holes(servo_cutout_config_top_middle)
SERVO_BRACKET_HOLES_RIGHT_UP = transform_servo_bracket_holes(servo_cutout_config_right_top)
SERVO_BRACKET_HOLES_LEFT_DOWN = transform_servo_bracket_holes(servo_cutout_config_left_bottom)
SERVO_BRACKET_HOLES_CENTER_DOWN = transform_servo_bracket_holes(servo_cutout_config_bottom_middle)
SERVO_BRACKET_HOLES_RIGHT_DOWN = transform_servo_bracket_holes(servo_cutout_config_right_bottom)
SERVO_BRACKET_HOLES_HEAD = transform_servo_bracket_holes(servo_cutout_config_right_middle)

LIST_BRACKET_HOLES = (
    SERVO_BRACKET_HOLES_LEFT_UP
    + SERVO_BRACKET_HOLES_CENTER_UP
    + SERVO_BRACKET_HOLES_RIGHT_UP
    + SERVO_BRACKET_HOLES_LEFT_DOWN
    + SERVO_BRACKET_HOLES_CENTER_DOWN
    + SERVO_BRACKET_HOLES_RIGHT_DOWN
    + SERVO_BRACKET_HOLES_HEAD
)


# Positionen + jeweils EIN Radius pro Achteck (unterschiedliche Größe, aber regelmäßig)
positions_and_sizes = [
    (octagon_position_left_top, octagon_size),   
    (octagon_position_right_top, octagon_size),   
    (octagon_position_right_bottom, octagon_size),  
    (octagon_position_left_bottom, octagon_size),  
    (octagon_position_top_middle, octagon_size),         
    (octagon_position_right_middle, octagon_size),         
    (octagon_position_bottom_middle, octagon_size),  
]



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


r_oct = 50          # Umkreisradius der Achtecke (Hantelköpfe)
abstand = 90        # Abstand der Achteck-Mittelpunkte zueinander
steg_breite = 80    # Höhe des Verbindungsstegs (Griff)
drehung = 22.5      # Rotation der Achtecke in Grad

with BuildSketch() as hantel:
    with Locations((-abstand / 2, 0), (abstand / 2, 0)):
        RegularPolygon(radius=r_oct, side_count=8, rotation=drehung)
    Rectangle(abstand, steg_breite)


INNER_START = project_point_to_x_axis(INNER_UPPER[0])
INNER_END = project_point_to_x_axis(INNER_UPPER[-1])

INNER_POINTS = (
    [INNER_START]
    + INNER_UPPER
    + [INNER_END]
    + mirror_points_x_axis(list(reversed(INNER_UPPER)))
)

SMALL_HOLES = SERVO_BRACKET_HOLES

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



def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Rectangle(rect_w, rect_h)
        for pos, r in positions_and_sizes:
            with Locations(pos):
                Polygon(*octagon_points((0, 0), r, rotation_deg=22.5))

        # Polygon(*INNER_POINTS, mode=Mode.SUBTRACT)
        add(hantel, mode=Mode.SUBTRACT)


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
    export_step(result, "generated/body_layer_common.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/body_layer_common.dxf")

    show(result, name="body_layer_common", clear=True)

        
if __name__ == "__main__":
    main()
