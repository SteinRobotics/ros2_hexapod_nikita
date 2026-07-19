#!/usr/bin/env python3

import math
from pathlib import Path

from build123d import *

from ocp_utils import show

import body_common
from body_common import (
    SERVO_BACK_CUTOUT_LEFT_UP,
    SERVO_BACK_CUTOUT_CENTER_UP,
    SERVO_BACK_CUTOUT_RIGHT_UP,
    SERVO_BACK_CUTOUT_LEFT_DOWN,
    SERVO_BACK_CUTOUT_CENTER_DOWN,
    SERVO_BACK_CUTOUT_RIGHT_DOWN,
    SERVO_BACK_CUTOUT_HEAD,
    LIST_BRACKET_HOLES,
)


from geometry_utils import (
    mirror_circle_holes_vertical_axis as mirror_holes_x_axis,
    mirror_points_horizontal_axis as mirror_points_x_axis,
    mirror_slots_vertical_axis as mirror_slots_x_axis,
)

THICKNESS = 1.0
AXIS_X_TOP_BOTTOM = 0.221
AXIS_X_CENTER = -2.779


surface = body_common.build_surface()


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

INNER_START = body_common.project_point_to_x_axis(INNER_UPPER[0])
INNER_END = body_common.project_point_to_x_axis(INNER_UPPER[-1])

INNER_POINTS = (
    [INNER_START]
    + INNER_UPPER
    + [INNER_END]
    + mirror_points_x_axis(list(reversed(INNER_UPPER)))
)

SMALL_HOLES = body_common.SERVO_BRACKET_HOLES

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
        add(surface)
        Polygon(*SERVO_BACK_CUTOUT_HEAD, mode=Mode.SUBTRACT)
        Polygon(*SERVO_BACK_CUTOUT_LEFT_UP, mode=Mode.SUBTRACT)
        Polygon(*SERVO_BACK_CUTOUT_CENTER_UP, mode=Mode.SUBTRACT)
        Polygon(*SERVO_BACK_CUTOUT_RIGHT_UP, mode=Mode.SUBTRACT)
        Polygon(*SERVO_BACK_CUTOUT_LEFT_DOWN, mode=Mode.SUBTRACT)
        Polygon(*SERVO_BACK_CUTOUT_CENTER_DOWN, mode=Mode.SUBTRACT)
        Polygon(*SERVO_BACK_CUTOUT_RIGHT_DOWN, mode=Mode.SUBTRACT)

        # Polygon(*INNER_POINTS, mode=Mode.SUBTRACT)
        add(body_common.hantel, mode=Mode.SUBTRACT)

        for x, y, radius in LIST_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

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

        
if __name__ == "__main__":
    main()
