#!/usr/bin/env python3

from pathlib import Path

from build123d import *

from ocp_utils import show


THICKNESS = 1.0
M2_RADIUS = 1.0
M2_5_RADIUS = 1.25

OUTLINE_POINTS = [
    (-16.25, -25.75),
    (-19.25, -22.75),
    (-19.25, -16.75),
    (-12.25,  -9.75),
    (  9.75,  -9.75),
    (  9.75,   2.25),
    ( 10.75,   2.25),
    ( 10.75,   9.25),
    (  6.75,   9.25),
    (  6.75,  12.25),
    (  9.75,  15.25),
    ( 13.75,  15.25),
    ( 18.75,   9.25),
    ( 91.75,   0.25),
    ( 91.75, -16.75),
]

BODY_RAIL_HOLES = [
    (-14.25, -19.75, M2_5_RADIUS),
    ( 52.75, -19.75, M2_5_RADIUS),
]

# Rectangle corners for the servo bracket holes; center is the drawing origin.
# Holes are placed at the first three corners (top-right, bottom-right, bottom-left).
SERVO_BRACKET_RECT = [
    ( 10.25,  12.25),  # top-right
    ( 10.25, -12.25),  # bottom-right
    (-10.25, -12.25),  # bottom-left
    (-10.25,  12.25),  # top-left (no hole)
]

SERVO_BRACKET_HOLES = [
    (x, y, M2_RADIUS) for x, y in SERVO_BRACKET_RECT[:3]
]

TIBIA_MOUNT_HOLES = [
    (84.75, -8.25, M2_5_RADIUS),
]

TIP_SLOTS = [
    (88.25,  -3.75, 3.0, 4.0),
    (88.25, -12.75, 3.0, 4.0),
]


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Polygon(*OUTLINE_POINTS)

        for x, y, radius in BODY_RAIL_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for x, y, radius in SERVO_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for x, y, radius in TIBIA_MOUNT_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for x, y, width, height in TIP_SLOTS:
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
    export_step(result, "generated/leg_top.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/leg_top.dxf")

    show(result, name="leg_top", clear=True)


if __name__ == "__main__":
    main()
