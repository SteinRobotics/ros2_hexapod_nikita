#!/usr/bin/env python3

from pathlib import Path

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    ExportDXF,
    Locations,
    Mode,
    Part,
    Polygon,
    Sketch,
    add,
    export_step,
    extrude,
)

from geometry_utils import (
    mirror_circle_holes_vertical_axis as mirror_holes_y,
    mirror_points_vertical_axis as mirror_points_y,
)
from ocp_utils import show

THICKNESS = 1.0

OUTER_RIGHT_TOP = [
    (108.000,   0.000),
    (108.000,  23.000),
    (115.000,  30.000),
    (115.000,  34.000),
    ( 73.896,  75.000),
    ( 69.896,  75.000),
    ( 59.896,  65.000),
]

OUTER_RIGHT_BOTTOM = [
    ( 59.896, -65.000),
    ( 69.896, -75.000),
    ( 73.896, -75.000),
    (115.000, -34.000),
    (115.000, -30.000),
    (108.000, -23.000),
]

OUTER_POINTS = (
    OUTER_RIGHT_TOP
    + mirror_points_y(list(reversed(OUTER_RIGHT_TOP)))
    + mirror_points_y(list(reversed(OUTER_RIGHT_BOTTOM)))
    + OUTER_RIGHT_BOTTOM
)

SMALL_HOLES = [
    (-9.000, -7.580, 1.100),
    (-9.000, 8.420, 1.100),
    (26.603, 29.727, 1.100),
    (50.603, 29.727, 1.100),
    (-53.500, 38.099, 1.100),
    (-35.500, 38.099, 1.100),
    (26.603, 49.727, 1.100),
    (50.603, 49.727, 1.100),
]

LARGE_HOLES_RIGHT = [
    (46.000, -60.001, 1.500),
    (75.000, -37.080, 1.500),
    (97.000, -15.413, 1.500),
    (97.000, 15.499, 1.500),
    (75.000, 36.920, 1.500),
    (46.000, 59.999, 1.500),
]

LARGE_HOLES = LARGE_HOLES_RIGHT + mirror_holes_y(LARGE_HOLES_RIGHT)


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Polygon(*OUTER_POINTS)

        for x, y, radius in SMALL_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for x, y, radius in LARGE_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

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
    export_step(result, "generated/body_layer_0.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/body_layer_0.dxf")

    show(result, name="body_layer_0", clear=True)


if __name__ == "__main__":
    main()
