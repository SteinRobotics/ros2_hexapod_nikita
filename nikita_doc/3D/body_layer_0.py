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

from ocp_utils import show

THICKNESS = 1.0

# Converted from imported/body_layer_0.svg path geometry.
# Coordinates are in the same CAD frame as the imported result.
OUTER_POINTS = [
    (241.236, 104.765),
    (241.236, 127.765),
    (248.236, 134.765),
    (248.236, 138.765),
    (207.132, 179.765),
    (203.132, 179.765),
    (193.132, 169.765),
    (73.132, 169.765),
    (63.132, 179.765),
    (59.132, 179.765),
    (18.236, 138.765),
    (18.236, 134.765),
    (25.236, 127.765),
    (25.236, 104.765),
    (25.236, 81.765),
    (18.236, 74.765),
    (18.236, 70.765),
    (59.132, 29.765),
    (63.132, 29.765),
    (73.132, 39.765),
    (193.132, 39.765),
    (203.132, 29.765),
    (207.132, 29.765),
    (248.236, 70.765),
    (248.236, 74.765),
    (241.236, 81.765),
]

SMALL_HOLES = [
    (124.236, 97.185, 1.100),
    (124.236, 113.185, 1.100),
    (159.839, 134.492, 1.100),
    (183.839, 134.492, 1.100),
    (79.736, 142.864, 1.100),
    (97.736, 142.864, 1.100),
    (159.839, 154.492, 1.100),
    (183.839, 154.492, 1.100),
]

LARGE_HOLES = [
    (87.236, 44.764, 1.500),
    (179.236, 44.764, 1.500),
    (58.236, 67.685, 1.500),
    (208.236, 67.685, 1.500),
    (30.236, 89.352, 1.500),
    (230.236, 89.352, 1.500),
    (30.236, 120.264, 1.500),
    (230.236, 120.264, 1.500),
    (58.236, 141.685, 1.500),
    (208.236, 141.685, 1.500),
    (87.236, 164.764, 1.500),
    (179.236, 164.764, 1.500),
]


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
