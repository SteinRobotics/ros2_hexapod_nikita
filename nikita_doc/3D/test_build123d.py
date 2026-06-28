#!/usr/bin/env python3

from build123d import *
from itertools import product
import math
import sys

try:
    from ocp_vscode import show_object
except Exception:
    show_object = None


OUTER_DIAMETER = 60
INNER_DIAMETER = 30
THICKNESS = 1
ROTATION_DEGREES = 30
SQUARE_SIZE = 20
HOLE_RADIUS = 3


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        RegularPolygon(OUTER_DIAMETER / 2, 6)

        corner_offset = SQUARE_SIZE / 2
        for dx, dy in product([-1, 1], repeat=2):
            with Locations((dx * corner_offset, dy * corner_offset)):
                Circle(HOLE_RADIUS, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)
    export_step(result, "hex_with_hole.step")
    export_stl(result, "hex_with_hole.stl")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("hex_with_hole.dxf")
    
    # if show_object is not None:
    #     try:
    #         show_object(result, name="hex_with_hole", clear=True)
    #     except Exception as exc:
    #         print(f"OCP viewer display failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
