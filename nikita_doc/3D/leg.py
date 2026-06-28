#!/usr/bin/env python3

from build123d import *
import sys

try:
    from ocp_vscode import show_object
except Exception:
    show_object = None


THICKNESS = 1.0

OUTLINE_POINTS = [
    (0.0, 0.0),
    (-3.0, 3.0),
    (-3.0, 9.0),
    (4.0, 16.0),
    (26.0, 16.0),
    (26.0, 28.0),
    (27.0, 28.0),
    (27.0, 35.0),
    (23.0, 35.0),
    (23.0, 38.0),
    (26.0, 41.0),
    (30.0, 41.0),
    (35.0, 35.0),
    (108.0, 26.0),
    (108.0, 9.0),
]

HOLES = [
    (2.0, 6.0, 1.2),
    (69.0, 6.0, 1.2),
    (101.0, 17.5, 1.2),
    (26.5, 37.9, 1.0),
    (26.5, 13.4, 1.0),
    (5.9, 13.4, 1.0),
]

SLOTS = [
    (104.5, 22.0, 3.0, 4.0),
    (104.5, 13.0, 3.0, 4.0),
]


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Polygon(*OUTLINE_POINTS)

        for x, y, radius in HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for x, y, width, height in SLOTS:
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
    export_step(result, "leg_layout.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("leg_layout.dxf")

    if show_object is not None:
        try:
            show_object(result, name="leg_layout", clear=True)
        except Exception as exc:
            print(f"OCP viewer display failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
