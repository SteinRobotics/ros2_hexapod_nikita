#!/usr/bin/env python3

from pathlib import Path

from build123d import (
    BuildLine,
    BuildPart,
    BuildSketch,
    Circle,
    Curve,
    Line,
    ExportDXF,
    Locations,
    Mode,
    Part,
    Pos,
    Polygon,
    Rectangle,
    Sketch,
    Sphere,
    add,
    export_step,
    extrude,
)

from ocp_utils import show

THICKNESS = 1.0
M2_RADIUS = 1.0
M2_5_RADIUS = 1.25

# Set to an OUTLINE_POINTS index to show the matching polygon edge in viewer.
DEBUG_POINT_INDEX: int | None = None
DEBUG_POINT_INDEX = 15

OUTLINE_POINTS = [
    (-16.25, -25.75),  # 0
    (-19.25, -22.75),  # 1
    (-19.25, -16.75),  # 2
    (-12.25,  -9.75),  # 3
    (  9.75,  -9.75),  # 4
    (  9.75,   2.25),  # 5
    ( 10.75,   2.25),  # 6 
    ( 10.75,   9.25),  # 7 
    (  6.75,   9.25),  # 8 
    (  6.75,  12.25),  # 9
    (  9.75,  15.25),  # 10
    ( 13.75,  15.25),  # 11
    ( 18.75,   9.25),  # 12
    ( 91.75,   0.25),  # 13
    ( 91.75, -16.75),  # 14
    ( 53.00, -25.75),  # 15
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


def build_debug_edge_from_point(point_index: int) -> Curve:
    point_count = len(OUTLINE_POINTS)
    if point_index < 0 or point_index >= point_count:
        raise ValueError(f"Point index must be in [0, {point_count - 1}]")

    x1, y1 = OUTLINE_POINTS[point_index]
    x2, y2 = OUTLINE_POINTS[(point_index + 1) % point_count]

    with BuildLine() as edge:
        Line((x1, y1, 0), (x2, y2, 0))

    return edge.line


def main() -> None:
    surface = build_surface()
    result = build_model(surface)
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/leg_bottom.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/leg_bottom.dxf")

    show(result, name="leg_bottom", clear=True)

    if DEBUG_POINT_INDEX is not None:
        edge = build_debug_edge_from_point(DEBUG_POINT_INDEX)
        x, y = OUTLINE_POINTS[DEBUG_POINT_INDEX]
        marker = Pos(x, y, THICKNESS + 0.4) * Sphere(0.5)
        show(edge, name=f"outline_edge_{DEBUG_POINT_INDEX}", clear=False)
        show(marker, name=f"outline_point_{DEBUG_POINT_INDEX}", clear=False)


if __name__ == "__main__":
    main()
