#!/usr/bin/env python3

from pathlib import Path

from build123d import *
from ocp_utils import show

import body_common

THICKNESS = 1.5

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

holes_for_toes = [
    (75.0, 37.0, 1.500),   
    (75.0, -37.0, 1.500),  
    (-75.0, -37.0, 1.500), 
    (-75.0, 37.0, 1.500),  
]


# move to generic helper file or to body_common.py
def chamfered_octagon(width: float, height: float, chamfer_length: float, rotation: float = 0.0) -> Sketch:
    with BuildSketch() as sk:
        Rectangle(width, height, rotation=rotation)
        chamfer(sk.vertices(), chamfer_length)
    return sk.sketch

octagon_positions = [
    body_common.octagon_position_left_top,
    body_common.octagon_position_right_top,
    body_common.octagon_position_right_bottom,
    body_common.octagon_position_left_bottom,
]

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        add(body_common.base_plate)
        for pos in octagon_positions:
            with Locations(pos):
                add(chamfered_octagon(55, 55, 4))

        for loc in body_common.hole_locations:
            with Locations(loc):
                Circle(body_common.hole_radius, mode=Mode.SUBTRACT)
        
        for x, y, radius in SMALL_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for x, y, radius in holes_for_toes:
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
