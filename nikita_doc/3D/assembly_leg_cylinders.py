#!/usr/bin/env python3

from pathlib import Path

from build123d import *

import leg_top
import cylinder_spacer
from ocp_utils import show


TARGET_HOLE = leg_top.TIBIA_MOUNT_HOLES[0]


def build_assembly() -> Compound:
    leg_part = leg_top.build_model(leg_top.build_surface())
    stud_part = cylinder_spacer.build_model()

    hole_x, hole_y, _hole_radius = TARGET_HOLE
    stud_z = leg_top.THICKNESS + cylinder_spacer.OVERALL_LENGTH / 2
    placed_stud = Pos(hole_x, hole_y, stud_z) * stud_part

    return Compound(children=[leg_part, placed_stud])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg_cylinders.step")

    show(assembly, name="assembly_leg_cylinders", clear=True)


if __name__ == "__main__":
    main()