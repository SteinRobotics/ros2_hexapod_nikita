#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, export_step

from cad_config import (
    SPACER_OUTER_DIAMETER,
    SPACER_OVERALL_LENGTH,
    SPACER_STUD_HOLE_DIAMETER,
)
import leg_bottom
import spacer
from ocp_utils import show


TARGET_HOLE = leg_bottom.TIBIA_MOUNT_HOLES[0]


def build_assembly() -> Compound:
    leg_part = leg_bottom.build_model(leg_bottom.build_surface())
    stud_part = spacer.build_model(
        outer_diameter=SPACER_OUTER_DIAMETER,
        inner_diameter=SPACER_STUD_HOLE_DIAMETER,
        length=SPACER_OVERALL_LENGTH,
    )

    hole_x, hole_y, _hole_radius = TARGET_HOLE
    stud_z = leg_bottom.THICKNESS + SPACER_OVERALL_LENGTH / 2
    placed_stud = Pos(hole_x, hole_y, stud_z) * stud_part

    return Compound(children=[leg_part, placed_stud])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg_with_spacers.step")

    show(assembly, name="assembly_leg_with_spacers", clear=True)


if __name__ == "__main__":
    main()