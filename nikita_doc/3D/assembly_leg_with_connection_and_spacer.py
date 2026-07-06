#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step

from cad_config import (
    SPACER_OUTER_DIAMETER,
    SPACER_OVERALL_LENGTH,
    SPACER_STUD_HOLE_DIAMETER,
)
import leg_bottom
import leg_connection
import leg_top
import spacer
from ocp_utils import show


TARGET_HOLE_X, TARGET_HOLE_Y, _ = leg_bottom.TIBIA_MOUNT_HOLES[0]


def build_assembly() -> Compound:
    connection_part = leg_connection.build_model(leg_connection.build_surface())
    leg_top_part = leg_top.build_model(leg_top.build_surface())
    leg_bottom_part = leg_bottom.build_model(leg_bottom.build_surface())
    spacer_part = spacer.build_model(
        outer_diameter=SPACER_OUTER_DIAMETER,
        inner_diameter=SPACER_STUD_HOLE_DIAMETER,
        length=SPACER_OVERALL_LENGTH,
    )

    # Keep the connection plate's rectangular outline centered on the tibia axis.
    # This lets the top/bottom hole pattern match the connection outline geometry.
    connection_offset_x = TARGET_HOLE_X + SPACER_OUTER_DIAMETER/2
    connection_offset_y = TARGET_HOLE_Y

    # Stack in +Z: leg_bottom -> spacer -> leg_top -> leg_connection.
    bottom_placed = leg_bottom_part
    spacer_z = leg_bottom.THICKNESS + SPACER_OVERALL_LENGTH / 2
    spacer_placed = Pos(TARGET_HOLE_X, TARGET_HOLE_Y, spacer_z) * spacer_part
    top_placed = Pos(0, 0, leg_bottom.THICKNESS + SPACER_OVERALL_LENGTH) * leg_top_part
    connection_placed = Pos(
        connection_offset_x,
        connection_offset_y,
        leg_bottom.THICKNESS + SPACER_OVERALL_LENGTH/2,
    ) * Rot(0, 90, 0) * connection_part

    return Compound(children=[connection_placed, top_placed, bottom_placed, spacer_placed])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg_with_connection_and_spacer.step")

    show(assembly, name="assembly_leg_with_connection_and_spacer", clear=True)


if __name__ == "__main__":
    main()
