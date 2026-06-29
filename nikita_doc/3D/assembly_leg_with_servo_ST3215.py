#!/usr/bin/env python3

from pathlib import Path

from build123d import *

import assembly_leg_with_spacers
import spacer  
import leg_top
from ocp_utils import show

# Midpoint of the three servo-bracket holes defined in leg_top.py
_SERVO_HOLES = leg_top.SERVO_BRACKET_HOLES
_SERVO_CENTER_X = sum(x for x, _y, _r in _SERVO_HOLES) / len(_SERVO_HOLES)
_SERVO_CENTER_Y = sum(y for _x, y, _r in _SERVO_HOLES) / len(_SERVO_HOLES)


def build_assembly() -> Compound:
    servo = import_step(str(Path(__file__).parent / "imported" / "ST3215.step"))
    leg_assembly = assembly_leg_with_spacers.build_assembly()

    bb = servo.bounding_box()
    servo_cx = (bb.min.X + bb.max.X) / 2
    servo_cz = (bb.min.Z + bb.max.Z) / 2

    # Rotate the leg 90° about X so it stands in the XZ plane (the servo's
    # front face, whose outward normal is +Y).  After that rotation the
    # original Y coordinate maps to Z, so we shift the bracket-hole cluster
    # to line up with the servo centre in both X and Z, and move the plate to
    # sit flush against the servo's front face (Y = bb.max.Y).
    leg_placed = (
        Pos(
            servo_cx + _SERVO_CENTER_X + 14.0,
            -bb.max.Y - spacer.OVERALL_LENGTH/2 - leg_top.THICKNESS,
            servo_cz + _SERVO_CENTER_Y - 5.5,
        )
        * Rot(270, 0, 90)
        * leg_assembly
    )

    return Compound(children=[servo, leg_placed])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg_with_servo_ST3215.step")

    show(assembly, name="assembly_leg_with_servo_ST3215", clear=True)


if __name__ == "__main__":
    main()
