#!/usr/bin/env python3

from pathlib import Path

from build123d import *

import assembly_leg_cylinders
import cylinder_spacer  
import leg_top
from ocp_utils import show

DARK_GRAY = Color(0.25, 0.25, 0.25)


def build_assembly() -> Compound:
    servo = import_step(str(Path(__file__).parent / "imported" / "HX-35H.stp"))
    servo.color = DARK_GRAY
    leg_assembly = assembly_leg_cylinders.build_assembly()

    servo_bb = servo.bounding_box()
    # servo_cx = (bb.min.X + bb.max.X) / 2
    # servo_cz = (bb.min.Z + bb.max.Z) / 2

    leg_placed = (
        Pos(
            servo_bb.min.X + 12.3,
            servo_bb.max.Y - 3.5,
            servo_bb.max.Z - 30.6,
        )
        * Rot(270, 180, 180)
        * leg_assembly
    )

    return Compound(children=[servo, leg_placed])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg_with_servo_HX35H.step")

    show(assembly, name="assembly_leg_with_servo_HX35H", clear=True)


if __name__ == "__main__":
    main()
