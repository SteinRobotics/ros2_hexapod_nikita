#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, import_step

from utils.colors import COLOR_DARK_GRAY

import assembly_foot

from utils.ocp_utils import show

from servo_simplified import build_model as build_servo

def build_assembly() -> Compound:
    servo = build_servo()
    servo.color = COLOR_DARK_GRAY
    foot_assembly = assembly_foot.build_assembly()

    servo_bb = servo.bounding_box()

    foot_placed = (
        Pos(
            servo_bb.min.X + 12.50,
            servo_bb.max.Y - 1.00,
            servo_bb.max.Z - 30.00,
        )
        * Rot(270, 180, 180)
        * foot_assembly
    )

    return Compound(children=[servo, foot_placed])


def main() -> None:
    assembly = build_assembly()
    # Path("generated").mkdir(exist_ok=True)
    # export_step(assembly, "generated/assembly_foot_servo.step")
    show(assembly, name="assembly_foot_servo", clear=True)


if __name__ == "__main__":
    main()
