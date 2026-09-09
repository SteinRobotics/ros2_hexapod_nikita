#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, import_step, RigidJoint

from utils.ocp_utils import show
from utils.colors import COLOR_DARK_GRAY

import servo_simplified
import assembly_coxa

ANGLE_COXA = 0.0  # degrees

BRACKET_Y_OFFSET = 5.2  # mm


def build_assembly() -> Compound:
    servo = servo_simplified.build_model()
    servo.color = COLOR_DARK_GRAY
    coxa_assembly = assembly_coxa.build_assembly()

    ##########################################################
    ## Connect servo to coxa assembly
    ##########################################################    
    servo.joints["rotation"].connect_to(
        coxa_assembly.joints["body_to_coxa_fixed"],
        angle=ANGLE_COXA,
    )
    servo = Pos(0, BRACKET_Y_OFFSET, 0) * servo



    leg = Compound(children=[servo, coxa_assembly])
    return leg

def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_head.step")

    show(assembly, name="assembly_head", clear=True)


if __name__ == "__main__":
    main()
