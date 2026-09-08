#!/usr/bin/env python3

import math
from pathlib import Path

from build123d import Compound, Pos, Rot, export_step

import assembly_leg
import assembly_body
import body_common
import servo_simplified
from utils.ocp_utils import show

assembly_leg.ANGLE_COXA = 0.00
assembly_leg.ANGLE_FEMUR = 0.00
assembly_leg.ANGLE_TIBIA = 0.00

# All 7 body servo positions minus "head", which is not a leg attachment.
LEG_POSITIONS = [k for k in body_common.SERVO_CUTOUT_CONFIGS if k != "head"]

# Z of the coxa servo's local origin so its horns sit inside the body layers.
_z_layer_2_center = (
    body_common.THICKNESS
    + assembly_body.SPACER_LENGTH_0_to_1
    + body_common.THICKNESS
    + assembly_body.SPACER_LENGTH_1_to_2
    + body_common.THICKNESS / 2
)

# Same formula as assembly_body_servo: front horn aligns with layer-2 centre.
_COXA_SERVO_Z = _z_layer_2_center - (
    servo_simplified.HORN_FRONT_Y / 2 + servo_simplified.HORN_DISTANCE_TO_BODY
)
# Midpoint of the M2 hole rows along the servo's local Z; used for XY alignment.
_COXA_SERVO_Z_MID = (servo_simplified.HOLE_Z_LOW + servo_simplified.HOLE_Z_HIGH) / 2
# Rot(X=-90) maps local Y → world -Z, so BRACKET_Y_OFFSET adds (not subtracts) to shaft_z.
_LEG_SHAFT_Z = _COXA_SERVO_Z + assembly_leg.BRACKET_Y_OFFSET


def build_assembly() -> Compound:
    body = assembly_body.build_assembly()

    leg_instances = []
    for name, config in body_common.SERVO_CUTOUT_CONFIGS.items():
        if name not in LEG_POSITIONS:
            continue

        leg = assembly_leg.build_assembly()
        rot = config.rotation_deg_clockwise
        rad = math.radians(rot)

        # Rot(X=-90) maps local Z → world +Y (outward); same sign as assembly_body_servo.
        sx = config.offset_x - _COXA_SERVO_Z_MID * math.sin(rad)
        sy = config.offset_y - _COXA_SERVO_Z_MID * math.cos(rad)

        # Rot(Z=-rot): spin to this leg's outward direction.
        instance = (
            Pos(sx, sy, _LEG_SHAFT_Z)
            * Rot(Z=-rot)
            * Rot(X=-90)
            * leg
        )
        instance.label = f"leg_{name}"
        leg_instances.append(instance)

    return Compound(
        label="assembly_complete",
        children=[body, *leg_instances],
    )


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_complete.step")
    show(assembly, name="assembly_complete", clear=True)


if __name__ == "__main__":
    main()
