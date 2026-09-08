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

CORRECTION_FOR_SERVO_HORN = 5.65
_COXA_SERVO_Z = _z_layer_2_center + (
    servo_simplified.HORN_FRONT_Y / 2 + servo_simplified.HORN_DISTANCE_TO_BODY
) + CORRECTION_FOR_SERVO_HORN

# Midpoint of the M2 hole rows along the servo's local Z; used for XY alignment.
CORRECTION_FOR_Y_POSITION = -30.0
_COXA_SERVO_Z_MID = CORRECTION_FOR_Y_POSITION + (servo_simplified.HOLE_Z_LOW + servo_simplified.HOLE_Z_HIGH) / 2
# World-Z of the leg origin: Rot(X=90) maps local Y→world Z, so subtract BRACKET_Y_OFFSET.
_LEG_SHAFT_Z = _COXA_SERVO_Z - assembly_leg.BRACKET_Y_OFFSET


def build_assembly() -> Compound:
    body = assembly_body.build_assembly()

    leg_instances = []
    for name, config in body_common.SERVO_CUTOUT_CONFIGS.items():
        if name not in LEG_POSITIONS:
            continue

        leg = assembly_leg.build_assembly()
        rot = config.rotation_deg_clockwise
        rad = math.radians(rot)

        # Rot(X=90) maps local Z → world -Y, inverting the SERVO_Z_MID sign vs
        # assembly_body_servo (which used Rot(X=-90) and subtracted SERVO_Z_MID).
        sx = config.offset_x + _COXA_SERVO_Z_MID * math.sin(rad)
        sy = config.offset_y + _COXA_SERVO_Z_MID * math.cos(rad)

        # Rot(X=90): maps the default -Z stack to world +Y (outward) and aligns
        #            the coxa bracket shaft-hole axis (world Y) with world +Z,
        #            so it mates with the vertical body servo shaft.
        # Rot(Z=-rot): rotates that outward direction to match this leg's position.
        instance = (
            Pos(sx, sy, _LEG_SHAFT_Z)
            * Rot(Z=-rot)
            * Rot(X=270)
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
