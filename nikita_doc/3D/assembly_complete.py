#!/usr/bin/env python3

import math
from pathlib import Path

from build123d import Compound, Pos, Rot, export_step

import assembly_body_servo
import assembly_coxa
import assembly_femur
import assembly_foot_servo
import body_common
import servo_simplified
from utils.ocp_utils import show


# All 7 body servo positions minus "head", which is not a leg attachment.
LEG_POSITIONS = [k for k in body_common.SERVO_CUTOUT_CONFIGS if k != "head"]


def _center_xy_below(part, reference_bb):
    bb = part.bounding_box()
    return Pos(
        (reference_bb.min.X + reference_bb.max.X) / 2 - (bb.min.X + bb.max.X) / 2,
        (reference_bb.min.Y + reference_bb.max.Y) / 2 - (bb.min.Y + bb.max.Y) / 2,
        reference_bb.min.Z - bb.max.Z,
    ) * part


def build_single_leg() -> Compound:
    """Coxa → femur → tibia/foot stacked downward in the leg's local frame."""
    coxa = assembly_coxa.build_assembly()
    femur = assembly_femur.build_assembly()
    foot = assembly_foot_servo.build_assembly()

    femur_placed = _center_xy_below(Rot(0, 0, 270) * femur, coxa.bounding_box())
    foot_placed = _center_xy_below(Rot(0, 0, 270) * foot, femur_placed.bounding_box())

    return Compound(children=[coxa, femur_placed, foot_placed])


def build_assembly() -> Compound:
    body = assembly_body_servo.build_assembly()
    leg = build_single_leg()

    # Outward offset (in body-layer XY plane) from servo origin to shaft axis.
    horn_outward = servo_simplified.HORN_Z_CTR - assembly_body_servo.SERVO_Z_MID
    # World Z at which the body servo shaft protrudes above body layer 2.
    shaft_z = (
        assembly_body_servo.SERVO_Z
        + servo_simplified.HORN_FRONT_Y / 2
        + servo_simplified.HORN_DISTANCE_TO_BODY
    )

    leg_instances = []
    for name, config in body_common.SERVO_CUTOUT_CONFIGS.items():
        if name not in LEG_POSITIONS:
            continue
        rot = config.rotation_deg_clockwise
        rad = math.radians(rot)

        # Body servo shaft centre in world XY.
        sx = config.offset_x + horn_outward * math.sin(rad)
        sy = config.offset_y + horn_outward * math.cos(rad)

        # Rot(X=90): maps the default -Z stack to world +Y (outward) and aligns
        #            the coxa bracket shaft-hole axis (world Y) with world +Z,
        #            so it mates with the vertical body servo shaft.
        # Rot(Z=-rot): rotates that outward direction to match this leg's position.
        instance = (
            Pos(sx, sy, shaft_z)
            * Rot(Z=-rot)
            * Rot(X=90)
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
