#!/usr/bin/env python3

import math
from pathlib import Path

from build123d import Compound, Pos, Rot, export_step

import assembly_coxa
import assembly_femur
import assembly_tibia
import assembly_head
import assembly_body_with_servos
import body_common
from utils.ocp_utils import show

ANGLE_LEG_COXA = 0.0    # °
ANGLE_LEG_FEMUR = 0.0   # °
ANGLE_LEG_TIBIA = 0.0   # °
ANGLE_HEAD_YAW = 0.0   # °
ANGLE_HEAD_PITCH = -60.0   # °

BRACKET_Y_OFFSET = 5.2  # mm

ANGLE_LEG_COXA %= 360.00
ANGLE_LEG_FEMUR %= 360.00
ANGLE_LEG_TIBIA %= 360.00
ANGLE_HEAD_YAW %= 360.00
ANGLE_HEAD_PITCH %= 360.00

# All 7 body servo positions minus "head", which is not a leg attachment.
LEG_POSITIONS = [k for k in body_common.SERVO_CUTOUT_CONFIGS if k != "head"]
HEAD_POSITION = body_common.SERVO_CUTOUT_CONFIGS["head"]

# Keep leg placement tied to the servo placement defined by the body assembly.
# The coxa shaft is offset from the servo origin by the bracket clearance.
_LEG_SHAFT_Z = assembly_body_with_servos.SERVO_Z + BRACKET_Y_OFFSET


def build_assembly() -> Compound:
    body = assembly_body_with_servos.build_assembly()

    # HEAD
    coxa_head = assembly_coxa.build_assembly()
    instance_servo_head = assembly_body_with_servos.servo_by_name[f"servo_head"]

    instance_servo_head.joints["rotation"].connect_to(
        coxa_head.joints["body_to_coxa_fixed"],
        angle=ANGLE_HEAD_YAW,
    )
    p = coxa_head.joints["body_to_coxa_fixed"].location.position
    coxa_head = (
        Pos(p.X, p.Y, p.Z)
        * Rot(180, 180, 180)
        * Pos(-p.X, -p.Y, -p.Z + BRACKET_Y_OFFSET)
        * coxa_head
    )

    # The head carries its own servo and is mounted at the coxa's outer
    # attachment point, just as the femur is mounted to each leg coxa.
    head = assembly_head.build_assembly()
    coxa_head.joints["coxa_to_femur_fixed"].connect_to(head.joints["servo_mount"])
    p = head.joints["servo_mount"].location.position
    head = (
        Pos(p.X, p.Y, p.Z)
        * Rot(0, ANGLE_HEAD_PITCH, 0)
        * Pos(-p.X, -p.Y - BRACKET_Y_OFFSET, -p.Z)
        * head
    )

    # LEGS
    leg_instances = []

    for name in LEG_POSITIONS:
        config = body_common.SERVO_CUTOUT_CONFIGS[name]

        coxa_assembly = assembly_coxa.build_assembly()
        femur_assembly = assembly_femur.build_assembly()
        tibia_assembly = assembly_tibia.build_assembly()

        
        # instance_servo = assembly_body_with_servos.servo_by_name[f"servo_{name}"]
        
        # instance_servo.joints["rotation"].connect_to(
        #     coxa_assembly.joints["body_to_coxa_fixed"],
        #     angle=ANGLE_LEG_COXA,
        # )
        p = coxa_assembly.joints["body_to_coxa_fixed"].location.position
        coxa_assembly = (
            Pos(p.X, p.Y, p.Z)
            * Rot(0, 180, 180)
            * Pos(-p.X, -p.Y + BRACKET_Y_OFFSET + assembly_body_with_servos.SERVO_Z_MID , -p.Z - BRACKET_Y_OFFSET)
            * coxa_assembly
        )

        # Connect the coxa to the femur and rotate the femur around the
        # coxa/femur joint, matching assembly_leg.py.
        coxa_assembly.joints["coxa_to_femur_fixed"].connect_to(
            femur_assembly.joints["femur_to_coxa_revolute"],
            angle=ANGLE_LEG_FEMUR,
        )
        p = coxa_assembly.joints["coxa_to_femur_fixed"].location.position
        femur_assembly = (
            Pos(p.X, p.Y, p.Z)
            * Rot(60, 0, 180)
            * Pos(p.X + BRACKET_Y_OFFSET, -p.Y, -p.Z)
            * femur_assembly
        )

        # Connect the femur to the tibia and rotate the tibia around the
        # femur/tibia joint, matching assembly_leg.py.
        femur_assembly.joints["femur_to_tibia_fixed"].connect_to(
            tibia_assembly.joints["tibia_to_femur_revolute"],
            angle=ANGLE_LEG_TIBIA,
        )
        p = femur_assembly.joints["femur_to_tibia_fixed"].location.position
        tibia_assembly = (
            Pos(p.X, p.Y, p.Z)
            * Rot((2 * ANGLE_LEG_FEMUR) % 360, 0, 180)
            * Pos(p.X - BRACKET_Y_OFFSET, -p.Y, -p.Z)
            * tibia_assembly
        )

        # The coxa servo is supplied by ``body``. Build only the leg parts
        # here, positioned around that already-placed servo.
        local_leg = Compound(children=[coxa_assembly, femur_assembly, tibia_assembly])

        rot = config.rotation_deg_clockwise
        rad = math.radians(rot)
        sx = config.offset_x - assembly_body_with_servos.SERVO_Z_MID * math.sin(rad)
        sy = config.offset_y - assembly_body_with_servos.SERVO_Z_MID * math.cos(rad)

        instance = (
            Pos(sx, sy, _LEG_SHAFT_Z)
            * Rot(Z=-rot)
            * Rot(X=-90)
            * local_leg
        )
        instance.label = f"leg_{name}"
        leg_instances.append(instance)

    return Compound(
        label="assembly_complete",
        children=[body, coxa_head, head, *leg_instances],
    )


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_complete.step")
    show(assembly, name="assembly_complete", clear=True)


if __name__ == "__main__":
    main()
