#!/usr/bin/env python3
"""Body assembly with the 7 coxa servos placed at their mounting positions."""

import math
from pathlib import Path

from build123d import Compound, Pos, Rot, export_step

from body_layer_0 import (
    LOCATION_RPI5, LOCATION_LEFT_SERVO_PLUG, LOCATION_RIGHT_SERVO_PLUG, LOCATION_RELAY
)
from utils.colors import COLOR_DARK_GRAY
from utils.ocp_utils import show

import assembly_body
import body_common
import servo_simplified
import board_rpi5
import board_servo_plug
import board_relay


# Z height of the servo local origin so that:
#   front horn (local Y < 0) sits inside body layer 2
#   back  horn (local Y > BODY_Y) sits inside body layer 1
_z_layer_2_center = (
    body_common.THICKNESS
    + assembly_body.SPACER_LENGTH_0_to_1
    + body_common.THICKNESS
    + assembly_body.SPACER_LENGTH_1_to_2
    + body_common.THICKNESS / 2
)
SERVO_Z = _z_layer_2_center - (
    servo_simplified.HORN_FRONT_Y / 2 + servo_simplified.HORN_DISTANCE_TO_BODY
)

# Midpoint of the two M2 hole rows along the servo's local Z axis.
# Used to centre the servo over the body-layer cutout.
SERVO_Z_MID = (servo_simplified.HOLE_Z_LOW + servo_simplified.HOLE_Z_HIGH) / 2


def build_assembly() -> Compound:
    body = assembly_body.build_assembly()

    # add servos
    servo_part = servo_simplified.build_model()
    servo_part.color = COLOR_DARK_GRAY

    servo_instances = []
    for name, config in body_common.SERVO_CUTOUT_CONFIGS.items():
        rot = config.rotation_deg_clockwise
        rad = math.radians(rot)

        # Shift origin so the servo's Z_MID aligns with the body-layer cutout centre.
        tx = config.offset_x - SERVO_Z_MID * math.sin(rad)
        ty = config.offset_y - SERVO_Z_MID * math.cos(rad)

        # Rot(X=-90): local Z → world outward, local Y → world -Z (horn downward).
        # Rot(Z=-rot): spin the servo to face the correct outward direction.
        instance = (
            Pos(tx, ty, SERVO_Z)
            * Rot(0, 0, -rot)
            * Rot(-90, 0, 0)
            * servo_part
        )
        instance.label = f"servo_{name}"
        servo_instances.append(instance)

    # add pcbs
    rpi_board = board_rpi5.build_board_with_spacers()
    z_pos = board_rpi5.cfg.thickness + board_rpi5.cfg.spacer_height + body_common.THICKNESS
    rpi_board = Pos(LOCATION_RPI5[0], LOCATION_RPI5[1], z_pos) * Rot(0, 180, 0) * rpi_board

    left_servo_plug_board = board_servo_plug.build_board_with_spacers()
    z_pos = board_servo_plug.cfg.thickness + board_servo_plug.cfg.spacer_height + body_common.THICKNESS
    left_servo_plug_board = Pos(LOCATION_LEFT_SERVO_PLUG[0], LOCATION_LEFT_SERVO_PLUG[1], z_pos) * Rot(0, 180, 0) * left_servo_plug_board

    right_servo_plug_board = board_servo_plug.build_board_with_spacers()
    z_pos = board_servo_plug.cfg.thickness + board_servo_plug.cfg.spacer_height + body_common.THICKNESS
    right_servo_plug_board = Pos(LOCATION_RIGHT_SERVO_PLUG[0], LOCATION_RIGHT_SERVO_PLUG[1], z_pos) * Rot(0, 180, 0) * right_servo_plug_board

    relay_board = board_relay.build_board_with_spacers()
    z_pos = board_relay.cfg.thickness + board_relay.cfg.spacer_height + body_common.THICKNESS
    relay_board = Pos(LOCATION_RELAY[0], LOCATION_RELAY[1], z_pos) * Rot(0, 180, 0) * relay_board

    return Compound(
        label="assembly_body_servo",
        children=[body, *servo_instances, rpi_board, left_servo_plug_board, right_servo_plug_board, relay_board],
    )


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_body_servo.step")
    show(assembly, name="assembly_body_servo", clear=True)


if __name__ == "__main__":
    main()
