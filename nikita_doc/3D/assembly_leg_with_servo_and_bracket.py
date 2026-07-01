#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, import_step

import assembly_leg_with_spacers
from cad_config import SPACER_OVERALL_LENGTH, ST3215_LEG_OFFSET_X, ST3215_LEG_OFFSET_Z
import leg_top
from ocp_utils import show

# Midpoint of the three servo-bracket holes defined in leg_top.py
_SERVO_HOLES = leg_top.SERVO_BRACKET_HOLES
_SERVO_CENTER_X = sum(x for x, _y, _r in _SERVO_HOLES) / len(_SERVO_HOLES)
_SERVO_CENTER_Y = sum(y for _x, y, _r in _SERVO_HOLES) / len(_SERVO_HOLES)


# Servo horn disk sits on the +Z face of the ST3215.
# These values were determined by inspecting the servo STEP geometry.
_HORN_X = -12.12   # horn disk centre X in servo native coords
_HORN_Y = 1.86     # horn disk centre Y in servo native coords

# Bracket flat-face centre in bracket native coords (Z=0 end face)
_BRACKET_FACE_CX = -174.10
_BRACKET_FACE_CY = 23.64
# The STEP file is exported at 2× scale relative to the servo model.
_BRACKET_SCALE = 0.5


def build_assembly() -> Compound:
    servo = import_step(str(Path(__file__).parent / "imported" / "ST3215.step"))
    leg_assembly = assembly_leg_with_spacers.build_assembly()
    bracket = import_step(
        str(Path(__file__).parent / "imported" / "HX-35HM Inclinded U Shape Bracket.stp")
    ).scale(_BRACKET_SCALE)

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
            servo_cx + _SERVO_CENTER_X + ST3215_LEG_OFFSET_X,
            -bb.max.Y - SPACER_OVERALL_LENGTH / 2 - leg_top.THICKNESS,
            servo_cz + _SERVO_CENTER_Y + ST3215_LEG_OFFSET_Z,
        )
        * Rot(270, 0, 90)
        * leg_assembly
    )

    # The HX-35HM Inclined U Shape Bracket mounts at the servo horn.
    # The servo horn disk is on the +Z face of the ST3215 (at Z = bb.max.Z).
    # Rot(270,0,0) = Rx(270°) maps (x,y,z)→(x,z,−y), so the scaled face
    # centre (cx·s, cy·s, 0) rotates to (cx·s, 0, −cy·s).  Pos is chosen
    # so that rotated centre lands on the horn: (horn_x, horn_y, horn_z).
    _s = _BRACKET_SCALE
    bracket_placed = (
        Pos(
            _HORN_X - _BRACKET_FACE_CX * _s,   # ≈ +74.93
            _HORN_Y,                             # ≈  +1.86
            bb.max.Z + _BRACKET_FACE_CY * _s,   # ≈ +24.18
        )
        * Rot(270, 0, 0)
        * bracket
    )

    return Compound(children=[servo, leg_placed, bracket_placed])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg_with_servo.step")

    show(assembly, name="assembly_leg_with_servo", clear=True)


if __name__ == "__main__":
    main()
