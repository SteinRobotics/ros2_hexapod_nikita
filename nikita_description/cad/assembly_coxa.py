#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, export_stl, import_step, RigidJoint


from utils.colors import COLOR_DARK_GRAY
import bracket_inclined
import bracket_u_shape
from utils.ocp_utils import show

BRACKET_INCLINED_Y_OFFSET = 4.5  # mm

def _center_xy_below(part, reference_bb):
    """Translate part so its XY is centred under reference_bb and its top (Z_max) touches reference_bb.min.Z."""
    bb = part.bounding_box()
    return Pos(
        (reference_bb.min.X + reference_bb.max.X) / 2 - (bb.min.X + bb.max.X) / 2,
        (reference_bb.min.Y + reference_bb.max.Y) / 2 - (bb.min.Y + bb.max.Y) / 2,
        reference_bb.min.Z - bb.max.Z,
    ) * part


def build_assembly() -> Compound:
    bracket_inclinded = bracket_inclined.build_bracket()
    bracket_inclinded.color = COLOR_DARK_GRAY
    
    bracket_straight = bracket_u_shape.build_bracket()
    bracket_straight.color = COLOR_DARK_GRAY

    bracket_straight_placed = Rot(180, 0, 90) * bracket_straight
    bracket_straight_placed = Pos(0, 0, 0) * bracket_straight_placed
    
    bracket_inclinded_placed =  bracket_inclinded_placed = _center_xy_below(Rot(0, 0, 0) * Rot(bracket_inclined.THETA_DEG, 0, 0) * bracket_inclinded, bracket_straight_placed.bounding_box())
    bracket_inclinded_placed = Pos(0, BRACKET_INCLINED_Y_OFFSET, 0) * bracket_inclinded_placed


    coxa = Compound(children=[bracket_straight_placed, bracket_inclinded_placed])
    RigidJoint("body_to_coxa_fixed", coxa, bracket_straight_placed.joints["fixed"].location)
    RigidJoint("coxa_to_femur_fixed", coxa, bracket_inclinded_placed.joints["fixed"].location)
    return coxa


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_coxa.step")
    export_stl(assembly, "generated/assembly_coxa.stl")
    show(assembly, name="assembly_coxa", clear=True)


if __name__ == "__main__":
    main()
