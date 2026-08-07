#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step


import assembly_foot
import foot_back
import servo_with_connections
from cad_config import FOOT_SPACER_OVERALL_LENGTH
from servo_with_connections import SERVO_CONNECTIONS_TOP_VIEW
from utils.ocp_utils import show


def build_assembly() -> Compound:
    servo = servo_with_connections.build_assembly()
    foot_assembly = assembly_foot.build_assembly()

    # Z position of foot_front's mating face in foot_assembly coordinates.
    foot_front_z = foot_back.THICKNESS + FOOT_SPACER_OVERALL_LENGTH

    conns = list(SERVO_CONNECTIONS_TOP_VIEW.values())
    conn_cy = conns[0][1]                             # y of the servo connection face
    conn_cz = sum(p[2] for p in conns) / len(conns)  # z centroid of the hole pattern

    # Rx(90°) maps foot_front's sketch-Y → servo-Z and foot-Z → servo-(-Y),
    # aligning SERVO_BRACKET_HOLES with SERVO_CONNECTIONS_TOP_VIEW exactly.
    foot_placed = (
        Pos(0, foot_front_z + conn_cy, conn_cz)
        * Rot(90, 0, 0)
        * foot_assembly
    )

    return Compound(children=[servo, foot_placed])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_tibia.step")

    show(assembly, name="assembly_tibia", clear=True)


if __name__ == "__main__":
    main()
