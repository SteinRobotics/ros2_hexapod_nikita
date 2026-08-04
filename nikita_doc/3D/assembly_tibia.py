#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, import_step

import assembly_foot
from servo_with_connections import SERVO_CONNECTIONS_TOP_VIEW
from utils.ocp_utils import show


def build_assembly() -> Compound:
    servo = import_step(str(Path(__file__).parent / "imported" / "ST3215.step"))
    foot_assembly = assembly_foot.build_assembly()

    # Centre of the four servo-bracket holes in the ST3215 model's coordinate frame.
    conns = list(SERVO_CONNECTIONS_TOP_VIEW.values())
    conn_cx = sum(p[0] for p in conns) / len(conns)
    conn_cy = sum(p[1] for p in conns) / len(conns)
    conn_cz = sum(p[2] for p in conns) / len(conns)

    # SERVO_BRACKET_HOLES are centred at the foot assembly's local origin.
    # Rot(270, 0, 90) keeps that centre at (0,0,0); Pos moves it to the
    # servo hole centroid so the patterns coincide.
    foot_placed = (
        Pos(conn_cx, conn_cy, conn_cz)
        * Rot(270, 0, 90)
        * foot_assembly
    )

    return Compound(children=[servo, foot_placed])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg_with_servo_ST3215.step")

    show(assembly, name="assembly_leg_with_servo_ST3215", clear=True)


if __name__ == "__main__":
    main()
