#!/usr/bin/env python3

from pathlib import Path

from build123d import *

from ocp_utils import show


OUTER_DIAMETER = 5.0
OVERALL_LENGTH = 32.0
STUD_HOLE_DIAMETER = 2.5


def build_model() -> Part:
    if STUD_HOLE_DIAMETER <= 0:
        raise ValueError("Stud hole diameter must remain positive")

    if STUD_HOLE_DIAMETER >= OUTER_DIAMETER:
        raise ValueError("Stud hole diameter must remain smaller than the outer diameter")

    with BuildPart() as model:
        Cylinder(OUTER_DIAMETER / 2, OVERALL_LENGTH)
        Cylinder(STUD_HOLE_DIAMETER / 2, OVERALL_LENGTH, mode=Mode.SUBTRACT)

    return model.part


def main() -> None:
    result = build_model()
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/cylinder_spacer.step")
    export_stl(result, "generated/cylinder_spacer.stl")

    show(result, name="cylinder_spacer", clear=True)


if __name__ == "__main__":
    main()