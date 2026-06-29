#!/usr/bin/env python3

from pathlib import Path

from build123d import *

from ocp_utils import show


OUTER_DIAMETER = 5.0
OVERALL_LENGTH = 32.0
STUD_HOLE_DIAMETER = 2.5


def build_model(
    outer_diameter: float = OUTER_DIAMETER,
    inner_diameter: float = STUD_HOLE_DIAMETER,
    length: float = OVERALL_LENGTH,
    build_method=BuildPart,
) -> Part:
    if inner_diameter <= 0:
        raise ValueError("Stud hole diameter must remain positive")

    if outer_diameter <= 0:
        raise ValueError("Outer diameter must remain positive")

    if length <= 0:
        raise ValueError("Length must remain positive")

    if inner_diameter >= outer_diameter:
        raise ValueError("Stud hole diameter must remain smaller than the outer diameter")

    with build_method() as model:
        Cylinder(outer_diameter / 2, length)
        Cylinder(inner_diameter / 2, length, mode=Mode.SUBTRACT)

    return model.part


def main() -> None:
    result = build_model()
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/spacer.step")
    export_stl(result, "generated/spacer.stl")

    show(result, name="spacer", clear=True)


if __name__ == "__main__":
    main()