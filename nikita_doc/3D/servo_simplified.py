#!/usr/bin/env python3
"""Simplified HX-35H servo body — outer shell only, no internal parts.

Dimensions measured from HX-35H.stp solid/bounding-box analysis (mm).
Coordinate frame: X = width, Y = depth (back = −Y, front = +Y), Z = height.
"""

from pathlib import Path

from build123d import (
    Box,
    BuildPart,
    Cylinder,
    Location,
    Locations,
    Mode,
    Part,
    Plane,
    export_step,
)

from utils.ocp_utils import show

# ── Outer-body dimensions (mm) ───────────────────────────────────────────────
# Main rectangular case (centre + front sections combined)
BODY_W     = 24.72   # X width
BODY_Y_MIN = -7.95   # back edge of main case
BODY_Y_MAX = 19.15   # front edge of main case
BODY_H     = 45.46   # total height (Z); base rests on Z = 0

# Back plate — slightly narrower, spans full height
BACK_W     = 24.50
BACK_Y_MIN = -16.55
BACK_Y_MAX = -6.85

# Output-shaft Z centre (shared by all round features)
GEAR_Z_CTR = 35.35

# Inner gear-housing cylinder (axis along Y)
GEAR_INNER_R = 8.90
GEAR_Y_MIN   = -17.13
GEAR_Y_MAX   = -0.03

# Outer gear-cap cylinder (large, short protrusion at the rear)
GEAR_CAP_R     = 9.50
GEAR_CAP_Y_MIN = -19.45
GEAR_CAP_Y_MAX = -13.95

# Output shaft (small cylinder protruding at the rear)
SHAFT_R     = 2.02
SHAFT_Y_MIN = -20.62
SHAFT_Y_MAX = -14.45

# Front connector / cable-exit box
CONN_W     = 20.00
CONN_H     = 20.00
CONN_D     = 5.80
CONN_Y_MIN = 15.25
CONN_Z_MIN = 25.35

# Front shaft bearing (cylinder at the front face)
FRONT_SHAFT_R     = 3.50
FRONT_SHAFT_Y_MIN = 14.15
FRONT_SHAFT_Y_MAX = 20.61

# M2 mounting holes (through the servo body in the Y direction)
M2_R        = 1.00
HOLE_X      = 10.25
HOLE_Z_LOW  = 2.60
HOLE_Z_HIGH = 27.05


# ── Helper ───────────────────────────────────────────────────────────────────

def _y_cylinder(
    radius: float, y_min: float, y_max: float, z_ctr: float,
    mode: Mode = Mode.ADD,
) -> None:
    """Place a cylinder whose axis runs along Y, centred at X = 0."""
    plane = Plane(origin=(0.0, (y_min + y_max) / 2, z_ctr), z_dir=(0, 1, 0))
    with Locations(Location(plane)):
        Cylinder(radius, y_max - y_min, mode=mode)


# ── Model ────────────────────────────────────────────────────────────────────

def build_model() -> Part:
    with BuildPart() as p:
        # 1 · main rectangular case
        with Locations([(0, (BODY_Y_MIN + BODY_Y_MAX) / 2, BODY_H / 2)]):
            Box(BODY_W, BODY_Y_MAX - BODY_Y_MIN, BODY_H)

        # 2 · back plate (slightly narrower, same height)
        with Locations([(0, (BACK_Y_MIN + BACK_Y_MAX) / 2, BODY_H / 2)]):
            Box(BACK_W, BACK_Y_MAX - BACK_Y_MIN, BODY_H)

        # 3 · inner gear housing
        _y_cylinder(GEAR_INNER_R, GEAR_Y_MIN, GEAR_Y_MAX, GEAR_Z_CTR)

        # 4 · outer gear cap (short wide protrusion at the rear)
        _y_cylinder(GEAR_CAP_R, GEAR_CAP_Y_MIN, GEAR_CAP_Y_MAX, GEAR_Z_CTR)

        # 5 · output shaft (protruding at the rear)
        _y_cylinder(SHAFT_R, SHAFT_Y_MIN, SHAFT_Y_MAX, GEAR_Z_CTR)

        # 6 · front connector / cable-exit box
        with Locations([(0, CONN_Y_MIN + CONN_D / 2, CONN_Z_MIN + CONN_H / 2)]):
            Box(CONN_W, CONN_D, CONN_H)

        # 7 · front shaft bearing
        _y_cylinder(FRONT_SHAFT_R, FRONT_SHAFT_Y_MIN, FRONT_SHAFT_Y_MAX, GEAR_Z_CTR)

        # 8 · M2 mounting holes (through in Y, spanning back-plate to front-case)
        hole_depth = BODY_Y_MAX - BACK_Y_MIN + 2.0   # +2 ensures clean Boolean cut
        hole_y_ctr = (BACK_Y_MIN + BODY_Y_MAX) / 2
        for z in (HOLE_Z_LOW, HOLE_Z_HIGH):
            for x in (HOLE_X, -HOLE_X):
                plane = Plane(origin=(x, hole_y_ctr, z), z_dir=(0, 1, 0))
                with Locations(Location(plane)):
                    Cylinder(M2_R, hole_depth, mode=Mode.SUBTRACT)

    return p.part


def main() -> None:
    result = build_model()
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/servo_simplified_HX35H.step")
    show(result, name="servo_simplified_HX35H", clear=True)


if __name__ == "__main__":
    main()
