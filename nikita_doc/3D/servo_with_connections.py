#!/usr/bin/env python3

from pathlib import Path

from build123d import *
from ocp_utils import show

from cad_config import (
    DARK_GRAY,
)

# Marker geometry — adjust to match your model's units/scale
MARKER_RADIUS = 0.5
MARKER_LENGTH = 2.0

# All positions are given as (x, y, z) tuples. Each group's markers are drawn
# with their cylinder axis along Y, since these positions come from a
# top-view / back-view convention where Y is the viewing direction.

CONNECTIONS_TOP_VIEW = {
    "right_down": (10.25, -16.5, 27.05),
    "left_down": (-10.25, -16.5, 27.05),
    "right_up":  ( 10.25, -16.5,  2.60),
    "left_up":   (-10.25, -16.5,  2.60),
}

CONNECTIONS_BACK_VIEW = {
    "right_down": (10.25, 17.5, 27.05),
    "left_down": (-10.25, 17.5, 27.05),
    "right_up":  ( 10.25, 17.5,  2.60),
    "left_up":   (-10.25, 17.5,  2.60),
}

SERVO_HORN_TOP_VIEW = {
    "center": (0.0, -20.5, 35.35),
    "right":  (7.0, -20.5, 35.35),
    "left":  (-7.0, -20.5, 35.35),
    "up":    (-0.0, -20.5, 42.35),
    "down":   (0.0, -20.5, 28.35),
}

SERVO_HORN_BACK_VIEW = {
    "center": (0.0, 22.0, 35.35),
    "right":  (7.0, 22.0, 35.35),
    "left":  (-7.0, 22.0, 35.35),
    "up":    (-0.0, 22.0, 42.35),
    "down":   (0.0, 22.0, 28.35),
}

# Group name -> (positions dict, marker color)
MARKER_GROUPS = {
    "connections_top": (CONNECTIONS_TOP_VIEW, Color(1, 0, 0)),
    "connections_back": (CONNECTIONS_BACK_VIEW, Color(1, 0.5, 0)),
    "servo_horn_top": (SERVO_HORN_TOP_VIEW, Color(0, 1, 0)),
    "servo_horn_back": (SERVO_HORN_BACK_VIEW, Color(0, 0.5, 1)),
}


def marker_location(position: tuple[float, float, float]) -> Location:
    """Location with Z aligned along global Y, so the marker cylinder's
    axis points along Y at the given position."""
    return Location(Plane(origin=position, z_dir=(0, 1, 0)))


def build_markers() -> list[Part]:
    """Build one small labeled Part per named position, colored by group."""
    marker_parts = []

    for group_name, (positions, color) in MARKER_GROUPS.items():
        for label, pos in positions.items():
            with BuildPart() as marker:
                with Locations(marker_location(pos)):
                    Cylinder(radius=MARKER_RADIUS, height=MARKER_LENGTH)
            marker.part.color = color
            marker.part.label = f"{group_name}:{label}"
            marker_parts.append(marker.part)

    return marker_parts


def build_assembly() -> Compound:
    servo = import_step(str(Path(__file__).parent / "imported" / "HX-35H.stp"))
    servo.color = DARK_GRAY

    marker_parts = build_markers()

    return Compound(children=[servo, *marker_parts])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/servoHX-35H_connection_markers.step")

    show(assembly, name="servoHX-35H_connection_markers", clear=True)


if __name__ == "__main__":
    main()