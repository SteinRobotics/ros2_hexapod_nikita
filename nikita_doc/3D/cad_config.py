#!/usr/bin/env python3

"""Shared numeric parameters for CAD parts and assemblies."""

# Color definitions
DARK_GRAY = (0.2, 0.2, 0.2)
CREAMY_WHITE = (0.95, 0.95, 0.9)
WINE_RED = (0.5, 0.0, 0.0)

# Spacer dimensions
FOOT_SPACER_OUTER_DIAMETER = 4.0
FOOT_SPACER_OVERALL_LENGTH = 32.0
FOOT_SPACER_STUD_HOLE_DIAMETER = 2.5

# ST3215 leg placement offsets relative to computed servo/hole centers
ST3215_LEG_OFFSET_X = 14.0
ST3215_LEG_OFFSET_Z = -5.5

# HX-35H assembly placement offsets (from servo bounding box)
HX35H_LEG_OFFSET_X = 12.3
HX35H_LEG_OFFSET_Y = -3.5
HX35H_LEG_OFFSET_Z = -30.6