#!/usr/bin/env python3


def mirror_points_vertical_axis(
    points: list[tuple[float, float]], axis_x: float = 0.0
) -> list[tuple[float, float]]:
    """Mirror 2D points across the vertical axis x=axis_x."""
    return [(2 * axis_x - x, y) for x, y in points]


def mirror_points_horizontal_axis(
    points: list[tuple[float, float]], axis_y: float = 0.0
) -> list[tuple[float, float]]:
    """Mirror 2D points across the horizontal axis y=axis_y."""
    return [(x, 2 * axis_y - y) for x, y in points]


def mirror_circle_holes_vertical_axis(
    holes: list[tuple[float, float, float]], axis_x: float = 0.0
) -> list[tuple[float, float, float]]:
    """Mirror circular hole centers across the vertical axis x=axis_x."""
    return [(2 * axis_x - x, y, radius) for x, y, radius in holes]


def mirror_circle_holes_horizontal_axis(
    holes: list[tuple[float, float, float]], axis_y: float = 0.0
) -> list[tuple[float, float, float]]:
    """Mirror circular hole centers across the horizontal axis y=axis_y."""
    return [(x, 2 * axis_y - y, radius) for x, y, radius in holes]


def mirror_slots_vertical_axis(
    slots: list[tuple[float, float, float, float]], axis_x: float = 0.0
) -> list[tuple[float, float, float, float]]:
    """Mirror slot centers across the vertical axis x=axis_x."""
    return [(2 * axis_x - x, y, width, height) for x, y, width, height in slots]
