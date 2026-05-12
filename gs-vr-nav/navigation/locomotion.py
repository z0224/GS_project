"""Continuous locomotion constrained by a geographic navigation mesh.

This module defines the movement API for VR walking or joystick locomotion.
Movement is continuous rather than teleport-based, and the resulting position
will be constrained by a 2D navigation mesh derived from geographic data.
"""

from dataclasses import dataclass

from shapely.geometry.base import BaseGeometry


@dataclass
class NavigationState:
    """Continuous navigation state in aligned world coordinates."""

    position_xyz: tuple[float, float, float]
    heading_rad: float


def update_locomotion(
    state: NavigationState,
    input_vector_xy: tuple[float, float],
    delta_time_s: float,
    nav_mesh: BaseGeometry,
    walk_speed_ms: float,
    collision_margin_m: float,
) -> NavigationState:
    """Advance navigation state using continuous movement and mesh constraints."""
    raise NotImplementedError()


def resolve_collision(
    current_xy: tuple[float, float],
    proposed_xy: tuple[float, float],
    nav_mesh: BaseGeometry,
    collision_margin_m: float,
) -> tuple[float, float]:
    """Resolve a proposed 2D movement against the navigation mesh boundary."""
    raise NotImplementedError()


if __name__ == "__main__":
    raise NotImplementedError("Command-line locomotion simulation is not implemented yet.")
