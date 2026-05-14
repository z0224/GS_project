"""Continuous locomotion state helpers.

Runtime locomotion is handled in Unity by CharacterController plus Blosm mesh
colliders. This Python module only keeps lightweight state types for offline
experiments.
"""

from dataclasses import dataclass


@dataclass
class NavigationState:
    """Continuous navigation state in aligned world coordinates."""

    position_xyz: tuple[float, float, float]
    heading_rad: float


def update_locomotion(
    state: NavigationState,
    input_vector_xy: tuple[float, float],
    delta_time_s: float,
    walk_speed_ms: float,
) -> NavigationState:
    """Advance navigation state without map collision."""
    raise NotImplementedError()


def resolve_collision(
    current_xy: tuple[float, float],
    proposed_xy: tuple[float, float],
) -> tuple[float, float]:
    """Resolve a proposed 2D movement; Unity/Blosm owns runtime collision."""
    raise NotImplementedError()


if __name__ == "__main__":
    raise NotImplementedError("Command-line locomotion simulation is not implemented yet.")
