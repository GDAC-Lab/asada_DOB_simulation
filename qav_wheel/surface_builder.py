from __future__ import annotations

from typing import Any

import numpy as np

from .surface import build_surface_blocks, get_drone_initial_pose, get_drone_initial_position, get_surface_config
from .types import InitialPoseSpec, SurfaceModelSpec

__all__ = ["build_initial_poses", "build_surface_model_spec", "build_surface_spawn_positions"]


def build_surface_model_spec(environment: dict[str, Any]) -> SurfaceModelSpec:
    surface_config = get_surface_config(environment)
    surface_asset_block, surface_geom_block, surface_evaluator = build_surface_blocks(surface_config)
    return SurfaceModelSpec(
        config=surface_config,
        asset_block=surface_asset_block,
        geom_block=surface_geom_block,
        evaluator=surface_evaluator,
    )


def build_surface_spawn_positions(initial_position: list[float], num_uavs: int, spawn_radius: float) -> list[list[float]]:
    if num_uavs == 1:
        return [[float(initial_position[0]), float(initial_position[1]), float(initial_position[2])]]

    positions: list[list[float]] = []
    for uav_index in range(num_uavs):
        angle = 2.0 * np.pi * uav_index / num_uavs
        positions.append(
            [
                float(initial_position[0] + spawn_radius * np.cos(angle)),
                float(initial_position[1] + spawn_radius * np.sin(angle)),
                float(initial_position[2]),
            ]
        )
    return positions


def build_initial_poses(
    drone: dict[str, Any],
    surface_spec: SurfaceModelSpec,
    num_uavs: int,
    spawn_radius: float,
) -> list[InitialPoseSpec]:
    initial_position = get_drone_initial_position(drone, surface_spec.evaluator, surface_spec.config)
    initial_positions = build_surface_spawn_positions(initial_position, num_uavs, spawn_radius)
    return [
        get_drone_initial_pose(drone, surface_spec.evaluator, surface_spec.config, x_coord=position[0], y_coord=position[1])
        for position in initial_positions
    ]