from __future__ import annotations

import math
import unittest

import numpy as np

from qav_wheel.surface_builder import build_initial_poses, build_surface_model_spec, build_surface_spawn_positions
from qav_wheel.surface import (
    build_surface_blocks,
    build_surface_evaluator,
    evaluate_height_function,
    evaluate_height_gradient,
    evaluate_surface_normal,
    get_surface_config,
    surface_can_use_plane_geom,
)


class SurfaceBehaviorTests(unittest.TestCase):
    def test_get_surface_config_maps_slope_mode_to_height_function(self) -> None:
        config = get_surface_config(
            {
                "surface": {
                    "mode": "slope",
                    "solref": [0.01, 1.0],
                    "contact": {"contype": 1, "conaffinity": 1},
                    "height_function": {"parameters": {"slope_x": 0.2, "slope_y": -0.1}},
                }
            }
        )

        self.assertEqual(config["type"], "height_function")
        self.assertEqual(config["height_function"]["name"], "slope")

    def test_slope_surface_returns_expected_height_gradient_and_normal(self) -> None:
        evaluator = build_surface_evaluator(
            {
                "type": "height_function",
                "height_function": {
                    "name": "slope",
                    "parameters": {"z_offset": 0.5, "slope_x": 0.2, "slope_y": -0.1},
                },
            }
        )

        self.assertIsNotNone(evaluator)
        assert evaluator is not None
        self.assertAlmostEqual(evaluate_height_function(evaluator, 2.0, 3.0), 0.6)
        self.assertEqual(evaluate_height_gradient(evaluator, 2.0, 3.0), (0.2, -0.1))
        normal = evaluate_surface_normal(evaluator, 2.0, 3.0)
        expected = np.array([-0.2, 0.1, 1.0], dtype=float)
        expected /= np.linalg.norm(expected)
        self.assertTrue(np.allclose(normal, expected))

    def test_gaussian_surface_uses_hfield_when_not_planar(self) -> None:
        asset_block, geom_block, evaluator = build_surface_blocks(
            {
                "type": "height_function",
                "material": "floor_mat",
                "solref": [0.01, 1.0],
                "contact": {"contype": 1, "conaffinity": 1},
                "height_function": {
                    "name": "gaussian",
                    "x_range": [-1.0, 1.0],
                    "y_range": [-1.0, 1.0],
                    "grid_resolution": [5, 5],
                    "parameters": {"amplitude": 0.4, "sigma_x": 0.5, "sigma_y": 0.5},
                },
            }
        )

        self.assertIn("<hfield", asset_block)
        self.assertIn('type="hfield"', geom_block)
        self.assertIsNotNone(evaluator)

    def test_flat_surface_can_use_plane_geom(self) -> None:
        evaluator = build_surface_evaluator(
            {
                "type": "height_function",
                "height_function": {
                    "name": "flat",
                    "parameters": {"z_offset": math.pi},
                },
            }
        )

        assert evaluator is not None
        self.assertTrue(surface_can_use_plane_geom(evaluator))

    def test_surface_builder_returns_surface_spec(self) -> None:
        surface_spec = build_surface_model_spec(
            {
                "surface": {
                    "mode": "plane",
                    "solref": [0.01, 1.0],
                    "contact": {"contype": 1, "conaffinity": 1},
                    "plane": {"size": [3.0, 3.0, 0.1]},
                }
            }
        )

        self.assertEqual(surface_spec.config["type"], "plane")
        self.assertIn('type="plane"', surface_spec.geom_block)
        self.assertIsNone(surface_spec.evaluator)

    def test_surface_spawn_positions_are_distributed_on_circle(self) -> None:
        positions = build_surface_spawn_positions([0.0, 0.0, 1.5], num_uavs=4, spawn_radius=2.0)

        self.assertEqual(len(positions), 4)
        self.assertEqual(positions[0], [2.0, 0.0, 1.5])
        self.assertTrue(np.allclose(positions[1], [0.0, 2.0, 1.5], atol=1.0e-9))

    def test_build_initial_poses_uses_surface_spec(self) -> None:
        drone = {
            "initial_position": [0.0, 0.0, 0.3],
            "wheels": {"offset_y": 0.2, "radius": 0.1},
        }
        surface_spec = build_surface_model_spec(
            {
                "surface": {
                    "mode": "flat",
                    "solref": [0.01, 1.0],
                    "contact": {"contype": 1, "conaffinity": 1},
                    "follow_surface_for_initial_position": False,
                    "height_function": {
                        "x_range": [-1.0, 1.0],
                        "y_range": [-1.0, 1.0],
                        "grid_resolution": [5, 5],
                        "parameters": {"z_offset": 0.0},
                    },
                }
            }
        )

        poses = build_initial_poses(drone, surface_spec, num_uavs=2, spawn_radius=1.0)

        self.assertEqual(len(poses), 2)
        self.assertEqual(poses[0].position, [1.0, 0.0, 0.3])
        self.assertTrue(np.allclose(poses[1].position, [-1.0, 0.0, 0.3], atol=1.0e-9))


if __name__ == "__main__":
    unittest.main()