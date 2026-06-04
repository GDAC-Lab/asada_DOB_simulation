from __future__ import annotations

import unittest

from qav_wheel.config import load_vehicle_params
from qav_wheel.model_builder import build_rotor_specs, build_uav_model_specs, build_xml_replacements


class ModelBuilderTests(unittest.TestCase):
    def test_default_vehicle_params_use_vertical_rotor_axes(self) -> None:
        params = load_vehicle_params()

        rotor_specs = build_rotor_specs(params)

        self.assertEqual([rotor_spec.thrust_axis for rotor_spec in rotor_specs], [(0.0, 0.0, 1.0)] * 4)

    def test_build_uav_model_specs_names_uavs_for_multi_uav_world(self) -> None:
        params = load_vehicle_params()

        specs = build_uav_model_specs(params, num_uavs=3)

        self.assertEqual(len(specs), 3)
        self.assertEqual(specs[0].body_name, "uav_1")
        self.assertEqual(specs[1].sensor_names.position, "uav_2_position")
        self.assertEqual(specs[2].actuator_names[0], "uav_3_thrust_fr")

    def test_build_xml_replacements_includes_multi_uav_blocks(self) -> None:
        params = load_vehicle_params()

        replacements, surface_evaluator, uav_specs = build_xml_replacements(params, num_uavs=2, spawn_radius=1.2)

        self.assertEqual(len(uav_specs), 2)
        self.assertIsNotNone(surface_evaluator)
        self.assertIn('name="uav_1"', replacements["__DRONE_BODY_BLOCK__"])
        self.assertIn('name="uav_2"', replacements["__DRONE_BODY_BLOCK__"])
        self.assertIn('name="uav_1_thrust_fr"', replacements["__ACTUATOR_BLOCK__"])
        self.assertIn('name="uav_2_position"', replacements["__SENSOR_BLOCK__"])
        self.assertIn("surface_geom", replacements["__SURFACE_GEOM_BLOCK__"])


if __name__ == "__main__":
    unittest.main()