from __future__ import annotations

import json
import unittest
from unittest.mock import patch

import numpy as np

from qav_wheel.simulation import (
    ActuatorSnapshot,
    ControlCommandDispatcher,
    StatePayloadPublisher,
    _apply_actuator_dynamics_step,
    _apply_sensor_fidelity,
    build_packet_metadata,
    build_state_payload,
)
from qav_wheel.types import (
    ActuatorDynamicsConfig,
    FidelityConfig,
    LoggingConfig,
    NetworkFidelityConfig,
    SensorFidelityConfig,
    SensorLayout,
    SensorNames,
    UAVModelSpec,
)


class _FakeSocket:
    def __init__(self, *payloads: dict[str, object]):
        import json

        self._queue = [json.dumps(payload).encode("utf-8") for payload in payloads]
        self.sent_packets: list[tuple[bytes, tuple[str, int]]] = []

    def recvfrom(self, _buffer_size: int) -> tuple[bytes, tuple[str, int]]:
        if not self._queue:
            raise BlockingIOError()
        return self._queue.pop(0), ("127.0.0.1", 5000)

    def sendto(self, payload: bytes, endpoint: tuple[str, int]) -> None:
        self.sent_packets.append((payload, endpoint))


class _FakeData:
    def __init__(self, nu: int):
        self.ctrl = np.zeros(nu, dtype=float)


class _FakeSensorData:
    def __init__(self) -> None:
        self.time = 1.25
        self.sensordata = np.array(
            [
                1.0,
                2.0,
                3.0,
                0.1,
                0.2,
                0.3,
                0.01,
                0.02,
                0.03,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            dtype=float,
        )


class _DummyRequest:
    def __init__(self, fidelity_mode: str):
        self.instance_id = 0
        self.fidelity_mode = fidelity_mode


class _DummyScene:
    def __init__(self, fidelity: FidelityConfig):
        self.fidelity = fidelity
        self.request = _DummyRequest(fidelity.mode)
        self.model = None
        self.data = None
        self.sensor_layouts = []
        self.surface_evaluator = None
        self.uav_specs = []
        self.geom_names = ()


class SimulationPacketTests(unittest.TestCase):
    def test_build_state_payload_includes_logged_actuator_and_sensor_truth(self) -> None:
        fidelity = FidelityConfig(
            mode="hil",
            logging=LoggingConfig(log_actuator_stats=True, log_sensor_truth=True),
            sensor_fidelity=SensorFidelityConfig(position_noise_std_m=0.01),
        )
        sensor_layout = SensorLayout(
            position=slice(0, 3),
            linear_velocity=slice(3, 6),
            angular_velocity=slice(6, 9),
            x_axis=slice(9, 12),
            y_axis=slice(12, 15),
            z_axis=slice(15, 18),
        )
        uav_spec = UAVModelSpec(
            name="qav",
            body_name="qav_body",
            actuator_names=("m1", "m2", "m3", "m4"),
            sensor_names=SensorNames(
                position="pos",
                linear_velocity="vel",
                angular_velocity="gyro",
                x_axis="x",
                y_axis="y",
                z_axis="z",
            ),
            contact_prefix="wheel",
        )
        actuator_snapshot = ActuatorSnapshot(
            requested_ctrl=np.array([4.0, 4.1, 4.2, 4.3]),
            applied_ctrl=np.array([3.5, 3.6, 3.7, 3.8]),
        )

        with patch(
            "qav_wheel.simulation.build_contact_report",
            return_value={
                "count": 0,
                "total_force_magnitude": 0.0,
                "max_force_magnitude": 0.0,
                "total_normal_force": 0.0,
                "max_normal_force": 0.0,
                "left_wheel": {},
                "right_wheel": {},
                "surface": {},
                "contacts": [],
            },
        ):
            payload = build_state_payload(
                object(),
                _FakeSensorData(),
                [sensor_layout],
                None,
                1.0,
                0,
                [uav_spec],
                (),
                fidelity,
                np.random.default_rng(0),
                actuator_snapshot,
                sequence=7,
                wall_time_send_ns=123,
                fidelity_mode="hil",
            )

        decoded = json.loads(payload.decode("utf-8"))
        self.assertEqual(decoded["sequence"], 7)
        self.assertEqual(decoded["fidelity_mode"], "hil")
        self.assertIn("actuator", decoded)
        self.assertIn("sensor_truth", decoded)
        self.assertEqual(decoded["actuator"]["requested_rotor_thrusts"], [4.0, 4.1, 4.2, 4.3])
        self.assertEqual(decoded["actuator"]["applied_rotor_thrusts"], [3.5, 3.6, 3.7, 3.8])
        self.assertEqual(decoded["sensor_truth"]["position"], [1.0, 2.0, 3.0])
        self.assertNotEqual(decoded["position"], decoded["sensor_truth"]["position"])

    def test_build_state_payload_omits_optional_fidelity_logs_by_default(self) -> None:
        sensor_layout = SensorLayout(
            position=slice(0, 3),
            linear_velocity=slice(3, 6),
            angular_velocity=slice(6, 9),
            x_axis=slice(9, 12),
            y_axis=slice(12, 15),
            z_axis=slice(15, 18),
        )
        uav_spec = UAVModelSpec(
            name="qav",
            body_name="qav_body",
            actuator_names=("m1", "m2", "m3", "m4"),
            sensor_names=SensorNames(
                position="pos",
                linear_velocity="vel",
                angular_velocity="gyro",
                x_axis="x",
                y_axis="y",
                z_axis="z",
            ),
            contact_prefix="wheel",
        )

        with patch(
            "qav_wheel.simulation.build_contact_report",
            return_value={
                "count": 0,
                "total_force_magnitude": 0.0,
                "max_force_magnitude": 0.0,
                "total_normal_force": 0.0,
                "max_normal_force": 0.0,
                "left_wheel": {},
                "right_wheel": {},
                "surface": {},
                "contacts": [],
            },
        ):
            payload = build_state_payload(
                object(),
                _FakeSensorData(),
                [sensor_layout],
                None,
                1.0,
                0,
                [uav_spec],
                (),
                FidelityConfig(),
                np.random.default_rng(0),
                ActuatorSnapshot(
                    requested_ctrl=np.array([4.0, 4.0, 4.0, 4.0]),
                    applied_ctrl=np.array([4.0, 4.0, 4.0, 4.0]),
                ),
                sequence=8,
                wall_time_send_ns=456,
                fidelity_mode="baseline",
            )

        decoded = json.loads(payload.decode("utf-8"))
        self.assertNotIn("actuator", decoded)
        self.assertNotIn("sensor_truth", decoded)

    def test_build_packet_metadata_uses_v2_defaults(self) -> None:
        metadata = build_packet_metadata(sequence=5, wall_time_send_ns=42, fidelity_mode="baseline")

        self.assertEqual(
            metadata,
            {
                "protocol_version": 2,
                "sequence": 5,
                "wall_time_send_ns": 42,
                "fidelity_mode": "baseline",
            },
        )

    def test_control_command_dispatcher_tracks_sequence_gap_for_fresh_commands(self) -> None:
        dispatcher = ControlCommandDispatcher(
            {
                "actuation": {"thrust_coefficient": 2.0e-5},
                "drone": {"body_box": {"mass": 0.9}, "wheels": {"mass": 0.1}},
                "simulation": {"gravity": [0.0, 0.0, -9.81]},
            },
            FidelityConfig(),
            num_uavs=1,
        )
        data = _FakeData(4)

        dispatcher.apply_next_command(_FakeSocket({"sequence": 1, "rotor_thrusts": [1.0, 1.0, 1.0, 1.0]}), data)
        dispatcher.apply_next_command(_FakeSocket({"sequence": 4, "rotor_thrusts": [2.0, 2.0, 2.0, 2.0]}), data)

        np.testing.assert_allclose(data.ctrl, np.array([2.0, 2.0, 2.0, 2.0]))
        self.assertEqual(dispatcher.stats.last_sequence_gap, 2)
        self.assertEqual(dispatcher.stats.missed_command_updates, 2)
        self.assertEqual(dispatcher.stats.last_applied_policy, "fresh")

    def test_control_command_dispatcher_zeroes_stale_packet(self) -> None:
        dispatcher = ControlCommandDispatcher(
            {
                "actuation": {"thrust_coefficient": 2.0e-5},
                "drone": {"body_box": {"mass": 0.9}, "wheels": {"mass": 0.1}},
                "simulation": {"gravity": [0.0, 0.0, -9.81]},
            },
            FidelityConfig(
                network=NetworkFidelityConfig(
                    stale_command_threshold_ms=5.0,
                    stale_command_policy="zero-thrust",
                )
            ),
            num_uavs=1,
        )
        data = _FakeData(4)
        data.ctrl[:] = [9.0, 9.0, 9.0, 9.0]

        dispatcher.apply_next_command(
            _FakeSocket(
                {
                    "sequence": 2,
                    "wall_time_send_ns": 0,
                    "rotor_thrusts": [1.0, 2.0, 3.0, 4.0],
                }
            ),
            data,
        )

        np.testing.assert_allclose(data.ctrl, np.zeros(4))
        self.assertEqual(dispatcher.stats.stale_command_count, 1)
        self.assertEqual(dispatcher.stats.stale_command_apply_count, 1)
        self.assertEqual(dispatcher.stats.last_applied_policy, "zero-thrust")

    def test_control_command_dispatcher_holds_last_command_after_timeout(self) -> None:
        dispatcher = ControlCommandDispatcher(
            {
                "actuation": {"thrust_coefficient": 2.0e-5},
                "drone": {"body_box": {"mass": 0.9}, "wheels": {"mass": 0.1}},
                "simulation": {"gravity": [0.0, 0.0, -9.81]},
            },
            FidelityConfig(network=NetworkFidelityConfig(stale_command_threshold_ms=10.0)),
            num_uavs=1,
        )
        data = _FakeData(4)

        with patch("qav_wheel.simulation.time.time_ns", return_value=1_000_000):
            dispatcher.apply_next_command(_FakeSocket({"sequence": 1, "rotor_thrusts": [1.0, 2.0, 3.0, 4.0]}), data)

        data.ctrl[:] = [0.0, 0.0, 0.0, 0.0]
        with patch("qav_wheel.simulation.time.time_ns", return_value=20_000_000):
            dispatcher.apply_next_command(_FakeSocket(), data)

        np.testing.assert_allclose(data.ctrl, np.array([1.0, 2.0, 3.0, 4.0]))
        self.assertEqual(dispatcher.stats.command_timeout_count, 1)
        self.assertEqual(dispatcher.stats.stale_command_apply_count, 1)
        self.assertEqual(dispatcher.stats.last_applied_policy, "hold-last-command")

    def test_state_payload_publisher_delays_hil_packets_until_due(self) -> None:
        publisher = StatePayloadPublisher(
            _DummyScene(
                FidelityConfig(
                    mode="hil",
                    network=NetworkFidelityConfig(enabled=True, state_tx_latency_ms=5.0),
                )
            )
        )

        def build_payload(realtime_factor, sequence, wall_time_send_ns, actuator_snapshot):
            self.assertIsNone(actuator_snapshot)
            return f"seq={sequence}".encode("utf-8")

        publisher.build_payload = build_payload
        sock = _FakeSocket()

        with patch("qav_wheel.simulation.time.time_ns", side_effect=[1_000_000, 7_000_000]):
            publisher.send_state(sock, "127.0.0.1", 5001, 1.0, None)
            publisher.send_state(sock, "127.0.0.1", 5001, 1.0, None)

        self.assertEqual(len(sock.sent_packets), 1)
        self.assertEqual(sock.sent_packets[0], (b"seq=1", ("127.0.0.1", 5001)))

    def test_control_command_dispatcher_delays_hil_command_until_due(self) -> None:
        dispatcher = ControlCommandDispatcher(
            {
                "actuation": {"thrust_coefficient": 2.0e-5},
                "drone": {"body_box": {"mass": 0.9}, "wheels": {"mass": 0.1}},
                "simulation": {"gravity": [0.0, 0.0, -9.81]},
            },
            FidelityConfig(
                mode="hil",
                network=NetworkFidelityConfig(enabled=True, command_rx_latency_ms=5.0),
            ),
            num_uavs=1,
        )
        data = _FakeData(4)

        with patch("qav_wheel.simulation.time.time_ns", side_effect=[1_000_000, 1_000_000, 7_000_000]):
            dispatcher.apply_next_command(_FakeSocket({"sequence": 1, "rotor_thrusts": [1.0, 2.0, 3.0, 4.0]}), data)
            np.testing.assert_allclose(data.ctrl, np.zeros(4))
            dispatcher.apply_next_command(_FakeSocket(), data)

        np.testing.assert_allclose(data.ctrl, np.array([1.0, 2.0, 3.0, 4.0]))
        self.assertEqual(dispatcher.stats.last_packet_sequence, 1)

    def test_apply_actuator_dynamics_step_applies_first_order_lag(self) -> None:
        applied = _apply_actuator_dynamics_step(
            np.array([4.0, 4.0, 4.0, 4.0]),
            np.zeros(4),
            0.001,
            FidelityConfig(mode="hil", actuator_dynamics=ActuatorDynamicsConfig(motor_tau_ms=10.0)),
            2.0e-5,
        )

        np.testing.assert_allclose(applied, np.full(4, 4.0 / 11.0), rtol=1.0e-6, atol=1.0e-6)

    def test_apply_actuator_dynamics_step_limits_thrust_rate(self) -> None:
        applied = _apply_actuator_dynamics_step(
            np.array([4.0, 4.0, 4.0, 4.0]),
            np.zeros(4),
            0.001,
            FidelityConfig(mode="hil", actuator_dynamics=ActuatorDynamicsConfig(thrust_rate_limit_n_per_s=500.0)),
            2.0e-5,
        )

        np.testing.assert_allclose(applied, np.full(4, 0.5), rtol=1.0e-9, atol=1.0e-9)

    def test_apply_sensor_fidelity_preserves_truth_in_baseline(self) -> None:
        position = np.array([1.0, 2.0, 3.0])
        velocity = np.array([0.1, 0.2, 0.3])
        angular_velocity_world = np.array([0.01, 0.02, 0.03])
        rotation_matrix = np.eye(3)

        measured = _apply_sensor_fidelity(
            position,
            velocity,
            angular_velocity_world,
            rotation_matrix,
            FidelityConfig(),
            np.random.default_rng(0),
        )

        for actual, expected in zip(measured, (position, velocity, angular_velocity_world, rotation_matrix), strict=True):
            np.testing.assert_allclose(actual, expected, rtol=0.0, atol=0.0)

    def test_apply_sensor_fidelity_adds_noise_in_hil(self) -> None:
        position = np.array([1.0, 2.0, 3.0])
        velocity = np.array([0.1, 0.2, 0.3])
        angular_velocity_world = np.array([0.01, 0.02, 0.03])
        rotation_matrix = np.eye(3)

        measured_position, measured_velocity, measured_angular_velocity, measured_rotation = _apply_sensor_fidelity(
            position,
            velocity,
            angular_velocity_world,
            rotation_matrix,
            FidelityConfig(
                mode="hil",
                sensor_fidelity=SensorFidelityConfig(
                    position_noise_std_m=0.01,
                    velocity_noise_std_m_per_s=0.02,
                    angular_velocity_noise_std_rad_per_s=0.03,
                    attitude_noise_std_rad=0.01,
                ),
            ),
            np.random.default_rng(0),
        )

        self.assertFalse(np.allclose(measured_position, position))
        self.assertFalse(np.allclose(measured_velocity, velocity))
        self.assertFalse(np.allclose(measured_angular_velocity, angular_velocity_world))
        self.assertFalse(np.allclose(measured_rotation, rotation_matrix))


if __name__ == "__main__":
    unittest.main()