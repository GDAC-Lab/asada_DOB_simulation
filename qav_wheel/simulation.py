from __future__ import annotations

import json
import random
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np

from .config import build_fidelity_config, load_vehicle_params
from .contact import build_contact_report, build_geom_name_lookup
from .model_builder import build_uav_model_specs, render_model_xml
from .network import (
    UDP_IP,
    CommandPacket,
    apply_multi_uav_thrust_command,
    apply_thrust_command,
    create_udp_socket,
    get_instance_ports,
    receive_control_command,
    receive_multi_uav_control_command,
)
from .paths import DEFAULT_PATH_RESOLVER, PathResolver
from .surface import get_surface_config
from .types import FidelityConfig, SensorLayout, SensorNames, SurfaceEvaluator, UAVModelSpec

__all__ = [
    "ControlCommandDispatcher",
    "RealtimeTracker",
    "SimulationRequest",
    "SimulationScene",
    "StatePayloadPublisher",
    "configure_viewer",
    "build_sensor_layout",
    "build_state_payload",
    "load_simulation_scene",
    "render_simulation_model",
    "run_viewer_loop",
    "validate_model",
    "run_simulation",
]


@dataclass(frozen=True)
class SimulationRequest:
    instance_id: int = 0
    recv_port: int | None = None
    send_port: int | None = None
    bind_ip: str = UDP_IP
    state_target_ip: str = UDP_IP
    num_uavs: int = 1
    spawn_radius: float = 1.5
    params_path: str | Path | None = None
    template_path: str | Path | None = None
    generated_xml_dir: str | Path | None = None
    fidelity_mode: str = "baseline"
    headless: bool = False
    duration_seconds: float | None = None

    def resolved_ports(self) -> tuple[int, int]:
        default_recv_port, default_send_port = get_instance_ports(self.instance_id)
        recv_port = default_recv_port if self.recv_port is None else self.recv_port
        send_port = default_send_port if self.send_port is None else self.send_port
        return recv_port, send_port


@dataclass(frozen=True)
class RenderedModelArtifacts:
    params: dict[str, Any]
    model_xml_path: Path
    surface_evaluator: SurfaceEvaluator | None
    uav_specs: list[UAVModelSpec]


@dataclass(frozen=True)
class SimulationScene:
    request: SimulationRequest
    params: dict[str, Any]
    fidelity: FidelityConfig
    model_xml_path: Path
    model: mujoco.MjModel
    data: mujoco.MjData
    sensor_layouts: list[SensorLayout]
    surface_evaluator: SurfaceEvaluator | None
    uav_specs: list[UAVModelSpec]
    geom_names: tuple[str, ...]


@dataclass
class RealtimeTracker:
    realtime_factor: float = 1.0
    _window_wall: float = 0.0
    _window_sim: float = 0.0

    def update(self, elapsed_wall: float, timestep: float) -> float:
        self._window_wall += elapsed_wall
        self._window_sim += timestep
        if self._window_wall >= 0.5:
            self.realtime_factor = self._window_sim / self._window_wall
            self._window_wall = 0.0
            self._window_sim = 0.0
        return self.realtime_factor


@dataclass
class CommandRuntimeStats:
    last_receive_time_ns: int | None = None
    last_packet_age_ms: float | None = None
    last_packet_sequence: int | None = None
    last_sequence_gap: int = 0
    missed_command_updates: int = 0
    stale_command_count: int = 0
    stale_command_apply_count: int = 0
    command_timeout_count: int = 0
    last_applied_policy: str = "fresh"


@dataclass(frozen=True)
class PendingDatagram:
    release_time_ns: int
    payload: bytes


@dataclass(frozen=True)
class PendingCommandDelivery:
    release_time_ns: int
    packet: CommandPacket


@dataclass(frozen=True)
class ActuatorSnapshot:
    requested_ctrl: np.ndarray
    applied_ctrl: np.ndarray


def _rotation_matrix_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1.0e-12:
        return np.eye(3, dtype=float)
    axis = rotvec / angle
    x_axis, y_axis, z_axis = axis
    skew = np.array(
        [
            [0.0, -z_axis, y_axis],
            [z_axis, 0.0, -x_axis],
            [-y_axis, x_axis, 0.0],
        ],
        dtype=float,
    )
    return np.eye(3, dtype=float) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def _apply_sensor_fidelity(
    position: np.ndarray,
    linear_velocity: np.ndarray,
    angular_velocity_world: np.ndarray,
    rotation_matrix: np.ndarray,
    fidelity: FidelityConfig,
    rng: np.random.Generator | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if fidelity.mode != "hil":
        return position, linear_velocity, angular_velocity_world, rotation_matrix

    sensor_fidelity = fidelity.sensor_fidelity
    measured_position = np.array(position, copy=True)
    measured_linear_velocity = np.array(linear_velocity, copy=True)
    measured_angular_velocity_world = np.array(angular_velocity_world, copy=True)
    measured_rotation_matrix = np.array(rotation_matrix, copy=True)
    if rng is None:
        return measured_position, measured_linear_velocity, measured_angular_velocity_world, measured_rotation_matrix

    if sensor_fidelity.position_noise_std_m > 0.0:
        measured_position += rng.normal(0.0, sensor_fidelity.position_noise_std_m, size=3)
    if sensor_fidelity.velocity_noise_std_m_per_s > 0.0:
        measured_linear_velocity += rng.normal(0.0, sensor_fidelity.velocity_noise_std_m_per_s, size=3)
    if sensor_fidelity.angular_velocity_noise_std_rad_per_s > 0.0:
        measured_angular_velocity_world += rng.normal(0.0, sensor_fidelity.angular_velocity_noise_std_rad_per_s, size=3)
    if sensor_fidelity.attitude_noise_std_rad > 0.0:
        noise_rotvec = rng.normal(0.0, sensor_fidelity.attitude_noise_std_rad, size=3)
        measured_rotation_matrix = measured_rotation_matrix @ _rotation_matrix_from_rotvec(noise_rotvec)
    return measured_position, measured_linear_velocity, measured_angular_velocity_world, measured_rotation_matrix


def _apply_actuator_dynamics_step(
    requested_ctrl: np.ndarray,
    applied_ctrl: np.ndarray,
    timestep: float,
    fidelity: FidelityConfig,
    thrust_coefficient: float,
) -> np.ndarray:
    if fidelity.mode != "hil":
        return np.array(requested_ctrl, copy=True)

    actuator_dynamics = fidelity.actuator_dynamics
    updated_ctrl = np.array(requested_ctrl, copy=True)
    if actuator_dynamics.motor_tau_ms > 0.0:
        tau_seconds = actuator_dynamics.motor_tau_ms / 1.0e3
        alpha = min(1.0, timestep / (tau_seconds + timestep))
        updated_ctrl = applied_ctrl + alpha * (updated_ctrl - applied_ctrl)

    if actuator_dynamics.omega_rate_limit_rad_per_s is not None:
        if thrust_coefficient <= 0.0:
            raise ValueError("actuation.thrust_coefficient must be positive when omega_rate_limit_rad_per_s is set")
        requested_omega = np.sqrt(np.maximum(0.0, updated_ctrl) / thrust_coefficient)
        applied_omega = np.sqrt(np.maximum(0.0, applied_ctrl) / thrust_coefficient)
        omega_step_limit = actuator_dynamics.omega_rate_limit_rad_per_s * timestep
        clipped_omega = applied_omega + np.clip(requested_omega - applied_omega, -omega_step_limit, omega_step_limit)
        updated_ctrl = thrust_coefficient * clipped_omega * clipped_omega

    if actuator_dynamics.thrust_rate_limit_n_per_s is not None:
        thrust_step_limit = actuator_dynamics.thrust_rate_limit_n_per_s * timestep
        updated_ctrl = applied_ctrl + np.clip(updated_ctrl - applied_ctrl, -thrust_step_limit, thrust_step_limit)

    return np.maximum(0.0, updated_ctrl)


class ActuatorModel:
    def __init__(self, scene: SimulationScene):
        self._scene = scene
        self._applied_ctrl = np.zeros(scene.model.nu, dtype=float)
        self._requested_ctrl = np.zeros(scene.model.nu, dtype=float)
        self._initialized = False
        self._thrust_coefficient = float(scene.params["actuation"]["thrust_coefficient"])

    def apply(self, data: mujoco.MjData) -> None:
        self._requested_ctrl = np.array(data.ctrl, copy=True)
        if not self._initialized:
            self._applied_ctrl = np.array(data.ctrl, copy=True)
            self._initialized = True
        self._applied_ctrl = _apply_actuator_dynamics_step(
            self._requested_ctrl,
            self._applied_ctrl,
            float(self._scene.model.opt.timestep),
            self._scene.fidelity,
            self._thrust_coefficient,
        )
        data.ctrl[:] = self._applied_ctrl

    def snapshot(self) -> ActuatorSnapshot:
        return ActuatorSnapshot(
            requested_ctrl=np.array(self._requested_ctrl, copy=True),
            applied_ctrl=np.array(self._applied_ctrl, copy=True),
        )


def configure_viewer(viewer: mujoco.viewer.Handle) -> None:
    viewer.cam.lookat[0] = 2.0
    viewer.cam.lookat[1] = 0.0
    viewer.cam.lookat[2] = 0.5
    viewer.cam.distance = 8.0
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 45


def _get_sensor_slice(model: mujoco.MjModel, sensor_name: str) -> slice:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    sensor_address = model.sensor_adr[sensor_id]
    sensor_dimension = model.sensor_dim[sensor_id]
    return slice(sensor_address, sensor_address + sensor_dimension)


def build_sensor_layout(model: mujoco.MjModel, sensor_names: SensorNames) -> SensorLayout:
    return SensorLayout(
        position=_get_sensor_slice(model, sensor_names.position),
        linear_velocity=_get_sensor_slice(model, sensor_names.linear_velocity),
        angular_velocity=_get_sensor_slice(model, sensor_names.angular_velocity),
        x_axis=_get_sensor_slice(model, sensor_names.x_axis),
        y_axis=_get_sensor_slice(model, sensor_names.y_axis),
        z_axis=_get_sensor_slice(model, sensor_names.z_axis),
    )


def build_sensor_layouts(model: mujoco.MjModel, uav_specs: list[UAVModelSpec]) -> list[SensorLayout]:
    return [build_sensor_layout(model, uav_spec.sensor_names) for uav_spec in uav_specs]


class StatePayloadPublisher:
    def __init__(self, scene: SimulationScene):
        self._scene = scene
        self._sequence = 0
        self._pending_datagrams: list[PendingDatagram] = []
        self._rng = random.Random(0)
        self._sensor_rng = np.random.default_rng(0)

    def _network_fidelity_enabled(self) -> bool:
        return self._scene.fidelity.mode == "hil" and self._scene.fidelity.network.enabled

    def _sample_delay_ms(self, base_latency_ms: float) -> float:
        jitter_ms = self._scene.fidelity.network.jitter_std_dev_ms
        sampled_delay_ms = base_latency_ms
        if jitter_ms > 0.0:
            sampled_delay_ms += self._rng.gauss(0.0, jitter_ms)
        return max(0.0, sampled_delay_ms)

    def _should_drop_packet(self) -> bool:
        loss_percent = self._scene.fidelity.network.packet_loss_percent
        return loss_percent > 0.0 and self._rng.random() * 100.0 < loss_percent

    def _flush_due_datagrams(self, sock: socket.socket, target_ip: str, send_port: int, now_ns: int) -> None:
        remaining_datagrams: list[PendingDatagram] = []
        for pending_datagram in self._pending_datagrams:
            if pending_datagram.release_time_ns <= now_ns:
                sock.sendto(pending_datagram.payload, (target_ip, send_port))
            else:
                remaining_datagrams.append(pending_datagram)
        self._pending_datagrams = remaining_datagrams

    def build_payload(
        self,
        realtime_factor: float,
        sequence: int,
        wall_time_send_ns: int,
        actuator_snapshot: ActuatorSnapshot | None,
    ) -> bytes:
        return build_state_payload(
            self._scene.model,
            self._scene.data,
            self._scene.sensor_layouts,
            self._scene.surface_evaluator,
            realtime_factor,
            self._scene.request.instance_id,
            self._scene.uav_specs,
            self._scene.geom_names,
            fidelity=self._scene.fidelity,
            sensor_rng=self._sensor_rng,
            actuator_snapshot=actuator_snapshot,
            sequence=sequence,
            wall_time_send_ns=wall_time_send_ns,
            fidelity_mode=self._scene.request.fidelity_mode,
        )

    def send_state(
        self,
        sock: socket.socket,
        target_ip: str,
        send_port: int,
        realtime_factor: float,
        actuator_snapshot: ActuatorSnapshot | None,
    ) -> None:
        self._sequence += 1
        now_ns = time.time_ns()
        payload = self.build_payload(realtime_factor, self._sequence, now_ns, actuator_snapshot)
        if not self._network_fidelity_enabled():
            sock.sendto(payload, (target_ip, send_port))
            return

        if not self._should_drop_packet():
            delay_ms = self._sample_delay_ms(self._scene.fidelity.network.state_tx_latency_ms)
            release_time_ns = now_ns + int(delay_ms * 1.0e6)
            self._pending_datagrams.append(PendingDatagram(release_time_ns=release_time_ns, payload=payload))
        self._flush_due_datagrams(sock, target_ip, send_port, now_ns)


def build_packet_metadata(
    *,
    sequence: int,
    wall_time_send_ns: int,
    fidelity_mode: str,
    protocol_version: int = 2,
) -> dict[str, int | str]:
    return {
        "protocol_version": int(protocol_version),
        "sequence": int(sequence),
        "wall_time_send_ns": int(wall_time_send_ns),
        "fidelity_mode": fidelity_mode,
    }


class ControlCommandDispatcher:
    def __init__(self, params: dict[str, Any], fidelity: FidelityConfig, num_uavs: int):
        self._params = params
        self._fidelity = fidelity
        self._num_uavs = num_uavs
        self._stats = CommandRuntimeStats()
        self._last_single_uav_command: list[float] | None = None
        self._last_multi_uav_command: list[list[float]] | None = None
        self._pending_commands: list[PendingCommandDelivery] = []
        self._rng = random.Random(0)

    @property
    def stats(self) -> CommandRuntimeStats:
        return self._stats

    def _network_fidelity_enabled(self) -> bool:
        return self._fidelity.mode == "hil" and self._fidelity.network.enabled

    def _sample_delay_ms(self, base_latency_ms: float) -> float:
        jitter_ms = self._fidelity.network.jitter_std_dev_ms
        sampled_delay_ms = base_latency_ms
        if jitter_ms > 0.0:
            sampled_delay_ms += self._rng.gauss(0.0, jitter_ms)
        return max(0.0, sampled_delay_ms)

    def _should_drop_packet(self) -> bool:
        loss_percent = self._fidelity.network.packet_loss_percent
        return loss_percent > 0.0 and self._rng.random() * 100.0 < loss_percent

    def _observed_command_age_ms(self, command_packet: CommandPacket, now_ns: int) -> float | None:
        wall_time_send_ns = command_packet.metrics.wall_time_send_ns
        if wall_time_send_ns is None:
            return command_packet.metrics.age_ms
        return max(0.0, (now_ns - wall_time_send_ns) / 1.0e6)

    def _is_stale(self, command_packet: CommandPacket, now_ns: int) -> bool:
        threshold_ms = self._fidelity.network.stale_command_threshold_ms
        observed_age_ms = self._observed_command_age_ms(command_packet, now_ns)
        return threshold_ms is not None and observed_age_ms is not None and observed_age_ms > threshold_ms

    def _queue_incoming_command(self, command_packet: CommandPacket, now_ns: int) -> None:
        if self._should_drop_packet():
            return
        delay_ms = self._sample_delay_ms(self._fidelity.network.command_rx_latency_ms)
        release_time_ns = now_ns + int(delay_ms * 1.0e6)
        self._pending_commands.append(PendingCommandDelivery(release_time_ns=release_time_ns, packet=command_packet))

    def _pop_latest_due_command(self, now_ns: int) -> CommandPacket | None:
        due_packets: list[CommandPacket] = []
        remaining_packets: list[PendingCommandDelivery] = []
        for pending_command in self._pending_commands:
            if pending_command.release_time_ns <= now_ns:
                due_packets.append(pending_command.packet)
            else:
                remaining_packets.append(pending_command)
        self._pending_commands = remaining_packets
        if not due_packets:
            return None
        return due_packets[-1]

    def _update_metrics(self, command_packet: CommandPacket, now_ns: int) -> bool:
        metrics = command_packet.metrics
        self._stats.last_receive_time_ns = metrics.receive_time_ns
        self._stats.last_packet_age_ms = self._observed_command_age_ms(command_packet, now_ns)
        previous_sequence = self._stats.last_packet_sequence
        current_sequence = metrics.sequence
        if previous_sequence is not None and current_sequence is not None:
            sequence_gap = max(0, current_sequence - previous_sequence - 1)
            self._stats.last_sequence_gap = sequence_gap
            self._stats.missed_command_updates += sequence_gap
        else:
            self._stats.last_sequence_gap = 0
        self._stats.last_packet_sequence = current_sequence
        is_stale = self._is_stale(command_packet, now_ns)
        if is_stale:
            self._stats.stale_command_count += 1
        return is_stale

    def _apply_stale_policy(self, data: mujoco.MjData) -> None:
        policy = self._fidelity.network.stale_command_policy
        self._stats.stale_command_apply_count += 1
        self._stats.last_applied_policy = policy
        if policy == "hold-last-command":
            if self._num_uavs == 1:
                if self._last_single_uav_command is not None:
                    apply_thrust_command(data, self._last_single_uav_command)
                return
            if self._last_multi_uav_command is not None:
                apply_multi_uav_thrust_command(data, self._last_multi_uav_command)
            return

        zero_single = [0.0] * data.ctrl.shape[0]
        if policy == "zero-thrust":
            data.ctrl[:] = zero_single
            return

        if policy == "hover-fallback":
            hover_value = float(self._params["drone"]["body_box"]["mass"] + 2.0 * float(self._params["drone"]["wheels"]["mass"]))
            hover_value *= abs(float(self._params["simulation"]["gravity"][2])) / 4.0
            data.ctrl[:] = [hover_value] * data.ctrl.shape[0]

    def _handle_missing_command(self, data: mujoco.MjData, now_ns: int) -> None:
        threshold_ms = self._fidelity.network.stale_command_threshold_ms
        if threshold_ms is None or self._stats.last_receive_time_ns is None:
            return
        age_since_receive_ms = (now_ns - self._stats.last_receive_time_ns) / 1.0e6
        if age_since_receive_ms <= threshold_ms:
            return
        self._stats.command_timeout_count += 1
        self._apply_stale_policy(data)

    def _receive_latest_command(self, sock: socket.socket) -> CommandPacket | None:
        stale_threshold_ms = None if self._network_fidelity_enabled() else self._fidelity.network.stale_command_threshold_ms
        if self._num_uavs == 1:
            return receive_control_command(sock, self._params, stale_command_threshold_ms=stale_threshold_ms)
        return receive_multi_uav_control_command(
            sock,
            self._params,
            self._num_uavs,
            stale_command_threshold_ms=stale_threshold_ms,
        )

    def apply_next_command(self, sock: socket.socket, data: mujoco.MjData) -> None:
        now_ns = time.time_ns()
        incoming_command = self._receive_latest_command(sock)
        if incoming_command is not None and self._network_fidelity_enabled():
            self._queue_incoming_command(incoming_command, now_ns)
            control_command = self._pop_latest_due_command(now_ns)
        elif incoming_command is not None:
            control_command = incoming_command
        else:
            control_command = self._pop_latest_due_command(now_ns) if self._network_fidelity_enabled() else None

        if self._num_uavs == 1:
            if control_command is not None:
                is_stale = self._update_metrics(control_command, now_ns)
                rotor_thrusts = control_command.rotor_thrusts
                if not isinstance(rotor_thrusts, list) or (rotor_thrusts and isinstance(rotor_thrusts[0], list)):
                    raise ValueError("Single-UAV command packet must contain a flat rotor thrust list")
                self._last_single_uav_command = [float(thrust) for thrust in rotor_thrusts]
                if is_stale:
                    self._apply_stale_policy(data)
                else:
                    apply_thrust_command(data, self._last_single_uav_command)
                    self._stats.last_applied_policy = "fresh"
            else:
                self._handle_missing_command(data, now_ns)
            return

        if control_command is not None:
            is_stale = self._update_metrics(control_command, now_ns)
            rotor_thrusts = control_command.rotor_thrusts
            if not isinstance(rotor_thrusts, list) or not rotor_thrusts or not isinstance(rotor_thrusts[0], list):
                raise ValueError("Multi-UAV command packet must contain a nested rotor thrust list")
            self._last_multi_uav_command = [[float(thrust) for thrust in uav_command] for uav_command in rotor_thrusts]
            if is_stale:
                self._apply_stale_policy(data)
            else:
                apply_multi_uav_thrust_command(data, self._last_multi_uav_command)
                self._stats.last_applied_policy = "fresh"
            return
        self._handle_missing_command(data, now_ns)


def _build_uav_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    sensor_layout: SensorLayout,
    surface_evaluator: SurfaceEvaluator | None,
    realtime_factor: float,
    instance_id: int,
    uav_spec: UAVModelSpec,
    geom_names: tuple[str, ...],
    fidelity: FidelityConfig,
    sensor_rng: np.random.Generator | None,
    actuator_snapshot: ActuatorSnapshot | None,
) -> dict[str, object]:
    true_position = np.array(data.sensordata[sensor_layout.position], copy=True)
    true_linear_velocity = np.array(data.sensordata[sensor_layout.linear_velocity], copy=True)
    true_angular_velocity_world = np.array(data.sensordata[sensor_layout.angular_velocity], copy=True)
    true_rotation_matrix = np.column_stack(
        (
            data.sensordata[sensor_layout.x_axis],
            data.sensordata[sensor_layout.y_axis],
            data.sensordata[sensor_layout.z_axis],
        )
    )
    position, linear_velocity, angular_velocity_world, rotation_matrix = _apply_sensor_fidelity(
        true_position,
        true_linear_velocity,
        true_angular_velocity_world,
        true_rotation_matrix,
        fidelity,
        sensor_rng,
    )
    angular_velocity_body = rotation_matrix.T @ angular_velocity_world
    true_angular_velocity_body = true_rotation_matrix.T @ true_angular_velocity_world
    contact_report = build_contact_report(model, data, surface_evaluator, contact_prefix=uav_spec.contact_prefix, geom_names=geom_names)

    uav_state: dict[str, object] = {
        "name": uav_spec.name,
        "time": data.time,
        "position": position.tolist(),
        "velocity": linear_velocity.tolist(),
        "angular_velocity_world": angular_velocity_world.tolist(),
        "angular_velocity_body": angular_velocity_body.tolist(),
        "rotation_matrix": rotation_matrix.reshape(-1).tolist(),
        "z": float(position[2]),
        "vz": float(linear_velocity[2]),
        "yaw_rate": float(angular_velocity_world[2]),
        "realtime_factor": float(realtime_factor),
        "instance_id": int(instance_id),
        "contact_summary": {
            "count": contact_report["count"],
            "total_force_magnitude": contact_report["total_force_magnitude"],
            "max_force_magnitude": contact_report["max_force_magnitude"],
            "total_normal_force": contact_report["total_normal_force"],
            "max_normal_force": contact_report["max_normal_force"],
            "left_wheel": contact_report["left_wheel"],
            "right_wheel": contact_report["right_wheel"],
            "surface": contact_report["surface"],
        },
        "contacts": contact_report["contacts"],
    }
    if actuator_snapshot is not None and fidelity.logging.log_actuator_stats:
        uav_state["actuator" ] = {
            "requested_rotor_thrusts": actuator_snapshot.requested_ctrl.tolist(),
            "applied_rotor_thrusts": actuator_snapshot.applied_ctrl.tolist(),
            "tracking_error": (actuator_snapshot.requested_ctrl - actuator_snapshot.applied_ctrl).tolist(),
        }
    if fidelity.logging.log_sensor_truth:
        uav_state["sensor_truth"] = {
            "position": true_position.tolist(),
            "velocity": true_linear_velocity.tolist(),
            "angular_velocity_world": true_angular_velocity_world.tolist(),
            "angular_velocity_body": true_angular_velocity_body.tolist(),
            "rotation_matrix": true_rotation_matrix.reshape(-1).tolist(),
        }
    return uav_state


def build_state_payload(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    sensor_layouts: list[SensorLayout],
    surface_evaluator: SurfaceEvaluator | None,
    realtime_factor: float,
    instance_id: int,
    uav_specs: list[UAVModelSpec],
    geom_names: tuple[str, ...],
    fidelity: FidelityConfig,
    sensor_rng: np.random.Generator | None,
    actuator_snapshot: ActuatorSnapshot | None,
    *,
    sequence: int,
    wall_time_send_ns: int,
    fidelity_mode: str,
) -> bytes:
    uav_states = [
        _build_uav_state(
            model,
            data,
            sensor_layout,
            surface_evaluator,
            realtime_factor,
            instance_id,
            uav_spec,
            geom_names,
            fidelity,
            sensor_rng,
            actuator_snapshot,
        )
        for sensor_layout, uav_spec in zip(sensor_layouts, uav_specs, strict=True)
    ]
    if len(uav_states) == 1:
        return json.dumps(
            {
                **build_packet_metadata(sequence=sequence, wall_time_send_ns=wall_time_send_ns, fidelity_mode=fidelity_mode),
                **uav_states[0],
                "sim_time": data.time,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")

    return json.dumps(
        {
            **build_packet_metadata(sequence=sequence, wall_time_send_ns=wall_time_send_ns, fidelity_mode=fidelity_mode),
            "time": data.time,
            "sim_time": data.time,
            "instance_id": int(instance_id),
            "num_uavs": len(uav_states),
            "realtime_factor": float(realtime_factor),
            "uavs": uav_states,
        },
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")


def render_simulation_model(
    request: SimulationRequest,
    *,
    path_resolver: PathResolver | None = None,
) -> RenderedModelArtifacts:
    resolver = path_resolver or DEFAULT_PATH_RESOLVER
    params = load_vehicle_params(params_path=request.params_path, path_resolver=resolver)
    model_xml_path, surface_evaluator, uav_specs = render_model_xml(
        params,
        instance_id=request.instance_id,
        num_uavs=request.num_uavs,
        spawn_radius=request.spawn_radius,
        template_path=request.template_path,
        generated_xml_dir=request.generated_xml_dir,
        path_resolver=resolver,
    )
    return RenderedModelArtifacts(
        params=params,
        model_xml_path=model_xml_path,
        surface_evaluator=surface_evaluator,
        uav_specs=uav_specs,
    )


def load_simulation_scene(
    request: SimulationRequest,
    *,
    path_resolver: PathResolver | None = None,
) -> SimulationScene:
    rendered_model = render_simulation_model(request, path_resolver=path_resolver)
    fidelity = build_fidelity_config(rendered_model.params)
    if request.fidelity_mode != fidelity.mode:
        fidelity = FidelityConfig(
            mode=request.fidelity_mode,
            network=fidelity.network,
            actuator_dynamics=fidelity.actuator_dynamics,
            sensor_fidelity=fidelity.sensor_fidelity,
            logging=fidelity.logging,
        )
    model = mujoco.MjModel.from_xml_path(str(rendered_model.model_xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return SimulationScene(
        request=request,
        params=rendered_model.params,
        fidelity=fidelity,
        model_xml_path=rendered_model.model_xml_path,
        model=model,
        data=data,
        sensor_layouts=build_sensor_layouts(model, rendered_model.uav_specs),
        surface_evaluator=rendered_model.surface_evaluator,
        uav_specs=rendered_model.uav_specs,
        geom_names=build_geom_name_lookup(model),
    )


def run_viewer_loop(scene: SimulationScene, sock: socket.socket) -> None:
    _, send_port = scene.request.resolved_ports()
    state_publisher = StatePayloadPublisher(scene)
    command_dispatcher = ControlCommandDispatcher(scene.params, scene.fidelity, scene.request.num_uavs)
    actuator_model = ActuatorModel(scene)
    realtime_tracker = RealtimeTracker()

    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        configure_viewer(viewer)
        while viewer.is_running():
            step_start = time.perf_counter()
            state_publisher.send_state(
                sock,
                scene.request.state_target_ip,
                send_port,
                realtime_tracker.realtime_factor,
                actuator_model.snapshot(),
            )
            command_dispatcher.apply_next_command(sock, scene.data)
            actuator_model.apply(scene.data)
            mujoco.mj_step(scene.model, scene.data)
            viewer.sync()

            elapsed_wall = time.perf_counter() - step_start
            realtime_tracker.update(elapsed_wall, scene.model.opt.timestep)

            remaining_time = scene.model.opt.timestep - elapsed_wall
            if remaining_time > 0:
                time.sleep(remaining_time)


def run_headless_loop(scene: SimulationScene, sock: socket.socket) -> None:
    _, send_port = scene.request.resolved_ports()
    state_publisher = StatePayloadPublisher(scene)
    command_dispatcher = ControlCommandDispatcher(scene.params, scene.fidelity, scene.request.num_uavs)
    actuator_model = ActuatorModel(scene)
    realtime_tracker = RealtimeTracker()
    duration_seconds = scene.request.duration_seconds

    while duration_seconds is None or scene.data.time < duration_seconds:
        step_start = time.perf_counter()
        state_publisher.send_state(
            sock,
            scene.request.state_target_ip,
            send_port,
            realtime_tracker.realtime_factor,
            actuator_model.snapshot(),
        )
        command_dispatcher.apply_next_command(sock, scene.data)
        actuator_model.apply(scene.data)
        mujoco.mj_step(scene.model, scene.data)

        elapsed_wall = time.perf_counter() - step_start
        realtime_tracker.update(elapsed_wall, scene.model.opt.timestep)

        remaining_time = scene.model.opt.timestep - elapsed_wall
        if remaining_time > 0:
            time.sleep(remaining_time)


def validate_model(
    instance_id: int = 0,
    num_uavs: int = 1,
    spawn_radius: float = 1.5,
    params_path: str | Path | None = None,
    template_path: str | Path | None = None,
    generated_xml_dir: str | Path | None = None,
    fidelity_mode: str = "baseline",
    path_resolver: PathResolver | None = None,
) -> int:
    request = SimulationRequest(
        instance_id=instance_id,
        num_uavs=num_uavs,
        spawn_radius=spawn_radius,
        params_path=params_path,
        template_path=template_path,
        generated_xml_dir=generated_xml_dir,
        fidelity_mode=fidelity_mode,
    )
    rendered_model = render_simulation_model(request, path_resolver=path_resolver)
    model = mujoco.MjModel.from_xml_path(str(rendered_model.model_xml_path))

    print(
        json.dumps(
            {
                "instance_id": instance_id,
                "fidelity_mode": request.fidelity_mode,
                "num_uavs": num_uavs,
                "xml_path": str(rendered_model.model_xml_path),
                "surface_type": get_surface_config(rendered_model.params["environment"])["type"],
                "surface_mesh_path": None,
                "surface_function": None if rendered_model.surface_evaluator is None else rendered_model.surface_evaluator.kind,
                "ngeom": int(model.ngeom),
                "nsensor": int(model.nsensor),
                "nu": int(model.nu),
            },
            ensure_ascii=False,
        )
    )
    return 0


def run_simulation(
    instance_id: int = 0,
    recv_port: int | None = None,
    send_port: int | None = None,
    bind_ip: str = UDP_IP,
    state_target_ip: str = UDP_IP,
    num_uavs: int = 1,
    spawn_radius: float = 1.5,
    params_path: str | Path | None = None,
    template_path: str | Path | None = None,
    generated_xml_dir: str | Path | None = None,
    fidelity_mode: str = "baseline",
    headless: bool = False,
    duration_seconds: float | None = None,
    path_resolver: PathResolver | None = None,
) -> None:
    request = SimulationRequest(
        instance_id=instance_id,
        recv_port=recv_port,
        send_port=send_port,
        bind_ip=bind_ip,
        state_target_ip=state_target_ip,
        num_uavs=num_uavs,
        spawn_radius=spawn_radius,
        params_path=params_path,
        template_path=template_path,
        generated_xml_dir=generated_xml_dir,
        fidelity_mode=fidelity_mode,
        headless=headless,
        duration_seconds=duration_seconds,
    )
    recv_port, _ = request.resolved_ports()
    scene = load_simulation_scene(request, path_resolver=path_resolver)
    sock = create_udp_socket(udp_ip=request.bind_ip, recv_port=recv_port)

    try:
        if request.headless:
            run_headless_loop(scene, sock)
        else:
            run_viewer_loop(scene, sock)
    finally:
        sock.close()
