from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .config import load_vehicle_params
from .network import UDP_IP, create_udp_socket, get_instance_ports

__all__ = [
    "build_hover_command_payload",
    "build_hover_controller_config",
    "compute_hover_control",
    "ControllerRuntimeStats",
    "StatePacketMetrics",
    "resolve_controller_ports",
    "run_hover_controller",
]

_DEFAULT_DESIRED_HEADING = np.array([1.0, 0.0, 0.0], dtype=float)
_DEFAULT_BODY_Z_AXIS = np.array([0.0, 0.0, 1.0], dtype=float)
_DEFAULT_BODY_Y_AXIS = np.array([0.0, 1.0, 0.0], dtype=float)
_CONTROLLER_DEFAULTS: dict[str, tuple[float, float, float]] = {
    "desired_heading": (1.0, 0.0, 0.0),
    "position_gain": (3.0, 3.0, 6.0),
    "velocity_gain": (2.2, 2.2, 4.0),
    "attitude_gain": (0.8, 0.8, 0.25),
    "angular_velocity_gain": (0.12, 0.12, 0.08),
}


@dataclass(frozen=True)
class HoverControllerConfig:
    mass: float
    gravity: float
    arm_x: float
    arm_y: float
    yaw_moment_ratio: float
    max_rotor_thrust: float
    thrust_coefficient: float
    command_mode: str
    desired_heading: np.ndarray
    position_gain: np.ndarray
    velocity_gain: np.ndarray
    attitude_gain: np.ndarray
    angular_velocity_gain: np.ndarray
    mixer: np.ndarray


@dataclass(frozen=True)
class StatePacketMetrics:
    receive_time_ns: int
    protocol_version: int
    sequence: int | None
    wall_time_send_ns: int | None
    fidelity_mode: str | None
    age_ms: float | None


@dataclass
class ControllerRuntimeStats:
    last_state_sequence: int | None = None
    last_state_age_ms: float | None = None
    last_state_sequence_gap: int = 0
    state_sequence_gap_count: int = 0
    timeout_count: int = 0
    last_controller_compute_ms: float | None = None
    last_command_sequence: int | None = None
    last_source_state_sequence: int | None = None


def resolve_controller_ports(instance_id: int, local_port: int | None, target_port: int | None) -> tuple[int, int]:
    default_target_port, default_local_port = get_instance_ports(instance_id)
    resolved_local_port = default_local_port if local_port is None else local_port
    resolved_target_port = default_target_port if target_port is None else target_port
    return resolved_local_port, resolved_target_port


def _normalize_vector(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    vector_norm = np.linalg.norm(vector)
    if vector_norm < 1.0e-6:
        return fallback
    return vector / vector_norm


def _state_vector(state: dict[str, Any], field_name: str) -> np.ndarray:
    return np.asarray(state[field_name], dtype=float).reshape(3)


def _parse_controller_vector(controller_params: dict[str, Any], field_name: str) -> np.ndarray:
    default_vector = _CONTROLLER_DEFAULTS[field_name]
    return np.asarray(controller_params.get(field_name, default_vector), dtype=float).reshape(3)


def _build_allocation_and_mixer(arm_x: float, arm_y: float, yaw_moment_ratio: float) -> np.ndarray:
    allocation_matrix = np.array(
        [
            [1.0, 1.0, 1.0, 1.0],
            [-arm_y, arm_y, -arm_y, arm_y],
            [-arm_x, -arm_x, arm_x, arm_x],
            [yaw_moment_ratio, -yaw_moment_ratio, -yaw_moment_ratio, yaw_moment_ratio],
        ],
        dtype=float,
    )
    return np.linalg.pinv(allocation_matrix)


def _validate_hover_rotor_layout(params: dict[str, Any]) -> None:
    rotor_configs = params.get("actuation", {}).get("rotors")
    if not isinstance(rotor_configs, list):
        return

    for rotor_index, rotor_config in enumerate(rotor_configs):
        if not isinstance(rotor_config, dict):
            raise ValueError(f"actuation.rotors[{rotor_index}] must be an object")

        thrust_axis = np.asarray(rotor_config.get("thrust_axis_body", _DEFAULT_BODY_Z_AXIS), dtype=float).reshape(3)
        thrust_axis = _normalize_vector(thrust_axis, _DEFAULT_BODY_Z_AXIS)
        if not np.allclose(thrust_axis, _DEFAULT_BODY_Z_AXIS, atol=1.0e-6):
            raise ValueError(
                "hover-controller currently supports only vertical rotor axes; "
                f"actuation.rotors[{rotor_index}] has thrust axis {tuple(thrust_axis.tolist())}"
            )


def _thrust_to_rotor_omega(rotor_thrusts: np.ndarray, thrust_coefficient: float) -> np.ndarray:
    if thrust_coefficient <= 0.0:
        raise ValueError("actuation.thrust_coefficient must be positive when command_mode is omega")
    return np.sqrt(np.maximum(0.0, rotor_thrusts) / thrust_coefficient)


def build_hover_controller_config(params: dict[str, Any]) -> HoverControllerConfig:
    _validate_hover_rotor_layout(params)
    controller_params = params.get("controller", {})
    body_box_mass = float(params["drone"]["body_box"]["mass"])
    wheel_mass = float(params["drone"]["wheels"]["mass"])
    mass = body_box_mass + 2.0 * wheel_mass
    gravity = abs(float(params["simulation"]["gravity"][2]))
    arm_x = abs(float(params["drone"]["arm"]["x"]))
    arm_y = abs(float(params["drone"]["arm"]["y"]))
    yaw_moment_ratio = abs(float(params["actuation"]["yaw_moment_ratio"]))
    max_rotor_thrust = float(params["actuation"]["max_rotor_thrust"])
    thrust_coefficient = float(params["actuation"]["thrust_coefficient"])
    command_mode = str(params["actuation"]["command_mode"])

    return HoverControllerConfig(
        mass=mass,
        gravity=gravity,
        arm_x=arm_x,
        arm_y=arm_y,
        yaw_moment_ratio=yaw_moment_ratio,
        max_rotor_thrust=max_rotor_thrust,
        thrust_coefficient=thrust_coefficient,
        command_mode=command_mode,
        desired_heading=_parse_controller_vector(controller_params, "desired_heading"),
        position_gain=_parse_controller_vector(controller_params, "position_gain"),
        velocity_gain=_parse_controller_vector(controller_params, "velocity_gain"),
        attitude_gain=_parse_controller_vector(controller_params, "attitude_gain"),
        angular_velocity_gain=_parse_controller_vector(controller_params, "angular_velocity_gain"),
        mixer=_build_allocation_and_mixer(arm_x, arm_y, yaw_moment_ratio),
    )


def compute_hover_control(state: dict[str, Any], target_position: np.ndarray, config: HoverControllerConfig) -> np.ndarray:
    position = _state_vector(state, "position")
    velocity = _state_vector(state, "velocity")
    angular_velocity = _state_vector(state, "angular_velocity_body")
    rotation_matrix = np.asarray(state["rotation_matrix"], dtype=float).reshape(3, 3)
    desired_heading = _normalize_vector(config.desired_heading, _DEFAULT_DESIRED_HEADING)

    position_error = target_position - position
    velocity_error = -velocity
    desired_force = config.position_gain * position_error + config.velocity_gain * velocity_error + np.array([0.0, 0.0, config.mass * config.gravity], dtype=float)

    body_z_axis = rotation_matrix[:, 2]
    collective_thrust = max(0.0, float(np.dot(desired_force, body_z_axis)))

    desired_body_z = _normalize_vector(desired_force, _DEFAULT_BODY_Z_AXIS)
    desired_body_y = np.cross(desired_body_z, desired_heading)
    if np.linalg.norm(desired_body_y) < 1.0e-6:
        desired_body_y = np.cross(desired_body_z, _DEFAULT_BODY_Y_AXIS)
    desired_body_y = desired_body_y / np.linalg.norm(desired_body_y)
    desired_body_x = np.cross(desired_body_y, desired_body_z)
    desired_body_x = desired_body_x / np.linalg.norm(desired_body_x)
    desired_rotation = np.column_stack((desired_body_x, desired_body_y, desired_body_z))

    attitude_error_matrix = 0.5 * (desired_rotation.T @ rotation_matrix - rotation_matrix.T @ desired_rotation)
    attitude_error = np.array(
        [
            attitude_error_matrix[2, 1],
            attitude_error_matrix[0, 2],
            attitude_error_matrix[1, 0],
        ],
        dtype=float,
    )
    moment_command = -config.attitude_gain * attitude_error - config.angular_velocity_gain * angular_velocity
    wrench = np.concatenate(([collective_thrust], moment_command))
    rotor_thrusts = config.mixer @ wrench
    return np.clip(rotor_thrusts, 0.0, config.max_rotor_thrust)


def build_hover_command_payload(rotor_thrusts: np.ndarray, config: HoverControllerConfig) -> bytes:
    return build_hover_command_payload_with_metadata(rotor_thrusts, config)


def build_hover_command_payload_with_metadata(
    rotor_thrusts: np.ndarray,
    config: HoverControllerConfig,
    *,
    sequence: int | None = None,
    source_state_sequence: int | None = None,
    wall_time_send_ns: int | None = None,
    fidelity_mode: str | None = None,
    protocol_version: int = 2,
) -> bytes:
    payload: dict[str, Any] = {}
    if sequence is not None:
        payload["protocol_version"] = int(protocol_version)
        payload["sequence"] = int(sequence)
    if source_state_sequence is not None:
        payload["source_state_sequence"] = int(source_state_sequence)
    if wall_time_send_ns is not None:
        payload["wall_time_send_ns"] = int(wall_time_send_ns)
    if fidelity_mode is not None:
        payload["fidelity_mode"] = fidelity_mode

    if config.command_mode == "omega":
        rotor_omega = _thrust_to_rotor_omega(rotor_thrusts, config.thrust_coefficient)
        payload["rotor_omega"] = rotor_omega.tolist()
    else:
        payload["rotor_thrusts"] = rotor_thrusts.tolist()
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def _build_state_packet_metrics(state: dict[str, Any], receive_time_ns: int) -> StatePacketMetrics:
    protocol_version = int(state.get("protocol_version", 1))
    sequence = state.get("sequence")
    wall_time_send_ns = state.get("wall_time_send_ns")
    fidelity_mode = state.get("fidelity_mode")
    normalized_sequence = int(sequence) if sequence is not None else None
    normalized_wall_time_send_ns = int(wall_time_send_ns) if wall_time_send_ns is not None else None
    age_ms = None
    if normalized_wall_time_send_ns is not None:
        age_ms = max(0.0, (receive_time_ns - normalized_wall_time_send_ns) / 1.0e6)
    return StatePacketMetrics(
        receive_time_ns=receive_time_ns,
        protocol_version=protocol_version,
        sequence=normalized_sequence,
        wall_time_send_ns=normalized_wall_time_send_ns,
        fidelity_mode=None if fidelity_mode is None else str(fidelity_mode),
        age_ms=age_ms,
    )


def _update_runtime_stats_for_state(stats: ControllerRuntimeStats, metrics: StatePacketMetrics, compute_time_ms: float) -> None:
    previous_sequence = stats.last_state_sequence
    current_sequence = metrics.sequence
    if previous_sequence is not None and current_sequence is not None:
        sequence_gap = max(0, current_sequence - previous_sequence - 1)
        stats.last_state_sequence_gap = sequence_gap
        stats.state_sequence_gap_count += sequence_gap
    else:
        stats.last_state_sequence_gap = 0
    stats.last_state_sequence = current_sequence
    stats.last_state_age_ms = metrics.age_ms
    stats.last_controller_compute_ms = compute_time_ms


def _read_latest_state(sock: socket.socket) -> tuple[dict[str, Any], StatePacketMetrics] | None:
    latest_packet: bytes | None = None
    while True:
        try:
            received_data, _ = sock.recvfrom(65535)
            latest_packet = received_data
        except BlockingIOError:
            break
        except ConnectionResetError:
            break

    if latest_packet is None:
        return None

    receive_time_ns = time.time_ns()
    state = json.loads(latest_packet.decode("utf-8"))
    if not isinstance(state, dict):
        return None
    if "uavs" in state:
        raise ValueError("hover-controller expects single-UAV state packets, but received a multi-UAV packet")
    return state, _build_state_packet_metrics(state, receive_time_ns)


def run_hover_controller(
    *,
    instance_id: int = 0,
    bind_ip: str = UDP_IP,
    target_ip: str = UDP_IP,
    local_port: int | None = None,
    target_port: int | None = None,
    params_path: str | Path | None = None,
    target_position: list[float] | tuple[float, float, float] | np.ndarray = (0.0, 0.0, 1.5),
    duration_seconds: float | None = None,
    state_timeout_seconds: float = 10.0,
    status_display_interval: float = 2.0,
    fidelity_mode: str = "baseline",
) -> None:
    params = load_vehicle_params(params_path=params_path)
    config = build_hover_controller_config(params)
    resolved_target_position = np.asarray(target_position, dtype=float).reshape(3)
    resolved_local_port, resolved_target_port = resolve_controller_ports(instance_id, local_port, target_port)
    sock = create_udp_socket(udp_ip=bind_ip, recv_port=resolved_local_port)

    print(
        f"Starting Python hover controller (instance={instance_id}, bind={bind_ip}:{resolved_local_port}, target={target_ip}:{resolved_target_port})."
    )

    start_simulation_time: float | None = None
    last_state_wall_time = time.monotonic()
    next_status_time = 0.0
    command_sequence = 0
    runtime_stats = ControllerRuntimeStats()

    try:
        while True:
            state_packet = _read_latest_state(sock)
            if state_packet is None:
                if time.monotonic() - last_state_wall_time >= state_timeout_seconds:
                    runtime_stats.timeout_count += 1
                    raise TimeoutError(f"No simulator state received within {state_timeout_seconds:.1f} s")
                time.sleep(0.001)
                continue

            state, state_metrics = state_packet
            last_state_wall_time = time.monotonic()
            simulation_time = float(state["time"])
            if start_simulation_time is None:
                start_simulation_time = simulation_time
            elapsed_simulation_time = simulation_time - start_simulation_time
            if duration_seconds is not None and elapsed_simulation_time >= duration_seconds:
                print(f"Python hover controller complete at t={elapsed_simulation_time:.2f} s")
                break

            compute_start = time.perf_counter()
            rotor_thrusts = compute_hover_control(state, resolved_target_position, config)
            compute_time_ms = (time.perf_counter() - compute_start) * 1.0e3
            _update_runtime_stats_for_state(runtime_stats, state_metrics, compute_time_ms)
            command_sequence += 1
            runtime_stats.last_command_sequence = command_sequence
            runtime_stats.last_source_state_sequence = state_metrics.sequence
            sock.sendto(
                build_hover_command_payload_with_metadata(
                    rotor_thrusts,
                    config,
                    sequence=command_sequence,
                    source_state_sequence=state_metrics.sequence,
                    wall_time_send_ns=time.time_ns(),
                    fidelity_mode=fidelity_mode,
                ),
                (target_ip, resolved_target_port),
            )

            if simulation_time >= next_status_time:
                position = np.asarray(state["position"], dtype=float).reshape(3)
                position_error = resolved_target_position - position
                print(
                    "[python-hover t=%.2f s] pos=[%.3f %.3f %.3f] m, err=[%.3f %.3f %.3f] m, state_age=%.2f ms, state_gap=%d, compute=%.3f ms, cmd=%s [%.3f %.3f %.3f %.3f]"
                    % (
                        elapsed_simulation_time,
                        position[0],
                        position[1],
                        position[2],
                        position_error[0],
                        position_error[1],
                        position_error[2],
                        -1.0 if runtime_stats.last_state_age_ms is None else runtime_stats.last_state_age_ms,
                        runtime_stats.last_state_sequence_gap,
                        -1.0 if runtime_stats.last_controller_compute_ms is None else runtime_stats.last_controller_compute_ms,
                        config.command_mode,
                        rotor_thrusts[0],
                        rotor_thrusts[1],
                        rotor_thrusts[2],
                        rotor_thrusts[3],
                    )
                )
                next_status_time = simulation_time + status_display_interval
    finally:
        sock.close()