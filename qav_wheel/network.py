from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass
from typing import Any

import mujoco

from .paths import ROTOR_NAMES

__all__ = [
    "UDP_IP",
    "PORT_SEND",
    "PORT_RECV",
    "RECV_BUFFER_SIZE",
    "CommandPacket",
    "PacketMetrics",
    "get_instance_ports",
    "create_udp_socket",
    "parse_control_input",
    "parse_command_packet",
    "parse_multi_uav_control_input",
    "parse_multi_uav_command_packet",
    "receive_control_command",
    "receive_multi_uav_control_command",
    "apply_thrust_command",
    "apply_multi_uav_thrust_command",
]

UDP_IP = "127.0.0.1"
PORT_SEND = 5001
PORT_RECV = 5000
RECV_BUFFER_SIZE = 1024


@dataclass(frozen=True)
class PacketMetrics:
    receive_time_ns: int
    protocol_version: int
    sequence: int | None
    source_state_sequence: int | None
    wall_time_send_ns: int | None
    fidelity_mode: str | None
    age_ms: float | None
    is_stale: bool


@dataclass(frozen=True)
class CommandPacket:
    rotor_thrusts: list[float] | list[list[float]]
    metrics: PacketMetrics


def get_instance_ports(instance_id: int) -> tuple[int, int]:
    if instance_id < 0:
        raise ValueError("instance_id must be non-negative")
    port_offset = 2 * instance_id
    return PORT_RECV + port_offset, PORT_SEND + port_offset


def create_udp_socket(udp_ip: str = UDP_IP, recv_port: int = PORT_RECV) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip, recv_port))
    sock.setblocking(False)
    return sock


def _parse_rotor_vector(control_input: dict[str, object], field_names: tuple[str, ...]) -> list[float] | None:
    for field_name in field_names:
        field_value = control_input.get(field_name)
        if isinstance(field_value, list) and len(field_value) == len(ROTOR_NAMES):
            return [float(value) for value in field_value]
    return None


def _get_thrust_coefficient(params: dict[str, Any]) -> float:
    thrust_coefficient = float(params["actuation"]["thrust_coefficient"])
    if thrust_coefficient <= 0.0:
        raise ValueError("actuation.thrust_coefficient must be positive")
    return thrust_coefficient


def _parse_single_uav_control_input(control_input: dict[str, object], params: dict[str, Any]) -> list[float] | None:
    rotor_thrusts = control_input.get("rotor_thrusts")
    if isinstance(rotor_thrusts, list) and len(rotor_thrusts) == len(ROTOR_NAMES):
        return [float(thrust) for thrust in rotor_thrusts]

    rotor_omega = _parse_rotor_vector(control_input, ("rotor_omega", "rotor_omegas"))
    if rotor_omega is not None:
        return _rotor_omega_to_thrust(rotor_omega, _get_thrust_coefficient(params))

    thrust = control_input.get("thrust")
    if thrust is None:
        return None

    scalar_thrust = float(thrust)
    return [scalar_thrust] * len(ROTOR_NAMES)


def _rotor_omega_to_thrust(rotor_omega: list[float], thrust_coefficient: float) -> list[float]:
    return [max(0.0, thrust_coefficient * omega * omega) for omega in rotor_omega]


def parse_control_input(control_input: dict[str, object], params: dict[str, Any]) -> list[float] | None:
    return _parse_single_uav_control_input(control_input, params)


def _build_packet_metrics(
    control_input: dict[str, object],
    *,
    receive_time_ns: int,
    stale_command_threshold_ms: float | None,
) -> PacketMetrics:
    protocol_version = int(control_input.get("protocol_version", 1))
    sequence = control_input.get("sequence")
    source_state_sequence = control_input.get("source_state_sequence")
    wall_time_send_ns = control_input.get("wall_time_send_ns")
    fidelity_mode = control_input.get("fidelity_mode")

    normalized_sequence = int(sequence) if sequence is not None else None
    normalized_source_state_sequence = int(source_state_sequence) if source_state_sequence is not None else None
    normalized_wall_time_send_ns = int(wall_time_send_ns) if wall_time_send_ns is not None else None
    normalized_fidelity_mode = None if fidelity_mode is None else str(fidelity_mode)
    age_ms = None
    if normalized_wall_time_send_ns is not None:
        age_ms = max(0.0, (receive_time_ns - normalized_wall_time_send_ns) / 1.0e6)
    is_stale = age_ms is not None and stale_command_threshold_ms is not None and age_ms > stale_command_threshold_ms
    return PacketMetrics(
        receive_time_ns=receive_time_ns,
        protocol_version=protocol_version,
        sequence=normalized_sequence,
        source_state_sequence=normalized_source_state_sequence,
        wall_time_send_ns=normalized_wall_time_send_ns,
        fidelity_mode=normalized_fidelity_mode,
        age_ms=age_ms,
        is_stale=is_stale,
    )


def parse_command_packet(
    control_input: dict[str, object],
    params: dict[str, Any],
    *,
    receive_time_ns: int | None = None,
    stale_command_threshold_ms: float | None = None,
) -> CommandPacket | None:
    rotor_thrusts = parse_control_input(control_input, params)
    if rotor_thrusts is None:
        return None
    resolved_receive_time_ns = time.time_ns() if receive_time_ns is None else int(receive_time_ns)
    return CommandPacket(
        rotor_thrusts=rotor_thrusts,
        metrics=_build_packet_metrics(
            control_input,
            receive_time_ns=resolved_receive_time_ns,
            stale_command_threshold_ms=stale_command_threshold_ms,
        ),
    )


def _receive_latest_packet(sock: socket.socket) -> bytes | None:
    latest_packet: bytes | None = None
    while True:
        try:
            received_data, _ = sock.recvfrom(RECV_BUFFER_SIZE)
            latest_packet = received_data
        except BlockingIOError:
            break
        except ConnectionResetError:
            break
    return latest_packet


def receive_control_command(
    sock: socket.socket,
    params: dict[str, Any],
    *,
    stale_command_threshold_ms: float | None = None,
) -> CommandPacket | None:
    received_data = _receive_latest_packet(sock)
    if received_data is None:
        return None

    control_input = json.loads(received_data.decode("utf-8"))
    if not isinstance(control_input, dict):
        return None
    return parse_command_packet(
        control_input,
        params,
        receive_time_ns=time.time_ns(),
        stale_command_threshold_ms=stale_command_threshold_ms,
    )


def parse_multi_uav_control_input(control_input: dict[str, object], params: dict[str, Any], num_uavs: int) -> list[list[float]] | None:
    rotor_thrusts = control_input.get("rotor_thrusts")
    if isinstance(rotor_thrusts, list) and len(rotor_thrusts) == num_uavs:
        parsed_commands: list[list[float]] = []
        for thrust_vector in rotor_thrusts:
            if not isinstance(thrust_vector, list) or len(thrust_vector) != len(ROTOR_NAMES):
                return None
            parsed_commands.append([float(thrust) for thrust in thrust_vector])
        return parsed_commands

    rotor_omegas = control_input.get("rotor_omegas")
    if isinstance(rotor_omegas, list) and len(rotor_omegas) == num_uavs:
        thrust_coefficient = _get_thrust_coefficient(params)
        parsed_commands = []
        for rotor_omega in rotor_omegas:
            if not isinstance(rotor_omega, list) or len(rotor_omega) != len(ROTOR_NAMES):
                return None
            parsed_commands.append(_rotor_omega_to_thrust([float(value) for value in rotor_omega], thrust_coefficient))
        return parsed_commands

    uavs = control_input.get("uavs")
    if isinstance(uavs, list) and len(uavs) == num_uavs:
        parsed_commands = []
        for uav_control_input in uavs:
            if not isinstance(uav_control_input, dict):
                return None
            rotor_command = _parse_single_uav_control_input(uav_control_input, params)
            if rotor_command is None:
                return None
            parsed_commands.append(rotor_command)
        return parsed_commands

    return None


def parse_multi_uav_command_packet(
    control_input: dict[str, object],
    params: dict[str, Any],
    num_uavs: int,
    *,
    receive_time_ns: int | None = None,
    stale_command_threshold_ms: float | None = None,
) -> CommandPacket | None:
    rotor_thrusts = parse_multi_uav_control_input(control_input, params, num_uavs)
    if rotor_thrusts is None:
        return None
    resolved_receive_time_ns = time.time_ns() if receive_time_ns is None else int(receive_time_ns)
    return CommandPacket(
        rotor_thrusts=rotor_thrusts,
        metrics=_build_packet_metrics(
            control_input,
            receive_time_ns=resolved_receive_time_ns,
            stale_command_threshold_ms=stale_command_threshold_ms,
        ),
    )


def receive_multi_uav_control_command(
    sock: socket.socket,
    params: dict[str, Any],
    num_uavs: int,
    *,
    stale_command_threshold_ms: float | None = None,
) -> CommandPacket | None:
    received_data = _receive_latest_packet(sock)
    if received_data is None:
        return None

    control_input = json.loads(received_data.decode("utf-8"))
    if not isinstance(control_input, dict):
        return None
    return parse_multi_uav_command_packet(
        control_input,
        params,
        num_uavs,
        receive_time_ns=time.time_ns(),
        stale_command_threshold_ms=stale_command_threshold_ms,
    )


def apply_thrust_command(data: mujoco.MjData, rotor_thrusts: list[float]) -> None:
    data.ctrl[:] = rotor_thrusts


def apply_multi_uav_thrust_command(data: mujoco.MjData, rotor_thrusts_by_uav: list[list[float]]) -> None:
    flattened_commands = [thrust for rotor_thrusts in rotor_thrusts_by_uav for thrust in rotor_thrusts]
    data.ctrl[:] = flattened_commands
