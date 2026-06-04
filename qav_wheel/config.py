from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

from .paths import DEFAULT_PATH_RESOLVER, PathResolver
from .types import ActuatorDynamicsConfig, FidelityConfig, LoggingConfig, NetworkFidelityConfig, SensorFidelityConfig

__all__ = ["build_fidelity_config", "clear_vehicle_params_cache", "load_vehicle_params"]


@lru_cache(maxsize=1)
def _load_vehicle_params_cached(resolved_params_path: str) -> dict[str, Any]:
    with Path(resolved_params_path).open("r", encoding="utf-8") as params_file:
        return json.load(params_file)


def clear_vehicle_params_cache() -> None:
    _load_vehicle_params_cached.cache_clear()


def load_vehicle_params(
    params_path: str | Path | None = None,
    *,
    path_resolver: PathResolver | None = None,
) -> dict[str, Any]:
    resolved_params_path = (path_resolver or DEFAULT_PATH_RESOLVER).get_params_path(params_path)
    return _load_vehicle_params_cached(str(resolved_params_path))


def _get_bool(config_section: dict[str, Any], field_name: str, default_value: bool) -> bool:
    raw_value = config_section.get(field_name, default_value)
    return bool(raw_value)


def _get_float(config_section: dict[str, Any], field_name: str, default_value: float) -> float:
    raw_value = config_section.get(field_name, default_value)
    return float(raw_value)


def _get_optional_float(config_section: dict[str, Any], field_name: str) -> float | None:
    raw_value = config_section.get(field_name)
    if raw_value is None:
        return None
    return float(raw_value)


def build_fidelity_config(params: dict[str, Any]) -> FidelityConfig:
    raw_fidelity_mode = params.get("fidelity_mode", "baseline")
    if isinstance(raw_fidelity_mode, dict):
        fidelity_mode = str(raw_fidelity_mode.get("mode", raw_fidelity_mode.get("baseline", "baseline")))
    else:
        fidelity_mode = str(raw_fidelity_mode)
    if fidelity_mode not in {"baseline", "hil"}:
        raise ValueError(f"Unsupported fidelity mode: {fidelity_mode}")

    network_section = params.get("network_fidelity", {})
    if not isinstance(network_section, dict):
        network_section = {}

    actuator_section = params.get("actuator_dynamics", {})
    if not isinstance(actuator_section, dict):
        actuator_section = {}

    sensor_section = params.get("sensor_fidelity", {})
    if not isinstance(sensor_section, dict):
        sensor_section = {}

    logging_section = params.get("logging_config", {})
    if not isinstance(logging_section, dict):
        logging_section = {}

    stale_policy = str(network_section.get("stale_command_policy", "hold-last-command"))
    if stale_policy not in {"hold-last-command", "zero-thrust", "hover-fallback"}:
        raise ValueError(f"Unsupported stale command policy: {stale_policy}")

    return FidelityConfig(
        mode=fidelity_mode,
        network=NetworkFidelityConfig(
            enabled=_get_bool(network_section, "enabled", False),
            state_tx_latency_ms=_get_float(network_section, "state_tx_latency_ms", 0.0),
            command_rx_latency_ms=_get_float(network_section, "command_rx_latency_ms", 0.0),
            packet_loss_percent=_get_float(network_section, "packet_loss_percent", 0.0),
            jitter_std_dev_ms=_get_float(network_section, "jitter_std_dev_ms", 0.0),
            stale_command_threshold_ms=_get_optional_float(network_section, "stale_command_threshold_ms"),
            stale_command_policy=stale_policy,
        ),
        actuator_dynamics=ActuatorDynamicsConfig(
            motor_tau_ms=_get_float(actuator_section, "motor_tau_ms", 0.0),
            thrust_rate_limit_n_per_s=_get_optional_float(actuator_section, "thrust_rate_limit_n_per_s"),
            omega_rate_limit_rad_per_s=_get_optional_float(actuator_section, "omega_rate_limit_rad_per_s"),
        ),
        sensor_fidelity=SensorFidelityConfig(
            position_noise_std_m=_get_float(sensor_section, "position_noise_std_m", 0.0),
            velocity_noise_std_m_per_s=_get_float(sensor_section, "velocity_noise_std_m_per_s", 0.0),
            angular_velocity_noise_std_rad_per_s=_get_float(sensor_section, "angular_velocity_noise_std_rad_per_s", 0.0),
            attitude_noise_std_rad=_get_float(sensor_section, "attitude_noise_std_rad", 0.0),
        ),
        logging=LoggingConfig(
            log_network_stats=_get_bool(logging_section, "log_network_stats", False),
            log_actuator_stats=_get_bool(logging_section, "log_actuator_stats", False),
            log_sensor_truth=_get_bool(logging_section, "log_sensor_truth", False),
        ),
    )
