from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

__all__ = [
    "ActuatorDynamicsConfig",
    "FidelityConfig",
    "InitialPoseSpec",
    "LoggingConfig",
    "NetworkFidelityConfig",
    "RotorSpec",
    "SensorFidelityConfig",
    "SensorLayout",
    "SensorNames",
    "SurfaceEvaluator",
    "SurfaceModelSpec",
    "UAVModelSpec",
]


@dataclass(frozen=True)
class NetworkFidelityConfig:
    enabled: bool = False
    state_tx_latency_ms: float = 0.0
    command_rx_latency_ms: float = 0.0
    packet_loss_percent: float = 0.0
    jitter_std_dev_ms: float = 0.0
    stale_command_threshold_ms: float | None = None
    stale_command_policy: Literal["hold-last-command", "zero-thrust", "hover-fallback"] = "hold-last-command"


@dataclass(frozen=True)
class ActuatorDynamicsConfig:
    motor_tau_ms: float = 0.0
    thrust_rate_limit_n_per_s: float | None = None
    omega_rate_limit_rad_per_s: float | None = None


@dataclass(frozen=True)
class SensorFidelityConfig:
    position_noise_std_m: float = 0.0
    velocity_noise_std_m_per_s: float = 0.0
    angular_velocity_noise_std_rad_per_s: float = 0.0
    attitude_noise_std_rad: float = 0.0


@dataclass(frozen=True)
class LoggingConfig:
    log_network_stats: bool = False
    log_actuator_stats: bool = False
    log_sensor_truth: bool = False


@dataclass(frozen=True)
class FidelityConfig:
    mode: Literal["baseline", "hil"] = "baseline"
    network: NetworkFidelityConfig = NetworkFidelityConfig()
    actuator_dynamics: ActuatorDynamicsConfig = ActuatorDynamicsConfig()
    sensor_fidelity: SensorFidelityConfig = SensorFidelityConfig()
    logging: LoggingConfig = LoggingConfig()


@dataclass(frozen=True)
class InitialPoseSpec:
    position: list[float]
    quaternion: tuple[float, float, float, float] | None = None


@dataclass(frozen=True)
class RotorSpec:
    suffix: str
    position: tuple[float, float, float]
    thrust_axis: tuple[float, float, float]
    yaw_moment_ratio: float
    spin_sign: float


@dataclass(frozen=True)
class SensorLayout:
    position: slice
    linear_velocity: slice
    angular_velocity: slice
    x_axis: slice
    y_axis: slice
    z_axis: slice


@dataclass(frozen=True)
class SensorNames:
    position: str
    linear_velocity: str
    angular_velocity: str
    x_axis: str
    y_axis: str
    z_axis: str


@dataclass(frozen=True)
class SurfaceEvaluator:
    kind: str
    parameters: dict[str, float]


@dataclass(frozen=True)
class SurfaceModelSpec:
    config: dict[str, Any]
    asset_block: str
    geom_block: str
    evaluator: SurfaceEvaluator | None


@dataclass(frozen=True)
class UAVModelSpec:
    name: str
    body_name: str
    actuator_names: tuple[str, str, str, str]
    sensor_names: SensorNames
    contact_prefix: str
