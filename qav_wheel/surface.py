from __future__ import annotations

import math
from typing import Any

import numpy as np

from .paths import SURFACE_GEOM_NAME, SURFACE_HFIELD_NAME
from .types import InitialPoseSpec, SurfaceEvaluator

__all__ = [
    "format_scalar",
    "format_vector",
    "get_surface_config",
    "build_surface_evaluator",
    "surface_is_approximately_flat",
    "surface_can_use_plane_geom",
    "evaluate_height_function",
    "evaluate_height_gradient",
    "evaluate_surface_properties",
    "evaluate_surface_normal",
    "build_surface_blocks",
    "get_drone_initial_pose",
    "get_drone_initial_position",
]


def format_scalar(value: Any) -> str:
    if isinstance(value, str):
        return value
    return f"{float(value):g}"


def format_vector(values: list[float]) -> str:
    return " ".join(format_scalar(value) for value in values)


def _build_xml_attributes(attributes: dict[str, str | None]) -> str:
    return " ".join(
        f'{attribute_name}="{attribute_value}"'
        for attribute_name, attribute_value in attributes.items()
        if attribute_value is not None and attribute_value != ""
    )


def _copy_surface_config(surface_config: dict[str, Any]) -> dict[str, Any]:
    copied_config = dict(surface_config)

    plane_config = copied_config.get("plane")
    if isinstance(plane_config, dict):
        copied_config["plane"] = dict(plane_config)

    contact_config = copied_config.get("contact")
    if isinstance(contact_config, dict):
        copied_config["contact"] = dict(contact_config)

    height_function_config = copied_config.get("height_function")
    if isinstance(height_function_config, dict):
        copied_height_function_config = dict(height_function_config)
        parameters = copied_height_function_config.get("parameters")
        if isinstance(parameters, dict):
            copied_height_function_config["parameters"] = dict(parameters)
        copied_config["height_function"] = copied_height_function_config

    return copied_config


def get_surface_config(environment: dict[str, Any]) -> dict[str, Any]:
    if "surface" in environment:
        surface_config = _copy_surface_config(environment["surface"])
        surface_mode = str(surface_config.get("mode", "")).strip().lower()
        if surface_mode:
            if surface_mode in {"plane", "floor"}:
                surface_config["type"] = "plane"
            elif surface_mode in {"flat", "slope", "paraboloid", "sinusoidal", "gaussian"}:
                surface_config["type"] = "height_function"
                surface_config.setdefault("height_function", {})["name"] = surface_mode
            elif surface_mode != "height_function":
                raise ValueError(f"Unsupported environment.surface.mode: {surface_mode}")
        return surface_config

    return {
        "type": "plane",
        "material": "floor_mat",
        "solref": environment["floor_solref"],
        "contact": environment["floor_contact"],
        "plane": {"size": environment["floor_size"]},
    }


def _build_surface_geom_attributes(surface_config: dict[str, Any]) -> dict[str, str | None]:
    contact = surface_config["contact"]
    attributes: dict[str, str | None] = {
        "name": SURFACE_GEOM_NAME,
        "solref": format_vector(surface_config["solref"]),
        "contype": format_scalar(contact["contype"]),
        "conaffinity": format_scalar(contact["conaffinity"]),
    }
    if "material" in surface_config:
        attributes["material"] = format_scalar(surface_config["material"])
    if "rgba" in surface_config:
        attributes["rgba"] = format_vector(surface_config["rgba"])
    return attributes


def _get_surface_function_parameters(surface_config: dict[str, Any]) -> dict[str, float]:
    function_config = surface_config["height_function"]
    raw_parameters = function_config.get("parameters", {})
    return {parameter_name: float(parameter_value) for parameter_name, parameter_value in raw_parameters.items()}


def build_surface_evaluator(surface_config: dict[str, Any]) -> SurfaceEvaluator | None:
    if surface_config["type"] != "height_function":
        return None

    function_config = surface_config["height_function"]
    return SurfaceEvaluator(
        kind=str(function_config["name"]),
        parameters=_get_surface_function_parameters(surface_config),
    )


def _get_surface_parameter(parameters: dict[str, float], parameter_name: str, default_value: float = 0.0) -> float:
    return float(parameters.get(parameter_name, default_value))


def _validate_gaussian_sigma(sigma_x: float, sigma_y: float) -> None:
    if sigma_x <= 0.0 or sigma_y <= 0.0:
        raise ValueError("gaussian sigma_x and sigma_y must be positive")


def _evaluate_surface_terms(surface_evaluator: SurfaceEvaluator, x_coord: float, y_coord: float) -> tuple[float, float, float]:
    parameters = surface_evaluator.parameters
    z_offset = _get_surface_parameter(parameters, "z_offset")

    if surface_evaluator.kind == "flat":
        return z_offset, 0.0, 0.0
    if surface_evaluator.kind == "slope":
        slope_x = _get_surface_parameter(parameters, "slope_x")
        slope_y = _get_surface_parameter(parameters, "slope_y")
        return z_offset + slope_x * x_coord + slope_y * y_coord, slope_x, slope_y
    if surface_evaluator.kind == "paraboloid":
        curvature_x = _get_surface_parameter(parameters, "curvature_x")
        curvature_y = _get_surface_parameter(parameters, "curvature_y")
        return z_offset + curvature_x * x_coord**2 + curvature_y * y_coord**2, 2.0 * curvature_x * x_coord, 2.0 * curvature_y * y_coord
    if surface_evaluator.kind == "sinusoidal":
        amplitude = _get_surface_parameter(parameters, "amplitude")
        frequency_x = _get_surface_parameter(parameters, "frequency_x", 1.0)
        frequency_y = _get_surface_parameter(parameters, "frequency_y", 1.0)
        phase_x = _get_surface_parameter(parameters, "phase_x")
        phase_y = _get_surface_parameter(parameters, "phase_y")
        phase_x_value = frequency_x * x_coord + phase_x
        phase_y_value = frequency_y * y_coord + phase_y
        sin_x = math.sin(phase_x_value)
        cos_x = math.cos(phase_x_value)
        sin_y = math.sin(phase_y_value)
        cos_y = math.cos(phase_y_value)
        height = z_offset + amplitude * sin_x * sin_y
        return height, amplitude * frequency_x * cos_x * sin_y, amplitude * frequency_y * sin_x * cos_y
    if surface_evaluator.kind == "gaussian":
        amplitude = _get_surface_parameter(parameters, "amplitude")
        center_x = _get_surface_parameter(parameters, "center_x")
        center_y = _get_surface_parameter(parameters, "center_y")
        sigma_x = _get_surface_parameter(parameters, "sigma_x", 1.0)
        sigma_y = _get_surface_parameter(parameters, "sigma_y", 1.0)
        _validate_gaussian_sigma(sigma_x, sigma_y)
        x_term = (x_coord - center_x) / sigma_x
        y_term = (y_coord - center_y) / sigma_y
        gaussian_value = amplitude * math.exp(-0.5 * (x_term * x_term + y_term * y_term))
        height = z_offset + gaussian_value
        return height, -gaussian_value * (x_coord - center_x) / (sigma_x**2), -gaussian_value * (y_coord - center_y) / (sigma_y**2)

    raise ValueError(f"Unsupported height_function name: {surface_evaluator.kind}")


def _evaluate_height_function_grid(surface_evaluator: SurfaceEvaluator, x_values: np.ndarray, y_values: np.ndarray) -> np.ndarray:
    grid_x, grid_y = np.meshgrid(x_values, y_values, indexing="xy")
    parameters = surface_evaluator.parameters
    z_offset = _get_surface_parameter(parameters, "z_offset")

    if surface_evaluator.kind == "flat":
        return np.full_like(grid_x, z_offset, dtype=float)
    if surface_evaluator.kind == "slope":
        slope_x = _get_surface_parameter(parameters, "slope_x")
        slope_y = _get_surface_parameter(parameters, "slope_y")
        return z_offset + slope_x * grid_x + slope_y * grid_y
    if surface_evaluator.kind == "paraboloid":
        curvature_x = _get_surface_parameter(parameters, "curvature_x")
        curvature_y = _get_surface_parameter(parameters, "curvature_y")
        return z_offset + curvature_x * np.square(grid_x) + curvature_y * np.square(grid_y)
    if surface_evaluator.kind == "sinusoidal":
        amplitude = _get_surface_parameter(parameters, "amplitude")
        frequency_x = _get_surface_parameter(parameters, "frequency_x", 1.0)
        frequency_y = _get_surface_parameter(parameters, "frequency_y", 1.0)
        phase_x = _get_surface_parameter(parameters, "phase_x")
        phase_y = _get_surface_parameter(parameters, "phase_y")
        return z_offset + amplitude * np.sin(frequency_x * grid_x + phase_x) * np.sin(frequency_y * grid_y + phase_y)
    if surface_evaluator.kind == "gaussian":
        amplitude = _get_surface_parameter(parameters, "amplitude")
        center_x = _get_surface_parameter(parameters, "center_x")
        center_y = _get_surface_parameter(parameters, "center_y")
        sigma_x = _get_surface_parameter(parameters, "sigma_x", 1.0)
        sigma_y = _get_surface_parameter(parameters, "sigma_y", 1.0)
        _validate_gaussian_sigma(sigma_x, sigma_y)
        exponent = -0.5 * (np.square((grid_x - center_x) / sigma_x) + np.square((grid_y - center_y) / sigma_y))
        return z_offset + amplitude * np.exp(exponent)

    raise ValueError(f"Unsupported height_function name: {surface_evaluator.kind}")


def surface_is_approximately_flat(surface_evaluator: SurfaceEvaluator, tolerance: float = 1.0e-12) -> bool:
    parameters = surface_evaluator.parameters
    if surface_evaluator.kind == "flat":
        return True
    if surface_evaluator.kind == "paraboloid":
        return (
            abs(_get_surface_parameter(parameters, "curvature_x")) <= tolerance
            and abs(_get_surface_parameter(parameters, "curvature_y")) <= tolerance
        )
    if surface_evaluator.kind == "sinusoidal":
        return abs(_get_surface_parameter(parameters, "amplitude")) <= tolerance
    return False


def surface_can_use_plane_geom(surface_evaluator: SurfaceEvaluator) -> bool:
    return surface_evaluator.kind == "slope" or surface_is_approximately_flat(surface_evaluator)


def evaluate_height_function(surface_evaluator: SurfaceEvaluator, x_coord: float, y_coord: float) -> float:
    height, _, _ = _evaluate_surface_terms(surface_evaluator, x_coord, y_coord)
    return height


def evaluate_height_gradient(surface_evaluator: SurfaceEvaluator, x_coord: float, y_coord: float) -> tuple[float, float]:
    _, dh_dx, dh_dy = _evaluate_surface_terms(surface_evaluator, x_coord, y_coord)
    return dh_dx, dh_dy


def evaluate_surface_properties(surface_evaluator: SurfaceEvaluator, x_coord: float, y_coord: float) -> tuple[float, np.ndarray]:
    height, dh_dx, dh_dy = _evaluate_surface_terms(surface_evaluator, x_coord, y_coord)
    normal = np.array([-dh_dx, -dh_dy, 1.0], dtype=float)
    return height, normal / np.linalg.norm(normal)


def evaluate_surface_normal(surface_evaluator: SurfaceEvaluator, x_coord: float, y_coord: float) -> np.ndarray:
    _, normal = evaluate_surface_properties(surface_evaluator, x_coord, y_coord)
    return normal


def _get_height_function_grid(surface_config: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray, SurfaceEvaluator]:
    function_config = surface_config["height_function"]
    surface_evaluator = build_surface_evaluator(surface_config)
    if surface_evaluator is None:
        raise ValueError("Surface evaluator is required for height_function surfaces")

    x_min, x_max = [float(value) for value in function_config["x_range"]]
    y_min, y_max = [float(value) for value in function_config["y_range"]]
    grid_x, grid_y = [int(value) for value in function_config["grid_resolution"]]
    if grid_x < 2 or grid_y < 2:
        raise ValueError("height_function grid_resolution must be at least [2, 2]")

    x_values = np.linspace(x_min, x_max, grid_x)
    y_values = np.linspace(y_min, y_max, grid_y)
    heights = _evaluate_height_function_grid(surface_evaluator, x_values, y_values)

    return x_values, y_values, heights, surface_evaluator


def _get_surface_plane_size(surface_config: dict[str, Any]) -> list[float]:
    plane_config = surface_config.get("plane")
    if isinstance(plane_config, dict) and "size" in plane_config:
        return [float(value) for value in plane_config["size"]]

    function_config = surface_config["height_function"]
    x_min, x_max = [float(value) for value in function_config["x_range"]]
    y_min, y_max = [float(value) for value in function_config["y_range"]]
    return [0.5 * (x_max - x_min), 0.5 * (y_max - y_min), 0.1]


def _build_plane_surface_blocks(
    surface_config: dict[str, Any],
    surface_geom_attributes: dict[str, str | None],
    surface_evaluator: SurfaceEvaluator | None,
) -> tuple[str, str, SurfaceEvaluator | None]:
    plane_size = _get_surface_plane_size(surface_config)
    geom_attributes = {
        **surface_geom_attributes,
        "type": "plane",
        "size": format_vector(plane_size),
    }
    if surface_evaluator is not None:
        origin_height = evaluate_height_function(surface_evaluator, 0.0, 0.0)
        geom_attributes["pos"] = format_vector([0.0, 0.0, origin_height])
        geom_attributes["zaxis"] = format_vector(evaluate_surface_normal(surface_evaluator, 0.0, 0.0).tolist())

    geom_block = f'    <geom {_build_xml_attributes(geom_attributes)}/>'
    return "", geom_block, surface_evaluator


def _build_hfield_surface_blocks(
    surface_config: dict[str, Any],
    surface_geom_attributes: dict[str, str | None],
) -> tuple[str, str, SurfaceEvaluator]:
    x_values, y_values, heights, surface_evaluator = _get_height_function_grid(surface_config)
    min_height = float(np.min(heights))
    max_height = float(np.max(heights))
    height_span = max_height - min_height
    if height_span <= 1.0e-12:
        return _build_plane_surface_blocks(surface_config, surface_geom_attributes, surface_evaluator)

    normalized_heights = (heights - min_height) / height_span
    # MuJoCo's XML parser is sensitive to extreme scientific-notation values in
    # hfield elevation attributes, so collapse numerically insignificant tails.
    np.clip(normalized_heights, 0.0, 1.0, out=normalized_heights)
    normalized_heights[normalized_heights < 1.0e-12] = 0.0
    normalized_heights[normalized_heights > 1.0 - 1.0e-12] = 1.0
    elevation_values = " ".join(f"{value:.9g}" for value in normalized_heights.reshape(-1))
    x_radius = 0.5 * float(x_values[-1] - x_values[0])
    y_radius = 0.5 * float(y_values[-1] - y_values[0])
    center_x = 0.5 * float(x_values[0] + x_values[-1])
    center_y = 0.5 * float(y_values[0] + y_values[-1])
    base_thickness = max(float(surface_config["height_function"].get("base_thickness", 0.1)), 1.0e-4)

    asset_attributes = {
        "name": SURFACE_HFIELD_NAME,
        "nrow": format_scalar(int(len(y_values))),
        "ncol": format_scalar(int(len(x_values))),
        "size": format_vector([x_radius, y_radius, height_span, base_thickness]),
        "elevation": elevation_values,
    }
    geom_attributes = {
        **surface_geom_attributes,
        "type": "hfield",
        "hfield": SURFACE_HFIELD_NAME,
        "pos": format_vector([center_x, center_y, min_height]),
    }
    asset_block = f'    <hfield {_build_xml_attributes(asset_attributes)}/>'
    geom_block = f'    <geom {_build_xml_attributes(geom_attributes)}/>'
    return asset_block, geom_block, surface_evaluator


def build_surface_blocks(surface_config: dict[str, Any]) -> tuple[str, str, SurfaceEvaluator | None]:
    surface_type = str(surface_config["type"])
    surface_geom_attributes = _build_surface_geom_attributes(surface_config)

    if surface_type == "plane":
        return _build_plane_surface_blocks(surface_config, surface_geom_attributes, None)

    if surface_type == "height_function":
        surface_evaluator = build_surface_evaluator(surface_config)
        if surface_evaluator is None:
            raise ValueError("Surface evaluator is required for height_function surfaces")
        if surface_can_use_plane_geom(surface_evaluator):
            return _build_plane_surface_blocks(surface_config, surface_geom_attributes, surface_evaluator)
        return _build_hfield_surface_blocks(surface_config, surface_geom_attributes)

    raise ValueError(f"Unsupported environment.surface.type: {surface_type}")


def _wheel_contact_height(
    surface_evaluator: SurfaceEvaluator,
    x_coord: float,
    y_coord: float,
    wheel_offset_y: float,
    wheel_radius: float,
    roll_angle: float,
    side_sign: float,
) -> float:
    contact_y = y_coord + side_sign * wheel_offset_y * math.cos(roll_angle) + wheel_radius * math.sin(roll_angle)
    return evaluate_height_function(surface_evaluator, x_coord, contact_y)


def _solve_initial_roll_angle(
    surface_evaluator: SurfaceEvaluator,
    x_coord: float,
    y_coord: float,
    wheel_offset_y: float,
    wheel_radius: float,
) -> float:
    if wheel_offset_y <= 0.0:
        return 0.0

    max_roll = math.radians(80.0)

    def residual(roll_angle: float) -> float:
        left_height = _wheel_contact_height(surface_evaluator, x_coord, y_coord, wheel_offset_y, wheel_radius, roll_angle, 1.0)
        right_height = _wheel_contact_height(surface_evaluator, x_coord, y_coord, wheel_offset_y, wheel_radius, roll_angle, -1.0)
        return left_height - right_height - 2.0 * wheel_offset_y * math.sin(roll_angle)

    sample_angles = np.linspace(-max_roll, max_roll, 257)
    sample_residuals = [residual(float(angle)) for angle in sample_angles]
    best_index = min(range(len(sample_residuals)), key=lambda index: abs(sample_residuals[index]))
    best_angle = float(sample_angles[best_index])

    for lower_index in range(len(sample_angles) - 1):
        lower_residual = sample_residuals[lower_index]
        upper_residual = sample_residuals[lower_index + 1]
        if lower_residual == 0.0:
            return float(sample_angles[lower_index])
        if lower_residual * upper_residual > 0.0:
            continue

        lower_angle = float(sample_angles[lower_index])
        upper_angle = float(sample_angles[lower_index + 1])
        for _ in range(60):
            midpoint_angle = 0.5 * (lower_angle + upper_angle)
            midpoint_residual = residual(midpoint_angle)
            if abs(midpoint_residual) <= 1.0e-10:
                return midpoint_angle
            if lower_residual * midpoint_residual <= 0.0:
                upper_angle = midpoint_angle
                upper_residual = midpoint_residual
            else:
                lower_angle = midpoint_angle
                lower_residual = midpoint_residual
        return 0.5 * (lower_angle + upper_angle)

    return best_angle


def _get_initial_wheel_contact_clearance(surface_config: dict[str, Any], drone: dict[str, Any]) -> float:
    configured_clearance = surface_config.get("initial_wheel_contact_clearance")
    if configured_clearance is not None:
        return float(configured_clearance)

    base_initial_position = [float(value) for value in drone["initial_position"]]
    wheel_radius = float(drone["wheels"]["radius"])
    return base_initial_position[2] - wheel_radius


def _get_initial_pitch_angle(surface_evaluator: SurfaceEvaluator, x_coord: float, y_coord: float) -> float:
    dh_dx, _ = evaluate_height_gradient(surface_evaluator, x_coord, y_coord)
    return -math.atan(dh_dx)


def _compose_roll_pitch_quaternion(roll_angle: float, pitch_angle: float) -> tuple[float, float, float, float]:
    half_roll = 0.5 * roll_angle
    half_pitch = 0.5 * pitch_angle
    cos_roll = math.cos(half_roll)
    sin_roll = math.sin(half_roll)
    cos_pitch = math.cos(half_pitch)
    sin_pitch = math.sin(half_pitch)
    return (
        cos_roll * cos_pitch,
        sin_roll * cos_pitch,
        cos_roll * sin_pitch,
        sin_roll * sin_pitch,
    )


def get_drone_initial_pose(
    drone: dict[str, Any],
    surface_evaluator: SurfaceEvaluator | None,
    surface_config: dict[str, Any],
    x_coord: float | None = None,
    y_coord: float | None = None,
) -> InitialPoseSpec:
    base_initial_position = [float(value) for value in drone["initial_position"]]
    resolved_x_coord = base_initial_position[0] if x_coord is None else float(x_coord)
    resolved_y_coord = base_initial_position[1] if y_coord is None else float(y_coord)
    initial_position = [resolved_x_coord, resolved_y_coord, base_initial_position[2]]
    follow_surface_for_initial_position = bool(surface_config.get("follow_surface_for_initial_position", True))
    if not follow_surface_for_initial_position or surface_evaluator is None:
        return InitialPoseSpec(position=initial_position)

    wheel_config = drone["wheels"]
    wheel_offset_y = float(wheel_config["offset_y"])
    wheel_radius = float(wheel_config["radius"])
    roll_angle = _solve_initial_roll_angle(surface_evaluator, resolved_x_coord, resolved_y_coord, wheel_offset_y, wheel_radius)
    pitch_angle = _get_initial_pitch_angle(surface_evaluator, resolved_x_coord, resolved_y_coord)
    left_height = _wheel_contact_height(surface_evaluator, resolved_x_coord, resolved_y_coord, wheel_offset_y, wheel_radius, roll_angle, 1.0)
    right_height = _wheel_contact_height(surface_evaluator, resolved_x_coord, resolved_y_coord, wheel_offset_y, wheel_radius, roll_angle, -1.0)
    base_clearance = _get_initial_wheel_contact_clearance(surface_config, drone)
    initial_position[2] = base_clearance + 0.5 * (left_height + right_height) + wheel_radius * math.cos(roll_angle)
    quaternion = _compose_roll_pitch_quaternion(roll_angle, pitch_angle)
    return InitialPoseSpec(position=initial_position, quaternion=quaternion)


def get_drone_initial_position(
    drone: dict[str, Any],
    surface_evaluator: SurfaceEvaluator | None,
    surface_config: dict[str, Any],
) -> list[float]:
    return get_drone_initial_pose(drone, surface_evaluator, surface_config).position
