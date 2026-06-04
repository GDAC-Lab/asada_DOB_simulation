from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import numpy as np

from .paths import DEFAULT_PATH_RESOLVER, PathResolver
from .surface import format_scalar, format_vector
from .surface_builder import build_initial_poses, build_surface_model_spec
from .types import InitialPoseSpec, RotorSpec, SensorNames, SurfaceEvaluator, UAVModelSpec

__all__ = [
    "build_sensor_names",
    "build_rotor_specs",
    "build_uav_model_specs",
    "build_sensor_block",
    "build_xml_replacements",
    "render_model_xml",
]


def _normalize_vector(values: list[float], field_name: str) -> tuple[float, float, float]:
    vector = np.asarray([float(value) for value in values], dtype=float)
    vector_norm = float(np.linalg.norm(vector))
    if vector.shape != (3,):
        raise ValueError(f"{field_name} must have exactly 3 elements")
    if vector_norm <= 1.0e-9:
        raise ValueError(f"{field_name} must not be the zero vector")
    normalized_vector = vector / vector_norm
    return float(normalized_vector[0]), float(normalized_vector[1]), float(normalized_vector[2])


def _parse_rotor_spec(rotor_config: dict[str, Any], rotor_index: int, default_yaw_moment_ratio: float) -> RotorSpec:
    rotor_suffix = str(rotor_config.get("name", "")).strip().lower()
    if not rotor_suffix:
        raise ValueError(f"actuation.rotors[{rotor_index}] must define a non-empty name")

    raw_position = rotor_config.get("position_body")
    if not isinstance(raw_position, list):
        raise ValueError(f"actuation.rotors[{rotor_index}].position_body must be a 3-element list")

    raw_thrust_axis = rotor_config.get("thrust_axis_body")
    if not isinstance(raw_thrust_axis, list):
        raise ValueError(f"actuation.rotors[{rotor_index}].thrust_axis_body must be a 3-element list")

    position = tuple(float(value) for value in raw_position)
    if len(position) != 3:
        raise ValueError(f"actuation.rotors[{rotor_index}].position_body must have exactly 3 elements")

    thrust_axis = _normalize_vector(raw_thrust_axis, f"actuation.rotors[{rotor_index}].thrust_axis_body")
    yaw_moment_ratio = float(rotor_config.get("yaw_moment_ratio", default_yaw_moment_ratio))
    spin_sign = float(rotor_config.get("spin_sign", 1.0))
    if abs(spin_sign) <= 1.0e-9:
        raise ValueError(f"actuation.rotors[{rotor_index}].spin_sign must be non-zero")

    return RotorSpec(
        suffix=rotor_suffix,
        position=position,
        thrust_axis=thrust_axis,
        yaw_moment_ratio=yaw_moment_ratio,
        spin_sign=1.0 if spin_sign > 0.0 else -1.0,
    )


def build_rotor_specs(params: dict[str, Any]) -> list[RotorSpec]:
    actuation = params["actuation"]
    default_yaw_moment_ratio = float(actuation["yaw_moment_ratio"])
    rotor_configs = actuation.get("rotors")
    if isinstance(rotor_configs, list) and len(rotor_configs) > 0:
        rotor_specs = [_parse_rotor_spec(rotor_config, rotor_index, default_yaw_moment_ratio) for rotor_index, rotor_config in enumerate(rotor_configs)]
        if len(rotor_specs) != 4:
            raise ValueError("actuation.rotors must define exactly 4 rotors to preserve the current controller interface")
        rotor_suffixes = [rotor_spec.suffix for rotor_spec in rotor_specs]
        if len(set(rotor_suffixes)) != len(rotor_suffixes):
            raise ValueError("actuation.rotors names must be unique")
        return rotor_specs

    drone = params["drone"]
    arm_x = float(drone["arm"]["x"])
    arm_y = float(drone["arm"]["y"])
    propeller_z = float(drone["propeller"]["z"])
    return [
        RotorSpec("fr", (arm_x, -arm_y, propeller_z), (0.0, 0.0, 1.0), default_yaw_moment_ratio, 1.0),
        RotorSpec("fl", (arm_x, arm_y, propeller_z), (0.0, 0.0, 1.0), default_yaw_moment_ratio, -1.0),
        RotorSpec("br", (-arm_x, -arm_y, propeller_z), (0.0, 0.0, 1.0), default_yaw_moment_ratio, -1.0),
        RotorSpec("bl", (-arm_x, arm_y, propeller_z), (0.0, 0.0, 1.0), default_yaw_moment_ratio, 1.0),
    ]


def build_sensor_names(params: dict[str, Any], body_name: str | None = None, sensor_prefix: str | None = None) -> SensorNames:
    sensor_items = params["sensors"]["items"]
    if body_name is None and sensor_prefix is None:
        return SensorNames(
            position=sensor_items["position"]["name"],
            linear_velocity=sensor_items["linear_velocity"]["name"],
            angular_velocity=sensor_items["angular_velocity"]["name"],
            x_axis=sensor_items["x_axis"]["name"],
            y_axis=sensor_items["y_axis"]["name"],
            z_axis=sensor_items["z_axis"]["name"],
        )

    prefix = sensor_prefix or body_name or "uav"
    return SensorNames(
        position=f"{prefix}_position",
        linear_velocity=f"{prefix}_linear_velocity",
        angular_velocity=f"{prefix}_angular_velocity",
        x_axis=f"{prefix}_x_axis",
        y_axis=f"{prefix}_y_axis",
        z_axis=f"{prefix}_z_axis",
    )


def build_uav_model_specs(params: dict[str, Any], num_uavs: int) -> list[UAVModelSpec]:
    base_name = str(params["drone"]["name"])
    rotor_suffixes = tuple(rotor_spec.suffix for rotor_spec in build_rotor_specs(params))
    specs: list[UAVModelSpec] = []
    for uav_index in range(num_uavs):
        if num_uavs == 1:
            body_name = base_name
            sensor_prefix = base_name
            contact_prefix = ""
        else:
            body_name = f"uav_{uav_index + 1}"
            sensor_prefix = body_name
            contact_prefix = f"{body_name}_"
        specs.append(
            UAVModelSpec(
                name=body_name,
                body_name=body_name,
                actuator_names=tuple(f"{body_name}_thrust_{suffix}" for suffix in rotor_suffixes),
                sensor_names=build_sensor_names(params, body_name=body_name, sensor_prefix=sensor_prefix),
                contact_prefix=contact_prefix,
            )
        )
    return specs


def _build_rotor_site_lines(
    rotor_specs: list[RotorSpec],
    prefix: str,
    propeller_radius: float,
    propeller_thickness: float,
    prop_rgba: list[float],
) -> list[str]:
    return [
        f'      <site name="{prefix}prop_{rotor_spec.suffix}" pos="{format_vector(list(rotor_spec.position))}" zaxis="{format_vector(list(rotor_spec.thrust_axis))}" type="cylinder" size="{format_vector([propeller_radius, propeller_thickness])}" rgba="{format_vector(prop_rgba)}"/>'
        for rotor_spec in rotor_specs
    ]


def _build_wheel_body_block(drone: dict[str, Any], prefix: str, side_name: str, wheel_offset_y: float) -> list[str]:
    wheel_direction = -1.0 if side_name == "right" else 1.0
    wheel_body_name = f"{prefix}{side_name}_wheel"
    wheel_joint_name = f"{prefix}joint_{'rw' if side_name == 'right' else 'lw'}"
    wheel_geom_name = f"{prefix}{side_name}_wheel_geom"
    return [
        f'      <body name="{wheel_body_name}" pos="{format_vector([0.0, wheel_direction * wheel_offset_y, 0.0])}">',
        f'        <joint name="{wheel_joint_name}" type="hinge" axis="0 1 0" damping="{format_scalar(drone["wheels"]["joint_damping"])}"/>',
        f'        <geom name="{wheel_geom_name}" type="cylinder" size="{format_vector([drone["wheels"]["radius"], drone["wheels"]["thickness"]])}" euler="90 0 0" mass="{format_scalar(drone["wheels"]["mass"])}" rgba="{format_vector(drone["wheels"]["rgba"])}" solref="{format_vector(drone["wheels"]["solref"])}" contype="{format_scalar(drone["wheels"]["contact"]["contype"])}" conaffinity="{format_scalar(drone["wheels"]["contact"]["conaffinity"])}"/>',
        "      </body>",
    ]


def build_sensor_block(params: dict[str, Any], uav_specs: list[UAVModelSpec]) -> str:
    sensor_config = params["sensors"]
    sensor_lines: list[str] = []
    item_types = {item_name: item["type"] for item_name, item in sensor_config["items"].items()}
    for uav_spec in uav_specs:
        sensor_lines.extend(
            [
                f'    <{item_types["position"]} name="{uav_spec.sensor_names.position}" objtype="xbody" objname="{uav_spec.body_name}"/>',
                f'    <{item_types["linear_velocity"]} name="{uav_spec.sensor_names.linear_velocity}" objtype="xbody" objname="{uav_spec.body_name}"/>',
                f'    <{item_types["angular_velocity"]} name="{uav_spec.sensor_names.angular_velocity}" objtype="xbody" objname="{uav_spec.body_name}"/>',
                f'    <{item_types["x_axis"]} name="{uav_spec.sensor_names.x_axis}" objtype="xbody" objname="{uav_spec.body_name}"/>',
                f'    <{item_types["y_axis"]} name="{uav_spec.sensor_names.y_axis}" objtype="xbody" objname="{uav_spec.body_name}"/>',
                f'    <{item_types["z_axis"]} name="{uav_spec.sensor_names.z_axis}" objtype="xbody" objname="{uav_spec.body_name}"/>',
            ]
        )
    return "\n".join(sensor_lines)


def _build_uav_color(uav_index: int, alpha: float) -> list[float]:
    palette = (
        [0.92, 0.24, 0.24],
        [0.18, 0.54, 0.95],
        [0.20, 0.72, 0.38],
        [0.94, 0.66, 0.18],
        [0.61, 0.31, 0.88],
    )
    red, green, blue = palette[uav_index % len(palette)]
    return [red, green, blue, alpha]


def _build_drone_body_block(params: dict[str, Any], rotor_specs: list[RotorSpec], uav_specs: list[UAVModelSpec], initial_poses: list[InitialPoseSpec]) -> str:
    drone = params["drone"]
    propeller_radius = float(drone["propeller"]["radius"])
    propeller_thickness = float(drone["propeller"]["thickness"])
    wheel_offset_y = float(drone["wheels"]["offset_y"])

    body_blocks: list[str] = []
    for uav_index, (uav_spec, initial_pose) in enumerate(zip(uav_specs, initial_poses, strict=True)):
        prefix = uav_spec.contact_prefix
        body_box_rgba = _build_uav_color(uav_index, float(drone["body_box"]["rgba"][3]))
        prop_rgba = _build_uav_color(uav_index, 0.55)
        rotor_site_lines = _build_rotor_site_lines(rotor_specs, prefix, propeller_radius, propeller_thickness, prop_rgba)
        body_attributes = [f'name="{uav_spec.body_name}"', f'pos="{format_vector(initial_pose.position)}"']
        if initial_pose.quaternion is not None:
            body_attributes.append(f'quat="{format_vector(initial_pose.quaternion)}"')
        body_blocks.append(
            "\n".join(
                [
                    f'    <body {" ".join(body_attributes)}>',
                    '      <joint type="free"/>',
                    *rotor_site_lines,
                    f'      <geom type="mesh" mesh="drone_cad" contype="{format_scalar(drone["mesh"]["contact"]["contype"])}" conaffinity="{format_scalar(drone["mesh"]["contact"]["conaffinity"])}" mass="0" group="{format_scalar(drone["mesh"]["contact"]["group"])}"/>',
                    f'      <geom name="{prefix}drone_body_box" type="box" size="{format_vector(drone["body_box"]["size"])}" euler="{format_vector(drone["body_box"]["euler"])}" mass="{format_scalar(drone["body_box"]["mass"])}" group="{format_scalar(drone["body_box"]["contact"]["group"])}" rgba="{format_vector(body_box_rgba)}" contype="{format_scalar(drone["body_box"]["contact"]["contype"])}" conaffinity="{format_scalar(drone["body_box"]["contact"]["conaffinity"])}"/>',
                    *_build_wheel_body_block(drone, prefix, "right", wheel_offset_y),
                    *_build_wheel_body_block(drone, prefix, "left", wheel_offset_y),
                    '    </body>',
                ]
            )
        )
    return "\n\n".join(body_blocks)


def _build_actuator_block(rotor_specs: list[RotorSpec], uav_specs: list[UAVModelSpec]) -> str:
    actuator_lines: list[str] = []
    for uav_spec in uav_specs:
        prefix = uav_spec.contact_prefix
        for actuator_name, rotor_spec in zip(uav_spec.actuator_names, rotor_specs, strict=True):
            torque_axis = [rotor_spec.spin_sign * rotor_spec.yaw_moment_ratio * axis_component for axis_component in rotor_spec.thrust_axis]
            gear_vector = [*rotor_spec.thrust_axis, *torque_axis]
            actuator_lines.append(
                f'    <motor name="{actuator_name}" site="{prefix}prop_{rotor_spec.suffix}" gear="{format_vector(gear_vector)}"/>'
            )
    return "\n".join(actuator_lines)


def _resolve_mesh_file_reference(params: dict[str, Any], template_path: Path, output_path: Path) -> str:
    raw_mesh_file = Path(str(params["drone"]["mesh"]["file"]))
    if raw_mesh_file.is_absolute():
        return format_scalar(raw_mesh_file.as_posix())

    mesh_file_path = (template_path.parent / raw_mesh_file).resolve()
    relative_mesh_path = Path(os.path.relpath(mesh_file_path, start=output_path.parent))
    return format_scalar(relative_mesh_path.as_posix())


def build_xml_replacements(params: dict[str, Any], num_uavs: int = 1, spawn_radius: float = 1.5) -> tuple[dict[str, str], SurfaceEvaluator | None, list[UAVModelSpec]]:
    simulation = params["simulation"]
    actuation = params["actuation"]
    environment = params["environment"]
    surface_spec = build_surface_model_spec(environment)
    rotor_specs = build_rotor_specs(params)
    uav_specs = build_uav_model_specs(params, num_uavs)
    initial_poses = build_initial_poses(params["drone"], surface_spec, num_uavs, spawn_radius)

    replacements = {
        "__STATISTIC_EXTENT__": format_scalar(environment["statistic_extent"]),
        "__STATISTIC_CENTER__": format_vector(environment["statistic_center"]),
        "__GRAVITY__": format_vector(simulation["gravity"]),
        "__TIMESTEP__": format_scalar(simulation["timestep"]),
        "__MAX_ROTOR_THRUST__": format_scalar(actuation["max_rotor_thrust"]),
        "__MESH_FILE__": format_scalar(params["drone"]["mesh"]["file"]),
        "__MESH_SCALE__": format_vector(params["drone"]["mesh"]["scale"]),
        "__SURFACE_ASSET_BLOCK__": surface_spec.asset_block,
        "__SURFACE_GEOM_BLOCK__": surface_spec.geom_block,
        "__WALL_POS__": format_vector(environment["wall_position"]),
        "__WALL_SIZE__": format_vector(environment["wall_size"]),
        "__WALL_RGBA__": format_vector(environment["wall_rgba"]),
        "__WALL_SOLREF__": format_vector(environment["wall_solref"]),
        "__WALL_CONTYPE__": format_scalar(environment["wall_contact"]["contype"]),
        "__WALL_CONAFFINITY__": format_scalar(environment["wall_contact"]["conaffinity"]),
        "__DRONE_BODY_BLOCK__": _build_drone_body_block(params, rotor_specs, uav_specs, initial_poses),
        "__ACTUATOR_BLOCK__": _build_actuator_block(rotor_specs, uav_specs),
        "__SENSOR_BLOCK__": build_sensor_block(params, uav_specs),
    }
    return replacements, surface_spec.evaluator, uav_specs


def render_model_xml(
    params: dict[str, Any],
    instance_id: int = 0,
    output_path: Path | None = None,
    num_uavs: int = 1,
    spawn_radius: float = 1.5,
    template_path: str | Path | None = None,
    generated_xml_dir: str | Path | None = None,
    path_resolver: PathResolver | None = None,
) -> tuple[Path, SurfaceEvaluator | None, list[UAVModelSpec]]:
    resolver = path_resolver or DEFAULT_PATH_RESOLVER
    resolved_template_path = resolver.get_xml_template_path(template_path)
    resolved_output_path = output_path or resolver.get_generated_xml_path(instance_id, output_directory=generated_xml_dir)
    template_text = resolved_template_path.read_text(encoding="utf-8")
    rendered_text = template_text
    replacements, surface_evaluator, uav_specs = build_xml_replacements(params, num_uavs=num_uavs, spawn_radius=spawn_radius)
    replacements["__MESH_FILE__"] = _resolve_mesh_file_reference(params, resolved_template_path, resolved_output_path)
    for placeholder, value in replacements.items():
        rendered_text = rendered_text.replace(placeholder, value)
    resolved_output_path.parent.mkdir(parents=True, exist_ok=True)
    resolved_output_path.write_text(rendered_text, encoding="utf-8")
    return resolved_output_path, surface_evaluator, uav_specs
