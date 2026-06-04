from __future__ import annotations

import argparse

from .network import PORT_RECV, PORT_SEND
from .paths import DEFAULT_GENERATED_XML_DIR
from .python_controller import run_hover_controller
from .simulation import run_simulation, validate_model

__all__ = ["build_cli_parser", "main"]


def resolve_simulation_ips(arguments: argparse.Namespace) -> tuple[str, str]:
    legacy_udp_ip = arguments.udp_ip or "127.0.0.1"
    bind_ip = arguments.bind_ip or legacy_udp_ip
    state_target_ip = arguments.state_target_ip or legacy_udp_ip
    return bind_ip, state_target_ip


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MuJoCo wheeled UAV simulator utilities")
    parser.set_defaults(instance_id=0, num_uavs=1, spawn_radius=1.5, recv_port=None, send_port=None, bind_ip=None, state_target_ip=None, udp_ip=None, params_file=None, xml_template_file=None, generated_xml_dir=None, fidelity_mode="baseline", headless=False, duration_seconds=None)
    subparsers = parser.add_subparsers(dest="command")

    simulate_parser = subparsers.add_parser("simulate", help="Run the MuJoCo simulator with viewer")
    check_model_parser = subparsers.add_parser("check-model", help="Render the XML/model and exit")
    hover_controller_parser = subparsers.add_parser("hover-controller", help="Run a Python hover controller that communicates over UDP")

    for subparser in (simulate_parser, check_model_parser):
        subparser.add_argument("--instance-id", type=int, default=0, help="Simulation instance id used to derive default ports and XML output")
        subparser.add_argument("--num-uavs", type=int, default=1, help="Number of UAVs to place in a single MuJoCo world")
        subparser.add_argument("--spawn-radius", type=float, default=1.5, help="Radius used to place multiple UAVs around the origin")
        subparser.add_argument("--params-file", default=None, help="Path to a vehicle_params.json file to load instead of the repository default")
        subparser.add_argument("--xml-template-file", default=None, help="Path to a MuJoCo XML template file to use instead of qav_wheel.template.xml")
        subparser.add_argument("--generated-xml-dir", default=None, help=f"Directory to write generated XML files into instead of {DEFAULT_GENERATED_XML_DIR}")
        subparser.add_argument("--fidelity-mode", choices=("baseline", "hil"), default="baseline", help="Select whether the run is tagged as baseline physics mode or HIL mode")

    simulate_parser.add_argument("--recv-port", type=int, default=None, help=f"UDP port to receive commands on (default: {PORT_RECV} + 2 * instance-id)")
    simulate_parser.add_argument("--send-port", type=int, default=None, help=f"UDP port to send state on (default: {PORT_SEND} + 2 * instance-id)")
    simulate_parser.add_argument("--bind-ip", default=None, help="Local IP address to bind the simulator command socket to")
    simulate_parser.add_argument("--state-target-ip", default=None, help="Destination IP address used when sending simulator state packets")
    simulate_parser.add_argument("--udp-ip", default=None, help="Legacy shorthand that sets both --bind-ip and --state-target-ip")
    simulate_parser.add_argument("--headless", action="store_true", help="Run without opening the MuJoCo viewer")
    simulate_parser.add_argument("--duration-seconds", type=float, default=None, help="Optional simulation duration limit in seconds")

    hover_controller_parser.add_argument("--instance-id", type=int, default=0, help="Controller instance id used to derive default ports")
    hover_controller_parser.add_argument("--bind-ip", default="127.0.0.1", help="Local IP address to bind the controller state-receive socket to")
    hover_controller_parser.add_argument("--target-ip", default="127.0.0.1", help="Destination IP address used when sending control commands to the simulator")
    hover_controller_parser.add_argument("--local-port", type=int, default=None, help=f"UDP port to receive simulator state on (default: {PORT_SEND} + 2 * instance-id)")
    hover_controller_parser.add_argument("--target-port", type=int, default=None, help=f"UDP port to send control commands to (default: {PORT_RECV} + 2 * instance-id)")
    hover_controller_parser.add_argument("--params-file", default=None, help="Path to a vehicle_params.json file to load instead of the repository default")
    hover_controller_parser.add_argument("--target-position", nargs=3, type=float, metavar=("X", "Y", "Z"), default=[0.0, 0.0, 1.5], help="Hover target position in world coordinates")
    hover_controller_parser.add_argument("--duration-seconds", type=float, default=None, help="Optional controller run duration limit in simulation seconds")
    hover_controller_parser.add_argument("--state-timeout-seconds", type=float, default=10.0, help="Maximum wall-clock time to wait for state packets before stopping")
    hover_controller_parser.add_argument("--status-display-interval", type=float, default=2.0, help="Interval for controller status printouts in simulation seconds")
    hover_controller_parser.add_argument("--fidelity-mode", choices=("baseline", "hil"), default="baseline", help="Tag outgoing controller packets as baseline physics mode or HIL mode")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_cli_parser()
    arguments = parser.parse_args(argv)
    command = arguments.command or "simulate"

    if arguments.instance_id < 0:
        parser.error("--instance-id must be non-negative")
    if arguments.num_uavs <= 0:
        parser.error("--num-uavs must be positive")
    if arguments.spawn_radius <= 0.0:
        parser.error("--spawn-radius must be positive")
    if arguments.duration_seconds is not None and arguments.duration_seconds <= 0.0:
        parser.error("--duration-seconds must be positive")
    if hasattr(arguments, "state_timeout_seconds") and arguments.state_timeout_seconds <= 0.0:
        parser.error("--state-timeout-seconds must be positive")
    if hasattr(arguments, "status_display_interval") and arguments.status_display_interval <= 0.0:
        parser.error("--status-display-interval must be positive")

    if command == "simulate":
        bind_ip, state_target_ip = resolve_simulation_ips(arguments)
        run_simulation(
            instance_id=arguments.instance_id,
            recv_port=arguments.recv_port,
            send_port=arguments.send_port,
            bind_ip=bind_ip,
            state_target_ip=state_target_ip,
            num_uavs=arguments.num_uavs,
            spawn_radius=arguments.spawn_radius,
            params_path=arguments.params_file,
            template_path=arguments.xml_template_file,
            generated_xml_dir=arguments.generated_xml_dir,
            fidelity_mode=arguments.fidelity_mode,
            headless=arguments.headless,
            duration_seconds=arguments.duration_seconds,
        )
        return 0
    if command == "check-model":
        return validate_model(
            instance_id=arguments.instance_id,
            num_uavs=arguments.num_uavs,
            spawn_radius=arguments.spawn_radius,
            params_path=arguments.params_file,
            template_path=arguments.xml_template_file,
            generated_xml_dir=arguments.generated_xml_dir,
            fidelity_mode=arguments.fidelity_mode,
        )
    if command == "hover-controller":
        run_hover_controller(
            instance_id=arguments.instance_id,
            bind_ip=arguments.bind_ip,
            target_ip=arguments.target_ip,
            local_port=arguments.local_port,
            target_port=arguments.target_port,
            params_path=arguments.params_file,
            target_position=arguments.target_position,
            duration_seconds=arguments.duration_seconds,
            state_timeout_seconds=arguments.state_timeout_seconds,
            status_display_interval=arguments.status_display_interval,
            fidelity_mode=arguments.fidelity_mode,
        )
        return 0

    parser.error(f"Unsupported command: {command}")
    return 2
