from __future__ import annotations

import argparse

from qav_wheel.simulation import run_simulation


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a bounded headless MuJoCo simulation")
    parser.add_argument("--instance-id", type=int, default=0)
    parser.add_argument("--num-uavs", type=int, default=1)
    parser.add_argument("--spawn-radius", type=float, default=1.5)
    parser.add_argument("--params-file", default=None)
    parser.add_argument("--generated-xml-dir", default=None)
    parser.add_argument("--duration-seconds", type=float, required=True)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    run_simulation(
        instance_id=arguments.instance_id,
        num_uavs=arguments.num_uavs,
        spawn_radius=arguments.spawn_radius,
        params_path=arguments.params_file,
        generated_xml_dir=arguments.generated_xml_dir,
        headless=True,
        duration_seconds=arguments.duration_seconds,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())