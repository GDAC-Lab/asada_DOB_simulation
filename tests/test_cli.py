import unittest

from qav_wheel.cli import build_cli_parser, resolve_simulation_ips


class CliParserTests(unittest.TestCase):
    def test_simulate_headless_duration_options_are_parsed(self) -> None:
        parser = build_cli_parser()

        arguments = parser.parse_args([
            'simulate',
            '--headless',
            '--duration-seconds',
            '4.5',
            '--num-uavs',
            '3',
        ])

        self.assertTrue(arguments.headless)
        self.assertEqual(arguments.duration_seconds, 4.5)
        self.assertEqual(arguments.num_uavs, 3)

    def test_simulate_fidelity_mode_is_parsed(self) -> None:
        parser = build_cli_parser()

        arguments = parser.parse_args([
            'simulate',
            '--fidelity-mode',
            'hil',
        ])

        self.assertEqual(arguments.fidelity_mode, 'hil')

    def test_simulate_ip_endpoints_can_be_split(self) -> None:
        parser = build_cli_parser()

        arguments = parser.parse_args([
            'simulate',
            '--bind-ip',
            '0.0.0.0',
            '--state-target-ip',
            '192.168.0.42',
        ])

        bind_ip, state_target_ip = resolve_simulation_ips(arguments)
        self.assertEqual(bind_ip, '0.0.0.0')
        self.assertEqual(state_target_ip, '192.168.0.42')

    def test_legacy_udp_ip_still_sets_both_endpoints(self) -> None:
        parser = build_cli_parser()

        arguments = parser.parse_args([
            'simulate',
            '--udp-ip',
            '10.0.0.5',
        ])

        bind_ip, state_target_ip = resolve_simulation_ips(arguments)
        self.assertEqual(bind_ip, '10.0.0.5')
        self.assertEqual(state_target_ip, '10.0.0.5')

    def test_hover_controller_options_are_parsed(self) -> None:
        parser = build_cli_parser()

        arguments = parser.parse_args([
            'hover-controller',
            '--bind-ip',
            '0.0.0.0',
            '--target-ip',
            '192.168.0.42',
            '--duration-seconds',
            '5',
            '--target-position',
            '0',
            '0',
            '1.8',
        ])

        self.assertEqual(arguments.bind_ip, '0.0.0.0')
        self.assertEqual(arguments.target_ip, '192.168.0.42')
        self.assertEqual(arguments.duration_seconds, 5.0)
        self.assertEqual(arguments.target_position, [0.0, 0.0, 1.8])

    def test_hover_controller_fidelity_mode_is_parsed(self) -> None:
        parser = build_cli_parser()

        arguments = parser.parse_args([
            'hover-controller',
            '--fidelity-mode',
            'hil',
        ])

        self.assertEqual(arguments.fidelity_mode, 'hil')


if __name__ == '__main__':
    unittest.main()