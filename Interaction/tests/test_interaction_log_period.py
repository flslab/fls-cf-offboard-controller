"""Period compatibility checks for the deployed FLS logging protocol."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from cflib.crazyflie.log import LogConfig
from Interaction.log_manager import InteractionLogger


class FakeCallbacks:
    def __init__(self):
        self.callbacks = []

    def add_callback(self, callback):
        self.callbacks.append(callback)


class FakeLogConfig:
    instances = []

    def __init__(self, name, period_in_ms):
        self.name = name
        self.period_in_ms = period_in_ms
        self.variables = []
        self.data_received_cb = FakeCallbacks()
        self.started = False
        self.instances.append(self)

    def add_variable(self, name, value_type):
        self.variables.append((name, value_type))

    def start(self):
        self.started = True


class InteractionLogPeriodTests(unittest.TestCase):
    def setUp(self):
        FakeLogConfig.instances = []
        self.logger = InteractionLogger.__new__(InteractionLogger)
        self.logger.cf_log_data = None
        self.logger.cf_var_logger = None
        self.logger._cf_log_group_callback = Mock()
        self.cf = SimpleNamespace(log=SimpleNamespace(add_config=Mock()))

    def initialize(self, config, default_period=10):
        with patch('Interaction.log_manager.LogConfig', FakeLogConfig):
            self.logger.init_cf_logger(
                self.cf, config, cf_log_period=default_period
            )
        return FakeLogConfig.instances

    def test_default_period_matches_ordinary_logger_compensation(self):
        blocks = self.initialize({
            'STATE': {'stateEstimate.x': {'type': 'float', 'data': []}},
        })

        self.assertEqual(len(blocks), 1)
        self.assertEqual(blocks[0].period_in_ms, 100)
        self.assertEqual(
            blocks[0].variables, [('stateEstimate.x', 'float')]
        )
        self.assertTrue(blocks[0].started)

    def test_installed_cflib_encodes_compensated_period_as_legacy_byte(self):
        self.assertEqual(LogConfig('STATE', period_in_ms=100).period, 10)

    def test_group_override_is_consumed_without_mutating_source(self):
        config = {
            'STATE': {
                'log_period_ms': 10,
                'stateEstimate.x': {'type': 'float', 'data': []},
            },
        }
        blocks = self.initialize(config, default_period=20)

        self.assertEqual(blocks[0].period_in_ms, 100)
        self.assertNotIn('log_period_ms', self.logger.cf_log_data['STATE'])
        self.assertEqual(config['STATE']['log_period_ms'], 10)
        self.assertEqual(
            blocks[0].variables, [('stateEstimate.x', 'float')]
        )


if __name__ == '__main__':
    unittest.main()
