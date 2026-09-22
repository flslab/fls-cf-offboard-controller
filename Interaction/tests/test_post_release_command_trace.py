import unittest
from unittest.mock import MagicMock, patch

from Interaction.post_release_command_trace import ReleaseCommandTrace
from Interaction.post_release_pi_planner import PiEventPlanner


class CommandTraceTests(unittest.TestCase):
    def test_compact_optional_block_and_cleanup(self):
        cf = MagicMock()
        trace = ReleaseCommandTrace(cf)
        with patch('Interaction.post_release_command_trace.LogConfig') as config:
            trace.start()
            config.assert_called_once_with(name='ReleaseTargets', period_in_ms=200)
            self.assertEqual(config.return_value.add_variable.call_count, 7)
            trace._sample(123, {'controller.roll': 2.5}, None)
            rows = trace.drain()
            self.assertEqual(rows[-1]['fc_timestamp_ms'], 123)
            self.assertFalse(rows[-1]['source_snapshot_atomic'])
            trace.close()
            config.return_value.delete.assert_called_once()
            trace._sample(124, {}, None)
            self.assertEqual(trace.drain(), [])

    def test_missing_toc_is_diagnostic_only(self):
        cf = MagicMock()
        cf.log.add_config.side_effect = KeyError('controller.roll')
        trace = ReleaseCommandTrace(cf)
        trace.start()
        self.assertEqual(trace.drain()[0]['event'], 'unavailable')
        trace.close()

    def test_bounded_buffers_report_loss(self):
        trace = ReleaseCommandTrace(MagicMock())
        for i in range(520):
            trace._sample(i, {}, None)
        self.assertEqual(trace.drain()[0]['dropped'], 8)
        planner = PiEventPlanner(MagicMock())
        with planner._lock:
            for i in range(130):
                planner._record_diagnostic('test', value=i)
        rows = planner.drain_diagnostics()
        self.assertEqual(rows[0]['dropped'], 2)
        self.assertEqual(rows[-1]['value'], 129)
        self.assertEqual(planner.drain_diagnostics(), [])


if __name__ == '__main__':
    unittest.main()
