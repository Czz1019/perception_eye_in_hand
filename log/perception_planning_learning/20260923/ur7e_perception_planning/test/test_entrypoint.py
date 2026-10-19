"""Distinguish a DDS take interrupted by shutdown from a live runtime fault."""
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from ur7e_perception_planning_core import entrypoint


@pytest.mark.parametrize('context_alive', [False, True])
def test_take_error_during_shutdown_and_live_context(monkeypatch, context_alive):
    node = Mock()
    executor = Mock()
    executor.spin.side_effect = RuntimeError('Unable to convert call argument to Python object')
    ros = SimpleNamespace(init=Mock(), ok=lambda: context_alive, shutdown=Mock())
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', SimpleNamespace(
        SingleThreadedExecutor=lambda: executor,
        ExternalShutdownException=type('ExternalShutdownException', (Exception,), {})))
    monkeypatch.setitem(sys.modules, 'ur7e_perception_planning_core.mapper',
                        SimpleNamespace(PlanningMapper=lambda: node))
    monkeypatch.setattr(sys, 'argv', ['perception_planning', 'mapper'])
    if context_alive:
        with pytest.raises(RuntimeError, match='Unable to convert'):
            entrypoint.main()
    else:
        entrypoint.main()
    node.close.assert_called_once()
    node.destroy_node.assert_called_once()
    executor.shutdown.assert_called_once()
