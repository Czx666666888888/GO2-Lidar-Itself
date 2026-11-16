"""Mock atomic parameter acknowledgement; no ROS graph or hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('outcome', ['ok', 'absent', 'timeout', 'rejected'])
def test_round_service_requires_bounded_success(monkeypatch, outcome):
    import rclpy
    path = ROOT/'camera_ws/scripts/advance_yolo_round.py'
    spec = importlib.util.spec_from_file_location('advance_round_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    future = NS(done=lambda: outcome != 'timeout', exception=lambda: None,
                result=lambda: NS(result=NS(successful=outcome != 'rejected', reason='bad history')))
    client = NS(wait_for_service=Mock(return_value=outcome != 'absent'),
                call_async=Mock(return_value=future))
    node = NS(create_client=Mock(return_value=client), destroy_node=Mock())
    monkeypatch.setattr(rclpy, 'init', Mock())
    monkeypatch.setattr(rclpy, 'create_node', lambda *a: node)
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', Mock())
    monkeypatch.setattr(rclpy, 'shutdown', Mock())
    if outcome == 'ok':
        module.main(['[[1.0, 2.0]]'])
        request = client.call_async.call_args.args[0]
        assert request.parameters[0].name == 'completed_targets_json'
        assert request.parameters[0].value.string_value == '[[1.0, 2.0]]'
    else:
        with pytest.raises(RuntimeError):
            module.main(['[[1.0, 2.0]]'])
    node.create_client.assert_called_once()
    assert node.create_client.call_args.args[1] == '/camera_target_to_map/set_parameters_atomically'
    client.wait_for_service.assert_called_once_with(timeout_sec=3.)
    node.destroy_node.assert_called_once()
    rclpy.shutdown.assert_called_once()
