"""Verify configuration errors enter ROS logging without a ROS runtime."""
import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock

import pytest


@pytest.fixture
def helper(monkeypatch):
    logger = Mock()
    logging_module = ModuleType('rclpy.logging')
    logging_module.get_logger = Mock(return_value=logger)
    ros = ModuleType('rclpy')
    ros.logging = logging_module
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setitem(sys.modules, 'rclpy.logging', logging_module)
    source = Path(__file__).parents[1] / 'nuwave_utils_pkg' / 'file_helpers.py'
    spec = importlib.util.spec_from_file_location('_yaml_logging_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, logger


@pytest.mark.parametrize('content, expected', [('motors: [1, 2]', {'motors': [1, 2]}), ('', {})])
def test_valid_and_empty_yaml_remain_compatible(helper, tmp_path, content, expected):
    module, logger = helper
    path = tmp_path / 'config.yaml'
    path.write_text(content)
    assert module.load_yaml(str(path)) == expected
    logger.error.assert_not_called()


@pytest.mark.parametrize('failure', ['missing', 'malformed', 'unreadable'])
def test_yaml_errors_use_ros_logger_with_path_and_context(helper, tmp_path, monkeypatch, capsys, failure):
    module, logger = helper
    path = tmp_path / 'config.yaml'
    if failure != 'missing':
        path.write_text('motors: [')
    if failure == 'unreadable':
        monkeypatch.setattr(module, 'open', Mock(side_effect=PermissionError('denied')), raising=False)
    assert module.load_yaml(str(path), '[PWM]') == {}
    logger.error.assert_called_once()
    message = logger.error.call_args.args[0]
    assert str(path) in message and '[PWM]' in message
    assert capsys.readouterr().out == ''
