from houston_pkg.houston_functions import scale_controller_input, publish_boolean, _scale_to_unit, parse_joystick, joy_arm_callback, publish_twist_get_msg
import pytest
import numpy as np

def test_scale_controller_input():
    np.testing.assert_allclose(scale_controller_input(0.5), 0.2685281070883853, atol=1e-6)
    np.testing.assert_allclose(scale_controller_input(0.25), 0.1142138192803201, atol=1e-6)
    np.testing.assert_allclose(scale_controller_input(0.98387), 0.9572620235040501, atol=1e-6)

def test_publish_boolean():
    class MockPub:
        def __init__(self):
            self.msg = None
        def publish(self, msg):
            self.msg = msg

    mock_pub = MockPub()
    publish_boolean(True, mock_pub)
    assert mock_pub.msg == Bool(True)
    publish_boolean(False, mock_pub)
    assert mock_pub.msg == Bool(False)

def test_scale_to_unit():
    np.testing.assert_allclose(_scale_to_unit(1, 1, 1), (1, 1, 1), atol=1e-6)
    np.testing.assert_allclose(_scale_to_unit(.847, .193, .957), (.847, .193, .957), atol=1e-6)
    np.testing.assert_allclose(_scale_to_unit(1, 13, 5), (0.07692307692307693, 1.0, 0.38461538461538464), atol=1e-6)
    np.testing.assert_allclose(_scale_to_unit(28, 44, 44) ,(0.6363636363636364, 1.0, 1.0), atol=1e-6)


def test_parse_joystick():
    pass
    #assert parse_joystick(mock cfg_list : not a list, mock msg : any, expo_enabled : Boolean) is {"axis": {}, "button": {}}
    #assert parse_joystick(mock cfg_list : list, mock msg : any, expo_enabled : Boolean) is {"axis": (expected axis values), "button": (expected button values)}

def test_joy_arm_callback():
    pass
    # assert joy_arm_callback(mock msg : any) is (expected return_map)

def test_publish_twist_get_msg():
    pass
    # assert publish_twist_get_msg(mock axis_values : any, mock button_values : any)