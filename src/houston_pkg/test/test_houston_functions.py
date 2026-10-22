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
    # assert publish_twist_get_msg(mock axis_values : any, mock button_values : any) is (expected returned msg : Twist)

def test_publish_arm_commands():
    pass
    # assert publish_arm_commands(mock axis_values : any, mock button_values : any) is (expected returned msg : Float32MultiArray)

def test_set_button_enabled():
    pass
    #class MockEnabled:
        # def __init__(self):
            #self.enabled = none
            #self.inputString = none
        # def set-enabled(new-enabled-val : boolean, inString : String):
            #self.enabled = new-enabled-val
            #self.inputString = inString
    
    #mock_enabled = MockEnabled()
    #set_button_enabled(mock button_values : list, test-specific toggle : String, test-specific prev_button : int, mock_enabled.set-enabled, test-specific already_enabled : boolean)
    # assert mock_enabled.enabled is (expected enabled)
    # assert mock_enabled.inputString is (expected inputString)

def test_check_if_stale_and_set_msg():
    pass
    #class MockLogger:
        #def __init__(self):
            #self.warning = none
        #def warn(s : String):
            #self.warning = s
    #class MockStabilizeEnabler:
        #def __init__(self):
            #self.ran_stabilize_enabler = none
        #def set_stabilize_enabled
            #self.ran_stabilize_enabler = true
    #class MockMSG:
    #mock_logger = MockLogger()
    #mock_stabilize_enabler = MockStabilizeEnabler()
    # First test condition where stale
    # check_if_stale_and_set_msg(mock clock : any, test-specific last_stabilizer_time : any, test-specific last_stabilizer_twist : any, mock stabilizer_timeout : any, mock_stabilize_enabler.set_stabilize_enabled, mock_logger, mock_msg)
    # assert mock_logger.warning is "Stabilizer messages stale; stabilization disabled"
    # assert mock_stabilize_enabler.ran_stabilize_enabler is true