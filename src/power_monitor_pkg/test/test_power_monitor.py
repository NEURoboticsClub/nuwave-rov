"""Tests for PowerMonitorPublisher"""

from unittest.mock import MagicMock, patch
import pytest
import rclpy

from power_monitor_pkg import power_monitor_pub
from power_monitor_pkg.power_monitor_pub import PowerMonitorPublisher, PowerMonitorInitError, main

class TestINA226:
    """Tests the initialization of the power monitor"""

    def test_init_ina226_i2c_address(self):
        """Tests that the default I2C address is set to 0x4F"""
        rclpy.init()
        mock_ina226 = MagicMock()
        try:
            with patch.object(power_monitor_pub, "INA226", return_value=mock_ina226):
                node = PowerMonitorPublisher()
            assert node.get_parameter("i2c_address").value == 0x4F
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_init_ina226_i2c_bus(self):
        """Tests that the default I2C bus is set to 1"""
        rclpy.init()
        mock_ina226 = MagicMock()
        try:
            with patch.object(power_monitor_pub, "INA226", return_value=mock_ina226):
                node = PowerMonitorPublisher()
            assert node.get_parameter("i2c_bus").value == 1
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_init_ina226_shunt_resistor(self):
        """Tests that the default shunt resistor is set to 0.004"""
        rclpy.init()
        mock_ina226 = MagicMock()
        try:
            with patch.object(power_monitor_pub, "INA226", return_value=mock_ina226):
                node = PowerMonitorPublisher()
            assert node.get_parameter("shunt_resistor").value == 0.004
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_init_ina226_max_expected_current(self):
        """Tests that the default maximum expected current is set to 17.0"""
        rclpy.init()
        mock_ina226 = MagicMock()
        try:
            with patch.object(power_monitor_pub, "INA226", return_value=mock_ina226):
                node = PowerMonitorPublisher()
            assert node.get_parameter("max_expected_current").value == 17.0
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_init_ina226_topic_prefix(self):
        """Tests that the default topic prefix is set to power_monitor"""
        rclpy.init()
        mock_ina226 = MagicMock()
        try:
            with patch.object(power_monitor_pub, "INA226", return_value=mock_ina226):
                node = PowerMonitorPublisher()
            assert node.get_parameter("topic_prefix").value == "power_monitor"
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_init_ina226_topic(self):
        """Tests that the default topic is set to power_monitor"""
        rclpy.init()
        mock_ina226 = MagicMock()
        try:
            with patch.object(power_monitor_pub, "INA226", return_value=mock_ina226):
                node = PowerMonitorPublisher()
            assert node.get_parameter("topic").value == "power_monitor"
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_init_ina226_configure(self):
        """Tests that the sensor is properly configured during initialization"""
        rclpy.init()
        mock_ina226 = MagicMock()
        try:
            with patch.object(power_monitor_pub, "INA226", return_value=mock_ina226):
                node = PowerMonitorPublisher()
            mock_ina226.configure.assert_called_once()
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_init_ina226_calibrate(self):
        """Tests that the sensor is properly calibrated with the expected current"""
        rclpy.init()
        mock_ina226 = MagicMock()
        try:
            with patch.object(power_monitor_pub, "INA226", return_value=mock_ina226):
                node = PowerMonitorPublisher()
            mock_ina226.calibrate.assert_called_once_with(max_expected_current=17.0)
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_init_ina226_failure(self):
        """Tests that initialization fails if the sensor can't connect"""
        rclpy.init()
        try:
            with patch.object(power_monitor_pub, "INA226",
                side_effect=RuntimeError("Connection failed")):
                with pytest.raises(PowerMonitorInitError):
                    PowerMonitorPublisher()
        finally:
            rclpy.shutdown()

class TestTimerCallback:
    """Tests the timer_callback method"""

    def test_timer_callback_publishes_measurements(self):
        """Tests that timer_callback publishes the measurements from the sensor properly"""
        rclpy.init()
        readings = {
            "bus_voltage": 12.5,
            "current": 2.0,
            "power": 25.0,
            "shunt_voltage": 0.008,
        }
        try:
            with patch.object(power_monitor_pub, "INA226") as mock_sensor:
                mock_sensor.return_value.read_all.return_value = readings
                node = PowerMonitorPublisher()
            node.pub_voltage = MagicMock()
            node.pub_current = MagicMock()
            node.pub_power = MagicMock()
            node.pub_shunt = MagicMock()

            node.timer_callback()

            assert node.pub_voltage.publish.call_args.args[0].data == 12.5
            assert node.pub_current.publish.call_args.args[0].data == 2.0
            assert node.pub_power.publish.call_args.args[0].data == 25.0
            assert node.pub_shunt.publish.call_args.args[0].data == 0.008
            node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_timer_callback_read_error(self):
        """Tests that timer_callback records a sensor error without publishing anything"""
        rclpy.init()
        try:
            with patch.object(power_monitor_pub, "INA226") as mock_sensor:
                mock_sensor.return_value.read_all.side_effect = RuntimeError("Sensor error")
                node = PowerMonitorPublisher()
            node.pub_voltage = MagicMock()
            node.pub_current = MagicMock()
            node.pub_power = MagicMock()
            node.pub_shunt = MagicMock()

            node.timer_callback()

            assert node.read_error_logged is True
            node.pub_voltage.publish.assert_not_called()
            node.pub_current.publish.assert_not_called()
            node.pub_power.publish.assert_not_called()
            node.pub_shunt.publish.assert_not_called()
            node.destroy_node()
        finally:
            rclpy.shutdown()

class TestMain:
    """Tests the main function"""

    def test_main_initializes_and_shuts_down(self, monkeypatch):
        """Tests that main starts the node and cleans up it up when done"""
        mock_node = MagicMock()
        monkeypatch.setattr(power_monitor_pub.rclpy, "init", MagicMock())
        monkeypatch.setattr(power_monitor_pub, "PowerMonitorPublisher",
                            MagicMock(return_value=mock_node))
        monkeypatch.setattr(power_monitor_pub.rclpy, "spin", MagicMock())
        monkeypatch.setattr(power_monitor_pub.rclpy, "shutdown", MagicMock())

        main()

        power_monitor_pub.rclpy.init.assert_called_once_with(args=None)
        power_monitor_pub.rclpy.spin.assert_called_once_with(mock_node)
        mock_node.destroy_node.assert_called_once()
        power_monitor_pub.rclpy.shutdown.assert_called_once()

    def test_main_handles_initialization_failure(self, monkeypatch):
        """Tests that main shuts down ROS if the node fails to properly initialize"""
        monkeypatch.setattr(power_monitor_pub.rclpy, "init", MagicMock())
        monkeypatch.setattr(power_monitor_pub, "PowerMonitorPublisher",
                            MagicMock(side_effect=PowerMonitorInitError()))
        monkeypatch.setattr(power_monitor_pub.rclpy, "shutdown", MagicMock())

        main()

        power_monitor_pub.rclpy.shutdown.assert_called_once()

if __name__ == "__main__":
    pytest.main()
