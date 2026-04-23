import sys
import types
import time

# --- Stub hardware BEFORE importing ThrusterNode ---
fake_gpio = types.ModuleType("Jetson.GPIO")
fake_gpio.cleanup = lambda: None
fake_jetson = types.ModuleType("Jetson")
fake_jetson.GPIO = fake_gpio
sys.modules["Jetson"] = fake_jetson
sys.modules["Jetson.GPIO"] = fake_gpio

class FakePCA9685:
    def __init__(self, bus=None, address=None): pass
    def setPWMFreq(self, f): pass
    def setRotationAngle(self, ch, ang): pass
    def exit_PCA9685(self): pass

# Replace with the actual package path where PCA9685.py lives
PKG = "thruster_pkg"
fake_mod = types.ModuleType(f"{PKG}.PCA9685")
fake_mod.PCA9685 = FakePCA9685
sys.modules[f"{PKG}.PCA9685"] = fake_mod

# Skip the 2s arming sleep in _arm()
time.sleep = lambda *_: None

import rclpy
from rclpy.executors import MultiThreadedExecutor
from thruster_pkg.new_thruster_driver import ThrusterNode


def main():
    rclpy.init()

    N = 8
    nodes = []
    for i in range(N):
        nodes.append(ThrusterNode(
            node_name=f'thruster_node_{i}',
            cli_args=[
                '--ros-args',
                '-p', f'channel:={i}',
                '-p', f'topic:=thruster/thruster_{i}',
            ],
        ))

    exe = MultiThreadedExecutor()
    for n in nodes:
        exe.add_node(n)

    try:
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()