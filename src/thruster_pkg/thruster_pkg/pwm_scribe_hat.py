from .pwm_scribe_base import PWMScribeBase
from .PCA9685 import PCA9685

class PWMScribeHat(PWMScribeBase):
    def __init__(self, bus, addr):
        self.board = PCA9685(bus=bus, address=addr)

    def setup(self):
        ...

    def set_pwm(self, channel: int, pulse_us: int):
        ...

    def shutdown(self):
        ...