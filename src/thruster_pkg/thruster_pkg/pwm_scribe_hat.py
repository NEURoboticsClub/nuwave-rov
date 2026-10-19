from .pwm_scribe_base import PWMScribeBase
from .pca9685_driver import PCA9685

class PWMScribeHat(PWMScribeBase):
    def __init__(self, bus, addr):
        self.board = PCA9685(bus=bus, address=addr)

    def setup(self, pwm_freq: int):
        if pwm_freq != 50:
            raise ValueError(f'PWM hat requires 50 Hz, got {pwm_freq} Hz')
        self.board.setPWMFreq(pwm_freq)

    def set_pwm(self, channel: int, pwm_us: float):
        """Write microseconds directly; the board pulse writer requires 50 Hz."""
        try:
            self.board.setServoPulse(channel, pwm_us)
        except Exception as e:
            print(f'Failed to send pulse to channel {channel}: {e}')

    def shutdown(self):
        self.board.exit_PCA9685()
