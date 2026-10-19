from .pwm_scribe_base import PWMScribeBase
from .PCA9685 import PCA9685

class PWMScribeHat(PWMScribeBase):
    def __init__(self, bus, addr):
        self.board = PCA9685(bus=bus, address=addr)

    def setup(self, pwm_freq : int):
        self.board.setPWMFreq(pwm_freq)

    def set_pwm(self, channel: int, pwm_us: float):

        us_centered = pwm_us - 1500.0 # TODO: un-hard code this
        angle = ((us_centered / 500.0) * 90.0) + 90.0 # TODO: un-hard code this
        
        angle = int(max(0.0, min(180.0, angle)))
        # print(f'Angle: {angle}')

        try:
            self.board.setRotationAngle(channel, angle)
        except Exception as e:
            print(f'Failed to send angle to channel {channel}: {e}')

    def shutdown(self):
        self.board.exit_PCA9685()