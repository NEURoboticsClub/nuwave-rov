import pyfirmata
from .pwm_scribe_base import PWMScribeBase

class PWMScribeArduino(PWMScribeBase):
    def __init__(self, port: str, pin_map: list[int]):
        self.port = port
        self.pin_map = pin_map
        self.board = None

    def setup(self):
        self.board = pyfirmata.ArduinoMega(self.port)
        for pin in self.pin_map:
            self.board.digital[pin].mode = pyfirmata.SERVO

    def set_pwm(self, channel: int, angle_deg: int):
        pin = self.pin_map[channel]
        self.board.digital[pin].write(angle_deg)

    def shutdown(self):
        if self.board:
            self.board.exit()