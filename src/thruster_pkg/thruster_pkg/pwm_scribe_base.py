from abc import ABC, abstractmethod

class PWMScribeBase(ABC):
    @abstractmethod
    def setup(self) -> None:
        ...

    @abstractmethod
    def set_pwm(self, channel: int, pulse_us: int) -> None:
        ...

    @abstractmethod
    def shutdown(self) -> None:
        ...