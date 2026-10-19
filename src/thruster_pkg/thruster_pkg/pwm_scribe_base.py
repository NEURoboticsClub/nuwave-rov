from abc import ABC, abstractmethod

class PWMScribeBase(ABC):
    @abstractmethod
    def setup(self, pwm_freq : int) -> None:
        ...

    @abstractmethod
    def set_pwm(self, channel: int, pwm_us: float) -> None:
        ...

    @abstractmethod
    def shutdown(self) -> None:
        ...