# mock_ms5837.py
import random

class MS5837_30BA:
    def __init__(self, bus=1):
        self.bus = bus
        self._pressure = 1013.25  # mbar at sea level
        self._temperature = 20.0  # °C
        self._initialized = False
    
    def init(self):
        # Mock: always succeeds without needing real I2C
        print(f"[MOCK] MS5837 initialized on simulated bus {self.bus}")
        self._initialized = True
        return True
    
    def read(self):
        if not self._initialized:
            return False
        # Simulate slight variations
        self._pressure += random.uniform(-0.5, 0.5)
        self._temperature += random.uniform(-0.1, 0.1)
        return True
    
    def pressure(self, conversion=1.0):
        return self._pressure * conversion
    
    def temperature(self):
        return self._temperature
    
    def depth(self):
        # Simulate depth: ~1 mbar per 1cm depth
        return max(0, (self._pressure - 1013.25) / 100.0)  # meters

# Add any other classes/constants you need
MS5837_02BA = MS5837_30BA
DENSITY_FRESHWATER = 997
DENSITY_SALTWATER = 1029
