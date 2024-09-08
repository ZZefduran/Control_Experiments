import numpy as np
from scipy import signal
from .single_wave_generator_base import SingleWaveGeneratorBase


class TrapezoidWaveGenerator(SingleWaveGeneratorBase):
    def __init__(self, sample_freq) -> None:
        super().__init__(sample_freq)
        self.bounds_ratio = 0.5

    def generate_wave(self, wave_freq, t):
        slope = 1 / (1 - self.bounds_ratio)
        wave = slope * signal.sawtooth(2 * np.pi * wave_freq * (t + 1 / (4 * wave_freq)), width=0.5)
        wave[wave > 1] = 1
        wave[wave < -1] = -1
        return wave
    
    def set_bounds(self, bounds_ratio):
        self.bounds_ratio = bounds_ratio
