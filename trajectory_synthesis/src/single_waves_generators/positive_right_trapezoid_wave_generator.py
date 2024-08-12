import numpy as np
from .single_wave_generator_base import SingleWaveGeneratorBase


class PositiveRightTrapezoidWaveGenerator(SingleWaveGeneratorBase):
    def generate_wave(self, wave_freq, t):
        wave = np.empty_like(t)
        n = len(wave)
        for i in range(n):
            if i < n/6 or i > 5*n/6:
                val = 0
            elif i < 0.5*n:
                val = 3*i/n - 0.5
            else:
                val = 1
            wave[i] = val
        return wave
