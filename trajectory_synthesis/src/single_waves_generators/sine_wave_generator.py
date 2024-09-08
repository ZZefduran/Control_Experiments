import numpy as np
from .single_wave_generator_base import SingleWaveGeneratorBase


class SineWaveGenerator(SingleWaveGeneratorBase):
    def generate_wave(self, wave_freq, t):
        wave = np.sin(2 * np.pi * wave_freq * t)
        return wave
