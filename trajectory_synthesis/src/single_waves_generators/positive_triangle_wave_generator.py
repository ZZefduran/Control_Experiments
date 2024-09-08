import numpy as np
from scipy import signal
from .single_wave_generator_base import SingleWaveGeneratorBase


class PositiveTriangleWaveGenerator(SingleWaveGeneratorBase):
    def generate_wave(self, wave_freq, t):
        wave = abs(signal.sawtooth(2 * np.pi * wave_freq * (t + 1 / (4 * wave_freq)), width=0.5))
        return wave
