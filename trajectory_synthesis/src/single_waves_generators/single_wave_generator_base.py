from abc import ABC, abstractmethod

import numpy as np


class SingleWaveGeneratorBase(ABC):
    def __init__(self, sample_freq) -> None:
        self.sample_freq = sample_freq

    def generate(self, wave_freq):
        t = np.arange(self.sample_freq/wave_freq) / self.sample_freq
        return self.generate_wave(wave_freq, t)

    @abstractmethod
    def generate_wave(self, wave_freq, t):
        pass
