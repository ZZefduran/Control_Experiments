from .sine_wave_generator import SineWaveGenerator
from .trapezoid_wave_generator import TrapezoidWaveGenerator
from .positive_triangle_wave_generator import PositiveTriangleWaveGenerator
from .positive_right_trapezoid_wave_generator import PositiveRightTrapezoidWaveGenerator

class SingleWaveGeneratorFactory:
    def get(shape):
        shapes = {
            "sine": SineWaveGenerator,
            "trapezoid": TrapezoidWaveGenerator,
            "positive_triangle": PositiveTriangleWaveGenerator,
            "positive_right_trapezoid": PositiveRightTrapezoidWaveGenerator
        }

        if shape not in shapes:
            raise WaveShapeException(
                f"{shape} is not a valid wave type. Should be one of {list(shapes.keys())}")

        return shapes[shape]

class WaveShapeException(Exception):
    pass