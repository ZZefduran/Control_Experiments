from enum import Enum


class PlotMode:
    DEFAULT = "lines+markers"
    LINES = "lines"
    SCATTER = "markers"
    HLINE = "hline"
    VLINE = "vline"
    VRECT = "vrect"
    HIST = "histogram"


class PlotColors:
    TARGET = "#2ca02c"
    REAL = "#1f77b4"
    SIM = "#ff7f0e"
    SIM_ERR = "#FFDFBF"


class HistNorm(Enum):
    PERCENT = "percent"
    PROBABILITY = "probability"
    DENSITY = "density"
    PROBABILITY_DENSITY = "probability density"

class HistDistribution(Enum):
    NONE = 0
    AUTO = 1
    NORMAL = 2
    LAPLACE = 3
    UNIFORM = 4
