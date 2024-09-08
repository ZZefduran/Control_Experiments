from plot_enums import PlotMode, HistNorm, HistDistribution


class PlotData:
    def __init__(
        self,
        name=None,
        x=None,
        y=None,
        y_err=None,
        y_err_minus=None,
        legendgroup=None,
        showlegend=True,
        color=None,
        line_color=None,
        line_width=None,
        color_range=(None, None),
        color_scale=None,
        opacity=None,
        marker_opacity=None,
        marker_size=None,
        mode: PlotMode = PlotMode.DEFAULT,
        subplot=(1, 1),
        text=None,
        fill="none",
        continuous_err=False,
        err_color=None,
        err_name=None,
    ) -> None:
        self.name = name
        self.color = color
        self.line_color = line_color
        self.line_width = line_width
        self.color_range = color_range
        self.color_scale = color_scale
        self.opacity = opacity
        self.marker_opacity = marker_opacity
        self.marker_size = marker_size
        self.mode = mode
        self.x = x
        self.y = y
        self.y_err = y_err
        self.y_err_minus = y_err_minus
        self.legendgroup = legendgroup
        self.showlegend = showlegend
        self.subplot = subplot
        self.text = text
        self.fill = fill
        self.continuous_err = continuous_err
        self.err_color = err_color if err_color is not None else color
        self.err_name = err_name if err_name is not None else f"{name} error"

class HistPlotData(PlotData):
    def __init__(self, nbins=None, hist_norm: HistNorm = HistNorm.PROBABILITY_DENSITY, distribution: HistDistribution= HistDistribution.NONE, **kwds):
        self.nbins = nbins
        self.hist_norm = hist_norm
        self.distribution = distribution
        super().__init__(**kwds)
        self.mode = PlotMode.HIST
