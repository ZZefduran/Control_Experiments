from plotter.plot_data import PlotData
from plotter.plot_format import PlotFormat
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.express as px
import os
from functools import cached_property
from plot_enums import PlotMode, HistDistribution, HistNorm
import math
import statistics
import numpy as np
from scipy import stats


class Plotter:
    def __init__(self, dir, legendgroups=False) -> None:
        self.dir = dir
        self.data = []
        self.format = []
        self.subplots = (1, 1)
        self.legendgroups = legendgroups

    def add_data(self, data: PlotData):
        self._update_subplots_num(data.subplot)
        self.data.append(data)

    def add_format(self, format: PlotFormat):
        self.format.append(format)

    def _update_subplots_num(self, subplot):
        self.subplots = (max(self.subplots[0], subplot[0]), max(self.subplots[1], subplot[1]))

    def save(self, fig: go.Figure, name):
        file_name = name.lower().replace(" - ", "_").replace(" ", "_").replace("/", "_").replace(":", "_")
        html_file = os.path.join(self.dir, f"{file_name}.html")
        # png_file = os.path.join(self.dir, f"{file_name}.png")
        # fig.write_image(png_file, width=1500, height=750, scale=4)
        fig.write_html(html_file)

    def add_std(self,fig: go.Figure, plot_data: PlotData):
        std = np.sqrt(plot_data.std)
        name = plot_data.name
        row, col = plot_data.subplot
        fig.add_trace(
            go.Scatter(
            x=plot_data.x+plot_data.x[::-1],
            y=plot_data.y+std+(plot_data.y-std)[::-1],

            line_color=plot_data.color,
            fill='toself',
            fillcolor='rgba(0,100,80,0.2)',
            line=dict(color='rgba(255,255,255,0)'),
            hoverinfo="skip",
            showlegend=False
            ),# annotation_text=name,
            # line_dash="dash",
            row=row,
            col=col)

    def plot(self, title, headnote=None, save=True, shared_x=True, show=True, aggregate_legend=False, file_name=None):
        fig = make_subplots(
            rows=self.subplots[0],
            cols=self.subplots[1],
            shared_xaxes=shared_x,
            subplot_titles=self._subp_titles_placeholder,
            horizontal_spacing=0.05,
            vertical_spacing=0.05,
        )
        fig.update_layout(
            title_text=f"<b>{title}</b>",
            title_font_size=20,
            showlegend=True,
            legend_tracegroupgap=25,
            barmode="overlay"
        )
        if aggregate_legend:
            legend_keys = set()
        for plot_data in self.data:
            plot_data.legendgroup = plot_data.name
            if aggregate_legend:
                plot_data.showlegend = plot_data.name not in legend_keys
                legend_keys.add(plot_data.name)
            self._add_subplot(fig, plot_data)
        for plot_format in self.format:
            self._format_subplot(fig, plot_format)
        if headnote is not None:
            fig.add_annotation(text=headnote, showarrow=False, xref='paper',
                               x=0.5, yref='paper', y=1.075, font=dict(size=14))
        if show:
            fig.show()
        if save:
            if file_name is None:
                file_name = title
            self.save(fig, file_name)

    @cached_property
    def _subp_titles_placeholder(self):
        return tuple([" "] * (self.subplots[0] * self.subplots[1]))

    def _add_subplot(self, fig: go.Figure, plot_data: PlotData):
        y = plot_data.y
        name = plot_data.name
        row, col = plot_data.subplot
        if plot_data.mode == PlotMode.HLINE:
            fig.add_hline(
                y=y,
                annotation_text=name,
                line_dash="dash",
                row=row,
                col=col,
                line_color=plot_data.color,
            )
        elif plot_data.mode == PlotMode.VLINE:
            fig.add_vline(
                x=plot_data.x,
                annotation_text=name,
                line_dash="dash",
                row=row,
                col=col,
                line_color=plot_data.color,
            )
        elif plot_data.mode == PlotMode.VRECT:
            fig.add_vrect(
                x0=plot_data.x,
                x1=plot_data.y,
                fillcolor=plot_data.color,
                opacity=plot_data.opacity,
                layer="below",
                line_width=plot_data.line_width,
                row=row,
                col=col
            )
        elif plot_data.mode == PlotMode.HIST:

            params = {"x": plot_data.x,
                      "name": name,
                      "showlegend": plot_data.showlegend,
                      "legendgroup": plot_data.legendgroup,
                      "nbinsx": plot_data.nbins,
                      "histnorm": plot_data.hist_norm.value,
                      "marker_color": plot_data.color,
                      "opacity": plot_data.opacity,
                      "bingroup": 1}

            fig.add_trace(go.Histogram(**params), row=row, col=col)

            if any(x != plot_data.x[0] for x in plot_data.x):  # only if all values are not the same
                if plot_data.distribution.value == HistDistribution.AUTO.value:
                    dist, pars = self.dist_fit(plot_data.x)
                    print(dist.name)
                    try:
                        self.add_dist_curve(dist, pars, fig, plot_data)
                    except:
                        pass
                elif plot_data.distribution.value == HistDistribution.NORMAL.value:
                    self.add_normal_curve(fig, plot_data)

        else:
            marker_size = plot_data.marker_size
            if marker_size is None:
                marker_size = 10 if plot_data.mode == PlotMode.SCATTER else 4
            x = plot_data.x
            y_err_minus = plot_data.y_err_minus
            y_err = plot_data.y_err
            y_err_vis = (y_err is not None) and (not plot_data.continuous_err)
            color = plot_data.color
            line_color = plot_data.line_color
            cmin, cmax = plot_data.color_range
            color_scale = plot_data.color_scale
            opacity = plot_data.opacity
            marker_opacity = plot_data.marker_opacity
            fill = plot_data.fill
            # f"{self._get_subplot_idx(subp)}" ; legendgrouptitle=dict(text=legendgroup)
            legendgroup = plot_data.legendgroup if self.legendgroups else None
            showlegend = plot_data.showlegend
            if plot_data.text is None:
                text = [
                    f"x: {'{0:.3g}'.format(x_val)}<br>y: {'{0:.3g}'.format(y_val)}"
                    for x_val, y_val in zip(x, y)
                ]
            else:
                text = [
                    f"x: {'{0:.3g}'.format(x_val)}<br>y: {'{0:.3g}'.format(y_val)}<br>{text_val}"
                    for x_val, y_val, text_val in zip(x, y, plot_data.text)
                ]
            fig.add_trace(
                go.Scattergl(
                    x=x,
                    y=y,
                    name=name,
                    mode=plot_data.mode,
                    opacity=opacity,
                    marker=dict(
                        size=marker_size,
                        color=color,
                        colorscale=color_scale,
                        cmin=cmin,
                        cmax=cmax,
                        opacity=marker_opacity,
                    ),
                    showlegend=showlegend,
                    legendgroup=legendgroup,
                    line=dict(color=line_color),
                    text=text,
                    hoverinfo="text",
                    fill=fill,
                    error_y=dict(
                        symmetric=False, array=y_err, arrayminus=y_err_minus, visible=y_err_vis
                    ),
                ),
                row=row,
                col=col,
            )
            if plot_data.continuous_err:

                fig.add_trace(
                    go.Scatter(
                        x=np.append(x, x[::-1]),
                        y=np.append(
                            (y + y_err), np.array((y - y_err_minus))[::-1]
                        ),
                        showlegend=showlegend,
                        legendgroup=f"{legendgroup}_err",
                        text=text,
                        hoverinfo="text",
                        fill='toself',
                        fillcolor=plot_data.err_color,
                        line_color=plot_data.err_color,
                        name=plot_data.err_name
                    ),
                    row=row,
                    col=col,
                )



    @staticmethod
    def dist_fit(data):
        dists = (stats.laplace, stats.norm, stats.uniform, stats.laplace_asymmetric, stats.gamma)
        pars = [dist.fit(data) for dist in dists]
        return min(zip(dists, pars), key=lambda d: d[0].nnlf(d[1], data))

    def add_normal_curve(self, fig, plot_data):
        counts, area = self.get_area(fig)
        histnorm_factor = self.get_histnorm_factor(plot_data, area)
        dist = statistics.NormalDist.from_samples(plot_data.x)
        if dist.stdev != 0:
            x = np.linspace(dist.mean - 3*dist.stdev, dist.mean + 3*dist.stdev, 100)
            y = [dist.pdf(val) for val in x]
            factor = histnorm_factor * max(counts) / max(y)
            y = [val * factor for val in y]
            self.add_data(PlotData("Normal", x=x, y=y, mode=PlotMode.LINES, subplot=plot_data.subplot, color="black", showlegend=False))
        self.add_data(PlotData(f"Mean {round(dist.mean, 3)} Stdev {round(dist.stdev, 3)}", x=dist.mean, mode=PlotMode.VLINE, subplot=plot_data.subplot, color="black"))

    def add_dist_curve(self, dist, pars, fig, plot_data):
        counts, area = self.get_area(fig)
        histnorm_factor = self.get_histnorm_factor(plot_data, area)
        names = ["loc", "scale"]
        vlines = []
        if dist.name == "uniform":
            vlines = [pars[0], pars[0] + pars[1]]
        elif dist.name == "norm":
            vlines = [pars[0], pars[0] + pars[1], pars[0] + 2*pars[1], pars[0] - pars[1], pars[0] - 2*pars[1]]
        elif dist.name == "laplace":
            vlines = [pars[0]]
        elif dist.name == "laplace_asymmetric":
            names.insert(0, "kappa")
            vlines.append(pars[1])
        elif dist.name == "gamma":
            names.insert(0, "a")
        else:
            raise ValueError(f"Unsupported distribution: {dist.name}")
        x = np.linspace(dist.ppf(0.01, *pars), dist.ppf(0.99, *pars), 100)
        y = [dist.pdf(val, *pars) for val in x]
        factor = histnorm_factor * max(counts) / max(y)
        y = [val * factor for val in y]
        self.add_data(PlotData(dist.name, x=x, y=y, mode=PlotMode.LINES, subplot=plot_data.subplot, color="black"))
        for val in vlines:
            self.add_data(PlotData(f"{round(val, 3)}", x=val, mode=PlotMode.VLINE, subplot=plot_data.subplot, color="black"))

    def get_area(self, fig):
        f = fig.full_figure_for_development(warn=False)
        xbins = f.data[-1].xbins
        plotbins = list(np.arange(start=xbins['start'], stop=xbins['end']+xbins['size'], step=xbins['size']))
        counts, bins = np.histogram(list(f.data[-1].x), bins=plotbins)
        area = sum(
                    count * bin
                    for count, bin in zip(
                        counts, [bins[i] - bins[i - 1] for i in range(1, len(bins))]
                    )
                )

        return counts, area

    def get_histnorm_factor(self, plot_data, area):
        histnorm_factor = 1
        if plot_data.hist_norm is not None:
            if plot_data.hist_norm.value == HistNorm.PERCENT.value:
                histnorm_factor = 100 / len(plot_data.x)
            elif plot_data.hist_norm.value == HistNorm.PROBABILITY.value:
                histnorm_factor = 1 / len(plot_data.x)
            elif plot_data.hist_norm.value == HistNorm.DENSITY.value:
                histnorm_factor = len(plot_data.x) / area
            elif plot_data.hist_norm.value == HistNorm.PROBABILITY_DENSITY.value:
                histnorm_factor = 1 / area
        return histnorm_factor

    def _format_subplot(self, fig: go.Figure, plot_format):
        subp = plot_format.subplot
        x_label = plot_format.x_label
        y_label = plot_format.y_label
        title = plot_format.title
        if x_label is not None:
            fig.update_xaxes(title_text=x_label, row=subp[0], col=subp[1])
        if y_label is not None:
            fig.update_yaxes(title_text=y_label, row=subp[0], col=subp[1])
        if title is not None:
            idx = self._get_subplot_idx(subp)
            fig.layout.annotations[idx].update(text=title)
        # fig.update_traces(marker=dict(size=20), row=subp[0], col=subp[1])

    def _get_subplot_idx(self, subp):
        return (subp[0] - 1) + (subp[1] - 1) + ((self.subplots[1] - 1) * (subp[0] - 1))

    def plot_heat_map(self,df,column_names,title,threshold=0):
        fig = px.imshow(df,
            x=column_names,
            y=column_names,
            color_continuous_scale='RdBu_r',
            zmin=-1,
            zmax=1,
            color_continuous_midpoint=0,
            title=title)
        
        fig.update_traces(hovertemplate="Value: %{z:.2f}<extra></extra>")
        fig.update_xaxes(side='top')
        fig.update_layout(xaxis={'type': 'category'}, yaxis={'type': 'category'})
        

        fig.show()
  




    def plot_table(self,df):
        fig = go.Figure(data=[go.Table(
            header=dict(values=list(df.columns),
                        fill_color='paleturquoise',
                        align='center'),
            cells=dict(values=[df[column] for column in df.columns],
                    fill_color='lavender',
                    align='center'))
        ])

        fig.show()


    @staticmethod
    def get_square_layout_subplot_idx(n: int):
        n_long, n_short = Plotter._get_square_layout_dimensions(n)
        return [(row + 1, col + 1) for col in range(n_long) for row in range(n_short)]

    @staticmethod
    def get_grouped_square_layout_subplot_idx(n: int, group_size: int):
        n_long, n_short = Plotter._get_square_layout_dimensions(n)
        subplots = []
        for row in range(n_short):
            for col in range(n_long):
                subplots.append([(row * group_size + 1 + idx, col + 1) for idx in range(group_size)])
        return subplots, n_short, n_long

    @staticmethod
    def _get_square_layout_dimensions(n: int):
        sqrt = math.sqrt(n)
        sqrt_ceil = math.ceil(sqrt)
        sqrt_floor = math.floor(sqrt)
        if n % sqrt_ceil == 0:
            n_long = sqrt_ceil
            n_short = n // sqrt_ceil
        elif n % sqrt_floor == 0:
            n_long = n // sqrt_floor
            n_short = sqrt_floor
        else:  # probably some redundant space
            n_long = sqrt_ceil
            n_short = sqrt_ceil
        return n_long, n_short

    @staticmethod
    def get_vertical_layout_subplot_idx(n: int):
        return [(row + 1, 1) for row in range(n)]

    @staticmethod
    def title(name):
        return f"{name.replace('_', ' ')}".title()
