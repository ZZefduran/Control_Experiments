import numpy as np
from projects.utils.plotter.plotter import Plotter
from projects.utils.plotter.plot_format import PlotFormat
from projects.utils.plotter.plot_data import PlotData
import plotly.express.colors as pcolor

class PowerPlotter:
    def __init__(self, plot_dir, name, vis_dof) -> None:
        self.plot_dir = plot_dir
        self.name = name
        self.vis_dof = vis_dof

    def plot(self, data, should_save, should_show, aggregate_legend=True):
        keys = ["power", "effort", "dof_velocity"]
        labels = ["Power [watt]", "Effort [Nm]", "Velocity [rad/s]"]
        # subplot_idx = Plotter.get_vertical_layout_subplot_idx(len(keys))
        subplots, _, _ = Plotter.get_grouped_square_layout_subplot_idx(len(self.vis_dof), len(keys))

        plotter = Plotter(self.plot_dir, legendgroups=True)
        for dof, subplot_idx in zip(self.vis_dof.items(), subplots):
            dof_name= dof[0]
            dof_idx =dof[1]
            for color, (subplot, key, label) in enumerate(zip(subplot_idx, keys, labels)):
                key_title = Plotter.title(key)
                self.add_data(
                    plotter, data, dof_idx, key, subplot, pcolor.qualitative.D3[color], f"{key_title}"
                )
                x_label = "Frame ID [u.l]" if subplot == subplot_idx[-1] else None
                title = Plotter.title(dof_name) if subplot == subplot_idx[0] else ""
                plotter.add_format(PlotFormat(x_label, label, title=title, subplot=subplot))

        plotter.plot(
            Plotter.title(self.name),
            shared_x=True,
            save=should_save,
            show=should_show,
            aggregate_legend=aggregate_legend
        )

    @staticmethod
    def add_data(plotter: Plotter, data, dof_idx, key, subplot, color, name):
        y = data[key][:, dof_idx]
        x = np.arange(len(y))
        if y is not None:
            plotter.add_data(PlotData(name, x=x, y=y, subplot=subplot, color=color))
