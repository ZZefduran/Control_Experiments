import numpy as np
import matplotlib.pyplot as plt
from projects.utils.plotter.plotter import Plotter
from projects.utils.plotter.plot_format import PlotFormat
from projects.utils.plotter.plot_data import PlotData
from projects.utils.plotter.plot_enums import PlotMode


class DofPropsVsLossPlotter:
    def __init__(self, plot_dir, vis_dof, loss, default_params) -> None:
        self.plot_dir = plot_dir
        self.vis_dof = vis_dof
        self.loss = loss
        self.default_params = default_params

    def plot(self, master_log, stiction_enabled, frictionloss_enabled, should_save, should_show):
        num_batches = master_log["batch_idx"].max() + 1
        colors = self._get_batch_colors("viridis", num_batches)
        restitution_enabled = "restitution" in master_log
        keys, units = self.get_plot_labels(stiction_enabled, frictionloss_enabled, restitution_enabled)
        subplot_idx = Plotter.get_vertical_layout_subplot_idx(len(keys))
        if self.loss == "pos":
            label = "Position L2"
        elif self.loss == "vel":
            label = "Velocity L2"
        else:
            raise ValueError(f"Loss should be pos or vel, not {self.loss}")
        for dof in self.vis_dof:
            plotter = Plotter(self.plot_dir, legendgroups=True)
            x = np.array(master_log[f"{dof}_l2"])
            x_per_batch = np.array_split(x, num_batches)
            for subplot, key, unit in zip(subplot_idx, keys, units):
                x_label = label if subplot == subplot_idx[-1] else None
                self.add_to_plot(master_log, num_batches, colors, x_label, dof, plotter, x_per_batch, subplot, key, unit)

        plotter.plot(
            Plotter.title(f"dof_props_optimization_{dof}"),
            shared_x=True,
            save=should_save,
            show=should_show,
            aggregate_legend=True
        )

    def get_plot_labels(self, stiction_enabled, frictionloss_enabled, restitution_enabled):
        keys = ["armature", "friction", "damping"]
        units = ["kgm^2", "-", "Nms/rad"]
        if stiction_enabled:
            keys.append("stiction")
            units.append("Nm")
        if frictionloss_enabled:
            keys.append("frictionloss")
            units.append("Nm")
        if restitution_enabled:
            keys.append("restitution")
            units.append("-")
        return keys,units

    def add_to_plot(self, master_log, num_batches, colors, x_label, dof, plotter: Plotter, x_per_batch, subplot, key, unit):
        title = Plotter.title(key)
        log_key = key if key == "restitution" else f"{dof}_{key}"
        y = np.array(master_log[log_key])
        y_per_batch = np.array_split(y, num_batches)
        for i in range(num_batches):
            plotter.add_data(
                        PlotData(
                            f"Batch {i}",
                            x=x_per_batch[i],
                            y=y_per_batch[i],
                            subplot=subplot,
                            color=colors[i],
                            mode=PlotMode.SCATTER,
                        )
                    )
        best_loss, best_val = min(zip([val for val_per_batch in x_per_batch for val in val_per_batch], y))
        plotter.add_data(PlotData('{0:.3g}'.format(best_loss), x=best_loss, subplot=subplot, showlegend=False, mode=PlotMode.VLINE))
        plotter.add_data(PlotData(f"Best: {'{0:.3g}'.format(best_val)}", y=best_val, subplot=subplot, showlegend=False, mode=PlotMode.HLINE))
        if (prop_vals:=self.default_params.get(key)) is not None:
            if (default_val:=prop_vals.get(dof))  is not None:
                plotter.add_data(PlotData(f"Default: {'{0:.3g}'.format(default_val)}", y=default_val, subplot=subplot, showlegend=False, mode=PlotMode.HLINE))
        plotter.add_format(
                    PlotFormat(x_label, f"{title} [{unit}]", title=title, subplot=subplot)
                )

    def _get_batch_colors(self, cmap_name, n):
        cmap = plt.get_cmap(cmap_name)
        colors = [cmap(1.0)] if n == 1 else cmap((range(n) / (n - 1)).tolist())
        colors = [f"rgba({','.join([str(c) for c in color])})" for color in colors]
        return colors
