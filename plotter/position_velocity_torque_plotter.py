import numpy as np
from projects.utils.plotter.plotter import Plotter
from projects.utils.plotter.plot_format import PlotFormat
from projects.utils.plotter.plot_data import PlotData
from projects.utils.plotter.plot_enums import PlotColors, PlotMode


class PositionVelocityTorquePlotter:
    def __init__(self, plot_dir, name, vis_dof, dt) -> None:
        self.plot_dir = plot_dir
        self.name = name
        self.vis_dof = vis_dof
        self.dt = dt

    def plot(self, real_data, sim_data, should_save, should_show, aggregate_legend=False):
        keys = ["pos", "vel", "effort"]
        labels = ["Position [rad]", "Velocity [rad/s]", "Effort [Nm]"]
        subplots, _, _ = Plotter.get_grouped_square_layout_subplot_idx(len(self.vis_dof), len(keys))
        target_data = self.get_target_data(real_data)
        x = self.dt * np.arange(len(next(iter(target_data.values()))["pos"]))

        plotter = Plotter(self.plot_dir, legendgroups=True)
        for dof, subplot_idx in zip(self.vis_dof, subplots):
            stiction_data = [int(val) for val in sim_data.get(dof, {}).get("stiction_mask")]
            self.add_data(
                plotter,
                x,
                sim_data,
                dof,
                "ext_forces",
                subplot_idx[-1],
                "black",
                "External Forces Sim",
            )
            for subplot, key, label in zip(subplot_idx, keys, labels):
                key_title = Plotter.title(key)
                self.add_data(
                    plotter, x, target_data, dof, key, subplot, PlotColors.TARGET, f"{key_title} Target"
                )
                self.add_data(
                    plotter, x, real_data, dof, key, subplot, PlotColors.REAL, f"{key_title} Real"
                )
                self.add_data(plotter, x, sim_data, dof, key, subplot,
                              PlotColors.SIM, f"{key_title} Sim", stiction_data)
                x_label = "Time [sec]" if subplot == subplot_idx[-1] else None
                title = Plotter.title(dof) if subplot == subplot_idx[0] else ""
                plotter.add_format(PlotFormat(x_label, label, title=title, subplot=subplot))

        plotter.plot(
            Plotter.title(self.name),
            shared_x=True,
            save=should_save,
            show=should_show,
            aggregate_legend=aggregate_legend
        )

    def add_mask(self, plotter: Plotter, x, data, subplot, color):
        if data is not None:
            for block in self.get_blocks(data):
                plotter.add_data(
                    PlotData(
                        x=x[block[0]],
                        y=x[block[1]],
                        subplot=subplot,
                        line_width=0,
                        color=color,
                        mode=PlotMode.VRECT,
                        opacity=0.4,
                    )
                )

    @staticmethod
    def get_target_data(real_data):
        return {key: {"pos": val["target_pos"]} for key, val in real_data.items()}

    @staticmethod
    def get_blocks(data):
        blocks = []
        if data is not None:
            block = [None, None]
            for i in range(len(data)):
                val = data[i]
                next_val = data[i + 1] if i < len(data) - 1 else False
                prev_val = data[i - 1] if i > 0 else False
                if val and not prev_val:
                    block[0] = i
                if val and not next_val:
                    block[1] = i
                    blocks.append(block)
                    block = [None, None]
        return blocks

    @staticmethod
    def add_data(plotter: Plotter, x, data, dof, key, subplot, color, name, err=None):
        y = data.get(dof, {}).get(key)
        if y is not None:
            if err is None:
                plotter.add_data(PlotData(name, x=x, y=y, subplot=subplot, color=color))
            else:
                y_max = max(y)
                y_min = min(y)
                y_err = np.array([(y_max - y_val)*err_val for y_val, err_val in zip(y, err)])
                y_err_minus = np.array([(y_val - y_min)*err_val for y_val, err_val in zip(y, err)])
                plotter.add_data(PlotData(name, x=x, y=y, subplot=subplot, color=color, y_err=y_err,
                                 y_err_minus=y_err_minus, continuous_err=True, err_color="lightpink", err_name="Stiction"))
