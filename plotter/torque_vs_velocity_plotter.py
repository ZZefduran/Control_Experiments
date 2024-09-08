import numpy as np
from projects.utils.plotter.plotter import Plotter
from projects.utils.plotter.plot_format import PlotFormat
from projects.utils.plotter.plot_data import PlotData
from projects.utils.plotter.plot_enums import PlotColors, PlotMode


class TorqueVsVelocityPlotter:
    def __init__(self, plot_dir, vis_dof, joints_params) -> None:
        self.plot_dir = plot_dir
        self.vis_dof = vis_dof
        self.joints_params = joints_params

    def plot(self, real_data, sim_data, should_save, should_show):
        for dof in self.vis_dof:
            plotter = Plotter(self.plot_dir, legendgroups=True)
            self.plot_effort_envelop(plotter)
            self.add_data(plotter, real_data[dof], "Real", PlotColors.REAL)
            self.add_data(plotter, sim_data[dof], "Sim", PlotColors.SIM)
            plotter.add_format(
                PlotFormat(
                    "Velocity [rad/s]",
                    "Torque [Nm]",
                    title="Velocity Torque Scatter",
                    subplot=(1, 1),
                )
            )
            plotter.plot(
                Plotter.title(f"torque_vs_velocity_{dof}"),
                shared_x=True,
                save=should_save,
                show=should_show,
            )

    def plot_effort_envelop(self, plotter: Plotter):
        for dof in self.vis_dof:
            v_peak, v_break, _ = self.joints_params.vel[dof]
            max_effort = self.joints_params.max_effort[dof]
            ceil_vel = 2 * v_break - v_peak
            x = [v_peak, ceil_vel, -v_peak, -ceil_vel, v_peak]
            y = [max_effort, -max_effort, -max_effort, max_effort, max_effort]
            plotter.add_data(
                PlotData(
                    "Motor Characterization",
                    x=x,
                    y=y,
                    subplot=(1, 1),
                    color="teal",
                    opacity=0.5,
                    mode=PlotMode.DEFAULT,
                    fill="toself",
                )
            )

    def add_data(self, plotter: Plotter, data, name, color):
        plotter.add_data(
            PlotData(
                "",
                x=data["vel"],
                y=data["effort"],
                subplot=(1, 1),
                color=color,
                mode=PlotMode.SCATTER,
                legendgroup=name,
                showlegend=False,
            )
        )
        max_effort = np.array(data["effort"]).max()
        min_effort = np.array(data["effort"]).min()
        max_vel = np.array(data["vel"]).max()
        min_vel = np.array(data["vel"]).min()
        plotter.add_data(
            PlotData(
                f"{name}: [{min_effort:.2f},{max_effort:.2f}] Nm X [{min_vel:.2f},{max_vel:.2f}] rad/s",
                x=[min_vel, max_vel],
                y=[max_effort, max_effort],
                subplot=(1, 1),
                color=color,
                mode=PlotMode.DEFAULT,
                legendgroup=name,
            )
        )
        plotter.add_data(
            PlotData(
                "",
                x=[min_vel, max_vel],
                y=[min_effort, min_effort],
                subplot=(1, 1),
                color=color,
                mode=PlotMode.DEFAULT,
                legendgroup=name,
                showlegend=False,
            )
        )
        plotter.add_data(
            PlotData(
                "",
                x=[max_vel, max_vel],
                y=[min_effort, max_effort],
                subplot=(1, 1),
                color=color,
                mode=PlotMode.DEFAULT,
                legendgroup=name,
                showlegend=False,
            )
        )
        plotter.add_data(
            PlotData(
                "",
                x=[min_vel, min_vel],
                y=[min_effort, max_effort],
                subplot=(1, 1),
                color=color,
                mode=PlotMode.DEFAULT,
                legendgroup=name,
                showlegend=False,
            )
        )
