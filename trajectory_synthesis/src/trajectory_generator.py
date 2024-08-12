from argparse import ArgumentParser
import os
import sys
import numpy as np
import scipy as sp
from csv_writer import CsvWriter
from single_waves_generators.single_wave_generator_factory import SingleWaveGeneratorFactory
from single_waves_generators.single_wave_generator_base import SingleWaveGeneratorBase
from single_waves_generators.trapezoid_wave_generator import TrapezoidWaveGenerator
from cfg_data import CfgData
from single_waves_generators.spline_wave_generator import generate_spline
from omegaconf import OmegaConf
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3]))
import ast
import operator as op
import numbers


class TrajectoryGeneratorArgsParser(ArgumentParser):
    def __init__(self, default_yaml) -> None:
        super().__init__()
        self.add_argument("--cfg", type=str, default=default_yaml, help="YAML configuration file")
        self.args, _ = self.parse_known_args()


class TrajectoryGenerator:
    def __init__(self, cfg, default_cfg) -> None:
        if cfg is None:
            cfg = TrajectoryGeneratorArgsParser(default_cfg).args.cfg
        self.cfg_data = CfgData(cfg)

    def get_freq_trajectory(self, freq, swg: SingleWaveGeneratorBase, amplitude, mask, offsets):
        if isinstance(swg, TrapezoidWaveGenerator):
            swg.set_bounds(self.cfg_data.cfg.wave.trapezoid.bounds_ratio)
        single_wave = swg.generate(freq)
        return np.transpose(
            [single_wave * factor * amplitude + offset for factor, offset in zip(mask, offsets)]
        )

    def get_dof_limit(self, dof, limits):
        if self.cfg_data.cfg.asset_yaml.get("use_asset_yaml_default_kinematics_limits", True):
            path = self.cfg_data.cfg.asset_yaml.path
            if not os.path.isfile(path):
                raise ValueError(f"Cannot load asset yaml file: {path}")
            asset_cfg = OmegaConf.load(path)
            limits_cfg = asset_cfg.robot
        else:
            limits_cfg = self.cfg_data.cfg.asset_yaml
        peak_vel, max_vel, _ = limits_cfg.velocity[dof]
        max_acc = limits_cfg.acceleration[dof]
        max_jerk = limits_cfg.jerk[dof]
        min_pos, max_pos = limits[dof]
        return DofLimit(max_pos, min_pos, peak_vel, max_vel, max_acc, max_jerk)

    def generate_trajectory(
        self,
        offsets,
        custom_init,
        limits,
        amplitude,
        freq_range,
        amp_factor_range,
        side=None,
        dof_idx=-1,
        dof=None,
    ):
        shape = self.cfg_data.cfg.trajectory.shape
        sample_freq = self.cfg_data.cfg.trajectory.frequency
        repeat = self.cfg_data.cfg.wave.repeat
        mask = get_mask(shape, side, dof_idx, len(self.cfg_data.cfg.all_dof))
        if shape == "spline":
            self.generate_spline(
                offsets, limits, amplitude, amp_factor_range, dof, sample_freq, repeat, mask
            )
        else:
            traj = self.generate_waves_trajectory(
                offsets,
                custom_init,
                amplitude,
                freq_range,
                amp_factor_range,
                shape,
                sample_freq,
                repeat,
                mask,
            )
        if self.cfg_data.cfg.asset_yaml.check_limits:
            self.validate_trajectory_in_limits(traj, limits)
        return traj

    def generate_waves_trajectory(
        self,
        offsets,
        custom_init,
        amplitude,
        freq_range,
        amp_factor_range,
        shape,
        sample_freq,
        repeat,
        mask,
    ):
        all_waves_traj = []
        for _shape in shape.split("+"):
            swg = SingleWaveGeneratorFactory.get(_shape)(sample_freq)
            waves_all_freqs = []
            for freq in freq_range:
                for amplitude_factor in amp_factor_range:
                    single_wave = self.get_freq_trajectory(
                        freq, swg, amplitude * amplitude_factor, mask, offsets
                    )
                    if (
                        self.cfg_data.cfg.wave.get("partial", False)
                        and self.cfg_data.cfg.wave.partial.enable
                    ):
                        single_wave = self.get_partial_wave(single_wave)
                    repeated_wave = np.tile(single_wave, (repeat, 1))
                    freq_separator_sec = self.cfg_data.cfg.wave.get("separator")
                    if freq_separator_sec is not None:
                        repeated_wave = self.add_separator(
                            offsets, sample_freq, repeated_wave, freq_separator_sec
                        )
                    waves_all_freqs.append(repeated_wave)
            all_waves_traj.append(np.concatenate(waves_all_freqs))
        # traj = all_waves_traj[0]
        # for waves_traj in all_waves_traj[1:]:
        #     traj = np.concatenate((traj, waves_traj))
        traj = np.concatenate(all_waves_traj)
        traj = self.add_perimeters(traj, offsets, custom_init)
        return traj

    def add_perimeters(self, traj, offsets, custom_init):
        f = self.cfg_data.cfg.trajectory.frequency
        traj = np.concatenate(
            [np.tile(offsets, (int(0.5 * f), 1)), traj, np.tile(offsets, (int(0.5 * f), 1))]
        )
        if not np.array_equal(offsets, custom_init):
            max_pos_diff = max(abs(offsets - custom_init))
            max_vel = 0.5
            delta_t = (np.pi / 2) * max_pos_diff / max_vel
            n = int(delta_t * f)
            t = np.linspace(0, np.pi, n)
            factors = (1 - np.cos(t)) / 2
            ramp = np.empty((n, offsets.shape[0]))
            for i, factor in enumerate(factors):
                ramp[i] = (1 - factor) * custom_init + factor * offsets

            traj = np.vstack((ramp, traj, ramp[::-1]))
            traj = np.concatenate(
                [
                    np.tile(custom_init, (int(0.5 * f), 1)),
                    traj,
                    np.tile(custom_init, (int(0.5 * f), 1)),
                ]
            )
        return traj

    def add_separator(self, offsets, sample_freq, repeated_wave, freq_separator_sec):
        repetition = freq_separator_sec * sample_freq
        freq_separator = np.tile(offsets, (repetition, 1))
        repeated_wave = np.concatenate([repeated_wave, freq_separator])
        return repeated_wave

    def get_partial_wave(self, single_wave):
        total = self.cfg_data.cfg.wave.partial.total_parts
        idx = self.cfg_data.cfg.wave.partial.part - 1
        single_wave = np.array_split(single_wave, total)[idx]
        return single_wave

    def generate_spline(
        self, offsets, limits, amplitude, amp_factor_range, dof, sample_freq, repeat, mask
    ):
        dof_limit = self.get_dof_limit(dof, limits)
        show = self.cfg_data.cfg.trajectory.plot.show
        plateau_num = self.cfg_data.cfg.wave.get("plateau_num", 0)
        for amplitude_factor in amp_factor_range:
            single_spline = generate_spline(
                sample_freq, amplitude * amplitude_factor, repeat, dof_limit, show, plateau_num
            )
            traj = np.transpose(
                [single_spline * factor + offset for factor, offset in zip(mask, offsets)]
            )
        return traj

    def save_and_plot(self, data, csv_writer: CsvWriter, name, offsets=None, limits=None):
        csv_data = data
        csv_writer.update_data(csv_data)
        csv_writer.write_data(name)
        f=0
        # self.plot_entire_data(data, name)

    @staticmethod
    def to_title(name):
        return f"{name.replace('_', ' ')}".title()

    def plot(self, data, name, dofs, offsets, limits):
        subplot_num = len(dofs)
        plotter = Plotter(self.plots_dir)
        x = np.arange(len(data)) / self.cfg_data.cfg.trajectory.frequency
        subplot_idx, _ = plotter.get_square_layout_subplot_idx(subplot_num)
        for i, dof in enumerate(dofs):
            plotter.add_data(
                PlotData(
                    self.to_title(dof), x=x, y=data[:, i], subplot=subplot_idx[i], color="royalblue"
                )
            )
            if offsets is not None:
                plotter.add_data(
                    PlotData(
                        "{:.2f}".format(offsets[i]),
                        y=offsets[i],
                        mode=PlotMode.HLINE,
                        subplot=subplot_idx[i],
                    )
                )
            if limits is not None:
                plotter.add_data(
                    PlotData(
                        "{:.2f}".format(limits[dof][1]),
                        y=limits[dof][1],
                        mode=PlotMode.HLINE,
                        subplot=subplot_idx[i],
                        color="red",
                    )
                )
                plotter.add_data(
                    PlotData(
                        "{:.2f}".format(limits[dof][0]),
                        y=limits[dof][0],
                        mode=PlotMode.HLINE,
                        subplot=subplot_idx[i],
                        color="red",
                    )
                )
        for i, dof in enumerate(dofs):
            x_label = "Time [sec]" if subplot_idx[i][0] == plotter.subplots[0] else None
            y_label = "Pos [rad]"
            plotter.add_format(
                PlotFormat(
                    x_label, y_label, title=f"<b>{self.to_title(dof)}</b>", subplot=subplot_idx[i]
                )
            )
        should_save = self.cfg_data.cfg.trajectory.plot.save
        should_show = self.cfg_data.cfg.trajectory.plot.show
        plotter.plot(self.to_title(name), save=should_save, shared_x=True, show=should_show)

    def plot_entire_data(self, data: dict, name: str):
        subplot_num = len(data)
        plotter = Plotter(self.plots_dir)
        x = np.arange(len(next(iter(data.values())))) / self.cfg_data.cfg.trajectory.frequency
        subplot_idx, _ = plotter.get_square_layout_subplot_idx(subplot_num)
        for i, (key, val) in enumerate(data.items()):
            dof = "_".join(key.split("_")[:-1])
            dof_val = key.split("_")[-1]
            plotter.add_data(
                PlotData(self.to_title(dof), x=x, y=val, subplot=subplot_idx[i], color="royalblue")
            )
        for i, dof in enumerate(data.keys()):
            x_label = "Time [sec]" if subplot_idx[i][0] == plotter.subplots[0] else None
            y_label = dof_val
            plotter.add_format(
                PlotFormat(
                    x_label, y_label, title=f"<b>{self.to_title(dof)}</b>", subplot=subplot_idx[i]
                )
            )
        should_save = self.cfg_data.cfg.trajectory.plot.save
        should_show = self.cfg_data.cfg.trajectory.plot.show
        plotter.plot(self.to_title(name), save=should_save, shared_x=True, show=should_show)

    def set_control_gains(self, csv_writer: CsvWriter):
        pod_yaml_cfg = self.cfg_data.cfg.get("pod_yaml")
        if pod_yaml_cfg is None:
            print(
                "warning: control gains will not be written into the csv. reason: there is no pod_yaml section in the input yaml"
            )
        else:
            if pod_yaml_cfg.use_pod_yaml_control_gains:
                path = pod_yaml_cfg.path
                if os.path.isfile(path):
                    pod_cfg = OmegaConf.load(path)
                    data = {}
                    for control_gain in ["kp", "kd"]:
                        for dof in pod_cfg.standing.control[control_gain]:
                            dofs = [f"right_{dof}", f"left_{dof}"]
                            for dof_name in dofs:
                                if dof_name in self.cfg_data.cfg.all_dof:
                                    data[f"{dof_name}_{control_gain}"] = pod_cfg.standing.control[
                                        control_gain
                                    ][dof]
                    csv_writer.update_data(data)

                else:
                    raise ValueError(f"Cannot load pos yaml file: {path}")

                # TODO
                pass
            else:
                data = {}
                for dof in pod_yaml_cfg.kp:
                    data[f"{dof}_kp"] = pod_yaml_cfg.kp[dof]
                for dof in pod_yaml_cfg.kd:
                    data[f"{dof}_kd"] = pod_yaml_cfg.kd[dof]
                csv_writer.update_data(data)

    @staticmethod
    def makedir(path):
        if not os.path.exists(path):
            os.makedirs(path)

    def init_dirs(self):
        out_dir = self.cfg_data.cfg.log_dir
        self.csv_dir = os.path.join(out_dir, "csvs")
        self.plots_dir = os.path.join(out_dir, "plots")
        self.makedir(self.csv_dir)
        self.makedir(self.plots_dir)

    def generate(self):
        if self.cfg_data.cfg.trajectory.shape == "piecewise":
            self.generate_piecewise()
        else:
            self.generate_wave()

    def generate_piecewise(self):
        self.init_dirs()
        csv_writer = CsvWriter(self.csv_dir)
        if self.cfg_data.cfg.get("export_control_gains", False):
            self.set_control_gains(csv_writer)
        data = self.generate_piecewise_trajectory('motor')
        self.save_and_plot(data, csv_writer, self.cfg_data.cfg.log_name)

    def get_piecewise_vectors(self):
        x, y = zip(*self.cfg_data.cfg.piecewise.get("values"))
        x = self.parse_vector(x)
        y = self.parse_vector(y)
        return x, y

    def parse_vector(self, vec):
        return [self.eval_expr(v) for v in vec]

    def eval_expr(self, expr):
        if isinstance(expr, numbers.Number):
            return expr
        return self.eval_(ast.parse(expr, mode="eval").body)

    def eval_(self, node):
        operators = {
            ast.Add: op.add,
            ast.Sub: op.sub,
            ast.Mult: op.mul,
            ast.Div: op.truediv,
            ast.Pow: op.pow,
            ast.BitXor: op.xor,
            ast.USub: op.neg,
        }
        if isinstance(node, ast.Num):  # <number>
            return node.n
        elif isinstance(node, ast.BinOp):  # <left> <operator> <right>
            return operators[type(node.op)](self.eval_(node.left), self.eval_(node.right))
        elif isinstance(node, ast.UnaryOp):  # <operator> <operand> e.g., -1
            return operators[type(node.op)](self.eval_(node.operand))
        else:
            raise TypeError(node)

    def generate_piecewise_trajectory(self, dof):
        x, y = self.get_piecewise_vectors()
        t_max = x[-1]
        f = self.cfg_data.cfg.trajectory.frequency
        t = np.linspace(0, t_max, num=int(t_max * f) + 1)
        dof_data = sp.interpolate.interp1d(x, y)(t)
        return {f"{dof}_{self.cfg_data.cfg.piecewise.objective}": dof_data}

    def generate_wave(self):
        amplitudes = self.cfg_data.get_amplitudes(offsets)
        freq_range = self.cfg_data.get_freq_range()
        amp_factor_range = self.cfg_data.get_amplitude_range()

        self.init_dirs()

        csv_writer = CsvWriter(self.csv_dir)
        if self.cfg_data.cfg.get("export_control_gains", False):
            self.set_control_gains(csv_writer)
        if self.cfg_data.cfg.trajectory.shape in ["sine", "spline", "trapezoid", "sine+trapezoid"]:
            for dof_idx, dof in enumerate(self.cfg_data.cfg.all_dof):
                if dof in self.cfg_data.cfg.vis_dof:
                    data = self.generate_trajectory(
                        offsets,
                        custom_init,
                        limits,
                        amplitudes[dof],
                        freq_range,
                        amp_factor_range,
                        dof_idx=dof_idx,
                        dof=dof,
                    )
                    self.save_and_plot(
                        data, csv_writer, self.cfg_data.cfg.log_name, offsets, limits
                    )
        else:
            for side in self.cfg_data.cfg.vis_side:
                data = self.generate_trajectory(
                    offsets,
                    custom_init,
                    limits,
                    amplitudes[side],
                    freq_range,
                    amp_factor_range,
                    side=side,
                )
                self.save_and_plot(data, csv_writer, self.cfg_data.cfg.log_name, offsets, limits)

    def validate_trajectory_in_limits(self, traj, limits):
        out_of_range_dofs = []

        if limits is not None:
            min_vals = traj.min(axis=0)
            max_vals = traj.max(axis=0)
            out_of_range_dofs.extend(
                dof
                for i, dof in enumerate(self.cfg_data.cfg.all_dof)
                if min_vals[i] < limits[dof][0] or max_vals[i] > limits[dof][1]
            )
        else:
            print("Warning: Cannot check if trajectories are within limits")
        if out_of_range_dofs:
            raise TrajectoryOutOfBounds(
                f"These DOFs trajectories are out of limits: {out_of_range_dofs}"
            )
