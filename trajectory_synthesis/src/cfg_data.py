import os
from omegaconf import OmegaConf
import numpy as np

class CfgData:
    def __init__(self, cfg) -> None:
        self.cfg = OmegaConf.load(cfg)
        
    def get_amplitudes(self):
        amplitudes = {}
        if self.cfg.trajectory.shape in ["sine", "spline", "trapezoid", "sine+trapezoid"]: 
            amplitudes[dof] = abs(traj_limit - offsets[i])
        else:
            for side in self.cfg.vis_side:
                amplitudes[side] = self.cfg.wave.stomping.amplitude

        return amplitudes
    
    def get_dof_pos_data(self):
        asset_cfg = None
        soft_limits = None
        if asset_yaml_path is not None and os.path.isfile(asset_yaml_path):
            asset_cfg = OmegaConf.load(asset_yaml_path)
            soft_limits = asset_cfg.robot.dof_ext_props.soft_dof_pos_limits_real
        if asset_cfg is not None and self.cfg.asset_yaml.use_asset_yaml_default_joint_angles_init:
            offset_dic = asset_cfg.robot.init_state.default_joint_angles_init
        else:
            offset_dic = self.cfg.asset_yaml.default_joint_angles_init

        offsets = np.zeros(len(self.cfg.all_dof))
        for i, dof in enumerate(self.cfg.all_dof):
            offsets[i] = offset_dic[dof]

        custom_init = offsets.copy()
        custom_init_cfg = self.cfg.asset_yaml.get("custom_joint_angles_init", {})
        for i, dof in enumerate(self.cfg.all_dof):
            if dof in custom_init_cfg:
                custom_init[i] = custom_init_cfg[dof]

        return offsets, soft_limits, custom_init
    
    def get_freq_range(self):
        freq_range = self.get_range(self.cfg.wave.frequency)
        print(f"freq range: {freq_range}")
        return freq_range

    def get_amplitude_range(self):
        amp_range = self.get_range(self.cfg.wave.amplitude_factor)
        print(f"amplitude range: {amp_range}")
        return amp_range
    
    def get_range(self, range_cfg):
        start = range_cfg.start
        stop = range_cfg.stop
        step = range_cfg.step
        num = round((stop - start) / step) + 1
        freq_range = np.linspace(start, stop, num)
        return freq_range

