from traj_exp import CsvReaderExperiment
from omegaconf import OmegaConf
from trajectory_synthesis.src.trajectory_generator import TrajectoryGenerator

SIN_YAML = '/home/zzefduran/code/newBenchTest/Control_Experiments/trajectory_synthesis/config/sin_traj.yaml'
OUTPUT_DIR = '/home/zzefduran/Documents/exp_csvs/csvs'
traj_gen = TrajectoryGenerator(None, SIN_YAML)
traj_gen.generate()

csv_exp = CsvReaderExperiment(motor_name = 100,
                                csv_path = traj_gen.saved_path,
                                freq = 30,
                                output_dir=OUTPUT_DIR)
csv_exp.run_exp()

