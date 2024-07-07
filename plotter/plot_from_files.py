import os
import pandas as pd
from plotter import Plotter
from plot_data import PlotData
from pathlib import Path
from plot_format import PlotFormat

DATA_DIR = "/home/ori/ori/calibration/SDOF/8064/2023-06-06-const_vel/post_processing/data/steady_state_torque_vs_velocity"
# DATA_DIR = "/home/ori/ori/calibration/SDOF/8064/2023-06-06-const_vel/post_processing/data/steady_state_different_kd"

def to_title(name):
    return f"{name.replace('_', ' ')}".title()

def get_all_files_in_dir(dir, extension):
    ext_files = []
    for root, _, files in os.walk(dir):
        ext_files.extend([os.path.join(root, file) for file in files if file.endswith(extension)])
    return sorted(ext_files)


def plot_dir(data_dir):
    plot_dir = os.path.join(data_dir, "plots")
    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)
    plotter = Plotter(plot_dir)
    csvs = get_all_files_in_dir(data_dir, ".csv")
    for csv_file in csvs:
        df = pd.read_csv(csv_file)
        name = to_title(Path(csv_file).stem)
        plotter.add_data(PlotData(name, df.iloc[:, 0], df.iloc[:, 1]))
    x_label, y_label = df.columns
    title = to_title(os.path.basename(data_dir))
    plotter.add_format(
                PlotFormat(x_label, y_label, title=title, subplot=(1,1)))
    plotter.plot(title)




if __name__ == "__main__":
    plot_dir(DATA_DIR)