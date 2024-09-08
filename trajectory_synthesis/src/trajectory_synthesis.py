from trajectory_generator import TrajectoryGenerator

DEFAULT_YAML = "/home/zzefduran/code/newBenchTest/Control_Experiments/trajectory_synthesis/config/single_motor.yaml"

def generate(cfg = None):
    traj_gen = TrajectoryGenerator(cfg, DEFAULT_YAML)
    traj_gen.generate()

if __name__ == "__main__":
    generate()
    print("e")
