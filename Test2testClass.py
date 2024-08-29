import time
import pyCandle
from Futek import FutekClient
from ka3000_serial import ka3000
import yaml
from motor_power import tongui

from TestClass import MotorController, KtauExperiment

# set up motor supplier
supplier = tongui()
supplier.setOutputOn
time.sleep(2)


def load_params(motor_type):
    # Load the YAML file
    with open('Ktau_experiment_params.yaml', 'r') as file:
        all_params = yaml.safe_load(file)
    
    # Extract parameters for the specified motor type
    if motor_type in all_params:
        params = all_params[motor_type]
        return params
    else:
        raise ValueError(f"Motor type {motor_type} not found in YAML file")

# Example motor type
motor_type = 'E85x13'

# Load parameters for the specified motor type
params = load_params(motor_type)

# Extract parameters
voltage = params['voltage']
control_mode = getattr(pyCandle, params['control_mode'])
kp = params['kp']
kd = params['kd']
ki = params['ki']
ff = params['ff']
motor_name = params['motor_name']
torque_list = params['torque_list']

# Initialize the motor controller with the loaded parameters
motor_controller = MotorController(voltage, control_mode, kp, kd, ki, ff, motor_name)

# Initialize the Futek sensor
futek_client = FutekClient()

# Initializing korad & tonghui
korad = ka3000()
korad.setOutput(1)

# Setup power supplies
motor_controller.setup_power_supply()
time.sleep(2)

# Run and plot the experiment
ktau_experiment = KtauExperiment(motor_controller)
ktau_experiment.run_and_plot_experiment(torque_list, futek_client)

# Shutdown the motor controller
motor_controller.shutdown()
korad.setOutput(0)
