import time
import pyCandle
from Futek import FutekClient
from ka3000_serial import ka3000

# Import custom classes
from Experiments_Class import MotorController  # Ensure this file is named MotorController.py
from Experiments_Class import KtauExperiment

# Experiment parameters
voltage = 48
baud_rate = pyCandle.CAN_BAUD_2M
control_mode = pyCandle.IMPEDANCE  # Set to IMPEDANCE mode for torque control
target_frequency = 0.02
loop_duration = 1000
(kp, kd, ki, ff) = (100.0, 5.0, 0.0, 0.0)
motor_name = 207
motor_gear_ratio = 64
motor_torque_const = 0.0859
SERVER_IP = "192.168.31.50"
PORT = 1220
torque_list = [i for i in range(10, 100, 10)]  # Desired torques in arbitrary units

# Initialize the motor controller
motor_controller = MotorController(voltage, baud_rate, control_mode, target_frequency, loop_duration, kp, kd, ki, ff, motor_name)

# Initialize the Futek sensor
futek_client = FutekClient()

# Initializing korad & tonghui
korad = ka3000()
korad.setOutput(1)

# Setup power supplies
motor_controller.setup_power_supply()
time.sleep(0.5)

# Run and plot the experiment
ktau_experiment = KtauExperiment(motor_controller, motor_gear_ratio, motor_torque_const)
ktau_experiment.run_and_plot_experiment(torque_list, futek_client)

# Shutdown the motor controller
motor_controller.shutdown()
korad.setOutput(0)
