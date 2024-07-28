import time
import pyCandle
from Futek import FutekClient
from ka3000_serial import ka3000

# Import custom classes
from Ktau_Experiments_Class import MotorController, KtauExperiment


# Experiment parameters
voltage = 48.0
baud_rate = pyCandle.CAN_BAUD_2M
control_mode = pyCandle.IMPEDANCE  # Set to IMPEDANCE mode for torque control
(kp, kd, ki, ff) = (100.0, 5.0, 0.0, 0.0)
motor_name = 100
torque_list = [i for i in range(0, 35, 5)]  # Desired torques in arbitrary units

# Initialize the motor controller
motor_controller = MotorController(voltage, baud_rate, control_mode, kp, kd, ki, ff, motor_name)

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
