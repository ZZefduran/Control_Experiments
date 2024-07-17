import time
import pyCandle
from Futek import FutekClient
from ka3000_serial import ka3000

# Import custom classes
from TestClass import MotorController  # Ensure this file is named MotorController.py
from TestClass import KtauExperiment

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








# import time
# import pyCandle
# from TestClass import MotorController
# from TestClass import CheckTests


# if __name__ == "__main__":
#     voltage = 48
#     baud_rate = pyCandle.CAN_BAUD_2M
#     control_mode = pyCandle.VELOCITY_PID  # Set to velocity control mode for testing
#     target_frequency = 0.02
#     loop_duration = 10000 
#     (kp, kd, ki, ff) = (50.0, 5.0, 0.02, 0.0)
#     motor_name = 207

#     # Initialize the motor controller
#     motor_controller = MotorController(voltage, baud_rate,
#                                         control_mode, target_frequency, 
#                                         loop_duration, kp, kd, ki, 
#                                         ff, motor_name)
#     motor_controller.setup_power_supply()
#     time.sleep(0.5)


#     Check_Tests = CheckTests(MotorController)
# ### const
#     motor_controller.initialize_drives()
#     Check_Tests.const_vel(10)  # Set a small velocity
#     # motor_controller.const_pos(8)
#     # motor_controller.const_torque(5)
#     # Shutdown the motor controller
#     motor_controller.shutdown()
