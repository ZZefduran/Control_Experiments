import time
import pyCandle
from Futek import FutekClient
from ka3000_serial import ka3000
from motor_power import tongui

# Import custom classes
from Temperature_exp_class import MotorController, KtauExperiment

# set up motor supplier
# supplier = tongui()
# supplier.setOutputOn
# time.sleep(2)


# Experiment parameters
voltage = 48.0
baud_rate = pyCandle.CAN_BAUD_2M
control_mode = pyCandle.VELOCITY_PID  # Set to IMPEDANCE mode for torque control
(kp, kd, ki, ff) = (100.0, 5.0, 0.0, 0.0)
motor_name = 69
torque_list =  [i for i in range(10, 120, 10)]   #[i for i in range(0, 35, 5)]  # Desired torques in arbitrary units

# Initialize the motor controller
motor_controller = MotorController(voltage, baud_rate, control_mode, kp, kd, ki, ff, motor_name)

# Initialize the Futek sensor
futek_client = FutekClient()

# Initializing korad
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







# import pyCandle
# import time
# import math
# import sys  
# from motor_power import tongui
# import subprocess

# power = tongui()
# power.setOutputOn()

# def retrieve_motor_temperature(motor_id):
#     result = subprocess.run(['mdtool', 'setup', 'info', str(motor_id)], capture_output=True, text=True)
#     if result.returncode != 0:
#         print(f"Error running mdtool: {result.stderr}")
#         return None

#     temperature = None
#     print("mdtool output:", result.stdout)  # Print the entire output for debugging
    
#     for line in result.stdout.splitlines():
#         if 'MOSFET temperature' in line:
#             try:
#                 temperature = float(line.split(':')[1].strip().split()[0])
#                 print(f"Parsed temperature: {temperature}")
#             except ValueError:
#                 print(f"Failed to parse temperature from line: {line}")
    
#     if temperature is None:
#         print("Temperature not found in the mdtool output.")
    
#     return temperature


# # Create CANdle object and set FDCAN baudrate to 1Mbps
# candle = pyCandle.Candle(pyCandle.CAN_BAUD_2M, True)

# # Ping FDCAN bus in search of drives
# ids = candle.ping()
# print(ids)

# if len(ids) == 0: # If no drives found -> quit
#     sys.exit("EXIT FALIURE") 

# # Add all found to the update list
# candle.addMd80(ids[0])
# motor = candle.md80s[0]

# # Now we shall loop over all found drives to change control mode and enable them one by one
# candle.controlMd80SetEncoderZero(motor)      #  Reset encoder at current position
# candle.controlMd80Mode(motor, pyCandle.VELOCITY_PID)    # Set mode to impedance control
# candle.controlMd80Enable(motor, True)     # Enable the drive


# t = 0.0
# dt = 0.04

# # Begin update loop (it starts in the background)
# candle.begin()

# while motor.getVelocity() < 2.3:
#     motor.setTargetPosition(math.exp(0.1*t)*0.1) 
#     print(f'vel is: {motor.getVelocity()}')

# while True:
#     motor.setTargetPosition(math.exp(0.1*t)*0.1) 



#     t = t + dt
#     time.sleep(0.01)  # Add some delay

    

# # Close the update loop
# candle.end()
# power.setOutputOff()


# sys.exit("EXIT SUCCESS")



