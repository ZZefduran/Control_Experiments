# import time
# import pyCandle
# from Futek import FutekClient
# from ka3000_serial import ka3000

# # Import custom classes
# from devices_check_class import MotorController  # Ensure this file is named MotorController.py
# from devices_check_class import CheckTests

# # Experiment parameters
# voltage = 48
# baud_rate = pyCandle.CAN_BAUD_2M
# control_mode = pyCandle.IMPEDANCE  # Set to IMPEDANCE mode for torque control
# target_frequency = 0.02
# loop_duration = 1000
# SERVER_IP = "192.168.31.50"
# PORT = 1220
# (kp, kd, ki, ff) = (100.0, 5.0, 0.0, 0.0)
# motor_name = 207
# torque_list = [i for i in range(0, 90, 10)]  # Desired torques in arbitrary units

# # Initialize the motor controller
# motor_controller = MotorController(voltage, baud_rate, control_mode, target_frequency, loop_duration, kp, kd, ki, ff, motor_name)

# # Initialize the Futek sensor
# futek_client = FutekClient()

# # Initializing korad & tonghui
# korad = ka3000()
# korad.setOutput(1)

# # Setup power supplies
# motor_controller.setup_power_supply()
# time.sleep(0.5)
 


# from ka3000_serial import ka3000
# import pyCandle



# from TestClass import MotorController


# from motor_power import tongui


# korad = ka3000()
# # mot_supply = tongui()
# # # f=0

# for voltage in [26,25,24]:
#     korad.setVolt(voltage)
#     time.sleep(1)

# voltage = korad.measureVolt()
# print(f'the voltage is:{voltage}')

# current = korad.measureCurrent()
# print(f'the current is: {current}')

# for current in [0.2,0.3,0.4]:
# current = 0.5
# korad.setCurrent(2)
# time.sleep(1)
    

# hellowwww


# from motor_power import tongui
# supply = tongui()
# import time
# # Perform operations on the power supply
# supply.setOutputOn()
# supply.setVolt(45)
# time.sleep(3)
# supply.setOutputOff()
# # Close the connection
# supply.close()

# supply.setOutputOn()
# voltage = supply.getCurr()
# print(f'the voltage is:{voltage}')
# time.sleep(5)
# supply.setOutputOff()

# for current in [9,10,11, 12]:
#     supply.setCurrent(current)
#     time.sleep(2)

# supply.setCurrent(2.5)
# print(f'the current is:{3}')

# script for checking connected ports
# import pyvisa
# def list_visa_resources():
#     rm = pyvisa.ResourceManager()
#     resources = rm.list_resources()
#     return resources

# available_ports = list_visa_resources()
# print("Available VISA Resources:")
# for port in available_ports:
#     print(port)




# candle = pyCandle()
# print(f'Ktau is :{candle.md}')



# import serial

# try:
#     # Open the serial port
#     ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)  # Adjust baudrate and timeout as needed

#     # Do something with the serial port
#     ser.write(b'Hello, serial port!')

#     # Close the serial port when done
#     ser.close()

# except serial.SerialException as e:
#     print("Error opening serial port:", e)















# import os
# import time
# import pyCandle
# import numpy as np
# import math
# from motor_power import tongui
# import plotly.graph_objs as go
# import plotly.offline as pyo
# import pandas as pd
# from datetime import datetime
# from Futek import FutekClient
# import subprocess

# class MotorController:
#     def __init__(self, voltage, control_mode, 
#                   kp, kd, ki, ff, 
#                  motor_name):
#         self.supply = tongui()
#         self.Futek = FutekClient()
#         self.voltage = voltage
#         self.candle = pyCandle.Candle(pyCandle.CAN_BAUD_2M, True)
#         self.control_mode = control_mode
#         self.motor_name = motor_name
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.ff = ff
#         self.temperature = self.retrieve_motor_info(motor_name)
#         self.motor = None

#     def set_gains(self):
#         if self.control_mode == pyCandle.IMPEDANCE:
#             print('Control mode is impedance')
#             for md in self.candle.md80s:
#                 self.candle.controlMd80Mode(md.getId(), self.control_mode)
#                 md.setMaxTorque(150)
#                 md.setImpedanceControllerParams(self.kp, self.kd)
#         elif self.control_mode == pyCandle.VELOCITY_PID:
#             print('Control mode is velocity')
#             for md in self.candle.md80s:
#                 self.candle.controlMd80Mode(md.getId(), self.control_mode)
#                 md.setMaxTorque(150)
#                 md.setVelocityControllerParams(self.kp, self.ki, self.kd, self.ff)
#         elif self.control_mode == pyCandle.POSITION_PID:
#             print('Control mode is position')
#             for md in self.candle.md80s:
#                 self.candle.controlMd80Mode(md.getId(), self.control_mode)
#                 md.setMaxTorque(150)
#                 md.setPositionControllerParams(self.kp, self.ki, self.kd, self.ff)

#     def setup_power_supply(self):
#         self.supply.setOutputOn()
#         time.sleep(1)
#         self.supply.setVolt(self.voltage)

#     def initialize_drives(self):
#         self.ids = self.candle.ping()
#         if not self.ids:
#             print("EXIT FAILURE: No drives found")
#             return False
#         if len(self.ids) > 1:
#             print("EXIT FAILURE: More than one motor is connected")
#             return False
#         for drive_id in self.ids:
#             self.candle.addMd80(drive_id)
#             self.candle.controlMd80SetEncoderZero(drive_id)
#             self.candle.controlMd80Mode(drive_id, self.control_mode)
#             self.candle.controlMd80Enable(drive_id, True)
#         print(f"Motor initialized with ID: {self.ids[0]}")
#         self.set_gains()
#         return True

#     def get_state(self):
#         if self.ids:
#             drive = self.candle.md80s[0]
#             drive_id = drive.getId()
#             position = drive.getPosition()
#             velocity = drive.getVelocity()
#             torque = drive.getTorque()
#             print(f"Drive ID = {drive_id} Position: {position} Velocity: {velocity} Torque: {torque}")
#             return {"Drive ID": drive_id, "Position": position, "Velocity": velocity, "Torque": torque}
#         else:
#             raise Exception("No drives initialized. Please call initialize_drives first.")

#     def shutdown(self):
#         self.supply.setOutputOff()
#         print("Shutdown completed successfully.")

#     def Futek_zero(self):
#         self.Futek.set_zero()

#     def retrieve_motor_info(self, motor_id):
#         # Run the mdtool command to get motor information
#         result = subprocess.run(['mdtool', 'setup', 'info', str(motor_id)], capture_output=True, text=True)

#         # Check if the command was successful
#         if result.returncode != 0:
#             print(f"Error running mdtool: {result.stderr}")
#             return None, None
#         Temperature = None ###
#         for line in result.stdout.splitlines():
#             if 'MOSFET temperature' in line:####
#                 Temperature = float(line.split(':')[1].strip().split()[0])####

#         return Temperature



# class KtauExperiment:
#     def __init__(self, motor_controller):
#         self.motor_controller = motor_controller
#         self.motor_temperature = self.motor_controller.temperature ####
#         print(f"Motor temperatue is:{self.motor_temperature}")



#     def collect_data(self, torque, futek_client, motor_torques, futek_torques, time_values, t):
#         motor_torque = self.motor_controller.candle.md80s[0].getTorque()
#         motor_current = self.motor_controller.supply.getCurr()
#         futek_torque = futek_client.get_torque()

#         motor_torques.append(motor_torque)
#         futek_torques.append(futek_torque)
#         time_values.append(t)

#         print(f"| Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")


#     def ramp_up(self, torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau):
#         t = time_values[-1] if time_values else 0  # Start from the last time value if available
#         dt = 0.01  # Time step in seconds (10 milliseconds)
        
#         self.motor_controller.candle.begin()
        
#         count = 0
#         while True:
#             for md in self.motor_controller.candle.md80s:
#                 ramp_torque = torque * (count / 50)  # Linearly increase torque
#                 md.setTorque(ramp_torque)
#                 self.collect_data(ramp_torque, futek_client, motor_torques, futek_torques, time_values, t)
#                 temperature_f_print = self.motor_controller.retrieve_motor_info(self.motor_controller.motor.getId())
#                 print(f"Motor temperature is: {temperature_f_print}")
#                 print("up")
#             if abs(self.motor_controller.candle.md80s[0].getTorque() - torque) <= 0.1:
#                 break
#             elif self.motor_controller.candle.md80s[0].getTorque() >= torque +0.5:
#                 break
#             elif count >= 100:
#                 break





#             time.sleep(dt)
#             count += 1
#             t += dt

#         return t, count

#     def hold_torque(self, torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, count):
#         t = time_values[-1] if time_values else 0  # Start from the last time value if available
#         dt = 0.01  # Time step in seconds (10 milliseconds)

#         start_time = time.time()
#         while time.time() - start_time < 5:
#             for md in self.motor_controller.candle.md80s:
#                 md.setTorque(torque * (count / 50))

#             motor_torque = self.motor_controller.candle.md80s[0].getTorque()
#             motor_current = self.motor_controller.supply.getCurr()
#             futek_torque = futek_client.get_torque()

#             motor_torques.append(motor_torque)
#             futek_torques.append(futek_torque)
#             time_values.append(t)

#             print(f"Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")

#             temperature_f_print = self.motor_controller.retrieve_motor_info(self.motor_controller.motor.getId())
#             print(f"Motor temperature is: {temperature_f_print}")
#             print(f"hold")


#             time.sleep(dt)
#             t += dt
#         return t, count

#     def ramp_down(self, torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, count):
#         t = time_values[-1] if time_values else 0  # Start from the last time value if available
#         dt = 0.01  # Time step in seconds (10 milliseconds)

#         while count > 0:
#             for md in self.motor_controller.candle.md80s:
#                 ramp_torque = torque * (count / 50)  # Linearly decrease torque
#                 md.setTorque(ramp_torque)
#                 self.collect_data(ramp_torque, futek_client, motor_torques, futek_torques, time_values, t)
#                 temperature_f_print = self.motor_controller.retrieve_motor_info(self.motor_controller.motor.getId())
#                 print(f"Motor temperature is: {temperature_f_print}")
#             count -= 1


            
#             time.sleep(dt)
#             t += dt
#             print("Down ramp")

#         start_time = time.time()
#         while time.time() - start_time < 4:
#             for md in self.motor_controller.candle.md80s:
#                 zero_torque = 0
#                 md.setTorque(zero_torque)
#                 self.collect_data(zero_torque, futek_client, motor_torques, futek_torques, time_values, t)

#             time.sleep(dt)
#             t += dt
#             print("Down ramp")

#         return t
    
#     def calculate_linear_fit(self, x, y):
#         m, b = np.polyfit(x, y, 1)
#         fit_line = [m * xi + b for xi in x]
#         return fit_line, m, b
    
#     def run_experiment(self, torque_list, futek_client):
#         motor_torques = []
#         futek_torques = []
#         time_values = []  # List to store time values
#         currents_for_Ktau = []
#         Torques_for_Ktau = []
#         futek_for_Ktau = []

#         for torque in torque_list:
#             t, count = self.ramp_up(torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau)
#             t, count = self.hold_torque(torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, count)
#             t = self.ramp_down(torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, count)
#             for md in self.motor_controller.candle.md80s:
#                 md.setTorque(0)
#                 self.motor_controller.Futek_zero()
#             time.sleep(1.5)
#         self.motor_controller.candle.end()
#         return motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau

#     def save_data_and_plots(self, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, new_Ktau):
#         # Pad lists to make them the same length
#         max_len = max(len(motor_torques), len(futek_torques), len(time_values), len(currents_for_Ktau), len(Torques_for_Ktau), len(futek_for_Ktau))
#         motor_torques += [None] * (max_len - len(motor_torques))
#         futek_torques += [None] * (max_len - len(futek_torques))
#         time_values += [None] * (max_len - len(time_values))
#         currents_for_Ktau += [None] * (max_len - len(currents_for_Ktau))
#         Torques_for_Ktau += [None] * (max_len - len(Torques_for_Ktau))
#         futek_for_Ktau += [None] * (max_len - len(futek_for_Ktau))

#         # Create a new directory with the current date and time
#         current_date = datetime.now().strftime('%Y-%m-%d')
#         daily_directory = f'/home/zzefduran/code/newBenchTest/Control_Experiments/Ktau_experiments/{current_date}'
#         os.makedirs(daily_directory, exist_ok=True)

#         # Create a subdirectory for each experiment with a unique timestamp
#         current_time = datetime.now().strftime('%H:%M:%S')
#         experiment_directory = os.path.join(daily_directory, current_time)
#         os.makedirs(experiment_directory, exist_ok=True)


#         # Save the data to a CSV file
#         data = {
#             'Time(s)': time_values,
#             'MotorTorque(Nm)': motor_torques,
#             'FutekTorque(Nm)': futek_torques,
#             'MotorCurrent(A)': currents_for_Ktau,
#             'TorqueforKtau(Nm)': Torques_for_Ktau,
#             'FutekforKtau(Nm)': futek_for_Ktau
#         }
#         df = pd.DataFrame(data)
#         df.to_csv(os.path.join(experiment_directory, 'experiment_data.csv'), index=False)

#         # Save the plots to HTML files
#         pyo.plot(self.fig1, filename=os.path.join(experiment_directory, 'torque_comparison.html'), auto_open=False)
#         pyo.plot(self.fig2, filename=os.path.join(experiment_directory, 'torque_vs_current.html'), auto_open=False)

#         print(f"Data and plots saved in {experiment_directory}")




#     def run_and_plot_experiment(self, torque_list, futek_client):
#         if not self.motor_controller.initialize_drives():
#             print("Failed to initialize drives")
#             return

#         motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau = self.run_experiment(torque_list, futek_client)

#         futek_fit_line, m_futek, b_futek = self.calculate_linear_fit(currents_for_Ktau, futek_for_Ktau)
#         motor_fit_line, m_motor, b_motor = self.calculate_linear_fit(currents_for_Ktau, Torques_for_Ktau)

#         new_Ktau = self.motor_torque_constant * (m_futek / m_motor)

#         # Plot the results
#         trace1 = go.Scatter(
#             x=time_values,
#             y=futek_torques,
#             mode='lines+markers',
#             name='Futek Torque'
#         )

#         trace2 = go.Scatter(
#             x=time_values,
#             y=motor_torques,
#             mode='lines+markers',
#             name='Motor Torque'
#         )

#         trace4 = go.Scatter(
#             x=currents_for_Ktau,
#             y=Torques_for_Ktau,
#             mode='markers',
#             name='Torque vs Current',
#             marker=dict(color='blue')
#         )

#         trace5 = go.Scatter(
#             x=currents_for_Ktau,
#             y=futek_for_Ktau,
#             mode='markers',
#             name='Futek vs Current',
#             marker=dict(color='red')
#         )

#         # Add linear fit traces
#         trace6 = go.Scatter(
#             x=currents_for_Ktau,
#             y=motor_fit_line,
#             mode='lines',
#             name=f'Motor Linear Fit: y = {m_motor:.4f}x + {b_motor:.2f}',
#             line=dict(color='blue')
#         )

#         trace7 = go.Scatter(
#             x=currents_for_Ktau,
#             y=futek_fit_line,
#             mode='lines',
#             name=f'Futek Linear Fit: y = {m_futek:.4f}x + {b_futek:.2f}',
#             line=dict(color='red')
#         )

#         layout1 = go.Layout(
#             title='Motor Torque vs. Time',
#             xaxis=dict(title='Time (s)'),
#             yaxis=dict(title='Torque (Nm)'),
#             legend=dict(x=0, y=1)
#         )
        
#         layout2 = go.Layout(
#             title=f'New Ktau: {new_Ktau:.4f}',
#             xaxis=dict(title='Motor Current (A)'),
#             yaxis=dict(title='Torque (Nm)'),
#             legend=dict(x=0, y=1),
#         )

#         self.fig1 = go.Figure(data=[trace1, trace2], layout=layout1)
#         self.fig2 = go.Figure(data=[trace4, trace5, trace6, trace7], layout=layout2)
#         pyo.plot(self.fig1, filename='torque_comparison.html')
#         pyo.plot(self.fig2, filename='torque_vs_current.html')

#         # Save data and plots
#         self.save_data_and_plots(motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, new_Ktau)
   
