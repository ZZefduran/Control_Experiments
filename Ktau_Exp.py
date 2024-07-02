import time
import pyCandle
import plotly.graph_objs as go
import plotly.offline as pyo

from motor_power import tongui
from Futek import FutekClient
from TestClass import MotorController  # Ensure this file is named MotorController.py

# Experiment parameters
voltage = 24
baud_rate = pyCandle.CAN_BAUD_2M
control_mode = pyCandle.IMPEDANCE  # Set to IMPEDANCE mode for torque control
target_frequency = 0.02
loop_duration = 1000
(kp, kd, ki, ff) = (100.0, 5.0, 0.02, 0.0)
motor_name = 207
SERVER_IP = "192.168.31.50"
PORT = 1220
torque_list = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90]  # Desired torques in arbitrary units

# Initialize the motor controller
motor_controller = MotorController(voltage, baud_rate, control_mode, target_frequency, loop_duration, kp, kd, ki, ff, motor_name)

# Initialize the Futek sensor
futek_client = FutekClient()

# Setup power supply
motor_controller.setup_power_supply()
time.sleep(0.5)

if motor_controller.initialize_drives():
    motor_currents, motor_torques, futek_torques = motor_controller.Ktau_experiment(torque_list, futek_client)

    # Plot the results
    trace1 = go.Scatter(
        x=motor_currents,
        y=futek_torques,
        mode='lines+markers',
        name='Futek Torque'
    )

    trace2 = go.Scatter(
        x=motor_currents,
        y=motor_torques,
        mode='lines+markers',
        name='Motor Torque'
    )

    layout = go.Layout(
        title='Motor Current vs. Torque',
        xaxis=dict(title='Motor Current (A)'),
        yaxis=dict(title='Torque (Nm)'),
        legend=dict(x=0, y=1)
    )

    fig = go.Figure(data=[trace1, trace2], layout=layout)
    pyo.plot(fig, filename='torque_comparison.html')

# Shutdown the motor controller
motor_controller.shutdown()
