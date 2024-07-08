import time
import pyCandle
import plotly.graph_objs as go
import plotly.offline as pyo
import numpy as np

from motor_power import tongui
from Futek import FutekClient
from TestClass import MotorController  # Ensure this file is named MotorController.py
from ka3000_serial import ka3000

# Experiment parameters
voltage = 48
baud_rate = pyCandle.CAN_BAUD_2M
control_mode = pyCandle.IMPEDANCE  # Set to IMPEDANCE mode for torque control
target_frequency = 0.02
loop_duration = 1000
(kp, kd, ki, ff) = (100.0, 5.0, 0.02, 0.0)
motor_name = 207
SERVER_IP = "192.168.31.50"
PORT = 1220
torque_list = [i for i in range(0, 90, 10)]  # Desired torques in arbitrary units

# Initialize the motor controller
motor_controller = MotorController(voltage, baud_rate, control_mode, target_frequency, loop_duration, kp, kd, ki, ff, motor_name)

# Initialize the Futek sensor
futek_client = FutekClient()

# Initializing korad & tonghui
korad = ka3000()
korad.setOutput(1)

# Setup power power supplies 
motor_controller.setup_power_supply()
# korad.setCurrent(4.5)

# print(f'the motor volage is: {Mpower.getVolt()}',f'the brake current is: {korad.measureCurrent()}')
time.sleep(0.5)

if motor_controller.initialize_drives():
    motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau = motor_controller.Ktau_experiment(torque_list, futek_client)

    # Calculate the linear approximation for motor torque vs current
    m, b = np.polyfit(currents_for_Ktau, Torques_for_Ktau, 1)
    motor_fit_line = [m * x + b for x in currents_for_Ktau]

    # Calculate the linear approximation for futek torque vs current
    m_futek, b_futek = np.polyfit(currents_for_Ktau, futek_for_Ktau, 1)
    futek_fit_line = [m_futek * x + b_futek for x in currents_for_Ktau]

    # Plot the results
    trace1 = go.Scatter(
        x=time_values,
        y=futek_torques,
        mode='lines+markers',
        name='Futek Torque'
    )

    trace2 = go.Scatter(
        x=time_values,
        y=motor_torques,
        mode='lines+markers',
        name='Motor Torque'
    )

    trace3 = go.Scatter(
        x=time_values, 
        y=desired_torques,
        mode='lines+markers',
        name='Desired Torque'
    )

    trace5 = go.Scatter(
        x=currents_for_Ktau,
        y=Torques_for_Ktau,
        mode='markers',
        name='Torque vs Current',
        marker=dict(color='blue')
    )

    trace6 = go.Scatter(
        x=currents_for_Ktau,
        y=futek_for_Ktau,
        mode='markers',
        name='Futek vs Current',
        marker=dict(color='red')
    )

    # Add linear fit traces
    trace7 = go.Scatter(
        x=currents_for_Ktau,
        y=motor_fit_line,
        mode='lines',
        name=f'Motor Linear Fit: y = {m:.2f}x + {b:.2f}',
        line=dict(color='blue')
    )

    trace8 = go.Scatter(
        x=currents_for_Ktau,
        y=futek_fit_line,
        mode='lines',
        name=f'Futek Linear Fit: y = {m_futek:.2f}x + {b_futek:.2f}',
        line=dict(color='red')
    )

    layout1 = go.Layout(
        title='Motor Torque vs. Time',
        xaxis=dict(title='Time (s)'),
        yaxis=dict(title='Torque (Nm)'),
        legend=dict(x=0, y=1)
    )

    layout2 = go.Layout(
        title='Values to Calculate Ktau',
        xaxis=dict(title='Motor Current (A)'),
        yaxis=dict(title='Torque (Nm)'),
        legend=dict(x=0, y=1)
    )

    fig1 = go.Figure(data=[trace1, trace2, trace3], layout=layout1)
    fig2 = go.Figure(data=[trace5, trace6, trace7, trace8], layout=layout2)
    pyo.plot(fig1, filename='torque_comparison.html')
    pyo.plot(fig2, filename='torque_vs_current.html')

# Shutdown the motor controller
motor_controller.shutdown()
korad.setOutput(0)
