import time
import pyCandle
import plotly.graph_objs as go
import plotly.offline as pyo

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
torque_list = [i for i in range(0, 60, 10)]  # Desired torques in arbitrary units

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
    motor_currents, motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau = motor_controller.Ktau_experiment(torque_list, futek_client)

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
    
    # trace4 = go.Scatter(
    #     x=time_values,
    #     y=motor_torques,
    #     mode='lines+markers',
    #     name='Motor Torque vs Time'
    # )

    trace5 = go.Scatter(
    x=currents_for_Ktau,
    y=Torques_for_Ktau,
    mode='lines+markers',
    name='Torque vs current'
    )

    trace6 = go.Scatter(
    x=currents_for_Ktau,
    y=futek_for_Ktau,
    mode = 'lines+markers',
    name = 'futek vs current'
    )

    layout1 = go.Layout(
        title='Motor Torque vs. Time',
        xaxis=dict(title='Motor Current (A)'),
        yaxis=dict(title='Torque (Nm)'),
        legend=dict(x=0, y=1)
    )
    
    # layout2 = go.Layout(
    #     title='Motor Torque vs. Time',
    #     xaxis=dict(title='Time (s)'),
    #     yaxis=dict(title='Torque (Nm)'),
    #     legend=dict(x=0, y=1)
    # )


    layout2 = go.Layout(
        title='Values to calculate Ktau',
        xaxis=dict(title='Motor Current (A)'),
        yaxis=dict(title='Torques (Nm)'),
        legend=dict(x=0, y=1)
    )


    fig1 = go.Figure(data=[trace1, trace2, trace3], layout=layout1)
    fig2 = go.Figure(data = [trace5,trace6], layout=layout2)
    pyo.plot(fig1, filename='torque_comparison.html')
    pyo.plot(fig2, filename='torque_vs_current.html')


# Shutdown the motor controller
motor_controller.shutdown()
