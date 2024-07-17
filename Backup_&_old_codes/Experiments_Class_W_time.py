import os
import time
import pyCandle
import numpy as np
import math
from motor_power import tongui
import plotly.graph_objs as go
import plotly.offline as pyo
import pandas as pd
from datetime import datetime

class MotorController:
    def __init__(self, voltage, baud_rate, control_mode, 
                 target_frequency, loop_duration, kp, kd, ki, ff, 
                 motor_name):
        self.supply = tongui()
        self.voltage = voltage
        self.candle = pyCandle.Candle(baud_rate, True)
        self.control_mode = control_mode
        self.target_frequency = target_frequency
        self.loop_duration = loop_duration
        self.motor_name = motor_name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        self.ids = []

    def set_gains(self):
        if self.control_mode == pyCandle.IMPEDANCE:
            print('Control mode is impedance')
            for md in self.candle.md80s:
                self.candle.controlMd80Mode(md.getId(), self.control_mode)
                md.setMaxTorque(150)
                md.setImpedanceControllerParams(self.kp, self.kd)
        elif self.control_mode == pyCandle.VELOCITY_PID:
            print('Control mode is velocity')
            for md in self.candle.md80s:
                self.candle.controlMd80Mode(md.getId(), self.control_mode)
                md.setMaxTorque(150)
                md.setVelocityControllerParams(self.kp, self.ki, self.kd, self.ff)
        elif self.control_mode == pyCandle.POSITION_PID:
            print('Control mode is position')
            for md in self.candle.md80s:
                self.candle.controlMd80Mode(md.getId(), self.control_mode)
                md.setMaxTorque(150)
                md.setPositionControllerParams(self.kp, self.ki, self.kd, self.ff)

    def setup_power_supply(self):
        self.supply.setOutputOn()
        time.sleep(1)
        self.supply.setVolt(self.voltage)

    def initialize_drives(self):
        self.ids = self.candle.ping()
        if not self.ids:
            print("EXIT FAILURE: No drives found")
            return False
        if len(self.ids) > 1:
            print("EXIT FAILURE: More than one motor is connected")
            return False
        for drive_id in self.ids:
            self.candle.addMd80(drive_id)
            self.candle.controlMd80SetEncoderZero(drive_id)
            self.candle.controlMd80Mode(drive_id, self.control_mode)
            self.candle.controlMd80Enable(drive_id, True)
        print(f"Motor initialized with ID: {self.ids[0]}")
        self.set_gains()
        return True

    def get_state(self):
        if self.ids:
            drive = self.candle.md80s[0]
            drive_id = drive.getId()
            position = drive.getPosition()
            velocity = drive.getVelocity()
            torque = drive.getTorque()
            print(f"Drive ID = {drive_id} Position: {position} Velocity: {velocity} Torque: {torque}")
            return {"Drive ID": drive_id, "Position": position, "Velocity": velocity, "Torque": torque}
        else:
            raise Exception("No drives initialized. Please call initialize_drives first.")

    def shutdown(self):
        self.supply.setOutputOff()
        print("Shutdown completed successfully.")


class CheckTests:
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller

    def move_motor_sine_wave(self):
        t = 0.0
        dt = self.motor_controller.target_frequency
        self.motor_controller.candle.begin()
        for md in self.motor_controller.candle.md80s:
            md.setMaxTorque(80)
        for i in range(self.motor_controller.loop_duration):
            t += dt
            position = math.sin(t) * 2.0
            self.motor_controller.candle.md80s[0].setTargetPosition(position)
            state = self.motor_controller.get_state()
            time.sleep(0.01)
        self.motor_controller.candle.end()

    def const_torque(self, tor):
        self.motor_controller.candle.begin()
        for md in self.motor_controller.candle.md80s:
            while True:
                md.setTorque(tor)
                state = self.motor_controller.get_state()
                time.sleep(0.1)
        self.motor_controller.candle.end()
        print(f"Setting constant torque: {tor}")

    def const_vel(self, vel):
        self.motor_controller.candle.begin()
        for md in self.motor_controller.candle.md80s:
            while True:
                md.setTargetVelocity(vel)
                state = self.motor_controller.get_state()
                time.sleep(0.01)
        self.motor_controller.candle.end()
        print(f"Setting constant velocity: {vel}")

    def const_pos(self, pos):
        self.motor_controller.candle.begin()
        for md in self.motor_controller.candle.md80s:
            while True:
                md.setTargetPosition(pos)
                state = self.motor_controller.get_state()
                time.sleep(0.01)
        self.motor_controller.candle.end()
        print(f"Setting constant Position: {pos}")

    def run(self):
        self.motor_controller.setup_power_supply()
        if self.motor_controller.initialize_drives():
            self.move_motor_sine_wave()
        self.motor_controller.shutdown()


class KtauExperiment:
    def __init__(self, motor_controller, motor_gear_ratio, motor_torque_constant):
        self.motor_controller = motor_controller
        self.motor_gear_ratio = motor_gear_ratio
        self.motor_torque_constant = motor_torque_constant

    def collect_data(self, torque, futek_client, motor_torques, futek_torques, desired_torques, time_values, t):
        motor_torque = self.motor_controller.candle.md80s[0].getTorque()
        futek_torque = futek_client.get_torque()
        motor_current = self.motor_controller.supply.getCurr()
        

        motor_torques.append(motor_torque)
        futek_torques.append(futek_torque)
        desired_torques.append(torque)
        time_values.append(t)

        print(f"Desired Torque: {torque} | Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")

    def ramp_up(self, torque, futek_client, motor_torques, futek_torques, desired_torques, time_values):
        t = time_values[-1] if time_values else 0  # Start from the last time value if available
        dt = 0.01  # Time step in seconds (10 milliseconds)
        
        self.motor_controller.candle.begin()
        
        start_time = time.time()
        while time.time() - start_time < 2:
            for md in self.motor_controller.candle.md80s:
                current_time = time.time() - start_time
                ramp_torque = torque * (current_time / 2)  # Linearly increase torque
                md.setTorque(ramp_torque)
                self.collect_data(ramp_torque, futek_client, motor_torques, futek_torques, desired_torques, time_values, t)
                # print(f'Ramping Down - Desired Torque: {ramp_torque}')
            
            time.sleep(dt)
            t += dt

        return t

    def hold_torque(self, torque, futek_client, motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau):
        t = time_values[-1] if time_values else 0  # Start from the last time value if available
        dt = 0.01  # Time step in seconds (10 milliseconds)

        start_time = time.time()
        while time.time() - start_time < 5:
            for md in self.motor_controller.candle.md80s:
                md.setTorque(torque)

            motor_torque = self.motor_controller.candle.md80s[0].getTorque()
            futek_torque = futek_client.get_torque()
            motor_current = self.motor_controller.supply.getCurr()
            

            motor_torques.append(motor_torque)
            futek_torques.append(futek_torque)
            desired_torques.append(torque)
            time_values.append(t)

            print(f"Holding - Desired Torque: {torque} | Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")

            time.sleep(dt)
            t += dt

            if time.time() - start_time >= 2.47 and time.time() - start_time <= 2.53:
                I = motor_torque / (self.motor_gear_ratio * self.motor_torque_constant)
                currents_for_Ktau.append(I)
                Torques_for_Ktau.append(motor_torque)
                futek_for_Ktau.append(futek_torque)

        return t

    def ramp_down(self, torque, futek_client, motor_torques, futek_torques, desired_torques, time_values):
        t = time_values[-1] if time_values else 0  # Start from the last time value if available
        dt = 0.01  # Time step in seconds (10 milliseconds)

        start_time = time.time()
        while time.time() - start_time < 2:
            for md in self.motor_controller.candle.md80s:
                current_time = time.time() - start_time
                ramp_torque = torque * (1 - (current_time / 2))  # Linearly decrease torque
                md.setTorque(ramp_torque)
                self.collect_data(ramp_torque, futek_client, motor_torques, futek_torques, desired_torques, time_values, t)

                # print(f'Ramping Down - Desired Torque: {ramp_torque}')

            time.sleep(dt)
            t += dt

        return t
    
    def calculate_linear_fit(self, x, y):
        m, b = np.polyfit(x, y, 1)
        fit_line = [m * xi + b for xi in x]
        return fit_line, m, b
    
    def run_experiment(self, torque_list, futek_client):
        motor_torques = []
        futek_torques = []
        desired_torques = []
        time_values = []  # List to store time values
        currents_for_Ktau = []
        Torques_for_Ktau = []
        futek_for_Ktau = []

        for torque in torque_list:
            t = self.ramp_up(torque, futek_client, motor_torques, futek_torques, desired_torques, time_values)
            t = self.hold_torque(torque, futek_client, motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau)
            t = self.ramp_down(torque, futek_client, motor_torques, futek_torques, desired_torques, time_values)
            time.sleep(2)
        self.motor_controller.candle.end()
        return motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau

    def save_data_and_plots(self, motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, new_Ktau):
        # Pad lists to make them the same length
        max_len = max(len(motor_torques), len(futek_torques), len(desired_torques), len(time_values), len(currents_for_Ktau), len(Torques_for_Ktau), len(futek_for_Ktau))
        motor_torques += [None] * (max_len - len(motor_torques))
        futek_torques += [None] * (max_len - len(futek_torques))
        desired_torques += [None] * (max_len - len(desired_torques))
        time_values += [None] * (max_len - len(time_values))
        currents_for_Ktau += [None] * (max_len - len(currents_for_Ktau))
        Torques_for_Ktau += [None] * (max_len - len(Torques_for_Ktau))
        futek_for_Ktau += [None] * (max_len - len(futek_for_Ktau))

        # Create a new directory with the current date and time
        current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        directory = f'/home/zzefduran/code/newBenchTest/Control_Experiments/Ktau_experiments/{current_time}'
        os.makedirs(directory, exist_ok=True)

        # Save the data to a CSV file
        data = {
            'Time(s)': time_values,
            'MotorTorque(Nm)': motor_torques,
            'FutekTorque(Nm)': futek_torques,
            'DesiredTorque(Nm)': desired_torques,
            'MotorCurrent(A)': currents_for_Ktau,
            'TorqueforKtau(Nm)': Torques_for_Ktau,
            'FutekforKtau(Nm)': futek_for_Ktau
        }
        df = pd.DataFrame(data)
        df.to_csv(os.path.join(directory, 'experiment_data.csv'), index=False)

        # Save the plots to HTML files
        pyo.plot(self.fig1, filename=os.path.join(directory, 'torque_comparison.html'), auto_open=False)
        pyo.plot(self.fig2, filename=os.path.join(directory, 'torque_vs_current.html'), auto_open=False)

        print(f"Data and plots saved in {directory}")

    def run_and_plot_experiment(self, torque_list, futek_client):
        if not self.motor_controller.initialize_drives():
            print("Failed to initialize drives")
            return

        motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau = self.run_experiment(torque_list, futek_client)

        futek_fit_line, m_futek, b_futek = self.calculate_linear_fit(currents_for_Ktau, futek_for_Ktau)
        motor_fit_line, m_motor, b_motor = self.calculate_linear_fit(currents_for_Ktau, Torques_for_Ktau)

        new_Ktau = self.motor_torque_constant * (m_futek / m_motor)

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
            name=f'Motor Linear Fit: y = {m_motor:.4f}x + {b_motor:.2f}',
            line=dict(color='blue')
        )

        trace8 = go.Scatter(
            x=currents_for_Ktau,
            y=futek_fit_line,
            mode='lines',
            name=f'Futek Linear Fit: y = {m_futek:.4f}x + {b_futek:.2f}',
            line=dict(color='red')
        )

        layout1 = go.Layout(
            title='Motor Torque vs. Time',
            xaxis=dict(title='Time (s)'),
            yaxis=dict(title='Torque (Nm)'),
            legend=dict(x=0, y=1)
        )
        # before the " New Ktau: {new_Ktau:.4f}" insert the motor type
        layout2 = go.Layout(
            title=f'New Ktau: {new_Ktau:.4f}',
            xaxis=dict(title='Motor Current (A)'),
            yaxis=dict(title='Torque (Nm)'),
            legend=dict(x=0, y=1),
        )

        self.fig1 = go.Figure(data=[trace1, trace2, trace3], layout=layout1)
        self.fig2 = go.Figure(data=[trace5, trace6, trace7, trace8], layout=layout2)
        pyo.plot(self.fig1, filename='torque_comparison.html')
        pyo.plot(self.fig2, filename='torque_vs_current.html')

        # Save data and plots
        self.save_data_and_plots(motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, new_Ktau)


