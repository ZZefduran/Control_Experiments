import os
import time
import pyCandle
import numpy as np
import yaml
from motor_power import tongui
import plotly.graph_objs as go
import plotly.offline as pyo
import pandas as pd
from datetime import datetime
from Futek import FutekClient
import subprocess


class MotorController:
    def __init__(self, voltage, control_mode, kp, kd, ki, ff, motor_name):
        self.supply = tongui()
        self.Futek = FutekClient()
        self.voltage = voltage
        self.candle = pyCandle.Candle(pyCandle.CAN_BAUD_2M, True)
        self.control_mode = control_mode
        self.motor_name = motor_name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        self.temperature = self.retrieve_motor_info(motor_name)
        self.motor = None

    def set_gains(self):
        if self.control_mode == pyCandle.IMPEDANCE:
            print('Control mode is impedance')
            self.candle.controlMd80Mode(self.motor.getId(), self.control_mode)
            self.motor.setMaxTorque(150)
            self.motor.setImpedanceControllerParams(self.kp, self.kd)
        elif self.control_mode == pyCandle.VELOCITY_PID:
            print('Control mode is velocity')
            self.candle.controlMd80Mode(self.motor.getId(), self.control_mode)
            self.motor.setMaxTorque(150)
            self.motor.setVelocityControllerParams(self.kp, self.ki, self.kd, self.ff)
        elif self.control_mode == pyCandle.POSITION_PID:
            print('Control mode is position')
            self.candle.controlMd80Mode(self.motor.getId(), self.control_mode)
            self.motor.setMaxTorque(150)
            self.motor.setPositionControllerParams(self.kp, self.ki, self.kd, self.ff)

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
        drive_id = self.ids[0]
        self.candle.addMd80(drive_id)
        self.motor = self.candle.md80s[0]
        self.candle.controlMd80SetEncoderZero(drive_id)
        self.candle.controlMd80Mode(drive_id, self.control_mode)
        self.candle.controlMd80Enable(drive_id, True)
        print(f"Motor initialized with ID: {drive_id}")
        self.set_gains()
        return True

    def get_state(self):
        if self.motor:
            drive_id = self.motor.getId()
            position = self.motor.getPosition()
            velocity = self.motor.getVelocity()
            torque = self.motor.getTorque()
            print(f"Drive ID = {drive_id} Position: {position} Velocity: {velocity} Torque: {torque}")
            return {"Drive ID": drive_id, "Position": position, "Velocity": velocity, "Torque": torque}
        else:
            raise Exception("No drives initialized. Please call initialize_drives first.")

    def shutdown(self):
        self.supply.setOutputOff()
        print("Shutdown completed successfully.")

    def Futek_zero(self):
        self.Futek.set_zero()

    def retrieve_motor_info(self, motor_id):
        result = subprocess.run(['mdtool', 'setup', 'info', str(motor_id)], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error running mdtool: {result.stderr}")
            return None, None
        # gear_ratio = None
        # torque_constant = None
        Temperature = None ###
        for line in result.stdout.splitlines():
            if 'MOSFET temperature' in line:####
                Temperature = float(line.split(':')[1].strip().split()[0])####
        return Temperature  ####


class KtauExperiment:
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller
        self.motor_temperature = self.motor_controller.temperature ####
        print(f"Motor temperatue is:{self.motor_temperature}")

    def retrieve_motor_temperature(self, motor_id):
        result = subprocess.run(['mdtool', 'setup', 'info', str(motor_id)], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error running mdtool: {result.stderr}")
            return None

        temperature = None
        print("mdtool output:", result.stdout)  # Print the entire output for debugging
        
        for line in result.stdout.splitlines():
            if 'MOSFET temperature' in line:
                try:
                    temperature = float(line.split(':')[1].strip().split()[0])
                    print(f"Parsed temperature: {temperature}")
                except ValueError:
                    print(f"Failed to parse temperature from line: {line}")
        
        if temperature is None:
            print("Temperature not found in the mdtool output.")
        
        return temperature

    def load_params(self, motor_type):
        # Load the YAML file
        with open('experiment_params.yaml', 'r') as file:
            self.all_params = yaml.safe_load(file)
        
        # Extract parameters for the specified motor type
        if motor_type in self.all_params:
            params = self.all_params[motor_type]
            return params
        else:
            raise ValueError(f"Motor type {motor_type} not found in YAML file")

    def collect_data(self, torque, futek_client, motor_torques, futek_torques, time_values, t):
        motor_torque = self.motor_controller.motor.getTorque()
        motor_current = self.motor_controller.supply.getCurr()
        futek_torque = futek_client.get_torque()
        motor_torques.append(motor_torque)
        futek_torques.append(futek_torque)
        time_values.append(t)
        print(f"| Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")

    def ramp_up(self, torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau):
        t = time_values[-1] if time_values else 0
        dt = 0.01
        self.motor_controller.candle.begin()
        count = 0
        while True:
            ramp_torque = torque * (count / 50)
            self.motor_controller.motor.setTorque(ramp_torque)
            self.collect_data(ramp_torque, futek_client, motor_torques, futek_torques, time_values, t)
            if abs(self.motor_controller.motor.getTorque() - torque) <= 0.1:
                break
            elif self.motor_controller.motor.getTorque() >= torque + 0.5:
                break

            temperature_f_print = self.motor_controller.retrieve_motor_info(self.motor_controller.motor.getId())
            print(f"Motor temperature is: {temperature_f_print}")

            time.sleep(dt)
            count += 1
            t += dt

        return t, count

    def hold_torque(self, torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, count):
        t = time_values[-1] if time_values else 0
        dt = 0.01
        start_time = time.time()
        while time.time() - start_time < 5:
            self.motor_controller.motor.setTorque(torque * (count / 50))
            motor_torque = self.motor_controller.motor.getTorque()
            motor_current = self.motor_controller.supply.getCurr()
            futek_torque = futek_client.get_torque()
            motor_torques.append(motor_torque)
            futek_torques.append(futek_torque)
            time_values.append(t)
            print(f"Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")
            time.sleep(dt)
            t += dt

        return t, count

    def ramp_down(self, torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, count):
        t = time_values[-1] if time_values else 0
        dt = 0.01
        while count > 0:
            ramp_torque = torque * (count / 50)
            self.motor_controller.motor.setTorque(ramp_torque)
            self.collect_data(ramp_torque, futek_client, motor_torques, futek_torques, time_values, t)
            count -= 1
            time.sleep(dt)
            t += dt
            print("Down ramp")
        return t

    def calculate_linear_fit(self, x, y):
        m, b = np.polyfit(x, y, 1)
        fit_line = [m * xi + b for xi in x]
        return fit_line, m, b

    def run_experiment(self, torque_list, futek_client):
        motor_torques = []
        futek_torques = []
        time_values = []
        currents_for_Ktau = []
        Torques_for_Ktau = []
        futek_for_Ktau = []
        for torque in torque_list:
            t, count = self.ramp_up(torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau)
            t, count = self.hold_torque(torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, count)
            t = self.ramp_down(torque, futek_client, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, count)
            self.motor_controller.motor.setTorque(0)
            self.motor_controller.Futek_zero()
            time.sleep(1.5)
        self.motor_controller.candle.end()
        return motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau

    def save_data_and_plots(self, motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, new_Ktau):
        # Pad lists to make them the same length
        max_len = max(len(motor_torques), len(futek_torques), len(time_values), len(currents_for_Ktau), len(Torques_for_Ktau), len(futek_for_Ktau))
        motor_torques += [None] * (max_len - len(motor_torques))
        futek_torques += [None] * (max_len - len(futek_torques))
        time_values += [None] * (max_len - len(time_values))
        currents_for_Ktau += [None] * (max_len - len(currents_for_Ktau))
        Torques_for_Ktau += [None] * (max_len - len(Torques_for_Ktau))
        futek_for_Ktau += [None] * (max_len - len(futek_for_Ktau))

        # Create a new directory with the current date and time
        current_date = datetime.now().strftime('%Y-%m-%d')
        daily_directory = f'/home/zzefduran/code/newBenchTest/Control_Experiments/Ktau_experiments/{current_date}'
        os.makedirs(daily_directory, exist_ok=True)

        # Create a subdirectory for each experiment with a unique timestamp
        current_time = datetime.now().strftime('%H:%M:%S')
        experiment_directory = os.path.join(daily_directory, current_time)
        os.makedirs(experiment_directory, exist_ok=True)

    def run_and_plot_experiment(self, torque_list, futek_client):
        self.current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        if not self.motor_controller.initialize_drives():
            print("Failed to initialize drives")
            return
        motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau = self.run_experiment(torque_list, futek_client)
        futek_fit_line, m_futek, b_futek = self.calculate_linear_fit(currents_for_Ktau, futek_for_Ktau)
        motor_fit_line, m_motor, b_motor = self.calculate_linear_fit(currents_for_Ktau, Torques_for_Ktau)
        new_Ktau = self.motor_torque_constant * (m_futek / m_motor)
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
        trace4 = go.Scatter(
            x=currents_for_Ktau,
            y=Torques_for_Ktau,
            mode='markers',
            name='Torque vs Current',
            marker=dict(color='blue')
        )
        trace5 = go.Scatter(
            x=currents_for_Ktau,
            y=futek_for_Ktau,
            mode='markers',
            name='Futek vs Current',
            marker=dict(color='red')
        )
        trace6 = go.Scatter(
            x=currents_for_Ktau,
            y=motor_fit_line,
            mode='lines',
            name=f'Motor Linear Fit: y = {m_motor:.4f}x + {b_motor:.2f}',
            line=dict(color='blue')
        )
        trace7 = go.Scatter(
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
        layout2 = go.Layout(
            title=f'{self.current_time}<br>New Ktau: {new_Ktau:.4f}',
            xaxis=dict(title='Motor Current (A)'),
            yaxis=dict(title='Torque (Nm)'),
            legend=dict(x=0, y=1),
        )
        self.fig1 = go.Figure(data=[trace1, trace2], layout=layout1)
        self.fig2 = go.Figure(data=[trace4, trace5, trace6, trace7], layout=layout2)
        pyo.plot(self.fig1, filename='torque_comparison.html')
        pyo.plot(self.fig2, filename='torque_vs_current.html')
        self.save_data_and_plots(motor_torques, futek_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau, new_Ktau)