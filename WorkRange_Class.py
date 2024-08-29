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
from Futek import FutekClient
from ka3000_serial import ka3000



class MotorController:
    def __init__(self, kp, kd, ki, ff, motor_name):
        self.supply = tongui()
        self.Futek = FutekClient()
        self.candle = pyCandle.Candle(pyCandle.CAN_BAUD_2M, True)
        self.control_mode = pyCandle.VELOCITY_PID 
        self.motor_name = motor_name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        self.korad = ka3000()
        self.motor = None

    def set_gains(self):
        if self.control_mode == pyCandle.IMPEDANCE:
            print('Control mode is impedance')
            self.motor.setMaxTorque(150)
            self.motor.setImpedanceControllerParams(self.kp, self.kd)
        elif self.control_mode == pyCandle.VELOCITY_PID:
            print('Control mode is velocity')
            self.motor.setMaxTorque(150)
            self.motor.setVelocityControllerParams(self.kp, self.ki, self.kd, self.ff)
        elif self.control_mode == pyCandle.POSITION_PID:
            print('Control mode is position')
            self.motor.setMaxTorque(150)
            self.motor.setPositionControllerParams(self.kp, self.ki, self.kd, self.ff)
    
    def setup_power_supply(self):
        self.supply.setOutputOn()
        time.sleep(1)
        self.supply.setVolt(48.0)

    def initialize_drive(self):
        self.ids = self.candle.ping()
        if not self.ids:
            print("EXIT FAILURE: No drives found")
            return False
        if len(self.ids) > 1:
            print("EXIT FAILURE: More than one motor is connected")
            return False
        drive_id = self.ids[0]
        self.candle.addMd80(drive_id)  # Add the motor using the CANdle interface
        self.motor = self.candle.md80s[0]  # Retrieve the motor object from the list of added motors
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
            # print(f"Drive ID = {drive_id} Position: {position} Velocity: {velocity} Torque: {torque}")
            return {"Drive ID": drive_id, "Position": position, "Velocity": velocity, "Torque": torque}
        else:
            raise Exception("No motor initialized. Please call initialize_drive first.")

    def shutdown(self):
        self.supply.setOutputOff()
        print("Shutdown completed successfully.")


class MaxVelocityExperiment:
    def __init__(self, motor_controller, max_vel):
        self.motor_controller = motor_controller
        self.time_values = []
        self.velocity_values = []
        self.velocity = []
        self.torque_values = []
        self.max_vel = max_vel


    def ramp_up(self):
        t = 0.0
        dt = 0.01  # Time step
        self.motor_controller.candle.begin()
        max_vel = 2.2

        print("Starting max velocity test...")
        start_time = time.time()
        
        while time.time() - start_time < 30:
            t += dt
            state = self.motor_controller.get_state()
            target_velocity = math.exp(0.1 * t) * 0.05
            if state['Velocity'] < max_vel:
                self.motor_controller.motor.setTargetVelocity(target_velocity)
            else:
                now_vel = target_velocity
                break

            # Get current state
            self.time_values.append(t)
            self.velocity_values.append(state['Velocity'])

            print(f"Time: {t:.2f}s | Velocity: {state['Velocity']:.2f}")

            time.sleep(dt)
        return now_vel
    
    def keep_vel(self,time_values,now_vel):
        t = time_values[-1] if time_values else 0.0
        dt = 0.01

        start_time = time.time()
        while time.time() - start_time < 3:
            t += dt
            state = self.motor_controller.get_state()
            self.motor_controller.motor.setTargetVelocity(now_vel)

            self.time_values.append(t)
            self.velocity_values.append(state['Velocity'])

            print(f"Time: {t:.2f}s | Velocity: {state['Velocity']:.2f}")
            time.sleep(dt)

    def break_motor(self,now_vel, time_values):
        t = time_values[-1] if time_values else 0.0
        dt = 0.01
        max_curr = 4.5
        list = np.arange(0, 5, 0.025)
        i = 0

        while self.motor_controller.korad.measureCurrent() < max_curr:
            t += dt
            state = self.motor_controller.get_state()
            self.motor_controller.motor.setTargetVelocity(now_vel)

            self.time_values.append(t)
            self.velocity_values.append(state['Velocity'])
            self.velocity.append(state['Velocity'])
            self.torque_values.append(state['Torque'])
            
            self.motor_controller.korad.setCurrent(list[i])
            i+=1

            print(f"Time: {t:.2f}s | Velocity: {state['Velocity']:.2f}")
            time.sleep(dt)

            if state['Velocity'] < 0.1:
                break


            


    def save_and_plot_results(self):
        # Save results to a CSV file
        current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        directory = f'/home/zzefduran/code/newBenchTest/Control_Experiments/MaxVelocityExperiments/{current_time}'
        os.makedirs(directory, exist_ok=True)

        max_len = max(len(self.time_values), len(self.torque_values), len(self.velocity_values), len(self.velocity))
        self.torque_values += [None] * (max_len - len(self.torque_values))
        self.time_values += [None] * (max_len - len(self.time_values))
        self.velocity += [None] * (max_len - len(self.velocity))
        self.velocity_values += [None] * (max_len - len(self.velocity_values))

        data = {
            'Time(s)': self.time_values,
            'All Exp Velocity(rad/s)': self.velocity_values,
            'Torque(Nm)': self.torque_values,
            'Brake Velocity(rad/s)': self.velocity
            
        }


        df = pd.DataFrame(data)
        df.to_csv(os.path.join(directory, 'velocity_experiment_data.csv'), index=False)

        # Plot results
        trace = go.Scatter(x=self.time_values, y=self.velocity_values, mode='lines+markers', name='Actual Velocity')
        layout = go.Layout(title='Motor Velocity Over Time', xaxis=dict(title='Time (s)'), yaxis=dict(title='Velocity (rad/s)'))
        fig = go.Figure(data=[trace], layout=layout)
        pyo.plot(fig, filename=os.path.join(directory, 'velocity_plot.html'), auto_open=False)

        trace2 = go.Scatter(x=self.velocity, y=self.torque_values, mode='lines+markers', name='Actual Velocity')
        layout2 = go.Layout(title='Motor Velocity while braking', xaxis=dict(title='Velocity (rad/s)'), yaxis=dict(title='Torque (Nm)'))
        fig = go.Figure(data=[trace2], layout=layout2)
        pyo.plot(fig, filename=os.path.join(directory, 'velocity_plot.html'), auto_open=False)

        print(f"Data and plots saved in {directory}")


    def run_experiment(self):
        self.motor_controller.korad.setOutput(1)
        self.motor_controller.korad.setCurrent(0)
        now_vel = self.ramp_up()
        self.keep_vel(self.time_values, now_vel)
        self.break_motor(now_vel, self.time_values)
        self.motor_controller.candle.end()



# Example usage of the new MaxVelocityExperiment class
if __name__ == "__main__":
    # Define motor parameters
    (kp, kd, ki, ff) = (100.0, 5.0, 0.02, 0.0)
    motor_name = 69
    max_vel = 1.8

    # Initialize the motor controller
    motor_controller = MotorController(kp, kd, ki, ff, motor_name)

    # Setup power supply
    motor_controller.setup_power_supply()

    # Initialize the motor
    if motor_controller.initialize_drive():
        # Run the max velocity experiment
        max_velocity_experiment = MaxVelocityExperiment(motor_controller, max_vel)
        max_velocity_experiment.run_experiment()  # Run for 30 seconds
        max_velocity_experiment.save_and_plot_results()

    # Shutdown the motor controller
    motor_controller.shutdown()
