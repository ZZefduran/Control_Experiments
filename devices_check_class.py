import time
import pyCandle
import numpy as np
import math
from motor_power import tongui
from Futek import FutekClient

class MotorController:
    def __init__(self, voltage, baud_rate, control_mode, 
                 target_frequency, loop_duration, kp, kd, ki, ff, 
                 motor_name):
        self.supply = tongui()
        self.Futek = FutekClient()
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

    def Futek_zero(self):
        self.Futek.set_zero()

