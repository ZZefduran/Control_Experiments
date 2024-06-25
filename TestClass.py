import time
import pyCandle
import math
import sys
from motor_power import tongui

class MotorController2:
    def __init__(self, voltage, baud_rate, control_mode, target_frequency, loop_duration, motor_name=None):
        self.supply = tongui()
        self.voltage = voltage
        self.candle = pyCandle.Candle(baud_rate, True)
        self.control_mode = control_mode
        self.target_frequency = target_frequency
        self.loop_duration = loop_duration
        self.motor_name = motor_name
        self.ids = []

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
        return True

    def move_motor_sine_wave(self):
        t = 0.0
        dt = self.target_frequency
        self.candle.begin()
        for md in self.candle.md80s:
            md.setMaxTorque(80)
        for i in range(self.loop_duration):
            t += dt
            position = math.sin(t) * 2.0
            self.candle.md80s[0].setTargetPosition(position)
            state = self.get_state()
            time.sleep(0.01)
        self.candle.end()

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

    def const_torque(self, tor):
        t = 0.0
        dt = self.target_frequency
        self.candle.begin()
        for md in self.candle.md80s:
            self.candle.controlMd80Mode(md.getId(), pyCandle.TORQUE_PID)
            md.setMaxTorque(80)
        for _ in range(self.loop_duration):
            t += dt
            for md in self.candle.md80s:
                md.setTargetTorque(tor)
            state = self.get_state()
            time.sleep(0.01)
        self.candle.end()
        print(f"Setting constant torque: {tor}")

    def const_vel(self, vel):
        t = 0.0
        dt = self.target_frequency
        ki = 0.02
        kp = 100.0
        kd = 5.0
        self.candle.begin()
        for md in self.candle.md80s:
            self.candle.controlMd80Mode(md.getId(), pyCandle.VELOCITY_PID)
            md.setMaxTorque(80)
            md.setVelocityControllerParams(kp, ki, kd, 0.0)
        for _ in range(self.loop_duration):
            t += dt
            for md in self.candle.md80s:
                md.setTargetVelocity(vel)
                state = self.get_state()
                time.sleep(0.01)
        self.candle.end()
        print(f"Setting constant velocity: {vel}")

    def const_pos(self, pos):
        t = 0.0
        dt = self.target_frequency
        ki = 0.02
        kp = 100.0
        kd = 5.0
        # self.candle.begin()
        for md in self.candle.md80s:
            self.candle.controlMd80Mode(md.getId(), pyCandle.POSITION_PID)
            md.setMaxTorque(80)
            md.setPositionControllerParams(kp, ki, kd, 0.0)
        self.candle.begin()
        for _ in range(self.loop_duration):
            t += dt
            for md in self.candle.md80s:
                md.setTargetPosition(pos)
                state = self.get_state()
                time.sleep(0.01)
        self.candle.end()
        print(f"Setting constant Position: {pos}")
    def shutdown(self):
        self.supply.setOutputOff()
        print("Shutdown completed successfully.")

    def run(self):
        self.setup_power_supply()
        if self.initialize_drives():
            self.move_motor_sine_wave()
        self.shutdown()

if __name__ == "__main__":
    voltage = 48
    baud_rate = pyCandle.CAN_BAUD_2M
    control_mode = pyCandle.IMPEDANCE
    target_frequency = 0.02
    loop_duration = 1000
    motor_name = 207

    motor_controller = MotorController2(voltage, baud_rate, control_mode, target_frequency, loop_duration, motor_name)
    
    
    
######### sin position test ###########
    # motor_controller.run()



######### const velocity test ###########
    # motor_controller.setup_power_supply()
    # time.sleep(0.5)
    # if motor_controller.initialize_drives():
    #     motor_controller.const_vel(2.0)  # Set a small velocity
    #     # motor_controller.const_pos(2.0)
    # # Shutdown the motor controller
    # motor_controller.shutdown()