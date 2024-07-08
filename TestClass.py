import time
import pyCandle
import math
import sys
from motor_power import tongui

class MotorController:
    def __init__(self, voltage, baud_rate, control_mode, target_frequency, loop_duration, kp, kd, ki, ff, motor_name):
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
                md.setMaxTorque(100)
                md.setImpedanceControllerParams(self.kp, self.kd)
        elif self.control_mode == pyCandle.VELOCITY_PID:
            print('Control mode is velocity')
            for md in self.candle.md80s:
                self.candle.controlMd80Mode(md.getId(), self.control_mode)
                md.setMaxTorque(100)
                md.setVelocityControllerParams(self.kp, self.ki, self.kd, self.ff)
        elif self.control_mode == pyCandle.POSITION_PID:
            print('Control mode is position')
            for md in self.candle.md80s:
                self.candle.controlMd80Mode(md.getId(), self.control_mode)
                md.setMaxTorque(100)
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
        self.candle.begin()
        for md in self.candle.md80s:
            while True:
                md.setTorque(tor)
                state = self.get_state()
                time.sleep(0.1)
        self.candle.end()
        print(f"Setting constant torque: {tor}")

    def const_vel(self, vel):
        self.candle.begin()
        for md in self.candle.md80s:
            while True:
                md.setTargetVelocity(vel)
                state = self.get_state()
                time.sleep(0.01)
        self.candle.end()
        print(f"Setting constant velocity: {vel}")

    def const_pos(self, pos):
        self.candle.begin()
        for md in self.candle.md80s:
            while True:
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


    def Ktau_experiment(self, torque_list, futek_client):
        self.candle.begin()
        motor_torques = []
        futek_torques = []
        desired_torques = []
        time_values = []  # List to store time values
        currents_for_Ktau = []
        Torques_for_Ktau = []
        futek_for_Ktau = []
        current_torque_const = 0.0859
        GR = 64
        I = 0
        
        dt = 0.01  # Time step in seconds (10 milliseconds)
        t = 0  # Initialize time counter

        for tor in torque_list:
            # Ramp up to the desired torque over 2 seconds
            time.sleep(1)
            start_time = time.time()
            while time.time() - start_time < 2:
                for md in self.candle.md80s:
                    current_time = time.time() - start_time
                    ramp_torque = tor * (current_time / 2)  # Linearly increase torque
                    md.setTorque(ramp_torque)
                    
                motor_torque = self.candle.md80s[0].getTorque()
                motor_current = self.supply.getCurr()
                futek_torque = futek_client.get_torque()
                
                motor_torques.append(motor_torque)
                futek_torques.append(futek_torque)
                desired_torques.append(ramp_torque)
                time_values.append(t)  # Record the current time
                
                print(f"Ramping Up - Desired Torque: {ramp_torque} | Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")
                
                time.sleep(dt)  # Wait for 10 milliseconds between measurements
                t += dt  # Increment the time counter

            # Hold the desired torque for 3 seconds
            start_time = time.time()
            while time.time() - start_time < 3:
                for md in self.candle.md80s:
                    md.setTorque(tor)
                    
                motor_torque = self.candle.md80s[0].getTorque()
                motor_current = self.supply.getCurr()
                futek_torque = futek_client.get_torque()
                
                motor_torques.append(motor_torque)
                futek_torques.append(futek_torque)
                desired_torques.append(tor)
                time_values.append(t)  # Record the current time
                
                print(f"Holding - Desired Torque: {tor} | Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")
                
                time.sleep(dt)  # Wait for 10 milliseconds between measurements
                t += dt  # Increment the time counter

                if time.time() - start_time >= 1.47 and time.time() - start_time <= 1.53:
                    I = motor_torque/(GR*current_torque_const)  
                    currents_for_Ktau.append(I)  
                    Torques_for_Ktau.append(motor_torque)
                    futek_for_Ktau.append(futek_torque)


            # Ramp down to zero torque over 2 seconds
            start_time = time.time()
            while time.time() - start_time < 2:
                for md in self.candle.md80s:
                    current_time = time.time() - start_time
                    ramp_torque = tor * (1 - (current_time / 2))  # Linearly decrease torque
                    md.setTorque(ramp_torque)
                    
                motor_torque = self.candle.md80s[0].getTorque()
                motor_current = self.supply.getCurr()
                futek_torque = futek_client.get_torque()
                
                motor_torques.append(motor_torque)
                futek_torques.append(futek_torque)
                desired_torques.append(ramp_torque)
                time_values.append(t)  # Record the current time
                
                print(f"Ramping Down - Desired Torque: {ramp_torque} | Motor Torque: {motor_torque} | Motor Current: {motor_current} | Futek Torque: {futek_torque}")
                
                time.sleep(dt)  # Wait for 10 milliseconds between measurements
                t += dt  # Increment the time counter
                    

        self.candle.end()
        return  motor_torques, futek_torques, desired_torques, time_values, currents_for_Ktau, Torques_for_Ktau, futek_for_Ktau





# if __name__ == "__main__":
#     voltage = 48
#     baud_rate = pyCandle.CAN_BAUD_2M
#     control_mode = pyCandle.IMPEDANCE
#     target_frequency = 0.02
#     loop_duration = 1000
#     (kp, kd, ki, ff) = (100.0, 5.0, 0.02, 0.0)
#     motor_name = 207

#     motor_controller = MotorController(voltage, baud_rate, control_mode, target_frequency, loop_duration, kp, kd, ki, ff, motor_name)
    
    ######### sin position test ###########
    # motor_controller.run()

    ######## const velocity test ###########
    # motor_controller.setup_power_supply()
    # time.sleep(0.5)
    # if motor_controller.initialize_drives():
    #     # motor_controller.const_vel(2.0)
    #     motor_controller.const_torque(20)  # Set a small velocity
    #     # motor_controller.const_pos(2.0)
    # # Shutdown the motor controller
    # motor_controller.shutdown()


    # ff
