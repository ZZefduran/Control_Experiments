from motor_power import tongui
import pyCandle
import time
import subprocess

class MotorController:
    def __init__(self, voltage, baud_rate, control_mode, 
                  kp, kd, ki, ff, 
                 motor_name):
        self.supply = tongui()
        self.voltage = voltage
        self.candle = pyCandle.Candle(baud_rate, True)
        self.control_mode = control_mode
        self.motor_name = motor_name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        self.ids = []
        self.gear_ratio, self.torque_constant = self.retrieve_motor_info(motor_name)
        self.initialize_drives()

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
        self.candle.begin()
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

    def Futek_zero(self):
        self.Futek.set_zero()

    def retrieve_motor_info(self, motor_id):
        # Run the mdtool command to get motor information
        result = subprocess.run(['mdtool', 'setup', 'info', str(motor_id)], capture_output=True, text=True)

        # Check if the command was successful
        if result.returncode != 0:
            print(f"Error running mdtool: {result.stderr}")
            return None, None

        # Process the output to extract the gear ratio and torque constant
        gear_ratio = None
        torque_constant = None
        for line in result.stdout.splitlines():
            if 'gear ratio' in line:
                gear_ratio = float(line.split(':')[1].strip())
            elif 'motor torque constant' in line:
                torque_constant = float(line.split(':')[1].strip().split()[0])  # Remove 'Nm/A'

        return gear_ratio, torque_constant

    
    def torque_cmd(self, torque):
        for md in self.candle.md80s:
            md.setTorque(torque)


    def pos_cmd(self, position):
        for md in self.candle.md80s:
            md.setTargetPosition(position)

    def vel_cmd(self, velocity):
        for md in self.candle.md80s:
            md.setTargetVelocity(velocity)

