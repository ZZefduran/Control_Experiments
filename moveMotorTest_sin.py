import pyCandle
from MotorClass_Amit import MotorController

if __name__ == "__main__":
    voltage = 48
    baud_rate = pyCandle.CAN_BAUD_2M
    control_mode = pyCandle.IMPEDANCE
    target_frequency = 0.02
    loop_duration = 1000
    motor_name = 207

    # Initialize the motor controller
    motor_controller = MotorController(voltage, baud_rate, control_mode, target_frequency, loop_duration, motor_name)
    motor_controller.run()