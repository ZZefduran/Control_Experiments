import time
import pyCandle
from TestClass import MotorController2

if __name__ == "__main__":
    voltage = 48
    baud_rate = pyCandle.CAN_BAUD_2M
    control_mode = pyCandle.VELOCITY_PID  # Set to velocity control mode for testing
    target_frequency = 0.02
    loop_duration = 1000 
    motor_name = 207

    # Initialize the motor controller
    motor_controller = MotorController2(voltage, baud_rate, control_mode, target_frequency, loop_duration, motor_name)
    motor_controller.setup_power_supply()
    time.sleep(0.5)


### const
    motor_controller.initialize_drives()
    # motor_controller.const_vel(2.0)  # Set a small velocity
    # motor_controller.const_pos(7.0)
    # Shutdown the motor controller
    motor_controller.shutdown()
