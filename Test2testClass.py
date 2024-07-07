import time
import pyCandle
from TestClass import MotorController

if __name__ == "__main__":
    voltage = 48
    baud_rate = pyCandle.CAN_BAUD_2M
    control_mode = pyCandle.IMPEDANCE  # Set to velocity control mode for testing
    target_frequency = 0.02
    loop_duration = 10000 
    (kp, kd, ki, ff) = (50.0, 5.0, 0.02, 0.0)
    motor_name = 207

    # Initialize the motor controller
    motor_controller = MotorController(voltage, baud_rate,
                                        control_mode, target_frequency, 
                                        loop_duration, kp, kd, ki, 
                                        ff, motor_name)
    motor_controller.setup_power_supply()
    time.sleep(0.5)


### const
    motor_controller.initialize_drives()
    # motor_controller.const_vel(150)  # Set a small velocity
    # motor_controller.const_pos(8)
    motor_controller.const_torque(40)
    # Shutdown the motor controller
    motor_controller.shutdown()
