import time
import pyCandle
from Futek import FutekClient
from ka3000_serial import ka3000
from motor_controller import MotorController


class SinExp:
    def __init__(self,motor_name):
        self.motor_name = motor_name
        self.voltage = 48.0
        self.baud_rate = pyCandle.CAN_BAUD_2M
        self.control_mode = pyCandle.IMPEDANCE  # Set to IMPEDANCE mode for torque control
        self.kp, self.kd, self.ki, self.ff = 100.0, 5.0, 0.0, 0.0
       # Initialize the motor controller
        self.motor_controller = MotorController(self.voltage, self.baud_rate, self.control_mode,
                                                 self.kp, self.kd, self.ki, self.ff, self.motor_name)
        # Initialize the Futek sensor
        futek_client = FutekClient()

        # Initializing korad & tonghui
        korad = ka3000()
        korad.setOutput(1)

        # Setup power supplies
        self.motor_controller.setup_power_supply()
        time.sleep(2)
        
        # Run and plot the experiment
        torque = 10
        while True:
            self.motor_controller.torque_cmd(torque)
            self.motor_controller.get_state()
        f=0
        ktau_experiment = KtauExperiment(motor_controller)
        ktau_experiment.run_and_plot_experiment(torque_list, futek_client)

        # Shutdown the motor controller
        self.motor_controller.shutdown()
        korad.setOutput(0)


sin_exp = SinExp(motor_name = 100)