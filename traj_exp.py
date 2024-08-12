import time
import pyCandle
from Futek import FutekClient
from ka3000_serial import ka3000
from motor_controller import MotorController
import pandas as pd
import os
class CsvReaderExperiment:
    def __init__(self,motor_name,
                 csv_path,
                 freq,
                 output_dir):
        self.motor_name = motor_name
        self.voltage = 48.0
        self.baud_rate = pyCandle.CAN_BAUD_2M
        self.control_mode = pyCandle.IMPEDANCE  # Set to IMPEDANCE mode for torque control
        self.kp, self.kd, self.ki, self.ff = 100.0, 5.0, 0.0, 0.0
        self.dt = 1/freq
        self.output_dir = output_dir

       # Initialize the motor controller
        self.motor_controller = MotorController(self.voltage, self.baud_rate, self.control_mode,
                                                 self.kp, self.kd, self.ki, self.ff, self.motor_name)
        self.trajectory = pd.read_csv(csv_path)
        # Initialize the Futek sensor
        self.futek_client = FutekClient()

        # Initializing korad & tonghui
        self.korad = ka3000()
        self.all_data = {'start_time': [],
                         'end_time': [],

                         'target_pos': [],
                         'target_vel': [],
                         'target_torque': [],

                         'reported_torque': [],
                         'reported_pos': [],
                         'reported_vel': [],

                         'actual_torque': [],
                         }
        
        # korad.setOutput(1)

    def update_data(self):
        self.all_data['start_time'].append(self.timestamp_start)
        self.all_data['end_time'].append(self.timestamp_end)
        self.all_data['target_pos'].append(self.target_cmd['pos'])
        self.all_data['target_vel'].append(self.target_cmd['vel'])
        self.all_data['target_torque'].append(self.target_cmd['torque'])
        self.all_data['reported_pos'].append(self.actual_state['Position'])
        self.all_data['reported_vel'].append(self.actual_state['Velocity'])
        self.all_data['reported_torque'].append(self.actual_state['Torque'])
        self.all_data['actual_torque'].append(self.futek_state)

    def run_exp(self):
        self.korad.setCurrent(0.0)

        # Setup power supplies
        self.motor_controller.setup_power_supply()
        time.sleep(2)
        try:

            for index, row in self.trajectory.iterrows():
                self.timestamp_start = time.time()
                pos = row.get('motor_pos',0.0)
                vel = row.get('motor_vel',0.0)
                torque = row.get('motor_torque',0.0)
                print(f"Target_Position: {pos} Target_Velocity: {vel} Target_Torque: {torque}")
                self.motor_controller.torque_cmd(torque)
                self.motor_controller.vel_cmd(vel)
                self.motor_controller.pos_cmd(pos)
                self.target_cmd = {'pos': pos, 'vel': vel, 'torque': torque}
                self.get_and_update_data(pos, vel, torque)
                time.sleep(self.dt)
            
            # Shutdown the motor controller
            self.motor_controller.shutdown()
            self.korad.setOutput(0)
        finally:

            data = pd.DataFrame(self.all_data)
            time_stamp = time.strftime("%Y%m%d-%H%M%S")
            data.to_csv(os.path.join(self.output_dir, f'raw_data {time_stamp}.csv'), index=False)
            print(f"Data saved to {os.path.join(self.output_dir, f'raw_data {time_stamp}.csv')}")

    def get_and_update_data(self, pos, vel, torque):
        self.actual_state = self.motor_controller.get_state()
        self.futek_state = self.futek_client.get_torque()
        self.timestamp_end = time.time()

        self.update_data()

if __name__ == "__main__":
    csv_path = '/home/zzefduran/Documents/exp_csvs/csvs/new_Experiment.csv'

    csv_exp = CsvReaderExperiment(motor_name = 100,
                                csv_path = csv_path,
                                freq = 30)