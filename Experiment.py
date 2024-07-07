
# from kel103_serial import kel103
from ka3000_serial import ka3000
from TestClass import MotorController2 as Motor
# from temp import Temp
import os
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd
from motor_power import tongui
from Futek import FutekClient
import ktau_analyzer
import dynamic_ktau_analyzer
from omegaconf import OmegaConf
import velocity_torque_analyzer
from exp_plotter import ExpPlotter
import threading
import multiprocessing
import rclpy
# from motor_server.MyApp.joints_states_reader import MotorDataSubscriber

boot = True
MAINFOLDER = '/home/zzefduran/Documents/'


class ExperimentMaster():

    def __init__(self,
                 cfg_path) :
        
        self.cfg = OmegaConf.load(cfg_path)
        self.exp_type = self.cfg.exp_type
        self.mot_name = self.cfg.motor_name
        self.trajectory_type = self.cfg.trajectory_type
        self.torque_amplitude = self.cfg.torques_range.torques_end
        self.velocity_amplitude = self.cfg.velocity_range.vel_end
        self.iteration_time = self.cfg.iteration_time
        self.dt = self.cfg.dt
        self.motor_on = self.cfg.motor_on
        self.kp= 0
        self.kd = 0
        self.stop = False
        

        if self.exp_type=='KTAU':
            self.mode = 'torque'

        elif self.exp_type=='DYNAMIC_KTAU':
            self.mode = 'torque'
            # self.kd=5
            # self.kp =50
            self.brake_load_traj = np.arange(self.cfg.load_range.load_start, 
                                       self.cfg.load_range.load_end+self.cfg.load_range.load_step, 
                                       self.cfg.load_range.load_step)
        elif self.exp_type=='VEL_TORQUE':
            self.mode = 'velocity'
            self.voltages = np.arange(self.cfg.voltage_range.vol_start, 
                                        self.cfg.voltage_range.vol_end+self.cfg.voltage_range.vol_step, 
                                        self.cfg.voltage_range.vol_step)
            self.brake_load_traj = self.set_load_traj()
        
        elif self.exp_type=='WORK_RANGE':
            self.mode = 'torque'
            # self.mode = 'velocity'
            self.voltages = np.arange(self.cfg.voltage_range.vol_start, 
                                        self.cfg.voltage_range.vol_end+self.cfg.voltage_range.vol_step, 
                                        self.cfg.voltage_range.vol_step)
            self.brake_load_traj = self.set_load_traj()
            f=0
        else:
            print(self.exp_type,'is not a valid exp type')
            exit()

        
        self.korad_on = self.cfg.korad_on
        self.supply_control = self.cfg.supply.supply_control
        self.torque_meter_on = self.cfg.torque_meter_on
        self.torques_range = np.arange(self.cfg.torques_range.torques_start, 
                                       self.cfg.torques_range.torques_end+self.cfg.torques_range.torques_step, 
                                       self.cfg.torques_range.torques_step)
        self.velocity_range = np.arange(self.cfg.velocity_range.vel_start, 
                                        self.cfg.velocity_range.vel_end+self.cfg.velocity_range.vel_step, 
                                        self.cfg.velocity_range.vel_step)
        
        self.reset_data()
        self.t = 0.0
        self.dt = self.cfg.dt
        if self.mode=='torque':
            self.kp = 0
            self.kd = 0
        
        if self.mode=='velocity':
            self.kp = 0
            self.kd = 50


        if self.korad_on:
            if self.cfg.korad_type=='kel103':
                self.korad = kel103()
            if self.cfg.korad_type=='ka30000':
                self.korad = ka3000()

        self.motfolder = MAINFOLDER+'mot'+str(self.mot_name)
        isExist = os.path.exists(self.motfolder)
        if not isExist:
            os.makedirs(self.motfolder)
        if self.supply_control:
            self.set_voltage = self.cfg.supply.set_voltage
            self.supply = tongui()
            self.apply_voltage(self.set_voltage)
            self.current_vol = self.set_voltage
        if self.motor_on:
            self.motor  = Motor(voltage = self.set_voltage, 
                                motor_name=self.cfg.motor_name,
                                kp = 100,
                                kd = 5)
            
        if self.torque_meter_on:
            self.torque_meter = FutekClient()
        
        
        
    
    def apply_voltage(self,voltage):
        self.supply.setOutputOn()
        print('Setting supply voltage: ',voltage,'[V]')
        self.supply.setVolt(voltage)
        g=0

    def reset_data(self):
        self.tmp=[]
        self.motor_pos= []
        self.joint_pos = []
        self.joint_tor= []
        self.joint_vel= []
        self.time_list= []
        self.motor_vel = []
        self.motor_tor = []
        self.voltage = []
        self.actual_tor = []
        self.brake_load = []
        self.voltages_list = []
        self.iteration_list = []
        self.current_list = []


    def set_traj_const(self):
        steps = int(self.iteration_time/self.dt)
        traj = np.ones(int(steps))
        if self.mode=='torque':
            self.append_to_traj(traj*self.torque_amplitude)
        if self.mode=='velocity':
            self.append_to_traj(traj*self.velocity_amplitude)

    
    def append_to_traj(self,traj):
        if self.mode=='torque':
            self.motor.torque_trajectory = np.append(self.motor.torque_trajectory,traj)
        if self.mode=='velocity':
            self.motor.velocity_trajectory = np.append(self.motor.velocity_trajectory,traj)
    
    def set_load_traj(self):
        load_traj = []
        load_traj = np.append(load_traj,np.arange(self.cfg.load_range.load_start, 
                                   self.cfg.load_range.load_interest_start,
                                   self.cfg.load_range.load_step_large
                                   ))
        load_traj = np.append(load_traj,np.arange(self.cfg.load_range.load_interest_start, 
                                self.cfg.load_range.load_interest_end,
                                self.cfg.load_range.load_step_small
                                ))
        
        load_traj = np.append(load_traj,np.arange(self.cfg.load_range.load_interest_start_2, 
                                self.cfg.load_range.load_interest_end_2+self.cfg.load_range.load_step_small_2,
                                self.cfg.load_range.load_step_small_2
                                ))
        

        load_traj = np.append(load_traj,np.arange(self.cfg.load_range.load_interest_end, 
                                self.cfg.load_range.load_end,
                                self.cfg.load_range.load_step_large
                                ))
        return load_traj

    def get_len_traj(self):
        if self.mode=='torque':
            return len(self.motor.torque_trajectory)
        if self.mode=='velocity':
            return len(self.motor.velocity_trajectory)

    def get_last_traj_step(self):
        if self.mode=='torque':
            return self.motor.torque_trajectory[-1]
        if self.mode=='velocity':
            return self.motor.velocity_trajectory[-1]
        

    def set_traj_ramp(self,start_pose = 0):
            for val in self.range:
                traj = self.rampup_and_stay(val,start_pose)
                start_pose = val
                self.append_to_traj(traj)
    
    def ramp_for_each_torq(self,vals):
        self.rep_end = []
        for val in vals:
            rampandstay = self.rampup_and_stay(val,0)
            self.append_to_traj(rampandstay)
            down, zero_length= self.rampdown_and_stay(val,0)
            self.append_to_traj(down)
            self.rep_end.append(int(self.get_len_traj()-0.5*zero_length))        

    def only_ramp(self,start_pose,end_pose):
        steps = int(self.iteration_time/self.dt)
        traj = np.linspace(start_pose,end_pose,int(steps))
        self.append_to_traj(traj)
                
    def rampdown_and_stay(self,tor,end_pose):
        ramp_steps = int(self.cfg.ramp_time/self.dt)
        down = np.linspace(tor,end_pose,ramp_steps)
        end_pose_length = int(self.cfg.break_time/self.dt)
        down = np.append(down,end_pose*(np.ones(end_pose_length)))
        return down,end_pose_length
     
    def rampup_and_stay(self,tor,start_pose):
        ramp_steps = int(self.cfg.ramp_time/self.dt)
        tor_steps = int(self.cfg.peak_time/self.dt)
        up = np.linspace(start_pose,tor,ramp_steps)
        traj = np.append(up,tor*(np.ones(tor_steps)))
        return traj

    def set_traj_circle(self,start_pose = 0):
        
        for val in self.range:
            traj = self.rampup_and_stay(val,start_pose)
            start_pose = val
            self.append_to_traj(traj)
            
        
        for val in self.range[::-1][1:]: ## go back
            traj = self.rampup_and_stay(val,start_pose)
            start_pose = val
            self.append_to_traj(traj)  

    def set_traj_sine(self):
        steps = int(self.iteration_time/self.dt)
        x = np.linspace(-np.pi, np.pi, steps)
        self.motor.torque_trajectory = np.sin(x)*self.torque_amplitude

    def plot_traj(self):
        if self.mode=='torque':
            self.traj_to_plot = self.motor.torque_trajectory
        if self.mode=='velocity':
            self.traj_to_plot = self.motor.velocity_trajectory
        plt.figure()
        plt.plot(np.arange(len(self.traj_to_plot))*self.dt,self.traj_to_plot)
        plt.title('Exp '+self.mode+ ' trajectory')
        plt.xlabel('Time')
        plt.ylabel(self.mode)
        plt.grid()
        # plt.show(block=False)
        # plt.show()


    def set_trajectory(self):
        print('setting trajectory..')
        if self.mode=='torque':
            self.range = self.torques_range
            self.motor.torque_trajectory = []
        if self.mode=='velocity':
            self.range = self.velocity_range
            self.motor.velocity_trajectory = []

        start_pose = 0
        self.start_traj = 0
        self.end_traj = 0

        if self.cfg.slowrampstartend:
            if self.trajectory_type !='ramp_for_each_torq':
                slowramp = self.slowramp(start=start_pose,end=self.range[0],t=5)
                self.append_to_traj(slowramp)
                self.start_traj = len(slowramp)
                start_pose = self.range[0]
            
        if self.trajectory_type =='traj_all_ramp':
            self.only_ramp(start_pose=self.range[0],end_pose=self.range[-1])
        
        if self.trajectory_type =='ramp':
            self.set_traj_ramp(start_pose=start_pose)
        
        if self.trajectory_type =='sine':
            self.set_traj_sine()
        
        if self.trajectory_type =='circle':
            self.set_traj_circle(start_pose=start_pose)
        
        if self.trajectory_type =='const':
            self.set_traj_const()
        
        if self.trajectory_type =='ramp_for_each_torq':
            self.cfg.record_all = True
            self.ramp_for_each_torq(self.range)

        if self.cfg.slowrampstartend:
            last_step = self.get_last_traj_step()
            slowramp = self.slowramp(start=last_step,end=0,t=5)
            self.append_to_traj(slowramp)
            self.end_traj = self.get_len_traj()-len(slowramp)
        
        self.plot_traj()
        g=0
           
    def slowramp(self,start,end,t=0):
        steps = int(t/self.dt)
        step = (end-start)/steps
        if step!=0:
            slowramp = np.arange(start,end,step)
        else:
            slowramp = []

        return slowramp
    def get_rng_str(self,rng,typ = 'torque'):
        string = typ+'Min'+str(np.round(rng[0],1))+'Max'+str(np.round(rng[-1],1))
        
        return string

    def start_exp(self):

        self.expfolder = self.motfolder+'/'+str(int(time.time()))+'_exp_'+str(self.exp_type)
        self.exp_plotter = ExpPlotter(mode = self.mode,
                                      expfolder=self.expfolder,
                                      torque_meter_on = self.torque_meter_on,
                                      supply_control=self.supply_control)
        
        if self.korad_on:
            self.korad.start_exp()
        if self.exp_type=='KTAU':
            self.korad.setOutput(True)
            self.korad.setCurrent(self.cfg.load_range.load_start)
            self.current_brake = self.cfg.load_range.load_start
            self.expfolder = self.expfolder+'_'+str(self.get_rng_str(self.torques_range,typ = 'torque'))
        if self.exp_type=='DYNAMIC_KTAU':
            self.korad.setOutput(True)
            # print("cool down")
            # time.sleep(60)
            self.expfolder = self.expfolder+'_'+str(self.get_rng_str(self.torques_range,typ = 'torque'))
        if self.exp_type=='VEL_TORQUE':
            self.korad.setOutput(True)
            self.expfolder = self.expfolder+'_'+str(self.get_rng_str(self.voltages,typ = 'voltage'),)
            self.expfolder = self.expfolder+'_'+str(self.get_rng_str(self.brake_load_traj,typ = 'brake'))

            
    def end_exp(self):
        if self.motor_on:
            self.motor.end_exp()
            # self.node_thread.join()
            # self.motorsubscriber.destroy_node()

        if self.korad_on:
            self.korad.end_exp()
        # if self.supply_control:
            # self.supply.setOutputOff()
        
        # self.exp_thread.join()
        
        

    def check_motor(self):
        res = self.motor.motor_error()
        if res:
            self.stop=True

    def save_step_data(self,i):
        pos, vel, tor = self.motor_pos, self.motor_vel, self.motor_tor
        if pos is not None:
            if vel is not None:
                if tor is not None:
                    
                    self.joint_pos.append(pos)  # Motor
                    self.joint_tor.append(tor)
                    self.joint_vel.append(vel)
                    self.motor_tor.append(self.move_tor) # Target
                    self.motor_vel.append(self.move_vel)
                    self.motor_pos.append(self.move_pos)
                    self.time_list.append(self.t)
                    # print('state:',pos, vel, tor)
                    if self.supply_control:
                        cur = self.supply.getCurr()
                        self.current_list.append(cur)
                    
                    if self.torque_meter_on:
                        act_torque = self.torque_meter.get_torque()
                        self.actual_tor.append(act_torque)
                    if ((i%10)==0):
                        print('Desired Position: ',np.round(self.move_pos,2),' Velocity: ',np.round(self.move_vel,2),' Torque: ',np.round(self.move_tor,2))
                        print('Motor Position: ',np.round(pos,2),' Velocity: ',np.round(vel,2),' Torque: ',np.round(tor,2))
                        if self.torque_meter_on:
                            print('Actual Torque: ',np.round(act_torque,2))
                        print('Expected Current: ',np.round(((tor*self.cfg.gear_ratio)/(self.cfg.ktau)),2))
                    
                    self.brake_load.append(self.current_brake)
                    if self.supply_control:
                        self.voltages_list.append(self.current_vol)
                    
                    self.iteration_list.append(self.current_iter)


    def run_trajectory(self):
        print('Running experiment...')
        exp_started = False
        # self.check_motor()
        if not self.stop:

            for i in range(self.get_len_traj()): # range(len(self.motor.torque_trajectory)):
                if i in self.rep_end:
                    self.torque_meter.set_zero()
                self.move_pos,self.move_vel,self.move_tor = self.motor.run_new_step(i) 
                if self.cfg.record_all:
                    self.save_step_data(i)
                    self.t = self.t + self.dt
                    time.sleep(self.dt)
                else:
                    if i>self.start_traj:
                        if i<self.end_traj:
                            if not exp_started:
                                print('Exp_Started')
                                exp_started = True
                            self.save_step_data(i)
                            self.t = self.t + self.dt
                            time.sleep(self.dt)
                
            
            self.motor.const_torque(0.0)
            self.motor.const_vel(0.0)
            self.motor.const_pos(0.0)
        else:
            self.stop=True
            print('Trajectory not done due to motor Error')
            exit()
    
        

        
    def plot_output(self):
        self.save_data(save_csv=False)
        self.exp_plotter.plot_hist(self.data,self.expfolder,gear_ratio=self.cfg.gear_ratio)
        self.exp_plotter.plot_normalized(self.data,self.expfolder)
        


    def save_data(self,name='',save_csv=True):
        
        isExist = os.path.exists(self.expfolder)
        if not isExist:
            os.makedirs(self.expfolder)
        
        data = {'time': self.time_list,
                'position': self.joint_pos,
                'velocity': self.joint_vel,
                'torque': self.joint_tor, 
                'desired_position': self.motor_pos,
                'desired_velocity': self.motor_vel,
                'desired_torque': self.motor_tor,
                'brake_current': self.brake_load,
                'iteration': self.iteration_list,
                } 
                # 'temp' :  self.avragetemp}
        if self.supply_control:
            data['supply_current'] = self.current_list
        name = str(int(time.time()))+name

        if self.exp_type=='RMS':
            time.sleep(0.1)
            vol = input("Rms Voltage [V]: ")
            self.voltage = [vol]*len(self.motor_tor)
            data['voltage'] =  self.voltage

        if self.exp_type=='VEL_TORQUE':
            data['voltage'] =  self.voltages_list

        if self.exp_type=='WORK_RANGE':
            data['voltage'] =  self.voltages_list

        if self.torque_meter_on:
            data['actual_tor'] =  self.actual_tor

        
        path = self.expfolder+'/'+name+'.csv'
        data = pd.DataFrame(data)
        if save_csv:
            data.to_csv(path)

        self.data = data
        return data
    

    def analyze_exp(self):
        if self.exp_type=='KTAU':
                self.analyze_ktau_exp()
        if self.exp_type=='VEL_TORQUE':
                self.analyze_vel_tor_exp()
        if self.exp_type=='WORK_RANGE':
                self.analyze_vel_tor_exp()
        if self.exp_type=='DYNAMIC_KTAU':
                self.analyze_dynamic_ktau_exp()


    def exp_process(self):
        self.reset_data()
        for i in range(self.cfg.itertate.exp_iterations):
            print('Iteration ',i+1,'/',self.cfg.itertate.exp_iterations)
            self.current_iter = i+1
            self.start_exp()
            print('Starting Exp: ', str(self.exp_type), ' for motor '+str(self.mot_name) )
            if self.exp_type=='KTAU':
                self.run_ktau_exp()
            
            if self.exp_type=='DYNAMIC_KTAU':
                self.run_dynamic_ktau_exp()
            
            if self.exp_type=='VEL_TORQUE':
                self.run_vel_torque_exp()

            if self.exp_type=='WORK_RANGE':
                self.run_work_range_exp()

            if not self.cfg.itertate.concat:
                
                path = self.save_data()
                self.plot_output()
                self.analyze_exp()
                self.reset_data()

            time.sleep(2)
        
        self.end_exp()
        if self.cfg.itertate.concat:
            self.plot_output()
            self.save_data(name='Total')
        
        self.analyze_exp()

    def spin_node(self,node):
        
        rclpy.spin(node)



    def run_exp(self):
        
        self.exp_process()


    def analyze_ktau_exp(self):
        ktau_analyzer.main_analyzer(self.mot_name,
                                    self.expfolder,
                                    self.cfg.ktau,
                                    1/self.cfg.gear_ratio,
                                    )
    def analyze_dynamic_ktau_exp(self):
        
        dynamic_ktau_analyzer.main_analyzer(self.mot_name,
                                    self.expfolder,
                                    self.cfg.ktau,
                                    1/self.cfg.gear_ratio)
    def analyze_vel_tor_exp(self):
        velocity_torque_analyzer.main_analyzer(self.mot_name,
                                               self.expfolder,
                                               self.cfg.ktau,
                                    1/self.cfg.gear_ratio)

    def run_ktau_exp(self):
       
        
        
        self.set_trajectory()
        self.run_trajectory()
        
    def run_dynamic_ktau_exp(self):
       
        self.set_trajectory()
        for load in self.brake_load_traj:
            self.korad.setCurrent(load)
            self.current_brake = load
            time.sleep(0.5)
            self.run_trajectory()

    def run_work_range_exp(self):
        i=0 
        self.set_trajectory()
        for volt in self.voltages:
            if i>1:
                print ("Coolling motor: 10 sec") 
                time.sleep(10)
            time.sleep(2)
            self.apply_voltage(volt)
            self.current_vol = volt
            i=i+1
            for load in self.brake_load_traj:
                self.korad.setCurrent(load)
                self.current_brake = load
                time.sleep(2.0)
                self.run_trajectory()
        

    def run_vel_torque_exp(self):

        self.set_trajectory()
        for volt in self.voltages:
            time.sleep(15)
            self.apply_voltage(volt)
            self.current_vol = volt
            for load in self.brake_load_traj:
                self.korad.setCurrent(load)
                self.current_brake = load
                time.sleep(2.0)
                self.run_trajectory()


    def run_rms_exp(self):
        reps = 1
        self.set_trajectory()
        voltage = 15
        print('next voltage: ',voltage, ' [V]')
        for i in range(reps):
            self.reset_data()
            self.start_exp()
            self.run_trajectory()
            voltage = 24-1*(i+1)
            print('next voltage: ',voltage, ' [V]')
            self.save_data()
            self.plot_output()



