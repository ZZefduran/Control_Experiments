
import matplotlib.pyplot as plt
import numpy as np
import time
import os
import pandas as pd
from plotter.plotter import Plotter
from plotter.plot_data import PlotData
from plotter.plot_format import PlotFormat

class ExpPlotter():
    def __init__(self,
                 mode,
                 expfolder,
                 torque_meter_on,
                 supply_control

               ) -> None:
        
        self.mode = mode
        self.expfolder = expfolder
        self.torque_meter_on = torque_meter_on
        self.supply_control = supply_control
    
    def plot_normalized(self,data,expfolder):
        for column in data.columns:
            if column!='time':
                data[column] = (data[column]-min(data[column]))/(max(data[column])-min(data[column]))
        self.plot_hist(data,expfolder,gear_ratio=1,kind='Normalized')

    def load_data(self,path):
        return(pd.read_csv(path))


    def plot_hist(self,data,expfolder,gear_ratio = 64,kind = 'raw_data',zero=0):
        # plt.close('all')
        self.expfolder = expfolder
        isExist = os.path.exists(self.expfolder)
        if not isExist:
            os.makedirs(self.expfolder)
        plotter = Plotter(expfolder)

        # plt.figure()
        if self.mode =='torque':
            # plt.plot(data.time, data.desired_torque, label = "desired_torque")
            plotter.add_data(PlotData("desired_torque", x=data.time, y=data.desired_torque, subplot=(1, 1)))
        if self.mode =='velocity':
            # plt.plot(data.time, data.desired_velocity, label = "desired_velocity")
            plotter.add_data(PlotData("desired_velocity", x=data.time, y=data.desired_velocity, subplot=(1, 1)))
        if self.mode =='position':
            # plt.plot(data.time, data.desired_position, label = "desired_position")
            plotter.add_data(PlotData("desired_position", x=data.time, y=data.desired_position, subplot=(1, 1)))

        if self.supply_control:
        #    plt.plot(data.time, data.voltages_list, label = "voltage")
        #    plt.plot(data.time, data.supply_current, label = "supply_current")
           plotter.add_data(PlotData("supply_current", x=data.time, y=data.supply_current, subplot=(1, 1)))

        # plt.plot(data.time, data.velocity*gear_ratio, label = "vel")
        # plt.plot(data.time, data.torque, label = "torque")
        plotter.add_data(PlotData("reported_torque", x=data.time, y=data.torque, subplot=(1, 1)))
        # plt.plot(data.time,data.position, label = 'pose')
        plotter.add_data(PlotData("reported_pose", x=data.time, y=data.position, subplot=(1, 1)))
        # plt.plot(data.time,)
        # plt.plot(self.time,self.avragetemp, label = 'temp')
        if self.torque_meter_on:
            # plt.plot(data.time, data.actual_tor-zero, label = "actual_torque")
            plotter.add_data(PlotData("Futek", x=data.time, y=data.actual_tor-zero, subplot=(1, 1)))
        # plt.legend()
        # plt.grid()
        plotter.add_format(PlotFormat('time[s]', '', title=f'<b>Experiment</b>', subplot=(1, 1)))
        plotter.plot("Experiment history", save=True, shared_x=True, headnote=expfolder)
        name = str(np.round(time.time()))
        name = str(int(np.float64(name)))
        # plt.show()
        # plt.savefig(os.path.join(self.expfolder,name)+'_'+kind+'.png')
        # plt.close('all')

    def myplt(self,path,expfolder,kind):
        data = self.load_data(path)
        self.expfolder = expfolder
        isExist = os.path.exists(self.expfolder)
        if not isExist:
            os.makedirs(self.expfolder)
        plt.figure()
        # if self.mode =='torque':
        #     plt.plot(data.time, data.desired_torque, label = "desired_torque")
        # if self.mode =='velocity':
        #     plt.plot(data.time, data.desired_velocity, label = "desired_velocity")
        # if self.mode =='position':
        #     plt.plot(data.time, data.desired_position, label = "desired_position")

        # if self.supply_control:
        # #    plt.plot(data.time, data.voltages_list, label = "voltage")
        #    plt.plot(data.time, data.supply_current, label = "supply_current")

        # plt.plot(data.time, data.velocity*gear_ratio, label = "vel")
        # plt.plot(data.time, data.torque, label = "torque")
        plt.plot(data.time,data.position, label = 'pose')
        # plt.plot(self.time,self.avragetemp, label = 'temp')
        # if self.torque_meter_on:
        #     plt.plot(data.time, data.actual_tor, label = "actual_torque")
        
        # plt.plot(data.time, np.array(data.torque)/np.array(data.actual_tor), label = "ktau")
        plt.legend()
        plt.grid()
        name = str(np.round(time.time()))
        name = str(int(np.float64(name)))
        # plt.show()
        plt.savefig(os.path.join(self.expfolder,name)+'_'+kind+'.png')
        plt.close('all')


# exp_folder = '/home/zzefduran/Documents/mot7010/1685859699_exp_KTAU_torqueMin2Max14'
# path = os.path.join(exp_folder,'1685630341Total.csv')
# plotter = ExpPlotter(mode='torque',
#            expfolder=exp_folder,
#            torque_meter_on=True,
#            supply_control=True)
# plotter.myplt(path = path,expfolder=exp_folder,kind='myplot')


# exp_folder = '/home/zzefduran/Documents/mot8064_mentee/1690379913_exp_KTAU_torqueMin10Max60'
# for file in os.listdir(exp_folder):
#     if file.endswith('.csv'):
#         path = os.path.join(exp_folder,file)
# plotter = ExpPlotter(mode='torque',
#            expfolder=exp_folder,
#            torque_meter_on=True,
#            supply_control=False)
# # plotter.myplt(path = path,expfolder=exp_folder,kind='myplot')
# data = plotter.load_data(path)
# plotter.plot_hist(data=data,expfolder=exp_folder,gear_ratio=64,kind='hist',zero = 0)


