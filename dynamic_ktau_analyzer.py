import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np
import ktau_analyzer


def plot_ktau_vs_load(ktaus,loads,fold,mot,cross_zero=True):
    plt.figure()
    plt.plot(loads,ktaus,'o')
    plt.grid()
    plt.ylabel('Ktau')
    plt.xlabel('Brake Current [A]')
    if cross_zero:
        title = fold +'/ktau_vs_load_cross_zero '+str(mot)+'.png'
    else:
        title = fold +'/ktau_vs_load_not_cross_zero '+str(mot)+'.png'
    plt.savefig(title)



def main_analyzer(mot,FOLD,ktau,gear,):
    
    for file in os.listdir(FOLD):
        if file.endswith('csv'):
            path = os.path.join(FOLD,file)
            data = pd.read_csv(path)
            unique_loads = np.unique(data.brake_current)
            all_ktaus_cross_zero = []
            all_ktaus_not_cross_zero = []
            for load in unique_loads:
                data_load = data[data.brake_current==load]
                reported,measured,current,iterations =  ktau_analyzer.analyze_from_data_frame(data_load)
                new_ktau_cross_zero = ktau_analyzer.plot_ktau(mot,reported,current,measured,iterations,FOLD,gear,ktau,load=load,cross_zero=True)
                new_ktau_not_cross_zero = ktau_analyzer.plot_ktau(mot,reported,current,measured,iterations,FOLD,gear,ktau,load=load,cross_zero=False)
                all_ktaus_cross_zero.append(new_ktau_cross_zero)
                all_ktaus_not_cross_zero.append(new_ktau_not_cross_zero)

    plot_ktau_vs_load(all_ktaus_cross_zero,unique_loads,FOLD,mot,cross_zero=True)
    plot_ktau_vs_load(all_ktaus_not_cross_zero,unique_loads,FOLD,mot,cross_zero=False)

# mot = 8064
# FOLD = '/home/zzefduran/Documents/mot8064/1686651552_exp_DYNAMIC_KTAU_torqueMin50Max60'
# ktau = 0.0860
# gear = 1/0.016

# main_analyzer(mot,FOLD,ktau,gear)