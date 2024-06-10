import pyvisa
import time
import numpy as np

class tongui(object):
    def __init__(self):
        # Function to scan available interfaces (GPIB, USB, Serial, Ethernet)
        rm = pyvisa.ResourceManager()
        # The instrument wants to connect with this power supply throw this port
        self.ps = rm.open_resource('ASRL/dev/ttyUSB0::INSTR') 

    def close(self):
        # closes the connection to the power supply
        self.ps.close()

    def setOutputOn(self):
        # turn the power supplier on output mode
        self.ps.write('OUTPut 1')
        time.sleep(3)

    def setOutputOff(self):
        # turn on the outputting mode of the power supplier
        self.ps.write('OUTPut 0')
        time.sleep(2.5)
    
    def setVolt(self, message):
        # changes the voltage to a desired voltage value
        self.ps.write('VOLT '+str(message))
        time.sleep(1.5)
    
    def getVolt(self):
        # gets the current voltage reading
        v = self.ps.query('MEAS:VOLT?')
        return v

    def setCurrent(self, message):
        # sets the supplier for a desired current
        self.ps.write('CURR '+str(message))
        time.sleep(1.5)
    
    def getCurr(self):
        
        c = self.ps.query('MEAS:CURR?')
        c = np.float16(c[:c.find('  /n')])


        # print(c)
        return c
    


# # # Connect to the power supply
# rm = pyvisa.ResourceManager()
# ps = rm.open_resource('ASRL/dev/ttyUSB0::INSTR') # Replace 'GPIB0::5::INSTR' with the actual address of your power supply
# # Set the voltage to 15V
# t=0
# while(t<48):
#     ps.write('VOLTage '+str(t))
#     time.sleep(1.5)
#     ps.write('OUTPut 1')
#     time.sleep(3)
#     curnent=ps.query('MEAS:VOLT?')
#     print("ask for: "+str(t)+" the actural current is: "+curnent)
#     ps.write('OUTPut 0')
#     time.sleep(2)
#     t+=1


# # # Close the connection
# ps.close()


# supply = tongui()
# supply.setOutputOn()
# supply.setVolt(48)
# time.sleep(10)
# supply.setOutputOff()





