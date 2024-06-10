from ka3000_serial import ka3000
import time
# from motor_power import tongui


# korad = ka3000()
# mot_supply = tongui()
# f=0

# for voltage in [26,25,24]:
#     korad.setVolt(voltage)
#     time.sleep(1)

# voltage = korad.measureVolt()
# print(f'the voltage is:{voltage}')

# current = korad.measureCurrent()
# print(f'the current is: {current}')

# for current in [0.2,0.3,0.4]:
#     korad.setCurrent(current)
#     time.sleep(1)
    
from motor_power import tongui
tongi = tongui()
for massage in [11,10,9]:
    tongi.setCurrent(massage)
    time.sleep(1)

# import pyvisa
# script for checking connected ports
# def list_visa_resources():
#     rm = pyvisa.ResourceManager()
#     resources = rm.list_resources()
#     return resources

# available_ports = list_visa_resources()
# print("Available VISA Resources:")
# for port in available_ports:
#     print(port)
