from ka3000_serial import ka3000
import time
# from motor_power import tongui


korad = ka3000()
# # mot_supply = tongui()
# # # f=0

# for voltage in [26,25,24]:
#     korad.setVolt(voltage)
#     time.sleep(1)

# voltage = korad.measureVolt()
# print(f'the voltage is:{voltage}')

# current = korad.measureCurrent()
# print(f'the current is: {current}')

# for current in [0.2,0.3,0.4]:
# current = 0.5
korad.setCurrent(2)
# time.sleep(1)
    

# hellowwww


# from motor_power import tongui
# supply = tongui()
# import time
# # Perform operations on the power supply
# supply.setOutputOn()
# supply.setVolt(45)
# time.sleep(3)
# supply.setOutputOff()
# # Close the connection
# supply.close()

# supply.setOutputOn()
# voltage = supply.getCurr()
# print(f'the voltage is:{voltage}')
# time.sleep(5)
# supply.setOutputOff()

# for current in [9,10,11, 12]:
#     supply.setCurrent(current)
#     time.sleep(2)

# supply.setCurrent(2.5)
# print(f'the current is:{3}')

# script for checking connected ports
# import pyvisa
# def list_visa_resources():
#     rm = pyvisa.ResourceManager()
#     resources = rm.list_resources()
#     return resources

# available_ports = list_visa_resources()
# print("Available VISA Resources:")
# for port in available_ports:
#     print(port)




# import serial

# try:
#     # Open the serial port
#     ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)  # Adjust baudrate and timeout as needed

#     # Do something with the serial port
#     ser.write(b'Hello, serial port!')

#     # Close the serial port when done
#     ser.close()

# except serial.SerialException as e:
#     print("Error opening serial port:", e)




