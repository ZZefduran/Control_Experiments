import serial
import time
import numpy as np


class koradUdpComm(object):

    def __init__(self):
        # establishes a communication link between the program and the hardware device
        self.port = serial.Serial("/dev/ttyACM0",baudrate = 9600,timeout=1)
        f=0

    def close(self):
        #closes the serial port
        self.port.close()

    def udpSendRecv(self, message):

        # build the message
        messageb = bytearray()
        messageb.extend(map(ord, message))
        messageb.append(0x0a)

        startTime = time.time()
        while 1:
            sent = self.port.write(messageb)
            data = self.port.read(1024)
            if len(data) > 0:
                return data.decode('utf-8')
            
            if time.time() - startTime > 3:
                print ("UDP timeout")
                return " "

    def udpSend(self, message):
        # build the message
        messageb = bytearray()
        messageb.extend(map(ord, message))
        messageb.append(0x0a)

        sent = self.port.write(messageb)

class ka3000(object):

    def __init__(self):
        self.device = koradUdpComm()
        # self.setCurrent(0.0)


    def deviceInfo(self):
        # askes for the device identification string
        return self.device.udpSendRecv('*IDN?')
    
    def checkDevice(self):
        # checks if the connected device is the expected
        if 'KEL103' in self.deviceInfo():
            return True
        else:
            return False

    def measureVolt(self):
        # measure the current voltage 
        s = self.device.udpSendRecv('VSET1?')
        return float(s.strip('V\n'))
    
    def measureSetVolt(self):
        # measure the current voltage 
        s = self.device.udpSendRecv('VSET1?')
        return float(s.strip('V\n'))

    def setVolt(self, voltage):
        # askes from the power supplier to get to a desired voltage
        s = self.device.udpSend('VSET1:'+str(voltage))
        if self.measureSetVolt() != voltage:
            raise ValueError('Voltage set incorectly on the device')

    def measureCurrent(self):
        # measure the actual current 
        s = self.device.udpSendRecv('ISET1?')
        return float(s.strip('A\n'))

    def measureSetCurrent(self):
        # measure the current that has been set on the power supply
        s = self.device.udpSendRecv('ISET1?')
        if s == ' ':
            return 0.0
        else:
            return float(s.strip('A\n'))
        
    def setCurrent(self, current):
        # Gives the supplier a desired current to be in
        current = np.round(current,4)
        s = self.device.udpSend('ISET1:'+ str(current))
        print('Setting Brake Current: ',current, '[A]')
        # validation that the current current is the right one
        if self.measureSetCurrent() != current:
            raise ValueError('Current set incorectly on the device')

    def checkOutput(self):
        #checks if the output is on or off
        s = self.device.udpSendRecv(':INP?')
        if 'OFF' in s:
            return False
        if 'ON' in s:
            return True

    def setOutputTest(self, state):
        # Turns the output on or off
        if state == True:
            self.device.udpSend('OUT0')
          
        if state == False:
            self.device.udpSend('OUT1')
          
    def setOutput(self, state):
        if state == True:
            self.device.udpSend('OUT1')
            
        if state == False:
            self.device.udpSend('OUT0')
           

    def endComm(self):
        # colsing the communication
        self.device.close()


    # methuds for starting and ending the experiment
    def start_exp(self):
        self.setOutput(True) 

    def end_exp(self):
        self.setOutput(False)

