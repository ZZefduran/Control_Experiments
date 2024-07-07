import serial
import time
import numpy as np

class koradUdpComm(object):

    def __init__(self):
        # Try to connect to /dev/ttyACM0
        try:
            self.port = serial.Serial("/dev/serial/by-id/usb-Nuvoton_USB_Virtual_COM_001826960458-if00", baudrate=9600, timeout=1)
            print("Connected to /dev/ttyACM1")
        except serial.SerialException as e:
            raise RuntimeError(f"Could not connect to /dev/ttyACM1: {e}")

    def close(self):
        # Close the serial port
        if self.port:
            self.port.close()

    def udpSendRecv(self, message):
        messageb = bytearray(map(ord, message))
        messageb.append(0x0a)

        startTime = time.time()
        while True:
            self.port.write(messageb)
            data = self.port.read(1024)
            if data:
                return data.decode('utf-8')
            if time.time() - startTime > 3:
                print("UDP timeout")
                return " "

    def udpSend(self, message):
        messageb = bytearray(map(ord, message))
        messageb.append(0x0a)
        self.port.write(messageb)

class ka3000(object):

    def __init__(self):
        self.device = koradUdpComm()

    def deviceInfo(self):
        return self.device.udpSendRecv('*IDN?')

    def checkDevice(self):
        return 'KEL103' in self.deviceInfo()

    def measureVolt(self):
        s = self.device.udpSendRecv('VSET1?')
        return float(s.strip('V\n'))

    def measureSetVolt(self):
        s = self.device.udpSendRecv('VSET1?')
        return float(s.strip('V\n'))

    def setVolt(self, voltage):
        self.device.udpSend(f'VSET1:{voltage}')
        if self.measureSetVolt() != voltage:
            raise ValueError('Voltage set incorrectly on the device')

    def measureCurrent(self):
        s = self.device.udpSendRecv('ISET1?')
        return float(s.strip('A\n'))

    def measureSetCurrent(self):
        s = self.device.udpSendRecv('ISET1?')
        return float(s.strip('A\n')) if s.strip() else 0.0

    def setCurrent(self, current):
        current = np.round(current, 4)
        self.device.udpSend(f'ISET1:{current}')
        if self.measureSetCurrent() != current:
            raise ValueError('Current set incorrectly on the device')

    def checkOutput(self):
        s = self.device.udpSendRecv(':INP?')
        return 'ON' in s

    def setOutput(self, state):
        command = 'OUT1' if state else 'OUT0'
        self.device.udpSend(command)

    def endComm(self):
        self.device.close()

    def start_exp(self):
        self.setOutput(True)

    def end_exp(self):
        self.setOutput(False)

# if __name__ == "__main__":
#     korad = ka3000()
#     for voltage in [26, 25, 22]:
#         korad.setOutput(0)
#         korad.setVolt(voltage)
# #         time.sleep(1)
