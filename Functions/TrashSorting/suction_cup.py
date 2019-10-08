import serial
import time
import os,sys

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

class sucker():
    def __init__(self, num):
        baudRate = 9600
        serialPort = "/dev/ttyUSB"+str(num)
        self.ser = serial.Serial(serialPort, baudRate, timeout=0.5)
    def on(self, whichport):
        self.ser.write(str(whichport)+'1'+'\n')
    def off(self, whichport):
        self.ser.write(str(whichport)+'0'+'\n')
    def print_info(self):
        print(sucker.ser.name, sucker.ser.port, sucker.ser.baudrate,sucker.ser.bytesize ,sucker.ser.parity ,sucker.ser.stopbits,sucker.ser.isOpen())
if __name__ == '__main__':
    sucker = sucker(2)
    sucker.print_info()
    sucker.on(2)
    while True:
        a = sucker.ser.readline()
        print(a)
