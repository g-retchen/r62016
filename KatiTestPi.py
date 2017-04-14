import serial
from time import sleep

ser = serial.Serial('/dev/ttyACM0',9600)
sleep(3)

while True:
    ser.write('d')
    ser.write('4')
    #ser.write('4')
    #ser.write('p')
    ser.write('\n')
    sleep(.3)
