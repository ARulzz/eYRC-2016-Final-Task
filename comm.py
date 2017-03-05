import serial
import time

ser = serial.Serial("COM4", 9600)   # open serial port that Arduino is using
ser.write(b's')
time.sleep(1)
ser.write(b'e')
time.sleep(1)
ser.write(b'r')
time.sleep(1)
ser.write(b'i')
time.sleep(1)
ser.write(b'a')
time.sleep(1)
ser.write(b'l')
time.sleep(1)
ser.close()
