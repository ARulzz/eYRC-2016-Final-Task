import serial
import time

movements = '1f0'

ser=serial.Serial("COM5",9600)
	# ser.write(b'*')
for i in movements:
	ser.write(i)
	time.sleep(1)
	while True:
		if ser.read() == '!':
			break		
	print i
ser.close()