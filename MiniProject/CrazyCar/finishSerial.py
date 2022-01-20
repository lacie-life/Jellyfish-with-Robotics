import serial
import time

port = "/dev/ttyUSB0"

ser = serial.Serial(port, baudrate = 9600)

print ("starting")

while True:
	string1 = raw_input("Enter : ")
	string1_encode = string1.encode()
	ser.write(string1_encode)
	read_ser = ser.readline()
	print(read_ser)

