import serial 
import time
import string
import pynmea2

class gps:
	
	__latitude
	__longitude
	__altitude

	def __init__():
		self.__latitude = 0
		self.__longitude = 0
		self._altitude = 0

	def get_latitude() {
		return self.__latitude
	}

	def get_longitude() {
		return self.__longitude
	}

	def get_altitude() {
		return self.__altitude
	}

	def set_latitude_longitude():
	while True:
	port = "/dev/ttyAMA0"
	ser = serial.Serial(port, baudrate=9600, timeout=0.5)
	dataout = pynmea2.NMEAStreamReader()
	newdata = ser.readline()

	if newdata[0:6] == "$GPRMC":
		newmsg = pynmea2.parse(newdata)
		self.latitude = newmsg.latitude
		self.longitude = newmsg.longitude