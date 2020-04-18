import serial 
import time
import string
import pynmea2

class GpsRawData:
	
	_latitude
	_longitude
	_altitude

	def __init__():
		self._latitude = 0
		self._longitude = 0
		self._altitude = 0

	def _get_latitude():
		return self._latitude
	

	def _get_longitude():
		return self._longitude
	

	def _get_altitude():
		return self._altitude
	

	def update():
		count = 0
		while (count < 2):
			port = "/dev/ttyAMA0"
			ser = serial.Serial(port, baudrate=9600, timeout=0.5)
			dataout = pynmea2.NMEAStreamReader()
			newdata = ser.readline()

			if newdata[0:6] == "$GPRMC":
				newmsg = pynmea2.parse(newdata)
				self.latitude = newmsg.latitude
				self.longitude = newmsg.longitude
				count = count + 1