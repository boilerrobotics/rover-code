import serial 
import time
import string
import pynmea2

def readLatLng(port, baudrate, timeout):
	coordinates = [] #list to hold latitude and longitude
	ser = serial.Serial(port, baudrate, timeout) #gets data at timeout rate
	dataout = pynmea2.NMEAStreamReader()
	newdata = ser.readline()

	if newdata[0:6] == "$GPRMC": #out of all the input you get, this filters relevant input
		newmsg = pynmea2.parse(newdata)
		lat = newmsg.latitude
		lng = newmsg.longitude
		coordinates = [lat, lng]
		gps = "Latitude = " + str(lat) + "and Longitude = " + str(lng)
		print(gps)

	return coordinates