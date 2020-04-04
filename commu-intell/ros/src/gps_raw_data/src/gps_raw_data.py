import serial 
import time
import string
import pynmea2

def readLatLng():
	coordinates = []
	port = "/dev/ttyAMA0" #this port depends on the port you connect the gps to the Pi
	#you may have to change it accordingly
	ser = serial.Serial(port, baudrate = 9600, timeout = 0.5) #open serial port
	#reads every 0.5 seconds
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