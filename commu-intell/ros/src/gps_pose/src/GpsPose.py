"""
===============================================================================
Program Description 
	This is GPS interface script.

Author:         Maddy, Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        June 13, 2020
Status:         In progress
===============================================================================
"""

import serial 
import pynmea2
from datetime import datetime
import pytz

class GpsRawData:
	
	def __init__(self):
		self.latitude = 0
		self.longitude = 0
		self.last_update = 0
		self.raw_speed = 0

	def get_latitude(self):
		return self.latitude
	
	def get_longitude(self):
		return self.longitude
	
	def update(self):
		update_flag = False
		while not update_flag:
			try:
				ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5.0)
				line = ser.readline().decode("utf-8")
        			if line[0:6] == "$GPRMC":
						msg = pynmea2.parse(line)
						update_flag = True
						timestamp = datetime(
							year= msg.datestamp.year,
							month= msg.datestamp.month,
							day=msg.datestamp.day,
							hour=msg.timestamp.hour,
							minute=msg.timestamp.minute,
							second=msg.timestamp.second,
							tzinfo=pytz.utc
						)
						self.latitude = str(msg.latitude)
						self.longitude = str(msg.longitude)
						self.last_update = timestamp.isoformat()
						self.raw_speed = msg.spd_over_grnd*0.514444

			except serial.SerialException as e:
				print('Device error: {}'.format(e))
				break
			except pynmea2.ParseError as e:
				print('Parse error: {}'.format(e))
				continue
