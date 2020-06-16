"""
===============================================================================
Program Description 
	This is GPS raw data publisher node. It will publish the raw GPS data every 1 second.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        June 13, 2020
Status:         In progress
===============================================================================
"""

import rospy
import pynmea2
import serial
import math