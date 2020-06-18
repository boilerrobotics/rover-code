#!/usr/bin/env python

"""
===============================================================================
Program Description 
	This program will publish raw GPS data once the data is sent from the sensor.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        June 17, 2020
Status:         In progress
===============================================================================
"""

import rospy
from gps_pose.msg import GpsPose

def callback(gps):
    gps_data = GpsPose()
    gps_data.latitude = gps.latitude
    gps_data.longitude = gps.longitude
    gps_data.last_update = gps.last_update
    gps_data.raw_speed = gps.raw_speed
    
    print('Latitude: {}'.format(gps_data.latitude))
    print('Longitude: {}'.format(gps_data.longitude))
    print('Last Update: {}'.format(gps_data.last_update))
    print('Ground Speed: {}'.format(gps_data.raw_speed))
    print('---------------------------------------')

def listener():
    rospy.init_node('gps_raw_sub', anonymous=True)
    rospy.Subscriber('gps_raw', GpsPose, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()