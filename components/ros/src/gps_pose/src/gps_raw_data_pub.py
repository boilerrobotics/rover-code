#!/usr/bin/env python

"""
===============================================================================
Program Description 
	This program will publish raw GPS data once the data is sent from the sensor.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        June 16, 2020
Status:         In progress
===============================================================================
"""

import rospy
from gps_pose.msg import GpsPose
from GpsPose import GpsRawData

def gps_publisher():
    gps = GpsRawData()
    pub = rospy.Publisher('gps_raw', GpsPose, queue_size=10)
    rospy.init_node('gps_raw_pub', anonymous=True)
    while not rospy.is_shutdown():
        gps.update()
        gps_data = GpsPose()
        gps_data.latitude = gps.latitude
        gps_data.longitude = gps.longitude
        gps_data.last_update = gps.last_update
        gps_data.raw_speed = gps.raw_speed
        pub.publish(gps_data)

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
