#!/usr/bin/env python

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

import rospy
from gps_pose.srv import GetGpsPose, GetGpsPoseResponse
from gps_pose.msg import GpsPose
from GpsPose import GpsRawData

def handle_requests(request):
    gps = GpsRawData()
    gps.update()
    gps_data = GpsPose()
    gps_data.latitude = gps.latitude
    gps_data.longitude = gps.longitude
    gps_data.last_update = gps.last_update
    gps_data.raw_speed = gps.raw_speed
    print('Return GPS data')
    return GetGpsPoseResponse(gps_data)

def gps_raw_data_server():
    rospy.init_node('gps_raw_data_server')
    service = rospy.Service('gps_raw_data', GetGpsPose, handle_requests)
    print('The server started....')
    rospy.spin()

if __name__ == "__main__":
    gps_raw_data_server()
