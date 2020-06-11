#!/usr/bin/env python

import rospy
from gps_pose.srv import GetGpsPose, GetGpsPoseResponse
from gps_pose.msg import GpsPose
#from GpsPose import GpsRawData

def handle_requests(request):
    #gps = GpsRawData()
    #gps.update()
    gps_data = GpsPose()
    gps_data.latitude = '40.464293'
    gps_data.longitude = '-86.9476627'
    gps_data.altitude = 2.0
    return GetGpsPoseResponse(gps_data)

def gps_raw_data_server():
    rospy.init_node('gps_raw_data_server')
    s = rospy.Service('gps_raw_data', GetGpsPose, handle_requests)
    rospy.spin()

if __name__ == "__main__":
    gps_raw_data_server()
