#!/usr/bin/env python

import rospy
from gps_raw_data.srv import GetGpsRaw, GetGpsRawResponse
from gps_raw_data.msg import Gps

def handle_gps_request(reg):
    gps_data = Gps()
    # gps1.set_latitude_longitude()
    print('Getting GPS data')
    # assume that we fix the Gps data to my apartment
    gps_data.latitude = '40.464293'
    gps_data.longitude = '-86.9476627'
    gps_data.altitute = 2.0
    return GetGpsRawResponse(gps_data)

def gps_pose_server_node():
    rospy.init_node('gps_pose_server')
    s = rospy.Service('get_gps_raw', GetGpsRaw, handle_gps_request)
    print('GPS raw data service is running')
    rospy.spin()

if __name__ == "__main__":
    gps_pose_server_node()