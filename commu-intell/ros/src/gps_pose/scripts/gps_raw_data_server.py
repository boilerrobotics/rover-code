#!/usr/bin/env python

import rospy
from gps_pose.srv import GetGpsPose, GetGpsPoseResponse
from GpsPose import GpsPose

def handle_requests():
    gps = GpsPose()
    gps.update()
    return GetGpsPoseResponse(gps)

def gps_raw_data_server():
    rospy.init_node('gps_raw_data_server')
    s = rospy.Service('gps_raw_data', GetGpsPose, handle_requests)
    rospy.spin()

if __name__ == "__main__":
    gps_raw_data_server()
