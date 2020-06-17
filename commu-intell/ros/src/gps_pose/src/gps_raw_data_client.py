#!/usr/bin/env python

"""
===============================================================================
Program Description 
	This is GPS raw data cliient script.

Author:         Maddy, Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        June 16, 2020
Status:         In progress
===============================================================================
"""

import sys
import rospy
from gps_pose.srv import GetGpsPose

def gps_raw_data_client():
    rospy.wait_for_service('gps_raw_data')
    try:
        gps_raw_data = rospy.ServiceProxy('gps_raw_data', GetGpsPose)
        resp = gps_raw_data()
        return resp.gps
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) != 1:
        print usage()
        sys.exit(1)
    print "Requesting coordinates"
    gps_data = gps_raw_data_client()
    print('Latitude: {}'.format(gps_data.latitude))
    print('Longitude: {}'.format(gps_data.longitude))
    print('Last Update: {}'.format(gps_data.last_update))
    print('Ground Speed: {}'.format(gps_data.raw_speed))