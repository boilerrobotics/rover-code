#!/usr/bin/env python

import sys
import rospy
from gps_pose.srv import *

def gps_raw_data_client():
    rospy.wait_for_service('gps_raw_data')
    try:
        gps_raw_data = rospy.ServiceProxy('gps_raw_data', GpsPoseResponse)
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
    print('Altitute: {}'.format(gps_data.altitute))