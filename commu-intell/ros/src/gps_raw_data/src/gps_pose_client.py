#!/usr/bin/env python

import rospy
from gps_raw_data.srv import GetGpsRaw

def get_gps_raw_data():
    rospy.wait_for_service('get_gps_raw')
    try:
        get_gps_raw = rospy.ServiceProxy('get_gps_raw', GetGpsRaw)
        response = get_gps_raw()
        return response.gps
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print "Requesting to get GPS data"
    gps_data = get_gps_raw_data()
    print('Latitude: {}'.format(gps_data.latitude))
    print('Longitude: {}'.format(gps_data.longitude))
    print('Altitute: {}'.format(gps_data.altitute))
