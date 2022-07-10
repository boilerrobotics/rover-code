#!/usr/bin/env python

"""
===============================================================================
Program Description 
	This program will process gps raw data and generate command to turning robot.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        November 10, 2020
Status:         In progress
===============================================================================
"""

import rospy
from gps_pose.msg import GpsPose
from minirover.msg import WheelSpeed

import math
import datetime
import pytz
from dateutil.parser import parse


class GpsNavigation:

    RADIAS = 6371000  # meters

    def __init__(self, dest_lat, dest_long):
        self.destination = GpsPose()
        self.destination.latitude = dest_lat
        self.destination.longitude = dest_long

        self.prev_coordinate = None

        self.gps_data = GpsPose()
        self.pub = rospy.Publisher('wheel_speed', WheelSpeed, queue_size=1)
        rospy.init_node('gps_pid_navigation', anonymous=True)
        rospy.Subscriber('gps_raw', GpsPose, self.gps_raw_callback)
        rospy.spin()

    def gps_raw_callback(self, gps):

        self.gps_data.latitude = float(gps.latitude)
        self.gps_data.longitude = float(gps.longitude)
        self.gps_data.raw_speed = float(gps.raw_speed)
        self.gps_data.last_update = parse(gps.last_update).astimezone(
            pytz.timezone("America/Indianapolis"))

        print('Latitude: {}'.format(self.gps_data.latitude))
        print('Longitude: {}'.format(self.gps_data.longitude))
        print('Last Update: {}'.format(self.gps_data.last_update.isoformat()))
        print('Ground Speed: {}'.format(self.gps_data.raw_speed))
        print('***************************************')

        if self.prev_coordinate is None:
            self.prev_coordinate = self.gps_data
            return

        distance = self.calulate_distance(self.gps_data, self.destination)

        print('Target Position: {:.6f}, {:.6f}'.format(
            self.destination.latitude, self.destination.longitude))
        print('Distance from target meter(s): {:,}'.format(int(distance)))

        target_direction = self.calulate_heading(self.gps_data, self.destination)
        print('Direction to target: {:.2f} degrees CCW'.format(
            target_direction))   
        
        current_direction = self.calulate_heading(self.prev_coordinate, self.gps_data)
        print('Current Direction: {:.2f} degrees CCW'.format(
            current_direction))   

        self.compute_next_commend(distance, target_direction, current_direction)

        print('---------------------------------------')

        self.prev_coordinate = self.gps_data

    def calulate_distance(self, loc_a, loc_b):
        dif_long = math.radians(loc_b.longitude - loc_a.longitude)
        dif_lat = math.radians(loc_b.latitude - loc_a.latitude)

        lat1_radian = math.radians(loc_a.latitude)
        lat2_radian = math.radians(loc_b.latitude)
        
        a = math.sin(dif_lat/2) * math.sin(dif_lat/2) +  \
            math.sin(dif_long/2) * math.sin(dif_long/2) * \
            math.cos(lat1_radian) * math.cos(lat2_radian)

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = c * self.RADIAS

        return distance

    def calulate_heading(self, loc_a, loc_b):
        dif_long = math.radians(loc_b.longitude - loc_a.longitude)
        dif_lat = math.radians(loc_b.latitude - loc_a.latitude)

        heading = math.degrees(math.atan2(dif_lat, dif_long))

        if heading < 0:
            heading += 360

        return heading
    
    def compute_next_commend(distance, target_direction, current_direction):
        speed = WheelSpeed()
        if distance < 5:    
            WheelSpeed.left = 0
            WheelSpeed.right = 0
        
        else:
            diff_heading = target_direction - current_direction
            if diff_heading > 0:
                WheelSpeed.left = 0.6
                WheelSpeed.right = 0.3
            else:
                WheelSpeed.left = 0.3
                WheelSpeed.right = 0.6

        self.pub.publish(speed)


if __name__ == '__main__':
    destination_latitute = 30.46422
    destination_longitude = -86.9477585
    navigation = GpsNavigation(destination_latitute, destination_longitude)
