"""
===============================================================================
Program Description
        AR tag detection.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         April 11, 2022
Version:        0.1.0
===============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import cv2 as cv
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import math
'''
import adafruit_gps
import serial
'''

class PosController(Node):
    def __init__(self) -> None:
        super().__init__("pos_controller")
        self.create_subscription(Twist, "/pos", self.setPos, qos_profile_sensor_data)

        self.publisher_cmd_vel = self.create_publisher(
            Twist, "cmd_vel", qos_profile_sensor_data
        )
        '''
        self.uart = serial.Serial('COM5', baudrate=9600, timeout=3000)
        self.gps = adafruit_gps.GPS(self.uart)    
        '''
        self.subscriber_depth_image = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.cal_distance,
            qos_profile_sensor_data,
        )

        self.using_odom = True
        self.pos = [0, 0, 0]

        self.bridge = CvBridge()
        self.width = None
        self.center = None
        self.distance = None
        self.earth_radius = 6371000

        self.target_pos = [-.1, 0, 0]
        self.target_gps_location = {
                        'latitude': 40.46422,
                        'longitude': -86.94771
                    }
        self.current_gps_info = {
            'timestamp' : None,
            'satellites' : None,
            'altitude_m' : None,
            'speed_knots' : None,
            'track_angle' : None,
            'horizontal_dilution' : None,
            'height_geoid' : None, 
            'latitude' : None,
            'longitude' : None
        }

        time_period = 0.1

        self.timer = self.create_timer(time_period, self.pub_cmd_vel)
        '''
        self.update_gps = self.create_timer(time_period, self.update_gps_callback)

    def update_gps_callback(self):
        print('=' * 40)  # Print a separator line.
        self.gps.update()
        if not self.gps.has_fix:
        # Try again if we don't have a fix yet.
            print('Waiting for fix...')

        if self.gps.fix_quality == 1:
            print(self.gps.timestamp_utc)
            print(f'Latitude: {self.gps.latitude:.6f} degrees')
            self.current_gps_info['latitude'] = self.gps.latitude
            print(f'Longitude: {self.gps.longitude:.6f} degrees')
            self.current_gps_info['longitude'] = self.gps.longitude

            if self.gps.satellites is not None:
                print(f'# satellites: {self.gps.satellites}')
                self.current_gps_info['satellites'] = self.gps.satellites
            if self.gps.altitude_m is not None:
                print(f'Altitude: {self.gps.altitude_m} meters')
                self.current_gps_info['altitude_m'] = self.gps.altitude_m
            if self.gps.speed_knots is not None:
                print(f'Speed: {self.gps.speed_knots} knots')
                self.current_gps_info['speed_knots'] = self.gps.speed_knots
            if self.gps.track_angle_deg is not None:
                print(f'Track angle: {self.gps.track_angle_deg} degrees')
                self.current_gps_info['track_angle'] = self.gps.track_angle_deg
            if self.gps.horizontal_dilution is not None:
                print(f'Horizontal dilution: {self.gps.horizontal_dilution}')
                self.current_gps_info['horizontal_dilution'] = self.gps.horizontal_dilution
            if self.gps.height_geoid is not None:
                print(f'Height geoid: {self.gps.height_geoid} meters')        
                self.current_gps_info['height_geoid'] = self.gps.height_geoid
    '''

    def calibrate_heading(self):
        startPos = self.pos
        startTime = time.time()
        while time.time() - startTime < 3:
            msg = Twist()
            msg.linear.x = 1
            self.publisher_cmd_vel.publish(msg)
            time.sleep(3)
        msg = Twist()
        msg.linear.x = 0
        self.publisher_cmd_vel.publish(msg)
        x_error = self.pos[0] - startPos[0]
        y_error = self.pos[1] - startPos[1]

        self.pos[2] = math.atan2(y_error, x_error)

    def pub_cmd_vel(self):
        if(self.using_odom):
            '''
            if not self.isCalibrated:
                self.calibrate_heading()
                self.isCalibrated = True
            '''
            dist = math.sqrt(
                (self.target_pos[1] - self.pos[1]) ** 2
                + (self.target_pos[0] - self.pos[0]) ** 2
            )
            #       self.target_pos[2] = math.atan2(self.target_pos[1], self.target_pos[0])

            h_error = self.target_pos[2] - self.pos[2]

            msg = Twist()
            if abs(h_error) > 0.1:
                try:
                    msg.angular.z = -h_error / abs(h_error)
                except ZeroDivisionError:
                    msg.angular.z = 0.0
            elif dist > 0.1:
                msg.linear.x = 1.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

            print("------------------")
            print("Pos: ")
            print(f"x: {self.pos[0]:.2f}")
            print(f"y: {self.pos[1]:.2f}")
            print(f"heading: {self.pos[2]:.2f}")
            print()
            print(f"dist: {dist:.2f}")
            print(f"heading_error: {h_error:.2f}")

            self.publisher_cmd_vel.publish(msg)
        else:
            dif_long = math.radians(self.target_gps_location['longitude'] - self.current_gps_info['longitude'])
            dif_lat = math.radians(self.target_gps_location['latitude'] - self.current_gps_info['latitude'])

            lat1_radian = math.radians(self.current_gps_info['latitude'])
            lat2_radian = math.radians(self.target_gps_location['latitude'])

            a = math.sin(dif_lat/2) * math.sin(dif_lat/2) \
                + math.sin(dif_long/2) * math.sin(dif_long/2) \
                * math.cos(lat1_radian) * math.cos(lat2_radian)

            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            distance = c * self.earth_radius

            print(f'Target Position: {self.target_gps_location["latitude"]:.6f}, {self.target_gps_location["longitude"]:.6f}')
            print(f'Distance from target (meters): {distance:,.0f}')

            target_direction = math.degrees(math.atan2(dif_lat, dif_long))
            print(f'Direction to target: {target_direction:.2f} degrees')

            if(target_direction > .2):
                msg = Twist()
                msg.angular.z = math.copysign(1, target_direction)
            elif(distance > 1):
                msg = Twist()
                msg.linear.x = 1
            self.publisher_cmd_vel.publish(msg)



    def setPos(self, msg: Twist):
        self.pos = (msg.linear.x, msg.linear.y, msg.angular.z)

    def cal_distance(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            if self.center:
                self.distance = depth[self.center[::-1]]

        except CvBridgeError as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))
            return


def main(args=None):
    rclpy.init(args=args)

    mover = PosController()

    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
