'''
===============================================================================
Program Description 
	AR tag detection for minirover.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         April 11, 2022
Version:        0.1.0
===============================================================================
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_system_default
import cv2 as cv
import cv2.aruco as aruco

class ARTagDetector(Node):
    def __init__(self, interval) -> None:
        super().__init__('ardectector')
        self.subscriber_ = self.create_subscription(Image, "image_raw", self.detect, qos_profile_system_default)
        self.timer = self.create_timer(interval, self.detect)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.camera = cv.VideoCapture(0)

    def parse_coords(self, corners):
        coords = []
        for cd in corners:
            x = int(str(cd[0]).split('.')[0])
            y = int(str(cd[1]).split('.')[0])
            coords.append([x, y])
        return coords

    def detect(self, msg: Image):
        # print('callback start!')
        frame = Image
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            print('------------')
            ids = tuple(ids.tolist())
            coords = self.parse_coords(list(tuple(corners)[0][0]))
            tagmid = (coords[0][0] + coords[1][0] + coords[2][0] + coords[3][0]) / 4
            if tagmid < 220:
                seg = -1
                print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")
            elif (tagmid >= 220 and tagmid < 440):
                widthperc1 = (abs(coords[0][1] - coords[1][1]) / coords[0][1] + abs(coords[0][1] - coords[1][1]) / coords[1][1]) / 2
                widthperc2 = (abs(coords[2][1] - coords[3][1]) / coords[2][1] + abs(coords[2][1] - coords[3][1]) / coords[3][1]) / 2
                heightperc1 = (abs(coords[0][0] - coords[3][0]) / coords[0][0] + abs(coords[0][0] - coords[3][0]) /
                                coords[3][0]) / 2
                heightperc2 = (abs(coords[1][0] - coords[2][0]) / coords[1][0] + abs(coords[1][0] - coords[2][0]) /
                                coords[2][0]) / 2
                try:
                    genperc1 = (abs(abs(coords[0][0] - coords[1][0]) - abs(coords[0][1] - coords[3][1])) / abs(coords[0][0] - coords[1][0]) + abs(abs(coords[0][0] - coords[1][0]) - abs(coords[0][1] - coords[3][1])) / abs(coords[0][1] - coords[3][1])) / 2
                except ZeroDivisionError:
                    genperc1 = 0
                try:
                    genperc2 = (abs(abs(coords[2][0] - coords[3][0]) - abs(coords[1][1] - coords[2][1])) / abs(
                        coords[2][0] - coords[3][0]) + abs(
                        abs(coords[2][0] - coords[3][0]) - abs(coords[1][1] - coords[2][1])) / abs(
                        coords[1][1] - coords[2][1])) / 2
                except ZeroDivisionError:
                    genperc2 = 0
                seg = 0
                if all(x < 0.05 for x in [widthperc1, widthperc2, heightperc1, heightperc2, genperc1, genperc2]):
                    tagside = 5 # cm
                    tagarea = tagside**2
                    captureside = abs(coords[0][0] - coords[1][0]) * 0.0264583333
                    coeff = tagside * 18.1 - 6
                    distance = coeff / captureside
                    print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}\nDistance: {distance} cm")
                else:
                    print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")
            else:
                seg = 1
                print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")

            command = Twist()
            command.linear.x = 0.8
            if seg == -1:
                command.angular.z = -0.5
            else:
                command.angular.z = 0.5

            self.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)
  
    detector = ARTagDetector(0.1)
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()