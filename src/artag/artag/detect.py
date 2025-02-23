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


class ARTagDetector(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector")
        self.subscriber_raw_image = self.create_subscription(
            Image,
            "zed/zed_node/rgb/image_rect_color",
            self.detect,
            qos_profile_sensor_data,
        )
        self.publisher_cmd_vel = self.create_publisher(
            Twist, "cmd_vel", qos_profile_sensor_data
        )
        self.publisher_aruco_box = self.create_publisher(
            Image, "aruco_box", qos_profile=qos_profile_system_default
        )
        self.bridge = CvBridge()

    def parse_coords(self, corners):
        coords = []
        for cd in corners:
            x = int(str(cd[0]).split(".")[0])
            y = int(str(cd[1]).split(".")[0])
            coords.append([x, y])
        return coords

    def detect(self, msg: Image):
        try:
            image_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            aruco_parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(
                image_raw, aruco_dictionary, parameters=aruco_parameters
            )
            aruco_image = aruco.drawDetectedMarkers(image_raw.copy(), corners, ids)

            if len(corners) > 0:
                aruco_image = self.bridge.cv2_to_imgmsg(aruco_image, "bgr8")
                self.publisher_aruco_box.publish(aruco_image)
            return

        except CvBridgeError as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))
            return

        gray = cv.UMat(cv_image)
        print(type(gray))

        print(type(ids))
        # try:
        if ids.get() is not None:
            print("------------")
            aruco.drawDetectedMarkers(gray, corners, ids)
            cv.imshow("Image", gray)
            cv.waitKey(1)

            corners = [x.get() for x in corners]
            print(type(ids))
            print(ids.get())
            ids = tuple(np.array(ids.get()).tolist())

            coords = self.parse_coords(list(tuple(corners)[0][0]))
            tagmid = (coords[0][0] + coords[1][0] + coords[2][0] + coords[3][0]) / 4
            if tagmid < 220:
                seg = -1
                print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")
            elif tagmid >= 220 and tagmid < 440:
                widthperc1 = (
                    abs(coords[0][1] - coords[1][1]) / coords[0][1]
                    + abs(coords[0][1] - coords[1][1]) / coords[1][1]
                ) / 2
                widthperc2 = (
                    abs(coords[2][1] - coords[3][1]) / coords[2][1]
                    + abs(coords[2][1] - coords[3][1]) / coords[3][1]
                ) / 2
                heightperc1 = (
                    abs(coords[0][0] - coords[3][0]) / coords[0][0]
                    + abs(coords[0][0] - coords[3][0]) / coords[3][0]
                ) / 2
                heightperc2 = (
                    abs(coords[1][0] - coords[2][0]) / coords[1][0]
                    + abs(coords[1][0] - coords[2][0]) / coords[2][0]
                ) / 2
                try:
                    genperc1 = (
                        abs(
                            abs(coords[0][0] - coords[1][0])
                            - abs(coords[0][1] - coords[3][1])
                        )
                        / abs(coords[0][0] - coords[1][0])
                        + abs(
                            abs(coords[0][0] - coords[1][0])
                            - abs(coords[0][1] - coords[3][1])
                        )
                        / abs(coords[0][1] - coords[3][1])
                    ) / 2
                except ZeroDivisionError:
                    genperc1 = 0
                try:
                    genperc2 = (
                        abs(
                            abs(coords[2][0] - coords[3][0])
                            - abs(coords[1][1] - coords[2][1])
                        )
                        / abs(coords[2][0] - coords[3][0])
                        + abs(
                            abs(coords[2][0] - coords[3][0])
                            - abs(coords[1][1] - coords[2][1])
                        )
                        / abs(coords[1][1] - coords[2][1])
                    ) / 2
                except ZeroDivisionError:
                    genperc2 = 0
                seg = 0
                if all(
                    x < 0.05
                    for x in [
                        widthperc1,
                        widthperc2,
                        heightperc1,
                        heightperc2,
                        genperc1,
                        genperc2,
                    ]
                ):
                    tagside = 5  # cm
                    tagarea = tagside**2
                    captureside = abs(coords[0][0] - coords[1][0]) * 0.0264583333
                    coeff = tagside * 18.1 - 6
                    distance = coeff / captureside
                    print(
                        f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}\nDistance: {distance} cm"
                    )
                else:
                    print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")
            else:
                seg = 1
                print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")
            command = Twist()
            command.linear.x = 0.0
            if seg == -1:
                command.angular.z = -0.5
            else:
                command.angular.z = 0.5

            self.publisher_cmd_vel.publish(command)
        # except TypeError:
        # print("No tag")


def main(args=None):
    rclpy.init(args=args)

    detector = ARTagDetector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
