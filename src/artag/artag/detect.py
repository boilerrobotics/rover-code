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
            Twist, "cmd_vel", qos_profile_system_default
        )
        self.publisher_aruco_box = self.create_publisher(
            Image, "aruco_box", qos_profile=qos_profile_system_default
        )
        self.subscriber_depth_image = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.cal_distance,
            qos_profile_sensor_data,
        )
        self.bridge = CvBridge()
        self.width = None
        self.center = None
        self.distance = None

    def parse_coords(self, corners):
        coords = []
        for cd in corners:
            x = int(str(cd[0]).split(".")[0])
            y = int(str(cd[1]).split(".")[0])
            coords.append([x, y])
        return coords

    def detect(self, msg: Image):
        try:
            # https://github.com/ros-perception/vision_opencv/blob/humble/cv_bridge/python/cv_bridge/core.py
            image_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.width is None:
                self.width = image_raw.shape[1]
            aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            aruco_parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(
                image_raw, aruco_dictionary, parameters=aruco_parameters
            )
            aruco_image = aruco.drawDetectedMarkers(image_raw.copy(), corners, ids)

            if len(corners) > 0:
                # find self.center of aruco tag (only look for one tag)
                moment = cv.moments(corners[0])
                # self.center x,y
                self.center = (
                    int(moment["m10"] / moment["m00"]),
                    int(moment["m01"] / moment["m00"]),
                )
                # send turning signal
                command = Twist()
                if self.distance:
                    if self.distance > 0.8:
                        command.linear.x = -0.5
                        if np.sign(self.width / 2 - self.center[0]) > 0:
                            command.angular.z = 0.3
                            cv.putText(
                                aruco_image,
                                "turn_left",
                                (self.center[0] - 20, self.center[1] - 20),
                                cv.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (255, 0, 0),
                                2,
                            )
                        else:
                            command.angular.z = -0.3
                            cv.putText(
                                aruco_image,
                                "turn_right",
                                (self.center[0] - 20, self.center[1] - 20),
                                cv.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (255, 0, 0),
                                2,
                            )
                else:
                    command.linear.x = 0.0
                    command.angular.z = 0.0
                self.publisher_cmd_vel.publish(command)
                # publish aruco box
                if self.distance:
                    cv.putText(
                        aruco_image,
                        f"distance = {self.distance:.2} m",
                        (self.center[0] + 20, self.center[1] + 20),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2,
                    )
                aruco_image = self.bridge.cv2_to_imgmsg(aruco_image, "bgr8")
                self.publisher_aruco_box.publish(aruco_image)

        except CvBridgeError as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))
            return

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

    detector = ARTagDetector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
