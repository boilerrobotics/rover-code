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


class PosController(Node):
    def __init__(self) -> None:
        super().__init__("pos_controller")
        self.create_subscription(Twist, "/pos", self.setPos, qos_profile_sensor_data)

        self.publisher_cmd_vel = self.create_publisher(
            Twist, "cmd_vel", qos_profile_sensor_data
        )

        self.subscriber_depth_image = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.cal_distance,
            qos_profile_sensor_data,
        )
        self.pos = [0, 0, 0]
        self.bridge = CvBridge()
        self.width = None
        self.center = None
        self.distance = None
        self.isCalibrated = True

        self.target_pos = [0, 0, 0]
        time_period = 0.1

        self.timer = self.create_timer(time_period, self.pub_cmd_vel)

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
        if not self.isCalibrated:
            self.calibrate_heading()
            self.isCalibrated = True

        dist = math.sqrt(
            (self.target_pos[1] - self.pos[1]) ** 2
            + (self.target_pos[0] - self.pos[0]) ** 2
        )
        # self.target_pos[2] = math.atan2(self.target_pos[1], self.target_pos[0])

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
