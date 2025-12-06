import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.image_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_means = self.create_publisher(Twist, '/means', 10)
        self.threshold = 130
        self.speed = 0.3
        self.turn_speed = 0.5

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        frame = (frame * 255).astype(np.uint8)
        height, width = frame.shape
        middle = frame[height//2][width//2]
        left_region   = frame[:, ::width//3]
        center_region = frame[:, width//3:2*width//3]
        right_region  = frame[:, 2*width//3::]
        
        left_mean = np.mean(left_region)
        center_mean = np.mean(center_region)
        right_mean = np.mean(right_region)

        obstacle_left = left_mean < self.threshold
        obstacle_center = center_mean < self.threshold
        obstacle_right = right_mean < self.threshold

        twist = Twist()

        if obstacle_center:
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed if right_mean > left_mean else -self.turn_speed
        elif obstacle_left:
            twist.linear.x = self.speed
            twist.angular.z = 0.0
        elif obstacle_right:
            twist.linear.x = self.speed
            twist.angular.z = 0.0
        else:
            twist.linear.x = self.speed
            twist.angular.z = 0.0

        self.pub.publish(twist)
        means = Twist()
        means.linear.x = float(left_mean)
        means.linear.y = float(right_mean)
        means.linear.z = float(center_mean)
        means.angular.z = float(middle)
        self.pub_means.publish(means)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
