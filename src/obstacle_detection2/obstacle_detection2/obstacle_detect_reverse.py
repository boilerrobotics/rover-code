import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.depth_threshold = 130
        self.min_gap_width_ratio = #idk what value yet
        self.speed = 0.3
        self.turn_speed = 0.5

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        frame = np.nan_to_num(frame)
        frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
        frame = frame.astype(np.uint8)

        height, width = frame.shape

        left_region = frame[:, :width//3]
        center_region = frame[:, width//3:2*width//3]
        right_region = frame[:, 2*width//3:]

        left_mean = np.mean(left_region)
        center_mean = np.mean(center_region)
        right_mean = np.mean(right_region)
#reverse logic of first obstacle detect, free space is where its above the threshold
        free = frame > self.depth_threshold

#finds percentage of the space that is open, ex: 0.9 is open, 0.2 is blocked
        left_free_width = np.mean(free[:, :width//3])
        center_free_width = np.mean(free[:, width//3:2*width//3])
        right_free_width = np.mean(free[:, 2*width//3:])

        twist = Twist()

        directions = {
            "left": (left_mean, left_free_width),
            "center": (center_mean, center_free_width),
            "right": (right_mean, right_free_width)
        }

#reject directions that don't fit threshold e.g. too narrow
        valid_directions = {
            k: v for k, v in directions.items()
            if v[1] >= self.min_gap_width_ratio
        }

#if a wide enough, move toward the most open one
        if valid_directions:
            best_direction = max(valid_directions, key=lambda k: valid_directions[k][0])

            twist.linear.x = self.speed

            if best_direction == "left":
                twist.angular.z = self.turn_speed
            elif best_direction == "right":
                twist.angular.z = -self.turn_speed
            else:
                twist.angular.z = 0.0
        else:
#if no wide enough, rotate
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()