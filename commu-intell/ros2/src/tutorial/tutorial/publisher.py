'''
===============================================================================
Program Description 
	This program is a demo publisher.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         September 27, 2021
Version:        0.1.0
===============================================================================
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
class Publisher(Node):

    def __init__(self):
        super().__init__('demo_publisher')
        self.publisher_ = self.create_publisher(String, 'demo', 10)
        interval = 1  # seconds
        self.timer = self.create_timer(interval, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Current time is {datetime.now().replace(microsecond=0).isoformat()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
