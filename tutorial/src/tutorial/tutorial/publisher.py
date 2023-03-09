'''
===============================================================================
Program Description 
	This program is a demo publisher.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         November 05, 2021
Version:        0.2.0
===============================================================================
'''

import rclpy
import pytz
from rclpy.node import Node
from tutorial_interfaces.msg import Timestamp
from datetime import datetime
class Publisher(Node):

    def __init__(self):
        super().__init__('demo_publisher')
        self.publisher_ = self.create_publisher(Timestamp, 'demo', 10)
        interval = 1  # seconds
        self.timer = self.create_timer(interval, self.timer_callback)

    def timer_callback(self):
        msg = Timestamp()
        current_time = datetime.now(pytz.utc).replace(microsecond=0)
        msg.year = current_time.year
        msg.month = current_time.month
        msg.day = current_time.day
        msg.hour = current_time.hour
        msg.minute = current_time.minute
        msg.second = current_time.second
        msg.timezone = int(current_time.utcoffset().total_seconds()/(60**2))
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
