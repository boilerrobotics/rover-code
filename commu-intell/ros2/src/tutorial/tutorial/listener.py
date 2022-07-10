'''
===============================================================================
Program Description 
	This program is a demo listener.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         November 05, 2021
Version:        0.2.0
===============================================================================
'''

import rclpy
from rclpy.node import Node
from tutorial_interfaces.msg import Timestamp
from datetime import date, datetime
class Subscriber(Node):

    def __init__(self):
        super().__init__('demo_subscriber')
        self.subscription = self.create_subscription(
            Timestamp,
            'demo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        timestamp = datetime(
            year = msg.year,
            month = msg.month,
            day = msg.day,
            hour = msg.hour,
            minute = msg.minute,
            second = msg.second,
        )
        self.get_logger().info(f'Recieved: {timestamp.isoformat()}')


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()