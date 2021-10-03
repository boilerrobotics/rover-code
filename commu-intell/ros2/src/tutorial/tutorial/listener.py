'''
===============================================================================
Program Description 
	This program is a demo listener.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         September 29, 2021
Version:        0.1.0
===============================================================================
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Subscriber(Node):

    def __init__(self):
        super().__init__('demo_subscriber')
        self.subscription = self.create_subscription(
            String,
            'demo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.get_logger().info(f'Recieved: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()