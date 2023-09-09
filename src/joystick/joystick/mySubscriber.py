import rclpy
from rclpy.node import Node

from example_interfaces.msg import Float64MultiArray

class MySubscriber(Node):
    
    def __init__(self):
        super().__init__("mySubscriber")
        self.subscription = self.create_subscription(Float64MultiArray, 'demo', self.listenerCallback, 10)
        self.subscription
    
    def listenerCallback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args = None):
    rclpy.init(args = args)

    mySubscriber = MySubscriber()
    rclpy.spin(mySubscriber)

    mySubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

