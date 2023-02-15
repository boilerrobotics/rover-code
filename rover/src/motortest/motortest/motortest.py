import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import odrive
from odrive.enums import *

class MotorDriver(Node):

    def __init__(self) -> None:
        super().__init__('motordriver')
        # ros topic for command from teleop keyboard package
        self.cmd = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10)
        self.my_drive = odrive.find_any()
        self.my_drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        self.vel_lim = self.my_drive.axis0.controller.config.vel_limit - .2

        self.cmd  # prevent unused variable warning

    def cmd_callback(self, cmd):

        left_speed = cmd.linear.x
        right_speed = cmd.angular.z

        self.get_logger().info(f'Left Speed: {left_speed} Right Speed: {right_speed}')

        self.my_drive.axis0.controller.input_vel = left_speed * self.vel_lim

def main(args=None):
    rclpy.init(args=args)
    
    rover = MotorDriver()
    rclpy.spin(rover)
    rover.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()