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
        self.driver = odrive.find_any()
        self.driver.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        print(f'Connected to ODrive serial {self.driver.serial_number}')
        self.vel_lim = self.driver.axis0.controller.config.vel_limit - 0.2
        print(f'Speeed limit: {self.vel_lim} rpm')
        # self.cmd  # prevent unused variable warning

    def cmd_callback(self, cmd):
        left_speed = cmd.linear.x
        right_speed = cmd.angular.z

        self.get_logger().info(
            f'Left Speed: {left_speed} Right Speed: {right_speed}'
        )

        self.driver.axis0.controller.input_vel = left_speed * self.vel_lim

def main(args=None):
    rclpy.init(args=args)
    
    rover = MotorDriver()
    rclpy.spin(rover)
    rover.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()