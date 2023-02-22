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
        print("finding odrives...")
        right_serial = "206737A14152"
        left_serial = "208E31834E53"
        front_serial = ""

        #Right: 206737A14152
        #35627687035218
        #Left: 208E31834E53
        #35795088133715

        self.right_drive = odrive.find_any(right_serial)
        self.left_drive = odrive.find_any(left_serial)
        self.front_drive = odrive.find_any(front_serial)
        self.axes = [self.right_drive.axis0, self.right_drive.axis1, self.left_drive.axis0, self.left_drive.axis1, self.front_drive.axis0, self.front_drive.axis1]
        self.left_motors = [self.right_drive.axis0, self.right_drive.axis1, self.front_drive.axis0] #Assumes front 0 is on the right
        self.right_motors = [self.left_drive.axis0, self.left_drive.axis1, self.front_drive.axis1]

        self.drives = [self.right_drive, self.left_drive, self.front_drive]
        for axis in self.axes:
            axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        for drive in self.drives:
            print(f'Connected to ODrive serial {drive.serial_number}')
        self.vel_lim = self.driver.axis0.controller.config.vel_limit - 0.2
        print(f'Speeed limit: {self.vel_lim} rpm')
        # self.cmd  # prevent unused variable warning

    def cmd_callback(self, cmd):
        left_speed = cmd.linear.x
        right_speed = cmd.angular.z

        self.get_logger().info(
            f'Left Speed: {left_speed} Right Speed: {right_speed}'
        )
        for motor in self.left_motors:
            motor.controller.input_vel = left_speed * self.vel_lim
        for motor in self.right_motors:
            motor.controller.input_vel = right_speed * self.vel_lim

def main(args=None):
    rclpy.init(args=args)
    
    rover = MotorDriver()
    rclpy.spin(rover)
    rover.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()