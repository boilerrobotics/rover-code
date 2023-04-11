#Motors run fine initially before hitting a current limit error :(

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import odrive
from odrive.enums import *
import yaml

def find_odrvs() -> dict:
    '''
    This function will find ODrive that serial numbers are list
    in the config.yml. Need to improve to async operation.
    '''
    with open('src/motortest/motortest/config.yml') as fp:
        config = yaml.safe_load(fp) 

    print("finding odrives...")
    odrvs = {} # Looking for avaiable ODrive
    for section, serial in config['serial'].items():
        print(f'searching for serial number {serial}...')
        try: 
            odrv = odrive.find_any(serial_number=serial, timeout=10)
            odrvs[section] = odrv
            print(f'-> assign odrive {serial} to {section} section')
        except TimeoutError as e:
            print(f'error: Cannot find serial {serial} !!')
    print('--------------------------------------')

    return odrvs

class MotorDriver(Node):

    def __init__(self) -> None:
        super().__init__('motordriver')
        # ros topic for command from teleop keyboard package
        self.cmd = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10)
        self.odrvs = find_odrvs()

        self.left_motors = []
        self.right_motors = []
        for section, odrv in self.odrvs.items(): 
            if section == 'front':
                self.left_motors.append(odrv.axis0)
                self.right_motors.append(odrv.axis1)
            elif section == 'left':
                self.left_motors.extend([odrv.axis0, odrv.axis1])
            elif section == 'right':
                self.right_motors.extend([odrv.axis0, odrv.axis1])
 
        for axis in self.right_motors:
            axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        for axis in self.left_motors:
            axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        firstDrive = self.odrvs.values[0]
        self.vel_lim = firstDrive.axis0.controller.config.vel_limit - 1
        print(f'Speed limit: {self.vel_lim} rpm')
        # self.cmd  # prevent unused variable warning

    def cmd_callback(self, cmd):
        left_speed = cmd.linear.x
        right_speed = cmd.angular.z

        self.get_logger().info(
            f'Left Speed: {left_speed} Right Speed: {right_speed}'
        )
        for motor in self.left_motors:
            motor.controller.input_vel = -left_speed * self.vel_lim
        for motor in self.right_motors:
            motor.controller.input_vel = right_speed * self.vel_lim
            print(motor.controller.input_vel)

def main(args=None):
    rclpy.init(args=args)
    
    rover = MotorDriver()
    rclpy.spin(rover)
    rover.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()