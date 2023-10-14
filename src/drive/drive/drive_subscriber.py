import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

import odrive
from odrive.enums import AxisState

from std_msgs.msg import Float32
from shared_msgs.msg import DriveCommandMsg


class DriveSubscriberNode(Node):
    MAX_VEL = 20  # TODO; turns/s?

    RIGHT_SERIAL = "206737A14152"
    LEFT_SERIAL = "208E31834E53"
    FRONT_SERIAL = "2071316B4E53"

    def __init__(self):
        super().__init__('drive_subscriber')
        self.drive_subscription = self.create_subscription(
            DriveCommandMsg,
            'drive_powers',
            self.command_callback,
            qos_profile_system_default
        )

        self.vel_subscription = self.create_subscription(
            Float32,
            'drive_max_vel',
            self.max_vel_callback,
            qos_profile_system_default
        )

        self.right_drive = odrive.find_any(serial_number=self.RIGHT_SERIAL)
        self.left_drive = odrive.find_any(serial_number=self.LEFT_SERIAL)
        self.front_drive = odrive.find_any(serial_number=self.FRONT_SERIAL)

        self.left_axes = [self.front_drive.axis0, self.left_drive.axis0, self.left_drive.axis1]
        self.right_axes = [self.front_drive.axis1, self.right_drive.axis0, self.right_drive.axis1]

    def command_callback(self, message: DriveCommandMsg):
        # Run axes from drive command [-1.0, 1.0] powers
        for axis in self.left_axes:
            axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            axis.controller.input_vel = message.left * self.MAX_VEL

        for axis in self.right_axes:
            axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            axis.controller.input_vel = -message.right * self.MAX_VEL

        self.get_logger().info(f'Set powers l: {message.left}, r: {message.right}')

    def max_vel_callback(self, message: Float32):
        self.MAX_VEL = message.data
        self.get_logger().info(f'Set max vel: {message.data}')


def main(args=None):
    rclpy.init(args=args)

    drive_subscriber = DriveSubscriberNode()
    rclpy.spin(drive_subscriber)

    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
