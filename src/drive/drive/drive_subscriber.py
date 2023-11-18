import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

import odrive
from odrive.enums import AxisState, ControlMode, InputMode

from std_msgs.msg import Float32
from shared_msgs.msg import DriveCommandMsg, CombinedOdriveTelemetryMsg

from .telemetry import get_odrive_telemetry


class DriveSubscriberNode(Node):
    MAX_VEL = 20  # TODO; turns/s?
    MAX_ACCEL = 20

    RIGHT_SERIAL = "206737A14152"
    LEFT_SERIAL = "208E31834E53"
    FRONT_SERIAL = "2071316B4E53"

    TELEMETRY_PERIOD = 0.2  # seconds

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

        # Telemetry publishing
        self.telemetry_publisher = self.create_publisher(
            CombinedOdriveTelemetryMsg,
            'odrive_telemetry',
            qos_profile_sensor_data
        )
        self.timer = self.create_timer(self.TELEMETRY_PERIOD, self.publish_telemetry)

        # Find and initialize ODrives / axes
        self.get_logger().info('Scanning for odrives...')
        self.left_axes = []
        self.right_axes = []

        self.front_drive = self.find_odrive(self.FRONT_SERIAL)
        if self.front_drive is not None:
            self.left_axes += self.front_drive.axis0
            self.right_axes += self.front_drive.axis1

        self.left_drive = self.find_odrive(self.LEFT_SERIAL)
        if self.left_drive is not None:
            self.left_axes += [self.left_drive.axis0, self.left_drive.axis1]

        self.right_drive = self.find_odrive(self.RIGHT_SERIAL)
        if self.right_drive is not None:
            self.right_axes += [self.right_drive.axis0, self.right_drive.axis1]

        # Config all axes for ramped velocity control
        for axis in self.left_axes:
            axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            axis.controller.config.input_mode = InputMode.VEL_RAMP
            axis.controller.config.vel_ramp_rate = self.MAX_ACCEL

        for axis in self.right_axes:
            axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            axis.controller.config.input_mode = InputMode.VEL_RAMP
            axis.controller.config.vel_ramp_rate = self.MAX_ACCEL

        self.get_logger().info('Initialized axes and subscribers')

    def find_odrive(self, serial_number: str, timeout=2.0):
        try:
            drive = odrive.find_any(serial_number=serial_number, timeout=timeout)
            self.get_logger().info(f'Found odrive with serial {serial_number}')
            return drive
        except TimeoutError:
            self.get_logger().warn(f'Failed to find odrive with serial {serial_number}')
            return None

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

    def publish_telemetry(self):
        msg = CombinedOdriveTelemetryMsg()

        msg.right = get_odrive_telemetry(self.right_drive)
        msg.left = get_odrive_telemetry(self.left_drive)
        msg.front = get_odrive_telemetry(self.front_drive)

        self.telemetry_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    drive_subscriber = DriveSubscriberNode()
    rclpy.spin(drive_subscriber)

    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
