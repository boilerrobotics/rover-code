import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

import odrive
from odrive.enums import AxisState, ControlMode, InputMode

from std_msgs.msg import Float32
from shared_msgs.msg import DriveCommandMsg, OdriveTelemetry, OdriveAxisTelemetry, CombinedOdriveTelemetryMsg


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
        # TODO: better logging for finding odrives
        self.right_drive = odrive.find_any(serial_number=self.RIGHT_SERIAL)
        self.left_drive = odrive.find_any(serial_number=self.LEFT_SERIAL)
        self.front_drive = odrive.find_any(serial_number=self.FRONT_SERIAL)

        self.left_axes = [self.front_drive.axis0, self.left_drive.axis0, self.left_drive.axis1]
        self.right_axes = [self.front_drive.axis1, self.right_drive.axis0, self.right_drive.axis1]

        # Config all axes for ramped velocity control
        for axis in self.left_axes:
            axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            axis.controller.config.input_mode = InputMode.VEL_RAMP
            axis.controller.config.vel_ramp_rate = self.MAX_ACCEL

        for axis in self.right_axes:
            axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            axis.controller.config.input_mode = InputMode.VEL_RAMP
            axis.controller.config.vel_ramp_rate = self.MAX_ACCEL

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


def get_odrive_telemetry(drive) -> OdriveTelemetry:
    telemetry = OdriveTelemetry()

    telemetry.voltage = drive.vbus_voltage
    telemetry.current = drive.ibus
    telemetry.misconfigured = drive.misconfigured

    telemetry.axis0 = get_axis_telemetry(drive.axis0)
    telemetry.axis1 = get_axis_telemetry(drive.axis1)

    return telemetry


def get_axis_telemetry(axis) -> OdriveAxisTelemetry:
    telemetry = OdriveAxisTelemetry()

    telemetry.input_vel = axis.controller.input_vel
    telemetry.vel_setpoint = axis.controller.vel_setpoint

    telemetry.vel_estimate = axis.encoder.vel_estimate

    return telemetry


def main(args=None):
    rclpy.init(args=args)

    drive_subscriber = DriveSubscriberNode()
    rclpy.spin(drive_subscriber)

    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
