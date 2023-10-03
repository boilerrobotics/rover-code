import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from sensor_msgs.msg import Joy
from shared_msgs.msg import DriveCommandMsg


class TankDrivePublisherNode(Node):

    def __init__(self):
        super().__init__('tank_drive_publisher')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.controller_callback,
            qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(
            DriveCommandMsg,
            'drive_powers',
            qos_profile_system_default
        )

    def controller_callback(self, message: Joy):
        # Read controller data, republishing it as drive powers using a simple tank drive control scheme.
        left_pow, right_pow = tank_drive(message.axes[1], message.axes[3])

        msg = DriveCommandMsg()
        msg.left = left_pow
        msg.right = right_pow

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published l: {left_pow}, r: {right_pow}')


def tank_drive(translational: float, angular: float) -> tuple[float, float]:
    """
    Convert translational and angular input to left and right wheel powers.

    In a simple tank drive control scheme, the joystick 1 y-axis controls the forward/backward
    translation and the joystick 2 x-axis controls the left/right rotation. Speeds are returned as
    a tuple of [left, right] wheel velocities.

    :param translational: The forward / back power, in [-1.0, 1.0].
    :param angular: The left / right power, in [-1.0, 1.0].
    :return: The parsed left and right wheel speeds, in [-1.0, 1.0].
    """
    scale = max(1.0, abs(translational) + abs(angular))  # Scale net inputs > 1.0 down to 1.0

    return (translational - angular) / scale, \
           (translational + angular) / scale


def main(args=None):
    rclpy.init(args=args)

    tank_publisher = TankDrivePublisherNode()
    rclpy.spin(tank_publisher)

    tank_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
