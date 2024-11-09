import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from odrivelib.utils import Odrive


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__("drive_subscriber")
        self._subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.drive_callback,
            qos_profile_sensor_data,
        )

    # Create a hw abstraction layer to put the ODrive axis into left and right sides

    def drive_callback(self, msg: Twist):
        # Send the command to the ODrive
        pass


def main(args=None):
    rclpy.init(args=args)

    drive_subscriber = DiffDriveNode()
    rclpy.spin(drive_subscriber)

    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
