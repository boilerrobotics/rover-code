import asyncio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from pathlib import Path
import numpy
import odrive

from drive.odrivelib.utils import find_odrvs_async
from drive.odrivelib.axis import Axis


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__("drive_subscriber")
        self._subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.drive_callback,
            qos_profile_sensor_data,
        )
        #  Find all ODrives
        self.odrvs = asyncio.run(
            find_odrvs_async(
                config=Path(__file__).parents[4]
                / "share"
                / "drive"
                / "odrivelib"
                / "config.yml"
            )
        )
        assert len(self.odrvs) == 3, "All 3 ODrives must be connected"
        self.assign_odrive()
        self.get_logger().info("Odrives initialized")
        # configure ODrives
        self.linear_speed_limit = 3  # use turn for seconds
        self.angular_speed_limit = 1  # radians per second
        self.track_width = 1  # meters **TODO: change to actual value
        for axis in self.left_wheels + self.right_wheels:
            axis.request_close_loop_control()
            axis.controller.set_speed_limit(self.linear_speed_limit)
        self.get_logger().info("Odrives configured")

    def assign_odrive(self):
        """
        Group Odrives into a left side and right side.
        Each side has 3 axis.
        """
        self.left_wheels: list[Axis] = []
        self.right_wheels: list[Axis] = []
        for odrv in self.odrvs:
            match odrv.section:
                case "left":
                    self.left_wheels.extend([odrv.axis0, odrv.axis1])
                case "right":
                    self.right_wheels.extend([odrv.axis0, odrv.axis1])
                case "front":
                    self.left_wheels.append(odrv.axis0)
                    self.right_wheels.append(odrv.axis1)

    def drive_callback(self, msg: Twist):
        """
        Callback function that receives Twist messages.
        Convert Twist message to wheel speed.
        """
        left_speed = msg.linear.x - msg.angular.z * self.track_width / 2
        # negative sign because of the orientation of the wheels
        right_speed = (msg.linear.x + msg.angular.z * self.track_width / 2) * -1
        for axis in self.left_wheels:
            axis.controller.set_speed(left_speed)
        for axis in self.right_wheels:
            axis.controller.set_speed(right_speed)


def main(args=None):
    rclpy.init(args=args)

    drive_subscriber = DiffDriveNode()
    rclpy.spin(drive_subscriber)

    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
