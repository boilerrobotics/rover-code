import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

import asyncio

from odrivelib.utils import Odrive, find_odrvs_async
from odrivelib.controller import Controller

class DiffDriveNode(Node):

    def __init__(self):
        super().__init__("drive_subscriber")
        self._subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.drive_callback,
            qos_profile_sensor_data,
        )

        odrvs = asyncio.run(find_odrvs_async())
        self.odrvFront = odrvs[0]
        self.odrvLeft = odrvs[1]
        self.odrvRight = odrvs[2]
        for odrv in odrvs:
            odrv.axis0.request_close_loop_control()
            odrv.axis1.request_close_loop_control()
            if odrv.serial_number == "2071316B4E53":
                self.odrvFront = odrv
            if odrv.serial_number == "208E31834E53":
                self.odrvLeft = odrv
            if odrv.serial_number == "206737A14152":
                self.odrvRight = odrv
        
        

    # Set speed by multiplying twist linear velocity by max_speed
    def move_straight(self, vel: float):
        

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
