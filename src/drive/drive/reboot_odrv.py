

import asyncio
import rclpy
from rclpy.node import Node


from custom_interfaces.action import Reboot
from rclpy.action import ActionServer
from drive.odrivelib.utils import find_odrvs
from drive.odrivelib.utils import Odrive
from pathlib import Path


class OdriveReboot(Node):

    def __init__(self):
        super().__init__("reboot_actionserver")
        self.reboot_action = ActionServer(self, Reboot, "reboot", self.reboot_callback)
        self.odrvs = find_odrvs(
                config_file=Path(__file__).parents[4]
                / "share"
                / "drive"
                / "odrivelib"
                / "config.yml"
            )

    def reboot_callback(self, goal_handle):
        odr = None
        for odrv in self.odrvs:
            match odrv.section:
                case goal_handle.request.odrv: odr = odrv
            
        asyncio.run(odrv.reboot())

        goal_handle.succeed()
        result = Reboot.result()
        result.success = 'worked'
        return result



def main(args=None):
    rclpy.init(args=args)

    reboot_listener = OdriveReboot()

    rclpy.spin(reboot_listener)

    reboot_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
