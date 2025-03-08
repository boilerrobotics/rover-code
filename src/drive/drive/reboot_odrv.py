import asyncio
import rclpy
from rclpy.node import Node


from custom_interfaces.srv import Reboot
from drive.odrivelib.utils import find_odrvs_async
from drive.odrivelib.utils import Odrive
from pathlib import Path


class OdriveReboot(Node):

    def __init__(self):
        super().__init__("reboot_service")
        self.reboot_service = self.create_service(Reboot, "reboot", self.reboot_callback)
        self.right = None
        self.left = None
        self.rear = None

    async def reboot_callback(self, request, response):
        if(request == "rear"):
            await self.rear.reboot()
        response = True
        return response

    async def find_and_assign_odrvs(self):
        odrvs = await find_odrvs_async(config_file=Path(__file__).parents[4]
                / "share"
                / "drive"
                / "odrivelib"
                / "config.yml")
        for odrv in odrvs:
            match odrv.section:
                case "rear": self.rear = odrv
                case "left": self.left = odrv
                case "right": self.right = odrv



def main(args=None):
    rclpy.init(args=args)

    reboot_listener = OdriveReboot()
    asyncio.run(reboot_listener.find_and_assign_odrvs())

    rclpy.spin(reboot_listener)

    reboot_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
