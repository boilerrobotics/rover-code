'''
===============================================================================
Program Description 
	This program is a demo client.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         November 05, 2021
Version:        0.2.0
===============================================================================
'''

import rclpy
import sys
from rclpy.node import Node
from tutorial_interfaces.srv import ConvertTimezone

class AsyncClient(Node):

    def __init__(self):
        super().__init__('client_demo')
        self.client = self.create_client(
            ConvertTimezone, 
            'convert_timezone',
        )
        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Service is not avaiable...')
        self.req = ConvertTimezone.Request()
    
    def send_request(self):
        self.req.timezone = int(sys.argv[1])
        self.future = self.client.call_async(self.req)


def main():
    rclpy.init()

    client = AsyncClient()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error(f'Service faill => {e}')
            else:
                client.get_logger().info(f'Result is {response.timestamp}')
            
            break
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()