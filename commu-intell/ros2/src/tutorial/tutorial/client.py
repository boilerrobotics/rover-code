'''
===============================================================================
Program Description 
	This program is a demo client.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         November 05, 2021
Version:        0.1.0
===============================================================================
'''

import rclpy
import sys
from rclpy import client
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AsyncClient(Node):

    def __init__(self):
        super().__init__('client_demo')
        self.client = self.create_client(
            AddTwoInts,
            'add_two_ints'
        )
        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Service is not avaiable...')
        self.req = AddTwoInts.Request()
    
    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
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
                client.get_logger().info(f'Result is {response.sum}')
            
            break
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()