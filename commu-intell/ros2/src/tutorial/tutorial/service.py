'''
===============================================================================
Program Description 
	This program is a demo service.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         November 03, 2021
Version:        0.1.0
===============================================================================
'''
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class AdditionService(Node):

    def __init__(self):
        super().__init__('addition_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints',
            self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        
        return response
    
def main():
    rclpy.init()
    add_service = AdditionService()
    rclpy.spin(add_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        