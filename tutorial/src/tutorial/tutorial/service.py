'''
===============================================================================
Program Description 
	This program is a demo service.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         November 05, 2021
Version:        0.2.0
===============================================================================
'''

import rclpy
import pytz
from rclpy.node import Node
from tutorial_interfaces.srv import ConvertTimezone
from tutorial_interfaces.msg import Timestamp
from datetime import date, datetime, timedelta
class ConvertTimezoneService(Node):

    def __init__(self):
        super().__init__('convert_timezone_service')
        self.srv = self.create_service(
            ConvertTimezone, 
            'convert_timezone',
            self.convert_timezones_callback
        )
    
    def convert_timezones_callback(self, request, response):
        current_time = datetime.now(pytz.utc).replace(microsecond=0) \
                       + timedelta(hours=request.timezone)
        msg = Timestamp()
        msg.year = current_time.year
        msg.month = current_time.month
        msg.day = current_time.day
        msg.hour = current_time.hour
        msg.minute = current_time.minute
        msg.second = current_time.second
        msg.timezone = request.timezone
        response.timestamp = msg
        
        return response
    
def main():
    rclpy.init()
    service = ConvertTimezoneService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        