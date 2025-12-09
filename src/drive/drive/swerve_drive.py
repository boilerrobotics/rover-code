import asyncio
import rclpy
from drive.srv import AxisState
from drive.msg import ControlMessage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    LivelinessPolicy,
    Duration,
)
import math
from pathlib import Path
import numpy


from drive.odrivelib.utils import find_odrvs
from drive.odrivelib.axis import Axis


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__("ve_subscriber")
        self.qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            lifespan=Duration(seconds=0.1, nanoseconds=0),
        )
        self._subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.drive_callback,
            self.qos_profile,
        )
        self._pos_publisher = self.create_publisher(
            Twist, "pos", qos_profile_sensor_data
        )
        self._bat_publisher = self.create_publisher(
            String, "bat", qos_profile_sensor_data
        )
        self._volt_cur_publisher = self.create_publisher(
            String, "volt_cur", qos_profile_sensor_data
        )
        self._vel_publisher = self.create_publisher(
            String, "vel", qos_profile_sensor_data
        )
        self._state_publisher = self.create_vel_publishers()



        self.declare_parameter("speed_limit", 10.0)

        time_period = 0.1
        self.pose_update = self.create_timer(time_period, self.odometry_callback)
        self.check_err = self.create_timer(.1, self.emergency_diagnostic)
        self.x = 0
        self.y = 0
        self.z = 0
        self.track_width = 0.9398
        self.wheelbase = 1
        self.has_errors = False

        self.constant_diagnostic = self.create_timer(10, self.constant_diagnostic)

        self.update_speed = self.create_timer(1, self.update_linear_speed_limit)

       
        #  Find all ODrives

        self.linear_speed_limit = 10  # use turn for seconds
        self.angular_speed_limit = 3  # radians per second
        self.track_width = 1  # meters **TODO: change to actual value


    def create_vel_publishers(self):
        ids = []
        with open('node_ids.txt', 'r') as f:
            ids = [x for x in f.readlines()]
        publishers = []
        for x in ids:
            publishers.append(self.create_publisher(Twist, f'/odrive{x}/cmd_vel', self.qos_profile))
        return publishers

    def update_linear_speed_limit(self):
        self.linear_speed_limit = self.get_parameter("speed_limit").get_parameter_value().double_value


    def constant_diagnostic(self):
        for odrv in self.odrvs:
            odrv.check_errors()

    def odometry_callback(self):
        pass
  
    def emergency_diagnostic(self):
        if not self.has_errors:
            for odrv in self.odrvs:
                self.has_errors = odrv.check_and_print_errors()
                if(self.has_errors):
                    future = self.order_reboot(odrv)
                    while(not future.done()):
                        print("Not done rebooting")
                    print("done rebooting")
                    self.has_errors = False
                
        


    def drive_callback(self, msg: Twist):
        """
        Callback function that receives Twist messages.
        Convert Twist message to wheel speed.
        """
        vxw, vyw, vww = (msg.linear.x, msg.linear.y, msg.angular.z) # field relative velocities x-direction, y-direction, angular
        module_positions = [(self.wheelbase / 2, self.track_width/2), (self.wheelbase/2, -self.track_width/2), (-self.wheelbase/2, self.track_width/2), (-self.wheelbase/2, -self.track_width/2)] # constants for kinematics equations
        module_vel_pos = [] #chassis motor velocities/positions in order: front left velocity, front left position, front right velocity, position, back left, back right
        for x in module_positions:
            vx = vxw - vww * x[1] #module relative x velocity
            vy = vyw + vww * x[0] #module relative y velocity
            v = math.sqrt(vx**2 + vy**2)  #module relative velocity magnitude
            w = (math.atan2(vy/vx) + math.pi) / (2 * math.pi) #module relative velocity direction, scaled to [0, 1), which is the input odrive takes in circular position control mode
            module_vel_pos.append((v, w))

        for i, (v, w) in enumerate(module_vel_pos):
            msg_v = ControlMessage()
            msg_v.control_mode = 2
            msg_v.input_mode = 2
            msg_v.input_pos = 0.0
            msg_v.input_vel = v
            msg_v.input_torque = 0.0
            self._state_publisher[i].publish(msg_v)
            msg_w = ControlMessage()
            msg_w.control_mode = 3
            msg_w.input_mode = 1
            msg_w.input_pos = w
            msg_w.input_vel = 0.0
            msg_w.input_torque = 0.0
            self._state_publisher[i+1].publish(msg_w)


        

    # Function mapping voltage to battery percentage

    def get_battery(self, volt: float) -> int:
        if volt >= 25.2:
            return 100
        elif volt >= 24.9:
            return 95
        elif volt >= 24.67:
            return 90
        elif volt >= 24.49:
            return 85
        elif volt >= 24.14:
            return 80
        elif volt >= 23.9:
            return 75
        elif volt >= 23.72:
            return 70
        elif volt >= 23.48:
            return 65
        elif volt >= 23.25:
            return 60
        elif volt >= 23.13:
            return 55
        elif volt >= 23.01:
            return 50
        elif volt >= 22.89:
            return 45
        elif volt >= 22.77:
            return 40
        elif volt >= 22.72:
            return 35
        elif volt >= 22.6:
            return 30
        elif volt >= 22.48:
            return 25
        elif volt >= 22.36:
            return 20
        elif volt >= 22.24:
            return 15
        elif volt >= 22.12:
            return 10
        elif volt >= 21.65:
            return 5
        else:
            return 0


def main(args=None):
    rclpy.init(args=args)

    drive_subscriber = DiffDriveNode()

    rclpy.spin(drive_subscriber)

    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
