import asyncio
import rclpy
from custom_interfaces.srv import AxisState
from custom_interfaces.msg import ControlMessage, ControllerStatus, OdriveStatus
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

#from drive.odrivelib.utils import find_odrvs
#from drive.odrivelib.axis import Axis


class SwerveDriveNode(Node):

    def __init__(self):
        super().__init__("ve_subscriber")
        self.qos_publish = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            lifespan=Duration(seconds=0.1, nanoseconds=0),
        )
        self.qos_sub = QoSProfile(
            history=HistoryPolicy.KEEP_ALL
        )
        self._subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.drive_callback,
            self.qos_publish,
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
        self._state_publisher, self._controller_status_subscriber, self._odrive_status_subscriber = self.create_odrive_publishers_subscribers()

        self.odrive_status = [OdriveStatus() for x in range(8)]
        self.controller_status = [ControllerStatus() for x in range(8)]

        self.declare_parameter("speed_limit", 10.0)

        self.time_period = 0.1
        self.state_update = self.create_timer(self.time_period, self.state_callback)
        self.update_speed = self.create_timer(self.time_period, self.update_linear_speed_limit)

        self.x = 0
        self.y = 0
        self.z = 0

        self.track_width = 0.9398
        self.wheelbase = 1
        
        self.linear_speed_limit = 10  # use turn for seconds
        self.angular_speed_limit = 3  # radians per second

        self.has_errors = False



       
        #  Find all ODrives


    def create_odrive_publishers_subscribers(self):
        ids = []
        with open('/home/gparamas/rover-code/rover-code/src/drive/drive/node_ids.txt', 'r') as f:
            ids = [x.strip() for x in f.readlines()]
        publishers = []
        control_subscribers = []
        state_subscribers = []
        for x in ids:
            def set_controller_status(msg):
                self.controller_status[x] = msg
            def set_odrive_status(msg):
                self.odrive_status[x] = msg
            publishers.append(self.create_publisher(Twist, f'/odrive{x}/control_message', self.qos_sub))
            control_subscribers.append(self.create_subscription(ControllerStatus, f'/odrive{x}/controller_status', set_controller_status, self.qos_sub))
            state_subscribers.append(self.create_subscription(OdriveStatus, f'/odrive{x}/odrive_status', set_odrive_status, self.qos_sub))
        return publishers, control_subscribers, state_subscribers
    

    def update_linear_speed_limit(self):
        self.linear_speed_limit = self.get_parameter("speed_limit").get_parameter_value().double_value


    def state_callback(self):
        module_positions = [(self.wheelbase / 2, self.track_width/2), (self.wheelbase/2, -self.track_width/2), (-self.wheelbase/2, self.track_width/2), (-self.wheelbase/2, -self.track_width/2)] # constants for kinematics equations
        positions = [self.controller_status[1], self.controller_status[3], self.controller_status[5], self.controller_status[7]]
        velocities = [self.controller_status[0], self.controller_status[2], self.controller_status[4], self.controller_status[6]]
        vix, viy = [], [] #individual module x and y velocities w.r.t. chassis
        for v, p in zip(velocities, positions):
            vix.append(v.vel_estimate * math.cos(p.pos_estimate * 2 * math.pi))
            viy.append(v.vel_estimate * math.sin(p.pos_estimate * 2 * math.pi))
        vxb = sum(vix) / 4 #rover x velocity w.r.t. chassis
        vyb = sum(viy) / 4 #rover y velocity w.r.t. chassis
        wc = sum([v1 * lx - v2 * ly for v1, v2, (lx, ly) in zip(vix, viy, module_positions)]) / sum([lx**2 + ly **2 for lx, ly in module_positions]) #rover angular velocity w.r.t. chassis
        vx = vxb * math.cos(self.z) - vyb * math.sin(self.z)
        vy = vxb * math.sin(self.z) + vyb * math.cos(self.z)

            
        
        matrix1 = [
            [math.cos(self.z), -math.sin(self.z), 0],
            [math.sin(self.z), math.cos(self.z), 0],
            [0, 0, 1],
        ]
        matrix2 = [
            [1 - (wc * self.time_period) ** 2, -(wc * self.time_period) / 2, 0],
            [(wc * self.time_period) / 2, 1 - (wc * self.time_period) ** 2, 0],
            [0, 0, 1],
        ]
        vector = [vx * self.time_period, vy * self.time_period, wc * self.time_period]
        [dx, dy, d_theta] = numpy.dot(numpy.dot(matrix1, matrix2), vector)
        self.x += dx
        self.y += dy
        self.z += d_theta

        # Publish linear and angular position to pos

        msg = Twist()
        msg.linear.x = self.x
        msg.linear.y = self.y
        msg.angular.z = self.z
        self._pos_publisher.publish(msg)

        # Getting voltage and current using average measurements

        volt = sum(odrv.bus_voltage for odrv in self.odrive_status) / len(self.odrive_status)
        cur = sum(odrv.bus_current for odrv in self.odrive_status) / len(self.odrive_status)

        # Publish battery percentage to bat topic

        bat_msg = String()
        bat_msg.data = f"Battery: {self.get_battery(volt)}"
        self._bat_publisher.publish(bat_msg)

        # Publish voltage and current to volt_cur topic
        volt_cur_msg = String()
        volt_cur_msg.data = f"Voltage: {volt}\nCurrent: {cur}"
        self._volt_cur_publisher.publish(volt_cur_msg)

        # Publish velocity to vel topic
        vel_msg = String()
        vel_msg.data = f"Velocity-x: {vx}\nVelocity-y: {vy}\nAngular: {wc}"
        self._vel_publisher.publish(vel_msg)
  

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

    drive_subscriber = SwerveDriveNode()

    rclpy.spin(drive_subscriber)

    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
