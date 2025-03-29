import asyncio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from custom_interfaces.action import Reboot
from rclpy.action import ActionClient
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


from drive.odrivelib.utils import find_odrvs_async, find_odrvs
from drive.odrivelib.axis import Axis


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__("ve_subscriber")
        qos_profile = QoSProfile(
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
            qos_profile,
        )
        self.reboot_client = ActionClient(self, Reboot, 'reboot')
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

        self.declare_parameter("speed_limit", 10.0)

        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.check_err = self.create_timer(.1, self.diagnose)
        self.x = 0
        self.y = 0
        self.z = math.pi / 2
        self.r = 0.9398
        self.has_errors = False

        self.dianostic = self.create_timer(10, self.dianostic)

        self.update_speed = self.create_timer(1, self.update_linear_speed_limit)

       
        #  Find all ODrives

        self.odrvs = find_odrvs(
                config_file=Path(__file__).parents[4]
                / "share"
                / "drive"
                / "odrivelib"
                / "config.yml"
            )

        """req = Reboot.Request()
            req.odrive = "front"
            future = self.reboot_caller.call_async(req)
            response = future.result()
            print(response.success)"""

        self.assign_odrive()
        self.get_logger().info("Odrives initialized")
        # configure ODrives
        self.linear_speed_limit = 10  # use turn for seconds
        self.angular_speed_limit = 3  # radians per second
        self.track_width = 1  # meters **TODO: change to actual value
        for axis in self.left_wheels + self.right_wheels:
            axis.request_close_loop_control()
            axis.controller.set_speed_limit(self.linear_speed_limit)
        self.get_logger().info("Odrives configured")


    def order_reboot(self, odrv):
        goal_msg = Reboot.Goal()
        goal_msg.odrv = odrv.section
        self.reboot_client.wait_for_server()
        return self.reboot_client.send_goal_async(goal_msg)

    def update_linear_speed_limit(self):
        self.linear_speed_limit = self.get_parameter("speed_limit").get_parameter_value().double_value


    def dianostic(self):
        for odrv in self.odrvs:
            odrv.check_errors()

    def timer_callback(self):
        time_period = 0.1
        radius = 0.1143
        wheel_vel_left = (
            -1
            * sum([x.encoder.get_vel() for x in self.right_wheels])
            / 3
            / 16
            * math.pi
            * 2
            * radius
        )
        wheel_vel_right = (
            sum([x.encoder.get_vel() for x in self.left_wheels])
            / 3
            / 16
            * math.pi
            * 2
            * radius
        )
        vx = ((wheel_vel_left + wheel_vel_right) / 2) * math.cos(self.z)
        vy = ((wheel_vel_left + wheel_vel_right) / 2) * math.sin(self.z)
        wc = (wheel_vel_right - wheel_vel_left) / (2 * self.r)

        matrix1 = [
            [math.cos(self.z), -math.sin(self.z), 0],
            [math.sin(self.z), math.cos(self.z), 0],
            [0, 0, 1],
        ]
        matrix2 = [
            [1 - (wc * time_period) ** 2, -(wc * time_period) / 2, 0],
            [(wc * time_period) / 2, 1 - (wc * time_period) ** 2, 0],
            [0, 0, 1],
        ]
        vector = [vx * time_period, vy * time_period, wc * time_period]
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

        volt = sum(odrv.get_voltage() for odrv in self.odrvs) / 3
        cur = sum(odrv.get_current() for odrv in self.odrvs) / 3

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
                case "rear":
                    self.left_wheels.append(odrv.axis1)
                    self.right_wheels.append(odrv.axis0)
    
    def diagnose(self):
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
        

        left_speed = (
            (msg.linear.x - msg.angular.z * self.track_width / 2)
            * self.linear_speed_limit
            / 2
        )
        # negative sign because of the orientation of the wheels
        right_speed = (
            ((msg.linear.x + msg.angular.z * self.track_width / 2) * -1)
            * self.linear_speed_limit
            / 2
        )
        # print(left_speed, right_speed)
        for axis in self.left_wheels:
            axis.controller.set_speed(left_speed)
        for axis in self.right_wheels:
            axis.controller.set_speed(right_speed)

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
