#!/usr/bin/env python3
from audioop import add
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
from roboclaw_3 import Roboclaw
import rospy
import tf
from roboclaw_node.msg import MotorPosition, EncoderValues
from roboclaw_node.srv import HomeArm, MoveClaw

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


class Joint:
    def __init__(self, name, roboclaw, address):
        self.name = name
        self.roboclaw = roboclaw
        self.address = address
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")
            return
        self.motorNum = rospy.get_param("/roboclaw_node/joints/" + name + "/motor_num")
        self.reduction = rospy.get_param("/roboclaw_node/joints/" + name + "/reduction")
        self.speed = rospy.get_param("/roboclaw_node/joints/" + name + "/speed")
        self.is_homed = False

        PID = rospy.get_param("/roboclaw_node/joints/" + name + "/PIDMM")
        try:
            if self.motorNum == 1:
                self.roboclaw.SetM1PositionPID(self.address, PID[1], PID[2], PID[3], PID[4], 0, 0, 100000)
            elif self.motorNum == 2:
                self.roboclaw.SetM2PositionPID(self.address, PID[1], PID[2], PID[3], PID[4], 0, 0, 100000)
        except OSError as e:
            rospy.logwarn("Joint %s:\t PID Error: OSError: %d", self.name, e.errno)
            rospy.logdebug(e)
        self.set_enc(0)

    def read_enc(self):
        status, enc, crc = None, None, None

        try:
            if self.motorNum == 1:
                status, enc, crc = self.roboclaw.ReadEncM1(self.address)
            elif self.motorNum == 2:
                status, enc, crc = self.roboclaw.ReadEncM2(self.address)
            rospy.loginfo("Joint %s:\t EncM%d Reading: %d", self.name, self.motorNum, enc)
        except ValueError:
            rospy.logwarn("Joint %s:\t ReadEncM%d ValueError", self.name, self.motorNum)
            return
        except OSError as e:
            rospy.logwarn("Joint %s:\t error: ReadEncM%d OSError: %d", self.name, self.motorNum, e.errno)
            rospy.logdebug(e)
            return

        if enc != None:
            return float(enc) / self.reduction * 6.28

    def set_enc(self, count):
        try:
            if self.motorNum == 1:
                self.roboclaw.SetEncM1(self.address, count)
            elif self.motorNum == 2:
                self.roboclaw.SetEncM2(self.address, count)
            rospy.loginfo("Joint %s encoder set to %d", self.name, count)
        except OSError as e:
            rospy.logwarn("Joint %s:\t error: SetEncM%d OSError: %d", self.name, self.motorNum, e.errno)
            rospy.logdebug(e)

    def home_joint(self):
        rate = 10
        timeout = 10

        r_time = rospy.Rate(rate)
        loops = 0
        error_code = 0

        rospy.loginfo("Homing joint %s", self.name)
        try:
            if self.motorNum == 1:
                self.roboclaw.BackwardM1(self.address, 20)
                error_code = 0x400000
            elif self.motorNum == 2:
                self.roboclaw.BackwardM2(self.address, 20)
                error_code = 0x800000
            while self.roboclaw.ReadError(self.address)[1] != error_code:
                if loops >= timeout * 10:
                    rospy.logwarn("Homing joint %s failed, took longer than %ds", self.name, timeout)
                    return False
                loops += 1
                r_time.sleep()
        except OSError as e:
            rospy.logwarn("Homing joint %s error: %d", self.name, e.errno)
            rospy.logdebug(e)
            return False
        self.is_homed = True
        return True

    def go_to_position(self, rawPos):
        position = int(rawPos / 6.28 * self.reduction)

        try:
            if self.motorNum == 1:
                self.roboclaw.SpeedAccelDeccelPositionM1(self.address, self.speed[0], self.speed[1], self.speed[2], position, 1)
            elif self.motorNum == 2:
                self.roboclaw.SpeedAccelDeccelPositionM2(self.address, self.speed[0], self.speed[1], self.speed[2], position, 1)
            rospy.loginfo("Joint %s:\t command sent, position = %d", self.name, position)
        except OSError as e:
            rospy.logwarn("Joint %s:\t error: SpeedAccelDeccelPositionM%d OSError: %d", self.name, self.motorNum, e.errno)
            rospy.logdebug(e)

    def stop(self):
        if self.motorNum == 1:
            self.roboclaw.ForwardM1(self.address, 0)
        elif self.motorNum == 2:
            self.roboclaw.ForwardM2(self.address, 0)


class EEJoint:
    def __init__(self, name, roboclaw, address):
        self.name = name
        self.roboclaw = roboclaw
        self.address = address
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")
            return
        self.currentLimit = int(rospy.get_param("/roboclaw_node/joints/" + name + "/current_limit") * 100.0)
        self.motorNum = rospy.get_param("/roboclaw_node/joints/" + name + "/motor_num")
        self.status = 0  # 0 = open, 1 = closed

        try:
            if self.motorNum == 1:
                self.roboclaw.SetM1MaxCurrent(self.address, self.currentLimit)
                #rospy.loginfo(self.roboclaw.ReadM1MaxCurrent(self.address))
            elif self.motorNum == 2:
                self.roboclaw.SetM2MaxCurrent(self.address, self.currentLimit)
                #rospy.loginfo(self.roboclaw.ReadM2MaxCurrent(self.address))
        except OSError as e:
            rospy.logwarn("Joint %s:\t Current limit error: OSError: %d", self.name, e.errno)
            rospy.logdebug(e)

    def open_claw(self):
        rate = 10
        timeout = 6

        r_time = rospy.Rate(rate)
        loops = 0
        error_code = 0

        rospy.loginfo("Opening joint %s", self.name)
        try:
            if self.motorNum == 1:
                self.roboclaw.BackwardM1(self.address, 127)
                error_code = 0x400000
            elif self.motorNum == 2:
                self.roboclaw.BackwardM2(self.address, 127)
                error_code = 0x800000
            while self.roboclaw.ReadError(self.address)[1] != error_code:
                if loops >= timeout * 10:
                    rospy.logwarn("Opening joint %s failed, took longer than %ds", self.name, timeout)
                    self.stop()
                    return False
                loops += 1
                r_time.sleep()
        except OSError as e:
            rospy.logwarn("Opening joint %s error: %d", self.name, e.errno)
            rospy.logdebug(e)
            return False
        rospy.loginfo("Opening joint %s success", self.name)
        self.status = False
        return True

    def close_claw(self):
        rate = 10
        timeout = 12

        r_time = rospy.Rate(rate)
        loops = 0
        error_code = 0

        rospy.loginfo("Closing joint %s", self.name)
        try:
            if self.motorNum == 1:
                self.roboclaw.ForwardM1(self.address, 127)
                error_code = 0x010000
            elif self.motorNum == 2:
                self.roboclaw.ForwardM2(self.address, 127)
                error_code = 0x020000
            error = self.roboclaw.ReadError(self.address)[1]
            while error != error_code:
                rospy.loginfo(self.roboclaw.ReadCurrents(self.address))
                if loops >= timeout * 10:
                    rospy.logwarn("Closing joint %s failed, took longer than %ds", self.name, timeout)
                    self.stop()
                    return False
                loops += 1
                error = self.roboclaw.ReadError(self.address)[1]
                r_time.sleep()
        except OSError as e:
            rospy.logwarn("Opening joint %s error: %d", self.name, e.errno)
            rospy.logdebug(e)
            return False
        rospy.loginfo("Closing joint %s success", self.name)
        self.status = True
        return True

    def stop(self):
        if self.motorNum == 1:
            self.roboclaw.ForwardM1(self.address, 0)
        elif self.motorNum == 2:
            self.roboclaw.ForwardM2(self.address, 0)

class Node:
    def __init__(self):

        self.ERRORS = {0x000000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x000100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x000200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x000400: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x000800: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x001000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x002000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x004000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x008000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x010000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x020000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x040000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x080000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x100000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x200000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x400000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x800000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown)

        roboclaws = rospy.get_param("/roboclaw_node/motor_controllers")
        joint_params = rospy.get_param("/roboclaw_node/joints")
        self.joints = [None] * len(joint_params)

        joint_num = 1  # Change to 0 when including GL
        for controller in roboclaws:
            dev_name = roboclaws[controller]['dev']
            baud_rate = roboclaws[controller]['baud']
            address = roboclaws[controller]['address']
            roboclaw = Roboclaw(dev_name, baud_rate)

            try:
                rospy.loginfo("Connecting to Roboclaw at %d", address)
                roboclaw.Open()
            except Exception as e:
                rospy.logfatal("Could not connect to Roboclaw at %d", address)
                rospy.logdebug(e)
                rospy.signal_shutdown("Could not connect to Roboclaw")
                return

            try:
                version = roboclaw.ReadVersion(address)
            except AttributeError as e:
                rospy.logfatal("Could not connect to Roboclaw at %d", address)
                rospy.logdebug(e)
                rospy.signal_shutdown("Could not connect to Roboclaw")
                return
            except Exception as e:
                rospy.logerr(type(e).__name__)
                rospy.logwarn("Problem getting roboclaw version")
                rospy.logdebug(e)
                pass

            if not version[0]:
                rospy.logwarn("Could not get version from roboclaw")
            else:
                rospy.logdebug(repr(version[1]))

            rospy.loginfo("Connected to Roboclaw at %d", address)
            roboclaw.SetPinFunctions(address, 0x00, 0x62, 0x62)

            for joint in roboclaws[controller]['joints']:
                jointType = joint_params[joint]['type']
                if jointType == 'arm':
                    self.joints[joint_num] = Joint(joint, roboclaw, address)
                elif jointType == 'EE':
                    self.joints[joint_num] = EEJoint(joint, roboclaw, address)
                joint_num += 1

        # Diagnostic Stuff
        # self.updater = diagnostic_updater.Updater()
        # self.updater.setHardwareID("Roboclaw")
        # self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals))

        self.last_command_time = rospy.get_rostime()
        self.is_homing = False

        # Allow motor simulation:
        self.simulate = [0, 2, 3, 4, 5, 6]

        rospy.sleep(2)

        self.pub = rospy.Publisher("/brc_arm/motor_positions", EncoderValues, queue_size=10)
        self.sub = rospy.Subscriber("/brc_arm/motor_commands", MotorPosition, self.cmd_pos_callback)
        self.homeArm = rospy.Service("/brc_arm/home_arm", HomeArm, self.handle_home_arm)
        self.moveClaw = rospy.Service("/brc_arm/move_claw", MoveClaw, self.handle_move_claw)

    def run(self):
        if not rospy.is_shutdown():
            rospy.loginfo("Starting motor drive")
            r_time = rospy.Rate(10)

            while not rospy.is_shutdown():
                if self.is_homing == False and (rospy.get_rostime() - self.last_command_time).to_sec() > 1:
                    try:
                        # rospy.loginfo("Did not get command for 1 second, stopping")
                        for joint in self.joints[1:]:
                           #joint.stop()
                           dumb = 0
                    except OSError as e:
                        rospy.logerr("Could not stop")
                        rospy.logdebug(e)

                r_time.sleep()

    def handle_home_arm(self, req):
        homing_order = [0, 1, 2, 3, 4, 5]  # In order of joints to home
        self.is_homing = True
        rospy.sleep(0.5)

        status = [0] * len(homing_order)
        for i in homing_order:
            if i not in self.simulate and req.to_home[i] == 1:
                if i == homing_order[0] or i == homing_order[1] or self.joints[i-1].is_homed:
                    if self.joints[i].home_joint():
                        status[i] = 1
                    else:
                        rospy.logwarn("Could not home joint %d, previous joint has not been homed", i)
        self.is_homing = False
        return [status]

    def handle_move_claw(self, req):
        if req.goal_state == 1:
            self.joints[-1].close_claw()
        elif req.goal_state == 0:
            self.joints[-1].open_claw()
        return [self.joints[-1].status]

    # TODO need find solution to the OSError11 looks like sync problem with serial
    def publish_encoder(self, angle):
        for i in range(0, len(angle)):
            if i not in self.simulate:
                angle[i] = self.joints[i].read_enc()

        self.pub.publish(angle)

    def cmd_pos_callback(self, data):
        if self.is_homing == False:
            self.last_command_time = rospy.get_rostime()

            for i in range(0, len(data.angle) - 1):
                if i not in self.simulate:
                    self.joints[i].go_to_position(data.angle[i])

            self.publish_encoder(list(data.angle))

    # TODO: Need to make this work when more than one error is raised
    # TODO: Make work with mutliple motor controllers and motors
    def check_vitals(self, stat):
        try:
            status = self.roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(self.roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(self.roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(self.roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(self.roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            for joint in self.joints[1:]:
                joint.stop()
        except Exception as e:
            rospy.logerr("Shutdown did not work trying again")
            try:
                for joint in self.joints[1:]:
                    joint.stop()
            except Exception as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
