
from __future__ import division

from UDPComms import Subscriber, timeout,Publisher
from roboclaw_interface import RoboClaw
import math
import time
import RPi.GPIO as GPIO

FIRST_LINK = 457.2
SECOND_LINK = 457.2


FIRST_LINK = 500
SECOND_LINK = 500

SHOULDER_HOME_INPUT = 20
ELBOW_HOME_INPUT = 16
SHOULDER_HOME = -1
ELBOW_HOME = -2.78

# native angles = 0 at extension
# native angles = positive in the math direction

def find_serial_port():
    return '/dev/serial0'

class Arm:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SHOULDER_HOME_INPUT, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        GPIO.setup(ELBOW_HOME_INPUT, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        self.target_vel = Subscriber(8410)

        self.xyz_names = ["x", "y","yaw"]

        self.output_pub = Publisher(8420)

        self.cartesian_motors = ["shoulder","elbow","yaw"]
        self.motor_names = ["shoulder","elbow","yaw","roll","grip"]
        self.pwm_names = ["pitch"]
        self.ordering = ["shoulder","elbow","pitch","yaw","grip","roll"]
        self.native_positions = { motor:0 for motor in self.motor_names}

        self.currents = { motor:0 for motor in self.motor_names}
        self.xyz_positions    = { axis:0 for axis in self.xyz_names}
        self.elbow_left = True

        self.CPR = {'shoulder': -12.08 * 4776.38,
                    'elbow'   : -12.08 * 2442.96,
                    'yaw'     : -float(48)/27 * 34607,
                    'roll'    : 455.185*float(12*53/20),
                    'grip'    : 103.814*float(12*36/27)}
        #            'pitch'   : -2 * 34607}

        self.SPEED_SCALE = 20

        self.rc = RoboClaw(find_serial_port(), names = self.ordering,\
                                                    addresses = [130,128,129])
        self.zeroed = False

        self.storageLoc = [0,0]
	
        self.limits = {'shoulder':[-2.18,2.85],
                       'elbow'   : [-4,2.24], #-2.77
                       'yaw'     : [-3.7,3.7] }

        self.dock_pos = {'shoulder': 2.76,
                         'elbow'   : -2.51,
                         'yaw'     : -3.01 }
        self.dock_speeds = [.01,.006]

        self.forcing = False

        try:
            while 1:
                start_time = time.time()
                self.update()
                while (time.time() - start_time) < 0.1:
                    pass

        except KeyboardInterrupt:
            self.send_speeds( {motor: 0 for motor in self.motor_names}, {motor: 0 for motor in self.pwm_names} )
            raise
        except:
            self.send_speeds( {motor: 0 for motor in self.motor_names}, {motor: 0 for motor in self.pwm_names} )
            raise

    #deadbands & nonlinear controls on speed
    def condition_input(self,target): 
        target['x']     = - target['x']
        if(target['pitch'] > .1):
        	target['pitch'] = -1.23* (target['pitch']-.1)**2
        elif(target['pitch'] < -.1):
                target['pitch'] = 1.23* (target['pitch']+.1)**2
        else:
        	target['pitch'] = 0
        target['grip'] =  .04*target['grip']
        target['roll'] = .01*target['roll']
        target['z'] = - target['z']
        if(target['yaw'] > .1): 
        	target['yaw']  = 0.008* (1.1 * (target['yaw']-.1))**3
        elif(target['yaw'] < -.1): 
        	target['yaw']  = 0.008* (1.1 * (target['yaw']+.1))**3
        else:
        	target['yaw'] = 0
        # rotates command frame to end effector orientation
        angle = self.xyz_positions['yaw']
        x = -target['x']*abs(target['x'])
        y = target['y']*abs(target['y'])
        if(target['trueXYZ'] == 0):
        	target['x'] = x*math.cos(angle) - y*math.sin(angle)
        	target['y'] = x*math.sin(angle) + y*math.cos(angle)

        return target

    #output all speeds to all motors
    def send_speeds(self, speeds, target):
        for motor in self.motor_names:
            print('driving', motor, 'at', int(self.SPEED_SCALE * self.CPR[motor] * speeds[motor]))
            if int(self.SPEED_SCALE * self.CPR[motor] * speeds[motor]) == 0:
                self.rc.drive_duty(motor, 0)
            else:
                self.rc.drive_speed(motor, int(self.SPEED_SCALE * self.CPR[motor] * speeds[motor]))

        for motor in self.pwm_names:
            print('driving pwm', motor, 'at', int(20000*target[motor]))
            self.rc.drive_duty(motor, int(20000*target[motor]))

    #check if motors are homed, get position & currents
    def get_status(self):
        self.forcing = False
        for i,motor in enumerate(self.cartesian_motors):
            encoder = self.rc.read_encoder(motor)[1]
            print(motor,encoder)
            self.native_positions[motor] = 2 * math.pi * encoder/self.CPR[motor]
            self.currents[motor] = self.rc.read_current(motor)
            if(self.currents[motor] > 1.5):
                self.forcing = True

        self.xyz_positions = self.native_to_xyz(self.native_positions)
        print("Current Native: ", self.native_positions)
        print("Current    XYZ: ", self.xyz_positions)

    def xyz_to_native(self, xyz):
        native = {}

        distance = math.sqrt(xyz['x']**2 + xyz['y']**2)
        angle    = -math.atan2(xyz['x'], xyz['y'])

        offset = math.acos( ( FIRST_LINK**2 + distance**2 - SECOND_LINK**2  ) / (2*distance * FIRST_LINK) )
        inside = math.acos( ( FIRST_LINK**2 + SECOND_LINK**2 - distance**2  ) / (2*SECOND_LINK * FIRST_LINK) )

        if self.elbow_left:
            # is in first working configuration
            native['shoulder'] = angle + offset - math.pi
            native['elbow']    = - (math.pi - inside)
        else:
            native['shoulder'] = angle - offset - math.pi
            native['elbow']    = (math.pi - inside)

        native['yaw']   = xyz['yaw'] -native['shoulder']-native['elbow']
#        native['pitch'] = xyz['pitch']

        return native

    def native_to_xyz(self, native):
        xyz = {}
        xyz['x'] = -FIRST_LINK * math.sin(native['shoulder']) - SECOND_LINK * math.sin(native['shoulder'] + native['elbow'])
        xyz['y'] = FIRST_LINK * math.cos(native['shoulder']) + SECOND_LINK * math.cos(native['shoulder'] + native['elbow'])

        xyz['yaw']   = native['yaw']  + (native['shoulder']+native['elbow'])
#        xyz['pitch'] = native['pitch']
        return xyz

    # analytic jacobian of angles -> XYZ
    def dnative(self, dxyz):
        x = self.xyz_positions['x']
        y = self.xyz_positions['y']

        shoulder_diff_x = y/(x**2 + y**2) - (x/(FIRST_LINK*math.sqrt(x**2 + y**2)) - x*(FIRST_LINK**2 - SECOND_LINK**2 + x**2 + y**2)/(2*FIRST_LINK*(x**2 + y**2)**(3/2)))/math.sqrt(1 - (FIRST_LINK**2 - SECOND_LINK**2 + x**2 + y**2)**2/(4*FIRST_LINK**2*(x**2 + y**2)))

        shoulder_diff_y = -x/(x**2 + y**2) - (y/(FIRST_LINK*math.sqrt(x**2 + y**2)) - y*(FIRST_LINK**2 - SECOND_LINK**2 + x**2 + y**2)/(2*FIRST_LINK*(x**2 + y**2)**(3/2)))/math.sqrt(1 - (FIRST_LINK**2 - SECOND_LINK**2 + x**2 + y**2)**2/(4*FIRST_LINK**2*(x**2 + y**2)))

        elbow_diff_x = -x/(FIRST_LINK*SECOND_LINK*math.sqrt(1 - (FIRST_LINK**2 + SECOND_LINK**2 - x**2 - y**2)**2/(4*FIRST_LINK**2*SECOND_LINK**2)))

        elbow_diff_y = -y/(FIRST_LINK*SECOND_LINK*math.sqrt(1 - (FIRST_LINK**2 + SECOND_LINK**2 - x**2 - y**2)**2/(4*FIRST_LINK**2*SECOND_LINK**2)))

        dnative = {}
        dnative['shoulder'] = shoulder_diff_x * dxyz['x'] + shoulder_diff_y * dxyz['y'] 
        dnative['elbow']    = elbow_diff_x    * dxyz['x']    + elbow_diff_y * dxyz['y'] 

        print("Dxyz   : ", dxyz)
        print("Dnative: ", dnative)
        print("new location: ", self.native_to_xyz ( {motor:dnative[motor] + self.native_positions[motor] for motor in self.motor_names}) )
        return dnative

    #numerical jacobian of angles -> XYZ
    def dnative2(self, dxyz):
        h = 0.00000001
       # print dxyz, self.xyz_positions
        x_plus_h = { axis:self.xyz_positions[axis] + h*dxyz[axis] for axis in self.xyz_names}

        f_x_plus_h = self.xyz_to_native(x_plus_h)
        f_x        = self.xyz_to_native(self.xyz_positions)

        dnative = {motor:(f_x_plus_h[motor] - f_x[motor])/h for motor in self.cartesian_motors}

        print("Dxyz   : ", dxyz)
        print("Dnative: ", dnative)
        print("new location: ", self.native_to_xyz ( {motor:dnative[motor] + f_x[motor] for motor in self.cartesian_motors}) )
        return dnative

    def sign(self,val):
        return int(val > 0) - int(val < 0)

    #prevent movement near singularity or if the motor is out of bounds or at home
    def check_in_bounds(self, speeds):
        inBounds = True 
        
        if(abs(self.native_positions['elbow']) < .4):
            if(self.native_positions['elbow'] < 0 and self.sign(speeds['elbow']) == 1):
                inBounds = False
                print("SINGULARITY")
            elif(self.native_positions['elbow'] > 0 and self.sign(speeds['elbow']) == -1):
                inBounds = False
                print("SINGULARITY")
        for motor in self.cartesian_motors:
            if(self.sign(speeds[motor]) == -1):
                if(self.native_positions[motor] < self.limits[motor][0]):
                    inBounds = False
            elif(self.native_positions[motor] > self.limits[motor][1]):
                inBounds = False
        if(not inBounds):
            for motor in self.cartesian_motors:
               speeds[motor] = 0
        return speeds

    def check_limits(self, speeds):
        inBounds = True 
        if(GPIO.input(SHOULDER_HOME_INPUT) and self.sign(speeds['elbow']) == 1):
            self.rc.set_encoder('shoulder',SHOULDER_HOME)
            inBounds = False

        if(GPIO.input(ELBOW_HOME_INPUT) and self.sign(speeds['elbow']) == -1):
            self.rc.set_encoder('elbow',ELBOW_HOME*self.cpr['elbow'])
            inBounds = False
        if(not inBounds):
            for motor in self.cartesian_motors:
               speeds[motor] = 0
        return speeds

    #return to our docking position
    def dock(self,speeds):
        if(abs(self.native_positions['yaw']-self.dock_pos['yaw']-.03)>.01):
            speeds['shoulder'] = 0
            speeds['elbow'] = 0
            speeds['yaw'] = self.dock_speed(self.native_positions['yaw'],self.dock_pos['yaw']+.03,self.dock_speeds[0],self.dock_speeds[1])
        elif(abs(self.native_positions['elbow']-self.dock_pos['elbow']+.03)>.01):
            speeds['shoulder'] = 0
            speeds['elbow'] = self.dock_speed(self.native_positions['elbow'],self.dock_pos['elbow']-.03,self.dock_speeds[0],self.dock_speeds[1])
            speeds['yaw'] = 0
        elif(abs(self.native_positions['shoulder']-self.dock_pos['shoulder']-.03)>.01):
            speeds['shoulder'] = self.dock_speed(self.native_positions['shoulder'],self.dock_pos['shoulder']+.03,self.dock_speeds[0],self.dock_speeds[1])
            speeds['elbow'] = 0
            speeds['yaw'] = 0
        return speeds


    #PID for docking speed
    def dock_speed(self,curPos,desiredPos,P,maxV):
        dir = self.sign(desiredPos-curPos)
        output = abs(curPos-desiredPos)*P+.0005
        if(output > maxV):
            output = maxV
        return output*dir

    def update(self):
        print()
        print("new iteration")
        self.get_status()
        output = {}
        print("read status of shoulder", GPIO.input(20))
        for d in (self.native_positions,self.xyz_positions): 
            output.update(d)
        #print("HOME STATUS ", self.home_status)
        #output.update({"shoulder limit", self.home_status["shoulder"]})
        output.update({"forcing":self.forcing})
        self.output_pub.send(output)
        try:
            target = self.target_vel.get()
            # TODO: This shouldn't be necessary, how to fix in UDPComms?
            target = {bytes(key): value for key, value in target.iteritems()}

            target_f = self.condition_input(target)
            speeds = self.dnative2(target_f) 
            if(not self.zeroed):
                speeds['elbow'] = 0
                speeds['shoulder'] = 0
            speeds = self.check_in_bounds(speeds)
            speeds['elbow'] -= 0.002 * target_f['hat'][0]
            speeds['shoulder'] -= 0.002 * target_f['hat'][1]
            speeds['yaw'] += target_f['yaw']
            speeds['roll'] = target_f['roll']
            speeds['grip'] = target_f['grip'] + speeds['roll']
            speeds = self.check_limits(speeds)
            if speeds['elbow'] == 0 and speeds['shoulder'] == 0:
                self.elbow_left = self.native_positions['elbow'] < 0

            if(target_f['dock']):
                speeds=self.dock(speeds)
	   
            if target_f["reset"]:
                print ("RESETTING!!!")
                self.zeroed = True
                speeds = {motor: 0 for motor in self.motor_names}
                target_f = {motor: 0 for motor in self.pwm_names}
                self.send_speeds(speeds, target_f)
                self.rc.set_encoder("shoulder",0)
                self.rc.set_encoder("elbow",0)
                self.rc.set_encoder("yaw",0)

            elif target_f["resetdock"]:
                print ("RESETTING (in dock position)!!!")
                self.zeroed = True
                speeds = {motor: 0 for motor in self.motor_names}
                target_f = {motor: 0 for motor in self.pwm_names}
                self.send_speeds(speeds, target_f)
                self.rc.set_encoder("shoulder",int(self.CPR["shoulder"]*self.dock_pos['shoulder']/6.28))
                self.rc.set_encoder("elbow",int(self.CPR["elbow"]*self.dock_pos['elbow']/6.28))
                self.rc.set_encoder("yaw",int(self.CPR["yaw"]*self.dock_pos['yaw']/6.28))

        except timeout:
            print ("TIMEOUT No commands recived")
            speeds = {motor: 0 for motor in self.motor_names}
            target_f = {}
            target_f = {motor: 0 for motor in self.pwm_names}
        except ValueError:
            print ("ValueError The math failed")
            speeds = {motor: 0 for motor in self.motor_names}

            speeds['elbow'] -= 0.002 * target_f['hat'][0]
            speeds['shoulder'] -= 0.002 * target_f['hat'][1]
        except:
            speeds = {motor: 0 for motor in self.motor_names}
            target_f = {motor: 0 for motor in self.pwm_names}
            raise
        finally:
            print ("SPEEDS"), speeds, target_f
            self.send_speeds(speeds, target_f)
        # exit()




if __name__ == "__main__":
    a = Arm()
