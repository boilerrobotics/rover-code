
'''
===============================================================================
IMU 
    This program reads from MPU6050 sensor and output acceleration and angular 
    velocity

Author:         Annabel Li, li3317@purdue.edu
Maintainer:     Annabel Li, li3317@purdue.edu
Version:        April 5, 2020
Status:         In progress
===============================================================================
'''


from smbus2 import SMBus
import time
import sys

class imu_raw_data:

    address = 0x68
    bus = None
    GRA = 9.80665

    # power management
    pwr_1 = 0x6B
    pwr_2 = 0x6C

    # get register for acceleration x y z
    acce_xout = 0x3B
    acce_yout = 0x3D
    acce_zout = 0x3F
    acce_config = 0x1C # this includes self test and accelration range

    # delcare vars for scale modifier 
    # 2g : 32768 / 2 = 16384 (-32768 to + 32768)
    2g_modifier = 16384
    4g_modifier = 8192
    8g_modifier = 4096
    16g_modifier = 2048
    # don't fully get this 


    # get register for gyro x y z
    gyro_xout = 0x43
    gyro_yout = 0x45
    gyro_zout = 0x47
    gyro_config = 0x1B

    # declare vars for gyro range 
    # 250 degree per second (°/s) : 32768 / 250 = 131.072
    250deg_modifier = 131.0
    500deg_modifier = 65.5
    1000deg_modifier = 32.8
    2000deg_modifier = 16.4


    # constuctor
    def __init(self, b = 1):
        #self.address = address
        # get bus for i2c communication
        self.bus = SMBus(b)

        # Wake up MPU-6050 using power management register
        # write_byte_data(i2c_addr, register, value, force=None)
        self.bus.write_byte_data(self.address, self.pwr_1, 0x00)

    # read from bus 
    def read_i2c(self, reg):
        # read_byte_data(i2c_addr, register, force=None)
        h = self.bus.read_byte_data(self.address, reg) # [15 : 8]
        l = self.bus.read_byte_data(self.address, reg + 1) # [7 : 0]

        val = (h << 8) + l # get full 16 bits

        # 16 bits : total 65536 / 2 = 32768
        # signed : -32768 to 32768
        # if over, convert to negative 
        return -((65535 - val) + 1) if val >= 0x8000 else val

    def get_range(self, reg):
        # accelration range is +-2G by default
        range = self.bus.read_byte_data(self.address, reg)

        if range == 0x00: # AFS_SEL == 0
            return 2
        elif raw_data == 0x08: # 01 000
            return 4
        elif raw_data == 0x10: # 10 000
            return 8
        elif raw_data == 0x18: # 11 000 
            return 16
        else: # error
            return -1

    def set_range(self, reg, range):
        # reset to 0
        self.bus.write_byte_data(self.address, reg, 0x00)
        # set range
        self.bus.write_byte_data(self.address, reg, range)


    def get_accele_data(self): # returns a dict of x y z
        x = read_i2c(acce_xout)
        y = read_i2c(acce_yout)
        z = read_i2c(acce_zout)

        scale_modifier = None
        range = get_range(acce_config)
        if range == 2:
            scale_modifier = 2g_modifier
        elif range == 4:
            scale_modifier = 4g_modifier
        elif range == 8:
            scale_modifier = 8g_modifier
        elif range == 16:
            scale_modifier = 16g_modifier
        else:
            print("Error reading acceleration range \n")
            sys.exit(1)

        x = (x / scale_modifier) * self.GRA
        y = (y / scale_modifier) * self.GRA
        z = (z / scale_modifier) * self.GRA

        return {'x': x, 'y': y, 'z': z}

    def get_gyro_data(self): # returns a dict of x y z
        x = read_i2c(gyro_xout)
        y = read_i2c(gyro_yout)
        z = read_i2c(gyro_zout)

        scale_modifier = None
        range = get_range(gyro_config)
        if range == 2:
            scale_modifier = 250deg_modifier
        elif range == 4:
            scale_modifier = 250deg_modifier
        elif range == 8:
            scale_modifier = 1000deg_modifier
        elif range == 16:
            scale_modifier = 2000deg_modifier
        else:
            print("Error reading gyroscope range \n")
            sys.exit(1)

        x = (x / scale_modifier)
        y = (y / scale_modifier)
        z = (z / scale_modifier)

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        accel = get_accele_data()
        gyro = get_gyro_data()

        return {'suring' : accel['x'], 'heaving' : accel['y'], 'swaying' : accel['z'], 'pitch' : gyro['x'], 'yaw' : gyro['y'], 'roll' : gyro['z']}


        
    
'''
node : new file : imu_pose_publisher_node.py (publish raw data)
node : new file : imu_pose_subscriber_node.py
file : Imu_pose.py

setter :
publish Imu objects , have x y z _ _ _ for acceleration and gyroscope fields filled

git : make branch from master (imu_raw_data)


'''
