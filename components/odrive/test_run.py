import odrive
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive = odrive.find_any()

print(f'Axis error code = {my_drive.axis0.error}')
print(f'Encoder error code = {my_drive.axis0.encoder.error}')

my_drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

my_drive.axis0.controller.input_vel = 2
time.sleep(3)
my_drive.axis0.controller.input_vel = 0

