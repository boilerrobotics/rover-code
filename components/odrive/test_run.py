import odrive
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)
print("finding odrives...")
right_serial = "206737A14152"
left_serial = "208E31834E53"
front_serial = ""

#Right: 206737A14152
#35627687035218
#Left: 208E31834E53
#35795088133715

right_drive = odrive.find_any(right_serial)
left_drive = odrive.find_any(left_serial)
front_drive = odrive.find_any(front_serial)
axes = [right_drive.axis0, right_drive.axis1, left_drive.axis0, left_drive.axis1, front_drive.axis0, front_drive.axis1]
drives = [right_drive, left_drive, front_drive]

for axis in axes:
# print(f'Axis error code = {my_drive.axis0.error}')
# print(f'Encoder error code = {my_drive.axis0.encoder.error}')
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL

for axis in axes:
    axis.controller.input_vel = 2
time.sleep(3)
for axis in axes:
    axis.controller.input_vel = 0
