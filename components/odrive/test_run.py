import odrive
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
left = odrive.find_any(serial_number="208E31834E53")
print("finding another odrive...")
right = odrive.find_any(serial_number="206737A14152")

#Right: 206737A14152
#35627687035218
#Left: 208E31834E53
#35795088133715

# print(f'Axis error code = {my_drive.axis0.error}')
# print(f'Encoder error code = {my_drive.axis0.encoder.error}')

left.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
right.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
left.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
right.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL


left.axis0.controller.input_vel = 2
right.axis0.controller.input_vel = 2
left.axis1.controller.input_vel = 2
right.axis1.controller.input_vel = 2
time.sleep(3)
left.axis0.controller.input_vel = 0
right.axis0.controller.input_vel = 0
left.axis1.controller.input_vel = 0
right.axis1.controller.input_vel = 0
