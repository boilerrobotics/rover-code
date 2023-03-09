import odrive
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)
print("finding odrives...")
right_serial = "206737A14152"
left_serial = "208E31834E53"
front_serial = "2071316B4E53"

#Right: 206737A14152
#35627687035218
#Left: 208E31834E53
#35795088133715

# right_drive = odrive.find_any(serial_number = right_serial)
# left_drive = odrive.find_any(serial_number = left_serial)
front_drive = odrive.find_any(serial_number = front_serial)
axes = [front_drive.axis0, front_drive.axis1] #[right_drive.axis0, right_drive.axis1, left_drive.axis0, left_drive.axis1]#, front_drive.axis0, front_drive.axis1]
drives = [front_drive] #[right_drive, left_drive]#, front_drive]

for drive in drives:
    print(drive.serial_number)
    print(drive.vbus_voltage)
    print()

for axis in axes:
# print(f'Axis error code = {my_drive.axis0.error}')
# print(f'Encoder error code = {my_drive.axis0.encoder.error}')
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    axis.controller.input_vel = 2
    print("test")
time.sleep(3)
for axis in axes:
    axis.controller.input_vel = 0
