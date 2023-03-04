import odrive
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)
print("finding odrives...")
right_serial = "206737A14152"
left_serial = "208E31834E53"
front_serial = "2071316B4E53"

right_drive = odrive.find_any(serial_number = right_serial)
# left_drive = odrive.find_any(serial_number = left_serial)

#front_drive = odrive.find_any(serial_number = front_serial)
axes = [right_drive.axis0, right_drive.axis1]#, left_drive.axis0, left_drive.axis1]#, front_drive.axis0, front_drive.axis1]
drives = [right_drive]#, left_drive]#, front_drive]

for drive in drives:
    print(drive.serial_number)
    print(drive.vbus_voltage)

for axis in axes:
    #print(f"Calibrating {axis}")
    print(f'Axis error code = {axis.error}')
    print(f'Encoder error code = {axis.encoder.error}')

    axis.encoder.config.use_index = True
    axis.requested_state = AxisState.ENCODER_INDEX_SEARCH
#while AxisState(right_drive.axis0.current_state) != AxisState.IDLE or AxisState(right_drive.axis1.current_state) != AxisState.IDLE or AxisState(left_drive.axis0.current_state) != AxisState.IDLE or AxisState(left_drive.axis1.current_state) != AxisState.IDLE:
while AxisState(right_drive.axis0.current_state) != AxisState.IDLE or AxisState(right_drive.axis1.current_state) != AxisState.IDLE  :
    print("test")
    time.sleep(0.1)
for axis in axes:
    axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
#while AxisState(right_drive.axis0.current_state) != AxisState.IDLE or AxisState(right_drive.axis1.current_state) != AxisState.IDLE or AxisState(left_drive.axis0.current_state) != AxisState.IDLE or AxisState(left_drive.axis1.current_state) != AxisState.IDLE:
while AxisState(right_drive.axis0.current_state) != AxisState.IDLE or AxisState(right_drive.axis1.current_state) != AxisState.IDLE  :
    time.sleep(0.1)
for axis in axes:
    axis.encoder.config.pre_calibrated = True
    axis.motor.config.pre_calibrated = True

for drive in drives:
    drive.save_configuration()