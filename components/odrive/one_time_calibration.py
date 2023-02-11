import odrive
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive = odrive.find_any()

print(my_drive.serial_number)
print(my_drive.vbus_voltage)

print(f'Axis error code = {my_drive.axis0.error}')
print(f'Encoder error code = {my_drive.axis0.encoder.error}')

my_drive.axis0.encoder.config.use_index = True
my_drive.axis0.requested_state = AxisState.ENCODER_INDEX_SEARCH
while AxisState(my_drive.axis0.current_state) != AxisState.IDLE:
    time.sleep(0.1)

my_drive.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
while AxisState(my_drive.axis0.current_state) != AxisState.IDLE:
    time.sleep(0.1)

my_drive.axis0.encoder.config.pre_calibrated = True
my_drive.axis0.motor.config.pre_calibrated = True

my_drive.save_configuration()