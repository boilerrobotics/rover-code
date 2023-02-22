import odrive
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive = odrive.find_any()

print(my_drive.serial_number)
print(my_drive.vbus_voltage)

print(f'Axis error code = {my_drive.axis1.error}')
print(f'Encoder error code = {my_drive.axis1.encoder.error}')
with open('pre-cal.txt', 'w') as f:
    f.write(f'Axis: \n')
    f.write(str(my_drive.axis1.config))
    f.write('\n\n---------------------------\n\n')
    f.write('\nMotor: \n')
    f.write(str(my_drive.axis1.motor.config))
    f.write('\n\n---------------------------\n\n')
    f.write('\nEncoder: \n')
    f.write(str(my_drive.axis1.encoder.config))


# Calibrate motor and wait for it to finish
print("starting calibration...")
my_drive.axis1.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
while AxisState(my_drive.axis1.current_state) != AxisState.IDLE:
    time.sleep(0.1)

print(f'Axis error code = {my_drive.axis1.error}')
print(f'Encoder error code = {my_drive.axis1.encoder.error}')
with open('post-cal.txt', 'w') as f:
    f.write(f'Axis: \n')
    f.write(str(my_drive.axis1.config))
    f.write('\n\n---------------------------\n\n')
    f.write('\nMotor: \n')
    f.write(str(my_drive.axis1.motor.config))
    f.write('\n\n---------------------------\n\n')
    f.write('\nEncoder: \n')
    f.write(str(my_drive.axis1.encoder.config))

my_drive.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL

my_drive.axis1.controller.input_vel = 2
time.sleep(3)
my_drive.axis1.controller.input_vel = 0
