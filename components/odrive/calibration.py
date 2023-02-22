import odrive
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)

print("finding odrives...")
right_serial = "206737A14152"
left_serial = "208E31834E53"
front_serial = ""
right_drive = odrive.find_any(right_serial)
left_drive = odrive.find_any(left_serial)
front_drive = odrive.find_any(front_serial)
axes = [right_drive.axis0, right_drive.axis1, left_drive.axis0, left_drive.axis1, front_drive.axis0, front_drive.axis1]
drives = [right_drive, left_drive, front_drive]

# Calibrate motor and wait for it to finish
print("starting calibration...")
for axis in axes:
    print(f'Axis error code = {axis.error}')
    print(f'Encoder error code = {axis.encoder.error}')
    axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    axis.motor.config.pole_pairs = 7
    axis.motor.config.calibration_current = 20.0
    axis.motor.config.motor_type = 0
    axis.motor.config.resistance_calib_max_voltage = 4.0
    axis.motor.config.requested_current_range = 20.0
    axis.motor.config.current_control_bandwidth = 100.0
    axis.motor.config.torque_constant = 8.27/270
    axis.controller.config.control_mode = 2
    axis.controller.config.vel_limit = 10
    axis.encoder.config.mode = 1
    axis.encoder.config.cpr = 42
    axis.encoder.config.pre_calibrated = True
    axis.encoder.config.bandwidth = 100
    while AxisState(axis.current_state) != AxisState.IDLE:
        time.sleep(0.1)

    print(f'Axis error code = {axis.error}')
    print(f'Encoder error code = {axis.encoder.error}')

    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL

    axis.controller.input_vel = 2
    time.sleep(3)
    axis.controller.input_vel = 0
