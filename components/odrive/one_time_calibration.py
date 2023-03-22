import odrive
import utils
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)
odrvs = utils.find_odrvs()

for section, odrv in odrvs.items(): 
    utils.check_error(odrv, section) # Checking errors
    print(f'Calicating {section}...')    

    for axis in [odrv.axis0, odrv.axis1]:
        #print(f"Calibrating {axis}")
        print(f'Axis error code = {axis.error}')
        print(f'Encoder error code = {axis.encoder.error}')

        axis.encoder.config.use_index = True
        axis.requested_state = AxisState.ENCODER_INDEX_SEARCH
        while axis.current_state != AxisState.IDLE:
            utils.print_voltage_current(odrv)
            time.sleep(1)
        axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
        while axis.current_state != AxisState.IDLE:
            utils.print_voltage_current(odrv)
            time.sleep(1)
        axis.encoder.config.pre_calibrated = True
        axis.motor.config.pre_calibrated = True
    odrv.save_configuration()