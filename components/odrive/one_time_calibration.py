import odrive
import utils
from odrive.enums import *
import time

# Find a connected ODrive (this will block until you connect one)

def one_time(odrvs):
    for section, odrv in odrvs.items(): 
        utils.check_error(odrv, section)
        print(f'Calibating {section}...')    

        for axis in [odrv.axis0, odrv.axis1]:
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
            utils.check_error(odrv, section)
        odrv.save_configuration()

if __name__ == '__main__':
    odrvs = utils.find_odrvs()
    one_time(odrvs)