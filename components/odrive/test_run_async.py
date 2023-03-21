'''
This code is for testing odrive functionality.
'''
import time
import utils
import yaml
import odrive
import asyncio
from utils import check_version
from odrive.enums import *

def find_odrvs() -> dict:
    '''
    This function will find ODrive that serial numbers are list
    in the config.yml. Need to improve to async operation.
    '''
    with open('config.yml') as fp:
        config = yaml.safe_load(fp) 

    print("finding odrives...")
    odrvs = {} # Looking for avaiable ODrive
    for section, serial in config['serial'].items():
        print(f'searching for serial number {serial}...')
        try: 
            odrv = odrive.find_any_async(serial_number=serial)
            odrvs[section] = odrv
            print(f'-> assign odrive {serial} to {section} section')
            print(f'-> ', end='')
            check_version(odrv)
        except TimeoutError as e:
            print(f'error: Cannot find serial {serial} !!')
    print('--------------------------------------')

    return odrvs

def run_seconds(odrv, running_time: int, running_speed: int = 3, 
                delay_time: int = 3, debug_interval: int = 1) -> None:
    '''
    Run odrive for running_time seconds. 
    Delay for delay seconds both before and after running.
    Print out voltage and current every second. 
    '''
    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
    for _ in range(delay_time): 
        utils.print_voltage_current(odrv)
        time.sleep(debug_interval)
    odrv.axis0.controller.input_vel = running_speed
    odrv.axis1.controller.input_vel = running_speed
    for _ in range(running_time): 
        utils.print_voltage_current(odrv)
        time.sleep(debug_interval)
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    for _ in range(delay_time): 
        utils.print_voltage_current(odrv)
        time.sleep(debug_interval)
    odrv.axis0.controller.input_vel = -running_speed
    odrv.axis1.controller.input_vel = -running_speed
    for _ in range(running_time): 
        utils.print_voltage_current(odrv)
        time.sleep(debug_interval)
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    for _ in range(delay_time): 
        utils.print_voltage_current(odrv)
        time.sleep(debug_interval)

def test_run(odrvs, running_time=5, running_speed=3) -> None:
    '''
    Take a dictionary of section:odrive and perform test run.
    '''
    for section, odrv in odrvs.items(): 
        utils.check_error(odrv, section) # Checking errors
        print(f'Test run {section}...')    
        run_seconds(odrv, running_time, running_speed) # Test run
''' 
-------------------------------------------------------------------------------
'''
if __name__ == '__main__':
    odrvs = utils.find_odrvs()
    test_run(odrvs, running_time=5, running_speed=3) # Call test run fucntion