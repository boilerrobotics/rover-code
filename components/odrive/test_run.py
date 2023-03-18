'''
This code is for testing odrive functionality.
** Need to impliment using async **
'''
import time
import yaml
import utils
import odrive
from odrive.enums import *

def run_seconds(odrv, running_time: int, 
                running_speed: int = 3, delay_time: int = 2) -> None:
    '''
    Run odrive for running_time seconds. 
    Delay for delay seconds both before and after running.
    Print out voltage and current every second. 
    '''
    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
    for _ in range(delay_time): 
        utils.print_voltage_current(odrv)
        time.sleep(1)
    odrv.axis0.controller.input_vel = running_speed
    odrv.axis1.controller.input_vel = running_speed
    for _ in range(running_time): 
        utils.print_voltage_current(odrv)
        time.sleep(1)
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    for _ in range(delay_time): 
        utils.print_voltage_current(odrv)
        time.sleep(1)
    odrv.axis0.controller.input_vel = -running_speed
    odrv.axis1.controller.input_vel = -running_speed
    for _ in range(running_time): 
        utils.print_voltage_current(odrv)
        time.sleep(1)
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    for _ in range(delay_time): 
        utils.print_voltage_current(odrv)
        time.sleep(1)

def test_run(odrvs, running_time=5, running_speed=3) -> None:
    '''
    Take a dictionary of section:odrive and perform test run.
    '''
    for section, odrv in odrvs.items(): # Checking errors
        print(f'{section} odrive checking...') 
        utils.print_voltage_current(odrv)
        print(f'  {"error code:":<13}axis0{" "*10}axis1')
        # How can we get error code from enum
        print(f'  {"controller":<10}{odrv.axis0.controller.error:6}'
            f'{odrv.axis1.controller.error:15}')
        print(f'  {"encoder":<10}{odrv.axis0.encoder.error:6}'
            f'{odrv.axis1.encoder.error:15}')
        print(f'  {"motor":<10}{odrv.axis0.motor.error:6}'
            f'{odrv.axis1.motor.error:15}')
        print('--------------------------------------')
    for section, odrv in odrvs.items(): # Test run
        print(f'Test run {section}...')    
        run_seconds(odrv, running_time, running_speed)
'''
-------------------------------------------------------------------------------
'''
if __name__ == '__main__':
    with open('config.yml') as fp:
        config = yaml.safe_load(fp) 

    print("finding odrives...")
    odrvs = {} # Looking for avaiable ODrive
    for section, serial in config['serial'].items():
        print(f'searching for serial number {serial}...')
        try: 
            odrv = odrive.find_any(serial_number=serial, timeout=1)
            odrvs[section] = odrv
            print(f'-> assign odrive {serial} to {section} section')
        except TimeoutError as e:
            print(f'error: Cannot find serial {serial} !!')
    print('--------------------------------------')
    test_run(odrvs, running_time=5, running_speed=3) # Call test run fucntion