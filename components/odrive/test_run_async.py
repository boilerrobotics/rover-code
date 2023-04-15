'''
This code is for testing odrive functionality.
'''
import time
import utils
import yaml
import asyncio
from odrive.enums import *

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

async def test_run(odrv, running_time=5, running_speed=3) -> None:
    '''
    Test run asynchronously. Take one odrive at a time
    '''
    utils.check_error(odrv) # Checking errors
    run_seconds(odrv, running_time, running_speed) # Test run

async def main():
    with open('config.yml') as fp:
        config = yaml.safe_load(fp) 
    print("finding odrives...")
    odrvs = await asyncio.gather( # Find all ODrive at the same time
        *[utils.find_odrv(section, serial) 
          for section, serial in config['serial'].items()]
    )
    odrvs = [odrv for odrv in odrvs if odrv] # Filter None out
    print(len(odrvs)) # Check how many ODrives are found

''' 
-------------------------------------------------------------------------------
'''
if __name__ == '__main__':
    asyncio.run(main())