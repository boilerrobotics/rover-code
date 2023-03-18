'''
Utility functions for ODrive calibration and test.
'''

def print_voltage_current(odrv) -> None:
    '''
    Print voltage and current for debugging.
    '''
    print(f'  voltage = {odrv.vbus_voltage:5.2f} V'
          f'  current = {odrv.ibus:5.2f} A')