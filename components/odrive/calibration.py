import time
import utils
import test_run
from odrive.enums import *

def dump_config(odrv, filename) -> None:
    '''
    Dump all config profile for comparision
    '''
    with open(filename, 'w+') as fp:
        print(str(odrv), file=fp)
        print(f'{"---"*10}odrive config{"---"*10}', file=fp)
        print(str(odrv.config), file=fp)
        for i, axis in enumerate([odrv.axis0, odrv.axis1]):
            print(f'{"---"*10}odrive axis{i}{"---"*10}', file=fp)
            print(str(axis), file=fp)
            print(f'{"---"*10}odrive axis{i} config{"---"*10}', file=fp)
            print(str(axis.config), file=fp)
            print(f'{"---"*10}odrive axis{i} controller{"---"*10}', file=fp)
            print(str(axis.controller.config), file=fp)
            print(f'{"---"*10}odrive axis{i} motor{"---"*10}', file=fp)
            print(str(axis.motor.config), file=fp)
            print(f'{"---"*10}odrive axis{i} encoder{"---"*10}', file=fp)
            print(str(axis.encoder.config), file=fp)

def config_controller(controller) -> None:
    controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    #controller.config.pos_gain = 1
    controller.config.vel_limit = 10

def config_motor(motor) -> None:
    motor.config.pole_pairs = 7
    motor.config.calibration_current = 20.0
    motor.config.motor_type = MotorType.HIGH_CURRENT
    motor.config.resistance_calib_max_voltage = 5.0
    motor.config.requested_current_range = 20.0
    motor.config.current_control_bandwidth = 100.0
    motor.config.torque_constant = 8.27/270
    motor.config.current_lim = 20

def config_encoder(encoder) -> None:
    encoder.config.mode = EncoderMode.HALL
    encoder.config.cpr = 42
    encoder.config.pre_calibrated = False
    encoder.config.bandwidth = 100
    encoder.config.calib_scan_distance = 150

def run_calib(odrvs):
    for section, odrv in odrvs.items(): 
        utils.check_error(odrv, section) # Checking errors
        dump_config(odrv, filename=f'{section}-precal.txt')
        print(f'Calibrating {section}...')    
        odrv.config.brake_resistance = 0.5
        for axis in [odrv.axis0, odrv.axis1]:
            config_controller(axis.controller)
            config_motor(axis.motor)
            config_encoder(axis.encoder)
            axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
            while axis.current_state != AxisState.IDLE:
                utils.print_voltage_current(odrv)
                time.sleep(1)
            utils.check_error(odrv, section) # Check error again
        odrv.save_configuration() 
        dump_config(odrv, filename=f'{section}-postcal.txt')
        print(f'Section {section} calibration completed')

if __name__ == '__main__':
    odrvs = utils.find_odrvs()
    run_calib(odrvs)

