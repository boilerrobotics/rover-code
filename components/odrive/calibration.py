import asyncio
import time
from utils import Odrive, find_odrvs_async
from enums import ControlMode, MotorType, EncoderMode, AxisState
from test_run import test_run


def pre_calibration_on(odrvs):
    """
    Set configuration to use pre-calibration profile.
    """
    for section, odrv in odrvs.items():
        utils.check_error(odrv, section)
        print(f"Calibrating {section}...")

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


async def calibrate(odrv: Odrive):
    print(f"Calibrating {odrv.section} odrive...")
    odrv.check_errors()  # Checking errors before starting
    odrv.config.set_break_resistor(0.5)
    # need to reboot after set break resistor
    await odrv.reboot(save_config=True)
    await odrv.calibrate()
    print(f"{odrv.section} odrive calibration completed")
    odrv.check_errors()  # Checking errors at the end


async def main():
    odrvs = await find_odrvs_async()
    print("Running Calibration ...")
    for odrv in odrvs:
        await calibrate(odrv)
    # print("Test run ...")
    # test_run(odrvs, running_time=5, running_speed=3)
    # print("Setting pre-calibration ...")
    # pre_calibration_on(odrvs)
    # print("Test run ...")
    # test_run(odrvs, running_time=5, running_speed=3)


if __name__ == "__main__":
    asyncio.run(main())
