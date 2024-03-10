import asyncio

from utils import Odrive, find_odrvs_async


async def calibrate(odrv: Odrive):
    print(f"Calibrating {odrv.section} odrive...")
    odrv.check_errors()  # Checking errors before starting
    if odrv.config.set_break_resistor(0.5):
        print(f"need to reset the system after enabling break resistor")
        await odrv.reboot(save_config=True)
    await odrv.calibrate()
    if odrv.has_errors():
        print(f"{odrv.section} odrive calibration fail!")
        odrv.check_errors()  # Checking errors at the end
        return

    print(f"{odrv.section} odrive calibration completed. Test run...")
    for speed in [2, 5, 10]:
        await odrv.test_run(speed, 2)
    if odrv.has_errors():
        print(f"{odrv.section} test run fail!")
        return

    print(f"{odrv.section} test run completed. Save configuration profile...")
    odrv.save_calibration_profile()
    await odrv.reboot(save_config=True)
    # check if motor is calibrated
    # and encoder is ready
    # and index is found
    # then test run
    # or do we need to run offset calibration every time at the start up


async def main():
    odrvs = await find_odrvs_async()
    print("Running Calibration ...")
    for odrv in odrvs:
        await calibrate(odrv)

    # test_run(odrvs, running_time=5, running_speed=3)
    # print("Setting pre-calibration ...")
    # pre_calibration_on(odrvs)
    # print("Test run ...")
    # test_run(odrvs, running_time=5, running_speed=3)


if __name__ == "__main__":
    asyncio.run(main())
