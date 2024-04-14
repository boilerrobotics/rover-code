import asyncio

from utils import Odrive, find_odrvs_async


async def calibrate(odrv: Odrive):
    print(f"Calibrating {odrv.section} odrive...")
    odrv.check_errors()  # Checking errors before starting
    # set configurations then reboot if needed
    if odrv.config.set_break_resistor(0.5) | odrv.set_configs():
        print(f"need to reset the system for new configurations")
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
    # odrv.save_calibration_profile()
    await odrv.reboot(save_config=True)


async def main():
    odrvs = await find_odrvs_async()
    print("Running Calibration ...")
    for odrv in odrvs:
        await calibrate(odrv)


if __name__ == "__main__":
    asyncio.run(main())
