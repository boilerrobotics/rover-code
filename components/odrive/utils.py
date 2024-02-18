"""
Utility functions for ODrive calibration and test.
"""

import time
import yaml
import asyncio
import odrive
from enums import AxisError, ControllerError, EncoderError, MotorError, ODriveError
from sensing import OdriveSensing


def print_voltage_current(odrv, connection: OdriveSensing | None = None) -> None:
    """
    Print voltage and current for debugging.
    """
    print(f"  voltage = {odrv.vbus_voltage:5.2f} V" f"  current = {odrv.ibus:6.4f} A")
    if connection:
        connection.publish("brc/voltage", f"{odrv.vbus_voltage:5.2f}")
        connection.publish("brc/current", f"{odrv.ibus:7.5f}")


async def find_odrvs_async(timeout=3) -> dict[str, any]:
    """
    This function will find ODrives asynchronously.
    This function is not available on Odrive library version 0.5.4
    as `odrive.find_any_async()` function was introduced in version 0.6
    """
    with open("config.yml") as fp:
        config = yaml.safe_load(fp)

    tasks = [
        asyncio.create_task(
            odrive.find_any_async(serial_number=serial),
            name=f"{section}",
        )
        for section, serial in config["serial"].items()
    ]
    print("finding ODrives...")
    done, pending = await asyncio.wait(
        tasks, timeout=timeout, return_when=asyncio.ALL_COMPLETED
    )

    odrvs = {}
    for task in tasks:
        section = task.get_name()
        if task in pending:
            print(f"Warning! finding {section} ODrive failed")
            task.cancel()
        if task in done:
            odrv = task.result()
            odrvs[section] = odrv
            print(f"-> found {section} ODrive")
            print(f"-> ", end="")
            check_version(odrv)

    return odrvs


def find_odrvs() -> dict[str, any]:
    """
    This function will find ODrive that serial numbers are list
    in the config.yml
    """

    with open("config.yml") as fp:
        config = yaml.safe_load(fp)

    print("finding ODrives...")
    odrvs = {}  # Looking for available ODrive
    for section, serial in config["serial"].items():
        print(f"searching for serial number {serial}...")
        try:
            odrv = odrive.find_any(serial_number=serial, timeout=2)
            odrvs[section] = odrv
            print(f"-> assign odrive {serial} to {section} section")
            print(f"-> ", end="")
            check_version(odrv)
        except TimeoutError as e:
            print(f"error: Cannot find serial {serial} !!")
    print("--------------------------------------")

    return odrvs


def check_error(odrv, name: str | None = None) -> None:
    """
    This function will print the error
    """
    if name is not None:
        print(f"{name} odrive checking...")
    print_voltage_current(odrv)
    print(f'  {"system error:":<12} {ODriveError(odrv.error).name:^35}')
    print(f'  {"error code:":<12} {"axis-0":^35} | {"axis-1":^35}')
    print(
        f'  {"axis":<12} '
        f"{AxisError(odrv.axis0.error).name:^35} | "
        f"{AxisError(odrv.axis1.error).name:^35}"
    )
    print(
        f'  {"motor":<12} '
        f"{MotorError(odrv.axis0.motor.error).name:^35} | "
        f"{MotorError(odrv.axis1.motor.error).name:^35}"
    )
    print(
        f'  {"controller":<12} '
        f"{ControllerError(odrv.axis0.controller.error).name:^35} | "
        f"{ControllerError(odrv.axis1.controller.error).name:^35}"
    )
    print(
        f'  {"encoder":<12} '
        f"{EncoderError(odrv.axis0.encoder.error).name:^35} | "
        f"{EncoderError(odrv.axis1.encoder.error).name:^35}"
    )

    print("--------------------------------------")


def check_version(odrv) -> None:
    """
    Print out hardware version and firmware version.
    """
    print(
        f"Firmware version is {odrv.fw_version_major}."
        f"{odrv.fw_version_minor}.{odrv.fw_version_revision}"
        f'{" " * 3} Hardware version is {odrv.hw_version_major}.'
        f"{odrv.hw_version_minor}.{odrv.hw_version_variant}"
    )


def setup_telemetry(max_attempt=15) -> OdriveSensing | bool:
    """
    Set up a MQTT connection for telemetry.
    """
    connection = OdriveSensing()
    attempts = 0
    while attempts < max_attempt and not connection.is_connected():
        print(
            f"Attempt {attempts + 1}... connection status => {connection.is_connected()}"
        )
        time.sleep(1)
        attempts += 1
        if attempts == 15:
            return False
    print(f"Telemetry connection is established")
    return connection
