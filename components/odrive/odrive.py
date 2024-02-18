import yaml
import asyncio
import odrive
from enums import AxisError, ControllerError, EncoderError, MotorError, ODriveError
from sensing import OdriveSensing
from axis import Axis


class Odrive:
    """
    ODrive abstract class

    Full document:
    https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive

    This class is not included all configurations from the document
    """

    def __init__(self, odrv) -> None:
        self.odrv = odrv
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.vbus_voltage
        self.vbus_voltage: float = odrv.vbus_voltage
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.ibus
        self.ibus: float = odrv.ibus
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error
        self.error: int = odrv.error
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.hw_version_major
        self.hw_version = f"{odrv.hw_version_major}.{odrv.hw_version_variant}.{odrv.hw_version_variant}"
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.fw_version_major
        self.fw_version = f"{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}"
        # check axis.py
        self.axis0: Axis = odrv.axis0
        self.axis1: Axis = odrv.axis1

    def print_voltage_current(self, connection: OdriveSensing | None = None) -> None:
        """
        Print voltage and current for debugging.
        """
        print(
            f"  voltage = {self.vbus_voltage:5.2f} V" f"  current = {self.ibus:6.4f} A"
        )
        if connection:
            connection.publish("brc/voltage", f"{self.vbus_voltage:5.2f}")
            connection.publish("brc/current", f"{self.ibus:7.5f}")

    def check_error(self, name: str | None = None) -> None:
        """
        This function will print the error
        """
        if name is not None:
            print(f"{name} odrive checking...")
        self.print_voltage_current()

        print(f'  {"system error:":<12} {ODriveError(self.error).name:^35}')
        print(f'  {"error code:":<12} {"axis-0":^35} | {"axis-1":^35}')
        print(
            f'  {"axis":<12} '
            f"{AxisError(self.axis0.error).name:^35} | "
            f"{AxisError(self.axis1.error).name:^35}"
        )
        print(
            f'  {"motor":<12} '
            f"{MotorError(self.axis0.motor.error).name:^35} | "
            f"{MotorError(self.axis1.motor.error).name:^35}"
        )
        print(
            f'  {"controller":<12} '
            f"{ControllerError(self.axis0.controller.error).name:^35} | "
            f"{ControllerError(self.axis1.controller.error).name:^35}"
        )
        print(
            f'  {"encoder":<12} '
            f"{EncoderError(self.odrv.axis0.encoder.error).name:^35} | "
            f"{EncoderError(self.odrv.axis1.encoder.error).name:^35}"
        )

        print("--------------------------------------")

    def check_version(self) -> None:
        """
        Print out hardware version and firmware version.
        """
        print(
            f"Firmware version is {self.fw_version}."
            f'{" " * 3} Hardware version is {self.hw_version}.'
        )


async def find_odrvs_async(timeout=3) -> dict[str, Odrive]:
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
            odrv = Odrive(task.result())
            odrvs[section] = odrv
            print(f"-> found {section} ODrive")
            print(f"-> ", end="")
            odrv.check_version()

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
            odrv = Odrive(odrv)
            odrvs[section] = odrv
            print(f"-> assign odrive {serial} to {section} section")
            print(f"-> ", end="")
            odrv.check_version()
        except TimeoutError as e:
            print(f"error: Cannot find serial {serial} !!")
    print("--------------------------------------")

    return odrvs
