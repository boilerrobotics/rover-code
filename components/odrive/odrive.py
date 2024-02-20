import yaml
import asyncio
import odrive
from enums import ODriveError
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
        self.axis0 = Axis(odrv.axis0)
        self.axis1 = Axis(odrv.axis1)
        self.config = self.Config(odrv.config)

    def get_error(self) -> str:
        """
        Return ODrive system error
        """
        return ODriveError(self.error).name

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

    def check_errors(self, name: str | None = None) -> None:
        """
        This function will print the errors
        """
        if name is not None:
            print(f"{name} odrive checking...")
        self.print_voltage_current()
        print(f'  {"system error:":<20} {self.get_error():^35}')
        print(f'  {"error code:":<20} {"axis-0":^35} | {"axis-1":^35}')
        print(
            f'  {"axis":<20} '
            f"{self.axis0.get_error():^35} | "
            f"{self.axis1.get_error():^35}"
        )
        print(
            f'  {"motor":<20} '
            f"{self.axis0.motor.get_error():^35} | "
            f"{self.axis1.motor.get_error():^35}"
        )
        print(
            f'  {"controller":<20} '
            f"{self.axis0.controller.get_error():^35} | "
            f"{self.axis1.controller.get_error():^35}"
        )
        print(
            f'  {"encoder":<20} '
            f"{self.axis0.encoder.get_error():^35} | "
            f"{self.axis1.encoder.get_error():^35}"
        )
        print(f'  {"status :":<20} {"axis-0":^35} | {"axis-1":^35}')
        print(
            f'  {"motor calibrated":<20} '
            f"{self.axis0.motor.is_calibrated:^35} | "
            f"{self.axis1.motor.is_calibrated:^35}"
        )
        print(
            f'  {"encoder ready":<20} '
            f"{self.axis0.encoder.is_ready:^35} | "
            f"{self.axis1.encoder.is_ready:^35}"
        )
        print(
            f'  {"encoder index found":<20} '
            f"{self.axis0.encoder.index_found:^35} | "
            f"{self.axis1.encoder.index_found:^35}"
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

    class Config:
        """
        Odrive config class
        https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Config
        """

        def __init__(self, config) -> None:
            self.config = config
            # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Config.brake_resistance
            self.brake_resistance = 0.5
            # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Config.enable_brake_resistor
            self.enable_brake_resistor = True

        def set_break_resistor(self, brake_resistance: int | None = None) -> None:
            if brake_resistance:
                self.brake_resistance = brake_resistance
            self.config.brake_resistance = self.brake_resistance
            self.config.enable_brake_resistor = self.enable_brake_resistor
                
            


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
