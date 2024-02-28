import yaml
import asyncio
import odrive
from enums import ODriveError, Error
from sensing import OdriveSensing
from axis import Axis


class Odrive(Error):
    """
    ODrive abstract class

    Full document:
    https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive

    This class is not included all configurations from the document
    """

    def __init__(self, odrv, section) -> None:
        self.odrv = odrv
        self.section = section
        self.serial_number: str = f"{odrv.serial_number:x}".upper()
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.hw_version_major
        self.hw_version = f"{odrv.hw_version_major}.{odrv.hw_version_minor} {odrv.hw_version_variant}V"
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.fw_version_major
        self.fw_version = f"{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}"
        # check axis.py
        self.axis0 = Axis(odrv.axis0)
        self.axis1 = Axis(odrv.axis1)
        self.config = self.Config(odrv.config)

    def get_errors(self) -> str:
        """
        Return ODrive system errors
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error
        errors = self.decode_errors(self.odrv.error)
        return " & ".join([ODriveError(error).name for error in errors])

    def print_voltage_current(self, connection: OdriveSensing | None = None) -> None:
        """
        Print voltage and current for debugging.
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.vbus_voltage
        vbus_voltage: float = self.odrv.vbus_voltage
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.ibus
        ibus: float = self.odrv.ibus
        print(f"  voltage = {vbus_voltage:5.2f} V" f"  current = {ibus:6.4f} A")
        if connection:
            connection.publish("brc/voltage", f"{vbus_voltage:5.2f}")
            connection.publish("brc/current", f"{ibus:7.5f}")

    def print_errors(self, component, name: str) -> str: 
        """
        This function print part of the errors
        """
        if errors := component.get_errors():
            print(f'  {name} has {errors}')

    def check_errors(self) -> None:
        """
        This function will print the errors
        """
        print("-" * 100)
        print(f"{self.section} odrive status check...")
        self.print_voltage_current()
        self.print_errors(self, "system errors")
        self.print_errors(self.axis0, "axis 0")
        self.print_errors(self.axis1, "axis 1")
        self.print_errors(self.axis0.motor, "motor 0")
        self.print_errors(self.axis1.motor, "motor 1")
        self.print_errors(self.axis0.controller, "controller 0")
        self.print_errors(self.axis1.controller, "controller 1")
        self.print_errors(self.axis0.encoder, "encoder 0")
        self.print_errors(self.axis1.encoder, "encoder 1")
        print(f'  {"status :":<20}')
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
        print("-" * 100)

    def check_version(self) -> None:
        """
        Print out hardware version and firmware version.
        """
        print(
            f"Firmware version is {self.fw_version}"
            f'{" " * 3} Hardware version is {self.hw_version}'
        )

    async def reboot(self, save_config: bool = False) -> None:
        """
        Reboot Odrive then reconnect with 30 seconds timeout.
        Option to save config
        """
        try:
            if save_config:
                self.odrv.save_configuration()
            else:
                self.odrv.reboot()
        except Exception:
            print(
                f"rebooting {self.section} odrive..."
            )  # odrive will disapper. expect error will happen
        finally:
            # find odrive again
            try:
                odrv = await asyncio.wait_for(
                    odrive.find_any_async(serial_number=self.serial_number), timeout=30
                )
                self.odrv = odrv
                self.axis0 = Axis(odrv.axis0)
                self.axis1 = Axis(odrv.axis1)
            except asyncio.TimeoutError:
                print("Timeout: cannot reconnect with Odrive")
            except Exception as e:
                print(f"Unknown error: {e}")
            else:
                print(f"{self.section} odrive is online...")

    async def calibrate(self) -> None:
        print(self.axis0.get_state())
        print(self.axis1.get_state())
        self.print_voltage_current()
        # Calibration can be done only one axis at a time
        self.axis0.request_full_calibration()
        # self.axis1.request_full_calibration()
        while not (self.axis0.is_idel() & self.axis1.is_idel()):
            await asyncio.sleep(1)
            print(self.axis0.get_state())
            print(self.axis1.get_state())
            self.print_voltage_current()

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


async def find_odrvs_async(timeout=3, section: str | None = None) -> list[Odrive]:
    """
    This function will find all ODrives asynchronously.
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

    odrvs = []
    for task in tasks:
        section = task.get_name()
        if task in pending:
            print(f"Warning! {section} ODrive not found")
            task.cancel()
        if task in done:
            odrv = Odrive(task.result(), section)
            odrvs.append(odrv)
            print(f"found {section} ODrive", end="")
            print(f"-> ", end="")
            odrv.check_version()

    return odrvs


def find_odrvs() -> dict[str, any]:
    """
    This function will find ODrive that serial numbers are list
    in the config.yml. This fucntion needs to be changed to match
    expected output from `find_odrvs_async` function.
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
