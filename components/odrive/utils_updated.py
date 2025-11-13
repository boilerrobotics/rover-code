import yaml
import asyncio
import odrive
from enums import ODriveError, Error
from sensing import OdriveSensing
from axis import Axis


class Odrive(Error):
    """
    Single-axis ODrive helper wrapper

    Adapted to work with one active axis (axis0).
    Based on the latest ODrive fibre types documentation:
    https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html

    This keeps the same public methods and semantics from the
    original multi-axis implementation but operates only on axis0.
    """

    def __init__(self, odrv, section) -> None:
        self.odrv = odrv
        self.section = section
        self.serial_number: str = f"{odrv.serial_number:x}".upper()
        # firmware / hardware strings
        try:
            self.hw_version = f"{odrv.hw_version_major}.{odrv.hw_version_minor} {odrv.hw_version_variant}V"
        except Exception:
            # some firmware versions expose different attributes
            self.hw_version = getattr(odrv, "hw_version", "unknown")

        try:
            self.fw_version = f"{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}"
        except Exception:
            self.fw_version = getattr(odrv, "fw_version", "unknown")

        # single axis (axis0)
        self.axis = Axis(odrv.axis0, 0)
        # keep compatibility: older code may reference axis0/axis1
        self.axis0 = self.axis
        self.config = self.Config(odrv.config)

    def clear_errors(self) -> None:
        """Clear ODrive system errors."""
        self.odrv.clear_errors()

    def get_errors(self) -> str:
        """Return human-readable system errors."""
        errors = self.decode_errors(self.odrv.error)
        return " & ".join([ODriveError(error).name for error in errors])

    def has_errors(self) -> bool:
        """Return True if the axis or its components report errors."""
        return self.axis.has_errors()

    def print_voltage_current(self, connection: OdriveSensing | None = None) -> None:
        """Print voltage/current for debugging and optionally publish them via connection."""
        vbus_voltage: float = self.odrv.vbus_voltage
        ibus: float = self.odrv.ibus
        print(f"  voltage = {vbus_voltage:5.2f} V  current = {ibus:6.4f} A")
        if connection:
            connection.publish("brc/voltage", f"{vbus_voltage:5.2f}")
            connection.publish("brc/current", f"{ibus:7.5f}")

    def get_voltage(self) -> float:
        return float(self.odrv.vbus_voltage)

    def get_current(self) -> float:
        return float(self.odrv.ibus)

    def print_errors(self, component, name: str) -> bool:
        """Print part of the errors for `component`. Return True if any were printed."""
        if errors := component.get_errors():
            print(f"  ERROR: {name} has {errors}")
            return True
        return False

    def check_and_print_errors(self) -> bool:
        """Check and print errors for the single axis and its subcomponents."""
        has_errors = []
        has_errors.append(self.print_errors(self.axis, "axis 0"))
        has_errors.append(self.print_errors(self.axis.motor, "motor 0"))
        has_errors.append(self.print_errors(self.axis.controller, "controller 0"))
        has_errors.append(self.print_errors(self.axis.encoder, "encoder 0"))

        return any(has_errors)

    def check_errors(self) -> None:
        """Verbose status dump for debugging (single axis)."""
        print("-" * 100)
        print(f"{self.section} odrive status check...")
        self.print_voltage_current()
        # system-level errors
        self.print_errors(self, "system errors")

        print(f"Axis 0 current state: {self.axis.get_state()}")
        print(f"Motor is{'' if self.axis.motor.is_calibrated() else ' not'} calibrated")
        print(f"Encoder is{'' if self.axis.encoder.is_ready() else ' not'} ready")

        self.print_errors(self.axis, "axis 0")
        self.print_errors(self.axis.motor, "motor 0")
        self.print_errors(self.axis.controller, "controller 0")
        self.print_errors(self.axis.encoder, "encoder 0")

    def check_version(self) -> None:
        """Print firmware and hardware version information."""
        print(
            f"Firmware version is {self.fw_version}"
            f'{' ' * 3} Hardware version is {self.hw_version}'
        )

    async def reboot(self, save_config: bool = False) -> None:
        """
        Reboot ODrive then try to reconnect within 30 seconds.
        If save_config is True, the configuration is saved before reboot.
        """
        try:
            if save_config:
                self.odrv.save_configuration()
            else:
                self.odrv.reboot()
        except Exception:
            print(f"rebooting {self.section} odrive...")
        finally:
            try:
                odrv = await asyncio.wait_for(
                    odrive.find_any_async(serial_number=self.serial_number), timeout=30
                )
                self.odrv = odrv
                self.axis = Axis(odrv.axis0, 0)
                self.axis0 = self.axis
            except asyncio.TimeoutError:
                print("Timeout: cannot reconnect with Odrive")
            except Exception as e:
                print(f"Unknown error: {e}")
            else:
                print(f"{self.section} odrive is online...")

    def set_configs(self) -> bool:
        """
        Apply configuration for motor/controller/encoder.
        Return True if a reboot is required (kept the same boolean contract).
        """
        print("Applying configurations...")
        return bool(self.axis.set_configs())

    async def calibrate(self) -> None:
        """Run full calibration sequence for the single axis."""
        self.clear_errors()
        self.check_errors()

        name = f"Odrive {self.section} axis {self.axis.id}"
        if self.axis.motor.is_calibrated() and self.axis.encoder.is_ready():
            print(f"{name} is calibrated and ready. Calibration is not needed")
            return
        if self.axis.has_errors():
            print(f"{name} has error(s). Abort calibration")
            return

        self.axis.request_full_calibration()
        await asyncio.sleep(1)
        while not self.axis.is_idle():
            await asyncio.sleep(1)
            self.check_errors()

        if self.axis.motor.is_calibrated() and self.axis.encoder.is_ready():
            print(f"{name} is calibrated and ready. Calibration is completed")
            self.axis.encoder.use_pre_calibrated()
            self.axis.motor.use_pre_calibrated()

    async def test_run(self, speed: float, duration: int) -> None:
        """Run a test cycle on the single axis (both directions)."""
        self.check_errors()
        await self.test_run_cycle(self.axis, speed, duration)
        self.check_errors()

    async def test_run_cycle(self, axis: Axis, speed: float, duration: int) -> None:
        """Helper: run one axis through speed -> -speed -> stop sequence."""
        axis.request_close_loop_control()
        axis.controller.set_speed(speed)
        await asyncio.sleep(duration)
        axis.controller.set_speed(-speed)
        await asyncio.sleep(duration)
        axis.controller.stop()
        await asyncio.sleep(duration)
        axis.request_idle()

    class Config:
        """
        ODrive configuration helper (keeps previous configuration API).
        """

        def __init__(self, config) -> None:
            self.config = config
            self.brake_resistance = 0.5
            self.enable_brake_resistor = True

            # Keep compatibility: configure GPIO pins (if available)
            self.GPIO_MODE_DIGITAL = 0
            for pin in (9, 10, 11, 12, 13, 14):
                try:
                    setattr(self.config, f"gpio{pin}_mode", self.GPIO_MODE_DIGITAL)
                except Exception:
                    # some firmware / boards may not expose all gpio pins
                    pass

        def set_break_resistor(self, brake_resistance: int | None = None) -> bool:
            """
            Enable/adjust the brake resistor. Returns True when a change was made
            that may require a reboot; False when nothing changed.
            """
            if brake_resistance is not None:
                self.brake_resistance = brake_resistance
            self.config.brake_resistance = self.brake_resistance

            if getattr(self.config, "enable_brake_resistor", False):
                return False
            else:
                self.config.enable_brake_resistor = self.enable_brake_resistor
                return True


async def find_odrvs_async(
    config_file="config.yml", timeout=3, section: str | None = None
) -> list[Odrive]:
    """Find ODrives asynchronously from config file (returns list of Odrive wrappers).

    The config file is expected to have a `serial` mapping: { section_name: serial }
    """
    with open(config_file) as fp:
        config = yaml.safe_load(fp)

    tasks = []
    for section_name, serial in config.get("serial", {}).items():
        tasks.append(
            asyncio.create_task(odrive.find_any_async(serial_number=serial), name=section_name)
        )

    print("finding ODrives...")
    done, pending = await asyncio.wait(
        tasks, timeout=timeout, return_when=asyncio.ALL_COMPLETED
    )

    odrvs = []
    for task in tasks:
        section_name = task.get_name()
        if task in pending:
            print(f"Warning! {section_name} ODrive not found")
            task.cancel()
            continue
        if task in done:
            odrv_inst = Odrive(task.result(), section_name)
            odrvs.append(odrv_inst)
            print(f"found {section_name} ODrive -> ", end="")
            odrv_inst.check_version()

    return odrvs


def find_odrvs(config_file="config.yml") -> list[Odrive]:
    """Synchronous finder that scans the serial numbers listed in config.yml."""
    with open(config_file) as fp:
        config = yaml.safe_load(fp)

    print("finding ODrives...")
    odrvs = []
    for section, serial in config.get("serial", {}).items():
        print(f"searching for serial number {serial}...")
        try:
            odrv = odrive.find_any(serial_number=serial, timeout=2)
            odrv = Odrive(odrv, section)
            odrvs.append(odrv)
            print(f"-> assign odrive {serial} to {section} section")
            print(f"-> ", end="")
            odrv.check_version()
        except TimeoutError:
            print(f"error: Cannot find serial {serial} !!")
    print("--------------------------------------")

    return odrvs
