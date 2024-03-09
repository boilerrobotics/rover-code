from enums import MotorError, Error, MotorType


class Motor(Error):
    """
    ODrive motor abstract class

    Full document:
    https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor

    This class is not included all configurations from the document
    """

    def __init__(self, motor) -> None:
        self.motor = motor
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Error
        self.error: int = motor.error

    def get_errors(self) -> str:
        """
        Return motor errors
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Error
        errors = self.decode_errors(self.motor.error)
        return " & ".join([MotorError(error).name for error in errors])

    def is_calibrated(self) -> str:
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.is_calibrated
        return self.decode_status(self.motor.is_calibrated)

    def set_configs(self) -> None:
        """
        Full document: https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Config
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Config.pole_pairs
        self.motor.config.pole_pairs = 7
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Config.calibration_current
        self.motor.config.calibration_current = 30.0
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Config.resistance_calib_max_voltage
        self.motor.config.resistance_calib_max_voltage = 10.0
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.MotorType
        self.motor.config.motor_type = MotorType.HIGH_CURRENT
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Config.requested_current_range
        self.motor.config.requested_current_range = 30.0
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Config.current_control_bandwidth
        self.motor.config.current_control_bandwidth = 100.0
        # https://docs.odriverobotics.com/v/0.5.4/getting-started.html#torque-constant
        self.motor.config.torque_constant = 8.27 / 270
        # set current limit to high value during calibration
        self.set_current_limit(40, 8)

    def set_current_limit(
        self, current_limit: float, current_limit_margin: float
    ) -> None:
        """
        Set current limit and current limit margin
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Config.current_lim
        self.motor.config.current_lim = current_limit
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Config.current_lim_margin
        self.motor.config.current_lim_margin = current_limit_margin
