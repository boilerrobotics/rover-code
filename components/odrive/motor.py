from enums import MotorError, Error


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
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.is_calibrated
        self.is_calibrated: bool = motor.is_calibrated

    def get_errors(self) -> str:
        """
        Return motor errors
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Error
        errors = self.decode_errors(self.motor.error)
        return " & ".join([MotorError(error).name for error in errors])

