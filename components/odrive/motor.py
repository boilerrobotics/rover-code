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
