from motor import Motor
from controller import Controller
from encoder import Encoder
from enums import AxisError, AxisState, Error


class Axis(Error):
    """
    ODrive axis abstract class

    Full document:
    https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis

    This class is not included all configurations from the document
    """

    def __init__(self, axis) -> None:
        self.axis = axis
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.requested_state
        self.requested_state: int = axis.requested_state
        # see motor.py
        self.motor = Motor(axis.motor)
        # see controller.py
        self.controller = Controller(axis.controller)
        # see encoder.py
        self.encoder = Encoder(axis.encoder)

    def get_errors(self) -> str:
        """
        Return axis errors
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.Error
        errors = self.decode_errors(self.axis.error)
        return " & ".join([AxisError(error).name for error in errors])

    def is_idel(self) -> bool:
        return self.axis.current_state == AxisState.IDLE

    def get_state(self) -> str:
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState
        return AxisState(self.axis.current_state).name

    def request_full_calibration(self) -> str:
        self.axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

    # def set_configs(self) -> None:
