from motor import Motor
from controller import Controller
from encoder import Encoder
from enums import AxisError

class Axis:
    """
    ODrive axis abstract class

    Full document:
    https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis

    This class is not included all configurations from the document
    """

    def __init__(self, axis) -> None:
        self.axis = axis
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.Error
        self.error: int = axis.error
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState
        self.current_state: int = axis.current_state
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.requested_state
        self.requested_state: int = axis.requested_state
        # see motor.py
        self.motor: Motor = axis.motor
        # see controller.py
        self.controller: Controller = axis.controller
        # see encoder.py
        self.encoder: Encoder = axis.encoder

    def get_error(self) -> str:
        """
        Return axis error
        """
        return AxisError(self.error).name