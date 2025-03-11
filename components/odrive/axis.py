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

    def __init__(self, axis, id: int) -> None:
        self.axis = axis
        self.id = id
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

    def has_errors(self) -> bool:
        """
        Return true if there are any errors
        """
        return (
            self.axis.error != 0
            or self.motor.has_errors()
            or self.controller.has_errors()
            or self.encoder.has_errors()
        )

    def is_idle(self) -> bool:
        return self.axis.current_state == AxisState.IDLE

    def get_state(self) -> str:
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState
        return AxisState(self.axis.current_state).name

    def request_full_calibration(self) -> None:
        self.axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

    def request_close_loop_control(self) -> None:
        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL

    def request_index_search(self) -> None:
        self.axis.requested_state = AxisState.ENCODER_INDEX_SEARCH

    def request_offset_calibration(self) -> None:
        self.axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION

    def request_motor_calibration(self) -> None:
        self.axis.requested_state = AxisState.MOTOR_CALIBRATION

    def request_idle(self) -> None:
        self.axis.requested_state = AxisState.IDLE

    def set_configs(self) -> bool:
        """
        Set configurations. Return true, if reboot is needed.
        """
        return (
            True
            if (
                self.controller.set_configs()
                | self.motor.set_configs()
                | self.encoder.set_configs()
            )
            else False
        )
