from motor import Motor
from controller import Controller
from encoder import Encoder  # If using absolute encoder. If not, replace with SensorlessEstimator wrapper.
from enums import AxisError, AxisState, Error


class Axis(Error):
    """
    ODrive S1 axis abstraction
    """

    def __init__(self, axis, id: int) -> None:
        self.axis = axis
        self.id = id

        # S1 still exposes requested_state but internally handles more logic
        self.requested_state: int = axis.requested_state

        # Updated subsystem references
        self.motor = Motor(axis.motor)
        self.controller = Controller(axis.controller)

        # S1 note: change depending on your hardware encoder type
        # If using the standard AS5047 / absolute SPI encoder, keep this:
        self.encoder = Encoder(axis.encoder)

        # If using sensorless instead:
        # from sensorless import SensorlessEstimator
        # self.encoder = SensorlessEstimator(axis.sensorless_estimator)

    # ------------------------
    # Error Handling
    # ------------------------
    def get_errors(self) -> str:
        """
        Return axis-level errors for S1 firmware.
        """
        raw = self.axis.active_errors  # changed from .error
        errors = self.decode_errors(raw)
        return " & ".join([AxisError(error).name for error in errors])

    def has_errors(self) -> bool:
        """
        True if axis or any subsystem has errors.
        """
        return (
            self.axis.active_errors != 0
            or self.motor.has_errors()
            or self.controller.has_errors()
            or self.encoder.has_errors()
        )

    # ------------------------
    # State Handling
    # ------------------------
    def is_idle(self) -> bool:
        return self.axis.current_state == AxisState.IDLE

    def get_state(self) -> str:
        return AxisState(self.axis.current_state).name

    # ------------------------
    # State Requests (S1 Names Are Mostly the Same)
    # ------------------------
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

    # ------------------------
    # Config Write
    # ------------------------
    def set_configs(self) -> bool:
        """
        Apply configs to subsystems.
        Return True if a reboot is required (typical for S1 motor settings).
        """
        reboot_needed = (
            self.controller.set_configs()
            | self.motor.set_configs()
            | self.encoder.set_configs()
        )
        return bool(reboot_needed)