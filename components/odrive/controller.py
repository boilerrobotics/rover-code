from enums import ControllerError, ControlMode, Error


class Controller(Error):
    """
    ODrive controller abstract class

    Full document:
    https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller

    This class is not included all configurations from the document
    """

    def __init__(self, controller) -> None:
        self.controller = controller
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.Error
        self.error: int = controller.error
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.input_vel
        self.input_vel: float = controller.input_vel
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.mechanical_power
        self.mechanical: float = controller.mechanical_power
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.electrical_power
        self.electrical_power: float = controller.electrical_power

    def get_errors(self) -> str:
        """
        Return controller errors
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.Error
        errors = self.decode_errors(self.controller.error)
        return " & ".join([ControllerError(error).name for error in errors])

    def set_configs(self) -> None:
        self.config.control_mode = ControlMode.VELOCITY_CONTROL
