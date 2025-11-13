from enums import ControllerError, ControlMode, InputMode, Error


class Controller(Error):
    """
    ODrive S1 controller abstraction.
    """

    def __init__(self, controller) -> None:
        self.controller = controller

        # S1: input_vel still valid
        self.input_vel: float = controller.input_vel

        # Power measurements (unchanged)
        self.mechanical: float = controller.mechanical_power
        self.electrical_power: float = controller.electrical_power

    # ---------------------------------------------------------
    # Error Handling
    # ---------------------------------------------------------
    def get_errors(self) -> str:
        errors = self.decode_errors(self.controller.active_errors)
        return " & ".join([ControllerError(err).name for err in errors])

    def has_errors(self) -> bool:
        return self.controller.active_errors != 0

    # ---------------------------------------------------------
    # Velocity Control
    # ---------------------------------------------------------
    def set_speed(self, speed: float) -> None:
        """
        Set velocity in turns per second.
        """
        self.controller.input_vel = speed

    def stop(self) -> None:
        self.set_speed(0)

    # ---------------------------------------------------------
    # Limits
    # ---------------------------------------------------------
    def get_speed_limit(self) -> float:
        return self.controller.config.vel_limit

    def set_speed_limit(self, speed: float) -> None:
        self.controller.config.vel_limit = speed

    # ---------------------------------------------------------
    # Config Setup
    # ---------------------------------------------------------
    def set_configs(self, vel_limit=10) -> bool:
        """
        Configure the controller for S1 velocity mode.
        """

        cfg = self.controller.config

        # Control loop type
        cfg.control_mode = ControlMode.VELOCITY_CONTROL

        # Speed limit
        cfg.vel_limit = vel_limit

        # Input mode behavior
        cfg.input_mode = InputMode.VEL_RAMP

        # How fast velocity can ramp
        cfg.vel_ramp_rate = 3.0

        # Controller config changes do NOT require reboot on S1
        return False
