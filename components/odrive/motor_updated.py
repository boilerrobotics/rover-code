from enums import MotorError, Error, MotorType


class Motor(Error):
    """
    ODrive S1 motor abstraction
    """

    def __init__(self, motor) -> None:
        self.motor = motor
        # S1: motor.error -> motor.active_errors
        self.error: int = motor.active_errors

    # ---------------------------------------------------------
    # Error Handling
    # ---------------------------------------------------------
    def get_errors(self) -> str:
        """
        Return motor error string (S1 uses active_errors)
        """
        errors = self.decode_errors(self.motor.active_errors)
        return " & ".join([MotorError(error).name for error in errors])

    def has_errors(self) -> bool:
        """
        Return true if any errors exist
        """
        return self.motor.active_errors != 0

    # ---------------------------------------------------------
    # Calibration
    # ---------------------------------------------------------
    def is_calibrated(self) -> bool:
        """
        S1: some motors have a .is_calibrated() function.
        If not present, fallback to pre_calibrated.
        """
        if hasattr(self.motor, "is_calibrated"):
            return self.motor.is_calibrated()
        return self.motor.config.pre_calibrated

    def use_pre_calibrated(self) -> None:
        self.motor.config.pre_calibrated = True

    # ---------------------------------------------------------
    # Configuration
    # ---------------------------------------------------------
    def set_configs(self) -> bool:
        """
        Apply motor configuration for ODrive S1.
        Return True if a reboot is required.
        """
        cfg = self.motor.config

        # Core motor settings (unchanged in S1)
        cfg.pole_pairs = 7
        cfg.motor_type = MotorType.HIGH_CURRENT
        cfg.current_control_bandwidth = 100.0

        # Motor torque constant
        cfg.torque_constant = 8.27 / 270  # (Nm/A) = 8.27 / KV

        # Set current limits used during calibration + running
        self.set_current_limit(20, 8)

        # Most motor config changes on S1 require a reboot
        return True

    # ---------------------------------------------------------
    # Current Limit
    # ---------------------------------------------------------
    def set_current_limit(
        self, current_limit: float, current_limit_margin: float
    ) -> bool:
        """
        Set current limits (identical field names in S1).
        """
        cfg = self.motor.config
        cfg.current_lim = current_limit
        cfg.current_lim_margin = current_limit_margin
        return False
