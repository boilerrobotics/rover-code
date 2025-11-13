import math
from enums import EncoderError, Error, EncoderMode


class Encoder(Error):
    """
    ODrive S1 encoder abstraction.
    """

    def __init__(self, encoder) -> None:
        self.encoder = encoder

    # ---------------------------------------------------------
    # Error Handling
    # ---------------------------------------------------------
    def get_errors(self) -> str:
        """
        Return encoder error string (S1 uses active_errors)
        """
        errors = self.decode_errors(self.encoder.active_errors)
        return " & ".join([EncoderError(err).name for err in errors])

    def has_errors(self) -> bool:
        return self.encoder.active_errors != 0

    # ---------------------------------------------------------
    # Velocity / Ready / Index
    # ---------------------------------------------------------
    def get_vel(self) -> float:
        return self.encoder.vel_estimate

    def is_ready(self) -> bool:
        return self.encoder.is_ready

    def index_found(self) -> bool:
        return self.encoder.index_found

    # ---------------------------------------------------------
    # Calibration
    # ---------------------------------------------------------
    def use_pre_calibrated(self) -> None:
        self.encoder.config.pre_calibrated = True

    # ---------------------------------------------------------
    # Configuration
    # ---------------------------------------------------------
    def set_configs(self) -> bool:
        """
        Apply encoder configuration.
        Returns True if reboot/reset is needed (e.g., CPR changed).
        """
        need_reset = False
        cfg = self.encoder.config

        # Encoder mode: Hall, Incremental, or Absolute
        cfg.mode = EncoderMode.HALL

        # Counts per revolution
        cpr = 42
        if cfg.cpr != cpr:
            need_reset = True
            cfg.cpr = cpr

        # Calibration scan distance for full rotation
        cfg.calib_scan_distance = (2 * math.pi) * 10  # S1 still expects radians

        # Bandwidth
        cfg.bandwidth = 100

        return need_reset
