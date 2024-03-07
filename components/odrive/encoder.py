from enums import EncoderError, Error


class Encoder(Error):
    """
    ODrive encoder abstract class

    Full document:
    https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder

    This class is not included all configurations from the document
    """

    def __init__(self, encoder) -> None:
        self.encoder = encoder
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.Error
        self.error: int = encoder.error

    def get_errors(self) -> str:
        """
        Return encoder errors
        """
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.Error
        errors = self.decode_errors(self.encoder.error)
        return " & ".join([EncoderError(error).name for error in errors])

    def is_ready(self) -> str:
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.is_ready
        return self.decode_status(self.encoder.is_ready)

    def index_found(self) -> str:
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.index_found
        return self.decode_status(self.encoder.index_found)
