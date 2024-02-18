from enums import EncoderError


class Encoder:
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
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.is_ready
        self.is_ready: bool = encoder.is_ready
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.index_found
        self.index_found: bool = encoder.index_found


    def get_error(self) -> str:
        """
        Return encoder error
        """
        return EncoderError(self.error).name
