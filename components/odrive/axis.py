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
