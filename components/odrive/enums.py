# This is enum.py from library version 0.5.4

import enum


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.GpioMode
class GpioMode(enum.IntEnum):
    DIGITAL                                         = 0
    DIGITAL_PULL_UP                                 = 1
    DIGITAL_PULL_DOWN                               = 2
    ANALOG_IN                                       = 3
    UART_A                                          = 4
    UART_B                                          = 5
    UART_C                                          = 6
    CAN_A                                           = 7
    I2C_A                                           = 8
    SPI_A                                           = 9
    PWM                                             = 10
    ENC0                                            = 11
    ENC1                                            = 12
    ENC2                                            = 13
    MECH_BRAKE                                      = 14
    STATUS                                          = 15
    BRAKE_RES                                       = 16
    AUTO                                            = 17


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.StreamProtocolType
class StreamProtocolType(enum.IntEnum):
    FIBRE                                           = 0
    ASCII                                           = 1
    STDOUT                                          = 2
    ASCII_AND_STDOUT                                = 3
    OTHER                                           = 4


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Can.Protocol
class Protocol(enum.IntFlag):
    NONE                                            = 0x00000000
    SIMPLE                                          = 0x00000001


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState
class AxisState(enum.IntEnum):
    UNDEFINED                                       = 0
    IDLE                                            = 1
    STARTUP_SEQUENCE                                = 2
    FULL_CALIBRATION_SEQUENCE                       = 3
    MOTOR_CALIBRATION                               = 4
    ENCODER_INDEX_SEARCH                            = 6
    ENCODER_OFFSET_CALIBRATION                      = 7
    CLOSED_LOOP_CONTROL                             = 8
    LOCKIN_SPIN                                     = 9
    ENCODER_DIR_FIND                                = 10
    HOMING                                          = 11
    ENCODER_HALL_POLARITY_CALIBRATION               = 12
    ENCODER_HALL_PHASE_CALIBRATION                  = 13
    ANTICOGGING_CALIBRATION                         = 14


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.Mode
class EncoderMode(enum.IntEnum):
    INCREMENTAL                                     = 0
    HALL                                            = 1
    SINCOS                                          = 2
    SPI_ABS_CUI                                     = 256
    SPI_ABS_AMS                                     = 257
    SPI_ABS_AEAT                                    = 258
    SPI_ABS_RLS                                     = 259
    SPI_ABS_MA732                                   = 260


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
class ControlMode(enum.IntEnum):
    VOLTAGE_CONTROL                                 = 0
    TORQUE_CONTROL                                  = 1
    VELOCITY_CONTROL                                = 2
    POSITION_CONTROL                                = 3


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode
class InputMode(enum.IntEnum):
    INACTIVE                                        = 0
    PASSTHROUGH                                     = 1
    VEL_RAMP                                        = 2
    POS_FILTER                                      = 3
    MIX_CHANNELS                                    = 4
    TRAP_TRAJ                                       = 5
    TORQUE_RAMP                                     = 6
    MIRROR                                          = 7
    TUNING                                          = 8


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.MotorType
class MotorType(enum.IntEnum):
    HIGH_CURRENT                                    = 0
    GIMBAL                                          = 2
    ACIM                                            = 3


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error
class ODriveError(enum.IntFlag):
    NONE                                            = 0x00000000
    CONTROL_ITERATION_MISSED                        = 0x00000001
    DC_BUS_UNDER_VOLTAGE                            = 0x00000002
    DC_BUS_OVER_VOLTAGE                             = 0x00000004
    DC_BUS_OVER_REGEN_CURRENT                       = 0x00000008
    DC_BUS_OVER_CURRENT                             = 0x00000010
    BRAKE_DEADTIME_VIOLATION                        = 0x00000020
    BRAKE_DUTY_CYCLE_NAN                            = 0x00000040
    INVALID_BRAKE_RESISTANCE                        = 0x00000080


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Can.Error
class CanError(enum.IntFlag):
    NONE                                            = 0x00000000
    DUPLICATE_CAN_IDS                               = 0x00000001
    BUS_OFF                                         = 0x00000002
    LOW_LEVEL                                       = 0x00000004
    PROTOCOL_INIT                                   = 0x00000008


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.Error
class AxisError(enum.IntFlag):
    NONE                                            = 0x00000000
    INVALID_STATE                                   = 0x00000001
    WATCHDOG_TIMER_EXPIRED                          = 0x00000800
    MIN_ENDSTOP_PRESSED                             = 0x00001000
    MAX_ENDSTOP_PRESSED                             = 0x00002000
    ESTOP_REQUESTED                                 = 0x00004000
    HOMING_WITHOUT_ENDSTOP                          = 0x00020000
    OVER_TEMP                                       = 0x00040000
    UNKNOWN_POSITION                                = 0x00080000


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Error
class MotorError(enum.IntFlag):
    NONE                                            = 0x00000000
    PHASE_RESISTANCE_OUT_OF_RANGE                   = 0x00000001
    PHASE_INDUCTANCE_OUT_OF_RANGE                   = 0x00000002
    DRV_FAULT                                       = 0x00000008
    CONTROL_DEADLINE_MISSED                         = 0x00000010
    MODULATION_MAGNITUDE                            = 0x00000080
    CURRENT_SENSE_SATURATION                        = 0x00000400
    CURRENT_LIMIT_VIOLATION                         = 0x00001000
    MODULATION_IS_NAN                               = 0x00010000
    MOTOR_THERMISTOR_OVER_TEMP                      = 0x00020000
    FET_THERMISTOR_OVER_TEMP                        = 0x00040000
    TIMER_UPDATE_MISSED                             = 0x00080000
    CURRENT_MEASUREMENT_UNAVAILABLE                 = 0x00100000
    CONTROLLER_FAILED                               = 0x00200000
    I_BUS_OUT_OF_RANGE                              = 0x00400000
    BRAKE_RESISTOR_DISARMED                         = 0x00800000
    SYSTEM_LEVEL                                    = 0x01000000
    BAD_TIMING                                      = 0x02000000
    UNKNOWN_PHASE_ESTIMATE                          = 0x04000000
    UNKNOWN_PHASE_VEL                               = 0x08000000
    UNKNOWN_TORQUE                                  = 0x10000000
    UNKNOWN_CURRENT_COMMAND                         = 0x20000000
    UNKNOWN_CURRENT_MEASUREMENT                     = 0x40000000
    UNKNOWN_VBUS_VOLTAGE                            = 0x80000000
    UNKNOWN_VOLTAGE_COMMAND                         = 0x100000000
    UNKNOWN_GAINS                                   = 0x200000000
    CONTROLLER_INITIALIZING                         = 0x400000000
    UNBALANCED_PHASES                               = 0x800000000


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.Error
class ControllerError(enum.IntFlag):
    NONE                                            = 0x00000000
    OVERSPEED                                       = 0x00000001
    INVALID_INPUT_MODE                              = 0x00000002
    UNSTABLE_GAIN                                   = 0x00000004
    INVALID_MIRROR_AXIS                             = 0x00000008
    INVALID_LOAD_ENCODER                            = 0x00000010
    INVALID_ESTIMATE                                = 0x00000020
    INVALID_CIRCULAR_RANGE                          = 0x00000040
    SPINOUT_DETECTED                                = 0x0000008


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.Error
class EncoderError(enum.IntFlag):
    NONE                                            = 0x00000000
    UNSTABLE_GAIN                                   = 0x00000001
    CPR_POLEPAIRS_MISMATCH                          = 0x00000002
    NO_RESPONSE                                     = 0x00000004
    UNSUPPORTED_ENCODER_MODE                        = 0x00000008
    ILLEGAL_HALL_STATE                              = 0x00000010
    INDEX_NOT_FOUND_YET                             = 0x00000020
    ABS_SPI_TIMEOUT                                 = 0x00000040
    ABS_SPI_COM_FAIL                                = 0x00000080
    ABS_SPI_NOT_READY                               = 0x00000100
    HALL_NOT_CALIBRATED_YET                         = 0x00000200


# https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.SensorlessEstimator.Error
class SensorlessEstimatorError(enum.IntFlag):
    NONE                                            = 0x00000000
    UNSTABLE_GAIN                                   = 0x00000001
    UNKNOWN_CURRENT_MEASUREMENT                     = 0x00000002
