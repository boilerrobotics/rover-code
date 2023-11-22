from typing import Any

from shared_msgs.msg import OdriveTelemetry, OdriveAxisTelemetry


def get_odrive_telemetry(drive: Any | None) -> OdriveTelemetry:
    telemetry = OdriveTelemetry()

    if drive is None:
        return telemetry

    telemetry.voltage = drive.vbus_voltage
    telemetry.current = drive.ibus
    telemetry.misconfigured = drive.misconfigured

    telemetry.axis0 = get_axis_telemetry(drive.axis0)
    telemetry.axis1 = get_axis_telemetry(drive.axis1)

    return telemetry


def get_axis_telemetry(axis) -> OdriveAxisTelemetry:
    telemetry = OdriveAxisTelemetry()

    telemetry.input_vel = axis.controller.input_vel
    telemetry.vel_setpoint = axis.controller.vel_setpoint

    telemetry.vel_estimate = axis.encoder.vel_estimate

    telemetry.errors = odrive_errors_to_names(axis.active_errors)
    return telemetry


# https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error
# https://stackoverflow.com/questions/9298865/get-n-th-bit-of-an-integer
def odrive_errors_to_names(e: int) -> list[str]:
    codes = [
        (0, "INITIALIZING"),
        (1, "SYSTEM_LEVEL"),
        (2, "TIMING_ERROR"),
        (3, "MISSING_ESTIMATE"),
        (4, "BAD_CONFIG"),
        (5, "DRV_FAULT"),
        (6, "MISSING_INPUT"),
        (8, "DC_BUS_OVER_VOLTAGE"),
        (9, "DC_BUS_UNDER_VOLTAGE"),
        (10, "DC_BUS_OVER_CURRENT"),
        (11, "DC_BUS_OVER_REGEN_CURRENT"),
        (12, "CURRENT_LIMIT_VIOLATION"),
        (13, "MOTOR_OVER_TEMP"),
        (14, "INVERTER_OVER_TEMP"),
        (15, "VELOCITY_LIMIT_VIOLATION"),
        (16, "POSITION_LIMIT_VIOLATION"),
        (24, "WATCHDOG_TIMER_EXPIRED"),
        (25, "ESTOP_REQUESTED"),
        (26, "SPINOUT_DETECTED"),
        (27, "BRAKE_RESISTOR_DISARMED"),
        (28, "THERMISTOR_DISCONNECTED"),
        (30, "CALIBRATION_ERROR")
    ]
    return [name for (bit, name) in codes if (e >> bit) & 1]
