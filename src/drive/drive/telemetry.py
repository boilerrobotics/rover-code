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

    return telemetry
