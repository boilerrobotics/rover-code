"""
Utility functions for ODrive calibration and test.
"""

import time
from sensing import OdriveSensing


def setup_telemetry(max_attempt=15) -> OdriveSensing | bool:
    """
    Set up a MQTT connection for telemetry.
    """
    connection = OdriveSensing()
    attempts = 0
    while attempts < max_attempt and not connection.is_connected():
        print(
            f"Attempt {attempts + 1}... connection status => {connection.is_connected()}"
        )
        time.sleep(1)
        attempts += 1
        if attempts == 15:
            return False
    print(f"Telemetry connection is established")
    return connection
