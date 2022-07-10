'''
===============================================================================
Program Description
	This program is written to test the function for reading
    Adafruit Ultimate GPS

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         October 23, 2021
Version:        0.1.0
===============================================================================
'''

import adafruit_gps
import serial
import time
import math

uart = serial.Serial('COM5', baudrate=9600, timeout=3000)
gps = adafruit_gps.GPS(uart)

earth_radius = 6371000 # meters

# target_location = {
#     'latitude': 13.214337,
#     'longitude': 100.938345
# }

target_location = {
    'latitude': 40.46422,
    'longitude': -86.94771
}

'''
the GPS module behavior:
  https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
'''

# Turn on the basic GGA and RMC info
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Set update rate to once a second (1hz).
gps.send_command(b'PMTK220,1000')

while True:
    print('=' * 40)  # Print a separator line.
    gps.update()
    if not gps.has_fix:
        # Try again if we don't have a fix yet.
        print('Waiting for fix...')
    
    if gps.fix_quality == 1:
        print(gps.timestamp_utc)
        print(f'Latitude: {gps.latitude:.6f} degrees')
        print(f'Longitude: {gps.longitude:.6f} degrees')
        if gps.satellites is not None:
            print(f'# satellites: {gps.satellites}')
        if gps.altitude_m is not None:
            print(f'Altitude: {gps.altitude_m} meters')
        if gps.speed_knots is not None:
            print(f'Speed: {gps.speed_knots} knots')
        if gps.track_angle_deg is not None:
            print(f'Track angle: {gps.track_angle_deg} degrees')
        if gps.horizontal_dilution is not None:
            print(f'Horizontal dilution: {gps.horizontal_dilution}')
        if gps.height_geoid is not None:
            print(f'Height geoid: {gps.height_geoid} meters')

        dif_long = math.radians(target_location['longitude'] - gps.longitude)
        dif_lat = math.radians(target_location['latitude'] - gps.latitude)

        lat1_radian = math.radians(gps.latitude)
        lat2_radian = math.radians(target_location['latitude'])

        a = math.sin(dif_lat/2) * math.sin(dif_lat/2) \
            + math.sin(dif_long/2) * math.sin(dif_long/2) \
            * math.cos(lat1_radian) * math.cos(lat2_radian)

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = c * earth_radius

        print(f'Target Position: {target_location["latitude"]:.6f}, {target_location["longitude"]:.6f}')
        print(f'Distance from target (meters): {distance:,.0f}')

        target_direction = math.degrees(math.atan2(dif_lat, dif_long))
        print(f'Direction to target: {target_direction:.2f} degrees')

    time.sleep(1)
    