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

uart = serial.Serial('COM5', baudrate=9600, timeout=3000)
gps = adafruit_gps.GPS(uart)

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

    time.sleep(1)

# import pynmea2
# import serial
# import math

# ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5.0)

# earth_radius = 6371000 # meters

# p_constant = 10

# target_location = {
#     'latitude': 13.214337,
#     'longitude': 100.938345
# }

# # target_location = {
# #     'latitude': 39.46422,
# #     'longitude': -86.94771
# # }

# prev_location = {
#     'latitude': 0.0,
#     'longitude': 0.0
# }

# while True:
#     try:
#         line = ser.readline().decode('utf-8')
#         if line[0:6] == '$GPRMC':
#             msg = pynmea2.parse(line)
#             print('--------------------')
#             print('Latitude(raw): {}'.format(msg.latitude))
#             print('Longitude(raw): {}'.format(msg.longitude))
#             print('Speed (from GPS): {:.3f} m/s'.format(msg.spd_over_grnd*0.514444))
#             print('Timestamp: {} {} UTC'.format(msg.datestamp, msg.timestamp))
#             print('Current Position: {:.6f}, {:.6f}'.format(msg.latitude, msg.longitude))

#             dif_long = math.radians(target_location['longitude'] - msg.longitude)
#             dif_lat = math.radians(target_location['latitude'] - msg.latitude)

#             lat1_radian = math.radians(msg.latitude)
#             lat2_radian = math.radians(target_location['latitude'])

#             a = math.sin(dif_lat/2) * math.sin(dif_lat/2) \
#                 + math.sin(dif_long/2) * math.sin(dif_long/2) \
#                 * math.cos(lat1_radian) * math.cos(lat2_radian)

#             c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
#             distance = c * earth_radius

#             print('Target Position: {:.6f}, {:.6f}'.format(target_location['latitude'], target_location['longitude']))
#             print('Distance from target (meters): {:,}'.format(int(distance)))

#             target_direction = math.degrees(math.atan2(dif_lat, dif_long))
#             print('Direction to target: {:.2f} degrees'.format(target_direction))
            
#             delta_lat = msg.latitude - prev_location['latitude']
#             delta_long = msg.longitude - prev_location['longitude']

#             current_direction = math.degrees(math.atan2(delta_lat, delta_long))
#             print('Currect Direction: {:.2f} degrees'.format(current_direction))

#             direction_error = target_direction - current_direction
#             speed_different = direction_error/180 * p_constant
            
#             prev_location['latitude'] = msg.latitude
#             prev_location['longitude'] = msg.longitude

#     except serial.SerialException as err:
#         print('Device error: {}'.format(err))
#         break
#     except pynmea2.ParseError as err:
#         print('Parse error: {}'.format(err))
#         continue