import pynmea2
import serial
import math

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5.0)

earth_radius = 6371000 # meters

target_location = {
    'latitude': 13.214337,
    'longitude': 100.938345
}

# target_location = {
#     'latitude': 39.46422,
#     'longitude': -86.94771
# }

prev_location = {
    'latitude': 0.0,
    'longitude': 0.0
}

while True:
    try:
        line = ser.readline().decode("utf-8")
        if line[0:6] == "$GPRMC":
            msg = pynmea2.parse(line)
            print('--------------------')
            print('Latitude(raw): {}'.format(msg.latitude))
            print('Longitude(raw): {}'.format(msg.longitude))
            print('Speed (from GPS): {:.3f} m/s'.format(msg.spd_over_grnd*0.514444))
            print('Timestamp: {} {} UTC'.format(msg.datestamp, msg.timestamp))
            print('Current Position: {:.6f}, {:.6f}'.format(msg.latitude, msg.longitude))

            dif_long = math.radians(target_location['longitude'] - msg.longitude)
            dif_lat = math.radians(target_location['latitude'] - msg.latitude)

            lat1_radian = math.radians(msg.latitude)
            lat2_radian = math.radians(target_location['latitude'])

            a = math.sin(dif_lat/2) * math.sin(dif_lat/2) \
                + math.sin(dif_long/2) * math.sin(dif_long/2) \
                * math.cos(lat1_radian) * math.cos(lat2_radian)

            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            distance = c * earth_radius

            print('Target Position: {:.6f}, {:.6f}'.format(target_location['latitude'], target_location['longitude']))
            print('Distance from target (meters): {:,}'.format(int(distance)))

            target_direction = math.degrees(math.atan2(dif_lat, dif_long))
            print('Direction to target: {:.2f} degrees'.format(target_direction))
            
            delta_lat = msg.latitude - prev_location['latitude']
            delta_long = msg.longitude - prev_location['longitude']
            
            prev_location['latitude'] = msg.latitude
            prev_location['longitude'] = msg.longitude

    except serial.SerialException as e:
        print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        continue