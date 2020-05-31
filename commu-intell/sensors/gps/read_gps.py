import pynmea2
import serial


ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5.0)

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
            
    except serial.SerialException as e:
        print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        continue