import odrive
from odrive.enums import *

import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {str(rc)}")
    print(f'Client => {client}')
    print(f'Userdata => {userdata}')
    print(f'Flag => {flags}')


    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.

client = mqtt.Client()
client.on_connect = on_connect
client.connect("localhost", 1883, 60)

print("finding odrives...")
odrv = odrive.find_any()
print(odrv.vbus_voltage)

client.loop_forever()
