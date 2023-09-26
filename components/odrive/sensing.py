import odrive
import time
from odrive.enums import *

import paho.mqtt.client as mqtt


def on_connect(client, userdata, flags, rc):
    """
    Callback when connection is established.
    """
    print('-------------------------------------------------------------------')
    print(f"Connected with result code {str(rc)}")
    print(f'Client => {str(client)}')
    print(f'Userdata => {userdata}')
    print(f'Flag => {flags}')
    print('-------------------------------------------------------------------')


def on_disconnect(client, userdata, rc):
    """
    Callback when the connection is destroyed. It is not working yet.
    """
    print('-------------------------------------------------------------------')
    print(f"Disconnected with result code {str(rc)}")
    print(f'Client => {str(client)}')
    print(f'Userdata => {userdata}')
    print('-------------------------------------------------------------------')


def on_publish(client, userdata, mid):
    """
    Callback when message is sent. Good for debugging.
    """
    print('-------------------------------------------------------------------')
    print(f"Publish with result code {mid}")
    print(f'Client => {str(client)}')
    print(f'Userdata => {userdata}')
    print('-------------------------------------------------------------------')


client = mqtt.Client()  # Create connection and define callback functions
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_publish = on_publish

# Start connection with non-blocking. The process will go on
client.connect_async('66.253.158.154', 1883, 60)
client.loop_start()

attempts = 0
while not client.is_connected() and attempts < 15:  # Verify the connection
    print(f'Attempt {attempts + 1}... connection status => {client.is_connected()}')
    time.sleep(1)
    attempts += 1
    if attempts == 15:
        exit()

# Test publishing
for _ in range(10):
    client.publish('/odrive/bus_voltage', 10)
    time.sleep(1)

# Test disconnect but it is not working
client.disconnect()
attempts = 0
while client.is_connected() and attempts < 15:
    print(f'Attempt {attempts + 1}... connection status => {client.is_connected()}')
    time.sleep(1)
    attempts += 1

client.disconnect_callback()
