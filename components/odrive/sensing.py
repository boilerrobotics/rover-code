import time
import paho.mqtt.client as mqtt


class OdriveSensing:
    def __init__(self) -> None:
        self.client: mqtt.Client = (
            mqtt.Client()
        )  # Create connection and define callback functions
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        # client.on_publish = on_publish
        self.client.connect_async("66.253.158.154", 1883, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback when connection is established.
        """
        print("-------------------------------------------------------------------")
        print(f"Connected with result code {str(rc)}")
        print(f"Client => {str(client)}")
        print(f"Userdata => {userdata}")
        print(f"Flag => {flags}")
        print("-------------------------------------------------------------------")

    def on_disconnect(self, client, userdata, rc):
        """
        Callback when the connection is destroyed. It is not working yet.
        """
        print("-------------------------------------------------------------------")
        print(f"Disconnected with result code {str(rc)}")
        print(f"Client => {str(client)}")
        print(f"Userdata => {userdata}")
        print("-------------------------------------------------------------------")

    def on_publish(self, client, userdata, mid):
        """
        Callback when message is sent. Good for debugging.
        """
        print("-------------------------------------------------------------------")
        print(f"Publish with result code {mid}")
        print(f"Client => {str(client)}")
        print(f"Userdata => {userdata}")
        print("-------------------------------------------------------------------")

    def is_connected(self):
        return self.client.is_connected()

    def publish(self, topic, message):
        return self.client.publish(topic, message)


if __name__ == "__main__":
    client = OdriveSensing()
    attempts = 0
    while not client.is_connected() and attempts < 15:  # Verify the connection
        print(f"Attempt {attempts + 1}... connection status => {client.is_connected()}")
        time.sleep(1)
        attempts += 1
        if attempts == 15:
            exit()

    # Test publishing
    for _ in range(10):
        client.publish("brc/odrive/bus_voltage", 10)
        time.sleep(1)

    # Test disconnect but it is not working
    # client.disconnect()
    # attempts = 0
    # while client.is_connected() and attempts < 15:
    #     print(f'Attempt {attempts + 1}... connection status => {client.is_connected()}')
    #     time.sleep(1)
    #     attempts += 1
