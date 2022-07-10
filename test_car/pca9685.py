from board import SCL, SDA
import busio
import time

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# steering = servo.Servo(pca.channels[0], min_pulse=220, max_pulse=440)
steering = servo.Servo(pca.channels[0])
pca.channels[3].duty_cycle = int((75/100)*65536)

for i in range(40, 140, 5):
    steering.angle = i
    print(i)
    time.sleep(1)
for i in range(40, 140, 5):
    steering.angle = 180 - i
    print(180 - i)
    time.sleep(1)



steering.angle = 90

pca.deinit()
