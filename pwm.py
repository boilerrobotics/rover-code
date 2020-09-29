import board
import busio
import adafruit_pca9685
import getch

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

# Set frequency
pca.frequency = 1600

# Assign forward wheels to channels
front_left_f = pca.channel[0]
mid_left_f = pca.channel[1]
back_left_f = pca.channel[2]
front_right_f = pca.channel[3]
mid_right_f = pca.channel[4]
back_right_f = pca.channel[5]

# Assign reverse wheels to channels
front_left_r = pca.channel[6]
mid_left_r = pca.channel[7]
back_left_r = pca.channel[8]
front_right_r = pca.channel[9]
mid_right_r = pca.channel[10]
back_right_r = pca.channel[11]

while (True):
    command = getch.getch()
    
    if (command == 'x'):
        front_left_f.duty_cycle = 0
        mid_left_f.duty_cycle = 0
        back_left_f.duty_cycle = 0
        front_right_f.duty_cycle = 0
        mid_right_f.duty_cycle = 0
        back_right_f.duty_cycle = 0

        front_left_r.duty_cycle = 0
        mid_left_r.duty_cycle = 0
        back_left_r.duty_cycle = 0
        front_right_r.duty_cycle = 0
        mid_right_r.duty_cycle = 0
        back_right_r.duty_cycle = 0

    elif (command == 'w'):
        front_left_f.duty_cycle = 0x00ff
        mid_left_f.duty_cycle = 0x00ff
        back_left_f.duty_cycle = 0x00ff
        front_right_f.duty_cycle = 0x00ff
        mid_right_f.duty_cycle = 0x00ff
        back_right_f.duty_cycle = 0x00ff

        front_left_r.duty_cycle = 0
        mid_left_r.duty_cycle = 0
        back_left_r.duty_cycle = 0
        front_right_r.duty_cycle = 0
        mid_right_r.duty_cycle = 0
        back_right_r.duty_cycle = 0

    elif (command == 's'):
        front_left_f.duty_cycle = 0
        mid_left_f.duty_cycle = 0
        back_left_f.duty_cycle = 0
        front_right_f.duty_cycle = 0
        mid_right_f.duty_cycle = 0
        back_right_f.duty_cycle = 0

        front_left_r.duty_cycle = 0x00ff
        mid_left_r.duty_cycle = 0x00ff
        back_left_r.duty_cycle = 0x00ff
        front_right_r.duty_cycle = 0x00ff
        mid_right_r.duty_cycle = 0x00ff
        back_right_r.duty_cycle = 0x00ff

    elif (command == 'a'):
        front_left_f.duty_cycle = 0
        mid_left_f.duty_cycle = 0
        back_left_f.duty_cycle = 0
        front_right_f.duty_cycle = 0x00ff
        mid_right_f.duty_cycle = 0x00ff
        back_right_f.duty_cycle = 0x00ff

        front_left_r.duty_cycle = 0x00ff
        mid_left_r.duty_cycle = 0x00ff
        back_left_r.duty_cycle = 0x00ff
        front_right_r.duty_cycle = 0
        mid_right_r.duty_cycle = 0
        back_right_r.duty_cycle = 0

    elif (command == 'd'):
        front_left_f.duty_cycle = 0x00ff
        mid_left_f.duty_cycle = 0x00ff
        back_left_f.duty_cycle = 0x00ff
        front_right_f.duty_cycle = 0
        mid_right_f.duty_cycle = 0
        back_right_f.duty_cycle = 0

        front_left_r.duty_cycle = 0
        mid_left_r.duty_cycle = 0
        back_left_r.duty_cycle = 0
        front_right_r.duty_cycle = 0x00ff
        mid_right_r.duty_cycle = 0x00ff
        back_right_r.duty_cycle = 0x00ff