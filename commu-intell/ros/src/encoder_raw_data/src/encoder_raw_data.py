
#That number should be the 
#rotations per second of the wheel. 
#You don't need the wheel diameter anymore. 
#So you will start of with some raw data, 
#you will then get the interval from that with some 
#processing and use that 
#to calculate rotations per second.
# You will then publish that number.

## Find a way to import some signal from the encoder
import math as math
import rospy
from std_msgs.msg._String import String

# keeping counting pulses
# def getEncoder once get the hardware

def get_num_Of_pulses(period):
    # return number of pulses in that period
   return 10

## Some how find a way to Calculate rotation per seconds
def calculate_angular_velocity(pulses_per_second):
    pulses_per_rotations = 30 # go to encoder data sheet
    return (pulses_per_second / pulses_per_rotations * 2 * math.pi)

def calculate_linear_velocity(pulses_per_second):
   #Distance traveled = (Degrees turned / 360) * circumference
   diameter = 0.2
   #Circumference = math.Pi * diameter
   #Degree = 5000 # some random number to initialize
   #degree_per_min = Degree
   #time_in_min = 60
   #Distance_traveled = (Degree / 360) * Circumference 
   #Speed = Distance_traveled * time_in_min
   # Angular Speed ω = 2πn/Nt
   # ω = angular speed (rad/s)
   # n = number of pulses
   # t = sampling period (s)
   # N = pulses per rotation
   # fasfsg
   # fasfaf
   # Linear 
   # We want linear 
   #n = read_num_Of_pulses()
   #t = read_time()
   #N = read_pulse_per_rotation
   #ω = 2 * math.pi * n / N * t

    ## vel = angular * radius 
    # Use 360 
    # 

   return calculate_angular_velocity(pulses_per_second) * diameter / 2

## Publish that number in float format.
# Change the node name and the topic and message type
def Publishers():
    pub = rospy.Publisher('sensor_msgs_encoder', float, queue_size = 10)
    rospy.init_node('encoder_raw_data_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        period = 1 # seconds
        velocity = calculate_linear_velocity(get_num_Of_pulses(period)) # meter per seconds
        pub.publish(velocity)
        rate.sleep()
   
    if __name__ == '__main__':
        try:
           Publishers()
        except rospy.ROSInterruptException:
           pass