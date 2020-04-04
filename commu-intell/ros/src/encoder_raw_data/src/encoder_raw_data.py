
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

def read_num_Of_pulses():
   return 10

def read_time():
   return 60

def read_pulse_per_rotation():
   return (read_num_Of_pulses / read_time)

## Some how find a way to Calculate rotation per seconds
def calculateSpeed():
   #Distance traveled = (Degrees turned / 360) * circumference
   Diameter = 0.2
   Circumference = math.Pi * Diameter
   Degree = 5000 # some random number to initialize
   degree_per_min = Degree
   time_in_min = 60
   Distance_traveled = (Degree / 360) * Circumference 
   Speed = Distance_traveled * time_in_min
   # Angular Speed ω = 2πn/Nt
   # ω = angular speed (rad/s)
   # n = number of pulses
   # t = sampling period (s)
   # N = pulses per rotation
   # fasfsg
   #fasfaf
   n = read_num_Of_pulses()
   t = read_time()
   N = read_pulse_per_rotation
   ω = 2 * math.pi * n / N * t
   return Speed 

## Publish that number in float format.
def Publishers():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        speed = "The Speed: %f" % pub.calculateSpeed
        pub.publish(speed)
        rate.sleep()
   
    if __name__ == '__main__':
        try:
           Publishers()
        except rospy.ROSInterruptException:
           pass