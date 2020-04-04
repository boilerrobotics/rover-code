import math as math
import rospy
from std_msgs.msg._String import String
#==========================================================================================#
# This program calculate and publish the linear velocity of the encoder                    #
#                                                                                          #
# Written by: Pum Khai                                                                     #
# Start Date: April 3, 2020                                                                #
# Last Edit: April 4, 2020                                                                 #   
#                                                                                          #
# Status:Incomplete (Need working on get_num_of_pulese(period) function)                   #
# and getEncoder once get the hardware                                                     #
#                                                                                          #    
# Maintance:                                                                               #                            
#==========================================================================================#

# keeping counting pulses
# def getEncoder once get the hardware

def get_num_Of_pulses(period):
    # return number of pulses in that period
    # KEEP WORKING ON IT
   return 10


# DO NOT CHANGE ANYTHING BELOW
## Some how find a way to Calculate linear_velocity
def calculate_angular_velocity(pulses_per_second):
    pulses_per_rotations = 30 # go to encoder data sheet
    return (pulses_per_second / pulses_per_rotations * 2 * math.pi)

def calculate_linear_velocity(pulses_per_second):
   diameter = 0.2
   return calculate_angular_velocity(pulses_per_second) * diameter / 2

## Publish that number in float format.
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