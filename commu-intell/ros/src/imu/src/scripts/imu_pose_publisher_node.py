'''
===============================================================================
imu_pose_publisher_node.py 
    This program publish raw data from MPU6050 to a ros node in forms of a
    mpu6050 object

Author:         Annabel Li, li3317@purdue.edu
Maintainer:     Annabel Li, li3317@purdue.edu
Version:        April 17, 2020
Status:         In progress
===============================================================================
'''


import rospy
from imu import imu_raw_data
from IMU.msg import IMU


def imu_publisher():
	pub = rospy.Publisher('imu_raw_data', IMU, queue_size=10)
	rospy.init_node('imu_raw_data_publisher')
	rate = rospy.Rate(1) # in hz
	

	imu = imu_raw_data()
    data = imu.get_all_data()
    imu_data = IMU()
    imu_data.surging = data['surging']
    imu_data.heaving = data['heaving']
    imu_data.swaying = data['swaying']
    imu_data.pitch = data['pitch']
    imu_data.yaw = data['yaw']
    imu_data.roll = data['roll']



	while not rospy.is_shutdown():
		rospy.loginfo(imu_data)
        pub.publish(imu_data)
		rate.sleep()

if __name__ == '__main__':
	try:
		imu_publisher()
	except rospy.ROSInterruptException:
		pass
