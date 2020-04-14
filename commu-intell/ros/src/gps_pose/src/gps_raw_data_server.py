import rospy
from gps_pose.srv import GpsPose, GpsPoseResponse
from GpsPose import GpsPose

def handle_requests():
    gps = GpsPose()
    gps.update()
    return GpsPoseResponse(gps)

def gps_raw_data_server():
    rospy.init_node('gps_raw_data_server')
    s = rospy.Service('gps_raw_data', GpsPose, handle_requests)
    rospy.spin()

if __name__ == "__main__":
    gps_raw_data_server()
