import rospy
from gps_raw_data.srv import gps_srv, gps_srvResponse
from gps_pose import gps

def handle_requests():
    gps1 = gps()
    #gps1.set_latitude_longitude()
    return gps_srvResponse(gps1)

def gps_pose_server_node():
    rospy.init_node('gps_pose_server_node')
    s = rospy.Service('gps_pose', gps_srv, handle_requests)
    rospy.spin()

if __name__ == "__main__":
    gps_pose_server_node()