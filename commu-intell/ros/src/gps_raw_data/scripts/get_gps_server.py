import rospy
from gps_raw_data.srv import gps_srv, gps_srvResponse

def handle_reuest():
    return gps_srvResponse(100, 100)

def get_gps_server():
    rospy.init_node('get_gps_server')
    s = rospy.Service('get_gps_server', gps_srv, handle_requests)
    rospy.spin()

if __name__ == "__main__":
    get_gps_server()