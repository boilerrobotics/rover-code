
from gps_raw_data.srv import gps_srv, gps_srvResponse
import rospy

def handle_two_floats(req):
    return gps_srvResponse(req.a, req.b)

def get_gps_server():
    rospy.init_node('get_gps_server')
    s = rospy.Service('get_gps_server', gps_srv, handle_two_floats)
    rospy.spin()

if __name__ == "__main__":
    get_gps_server()