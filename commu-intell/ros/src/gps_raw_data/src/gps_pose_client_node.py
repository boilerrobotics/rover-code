import sys
import rospy
from gps_raw_data.srv import *
# from gps_pose import gps

def gps_pose_client_node(x, y):
    rospy.wait_for_service('Gps_pose')
    try:
        Gps_pose = rospy.ServiceProxy('Gps_pose', gpsSrv)
        #gps1 = gps()
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))