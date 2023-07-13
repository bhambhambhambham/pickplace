#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from pickplace_include.srv import pickplace, pickplaceRequest, pickplaceResponse
from bhampick.srv import objectlist, objectlistRequest, objectlistResponse
from std_msgs.msg import Float32MultiArray

if __name__ == "__main__":
    rospy.init_node("sender")
    srv = objectlistRequest()
    client = rospy.ServiceProxy('transformobj', objectlist)
    # objs = [[0.65, 0.16, 0.15, 0.06, 0.04, 0.06], [0.65, 0.60, 0.24, 0.02, 0.08, 0.02], [0.65, 0.54, 0.17, 0.06, 0.04, 0.04]]
    # objs = [[0.65, 0.12, 0.17, 0.06, 0.08, 0.04], [0.65, 0.14, -0.15, 0.02, 0.1, 0.04], [0.64, 0.64, -0.04, 0.04, 0.08, 0.04]]
    
    # objs = [[0.65, 0.16, 0.55, 0.06, 0.04, 0.06], [0.65, 0.60, 0.64, 0.02, 0.08, 0.02], [0.65, 0.54, 0.57, 0.06, 0.04, 0.04]]
    objs = [[0.65, 0.12, -0.23, 0.06, 0.08, 0.04], [0.65, 0.14, -0.55, 0.02, 0.1, 0.04], [0.64, 0.64, 0.-0.44, 0.04, 0.08, 0.04]]
    srv.objects = [item for sublist in objs for item in sublist]
    srv.place = True
    srv.floor = 4
    try:
        rospy.wait_for_service('transformobj')
        response = client(srv)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call service: %s", str(e))
        exit(1)
    rospy.signal_shutdown("Service completed")