#!/usr/bin/env python3
# roslaunch hw hw2launch.launch

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped , Pose, PoseArray

def movetogoal():
    pub = rospy.Publisher("posear", PoseArray(), queue_size=10)
    a = Pose()
    euler = tf.transformations.quaternion_from_euler(1,0,0)
    a.orientation = euler
    ar = []
    ar.append(a)
    pub.publish(ar)
    rospy.spin()
    


if __name__ == "__main__":
    movetogoal()