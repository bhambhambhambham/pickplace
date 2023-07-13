#! /usr/bin/env python3

import rospy
from bhampick.msg import dimmsg

def pubs():
    r = rospy.Rate(2)
    dim = dimmsg()
    dim.dimensionx = 0.024
    dim.dimensiony = 0.1363
    dim.dimensionz = 0.0727
    while not rospy.is_shutdown():
        pub.publish(dim)
        r.sleep()
    return
    
if __name__ == "__main__":
    rospy.init_node("pubdim")
    global pub
    pub = rospy.Publisher('/dim', dimmsg, queue_size = 10)
    rospy.sleep(1)
    pubs()